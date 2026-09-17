"""Tests for installing shell completion, not merely generating it.

The generator worked from the first release while nothing installed the script, so pressing TAB
completed filenames and the feature was invisible. These tests cover the installing half: where the
script lands, that the shell is made to read it, that running twice is harmless, and that the
generated script really does complete the commands and config keys it claims to.
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
import shutil
import subprocess
from typing import Final
from unittest import mock

import pytest

from isaac_core.cli import completion
from isaac_core.cli.completion import (
    completion_script,
    detect_shell,
    install_completion,
    installed_script_path,
    is_installed,
)

_SHELLS: Final = ("bash", "zsh")


@pytest.fixture()
def isolated_home(tmp_path: Path) -> Iterator[Path]:
    """Point the installer at a throwaway home directory."""
    rc_files = {"bash": tmp_path / ".bashrc", "zsh": tmp_path / ".zshrc"}
    for rc in rc_files.values():
        rc.write_text("# pre-existing content\nexport SENTINEL=keep-me\n", encoding="utf-8")
    with (
        mock.patch.object(completion, "_INSTALL_DIR", tmp_path / ".local/share/isaac-core"),
        mock.patch.object(completion, "_DYNAMIC_DIR", tmp_path / ".local/share/bash-completion/completions"),
        mock.patch.object(completion, "_RC_FILES", rc_files),
    ):
        yield tmp_path


@pytest.mark.parametrize("shell", _SHELLS)
def test_install_writes_the_script_and_sources_it(shell: str, isolated_home: Path) -> None:
    assert is_installed(shell) is False

    changed = install_completion(shell)

    script = installed_script_path(shell)
    assert script.is_file(), f"no script at {script}"
    assert script.read_text(encoding="utf-8") == completion_script(shell)
    assert is_installed(shell) is True
    assert any("wrote" in line for line in changed)
    assert any("source line" in line for line in changed)


@pytest.mark.parametrize("shell", _SHELLS)
def test_install_leaves_existing_rc_content_alone(shell: str, isolated_home: Path) -> None:
    # Appending to somebody's shell configuration is only acceptable if it cannot lose anything.
    install_completion(shell)
    rc_text = completion._RC_FILES[shell].read_text(encoding="utf-8")
    assert "export SENTINEL=keep-me" in rc_text


@pytest.mark.parametrize("shell", _SHELLS)
def test_installing_twice_adds_one_block(shell: str, isolated_home: Path) -> None:
    install_completion(shell)
    second = install_completion(shell)
    rc_text = completion._RC_FILES[shell].read_text(encoding="utf-8")
    assert rc_text.count(completion._RC_BEGIN) == 1, "the rc block was added twice"
    assert any("already sources it" in line for line in second)


@pytest.mark.parametrize("shell", _SHELLS)
def test_reinstalling_refreshes_the_script(shell: str, isolated_home: Path) -> None:
    # An upgrade must replace a stale script, otherwise completion silently keeps old behaviour.
    install_completion(shell)
    script = installed_script_path(shell)
    script.write_text("# stale\n", encoding="utf-8")
    install_completion(shell)
    assert script.read_text(encoding="utf-8") == completion_script(shell)


def test_is_installed_is_false_when_only_the_script_exists(isolated_home: Path) -> None:
    # A script nobody sources is not installed. This is exactly the state the project shipped in.
    script = installed_script_path("bash")
    script.parent.mkdir(parents=True, exist_ok=True)
    script.write_text(completion_script("bash"), encoding="utf-8")
    assert is_installed("bash") is False


def test_is_installed_is_false_when_only_the_rc_line_exists(isolated_home: Path) -> None:
    rc = completion._RC_FILES["bash"]
    rc.write_text(f"{completion._RC_BEGIN}\n# nothing here\n{completion._RC_END}\n", encoding="utf-8")
    assert is_installed("bash") is False


@pytest.mark.parametrize(
    ("shell_path", "expected"),
    [
        ("/bin/bash", "bash"),
        ("/usr/bin/zsh", "zsh"),
        ("/bin/fish", None),
        ("", None),
    ],
)
def test_detect_shell_reads_the_shell_variable(
    shell_path: str, expected: str | None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("SHELL", shell_path)
    assert detect_shell() == expected


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is needed to run the generated script")
@pytest.mark.parametrize(
    ("typed", "expected_member"),
    [
        (["isaac-core", "r"], "run"),
        (["isaac-core", "d"], "doctor"),
        (["isaac-core", "co"], "config"),
        (["isaac-core", "config", ""], "dump"),
        (["isaac-core", "run", "--"], "--isaac-path"),
        (["isaac-core", "run", "--set", "sim.h"], "sim.headless"),
    ],
)
def test_the_generated_bash_script_completes_what_it_claims(
    typed: list[str], expected_member: str, tmp_path: Path
) -> None:
    # The complaint that started this was `isaac-core r<TAB>` offering filenames. Driving the real
    # completion function in a real bash is the only way to know the script works, rather than
    # knowing that a string was generated.
    script = tmp_path / "completion.bash"
    script.write_text(completion_script("bash"), encoding="utf-8")
    words = " ".join(f'"{word}"' for word in typed)
    program = f"""
        source "{script}"
        COMP_WORDS=({words})
        COMP_CWORD={len(typed) - 1}
        COMPREPLY=()
        _isaac_core_complete
        printf '%s\\n' "${{COMPREPLY[@]}}"
    """
    result = subprocess.run(["bash", "-c", program], capture_output=True, text=True, timeout=120, check=False)
    offered = result.stdout.split()
    assert expected_member in offered, f"typing {typed} offered {offered!r}, expected {expected_member!r}"


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is needed to run the generated script")
def test_the_generated_script_is_valid_shell() -> None:
    for shell in _SHELLS:
        result = subprocess.run(
            ["bash", "-n"],
            input=completion_script(shell),
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )
        # The zsh script uses zsh builtins bash does not have, so only bash's own script must parse.
        if shell == "bash":
            assert result.returncode == 0, f"bash script does not parse: {result.stderr}"


def test_bash_is_also_installed_where_a_running_shell_will_find_it(isolated_home: Path) -> None:
    # The rc line only takes effect in a NEW shell, so installing completion appeared to do nothing:
    # TAB still completed filenames in the shell that had just run setup. bash-completion loads a file
    # named after the command from its completions directory at the moment TAB is pressed, which an
    # already-running shell picks up.
    from isaac_core.cli.completion import dynamic_script_path

    install_completion("bash")
    dynamic = dynamic_script_path("bash")
    assert dynamic is not None
    assert dynamic.is_file(), f"nothing at {dynamic}"
    assert dynamic.name == "isaac-core", "bash-completion loads the file named after the command"
    assert dynamic.read_text(encoding="utf-8") == completion_script("bash")


def test_zsh_has_no_dynamic_location_and_says_so() -> None:
    # zsh needs the directory on fpath, which we do not edit, so the rc line is the only route there.
    from isaac_core.cli.completion import dynamic_script_path

    assert dynamic_script_path("zsh") is None


def test_reinstalling_refreshes_the_dynamic_copy_too(isolated_home: Path) -> None:
    from isaac_core.cli.completion import dynamic_script_path

    install_completion("bash")
    dynamic = dynamic_script_path("bash")
    assert dynamic is not None
    dynamic.write_text("# stale\n", encoding="utf-8")
    install_completion("bash")
    assert dynamic.read_text(encoding="utf-8") == completion_script("bash")
