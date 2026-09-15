"""Tests for the isaac-core shell completion subcommand and config-key walker."""

from __future__ import annotations

from pathlib import Path
import subprocess

import pytest

from isaac_core.cli.completion import config_keys
from isaac_core.cli.main import main


def test_config_keys_include_known_real_keys() -> None:
    keys = set(config_keys())
    assert "sim.headless" in keys
    assert "sim.control_plane.port" in keys
    assert "geo.enu_reference.lat_deg" in keys
    assert "cesium.tileset_server_url" in keys


def test_config_keys_expand_default_dict_entries() -> None:
    # Dict-typed fields emit the concrete keys of the DEFAULT config (drone_0 / eo).
    keys = set(config_keys())
    assert "vehicles.drone_0.pose_source" in keys
    assert "vehicles.drone_0.cameras.eo.fov_deg" in keys


def test_config_keys_exclude_private_and_method_names() -> None:
    keys = config_keys()
    # No dunders, no derived-property or method names, no leading/trailing dots.
    assert all(not key.startswith("_") for key in keys)
    assert all("._" not in key for key in keys)
    assert not any(key.endswith(".") for key in keys)
    for junk in ("camera_keys", "topic_resolver", "is_single_vehicle", "resolved_udp_port", "model_dump"):
        assert junk not in keys
        assert all(junk not in key.split(".") for key in keys)


def test_config_keys_are_sorted_and_unique() -> None:
    keys = config_keys()
    assert keys == sorted(keys)
    assert len(keys) == len(set(keys))


@pytest.mark.parametrize(
    "key",
    [
        "sim.headless",
        "sim.control_plane.port",
        "geo.enu_reference.lat_deg",
        "cesium.tileset_server_url",
        "vehicles.drone_0.cameras.eo.fov_deg",
    ],
)
def test_every_sampled_key_actually_resolves(key: str) -> None:
    # The value of the feature: a completed key must resolve in a loaded config.
    # `config explain <key>` returns 0 for a real key and 1 for a non-existent one.
    assert main(["config", "explain", key]) == 0


def test_all_emitted_keys_resolve_in_default_config() -> None:
    # Stronger guarantee: no emitted key is junk that would not resolve.
    from isaac_core.cli.config_cmd import _MISSING, _resolve_dotted_key
    from isaac_core.config import load

    data = load().model_dump()
    for key in config_keys():
        assert _resolve_dotted_key(data, key) is not _MISSING, f"emitted key does not resolve: {key}"


@pytest.mark.parametrize("shell", ["bash", "zsh"])
def test_completion_prints_nonempty_script_and_exits_zero(shell: str, capsys: pytest.CaptureFixture[str]) -> None:
    code = main(["completion", shell])
    assert code == 0
    out = capsys.readouterr().out
    assert len(out.strip()) > 0
    assert "isaac-core" in out
    assert "complete -F" in out


def test_completion_unknown_shell_exits_nonzero(capsys: pytest.CaptureFixture[str]) -> None:
    # argparse rejects an invalid choice with a SystemExit (non-zero code).
    with pytest.raises(SystemExit) as exc_info:
        main(["completion", "fish"])
    assert exc_info.value.code != 0


def test_completion_no_shell_exits_nonzero_with_message(capsys: pytest.CaptureFixture[str]) -> None:
    code = main(["completion"])
    assert code == 1
    err = capsys.readouterr().err
    assert "shell" in err.lower()


def test_completion_list_keys_prints_one_per_line(capsys: pytest.CaptureFixture[str]) -> None:
    code = main(["completion", "--list-keys"])
    assert code == 0
    lines = [line for line in capsys.readouterr().out.splitlines() if line]
    assert "sim.headless" in lines
    assert set(lines) == set(config_keys())


def test_emitted_bash_script_passes_bash_syntax_check(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    code = main(["completion", "bash"])
    assert code == 0
    script = capsys.readouterr().out

    script_path = tmp_path / "completion.bash"
    script_path.write_text(script, encoding="utf-8")

    result = subprocess.run(
        ["bash", "-n", str(script_path)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
