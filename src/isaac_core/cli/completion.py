"""Shell tab-completion for the ``isaac-core`` command-line tool.

No third-party completion library is used or required. Instead this follows the
kubectl/docker pattern: ``isaac-core completion bash`` (or ``zsh``) prints a shell
script the user evaluates, and that script calls back into the CLI at runtime --
``isaac-core completion --list-keys`` -- to obtain the set of settable config keys.

Deriving the key list from the pydantic schema at runtime keeps completion always in
sync with the schema, rather than baking a list into the shell script that silently
rots. The key walker only ever emits keys that actually resolve in a loaded config, so
completing them is never misleading.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
from typing import TypeGuard, get_args, get_origin

from pydantic import BaseModel
from pydantic.fields import FieldInfo

from isaac_core.config import IsaacCoreConfig

# Subcommands the top-level parser exposes, for command-name completion.
_TOP_LEVEL_COMMANDS: tuple[str, ...] = ("run", "doctor", "config", "completion")

# ``config`` sub-subcommands, for nested command-name completion.
_CONFIG_SUBCOMMANDS: tuple[str, ...] = ("dump", "explain")

# Shells the ``completion`` subcommand can emit a script for.
_SUPPORTED_SHELLS: tuple[str, ...] = ("bash", "zsh")

# Where an installed script is written. One directory for both shells keeps removal obvious.
_INSTALL_DIR: Path = Path("~/.local/share/isaac-core").expanduser()

# bash-completion loads a file named after the command from here, on demand, at the moment TAB is
# pressed. Writing it there is what makes completion work in a shell that is ALREADY RUNNING -- the rc
# line alone only takes effect in a new shell, which is why installing it appeared to do nothing.
_DYNAMIC_DIR: Path = Path("~/.local/share/bash-completion/completions").expanduser()

# Marker lines wrapping the block appended to the user's shell rc file. Present so installing twice
# does nothing, and so a user can find and delete the block by searching for the name.
_RC_BEGIN: str = "# >>> isaac-core completion >>>"
_RC_END: str = "# <<< isaac-core completion <<<"

# The rc file each shell reads for interactive sessions.
_RC_FILES: dict[str, Path] = {
    "bash": Path("~/.bashrc").expanduser(),
    "zsh": Path("~/.zshrc").expanduser(),
}

# A ``dict[key, value]`` annotation always resolves to exactly two type arguments.
_DICT_TYPE_ARG_COUNT: int = 2


def _is_model(annotation: object) -> TypeGuard[type[BaseModel]]:
    """Return whether an annotation is a pydantic model class.

    Args:
        annotation: A field annotation.

    Returns:
        ``True`` if ``annotation`` is a subclass of :class:`pydantic.BaseModel`.

    """
    return isinstance(annotation, type) and issubclass(annotation, BaseModel)


def _walk_model(model: type[BaseModel], prefix: str) -> list[str]:
    """Walk a pydantic model, returning every settable dotted key beneath ``prefix``.

    Nested models are descended into recursively. Dict-typed fields are handled by
    :func:`_walk_default_dict`, which reads the concrete keys present in the default
    configuration so that only genuinely settable keys are emitted.

    Args:
        model: The pydantic model class to walk.
        prefix: The dotted prefix accumulated so far (empty at the root).

    Returns:
        A list of dotted keys, in field-declaration order.

    """
    keys: list[str] = []
    for name, field in model.model_fields.items():
        dotted = f"{prefix}{name}"
        annotation = field.annotation
        if _is_model(annotation):
            keys.extend(_walk_model(annotation, f"{dotted}."))
        elif _is_dict_field(annotation):
            keys.extend(_walk_default_dict(model, name, dotted, field))
        else:
            keys.append(dotted)
    return keys


def _is_dict_field(annotation: object) -> bool:
    """Return whether a field annotation is a ``dict[...]`` type.

    Args:
        annotation: A field annotation.

    Returns:
        ``True`` if the annotation's origin is :class:`dict`.

    """
    return get_origin(annotation) is dict


def _walk_default_dict(model: type[BaseModel], name: str, dotted: str, field: FieldInfo) -> list[str]:
    """Expand a dict-typed field using the concrete keys present in its default value.

    Dict fields such as ``vehicles`` and ``cameras`` are keyed by ids chosen by the
    user, so there is no fixed set of dotted keys. Rather than emit a placeholder that
    would not resolve, this reads the default instance (e.g. the ``drone_0`` vehicle and
    its ``eo`` camera) and emits the real dotted keys under it. If the value type is a
    pydantic model, its subtree is walked; otherwise the concrete leaf keys are emitted.

    Args:
        model: The parent model owning the field.
        name: The dict field's attribute name.
        dotted: The dotted path to the dict field.
        field: The field's pydantic metadata.

    Returns:
        A list of dotted keys for the default dict entries, possibly empty.

    """
    default_dict = _default_dict_value(model, name)
    if not default_dict:
        return []

    value_type = _dict_value_type(field.annotation)
    keys: list[str] = []
    for entry_key in default_dict:
        entry_prefix = f"{dotted}.{entry_key}"
        if _is_model(value_type):
            keys.extend(_walk_model(value_type, f"{entry_prefix}."))
        else:
            keys.append(entry_prefix)
    return keys


def _default_dict_value(model: type[BaseModel], name: str) -> dict[str, object]:
    """Return the default value of a dict field on a model, as a plain dict.

    Args:
        model: The model owning the field.
        name: The dict field's attribute name.

    Returns:
        The default dict, or an empty dict if the field has no dict default.

    """
    instance = model()
    value = getattr(instance, name, None)
    if isinstance(value, dict):
        return dict(value)
    return {}


def _dict_value_type(annotation: object) -> object:
    """Return the value type of a ``dict[key, value]`` annotation.

    Args:
        annotation: A ``dict[...]`` annotation.

    Returns:
        The value type argument, or ``None`` if it cannot be determined.

    """
    args = get_args(annotation)
    if len(args) == _DICT_TYPE_ARG_COUNT:
        return args[1]
    return None


def config_keys() -> list[str]:
    """Return every settable dotted config key, derived from the pydantic schema.

    Walks :class:`~isaac_core.config.IsaacCoreConfig` recursively. Dict-typed fields
    contribute the concrete keys present in the default configuration (for example the
    ``drone_0`` vehicle and its ``eo`` camera), so every returned key resolves against a
    loaded config rather than being a placeholder.

    Returns:
        A sorted list of unique dotted keys.

    """
    return sorted(set(_walk_model(IsaacCoreConfig, "")))


# bash completion template. Calls `--list-keys` at runtime so candidates stay in sync with the
# schema; the `command -v` guard and 2>/dev/null keep it silent when the CLI is not on PATH.
_BASH_SCRIPT: str = r"""# bash completion for isaac-core
# Enable with:  source <(isaac-core completion bash)
_isaac_core_complete() {
    local cur prev words cword
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    if ! command -v isaac-core >/dev/null 2>&1; then
        return 0
    fi

    # Complete config keys after --set or `config explain`.
    if [[ "${prev}" == "--set" || "${prev}" == "explain" ]]; then
        local keys
        keys="$(isaac-core completion --list-keys 2>/dev/null)"
        COMPREPLY=( $(compgen -W "${keys}" -- "${cur}") )
        return 0
    fi

    # Complete the `config` sub-subcommands.
    if [[ "${COMP_WORDS[1]}" == "config" && "${COMP_CWORD}" -eq 2 ]]; then
        COMPREPLY=( $(compgen -W "dump explain" -- "${cur}") )
        return 0
    fi

    # Complete top-level flags for the current word.
    if [[ "${cur}" == -* ]]; then
        COMPREPLY=( $(compgen -W "--config --isaac-path --dry-run --set --help" -- "${cur}") )
        return 0
    fi

    # Default: top-level subcommands.
    if [[ "${COMP_CWORD}" -eq 1 ]]; then
        COMPREPLY=( $(compgen -W "run doctor config completion" -- "${cur}") )
        return 0
    fi

    return 0
}
complete -F _isaac_core_complete isaac-core
"""

# zsh completion script template.
#
# Registers a bash-style completion function via ``bashcompinit`` so a single well-tested
# candidate-generation path serves both shells. The runtime callback and guards mirror the
# bash script exactly.
_ZSH_SCRIPT: str = r"""# zsh completion for isaac-core
# Enable with:  source <(isaac-core completion zsh)
autoload -Uz +X compinit && compinit
autoload -Uz +X bashcompinit && bashcompinit

_isaac_core_complete() {
    local cur prev
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    if ! command -v isaac-core >/dev/null 2>&1; then
        return 0
    fi

    if [[ "${prev}" == "--set" || "${prev}" == "explain" ]]; then
        local keys
        keys="$(isaac-core completion --list-keys 2>/dev/null)"
        COMPREPLY=( $(compgen -W "${keys}" -- "${cur}") )
        return 0
    fi

    if [[ "${COMP_WORDS[1]}" == "config" && "${COMP_CWORD}" -eq 2 ]]; then
        COMPREPLY=( $(compgen -W "dump explain" -- "${cur}") )
        return 0
    fi

    if [[ "${cur}" == -* ]]; then
        COMPREPLY=( $(compgen -W "--config --isaac-path --dry-run --set --help" -- "${cur}") )
        return 0
    fi

    if [[ "${COMP_CWORD}" -eq 1 ]]; then
        COMPREPLY=( $(compgen -W "run doctor config completion" -- "${cur}") )
        return 0
    fi

    return 0
}
complete -F _isaac_core_complete isaac-core
"""

# Map a shell name to its completion script.
_SCRIPTS: dict[str, str] = {"bash": _BASH_SCRIPT, "zsh": _ZSH_SCRIPT}


def completion_script(shell: str) -> str:
    """Return the completion script for a supported shell.

    Args:
        shell: One of ``"bash"`` or ``"zsh"``.

    Returns:
        The completion script as a string.

    Raises:
        KeyError: If the shell is not supported.

    """
    return _SCRIPTS[shell]


def detect_shell() -> str | None:
    """Return the user's shell name if it is one we can complete for.

    Reads ``$SHELL`` rather than inspecting the parent process, because the parent of an installer
    is often not the interactive shell that will read the rc file.

    Returns:
        ``"bash"``, ``"zsh"``, or ``None`` when the shell is unknown or unsupported.

    """
    shell_path = os.environ.get("SHELL", "")
    name = Path(shell_path).name
    return name if name in _SUPPORTED_SHELLS else None


def installed_script_path(shell: str) -> Path:
    """Return where the completion script for *shell* is written.

    Args:
        shell: One of the supported shells.

    Returns:
        The absolute path of the installed script.

    """
    return _INSTALL_DIR / f"completion.{shell}"


def dynamic_script_path(shell: str) -> Path | None:
    """Return the on-demand load location for *shell*, if it has one.

    Args:
        shell: One of the supported shells.

    Returns:
        The path bash-completion loads on demand, or ``None`` for shells without such a mechanism.

    """
    return _DYNAMIC_DIR / "isaac-core" if shell == "bash" else None


def is_installed(shell: str) -> bool:
    """Report whether completion is installed *and* wired into the shell's rc file.

    Both halves matter: a script nobody sources is not installed, which is the state the project
    shipped in for a while -- the generator worked and TAB still completed filenames.

    Args:
        shell: One of the supported shells.

    Returns:
        ``True`` only if the script exists and the rc file sources it.

    """
    script = installed_script_path(shell)
    rc_file = _RC_FILES[shell]
    if not script.is_file() or not rc_file.is_file():
        return False
    return _RC_BEGIN in rc_file.read_text(encoding="utf-8")


def install_completion(shell: str) -> list[str]:
    """Write the completion script and make the shell source it.

    Idempotent: running twice rewrites the script (so an upgrade takes effect) and leaves the rc
    file alone if the block is already there.

    Args:
        shell: One of the supported shells.

    Returns:
        Human-readable lines describing what changed, for the caller to print.

    """
    script = completion_script(shell)
    script_path = installed_script_path(shell)
    script_path.parent.mkdir(parents=True, exist_ok=True)
    script_path.write_text(script, encoding="utf-8")
    done = [f"wrote {script_path}"]

    # Written second and separately: this one is picked up by an already-running shell, so a user who
    # just ran setup does not have to start a new one before TAB does anything.
    dynamic_path = dynamic_script_path(shell)
    if dynamic_path is not None:
        dynamic_path.parent.mkdir(parents=True, exist_ok=True)
        dynamic_path.write_text(script, encoding="utf-8")
        done.append(f"wrote {dynamic_path} (picked up by shells already running)")

    rc_file = _RC_FILES[shell]
    existing = rc_file.read_text(encoding="utf-8") if rc_file.is_file() else ""
    if _RC_BEGIN in existing:
        done.append(f"{rc_file} already sources it")
        return done

    block = f'\n{_RC_BEGIN}\n[ -f "{script_path}" ] && . "{script_path}"\n{_RC_END}\n'
    rc_file.parent.mkdir(parents=True, exist_ok=True)
    with rc_file.open("a", encoding="utf-8") as handle:
        handle.write(block)
    done.append(f"added a source line to {rc_file}")
    return done


def register_completion_subcommand(subparsers: "argparse._SubParsersAction[argparse.ArgumentParser]") -> None:
    """Register the ``completion`` subcommand on the parent subparsers.

    Args:
        subparsers: The parent subparsers action to attach to.

    """
    parser = subparsers.add_parser("completion", help="Print a shell completion script")
    parser.add_argument(
        "shell",
        nargs="?",
        choices=list(_SUPPORTED_SHELLS),
        help="Shell to emit a completion script for",
    )
    parser.add_argument(
        "--list-keys",
        action="store_true",
        help="Print every settable config key, one per line (used by the completion script)",
    )
    parser.add_argument(
        "--install",
        action="store_true",
        help="Install completion for your shell and wire it into your shell's rc file",
    )
    parser.add_argument(
        "--print",
        dest="print_script",
        action="store_true",
        help="Print the script to stdout instead of installing it",
    )


def run_completion(args: argparse.Namespace) -> int:
    """Handle the ``completion`` subcommand.

    With ``--list-keys`` it prints every settable dotted config key, one per line, for the
    generated shell script to consume. Otherwise it prints the completion script for the
    requested shell.

    Args:
        args: Parsed arguments carrying ``shell`` and ``list_keys``.

    Returns:
        Exit code (0 = success, non-zero = usage error).

    """

    if args.list_keys:
        for key in config_keys():
            print(key)
        return 0

    if args.install:
        shell = args.shell or detect_shell()
        if shell is None:
            print(
                "ERROR: could not tell which shell you use. Pass one: "
                f"isaac-core completion --install {'|'.join(_SUPPORTED_SHELLS)}",
                file=sys.stderr,
            )
            return 1
        for line in install_completion(shell):
            print(f"  {line}")
        if dynamic_script_path(shell) is not None:
            print(f"Completion installed for {shell}. It works in this shell too -- press TAB now.")
        else:
            print(f"Completion installed for {shell}. Start a new shell, or run:")
            print(f"  source {installed_script_path(shell)}")
        return 0

    shell = args.shell
    if shell is None:
        print(f"ERROR: specify a shell, one of: {', '.join(_SUPPORTED_SHELLS)}", file=sys.stderr)
        return 1
    if shell not in _SCRIPTS:
        print(f"ERROR: unsupported shell {shell!r}; choose one of: {', '.join(_SUPPORTED_SHELLS)}", file=sys.stderr)
        return 1

    print(completion_script(shell))
    return 0
