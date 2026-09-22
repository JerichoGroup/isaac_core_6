"""Configuration subcommands: ``config dump`` and ``config explain``.

``dump`` renders the fully-resolved configuration as parseable TOML.
``explain`` shows a single key's winning value and which source layer set it.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any


def register_config_subcommand(subparsers: "argparse._SubParsersAction[argparse.ArgumentParser]") -> None:
    """Register the ``config`` subcommand with its own sub-subcommands.

    Args:
        subparsers: The parent subparsers action to attach to.

    """
    config_parser = subparsers.add_parser("config", help="Configuration inspection")
    config_sub = config_parser.add_subparsers(dest="config_command")

    # dump
    dump_parser = config_sub.add_parser("dump", help="Print the resolved config as TOML")
    dump_parser.add_argument("--config", type=str, default=None, help="Path to configuration TOML file")
    _add_set_argument(dump_parser)

    # explain
    explain_parser = config_sub.add_parser("explain", help="Explain a specific config key")
    explain_parser.add_argument("key", type=str, help="Dotted config key (e.g. sim.headless)")
    explain_parser.add_argument("--config", type=str, default=None, help="Path to configuration TOML file")
    _add_set_argument(explain_parser)


def _add_set_argument(parser: argparse.ArgumentParser) -> None:
    """Give a config subcommand the same ``--set`` flag ``run`` has.

    Without this neither command could see one of the five documented resolution sources, so
    ``config dump`` could not preview what a ``--set`` would do and ``config explain`` -- whose entire
    job is saying which source won -- could not report the source most likely to be in play while
    someone debugs an override.

    Args:
        parser: The subcommand parser to extend.

    """
    parser.add_argument(
        "--set",
        nargs=2,
        metavar=("KEY", "VALUE"),
        action="append",
        default=[],
        help="Override a config key, exactly as `run` takes it (e.g. --set sim.headless true)",
    )


def overrides_from_args(pairs: list[list[str]] | None) -> dict[str, str] | None:
    """Turn repeated ``--set KEY VALUE`` pairs into the mapping ``load`` expects.

    Args:
        pairs: What argparse collected, or ``None``.

    Returns:
        The overrides, or ``None`` when there were none, since ``load`` distinguishes the two.

    """
    if not pairs:
        return None
    return {key: value for key, value in pairs}


def run_config_dump(config_path: str | None = None, cli_overrides: dict[str, str] | None = None) -> int:
    """Print the fully-resolved configuration as TOML to stdout.

    Args:
        config_path: Optional path to a TOML config file.
        cli_overrides: Optional ``--set`` overrides, applied exactly as ``run`` applies them so the dump
            shows what would actually launch.

    Returns:
        Exit code (0 = success).

    """
    from isaac_core.config import dump_toml, load

    path = Path(config_path) if config_path else None
    try:
        config = load(path=path, cli_overrides=cli_overrides)
    except (FileNotFoundError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    output = dump_toml(config)
    print(output)
    return 0


def run_config_explain(key: str, config_path: str | None = None, cli_overrides: dict[str, str] | None = None) -> int:
    """Print the winning value and source for a single dotted config key.

    Uses the provenance dict returned by :func:`~isaac_core.config.load_with_provenance`.

    Args:
        key: Dotted key (e.g. ``sim.headless``).
        config_path: Optional path to a TOML config file.
        cli_overrides: Optional ``--set`` overrides, so the reported source can be the flag itself.

    Returns:
        Exit code (0 = found, 1 = error or not found).

    """
    from isaac_core.config import load_with_provenance

    path = Path(config_path) if config_path else None
    try:
        config, provenance = load_with_provenance(path=path, cli_overrides=cli_overrides)
    except (FileNotFoundError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    value = _resolve_dotted_key(config.model_dump(), key)
    if value is _MISSING:
        print(f"ERROR: key '{key}' does not exist in the resolved configuration", file=sys.stderr)
        return 1

    source = provenance.get(key, "default")
    print(f"{key} = {_format_value(value)}")
    print(f"  source: {source}")
    return 0


# Sentinel for a missing key lookup.
_MISSING = object()


def _resolve_dotted_key(data: dict[str, Any], key: str) -> Any:
    """Walk a nested dict using a dotted key path.

    Args:
        data: The nested dict to traverse.
        key: Dotted key like ``"sim.headless"``.

    Returns:
        The value at the key, or :data:`_MISSING` if the path does not exist.

    """
    segments = key.split(".")
    current: Any = data
    for segment in segments:
        if not isinstance(current, dict):
            return _MISSING
        if segment not in current:
            return _MISSING
        current = current[segment]
    return current


def _format_value(value: object) -> str:
    """Format a configuration value for human display.

    Args:
        value: The value to format.

    Returns:
        A string representation.

    """
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return f'"{value}"'
    if isinstance(value, (list, tuple)):
        items = ", ".join(_format_value(item) for item in value)
        return f"[{items}]"
    if isinstance(value, dict):
        items = ", ".join(f"{k} = {_format_value(v)}" for k, v in value.items())
        return f"{{{items}}}"
    if value is None:
        return "null"
    return str(value)
