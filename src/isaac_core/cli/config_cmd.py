"""
Configuration subcommands: ``config dump`` and ``config explain``.

``dump`` renders the fully-resolved configuration as parseable TOML.
``explain`` shows a single key's winning value and which source layer set it.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any


def register_config_subcommand(subparsers: "argparse._SubParsersAction[argparse.ArgumentParser]") -> None:
    """
    Register the ``config`` subcommand with its own sub-subcommands.

    Args:
        subparsers: The parent subparsers action to attach to.

    """
    config_parser = subparsers.add_parser("config", help="Configuration inspection")
    config_sub = config_parser.add_subparsers(dest="config_command")

    # dump
    dump_parser = config_sub.add_parser("dump", help="Print the resolved config as TOML")
    dump_parser.add_argument("--config", type=str, default=None, help="Path to configuration TOML file")

    # explain
    explain_parser = config_sub.add_parser("explain", help="Explain a specific config key")
    explain_parser.add_argument("key", type=str, help="Dotted config key (e.g. sim.headless)")
    explain_parser.add_argument("--config", type=str, default=None, help="Path to configuration TOML file")


def run_config_dump(config_path: str | None = None) -> int:
    """
    Print the fully-resolved configuration as TOML to stdout.

    Args:
        config_path: Optional path to a TOML config file.

    Returns:
        Exit code (0 = success).

    """
    from isaac_core.config import dump_toml, load  # noqa: PLC0415

    path = Path(config_path) if config_path else None
    try:
        config = load(path=path)
    except (FileNotFoundError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    output = dump_toml(config)
    print(output)
    return 0


def run_config_explain(key: str, config_path: str | None = None) -> int:
    """
    Print the winning value and source for a single dotted config key.

    Uses the provenance dict returned by :func:`~isaac_core.config.load_with_provenance`.

    Args:
        key: Dotted key (e.g. ``sim.headless``).
        config_path: Optional path to a TOML config file.

    Returns:
        Exit code (0 = found, 1 = error or not found).

    """
    from isaac_core.config import load_with_provenance  # noqa: PLC0415

    path = Path(config_path) if config_path else None
    try:
        config, provenance = load_with_provenance(path=path)
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


def _resolve_dotted_key(data: dict[str, Any], key: str) -> Any:  # noqa: ANN401
    """
    Walk a nested dict using a dotted key path.

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
    """
    Format a configuration value for human display.

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
