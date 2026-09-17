"""Configuration source layers.

Each function here produces a plain nested dict from a single input layer,
making every source independently testable without needing the full load
machinery.

Resolution order (later wins):
    defaults -> TOML file -> environment variables -> CLI overrides
"""

from __future__ import annotations

from collections.abc import Mapping
import json
from pathlib import Path
import sys
from typing import Any

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


def defaults_source() -> dict[str, Any]:
    """Return the defaults layer: an empty dict.

    The pydantic field defaults *are* the shipped defaults. ``config/default.toml``
    is a documented copy for users to start from and is contract-tested to agree,
    but is not loaded during resolution. An empty dict lets pydantic fill in its
    own defaults.

    Returns:
        An empty dict.

    """
    return {}


def toml_file_source(path: Path) -> dict[str, Any]:
    """Parse a TOML configuration file into a nested dict.

    Uses :mod:`tomllib` on Python 3.11+ and :mod:`tomli` on 3.10.

    Args:
        path: Filesystem path to the TOML file.

    Returns:
        Parsed TOML as a nested dict.

    Raises:
        FileNotFoundError: If the file does not exist, with the path in the message.
        ValueError: If the file is malformed TOML, with both the path and the
            parse error in the message.

    """
    if not path.is_file():
        msg = f"configuration file not found: {path}"
        raise FileNotFoundError(msg)
    try:
        with path.open("rb") as handle:
            data: dict[str, Any] = tomllib.load(handle)
    except tomllib.TOMLDecodeError as exc:
        msg = f"malformed TOML in {path}: {exc}"
        raise ValueError(msg) from exc
    return data


def _coerce_env_value(raw: str) -> object:
    """Coerce a string environment variable value to the most specific Python type.

    Rules applied in order:
    - "true"/"false" (case-insensitive) -> bool
    - valid integer literal -> int
    - valid float literal -> float
    - JSON array or object -> parsed structure
    - anything else -> str (unchanged)

    Args:
        raw: The raw string value from the environment.

    Returns:
        The coerced value.

    """
    lowered = raw.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False

    try:
        return int(raw)
    except ValueError:
        pass

    try:
        return float(raw)
    except ValueError:
        pass

    if raw.startswith(("[", "{")):
        try:
            return json.loads(raw)
        except (json.JSONDecodeError, ValueError):
            pass

    return raw


def env_source(environ: Mapping[str, str], prefix: str = "ISAAC_CORE") -> dict[str, Any]:
    """Parse environment variables into a nested configuration dict.

    Variables are expected in the form ``PREFIX__SECTION__KEY=value``, where
    ``__`` (double underscore) separates nesting levels and each segment is
    lowercased before being used as a dict key.

    Values are coerced with :func:`_coerce_env_value`: true/false to bool, ints,
    floats, JSON arrays/objects, otherwise left as strings.

    Args:
        environ: A mapping of environment variable names to values.  Accept an
            explicit argument so tests never touch the real environment.
        prefix: Only variables starting with ``prefix + "__"`` are considered.

    Returns:
        Nested dict with coerced values.

    """
    separator = "__"
    full_prefix = prefix + separator
    result: dict[str, Any] = {}

    for key, raw_value in environ.items():
        if not key.startswith(full_prefix):
            continue
        remainder = key[len(full_prefix) :]
        segments = [seg.lower() for seg in remainder.split(separator)]
        if not segments or any(seg == "" for seg in segments):
            continue

        target = result
        for segment in segments[:-1]:
            target = target.setdefault(segment, {})
        target[segments[-1]] = _coerce_env_value(raw_value)

    return result


def cli_source(overrides: Mapping[str, Any]) -> dict[str, Any]:
    """Expand dotted-key CLI overrides into a nested dict.

    Accepts keys like ``"vehicles.drone_0.camera.fov_deg"`` and expands them
    to nested dicts with the final segment as the leaf key.

    String values (as always come from ``--set KEY VALUE``) are coerced with the same
    rules as environment variables -- bool/int/float/JSON -- so ``--set`` and
    ``ISAAC_CORE__*`` behave identically, including for list/dict fields like
    ``--set sim.extensions '["a","b"]'``. Non-string values (passed programmatically, e.g.
    from the devkit or tests) are left untouched.

    Args:
        overrides: Flat mapping of dotted keys to their values.

    Returns:
        Nested dict.

    """
    result: dict[str, Any] = {}
    for dotted_key, value in overrides.items():
        segments = dotted_key.split(".")
        target = result
        for segment in segments[:-1]:
            target = target.setdefault(segment, {})
        target[segments[-1]] = _coerce_env_value(value) if isinstance(value, str) else value
    return result
