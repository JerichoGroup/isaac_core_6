"""Layered configuration loader.

Resolves layered inputs into one validated :class:`~isaac_core.config.schema.IsaacCoreConfig`.
Resolution order, later wins::

    pydantic field defaults -> TOML file -> env vars -> CLI overrides

Provenance tracking records which source last set each leaf key, supporting
the ``isaac-core config explain`` command.
"""

from __future__ import annotations

from collections.abc import Mapping
import os
from pathlib import Path
from typing import Any

from isaac_core.config.schema import IsaacCoreConfig
from isaac_core.config.sources import cli_source, defaults_source, env_source, toml_file_source

# The environment variable pointing to a default config file.
_CONFIG_ENV_VAR = "ISAAC_CORE_CONFIG"


def deep_merge(
    base: dict[str, Any],
    override: dict[str, Any],
) -> dict[str, Any]:
    """Recursively merge two dicts, with *override* winning on conflicts.

    Never mutates its inputs. Lists and tuples are replaced wholesale rather than
    concatenated -- appending would make it impossible to shorten a list through an
    override layer.

    Args:
        base: The lower-priority dict.
        override: The higher-priority dict whose values win.

    Returns:
        A new merged dict.

    """
    merged: dict[str, Any] = {}
    for key in base:
        if key in override:
            base_val = base[key]
            over_val = override[key]
            if isinstance(base_val, dict) and isinstance(over_val, dict):
                merged[key] = deep_merge(base_val, over_val)
            else:
                merged[key] = over_val
        else:
            merged[key] = _copy_value(base[key])
    for key in override:
        if key not in base:
            merged[key] = _copy_value(override[key])
    return merged


def _copy_value(value: object) -> object:
    """Return a deep copy of a value, handling nested dicts and lists.

    Args:
        value: The value to copy.

    Returns:
        An independent copy.

    """
    if isinstance(value, dict):
        return {k: _copy_value(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_copy_value(item) for item in value]
    return value


def _flatten_keys(data: dict[str, Any], prefix: str = "") -> dict[str, str]:
    """Flatten a nested dict into dotted-key form, returning keys mapped to themselves.

    Args:
        data: Nested dict to flatten.
        prefix: Current key prefix (used during recursion).

    Returns:
        A flat dict with dotted keys mapped to dotted keys (values ignored,
        only keys matter for provenance).

    """
    result: dict[str, str] = {}
    for key, value in data.items():
        full_key = f"{prefix}{key}" if not prefix else f"{prefix}.{key}"
        if isinstance(value, dict):
            result.update(_flatten_keys(value, full_key))
        else:
            result[full_key] = full_key
    return result


def _resolve_config_path(path: Path | None, environ: Mapping[str, str]) -> Path | None:
    """Determine the configuration file path.

    Explicit *path* takes precedence over ``$ISAAC_CORE_CONFIG``.

    Args:
        path: Explicitly supplied path, or ``None``.
        environ: Environment mapping.

    Returns:
        The resolved path, or ``None`` if neither source provides one.

    """
    if path is not None:
        return path
    env_path = environ.get(_CONFIG_ENV_VAR)
    if env_path:
        return Path(env_path)
    return None


def _build_provenance(
    config_path: Path | None,
    file_data: dict[str, Any],
    env_data: dict[str, Any],
    cli_data: dict[str, Any],
) -> dict[str, str]:
    """Compute provenance for every leaf key that was explicitly set.

    Keys from defaults are not tracked because the caller never supplied them.

    Args:
        config_path: File used, or ``None``.
        file_data: Parsed content from the TOML file layer.
        env_data: Parsed content from environment variables.
        cli_data: Parsed content from CLI overrides.

    Returns:
        A mapping from dotted key to human-readable source name.

    """
    provenance: dict[str, str] = {}
    if config_path is not None and file_data:
        for dotted_key in _flatten_keys(file_data):
            provenance[dotted_key] = f"file:{config_path}"
    if env_data:
        prefix = "ISAAC_CORE"
        for dotted_key in _flatten_keys(env_data):
            env_var_name = prefix + "__" + "__".join(seg.upper() for seg in dotted_key.split("."))
            provenance[dotted_key] = f"env:{env_var_name}"
    if cli_data:
        for dotted_key in _flatten_keys(cli_data):
            provenance[dotted_key] = "cli"
    return provenance


def _merge_layers(
    file_data: dict[str, Any],
    env_data: dict[str, Any],
    cli_data: dict[str, Any],
) -> dict[str, Any]:
    """Merge the three override layers on top of an empty base.

    Args:
        file_data: From the TOML file.
        env_data: From environment variables.
        cli_data: From CLI overrides.

    Returns:
        The merged dict ready for pydantic validation.

    """
    merged: dict[str, Any] = {}
    if file_data:
        merged = deep_merge(merged, file_data)
    if env_data:
        merged = deep_merge(merged, env_data)
    if cli_data:
        merged = deep_merge(merged, cli_data)
    return merged


def load_with_provenance(
    path: Path | None = None,
    environ: Mapping[str, str] | None = None,
    cli_overrides: Mapping[str, Any] | None = None,
) -> tuple[IsaacCoreConfig, dict[str, str]]:
    """Load and validate configuration, returning both the config and provenance.

    Resolution order, later wins:
        defaults -> TOML file -> env -> CLI

    Validation happens once at the end so a single error report covers everything.

    Args:
        path: Explicit path to a TOML config file.  If ``None``,
            ``$ISAAC_CORE_CONFIG`` is checked.
        environ: Environment variable mapping.  Defaults to :data:`os.environ`.
        cli_overrides: Dotted-key overrides from CLI flags.

    Returns:
        A tuple of the validated config and a provenance dict mapping every
        dotted key to the source that last set it.

    Raises:
        FileNotFoundError: From :func:`~isaac_core.config.sources.toml_file_source`
            if the config file does not exist.
        ValueError: From :func:`~isaac_core.config.sources.toml_file_source` if
            the TOML is malformed.
        pydantic.ValidationError: If the merged data fails schema validation.

    """
    env_map: Mapping[str, str] = environ if environ is not None else os.environ

    # 1. Defaults (empty; pydantic fills in its own)
    defaults_source()

    # 2. TOML file
    config_path = _resolve_config_path(path, env_map)
    file_data: dict[str, Any] = toml_file_source(config_path) if config_path is not None else {}

    # 3. Environment variables
    env_data: dict[str, Any] = env_source(env_map)

    # 4. CLI overrides
    cli_data: dict[str, Any] = cli_source(cli_overrides) if cli_overrides else {}

    # Merge and validate.
    merged = _merge_layers(file_data, env_data, cli_data)
    provenance = _build_provenance(config_path, file_data, env_data, cli_data)
    config = IsaacCoreConfig(**merged)
    return config, provenance


def load(
    path: Path | None = None,
    environ: Mapping[str, str] | None = None,
    cli_overrides: Mapping[str, Any] | None = None,
) -> IsaacCoreConfig:
    """Load, merge, and validate configuration from all layers.

    Convenience wrapper around :func:`load_with_provenance` that discards
    provenance.

    Args:
        path: Explicit path to a TOML config file.
        environ: Environment variable mapping.
        cli_overrides: Dotted-key overrides from CLI flags.

    Returns:
        A validated :class:`~isaac_core.config.schema.IsaacCoreConfig`.

    """
    config, _ = load_with_provenance(path=path, environ=environ, cli_overrides=cli_overrides)
    return config


# ---------------------------------------------------------------------------
# TOML rendering
# ---------------------------------------------------------------------------

# Mapping from Python types to TOML formatters.
_TOML_BOOL_TRUE = "true"
_TOML_BOOL_FALSE = "false"


def _render_scalar(value: object) -> str | None:
    """Render a single scalar value to TOML, or return None if not a scalar.

    Args:
        value: The value to render.

    Returns:
        TOML string for scalar types, or ``None`` if the value is a container.

    """
    if isinstance(value, bool):
        return _TOML_BOOL_TRUE if value else _TOML_BOOL_FALSE
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, str):
        escaped = value.replace("\\", "\\\\").replace('"', '\\"')
        return f'"{escaped}"'
    if isinstance(value, Path):
        escaped = str(value).replace("\\", "\\\\").replace('"', '\\"')
        return f'"{escaped}"'
    return None


def _render_value(value: object) -> str:
    """Render a Python value as a TOML-compatible string.

    Args:
        value: The value to render.

    Returns:
        TOML representation.

    """
    scalar = _render_scalar(value)
    if scalar is not None:
        return scalar
    if isinstance(value, (list, tuple)):
        items = ", ".join(_render_value(item) for item in value)
        return f"[{items}]"
    if isinstance(value, dict):
        items = ", ".join(f"{k} = {_render_value(v)}" for k, v in value.items())
        return f"{{{items}}}"
    # Fallback for enums and other objects with a .value attribute.
    if hasattr(value, "value"):
        return _render_value(value.value)
    return f'"{value}"'


def _dump_section(data: dict[str, object], lines: list[str], prefix: str) -> None:
    """Recursively render a dict as TOML table sections and key/value pairs.

    Args:
        data: Nested dict to render.
        lines: Accumulator for output lines.
        prefix: Current TOML table path.

    """
    for key, value in data.items():
        if isinstance(value, dict):
            sub_prefix = f"{prefix}.{key}" if prefix else key
            lines.append("")
            lines.append(f"[{sub_prefix}]")
            _dump_section(value, lines, sub_prefix)
        elif value is not None:
            lines.append(f"{key} = {_render_value(value)}")


def dump_toml(config: IsaacCoreConfig) -> str:
    """Render a resolved configuration as a TOML string.

    Intended for ``isaac-core config dump``. The output, when loaded back, produces
    an equal configuration.

    Args:
        config: A validated configuration.

    Returns:
        A TOML string representing the full configuration.

    """
    data = config.model_dump()
    lines: list[str] = []

    # Separate prim_overrides (array of tables) from the rest.
    prim_overrides = data.pop("prim_overrides", ())

    # Top-level scalar values first (unlikely but for completeness).
    for key, value in list(data.items()):
        if not isinstance(value, dict):
            if value is not None:
                lines.append(f"{key} = {_render_value(value)}")
            data.pop(key)

    # Top-level sections.
    for key, value in data.items():
        if not isinstance(value, dict):
            continue
        lines.append("")
        lines.append(f"[{key}]")
        _dump_section(value, lines, key)

    # prim_overrides is an array of tables.
    for override in prim_overrides:
        lines.append("")
        lines.append("[[prim_overrides]]")
        for k, v in override.items():
            lines.append(f"{k} = {_render_value(v)}")

    lines.append("")
    return "\n".join(lines)
