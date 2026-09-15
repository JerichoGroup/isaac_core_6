"""Typed, layered configuration.

One schema, and the two files that use it are the shipped ``config/default.toml``
and whatever a user passes to ``--config``. Layer manifests are a different thing
entirely: they declare *wiring* (which config key maps to which prim attribute),
never values, so configuring a run never means editing more than one file.

Validation is in-process at load time via pydantic. There is no separate
command-line validation step.
"""

from isaac_core.config.loader import deep_merge, dump_toml, load, load_with_provenance
from isaac_core.config.schema import (
    AssetsConfig,
    CameraConfig,
    CesiumConfig,
    ControlPlaneConfig,
    EnuReference,
    FeaturesConfig,
    GeoConfig,
    GimbalConfig,
    IsaacCoreConfig,
    LoggingConfig,
    PrimOverride,
    Ros2Config,
    SidecarConfig,
    SidecarServiceConfig,
    SimConfig,
    VehicleConfig,
)
from isaac_core.config.sources import cli_source, defaults_source, env_source, toml_file_source

__all__ = [
    "AssetsConfig",
    "CameraConfig",
    "CesiumConfig",
    "ControlPlaneConfig",
    "EnuReference",
    "FeaturesConfig",
    "GeoConfig",
    "GimbalConfig",
    "IsaacCoreConfig",
    "LoggingConfig",
    "PrimOverride",
    "Ros2Config",
    "SidecarConfig",
    "SidecarServiceConfig",
    "SimConfig",
    "VehicleConfig",
    "cli_source",
    "deep_merge",
    "defaults_source",
    "dump_toml",
    "env_source",
    "load",
    "load_with_provenance",
    "toml_file_source",
]
