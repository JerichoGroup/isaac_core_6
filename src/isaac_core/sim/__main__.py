"""Entry point: ``<isaac_python> -m isaac_core.sim --config <path>``.

Sets the SimulationApp launch configuration BEFORE importing anything that triggers Kit
initialisation. Configuration is passed explicitly rather than through an environment
variable, so nothing depends on import order.

Run with::

    /path/to/isaacsim/python.sh -m isaac_core.sim --config config/default.toml

"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Mapping

    from isaac_core.config import IsaacCoreConfig
    from isaac_core.sim.manifest import LayerManifest
    from isaac_core.sim.planner import FeaturePlan

logger = logging.getLogger(__name__)


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments.

    Args:
        argv: Argument list (defaults to sys.argv[1:]).

    Returns:
        Parsed namespace.

    """
    parser = argparse.ArgumentParser(
        prog="python -m isaac_core.sim",
        description="Launch the Isaac Core simulation runtime.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Path to a TOML config file (layered on top of defaults).",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=False,
        help="Run without a GUI window.",
    )
    parser.add_argument(
        "--scene",
        type=str,
        default=None,
        help="Scene name or absolute path (overrides config).",
    )
    return parser.parse_args(argv)


def _configure_logging(config: IsaacCoreConfig) -> None:
    """Apply the resolved logging configuration.

    Args:
        config: The resolved :class:`~isaac_core.config.IsaacCoreConfig`.

    """
    logging_config = config.logging
    level = getattr(logging, str(logging_config.level).upper(), logging.INFO)
    logging.getLogger("isaac_core").setLevel(level)

    # `isaac_logs = false` means Isaac's chatter stays out of the terminal. Third-party
    # Python loggers are the bulk of it, so the root logger is what has to be raised;
    # Kit's own stream is handled separately with startup arguments.
    if logging_config.isaac_logs:
        logging.getLogger().setLevel(logging.INFO)
        return

    logging.getLogger().setLevel(logging.WARNING)

    # A level set on these loggers does not survive: each extension sets its own on the way
    # up, and the noisiest lines are emitted *while* extensions load, so there is no moment
    # afterwards to intervene. Filtering at the handler drops the records whenever they are
    # emitted, which is the only approach that actually works here.
    quiet = tuple(logging_config.quiet_loggers)

    def _drop_noisy(record: logging.LogRecord) -> bool:
        if record.levelno >= logging.WARNING:
            return True
        return not any(record.name == name or record.name.startswith(f"{name}.") for name in quiet)

    for handler in logging.getLogger().handlers:
        handler.addFilter(_drop_noisy)


def main(argv: list[str] | None = None) -> None:
    """Launch the simulation.

    This function is the sole entry point. It resolves config, plans features,
    then creates the runtime. The import of ``isaac_core.sim.runtime`` (which
    eventually imports ``SimulationApp``) happens AFTER config is ready.

    Args:
        argv: Command-line arguments. ``None`` means use ``sys.argv[1:]``.

    """
    args = _parse_args(argv)

    # Root stays quiet on purpose. Setting the root logger to INFO routes every third-party
    # Python logger -- asyncio, Isaac's own modules, Cesium -- to the terminal: over 2,800
    # lines of a 3,400-line launch came from that alone, burying our output.
    logging.basicConfig(
        level=logging.WARNING,
        format="%(asctime)s | %(name)s | %(levelname)s | %(message)s",
    )
    logging.getLogger("isaac_core").setLevel(logging.INFO)

    from isaac_core.config import load
    from isaac_core.sim.discovery import discover_layers

    cli_overrides: dict[str, object] = {}
    if args.headless:
        cli_overrides["sim.headless"] = True
    if args.scene:
        cli_overrides["sim.scene"] = args.scene

    config = load(path=args.config, cli_overrides=cli_overrides)
    _configure_logging(config)

    layer_paths = _resolve_layer_search_paths(config)
    manifests = discover_layers(layer_paths)

    # Pre-plan with an empty inspector to get the mount list; real capabilities are checked once the
    # stage opens. Planned per vehicle, because a single plan would hand every vehicle the first
    # vehicle's ports and topic names.
    pre_plan = _plan_all_vehicles(config, manifests)

    scene_path = _resolve_scene_path(config)

    from isaac_core.sim.runtime import SimulationRuntime

    with SimulationRuntime(config, pre_plan) as runtime:
        runtime.start()
        try:
            runtime.open_stage(scene_path, layer_paths)
        except Exception:
            # Kit suppresses stdout and our logging filter drops third-party handlers, so an
            # exception raised while composing the stage reaches nobody: the process exits 0
            # having printed nothing about why. That has already cost real debugging time twice
            # (a ConfigKeyError from an unresolvable binding, and a georeference mismatch), so
            # every startup failure is logged through our own logger -- which is visible -- and
            # then re-raised so the exit status is still non-zero.
            logger.exception("failed to open the stage; the simulator cannot start")
            raise
        runtime.run()


def _plan_all_vehicles(
    config: IsaacCoreConfig,
    manifests: Mapping[str, LayerManifest],
) -> FeaturePlan:
    """Plan the feature layers for every configured vehicle and merge the result.

    Each vehicle gets its own pass so every planned layer records the instance and camera it
    belongs to, which is what lets the configurator resolve per-vehicle ports and topics. Skipped
    layers are reported once per vehicle that could not have them, since the reason can differ.

    Args:
        config: The resolved configuration.
        manifests: All discovered layer manifests.

    Returns:
        One plan containing every vehicle's layers.

    """
    from isaac_core.sim.capabilities import FakeStageInspector, probe
    from isaac_core.sim.planner import FeaturePlan, plan_features

    enabled: list[Any] = []
    skipped: list[Any] = []
    for vehicle_id, vehicle in config.vehicles.items():
        plan = plan_features(
            # Derived, not just the explicit list: each vehicle's pose_source implies the
            # camera layer that reads it, so a minimal config composes and flies.
            requested_ids=list(config.required_feature_ids()),
            # Mount under the vehicle's own id, so prim paths match the manifests'
            # {instance} templates. Without this the planner's "default" placeholder
            # produced /Environment/default, which no binding refers to.
            instance=vehicle_id,
            manifests=manifests,
            capabilities=probe(FakeStageInspector(prims=frozenset())),
            strict=config.sim.strict_features,
        )
        enabled.extend(plan.enabled)
        skipped.extend(plan.skipped)
    return FeaturePlan(enabled=tuple(enabled), skipped=tuple(skipped))


def _resolve_layer_search_paths(config: IsaacCoreConfig) -> tuple[Path, ...]:
    """Collect all layer search paths from config and built-in locations.

    Args:
        config: The resolved configuration.

    Returns:
        Tuple of directories to scan for layer.toml files.

    """
    paths: list[Path] = []
    for p in config.assets.layer_search_paths:
        resolved = Path(p).expanduser().resolve()
        if resolved.is_dir():
            paths.append(resolved)

    # Layers shipped inside the package: manifest and USD side by side, which is the same shape a
    # third-party layer takes. There is no repo-relative fallback, so an installed package behaves
    # exactly like a source checkout.
    builtin = Path(__file__).resolve().parent.parent / "assets" / "layers"
    if builtin.is_dir():
        paths.append(builtin)

    return tuple(paths)


def _resolve_scene_path(config: IsaacCoreConfig) -> Path:
    """Resolve the scene name or path from config to an absolute file path.

    Args:
        config: The resolved configuration.

    Returns:
        Absolute path to the scene USD file.

    Raises:
        FileNotFoundError: If the scene cannot be found.

    """
    scene = config.sim.scene
    candidate = Path(scene).expanduser()
    # An absolute or relative path that points at a real file is used directly. Checking
    # is_file() (not just is_absolute) means "./my_scenes/foo.usda" resolves against the
    # working directory, which is what a path with a slash or suffix obviously means.
    if candidate.is_file():
        return candidate.resolve()

    # Search in configured asset paths, then built-in.
    for search_dir in config.assets.search_paths:
        resolved = Path(search_dir).expanduser().resolve()
        for suffix in (".usda", ".usd", ""):
            path = resolved / f"{scene}{suffix}"
            if path.is_file():
                return path

    # Scenes shipped inside the package.
    builtin = Path(__file__).resolve().parent.parent / "assets" / "scenes"
    for suffix in (".usda", ".usd", ""):
        path = builtin / f"{scene}{suffix}"
        if path.is_file():
            return path

    msg = f"scene {scene!r} not found in search paths or built-in locations"
    raise FileNotFoundError(msg)


if __name__ == "__main__":
    main()
