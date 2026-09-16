"""Prove every config key changes something, by measuring what the config layer produces.

`test_no_dead_keys.py` greps for a field name appearing somewhere in the tree, which a mention in a
comment satisfies. This is the stronger version: set each key to a different valid value and check
that the *product* of the configuration layer changes -- the feature plan, the attribute writes
applied to the stage, or the resolved per-vehicle naming, ports and mounts.

The resolved config itself is deliberately excluded from that signature. Including it would make
every key pass by definition, since "the value I set is in the config" says nothing about whether
anything consumes it.

Keys whose entire effect happens inside a running Isaac Sim or a spawned process cannot show up in a
static signature. Those are listed in ``RUNTIME_ONLY`` together with what does cover them, and the
list is kept honest from both directions: a key that starts showing up must leave it.
"""

from __future__ import annotations

import json
from pathlib import Path
import types
import typing
from typing import Any, Final

from isaac_core.config import IsaacCoreConfig, load
from isaac_core.config.schema import CameraConfig, VehicleConfig
from isaac_core.sim.capabilities import FakeStageInspector, probe
from isaac_core.sim.configurator import compute_writes
from isaac_core.sim.discovery import discover_layers
from isaac_core.sim.georeference import ResolvedEnuReference
from isaac_core.sim.planner import plan_features

_LAYERS_ROOT: Final = Path(__file__).resolve().parents[3] / "src" / "isaac_core" / "assets" / "layers"

# Keys whose whole effect is inside a running Isaac Sim or a spawned process, with what covers them.
RUNTIME_ONLY: Final[dict[str, str]] = {
    "assets.hdri": "dome light repointed during composition; test_composer_assets.py",
    "assets.layer_search_paths": "layer discovery; test_discovery.py",
    "assets.search_paths": "scene and HDRI resolution; test_scene_resolution.py",
    "cesium.delete_cache_on_launch": "cache deleted before app start; test_composer_assets.py",
    "cesium.max_cached_bytes": "tile tuning attributes; test_tile_tuning.py",
    "cesium.max_screen_space_error": "tile tuning attributes; test_tile_tuning.py",
    "cesium.max_simultaneous_tile_loads": "tile tuning attributes; test_tile_tuning.py",
    "cesium.preload_ancestors": "tile tuning attributes; test_tile_tuning.py",
    "cesium.preload_siblings": "tile tuning attributes; test_tile_tuning.py",
    "cesium.tileset_server_url": "tileset prim URL rewrite; test_tileset_url.py",
    "cesium.tilesets_root": "subtree searched for tilesets; test_tileset_url.py",
    "logging.isaac_logs": "Isaac's own logging; test_logging_config.py",
    "logging.level": "logging setup; test_logging_config.py",
    "logging.quiet_loggers": "logger silencing; test_logging_config.py",
    "ros2.domain_id": "exported as ROS_DOMAIN_ID before the bridge starts; test_gui_setup.py",
    "sidecar.enabled": "whether sidecar services start; test_service.py",
    "sim.boot_extensions": "extensions enabled before the app starts; test_gui_setup.py",
    "sim.control_plane.enabled": "whether the control server binds; test_runtime_handlers.py",
    "sim.control_plane.host": "control server bind address; test_server.py",
    "sim.control_plane.output_root": "capture path confinement; test_capture_frame.py",
    "sim.control_plane.port": "control server bind port; every live test connects to it",
    "sim.control_plane.token": "control server authentication; test_server.py",
    "sim.experience": "Kit experience file chosen at app start; test_gui_setup.py",
    "sim.extension_search_paths": "search paths registered at app start; test_gui_setup.py",
    "sim.extensions": "extensions enabled at app start; test_gui_setup.py",
    "sim.headless": "SimulationApp argument; the system tests run both ways",
    "sim.physics_dt": "timeline timestep; test_startup_warmup.py",
    "sim.renderer": "RTX render mode at app start; test_renderer.py",
    "sim.scene": "which USD is opened; test_scene_resolution.py",
    "sim.stage_units_in_meters": "stage metadata; test_stage_units.py",
    "sim.strict_features": "raise instead of skip when a layer cannot compose; test_planner.py",
    "sim.viewport_camera": "which camera the viewport looks through; test_gui_setup.py",
    "vehicles.drone_0.gimbal.max_rate_deg_s": "slew rate applied per frame; test_gimbal.py",
}

# Free-form mappings with no default entries, so there is nothing to vary without inventing a
# definition. Both are exercised end to end by their own tests instead.
_NOT_VARIABLE: Final = frozenset({"layers", "sidecar.services"})


def _unwrap(annotation: Any) -> Any:
    """Reduce Optional, Union, Annotated and Literal annotations to a concrete type."""
    origin = typing.get_origin(annotation)
    if origin in (typing.Union, types.UnionType):
        args = [a for a in typing.get_args(annotation) if a is not type(None)]
        return _unwrap(args[0]) if args else annotation
    if origin is typing.Literal:
        return str
    if hasattr(annotation, "__metadata__"):
        return _unwrap(typing.get_args(annotation)[0])
    return annotation


# Exact keys whose alternative cannot be inferred from the type alone.
_EXACT_SAMPLES: Final[dict[str, Any]] = {
    "features.enabled": ["camera_udp", "distance_sensor"],
    "logging.level": "debug",
    "logging.quiet_loggers": ["omni"],
    "prim_overrides": [{"prim": "/World/x", "attribute": "inputs:y", "value": 1.0}],
    "ros2.domain_id": 42,
    "sim.boot_extensions": ["omni.graph.action"],
    "sim.control_plane.host": "0.0.0.0",
    "sim.control_plane.output_root": "/tmp/other_out",
    "sim.experience": "/tmp/other.kit",
    "sim.extension_search_paths": ["/tmp/some_dir"],
    "sim.extensions": ["omni.graph.action"],
    "sim.renderer": "PathTracing",
    "sim.scene": "some-other-scene",
    "vehicles.drone_0.mount": "/World/Custom/{instance}",
}

# Suffix-matched samples, tried in order. A callable receives the current value.
_SUFFIX_SAMPLES: Final[tuple[tuple[str, Any], ...]] = (
    # The enum is world or body; there is no ned, because the wire-to-stage conversion happens once
    # on the way in. Compare by value, since an enum member's str() is not its value.
    ("rotation_frame", lambda current: "body" if getattr(current, "value", current) == "world" else "world"),
    ("pose_source", lambda current: "ros" if getattr(current, "value", current) != "ros" else "udp"),
    ("resolution", [640, 480]),
    ("mavros_namespace", "/custom_mavros"),
    ("topic", "/custom/topic_name"),
    ("mount_path", "/custom_mount"),
    ("hdri", "/tmp/some.exr"),
    ("tileset_server_url", "http://192.0.2.9:8088"),
    ("search_paths", ["/tmp/some_dir"]),
    ("token", "a-token"),
    ("rtsp_port", 9999),
    ("udp_port", 41234),
)


def _by_type(annotation: Any, current: Any) -> Any:
    """Return an alternative chosen from the annotation's concrete type."""
    base = _unwrap(annotation)
    if base is bool or isinstance(current, bool):
        return (not current) if isinstance(current, bool) else True
    if base is int:
        return (current + 7) if isinstance(current, int) else 4242
    if base is float:
        return (current + 3.5) if isinstance(current, (int, float)) and current is not None else 12.5
    if base is str:
        return f"{current}_changed" if isinstance(current, str) and current else "changed"
    return None


def _sample(annotation: Any, current: Any, key: str) -> Any:
    """Return a valid value that differs from *current*, or ``None`` if none can be invented."""
    if key in _EXACT_SAMPLES:
        return _EXACT_SAMPLES[key]
    for suffix, sample in _SUFFIX_SAMPLES:
        if key.endswith(suffix):
            return sample(current) if callable(sample) else sample
    return _by_type(annotation, current)


def _leaf_keys() -> dict[str, tuple[Any, Any]]:
    """Return dotted key -> (annotation, current value) for every leaf in the schema."""
    found: dict[str, tuple[Any, Any]] = {}

    def walk(model: Any, prefix: str) -> None:
        for name, field in type(model).model_fields.items():
            value = getattr(model, name)
            dotted = f"{prefix}.{name}" if prefix else name
            if hasattr(type(value), "model_fields"):
                walk(value, dotted)
            elif isinstance(value, dict) and value and hasattr(type(next(iter(value.values()))), "model_fields"):
                for child_key, child in value.items():
                    walk(child, f"{dotted}.{child_key}")
            else:
                found[dotted] = (field.annotation, value)

    walk(IsaacCoreConfig(), "")
    # Optional per-vehicle and per-camera fields default to None, so take their declared annotation
    # from the model rather than inferring a type from the value.
    for name, field in VehicleConfig.model_fields.items():
        key = f"vehicles.drone_0.{name}"
        if key in found:
            found[key] = (field.annotation, found[key][1])
    for name, field in CameraConfig.model_fields.items():
        key = f"vehicles.drone_0.cameras.eo.{name}"
        if key in found:
            found[key] = (field.annotation, found[key][1])
    return found


def _context_for(key: str) -> dict[str, str]:
    """Return overrides that make *key* relevant, so it is compared where it is used."""
    if key.startswith("vehicles.drone_0.distance_sensor"):
        return {"features.enabled": '["distance_sensor"]'}
    if key == "sim.control_plane.host":
        # Binding a non-loopback address without a token is refused, which is the point of the rule.
        return {"sim.control_plane.token": "a-token"}
    if key.endswith(("lla_topic", "orientation_topic", "mavros_namespace")):
        return {"vehicles.drone_0.pose_source": "ros"}
    return {}


def _signature(config: IsaacCoreConfig) -> str:
    """Return what the configuration layer produces, mirroring how the simulator plans."""
    parts: list[Any] = []
    manifests = discover_layers((_LAYERS_ROOT,))
    for vehicle_id, vehicle in config.vehicles.items():
        camera_id = next(iter(vehicle.cameras))
        try:
            plan = plan_features(
                requested_ids=list(config.required_feature_ids()),
                manifests=manifests,
                capabilities=probe(FakeStageInspector(prims=frozenset())),
                strict=config.sim.strict_features,
                instance=vehicle_id,
                camera=camera_id,
            )
            parts.append([planned.manifest.id for planned in plan.enabled])
            parts.append([(skipped.id, skipped.reason) for skipped in plan.skipped])
            enu = ResolvedEnuReference(reference=config.geo.enu_reference, source="config")
            writes = compute_writes(
                config, plan, enu, camera_prim="/World/cam", vehicle_id=vehicle_id, camera_id=camera_id
            )
            parts.append([(w.prim, w.attribute, repr(w.value)) for w in writes])
        except Exception as exc:
            parts.append(f"PLAN-ERROR {type(exc).__name__}: {exc}")
        parts.append(config.resolved_mount(vehicle_id))
        parts.append(config.resolved_udp_port(vehicle_id))
        for cid in vehicle.cameras:
            parts.append(config.resolved_rtsp_port(vehicle_id, cid))
            resolver = config.topic_resolver(vehicle_id, cid)
            parts.append([resolver.root, resolver.vehicle, resolver.camera])
    return json.dumps(parts, sort_keys=True, default=str)


def _classify() -> tuple[set[str], set[str], set[str], dict[str, str]]:
    """Return (observable, inert, untried, rejected) for every leaf key."""
    leaves = _leaf_keys()
    baselines: dict[str, str] = {}
    observable: set[str] = set()
    inert: set[str] = set()
    untried: set[str] = set()
    rejected: dict[str, str] = {}

    for key in sorted(leaves):
        annotation, current = leaves[key]
        new_value = _sample(annotation, current, key)
        if new_value is None or new_value == current:
            untried.add(key)
            continue
        context = _context_for(key)
        context_id = json.dumps(context, sort_keys=True)
        if context_id not in baselines:
            baselines[context_id] = _signature(load(cli_overrides=context) if context else IsaacCoreConfig())
        raw = json.dumps(new_value) if isinstance(new_value, (list, dict)) else str(new_value)
        try:
            changed = load(cli_overrides={**context, key: raw})
        except Exception as exc:
            # The sampled value must be valid, otherwise this test measures validation rather than
            # whether anything consumes the key.
            rejected[key] = f"{type(exc).__name__}: {exc}"
            continue
        try:
            after = _signature(changed)
        except Exception:
            observable.add(key)
            continue
        if after != baselines[context_id]:
            observable.add(key)
        else:
            inert.add(key)
    return observable, inert, untried, rejected


def test_the_key_walker_found_the_whole_schema() -> None:
    # Without this the classification below could pass by examining almost nothing.
    leaves = _leaf_keys()
    assert len(leaves) > 55, f"only found {len(leaves)} leaf keys; the walker is probably broken"
    for expected in (
        "sim.headless",
        "geo.enu_reference.lat_deg",
        "vehicles.drone_0.cameras.eo.fov_deg",
        "vehicles.drone_0.gimbal.start_pitch_deg",
        "cesium.tileset_server_url",
    ):
        assert expected in leaves, f"{expected} missing from the walk"


def test_every_config_key_changes_something() -> None:
    # A key that parses and validates but that nothing consumes is worse than not offering it,
    # because setting it looks like it worked.
    _, inert, _, _ = _classify()
    unexplained = inert - set(RUNTIME_ONLY)
    assert not unexplained, (
        "these keys changed no plan, no attribute write and no resolved name, so setting them "
        f"appears to do nothing: {sorted(unexplained)}"
    )


def test_the_runtime_only_list_does_not_hide_a_key_that_is_visible() -> None:
    # Keeps the list honest in the other direction: once a key shows up in the signature it must
    # leave, otherwise the list slowly becomes an excuse.
    observable, _, _, _ = _classify()
    stale = observable & set(RUNTIME_ONLY)
    assert not stale, f"these are observable and should leave RUNTIME_ONLY: {sorted(stale)}"


def test_only_free_form_mappings_go_unexercised() -> None:
    # Anything else unexercised means the sampler cannot invent a value for a key, which would hide
    # that key from both tests above.
    _, _, untried, _ = _classify()
    assert untried == _NOT_VARIABLE, f"expected only {sorted(_NOT_VARIABLE)} to be unexercised, got {sorted(untried)}"


def test_every_sampled_value_is_actually_valid() -> None:
    # If a sampled value is rejected, that key is never exercised and would silently drop out of the
    # checks above while looking like it had been covered.
    _, _, _, rejected = _classify()
    assert not rejected, f"the sampler produced invalid values, so these keys went unchecked: {rejected}"
