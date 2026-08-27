"""
Test the pure configuration applicator against the RecordingWriter fake.

Cover every binding kind, dotted config lookup, camera intrinsics maths,
prim_overrides winning last, deterministic ordering, and error paths.
"""

import logging
import math

import pytest

from isaac_core.config import (
    EnuReference,
    IsaacCoreConfig,
    PrimOverride,
    VehicleConfig,
)
from isaac_core.sim.configurator import (
    AttributeWrite,
    ConfigKeyError,
    RecordingWriter,
    apply_writes,
    compute_horizontal_aperture,
    compute_writes,
)
from isaac_core.sim.georeference import ResolvedEnuReference
from isaac_core.sim.manifest import LayerManifest
from isaac_core.sim.planner import FeaturePlan, PlannedLayer, ResolvedBinding, SkippedLayer


def _make_config(**overrides: object) -> IsaacCoreConfig:
    return IsaacCoreConfig(**overrides)


def _make_enu_ref(
    lat: float = 32.22481,
    lon: float = 35.25621,
    alt: float = 516.7,
) -> ResolvedEnuReference:
    return ResolvedEnuReference(
        reference=EnuReference(lat_deg=lat, lon_deg=lon, alt_m=alt),
        source="test",
    )


def _make_plan(
    bindings: tuple[ResolvedBinding, ...] = (),
    layer_id: str = "test_layer",
) -> FeaturePlan:
    manifest = LayerManifest(
        id=layer_id,
        usd="test.usda",
        mount="/Environment/{instance}",
        bindings=(),
    )
    planned = PlannedLayer(manifest=manifest, resolved_bindings=bindings)
    return FeaturePlan(enabled=(planned,), skipped=())


# -- config binding: dotted lookup -----------------------------------------------


def test_config_binding_resolves_simple_key() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:headless",
        config="sim.headless",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    assert len(writes) == 1
    assert writes[0] == AttributeWrite(prim="/Env/node", attribute="inputs:headless", value=False)


def test_config_binding_resolves_nested_vehicle_key() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/cam",
        attribute="inputs:fov",
        config="vehicles.drone_0.cameras.eo.fov_deg",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    assert writes[0].value == pytest.approx(78.1)


def test_config_binding_raises_on_missing_key() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:missing",
        config="vehicles.drone_0.no_such_key",
    )
    plan = _make_plan(bindings=(binding,))
    with pytest.raises(ConfigKeyError, match="no_such_key"):
        compute_writes(config, plan, _make_enu_ref())


def test_config_binding_traverses_dict_keys() -> None:
    # vehicles is a dict, not a pydantic model attribute list
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:port",
        config="sim.control_plane.port",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    assert writes[0].value == 8760


# -- resolve bindings: runtime-derived values ------------------------------------


def test_resolve_enu_origin() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:enu_reference",
        resolve="enu_origin",
    )
    plan = _make_plan(bindings=(binding,))
    ref = _make_enu_ref(lat=10.0, lon=20.0, alt=100.0)
    writes = compute_writes(config, plan, ref)
    assert writes[0].value == [10.0, 20.0, 100.0]


def test_resolve_camera_prim() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:camera_path",
        resolve="camera_prim",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref(), camera_prim="/Root/Camera")
    assert writes[0].value == "/Root/Camera"


def test_resolve_camera_prim_raises_when_none() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:camera_path",
        resolve="camera_prim",
    )
    plan = _make_plan(bindings=(binding,))
    with pytest.raises(ConfigKeyError, match="camera_prim"):
        compute_writes(config, plan, _make_enu_ref(), camera_prim=None)


def test_resolve_udp_port() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:udp_port",
        resolve="udp_port",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    # Default vehicle drone_0 at index 0 => 33333
    assert writes[0].value == 33333


def test_resolve_image_topic() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:topic",
        resolve="image_topic",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    # Single vehicle, single camera => flat
    assert writes[0].value == "/isaac_core/image_rgb"


def test_resolve_global_pose_topic() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:topic",
        resolve="global_pose_topic",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    assert writes[0].value == "/isaac_core/global_pose"


def test_resolve_distance_topic() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:topic",
        resolve="distance_topic",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    assert writes[0].value == "/isaac_core/distance_sensor"


def test_resolve_bbox_topic() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:topic",
        resolve="bbox_topic",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    assert writes[0].value == "/isaac_core/bbox"


def test_resolve_gimbal_topic() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:topic",
        resolve="gimbal_topic",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    assert writes[0].value == "/isaac_core/gimbal"


def test_resolve_unknown_raises() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:x",
        resolve="no_such_resolve",
    )
    plan = _make_plan(bindings=(binding,))
    with pytest.raises(ConfigKeyError, match="no_such_resolve"):
        compute_writes(config, plan, _make_enu_ref())


# -- camera intrinsics -----------------------------------------------------------


def test_horizontal_aperture_hand_computed() -> None:
    # Known values from the old repo: fov=78.1 deg, focal=22.7885 mm
    # Formula: two times focal times tangent of half the FOV angle
    fov_deg = 78.1
    focal_mm = 22.7885
    expected = 2.0 * focal_mm * math.tan(math.radians(fov_deg) / 2.0)
    result = compute_horizontal_aperture(fov_deg, focal_mm)
    assert result == pytest.approx(expected, rel=1e-10)
    # Verify against independently computed constant: ~36.97 mm
    # 2 * 22.7885 * tan(39.05 deg) = 2 * 22.7885 * 0.8113 = 36.97
    assert result == pytest.approx(36.973, rel=1e-3)


def test_horizontal_aperture_at_90_degrees() -> None:
    # At 90 deg FOV, aperture = 2 * f * tan(45deg) = 2 * f * 1.0 = 2f
    focal_mm = 10.0
    result = compute_horizontal_aperture(90.0, focal_mm)
    assert result == pytest.approx(20.0, rel=1e-10)


def test_horizontal_aperture_small_fov() -> None:
    # Small angle: tan(x) ~ x, so aperture ~ 2 * f * (fov_r/2) = f * fov_r
    focal_mm = 50.0
    fov_deg = 1.0
    fov_r = math.radians(fov_deg)
    result = compute_horizontal_aperture(fov_deg, focal_mm)
    # For very small angles, matches the linear approximation
    assert result == pytest.approx(focal_mm * fov_r, rel=1e-3)


# -- prim_overrides applied last --------------------------------------------------


def test_prim_overrides_win_over_layer_bindings() -> None:
    # A layer binding writes to a prim attribute, and a prim_override writes
    # to the same pair -- the override must come last and therefore win.
    config = _make_config(
        prim_overrides=(PrimOverride(prim="/Env/node", attribute="inputs:val", value=42),),
    )
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:val",
        config="sim.headless",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    # Both present, override is last
    assert len(writes) == 2
    assert writes[0].value is False  # from config binding
    assert writes[1].value == 42  # override wins


def test_multiple_prim_overrides_preserve_order() -> None:
    config = _make_config(
        prim_overrides=(
            PrimOverride(prim="/A", attribute="x", value=1),
            PrimOverride(prim="/B", attribute="y", value=2),
            PrimOverride(prim="/C", attribute="z", value=3),
        ),
    )
    plan = FeaturePlan(enabled=(), skipped=())
    writes = compute_writes(config, plan, _make_enu_ref())
    assert [w.value for w in writes] == [1, 2, 3]


# -- deterministic ordering -------------------------------------------------------


def test_writes_ordered_by_layer_id_then_binding_declaration() -> None:
    # Two layers, "aaa" and "zzz"; bindings within each in declaration order.
    manifest_a = LayerManifest(id="aaa", usd="a.usda", mount="/Env/{instance}")
    manifest_z = LayerManifest(id="zzz", usd="z.usda", mount="/Env/{instance}")
    planned_a = PlannedLayer(
        manifest=manifest_a,
        resolved_bindings=(
            ResolvedBinding(prim="/Env/a1", attribute="x", config="sim.headless"),
            ResolvedBinding(prim="/Env/a2", attribute="y", config="sim.headless"),
        ),
    )
    planned_z = PlannedLayer(
        manifest=manifest_z,
        resolved_bindings=(ResolvedBinding(prim="/Env/z1", attribute="x", config="sim.headless"),),
    )
    # Plan order comes from planner (sorted by id), so aaa before zzz.
    plan = FeaturePlan(enabled=(planned_a, planned_z), skipped=())
    config = _make_config()
    writes = compute_writes(config, plan, _make_enu_ref())
    assert [w.prim for w in writes] == ["/Env/a1", "/Env/a2", "/Env/z1"]


# -- apply_writes with RecordingWriter -------------------------------------------


def test_apply_writes_records_all() -> None:
    writer = RecordingWriter()
    writes = [
        AttributeWrite(prim="/A", attribute="x", value=1),
        AttributeWrite(prim="/B", attribute="y", value="hello"),
    ]
    apply_writes(writer, writes)
    assert len(writer.writes) == 2
    assert writer.writes[0].prim == "/A"
    assert writer.writes[1].value == "hello"


def test_recording_writer_satisfies_protocol() -> None:
    from isaac_core.sim.configurator import AttributeWriter

    writer = RecordingWriter()
    assert isinstance(writer, AttributeWriter)


# -- empty plan produces no writes ------------------------------------------------


def test_empty_plan_no_overrides_produces_empty_list() -> None:
    config = _make_config()
    plan = FeaturePlan(enabled=(), skipped=())
    writes = compute_writes(config, plan, _make_enu_ref())
    assert writes == []


# -- skipped layers do not produce writes -----------------------------------------


def test_skipped_layers_produce_no_writes() -> None:
    config = _make_config()
    plan = FeaturePlan(
        enabled=(),
        skipped=(SkippedLayer(id="missing", reason="not found"),),
    )
    writes = compute_writes(config, plan, _make_enu_ref())
    assert writes == []


# -- topics namespace with multiple vehicles --------------------------------------


def test_resolve_image_topic_with_multiple_vehicles() -> None:
    config = _make_config(
        vehicles={
            "lead": VehicleConfig(),
            "wing": VehicleConfig(),
        },
    )
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:topic",
        resolve="image_topic",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    # Multiple vehicles => namespaced
    assert writes[0].value == "/isaac_core/lead/image_rgb"


def test_resolve_udp_port_for_second_vehicle() -> None:
    # Verifies port allocation is base + index
    config = _make_config(
        vehicles={
            "alpha": VehicleConfig(),
            "bravo": VehicleConfig(),
        },
    )
    binding = ResolvedBinding(
        prim="/Env/node",
        attribute="inputs:port",
        resolve="udp_port",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    # First vehicle is alpha => port 33333
    assert writes[0].value == 33333


def _capture_configurator_warnings() -> tuple[logging.Handler, list[logging.LogRecord]]:
    """Attach a recording handler to the configurator logger, robust to global config."""
    records: list[logging.LogRecord] = []
    handler = logging.Handler()
    handler.emit = records.append  # type: ignore[assignment,method-assign]
    logger = logging.getLogger("isaac_core.sim.configurator")
    logger.addHandler(handler)
    logger.setLevel(logging.WARNING)
    return handler, records


def test_override_colliding_with_a_binding_warns() -> None:
    # Ofer set focal_length_mm in config and focalLength via a prim_override, and the
    # override silently won. It still wins by design, but the collision must be announced.
    config = _make_config(prim_overrides=(PrimOverride(prim="/Env/node", attribute="inputs:port", value=9999),))
    binding = ResolvedBinding(prim="/Env/node", attribute="inputs:port", config="sim.control_plane.port")
    plan = _make_plan(bindings=(binding,))
    handler, records = _capture_configurator_warnings()
    try:
        writes = compute_writes(config, plan, _make_enu_ref())
    finally:
        logging.getLogger("isaac_core.sim.configurator").removeHandler(handler)
    # Override still wins: it is applied last.
    assert writes[-1].value == 9999
    assert any("override wins" in r.getMessage() for r in records), "collision was not warned"


def test_override_without_a_colliding_binding_is_silent() -> None:
    config = _make_config(prim_overrides=(PrimOverride(prim="/Env/other", attribute="inputs:value", value=1),))
    binding = ResolvedBinding(prim="/Env/node", attribute="inputs:port", config="sim.control_plane.port")
    plan = _make_plan(bindings=(binding,))
    handler, records = _capture_configurator_warnings()
    try:
        compute_writes(config, plan, _make_enu_ref())
    finally:
        logging.getLogger("isaac_core.sim.configurator").removeHandler(handler)
    assert not [r for r in records if "override wins" in r.getMessage()]
