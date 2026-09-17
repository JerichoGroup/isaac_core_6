"""Test the pure configuration applicator against the RecordingWriter fake.

Cover every binding kind, dotted config lookup, camera intrinsics maths,
prim_overrides winning last, deterministic ordering, and error paths.
"""

import logging
import math

import pytest

from isaac_core.config import (
    CameraConfig,
    EnuReference,
    IsaacCoreConfig,
    PrimOverride,
    VehicleConfig,
)
from isaac_core.sim.configurator import (
    AttributeWrite,
    ConfigKeyError,
    RecordingWriter,
    _resolve_horizontal_aperture,
    _resolve_vertical_aperture,
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
        config="vehicles.drone_0.camera.fov_deg",
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
    # Reported bug: focal_length_mm in config and focalLength via a prim_override, and the
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


# -- camera intrinsics: aperture override precedence -----------------------------


def _config_with_camera(**camera_overrides: object) -> IsaacCoreConfig:
    """Build a single-vehicle config whose one camera carries the given overrides."""
    return IsaacCoreConfig(vehicles={"drone_0": VehicleConfig(camera=CameraConfig(**camera_overrides))})


def test_default_apertures_match_the_historical_derivation() -> None:
    # The regression guard that matters most: with no overrides, the apertures are exactly
    # what the fov-only formula produced before this change.
    config = _make_config()
    camera = CameraConfig()
    expected_h = compute_horizontal_aperture(camera.fov_deg, camera.focal_length_mm)
    assert _resolve_horizontal_aperture(config) == pytest.approx(expected_h, rel=1e-12)
    assert _resolve_vertical_aperture(config) == pytest.approx(expected_h * (camera.height / camera.width), rel=1e-12)


def test_explicit_horizontal_aperture_wins_over_fov() -> None:
    # An explicit aperture is written verbatim; fov_deg is not consulted for this axis.
    config = _config_with_camera(horizontal_aperture_mm=12.5)
    assert _resolve_horizontal_aperture(config) == pytest.approx(12.5)


def test_explicit_vertical_aperture_is_independent_of_horizontal() -> None:
    # Setting only the vertical override pins the vertical axis while the horizontal axis
    # keeps deriving from fov -- the two axes are resolved independently.
    config = _config_with_camera(vertical_aperture_mm=7.0)
    camera = CameraConfig()
    expected_h = compute_horizontal_aperture(camera.fov_deg, camera.focal_length_mm)
    assert _resolve_vertical_aperture(config) == pytest.approx(7.0)
    assert _resolve_horizontal_aperture(config) == pytest.approx(expected_h, rel=1e-12)


def test_explicit_aperture_with_non_default_fov_warns_and_names_the_winner() -> None:
    # Setting both makes fov_deg meaningless for that axis; the collision is announced with
    # the aperture named as the winner, rather than silently ignoring one of the two.
    config = _config_with_camera(horizontal_aperture_mm=20.0, fov_deg=45.0)
    handler, records = _capture_configurator_warnings()
    try:
        result = _resolve_horizontal_aperture(config)
    finally:
        logging.getLogger("isaac_core.sim.configurator").removeHandler(handler)
    assert result == pytest.approx(20.0)  # aperture wins
    messages = [r.getMessage() for r in records]
    assert any("explicit aperture wins" in m and "fov_deg is ignored" in m for m in messages)


def test_explicit_aperture_with_default_fov_does_not_warn() -> None:
    # Leaving fov_deg at its default alongside an explicit aperture is unambiguous, so no
    # collision warning fires.
    config = _config_with_camera(horizontal_aperture_mm=20.0)
    handler, records = _capture_configurator_warnings()
    try:
        _resolve_horizontal_aperture(config)
    finally:
        logging.getLogger("isaac_core.sim.configurator").removeHandler(handler)
    assert not [r for r in records if "explicit aperture wins" in r.getMessage()]


def test_f_stop_default_keeps_depth_of_field_off() -> None:
    # USD reads fStop == 0 as depth of field OFF; the default must preserve the historical
    # pinhole image rather than silently blurring.
    assert CameraConfig().f_stop == 0.0


def test_focus_distance_binding_writes_the_configured_value() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/cam",
        attribute="focusDistance",
        config="vehicles.drone_0.camera.focus_distance",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    assert writes[0].value == pytest.approx(400.0)


def test_f_stop_binding_writes_the_configured_value() -> None:
    config = _make_config()
    binding = ResolvedBinding(
        prim="/Env/cam",
        attribute="fStop",
        config="vehicles.drone_0.camera.f_stop",
    )
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(config, plan, _make_enu_ref())
    assert writes[0].value == pytest.approx(0.0)


# -- vehicle- and camera-scoped resolvers (M6 swarm) -----------------------------
#
# The resolvers used to pick the first vehicle with next(iter(...)); now the identity is
# threaded explicitly through compute_writes. Every test below pins BOTH the single-vehicle
# output (the regression guard) and the multi-vehicle namespacing.


def _resolve_one(
    resolve: str,
    config: IsaacCoreConfig,
    *,
    vehicle_id: str | None = None,
) -> object:
    """Resolve a single runtime binding and return the written value."""
    binding = ResolvedBinding(prim="/Env/node", attribute="inputs:x", resolve=resolve)
    plan = _make_plan(bindings=(binding,))
    writes = compute_writes(
        config,
        plan,
        _make_enu_ref(),
        camera_prim="/Root/Camera",
        vehicle_id=vehicle_id,
    )
    return writes[0].value


# -- single-vehicle output must stay byte-identical for EVERY resolver ------------


def test_single_vehicle_every_resolver_unchanged() -> None:
    # The most important compatibility constraint: a single-vehicle config produces exactly
    # the topics/values it produced before per-vehicle plumbing existed. Literal strings so a
    # regression is obvious at a glance.
    config = _make_config()
    camera = CameraConfig()
    expected_h = compute_horizontal_aperture(camera.fov_deg, camera.focal_length_mm)
    assert _resolve_one("udp_port", config) == 33333
    assert _resolve_one("image_topic", config) == "/isaac_core/image_rgb"
    assert _resolve_one("global_pose_topic", config) == "/isaac_core/global_pose"
    assert _resolve_one("distance_topic", config) == "/isaac_core/distance_sensor"
    assert _resolve_one("bbox_topic", config) == "/isaac_core/bbox"
    assert _resolve_one("rtsp_mount_path", config) == "/stream"
    assert _resolve_one("lla_topic", config) == "/mavros/global_position/global"
    assert _resolve_one("orientation_topic", config) == "/mavros/local_position/pose"
    assert _resolve_one("camera_horizontal_aperture", config) == pytest.approx(expected_h, rel=1e-12)
    assert _resolve_one("camera_vertical_aperture", config) == pytest.approx(
        expected_h * (camera.height / camera.width), rel=1e-12
    )
    assert _resolve_one("enu_origin", config) == [32.22481, 35.25621, 516.7]
    assert _resolve_one("camera_prim", config) == "/Root/Camera"


# -- two vehicles produce distinct, namespaced topics -----------------------------


def test_two_vehicles_topics_are_namespaced_per_vehicle() -> None:
    config = _make_config(vehicles={"lead": VehicleConfig(), "wing": VehicleConfig()})
    assert _resolve_one("global_pose_topic", config, vehicle_id="lead") == "/isaac_core/lead/global_pose"
    assert _resolve_one("global_pose_topic", config, vehicle_id="wing") == "/isaac_core/wing/global_pose"
    assert _resolve_one("image_topic", config, vehicle_id="lead") == "/isaac_core/lead/image_rgb"
    assert _resolve_one("image_topic", config, vehicle_id="wing") == "/isaac_core/wing/image_rgb"
    assert _resolve_one("distance_topic", config, vehicle_id="wing") == "/isaac_core/wing/distance_sensor"
    assert _resolve_one("bbox_topic", config, vehicle_id="wing") == "/isaac_core/wing/bbox"


def test_two_vehicles_produce_distinct_udp_ports() -> None:
    # Ports are base + declaration index, so distinct vehicles never share one.
    config = _make_config(vehicles={"lead": VehicleConfig(), "wing": VehicleConfig()})
    assert _resolve_one("udp_port", config, vehicle_id="lead") == 33333
    assert _resolve_one("udp_port", config, vehicle_id="wing") == 33334


# -- one camera per vehicle: no camera segment can appear --------------------------


def test_a_single_camera_never_adds_a_camera_segment() -> None:
    # A vehicle has exactly one camera and it has no name, so nothing can be disambiguated by a
    # camera segment. Topics and RTSP mounts are namespaced by vehicle only.
    config = _make_config(vehicles={"drone_0": VehicleConfig()})
    assert _resolve_one("image_topic", config) == "/isaac_core/image_rgb"
    assert _resolve_one("rtsp_mount_path", config) == "/stream"


def test_two_vehicles_namespace_by_vehicle_alone() -> None:
    # Full namespacing is now one level: index i of one vehicle's stream still cannot alias
    # another's, because the vehicle segment is present.
    config = _make_config(vehicles={"lead": VehicleConfig(), "wing": VehicleConfig()})
    assert _resolve_one("rtsp_mount_path", config, vehicle_id="lead") == "/lead/stream"
    assert _resolve_one("rtsp_mount_path", config, vehicle_id="wing") == "/wing/stream"
    assert _resolve_one("image_topic", config, vehicle_id="lead") == "/isaac_core/lead/image_rgb"
    assert _resolve_one("image_topic", config, vehicle_id="wing") == "/isaac_core/wing/image_rgb"


# -- explicit config values still win over derivation -----------------------------


def test_explicit_image_topic_wins_over_derivation() -> None:
    config = _make_config(
        vehicles={"drone_0": VehicleConfig(camera=CameraConfig(image_topic="/custom/image"))},
    )
    assert _resolve_one("image_topic", config) == "/custom/image"


def test_explicit_rtsp_mount_path_wins_and_is_slash_prefixed() -> None:
    config = _make_config(
        vehicles={"drone_0": VehicleConfig(camera=CameraConfig(rtsp_mount_path="feed"))},
    )
    # An explicit value without a leading slash is normalised, not namespaced.
    assert _resolve_one("rtsp_mount_path", config) == "/feed"


def test_explicit_udp_port_wins_over_index_allocation() -> None:
    config = _make_config(
        vehicles={"lead": VehicleConfig(), "wing": VehicleConfig(udp_port=44444)},
    )
    assert _resolve_one("udp_port", config, vehicle_id="wing") == 44444


def test_explicit_mavros_topics_win_over_namespace_derivation() -> None:
    config = _make_config(
        vehicles={"drone_0": VehicleConfig(lla_topic="/custom/lla", orientation_topic="/custom/att")},
    )
    assert _resolve_one("lla_topic", config) == "/custom/lla"
    assert _resolve_one("orientation_topic", config) == "/custom/att"


def test_mavros_topics_derive_from_per_vehicle_namespace() -> None:
    # Each aircraft has its own MAVROS namespace; derivation must follow the named vehicle.
    config = _make_config(
        vehicles={
            "lead": VehicleConfig(mavros_namespace="/lead/mavros"),
            "wing": VehicleConfig(mavros_namespace="/wing/mavros"),
        },
    )
    assert _resolve_one("lla_topic", config, vehicle_id="wing") == "/wing/mavros/global_position/global"
    assert _resolve_one("orientation_topic", config, vehicle_id="lead") == "/lead/mavros/local_position/pose"


# -- aperture resolvers scope to the vehicle's own camera --------------------------


def test_aperture_resolvers_scope_to_the_vehicles_own_camera() -> None:
    # Two vehicles with different explicit apertures; each resolves its own rather than always
    # the first vehicle's.
    config = _make_config(
        vehicles={
            "lead": VehicleConfig(camera=CameraConfig(horizontal_aperture_mm=12.0)),
            "wing": VehicleConfig(camera=CameraConfig(horizontal_aperture_mm=24.0)),
        },
    )
    assert _resolve_horizontal_aperture(config, "lead") == pytest.approx(12.0)
    assert _resolve_horizontal_aperture(config, "wing") == pytest.approx(24.0)


def test_compute_writes_defaults_to_first_vehicle_when_identity_omitted() -> None:
    # Backwards compatibility: existing callers pass no vehicle and get the first vehicle's
    # camera, exactly as before the plumbing was added.
    config = _make_config(vehicles={"lead": VehicleConfig(), "wing": VehicleConfig()})
    assert _resolve_one("udp_port", config) == 33333
    assert _resolve_one("global_pose_topic", config) == "/isaac_core/lead/global_pose"
