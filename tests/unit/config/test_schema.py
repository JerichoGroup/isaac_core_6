"""Tests for the configuration schema, its validation and its derived values."""

from pydantic import ValidationError
import pytest

from isaac_core.config import (
    CameraConfig,
    ControlPlaneConfig,
    GimbalConfig,
    IsaacCoreConfig,
    PrimOverride,
    VehicleConfig,
)
from isaac_core.contracts.frames import PoseSource, RotationFrame

# --------------------------------------------------------------------------- #
# defaults: the zero-config case must be a working single-vehicle setup
# --------------------------------------------------------------------------- #


def test_empty_config_yields_one_vehicle_with_one_camera() -> None:
    config = IsaacCoreConfig()
    assert list(config.vehicles) == ["drone_0"]
    assert list(config.vehicles["drone_0"].cameras) == ["eo"]


def test_default_pose_source_is_udp() -> None:
    assert IsaacCoreConfig().vehicles["drone_0"].pose_source is PoseSource.UDP


def test_default_camera_matches_the_previous_generation_intrinsics() -> None:
    camera = CameraConfig()
    assert camera.resolution == (1280, 720)
    assert camera.fov_deg == pytest.approx(78.1)
    assert camera.focal_length_mm == pytest.approx(22.7885)


def test_camera_exposes_width_and_height() -> None:
    camera = CameraConfig(resolution=(3840, 2160))
    assert (camera.width, camera.height) == (3840, 2160)


def test_config_is_frozen_so_runtime_patches_must_go_through_the_control_plane() -> None:
    config = IsaacCoreConfig()
    with pytest.raises(ValidationError):
        config.sim.headless = True


# --------------------------------------------------------------------------- #
# the airframe's rotation frame, which is also what the gimbal offset composes onto
# --------------------------------------------------------------------------- #


def test_vehicle_defaults_to_world_frame_so_heading_holds_when_pitched_down() -> None:
    assert VehicleConfig().rotation_frame is RotationFrame.WORLD


def test_rotation_frame_accepts_a_plain_string_from_toml() -> None:
    assert VehicleConfig(rotation_frame="body").rotation_frame is RotationFrame.BODY


def test_the_gimbal_has_no_rotation_frame_of_its_own() -> None:
    # It used to offer one, defaulting to body, that nothing read: only the vehicle's frame reaches
    # the pose node, and that frame is what the gimbal offset composes onto. A key that validates
    # and is documented while doing nothing is worse than no key.
    assert "rotation_frame" not in GimbalConfig.model_fields


def test_gimbal_rate_limit_is_optional_but_must_be_positive() -> None:
    assert GimbalConfig().max_rate_deg_s is None
    assert GimbalConfig(max_rate_deg_s=60.0).max_rate_deg_s == pytest.approx(60.0)
    with pytest.raises(ValidationError):
        GimbalConfig(max_rate_deg_s=0.0)


# --------------------------------------------------------------------------- #
# typos must fail loudly, not be ignored
# --------------------------------------------------------------------------- #


def test_unknown_key_is_rejected_rather_than_silently_ignored() -> None:
    with pytest.raises(ValidationError, match="Extra inputs are not permitted"):
        CameraConfig(fov_degrees=90.0)


def test_misspelled_pose_source_is_rejected() -> None:
    with pytest.raises(ValidationError):
        VehicleConfig(pose_source="upd")


@pytest.mark.parametrize("fov", [0.0, -10.0, 181.0])
def test_impossible_field_of_view_is_rejected(fov: float) -> None:
    with pytest.raises(ValidationError):
        CameraConfig(fov_deg=fov)


@pytest.mark.parametrize("resolution", [(0, 720), (1280, 0), (-1, -1)])
def test_non_positive_resolution_is_rejected(resolution: tuple[int, int]) -> None:
    with pytest.raises(ValidationError):
        CameraConfig(resolution=resolution)


def test_vehicle_without_a_camera_is_rejected() -> None:
    with pytest.raises(ValidationError, match="at least one camera"):
        VehicleConfig(cameras={})


def test_config_without_a_vehicle_is_rejected() -> None:
    with pytest.raises(ValidationError, match="at least one vehicle"):
        IsaacCoreConfig(vehicles={})


def test_vehicle_id_must_be_a_legal_topic_segment() -> None:
    # Vehicle ids become topic namespace segments, so they cannot contain dashes.
    with pytest.raises(ValidationError, match="invalid topic segment"):
        IsaacCoreConfig(vehicles={"my-drone": VehicleConfig()})


# --------------------------------------------------------------------------- #
# control plane safety
# --------------------------------------------------------------------------- #


def test_control_plane_defaults_to_loopback() -> None:
    assert ControlPlaneConfig().host == "127.0.0.1"


@pytest.mark.parametrize("host", ["127.0.0.1", "localhost", "::1"])
def test_loopback_control_plane_needs_no_token(host: str) -> None:
    assert ControlPlaneConfig(host=host).token == ""


def test_non_loopback_control_plane_without_a_token_is_rejected() -> None:
    # This channel patches a running sim and writes files; it must not be open.
    with pytest.raises(ValidationError, match="token must be set"):
        ControlPlaneConfig(host="0.0.0.0")


def test_non_loopback_control_plane_with_a_token_is_accepted() -> None:
    assert ControlPlaneConfig(host="10.0.0.5", token="s3cret").host == "10.0.0.5"


# --------------------------------------------------------------------------- #
# derived: udp ports
# --------------------------------------------------------------------------- #


def test_single_vehicle_gets_the_familiar_default_port() -> None:
    assert IsaacCoreConfig().resolved_udp_port("drone_0") == 33333


def test_swarm_ports_follow_declaration_order() -> None:
    config = IsaacCoreConfig(
        vehicles={
            "lead": VehicleConfig(),
            "wing_1": VehicleConfig(),
            "wing_2": VehicleConfig(),
        }
    )
    assert [config.resolved_udp_port(v) for v in config.vehicles] == [
        33333,
        33334,
        33335,
    ]


def test_explicit_udp_port_overrides_the_derived_one() -> None:
    config = IsaacCoreConfig(vehicles={"lead": VehicleConfig(), "wing_1": VehicleConfig(udp_port=40000)})
    assert config.resolved_udp_port("wing_1") == 40000


def test_unknown_vehicle_index_raises_key_error() -> None:
    with pytest.raises(KeyError, match="unknown vehicle"):
        IsaacCoreConfig().vehicle_index("nope")


# --------------------------------------------------------------------------- #
# derived: mount points
# --------------------------------------------------------------------------- #


def test_mount_defaults_to_environment_plus_vehicle_id() -> None:
    # Under /World: the authored scenes use /World/Environment, per USD convention of a
    # single default prim. Mounting at /Environment would put layers outside /World.
    assert IsaacCoreConfig().resolved_mount("drone_0") == "/World/Environment/drone_0"


def test_explicit_mount_overrides_the_derived_one() -> None:
    config = IsaacCoreConfig(vehicles={"lead": VehicleConfig(mount="/World/lead")})
    assert config.resolved_mount("lead") == "/World/lead"


def test_malformed_explicit_mount_is_rejected() -> None:
    config = IsaacCoreConfig(vehicles={"lead": VehicleConfig(mount="relative/path")})
    with pytest.raises(ValueError, match="invalid prim path"):
        config.resolved_mount("lead")


# --------------------------------------------------------------------------- #
# derived: topic namespacing collapses for the common case
# --------------------------------------------------------------------------- #


def test_one_vehicle_one_camera_gives_flat_topics() -> None:
    config = IsaacCoreConfig()
    resolver = config.topic_resolver("drone_0", "eo")
    assert resolver.resolve("image_rgb") == "/isaac_core/image_rgb"


def test_one_vehicle_two_cameras_namespaces_by_camera_only() -> None:
    config = IsaacCoreConfig(vehicles={"drone_0": VehicleConfig(cameras={"eo": CameraConfig(), "ir": CameraConfig()})})
    assert config.topic_resolver("drone_0", "ir").resolve("image_rgb") == "/isaac_core/ir/image_rgb"


def test_swarm_namespaces_by_vehicle_and_camera() -> None:
    config = IsaacCoreConfig(
        vehicles={
            "lead": VehicleConfig(cameras={"eo": CameraConfig(), "ir": CameraConfig()}),
            "wing_1": VehicleConfig(),
        }
    )
    assert config.topic_resolver("lead", "eo").resolve("image_rgb") == "/isaac_core/lead/eo/image_rgb"


def test_swarm_with_one_camera_each_still_namespaces_by_vehicle() -> None:
    config = IsaacCoreConfig(vehicles={"lead": VehicleConfig(), "wing_1": VehicleConfig()})
    assert config.topic_resolver("wing_1", "eo").resolve("image_rgb") == "/isaac_core/wing_1/image_rgb"


def test_vehicle_scoped_topics_ignore_the_camera_level() -> None:
    config = IsaacCoreConfig(vehicles={"lead": VehicleConfig(), "wing_1": VehicleConfig()})
    resolver = config.topic_resolver("lead", "eo")
    assert resolver.vehicle_scoped("global_pose") == "/isaac_core/lead/global_pose"


def test_camera_keys_are_dotted_and_ordered() -> None:
    config = IsaacCoreConfig(
        vehicles={
            "lead": VehicleConfig(cameras={"eo": CameraConfig(), "ir": CameraConfig()}),
            "wing_1": VehicleConfig(),
        }
    )
    assert config.camera_keys() == ("lead.eo", "lead.ir", "wing_1.eo")


def test_is_single_vehicle_reflects_the_vehicle_count() -> None:
    assert IsaacCoreConfig().is_single_vehicle
    config = IsaacCoreConfig(vehicles={"a": VehicleConfig(), "b": VehicleConfig()})
    assert not config.is_single_vehicle


# --------------------------------------------------------------------------- #
# layers own their own namespace (D13)
# --------------------------------------------------------------------------- #


def test_layer_settings_are_accepted_without_a_core_schema_change() -> None:
    # A third-party layer must be able to add settings with no change to isaac_core.
    config = IsaacCoreConfig(layers={"thermal_cam": {"emissivity": 0.95, "palette": "ironbow"}})
    assert config.layers["thermal_cam"]["emissivity"] == pytest.approx(0.95)


def test_layers_default_to_empty() -> None:
    assert IsaacCoreConfig().layers == {}


# --------------------------------------------------------------------------- #
# prim overrides: the escape hatch
# --------------------------------------------------------------------------- #


def test_prim_override_accepts_a_well_formed_path() -> None:
    override = PrimOverride(prim="/Environment/drone_0/Graph/node", attribute="inputs:enabled", value=False)
    assert override.value is False


def test_prim_override_rejects_a_malformed_path_at_load_time() -> None:
    with pytest.raises(ValidationError, match="invalid prim path"):
        PrimOverride(prim="not/absolute", attribute="inputs:enabled", value=1)


# --------------------------------------------------------------------------- #
# derived feature list
# --------------------------------------------------------------------------- #


def test_a_udp_vehicle_implies_the_udp_camera_layer() -> None:
    # A minimal config must compose and fly with nothing listed under [features]:
    # listing the layer AND setting pose_source is a duplicate source of truth.
    assert IsaacCoreConfig().required_feature_ids() == ("camera_udp",)


def test_a_ros_vehicle_implies_the_ros_camera_layer() -> None:
    config = IsaacCoreConfig(vehicles={"lead": VehicleConfig(pose_source="ros")})
    assert config.required_feature_ids() == ("camera_ros",)


def test_a_mixed_swarm_implies_both_camera_layers() -> None:
    config = IsaacCoreConfig(vehicles={"lead": VehicleConfig(pose_source="ros"), "wing_1": VehicleConfig()})
    assert set(config.required_feature_ids()) == {"camera_ros", "camera_udp"}


def test_explicit_features_come_first_and_derived_are_appended() -> None:
    config = IsaacCoreConfig(features={"enabled": ["distance_sensor"]})
    assert config.required_feature_ids() == ("distance_sensor", "camera_udp")


def test_derived_layers_are_not_duplicated_when_also_listed_explicitly() -> None:
    config = IsaacCoreConfig(features={"enabled": ["camera_udp"]})
    assert config.required_feature_ids() == ("camera_udp",)


def test_every_pose_source_implies_a_shipped_layer() -> None:
    # PoseSource used to carry script/replay/mavlink, which nothing dispatched on: a vehicle
    # configured that way composed and then never moved, with no error at all. The enum now holds
    # only wired sources, so this asserts the invariant instead of cataloguing the exceptions.
    for source in PoseSource:
        config = IsaacCoreConfig(vehicles={"v": VehicleConfig(pose_source=source)})
        assert config.required_feature_ids(), f"{source.value} implies no layer"
