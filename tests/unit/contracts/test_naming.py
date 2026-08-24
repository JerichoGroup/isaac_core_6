"""Tests for topic namespacing and prim path templating."""

import pytest

from isaac_core.contracts import prims, topics

# --------------------------------------------------------------------------- #
# topic joining
# --------------------------------------------------------------------------- #


def test_join_omits_none_segments_so_namespace_levels_collapse() -> None:
    assert topics.join("/isaac_core", None, None, "image_rgb") == "/isaac_core/image_rgb"


def test_join_includes_present_segments_in_order() -> None:
    assert topics.join("/isaac_core", "lead", "eo", "image_rgb") == "/isaac_core/lead/eo/image_rgb"


def test_join_returns_root_alone_when_no_segments_survive() -> None:
    assert topics.join("/isaac_core", None) == "/isaac_core"


def test_join_rejects_a_relative_root() -> None:
    with pytest.raises(ValueError, match="must be absolute"):
        topics.join("isaac_core", "image_rgb")


def test_join_rejects_a_segment_containing_a_slash() -> None:
    with pytest.raises(ValueError, match="invalid topic segment"):
        topics.join("/isaac_core", "lead/eo")


@pytest.mark.parametrize("segment", ["eo", "drone_0", "_private", "cam2"])
def test_validate_segment_accepts_legal_ros_tokens(segment: str) -> None:
    assert topics.validate_segment(segment) == segment


@pytest.mark.parametrize("segment", ["", "2cam", "with-dash", "with space", "a/b"])
def test_validate_segment_rejects_illegal_ros_tokens(segment: str) -> None:
    with pytest.raises(ValueError, match="invalid topic segment"):
        topics.validate_segment(segment)


# --------------------------------------------------------------------------- #
# TopicResolver
# --------------------------------------------------------------------------- #


def test_single_vehicle_single_camera_produces_flat_topic_names() -> None:
    # This is the shape the team already knows, and the common case.
    resolver = topics.TopicResolver()
    assert resolver.resolve(topics.IMAGE_RGB) == "/isaac_core/image_rgb"


def test_two_cameras_on_one_vehicle_namespace_by_camera_only() -> None:
    resolver = topics.TopicResolver(camera="eo")
    assert resolver.resolve(topics.IMAGE_RGB) == "/isaac_core/eo/image_rgb"


def test_swarm_namespaces_by_vehicle_and_camera() -> None:
    resolver = topics.TopicResolver(vehicle="lead", camera="eo")
    assert resolver.resolve(topics.IMAGE_RGB) == "/isaac_core/lead/eo/image_rgb"


def test_vehicle_scoped_data_does_not_sit_under_a_camera() -> None:
    # Pose and range belong to the aircraft, not to one of its cameras.
    resolver = topics.TopicResolver(vehicle="lead", camera="eo")
    assert resolver.vehicle_scoped(topics.GLOBAL_POSE) == "/isaac_core/lead/global_pose"


def test_for_camera_returns_a_scoped_copy_without_mutating() -> None:
    base = topics.TopicResolver(vehicle="lead")
    scoped = base.for_camera("ir")
    assert scoped.camera == "ir"
    assert base.camera is None


def test_topic_root_is_configurable_for_parallel_simulators() -> None:
    resolver = topics.TopicResolver(root="/sim_b")
    assert resolver.resolve(topics.BBOX) == "/sim_b/bbox"


# --------------------------------------------------------------------------- #
# prim path validation
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "path",
    ["/Environment", "/Environment/drone_0", "/tilesets", "/a/b/c/d"],
)
def test_validate_prim_path_accepts_well_formed_absolute_paths(path: str) -> None:
    assert prims.validate_prim_path(path) == path


@pytest.mark.parametrize(
    "path",
    ["", "/", "Environment", "/Environment/", "/Environment//x", "/2bad", "/a b"],
)
def test_validate_prim_path_rejects_malformed_paths(path: str) -> None:
    assert not prims.is_valid_prim_path(path)
    with pytest.raises(ValueError, match="invalid prim path"):
        prims.validate_prim_path(path)


def test_well_known_prim_roots_are_valid_paths() -> None:
    for root in (prims.ENVIRONMENT_ROOT, prims.TILESETS_ROOT, prims.BBOXES_ROOT):
        assert prims.validate_prim_path(root) == root


# --------------------------------------------------------------------------- #
# prim path templating
# --------------------------------------------------------------------------- #


def test_placeholders_finds_declared_fields() -> None:
    assert prims.placeholders("{mount}/Graph/{instance}") == {"mount", "instance"}


def test_placeholders_returns_empty_for_a_literal_path() -> None:
    assert prims.placeholders("/Environment/drone_0") == frozenset()


def test_render_substitutes_a_mount_point() -> None:
    assert (
        prims.render(
            "{mount}/ActionGraph/ros2_distance_publisher",
            mount="/Environment/drone_0",
        )
        == "/Environment/drone_0/ActionGraph/ros2_distance_publisher"
    )


def test_render_substitutes_an_instance_id() -> None:
    assert prims.render("/Environment/{instance}/thermal", instance="wing_1") == "/Environment/wing_1/thermal"


def test_render_collapses_duplicate_separators_from_substitution() -> None:
    # A mount ending in '/' is an easy manifest slip; absorb it rather than fail.
    assert prims.render("{mount}/Graph", mount="/Environment/eo/") == ("/Environment/eo/Graph")


def test_render_reports_missing_placeholder_values_by_name() -> None:
    with pytest.raises(ValueError, match="missing values for: instance"):
        prims.render("{mount}/{instance}", mount="/Environment")


def test_render_rejects_a_substitution_producing_a_malformed_path() -> None:
    with pytest.raises(ValueError, match="invalid prim path"):
        prims.render("{mount}/Graph", mount="relative/path")


def test_render_passes_a_literal_template_through_validated() -> None:
    assert prims.render("/Environment/drone_0") == "/Environment/drone_0"


# --------------------------------------------------------------------------- #
# child paths
# --------------------------------------------------------------------------- #


def test_child_appends_elements() -> None:
    assert prims.child("/Environment", "drone_0", "eo") == "/Environment/drone_0/eo"


def test_child_with_no_elements_returns_the_parent() -> None:
    assert prims.child("/Environment") == "/Environment"


def test_child_rejects_a_malformed_parent() -> None:
    with pytest.raises(ValueError, match="invalid prim path"):
        prims.child("Environment", "drone_0")
