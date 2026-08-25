"""
Interface tests for the quaternion/Euler bridging nodes.

These two nodes exist purely to connect Isaac's ROS 2 bridge to our math node. The
bridge exposes a ``PoseStamped`` or ``GeoPoseStamped`` orientation as four separate
``double`` attributes, while ``GlobalPositionToLocalPosition`` speaks ``vectord[3]``
of radians. Nothing in the bridge or OmniGraph's stock nodes closes that gap, so
these do.

Because the exact attribute names are what a human wires up in the GUI, and what the
saved ``.usda`` then references by name, they are a contract worth pinning: renaming
one silently breaks every stage file that referenced it.

Parsed as data, so these run with neither Isaac Sim nor ROS 2 present.
"""

import json
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
MATH_NODES = REPO_ROOT / "extensions/isaac_core_ogn.math/isaac_core_ogn/math/nodes"

QUAT_TO_EULER_OGN = MATH_NODES / "OgnQuaternionToEuler.ogn"
EULER_TO_QUAT_OGN = MATH_NODES / "OgnEulerToQuaternion.ogn"

# Component names the ROS 2 bridge uses for a quaternion, which is what these nodes
# have to line up with on the scalar side.
QUATERNION_COMPONENTS = ("qw", "qx", "qy", "qz")


def _spec(path: Path) -> dict[str, object]:
    """Return the single node definition inside an .ogn file."""
    data = json.loads(path.read_text(encoding="utf-8"))
    assert len(data) == 1, f"{path.name} should declare exactly one node"
    return dict(next(iter(data.values())))


def _attrs(path: Path, section: str) -> dict[str, dict[str, object]]:
    """Return a node's inputs or outputs."""
    attrs = _spec(path)[section]
    assert isinstance(attrs, dict)
    return attrs


def test_both_bridging_nodes_exist() -> None:
    assert QUAT_TO_EULER_OGN.is_file()
    assert EULER_TO_QUAT_OGN.is_file()


def test_quaternion_to_euler_node_type_key() -> None:
    assert list(json.loads(QUAT_TO_EULER_OGN.read_text(encoding="utf-8"))) == ["QuaternionToEuler"]


def test_euler_to_quaternion_node_type_key() -> None:
    assert list(json.loads(EULER_TO_QUAT_OGN.read_text(encoding="utf-8"))) == ["EulerToQuaternion"]


@pytest.mark.parametrize("component", QUATERNION_COMPONENTS)
def test_quaternion_to_euler_takes_each_scalar_component(component: str) -> None:
    # The bridge subscriber emits four separate doubles, so we must accept four.
    inputs = _attrs(QUAT_TO_EULER_OGN, "inputs")
    assert component in inputs
    assert inputs[component]["type"] == "double"


def test_quaternion_to_euler_defaults_to_identity() -> None:
    # A subscriber reports all zeros before its first message; qw defaulting to 1
    # means an unconnected node still represents a valid rotation.
    inputs = _attrs(QUAT_TO_EULER_OGN, "inputs")
    assert inputs["qw"]["default"] == 1.0
    for component in ("qx", "qy", "qz"):
        assert inputs[component]["default"] == 0.0


def test_quaternion_to_euler_emits_a_vector_for_the_math_node() -> None:
    # This is the output that wires into GlobalPositionToLocalPosition.
    outputs = _attrs(QUAT_TO_EULER_OGN, "outputs")
    assert outputs["euler_r"]["type"] == "vectord[3]"


@pytest.mark.parametrize("component", ("roll_r", "pitch_r", "yaw_r"))
def test_quaternion_to_euler_also_emits_scalars(component: str) -> None:
    outputs = _attrs(QUAT_TO_EULER_OGN, "outputs")
    assert outputs[component]["type"] == "double"


def test_euler_to_quaternion_takes_the_math_node_vector() -> None:
    inputs = _attrs(EULER_TO_QUAT_OGN, "inputs")
    assert inputs["euler_r"]["type"] == "vectord[3]"


@pytest.mark.parametrize("component", QUATERNION_COMPONENTS)
def test_euler_to_quaternion_emits_each_scalar_component(component: str) -> None:
    # The bridge publisher takes four separate doubles, so we must supply four.
    outputs = _attrs(EULER_TO_QUAT_OGN, "outputs")
    assert component in outputs
    assert outputs[component]["type"] == "double"


def test_euler_to_quaternion_also_emits_a_quatd() -> None:
    outputs = _attrs(EULER_TO_QUAT_OGN, "outputs")
    assert outputs["quaternion"]["type"] == "quatd[4]"


def test_euler_to_quaternion_quatd_default_is_identity_in_ijkr_order() -> None:
    # Isaac stores quatd as [x, y, z, w], so identity is [0, 0, 0, 1] — not
    # [1, 0, 0, 0]. Getting this backwards produces a 180 degree error.
    outputs = _attrs(EULER_TO_QUAT_OGN, "outputs")
    assert outputs["quaternion"]["default"] == [0.0, 0.0, 0.0, 1.0]


@pytest.mark.parametrize("path", (QUAT_TO_EULER_OGN, EULER_TO_QUAT_OGN), ids=("quat_to_euler", "euler_to_quat"))
def test_bridging_nodes_declare_an_exec_input(path: Path) -> None:
    assert _attrs(path, "inputs")["execIn"]["type"] == "execution"


@pytest.mark.parametrize("path", (QUAT_TO_EULER_OGN, EULER_TO_QUAT_OGN), ids=("quat_to_euler", "euler_to_quat"))
def test_bridging_nodes_document_units_in_every_angle_name(path: Path) -> None:
    # The old repo's debugger claimed degrees while sending radians. Every angle
    # attribute here must carry its unit in the name.
    for section in ("inputs", "outputs"):
        for name in _attrs(path, section):
            if any(word in name for word in ("roll", "pitch", "yaw", "euler")):
                assert name.endswith(("_r", "_deg")), f"{name} lacks a unit suffix"
