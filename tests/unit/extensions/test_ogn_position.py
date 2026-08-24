"""Tests for the isaac_core_ogn.position extension parsed as data (no Isaac Sim)."""

import json
from pathlib import Path

import pytest

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib

from isaac_core.contracts.frames import Frame
from isaac_core.contracts.packet import PACKET_SIZE
from isaac_core.contracts.ports import DEFAULT_POSE_UDP_PORT
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.protocol import HoldLastGoodDecoder, encode

# Path to the position extension root.
_EXT_ROOT = Path(__file__).resolve().parents[3] / "extensions" / "isaac_core_ogn.position"
_NODES_DIR = _EXT_ROOT / "isaac_core_ogn" / "position" / "nodes"
_UDP_OGN = _NODES_DIR / "OgnUdpToGlobalPosition.ogn"
_ROS2_OGN = _NODES_DIR / "OgnRos2ToGlobalPosition.ogn"
_EXT_TOML = _EXT_ROOT / "config" / "extension.toml"


# ─── .ogn structure tests (parse as data, no imports) ──────────────────────────


def test_udp_ogn_is_valid_json() -> None:
    json.loads(_UDP_OGN.read_text(encoding="utf-8"))


def test_ros2_ogn_is_valid_json() -> None:
    json.loads(_ROS2_OGN.read_text(encoding="utf-8"))


def test_udp_ogn_key_matches_filename() -> None:
    data = json.loads(_UDP_OGN.read_text(encoding="utf-8"))
    keys = list(data.keys())
    assert keys == ["UdpToGlobalPosition"]


def test_ros2_ogn_key_matches_filename() -> None:
    data = json.loads(_ROS2_OGN.read_text(encoding="utf-8"))
    keys = list(data.keys())
    assert keys == ["Ros2ToGlobalPosition"]


def test_udp_ogn_node_type_string() -> None:
    with _EXT_TOML.open("rb") as f:
        toml = tomllib.load(f)
    modules = toml["python"]["module"]
    ext_name = modules[0]["name"]
    data = json.loads(_UDP_OGN.read_text(encoding="utf-8"))
    node_key = list(data.keys())[0]
    full_type = f"{ext_name}.{node_key}"
    assert full_type == "isaac_core_ogn.position.UdpToGlobalPosition"


def test_ros2_ogn_node_type_string() -> None:
    with _EXT_TOML.open("rb") as f:
        toml = tomllib.load(f)
    modules = toml["python"]["module"]
    ext_name = modules[0]["name"]
    data = json.loads(_ROS2_OGN.read_text(encoding="utf-8"))
    node_key = list(data.keys())[0]
    full_type = f"{ext_name}.{node_key}"
    assert full_type == "isaac_core_ogn.position.Ros2ToGlobalPosition"


def test_udp_ogn_has_udp_port_input_with_correct_default() -> None:
    data = json.loads(_UDP_OGN.read_text(encoding="utf-8"))
    node = data["UdpToGlobalPosition"]
    port_def = node["inputs"]["udp_port"]
    assert port_def["type"] == "int"
    assert port_def["default"] == DEFAULT_POSE_UDP_PORT


def test_udp_ogn_declares_global_position_output() -> None:
    data = json.loads(_UDP_OGN.read_text(encoding="utf-8"))
    outputs = data["UdpToGlobalPosition"]["outputs"]
    assert "global_position" in outputs
    assert outputs["global_position"]["type"] == "vectord[3]"


def test_udp_ogn_declares_global_orientation_output() -> None:
    data = json.loads(_UDP_OGN.read_text(encoding="utf-8"))
    outputs = data["UdpToGlobalPosition"]["outputs"]
    assert "global_orientation" in outputs
    assert outputs["global_orientation"]["type"] == "vectord[3]"


def test_udp_ogn_declares_decode_failure_count_output() -> None:
    data = json.loads(_UDP_OGN.read_text(encoding="utf-8"))
    outputs = data["UdpToGlobalPosition"]["outputs"]
    assert "decode_failure_count" in outputs
    assert outputs["decode_failure_count"]["type"] == "int"


def test_ros2_ogn_declares_lla_topic_input() -> None:
    data = json.loads(_ROS2_OGN.read_text(encoding="utf-8"))
    inputs = data["Ros2ToGlobalPosition"]["inputs"]
    assert "lla_topic" in inputs
    assert inputs["lla_topic"]["type"] == "string"
    assert inputs["lla_topic"]["default"] == "/mavros/global_position/global"


def test_ros2_ogn_declares_orientation_topic_input() -> None:
    data = json.loads(_ROS2_OGN.read_text(encoding="utf-8"))
    inputs = data["Ros2ToGlobalPosition"]["inputs"]
    assert "orientation_topic" in inputs
    assert inputs["orientation_topic"]["type"] == "string"
    assert inputs["orientation_topic"]["default"] == "/mavros/local_position/pose"


def test_ros2_ogn_declares_global_position_output() -> None:
    data = json.loads(_ROS2_OGN.read_text(encoding="utf-8"))
    outputs = data["Ros2ToGlobalPosition"]["outputs"]
    assert "global_position" in outputs
    assert outputs["global_position"]["type"] == "vectord[3]"


def test_ros2_ogn_declares_global_orientation_output() -> None:
    data = json.loads(_ROS2_OGN.read_text(encoding="utf-8"))
    outputs = data["Ros2ToGlobalPosition"]["outputs"]
    assert "global_orientation" in outputs
    assert outputs["global_orientation"]["type"] == "vectord[3]"


@pytest.mark.parametrize(
    "ogn_path",
    [_UDP_OGN, _ROS2_OGN],
    ids=["udp", "ros2"],
)
def test_ogn_all_attributes_have_descriptions(ogn_path: Path) -> None:
    data = json.loads(ogn_path.read_text(encoding="utf-8"))
    node_key = list(data.keys())[0]
    node_def = data[node_key]
    for section_name in ("inputs", "outputs"):
        section = node_def.get(section_name, {})
        for port_name, port_def in section.items():
            assert "description" in port_def, f"{ogn_path.name}:{node_key}.{section_name}.{port_name} no description"
            assert port_def["description"], f"{ogn_path.name}:{node_key}.{section_name}.{port_name} empty description"


# ─── extension.toml tests ─────────────────────────────────────────────────────


def test_extension_toml_is_valid() -> None:
    with _EXT_TOML.open("rb") as f:
        tomllib.load(f)


def test_extension_toml_declares_omni_graph_dependency() -> None:
    with _EXT_TOML.open("rb") as f:
        toml = tomllib.load(f)
    deps = toml.get("dependencies", {})
    assert "omni.graph" in deps


def test_extension_toml_declares_ros2_bridge_dependency() -> None:
    with _EXT_TOML.open("rb") as f:
        toml = tomllib.load(f)
    deps = toml.get("dependencies", {})
    assert "isaacsim.ros2.bridge" in deps


# ─── Behavioural tests of shared kernel logic (HoldLastGoodDecoder) ───────────


def _make_valid_pose(lat: float = 32.0, lon: float = 35.0, alt: float = 100.0) -> GeodeticPose:
    return GeodeticPose(
        position=Lla(lat_deg=lat, lon_deg=lon, alt_m=alt),
        orientation=Rpy(roll_r=0.1, pitch_r=0.2, yaw_r=0.3, frame=Frame.NED),
    )


def test_hold_last_good_returns_none_before_first_valid_packet() -> None:
    decoder = HoldLastGoodDecoder()
    result = decoder.decode(b"\x00" * PACKET_SIZE)
    assert result is None
    assert decoder.failure_count == 1


def test_hold_last_good_returns_valid_pose_on_good_packet() -> None:
    decoder = HoldLastGoodDecoder()
    pose = _make_valid_pose()
    packet = encode(pose)
    result = decoder.decode(packet)
    assert result is not None
    assert result.position.lat_deg == pytest.approx(32.0)
    assert decoder.failure_count == 0


def test_hold_last_good_holds_pose_on_corrupt_packet() -> None:
    decoder = HoldLastGoodDecoder()
    good_pose = _make_valid_pose(lat=32.5, lon=35.5, alt=200.0)
    good_packet = encode(good_pose)

    # Feed a good packet first.
    result = decoder.decode(good_packet)
    assert result is not None
    assert result.position.lat_deg == pytest.approx(32.5)

    # Now feed a corrupt packet (wrong header).
    corrupt = b"\xff\xff" + good_packet[2:]
    result = decoder.decode(corrupt)

    # Must hold the previous good pose, NOT snap to zero.
    assert result is not None
    assert result.position.lat_deg == pytest.approx(32.5)
    assert result.position.lon_deg == pytest.approx(35.5)
    assert decoder.failure_count == 1


def test_hold_last_good_holds_through_multiple_corruptions() -> None:
    decoder = HoldLastGoodDecoder()
    good_pose = _make_valid_pose(lat=31.0, lon=34.0, alt=50.0)
    good_packet = encode(good_pose)

    decoder.decode(good_packet)

    # Several different kinds of corruption.
    decoder.decode(b"short")
    decoder.decode(b"\xac\xdc" + b"\x00" * 48 + b"\xff")
    decoder.decode(b"\x00" * PACKET_SIZE)

    assert decoder.failure_count == 3
    assert decoder.last_good is not None
    assert decoder.last_good.position.lat_deg == pytest.approx(31.0)


def test_hold_last_good_recovers_after_corruption() -> None:
    decoder = HoldLastGoodDecoder()
    pose_a = _make_valid_pose(lat=32.0, lon=35.0, alt=100.0)
    pose_b = _make_valid_pose(lat=33.0, lon=36.0, alt=200.0)

    decoder.decode(encode(pose_a))
    decoder.decode(b"\x00" * PACKET_SIZE)
    result = decoder.decode(encode(pose_b))

    assert result is not None
    assert result.position.lat_deg == pytest.approx(33.0)
    assert decoder.failure_count == 1
