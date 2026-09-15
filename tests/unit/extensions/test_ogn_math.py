"""Tests for the isaac_core_ogn.math extension parsed as data (no Isaac Sim)."""

import json
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib

# Path to the math extension root.
_EXT_ROOT = Path(__file__).resolve().parents[3] / "extensions" / "isaac_core_ogn.math"
_NODES_DIR = _EXT_ROOT / "isaac_core_ogn" / "math" / "nodes"
_OGN_PATH = _NODES_DIR / "OgnGlobalPositionToLocalPosition.ogn"
_EXT_TOML = _EXT_ROOT / "config" / "extension.toml"

# Valid OGN attribute types as confirmed in the real Isaac Sim 6 install (Kit 110.1.2).
# Derived by scanning all .ogn files under the Isaac Sim install's exts directory.
_VALID_OGN_TYPES = frozenset(
    {
        "bool",
        "bool[]",
        "colorf[4]",
        "double",
        "double[]",
        "double[2]",
        "double[3]",
        "double[3][]",
        "double[4][]",
        "execution",
        "float",
        "float[]",
        "float[2]",
        "float[3]",
        "int",
        "int[]",
        "matrixd[4]",
        "path",
        "pointf[3][]",
        "quatd[4]",
        "string",
        "target",
        "token",
        "token[]",
        "uchar",
        "uchar[]",
        "uint",
        "uint[]",
        "uint64",
        "vectord[3]",
    }
)


# ─── .ogn structure tests ─────────────────────────────────────────────────────


def test_ogn_is_valid_json() -> None:
    text = _OGN_PATH.read_text(encoding="utf-8")
    json.loads(text)


def test_ogn_has_single_key_matching_filename() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    keys = list(data.keys())
    assert keys == ["GlobalPositionToLocalPosition"]


def test_ogn_node_type_string() -> None:
    with _EXT_TOML.open("rb") as f:
        toml = tomllib.load(f)
    modules = toml["python"]["module"]
    ext_name = modules[0]["name"]
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    node_key = list(data.keys())[0]
    full_type = f"{ext_name}.{node_key}"
    assert full_type == "isaac_core_ogn.math.GlobalPositionToLocalPosition"


def test_ogn_declares_exec_in_input() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    inputs = data["GlobalPositionToLocalPosition"]["inputs"]
    assert "execIn" in inputs
    assert inputs["execIn"]["type"] == "execution"


def test_ogn_declares_enu_reference_input() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    inputs = data["GlobalPositionToLocalPosition"]["inputs"]
    assert "enu_reference" in inputs
    assert inputs["enu_reference"]["type"] == "vectord[3]"
    assert inputs["enu_reference"]["default"] == [32.22481, 35.25621, 516.7]


def test_ogn_declares_global_position_input() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    inputs = data["GlobalPositionToLocalPosition"]["inputs"]
    assert "global_position" in inputs
    assert inputs["global_position"]["type"] == "vectord[3]"


def test_ogn_declares_global_orientation_input() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    inputs = data["GlobalPositionToLocalPosition"]["inputs"]
    assert "global_orientation" in inputs
    assert inputs["global_orientation"]["type"] == "vectord[3]"


def test_ogn_declares_offset_roll_deg_input() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    inputs = data["GlobalPositionToLocalPosition"]["inputs"]
    assert "offset_roll_deg" in inputs
    assert inputs["offset_roll_deg"]["type"] == "float"
    assert inputs["offset_roll_deg"]["default"] == 0.0


def test_ogn_declares_offset_pitch_deg_input() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    inputs = data["GlobalPositionToLocalPosition"]["inputs"]
    assert "offset_pitch_deg" in inputs
    assert inputs["offset_pitch_deg"]["type"] == "float"
    assert inputs["offset_pitch_deg"]["default"] == 0.0


def test_ogn_declares_offset_yaw_deg_input() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    inputs = data["GlobalPositionToLocalPosition"]["inputs"]
    assert "offset_yaw_deg" in inputs
    assert inputs["offset_yaw_deg"]["type"] == "float"
    assert inputs["offset_yaw_deg"]["default"] == 0.0


def test_ogn_declares_rotation_frame_input_with_body_default() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    inputs = data["GlobalPositionToLocalPosition"]["inputs"]
    assert "rotation_frame" in inputs
    assert inputs["rotation_frame"]["type"] == "string"
    assert inputs["rotation_frame"]["default"] == "body"


def test_ogn_declares_global_position_output() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    outputs = data["GlobalPositionToLocalPosition"]["outputs"]
    assert "global_position" in outputs
    assert outputs["global_position"]["type"] == "vectord[3]"


def test_ogn_declares_global_orientation_output() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    outputs = data["GlobalPositionToLocalPosition"]["outputs"]
    assert "global_orientation" in outputs
    assert outputs["global_orientation"]["type"] == "vectord[3]"


def test_ogn_declares_local_position_output() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    outputs = data["GlobalPositionToLocalPosition"]["outputs"]
    assert "local_position" in outputs
    assert outputs["local_position"]["type"] == "vectord[3]"


def test_ogn_declares_local_orientation_output_as_quatd() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    outputs = data["GlobalPositionToLocalPosition"]["outputs"]
    assert "local_orientation" in outputs
    assert outputs["local_orientation"]["type"] == "quatd[4]"
    # IJKR default: identity quaternion [x=0, y=0, z=0, w=1]
    assert outputs["local_orientation"]["default"] == [0.0, 0.0, 0.0, 1.0]


def test_ogn_all_attributes_have_descriptions() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    node_def = data["GlobalPositionToLocalPosition"]
    for section_name in ("inputs", "outputs"):
        section = node_def.get(section_name, {})
        for port_name, port_def in section.items():
            assert "description" in port_def, f"{section_name}.{port_name} missing 'description'"
            assert port_def["description"], f"{section_name}.{port_name} has empty description"


def test_ogn_all_attribute_types_are_valid_kit_110() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    node_def = data["GlobalPositionToLocalPosition"]
    for section_name in ("inputs", "outputs"):
        section = node_def.get(section_name, {})
        for port_name, port_def in section.items():
            attr_type = port_def["type"]
            assert attr_type in _VALID_OGN_TYPES, (
                f"{section_name}.{port_name} uses type {attr_type!r} which was not found "
                f"in any .ogn file in the real Isaac Sim 6 install"
            )


def test_ogn_key_does_not_contain_ogn_or_sim() -> None:
    data = json.loads(_OGN_PATH.read_text(encoding="utf-8"))
    key = list(data.keys())[0]
    assert "Ogn" not in key
    assert "Sim" not in key


# ─── extension.toml tests ─────────────────────────────────────────────────────


def test_extension_toml_is_valid() -> None:
    with _EXT_TOML.open("rb") as f:
        tomllib.load(f)


def test_extension_toml_declares_omni_graph_dependency() -> None:
    with _EXT_TOML.open("rb") as f:
        toml = tomllib.load(f)
    deps = toml.get("dependencies", {})
    assert "omni.graph" in deps


def test_extension_toml_has_no_omni_isaac_dependencies() -> None:
    with _EXT_TOML.open("rb") as f:
        toml = tomllib.load(f)
    deps = toml.get("dependencies", {})
    for dep_name in deps:
        assert not dep_name.startswith(
            "omni.isaac."
        ), f"Dependency {dep_name!r} uses old omni.isaac.* namespace which was renamed in Isaac Sim 4.5+"


def test_extension_toml_python_module_matches_directory() -> None:
    with _EXT_TOML.open("rb") as f:
        toml = tomllib.load(f)
    modules = toml["python"]["module"]
    assert modules[0]["name"] == "isaac_core_ogn.math"
    module_path = _EXT_ROOT / "isaac_core_ogn" / "math"
    assert module_path.is_dir()


def test_extension_toml_ogn_path_exists() -> None:
    with _EXT_TOML.open("rb") as f:
        toml = tomllib.load(f)
    ogn_section = toml.get("ogn", {})
    ogn_path_str = ogn_section.get("path")
    assert ogn_path_str
    ogn_dir = _EXT_ROOT / ogn_path_str
    assert ogn_dir.is_dir()
