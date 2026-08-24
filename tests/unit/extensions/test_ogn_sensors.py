"""
Tests for the isaac_core_ogn.sensors extension.

Parses .ogn and node source files as data — no Isaac Sim, no OmniGraph runtime.
Verifies structural contracts, topic defaults, attribute completeness, and guards
against regressions of documented defects from the old repo.
"""

import json
from pathlib import Path

import pytest

from isaac_core.contracts import topics

# Path to the sensors extension nodes directory.
_NODES_DIR = (
    Path(__file__).resolve().parents[3]
    / "extensions"
    / "isaac_core_ogn.sensors"
    / "isaac_core_ogn"
    / "sensors"
    / "nodes"
)

# All .ogn files in this extension.
_OGN_FILES = sorted(_NODES_DIR.glob("*.ogn"))

# All .py node modules in this extension.
_PY_FILES = sorted(_NODES_DIR.glob("Ogn*.py"))

# Extension namespace for node type composition.
_EXT_NAMESPACE = "isaac_core_ogn.sensors"


# ─── Helper ───────────────────────────────────────────────────────────────────


def _load_ogn(path: Path) -> tuple[str, dict]:
    """Return (node_key, node_definition) from an .ogn file."""
    data = json.loads(path.read_text(encoding="utf-8"))
    key = next(iter(data))
    return key, data[key]


# ─── .ogn structural tests ───────────────────────────────────────────────────


@pytest.mark.parametrize("ogn_path", _OGN_FILES, ids=[p.name for p in _OGN_FILES])
def test_ogn_declares_inputs_with_type_and_description(ogn_path: Path) -> None:
    _, node_def = _load_ogn(ogn_path)
    for section_name in ("inputs", "outputs"):
        section = node_def.get(section_name, {})
        for port_name, port_def in section.items():
            assert "type" in port_def, f"{ogn_path.name}.{section_name}.{port_name} missing 'type'"
            assert "description" in port_def, f"{ogn_path.name}.{section_name}.{port_name} missing 'description'"


@pytest.mark.parametrize("ogn_path", _OGN_FILES, ids=[p.name for p in _OGN_FILES])
def test_ogn_all_descriptions_nonempty(ogn_path: Path) -> None:
    _, node_def = _load_ogn(ogn_path)
    assert node_def.get("description"), f"{ogn_path.name} top-level description is empty"
    for section_name in ("inputs", "outputs"):
        section = node_def.get(section_name, {})
        for port_name, port_def in section.items():
            desc = port_def.get("description", "")
            assert desc.strip(), f"{ogn_path.name}.{section_name}.{port_name} has empty description"


@pytest.mark.parametrize("ogn_path", _OGN_FILES, ids=[p.name for p in _OGN_FILES])
def test_ogn_has_exec_input(ogn_path: Path) -> None:
    _, node_def = _load_ogn(ogn_path)
    inputs = node_def.get("inputs", {})
    assert "execIn" in inputs, f"{ogn_path.name} missing 'execIn' execution trigger"
    assert inputs["execIn"]["type"] == "execution"


@pytest.mark.parametrize("ogn_path", _OGN_FILES, ids=[p.name for p in _OGN_FILES])
def test_ogn_node_type_composes_correctly(ogn_path: Path) -> None:
    key, _ = _load_ogn(ogn_path)
    expected_type = f"{_EXT_NAMESPACE}.{key}"
    assert "Ogn" not in key, f"Key {key!r} leaks 'Ogn' prefix"
    assert "Sim" not in key, f"Key {key!r} leaks 'Sim' prefix"
    assert expected_type.startswith("isaac_core_ogn.sensors.Ros2")


# ─── Topic name defaults agree with isaac_core.contracts.topics ───────────────


def test_global_pose_publisher_topic_default() -> None:
    _, node_def = _load_ogn(_NODES_DIR / "OgnRos2GlobalPosePublisher.ogn")
    topic_default = node_def["inputs"]["topic_name"]["default"]
    expected = topics.join(topics.ROOT, topics.GLOBAL_POSE)
    assert topic_default == expected, f"Expected {expected!r}, got {topic_default!r}"


def test_range_publisher_topic_default() -> None:
    _, node_def = _load_ogn(_NODES_DIR / "OgnRos2RangePublisher.ogn")
    topic_default = node_def["inputs"]["topic_name"]["default"]
    expected = topics.join(topics.ROOT, topics.DISTANCE_SENSOR)
    assert topic_default == expected, f"Expected {expected!r}, got {topic_default!r}"


def test_image_publisher_raw_topic_default() -> None:
    _, node_def = _load_ogn(_NODES_DIR / "OgnRos2ImagePublisher.ogn")
    raw_default = node_def["inputs"]["raw_topic"]["default"]
    expected = topics.join(topics.ROOT, topics.RAW_RGB)
    assert raw_default == expected, f"Expected {expected!r}, got {raw_default!r}"


def test_image_publisher_repub_topic_default() -> None:
    _, node_def = _load_ogn(_NODES_DIR / "OgnRos2ImagePublisher.ogn")
    repub_default = node_def["inputs"]["repub_topic"]["default"]
    expected = topics.join(topics.ROOT, topics.IMAGE_RGB)
    assert repub_default == expected, f"Expected {expected!r}, got {repub_default!r}"


def test_gimbal_topic_default() -> None:
    _, node_def = _load_ogn(_NODES_DIR / "OgnRos2Gimbal.ogn")
    topic_default = node_def["inputs"]["gimbal_topic"]["default"]
    expected = topics.join(topics.ROOT, topics.GIMBAL)
    assert topic_default == expected, f"Expected {expected!r}, got {topic_default!r}"


# ─── Publish rate inputs exist ────────────────────────────────────────────────


def test_global_pose_publisher_has_rate_input() -> None:
    _, node_def = _load_ogn(_NODES_DIR / "OgnRos2GlobalPosePublisher.ogn")
    assert "hz" in node_def["inputs"], "Missing publish rate input 'hz'"
    assert node_def["inputs"]["hz"]["type"] == "int"


def test_range_publisher_has_rate_input() -> None:
    _, node_def = _load_ogn(_NODES_DIR / "OgnRos2RangePublisher.ogn")
    assert "publish_rate_hz" in node_def["inputs"], "Missing publish rate input"
    assert node_def["inputs"]["publish_rate_hz"]["type"] == "int"


def test_image_publisher_has_rate_input() -> None:
    _, node_def = _load_ogn(_NODES_DIR / "OgnRos2ImagePublisher.ogn")
    assert "publish_rate_hz" in node_def["inputs"], "Missing publish rate input"
    assert node_def["inputs"]["publish_rate_hz"]["type"] == "float"


# ─── Attribute types are valid Kit 110 types ──────────────────────────────────

# These are the attribute type names verified against real .ogn files in
# /home/ofer/isaacsim/exts/isaacsim.ros2.nodes.
_VALID_OGN_TYPES = frozenset(
    {
        "execution",
        "string",
        "token",
        "int",
        "uint",
        "uint64",
        "float",
        "double",
        "bool",
        "vectord[3]",
        "vectorf[3]",
        "float[2]",
        "float[3]",
        "float[4]",
        "double[3]",
        "double[4]",
    }
)


@pytest.mark.parametrize("ogn_path", _OGN_FILES, ids=[p.name for p in _OGN_FILES])
def test_ogn_attribute_types_are_valid(ogn_path: Path) -> None:
    _, node_def = _load_ogn(ogn_path)
    for section_name in ("inputs", "outputs"):
        section = node_def.get(section_name, {})
        for port_name, port_def in section.items():
            attr_type = port_def["type"]
            assert attr_type in _VALID_OGN_TYPES, (
                f"{ogn_path.name}.{section_name}.{port_name}: " f"type {attr_type!r} not in validated Kit 110 types"
            )


# ─── Defect #3 regression guard: no rclpy.shutdown() in any node module ──────


@pytest.mark.parametrize("py_path", _PY_FILES, ids=[p.name for p in _PY_FILES])
def test_no_rclpy_shutdown_in_node_modules(py_path: Path) -> None:
    source = py_path.read_text(encoding="utf-8")
    # Check executable lines only — skip comments and docstrings that mention
    # the prohibition for documentation purposes.
    in_docstring = False
    for i, line in enumerate(source.splitlines(), start=1):
        stripped = line.strip()
        # Track triple-quoted docstring boundaries
        if stripped.startswith('"""') or stripped.startswith("'''"):
            # Toggle unless it opens and closes on the same line
            count = stripped.count('"""') + stripped.count("'''")
            if count == 1:
                in_docstring = not in_docstring
            continue
        if in_docstring:
            continue
        if stripped.startswith("#"):
            continue
        assert "rclpy.shutdown()" not in line, (
            f"{py_path.name}:{i} calls 'rclpy.shutdown()' — "
            f"this is defect #3 from the old repo. Nodes must only destroy their own node handle."
        )


# ─── Defect #2 regression guard: state on internal_state, not db ──────────────


@pytest.mark.parametrize("py_path", _PY_FILES, ids=[p.name for p in _PY_FILES])
def test_no_state_stored_on_db(py_path: Path) -> None:
    source = py_path.read_text(encoding="utf-8")
    assert "db._" not in source, (
        f"{py_path.name} stores private attributes on 'db' — "
        f"state must live in internal_state, not on the per-compute database wrapper."
    )


# ─── Defect #23 regression guard: no bare except ─────────────────────────────


@pytest.mark.parametrize("py_path", _PY_FILES, ids=[p.name for p in _PY_FILES])
def test_no_bare_except(py_path: Path) -> None:
    source = py_path.read_text(encoding="utf-8")
    lines = source.splitlines()
    for i, line in enumerate(lines, start=1):
        stripped = line.strip()
        if stripped.startswith("except:"):
            # Allow "except Exception" but not bare "except:"
            pytest.fail(f"{py_path.name}:{i} has bare except clause")


# ─── Image publisher uses the correct Isaac Sim 6 import path ─────────────────


def test_image_publisher_uses_isaacsim_core_nodes_import() -> None:
    source = (_NODES_DIR / "OgnRos2ImagePublisher.py").read_text(encoding="utf-8")
    assert "from isaacsim.core.nodes import BaseWriterNode" in source, (
        "Image publisher must use 'isaacsim.core.nodes' (Isaac Sim 6), " "not the dead 'omni.isaac.core_nodes' path"
    )
    # Check actual import statements, not documentation
    for line in source.splitlines():
        stripped = line.strip()
        if stripped.startswith("import ") or stripped.startswith("from "):
            assert (
                "omni.isaac.core_nodes" not in stripped
            ), "Image publisher still imports from dead 'omni.isaac.core_nodes' path"


# ─── Global pose publisher documents the legacy quaternion quirk ──────────────


def test_global_pose_publisher_documents_legacy_quirk() -> None:
    source = (_NODES_DIR / "OgnRos2GlobalPosePublisher.py").read_text(encoding="utf-8")
    assert (
        "LEGACY PROTOCOL QUIRK" in source or "legacy convention" in source.lower()
    ), "Global pose publisher must prominently document the deliberate quaternion abuse"
