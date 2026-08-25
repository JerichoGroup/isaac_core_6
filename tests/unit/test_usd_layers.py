"""
Structural validation of the authored USD layers.

USD is authored by hand in the Isaac Sim GUI, which means the usual mistakes are
possible: a Save-As leaves connections pointing at nodes that only existed in the
file it was copied from, an input ends up with two sources, or a required link is
simply never made. None of that is caught by anything else in the repo, and all of it
manifests as "the camera does not move" with no error message.

Both failures below were real. `camera_ros.usda` was created by Save-As from
`camera_udp.usda` and retained two connections to `udp_to_global_position`, a node
that does not exist in it -- which is why its `global_orientation` input silently read
as unconnected.

Parsed as text, so these run with neither Isaac Sim nor ROS 2 present. Text parsing is
deliberate: importing `pxr` would restrict these to Isaac's interpreter.
"""

from pathlib import Path
import re

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
USD_ROOT = REPO_ROOT / "usd"

CAMERA_LAYERS = sorted(USD_ROOT.glob("layers/camera_*/*.usda"))
SCENES = sorted(USD_ROOT.glob("scenes/*.usda"))

# The prim the pose graph drives. Its transform stack is a contract: the writers
# target these exact attribute names.
MOVED_PRIM = "/Root/Xform"
REQUIRED_XFORM_OPS = ("xformOp:translate", "xformOp:orient")


def _text(path: Path) -> str:
    """Return a USD file's text."""
    return path.read_text(encoding="utf-8")


def _graph_nodes(text: str) -> set[str]:
    """Return the names of every OmniGraphNode defined in a file."""
    return set(re.findall(r'def OmniGraphNode "([^"]+)"', text))


def _output_references(text: str) -> set[tuple[str, str, str]]:
    """Return every (graph, node, attribute) triple referenced by a connection."""
    return set(re.findall(r"</Root/(\w+)/(\w+)\.outputs:([\w:]+)>", text))


def _connections_for(text: str, node: str, attribute: str) -> list[str]:
    """Return the connection sources declared for one node input."""
    pattern = rf'def OmniGraphNode "{node}"(.*?)(?=\n        def OmniGraphNode |\Z)'
    block = re.search(pattern, text, re.DOTALL)
    if block is None:
        return []
    conn = re.search(rf"inputs:{re.escape(attribute)}\.connect = (.+?)(?=\n\s+\w|\Z)", block.group(1), re.DOTALL)
    if conn is None:
        return []
    return re.findall(r"</[^>]+>", conn.group(1))


def test_usd_files_were_found() -> None:
    # Without this, every parametrised test below would pass vacuously.
    assert CAMERA_LAYERS, f"no camera layers found under {USD_ROOT}"
    assert SCENES, f"no scenes found under {USD_ROOT}"


@pytest.mark.parametrize("layer", CAMERA_LAYERS, ids=lambda p: p.stem)
def test_no_connection_references_a_missing_node(layer: Path) -> None:
    # The Save-As trap: a connection surviving into a file whose target node does not.
    text = _text(layer)
    nodes = _graph_nodes(text)
    dangling = sorted(f"{node}.outputs:{attr}" for _, node, attr in _output_references(text) if node not in nodes)
    assert not dangling, (
        f"{layer.name} references nodes that do not exist in it " f"(likely left over from a Save-As): {dangling}"
    )


@pytest.mark.parametrize("layer", CAMERA_LAYERS, ids=lambda p: p.stem)
def test_math_node_inputs_have_exactly_one_source(layer: Path) -> None:
    # Two sources on one input is ambiguous even when one of them is dangling.
    text = _text(layer)
    for attribute in ("global_position", "global_orientation"):
        sources = _connections_for(text, "global_position_to_local_position", attribute)
        assert len(sources) == 1, (
            f"{layer.name}: global_position_to_local_position.{attribute} has "
            f"{len(sources)} sources, expected exactly 1: {sources}"
        )


@pytest.mark.parametrize("layer", CAMERA_LAYERS, ids=lambda p: p.stem)
def test_pose_writers_target_the_moved_prim(layer: Path) -> None:
    text = _text(layer)
    for op in REQUIRED_XFORM_OPS:
        assert f'inputs:name = "{op}"' in text, f"{layer.name}: no writer targets {op}"
    assert f'inputs:primPath = "{MOVED_PRIM}"' in text


@pytest.mark.parametrize("layer", CAMERA_LAYERS, ids=lambda p: p.stem)
def test_moved_prim_declares_translate_and_orient(layer: Path) -> None:
    # An orient op is required: writing a quaternion to a prim that only has
    # rotateXYZ silently does nothing.
    text = _text(layer)
    block = re.search(r'def Xform "Xform"\s*\([^)]*\)\s*\{(.*?)\n    \}', text, re.DOTALL)
    assert block is not None, f"{layer.name}: could not find {MOVED_PRIM}"
    for op in REQUIRED_XFORM_OPS:
        assert op in block.group(1), f"{layer.name}: {MOVED_PRIM} lacks {op}"


@pytest.mark.parametrize("layer", CAMERA_LAYERS, ids=lambda p: p.stem)
def test_published_orientation_is_a_real_quaternion(layer: Path) -> None:
    # Decision D16: GeoPoseStamped carries a genuine quaternion via EulerToQuaternion,
    # replacing the old repo's abuse of stuffing roll/pitch/yaw into x/y/z with w=1.
    text = _text(layer)
    wired = dict(re.findall(r"inputs:pose:orientation:(\w)\.connect = (\S+)", text))
    assert set(wired) == {
        "w",
        "x",
        "y",
        "z",
    }, f"{layer.name}: publisher orientation should have all four components wired, got {sorted(wired)}"
    for component, source in wired.items():
        assert "euler_to_quaternion" in source, (
            f"{layer.name}: orientation:{component} comes from {source}, " f"expected the EulerToQuaternion node"
        )


@pytest.mark.parametrize("layer", CAMERA_LAYERS, ids=lambda p: p.stem)
def test_moved_prim_has_a_cesium_globe_anchor(layer: Path) -> None:
    # Without an anchor the camera moves in local ENU while the terrain is
    # georeferenced, and the two drift apart.
    text = _text(layer)
    assert "cesium:anchor" in text or "GlobeAnchor" in text, f"{layer.name}: {MOVED_PRIM} needs a Cesium Globe Anchor"


@pytest.mark.parametrize("layer", CAMERA_LAYERS, ids=lambda p: p.stem)
def test_layers_do_not_embed_the_viewport_camera_gizmo(layer: Path) -> None:
    # This mesh is what made the old repo's camera layers 2.9 MB each.
    assert "OmniverseKitViewportCameraMesh" not in _text(layer)


@pytest.mark.parametrize("layer", CAMERA_LAYERS, ids=lambda p: p.stem)
def test_layers_stay_small(layer: Path) -> None:
    size_kb = layer.stat().st_size // 1024
    assert size_kb < 500, f"{layer.name} is {size_kb} kB — check for embedded geometry"


@pytest.mark.parametrize("scene", SCENES, ids=lambda p: p.stem)
def test_scene_declares_the_mount_and_capability_roots(scene: Path) -> None:
    text = _text(scene)
    assert 'def Xform "Environment"' in text, "scene needs /World/Environment as a mount point"
    # These two may be empty; the capability probe skips dependent features when
    # absent rather than crashing. Their presence is what makes those features usable.
    assert 'def Scope "tilesets"' in text
    assert 'def Scope "bboxes"' in text


@pytest.mark.parametrize("scene", SCENES, ids=lambda p: p.stem)
def test_scene_has_a_cesium_georeference(scene: Path) -> None:
    assert "CesiumGeoreferencePrim" in _text(scene)
