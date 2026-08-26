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

import numpy as np
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


def _prim_block(text: str, prim_type: str, name: str) -> str | None:
    """
    Return the body of a prim definition, or ``None`` if absent.

    The parenthetical metadata block after a prim name is optional in USD -- removing
    a Globe Anchor removes the `(prepend apiSchemas = [...])` that came with it. An
    earlier version of this helper required the parentheses and so silently stopped
    matching, turning real checks into false failures.
    """
    pattern = rf'def {prim_type} "{name}"(?:\s*\([^)]*\))?\s*\{{(.*?)\n(?:    |        )\}}'
    match = re.search(pattern, text, re.DOTALL)
    return match.group(1) if match else None


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
    # Targeting is by USD relationship, not by path string, so that the layer survives
    # being referenced under any mount point. This assertion used to check
    # `inputs:primPath` and went stale the moment that was correctly cleared.
    text = _text(layer)
    for op in REQUIRED_XFORM_OPS:
        assert f'inputs:name = "{op}"' in text, f"{layer.name}: no writer targets {op}"
    assert f"inputs:prim = <{MOVED_PRIM}>" in text, f"{layer.name}: no writer targets {MOVED_PRIM} by relationship"


@pytest.mark.parametrize("layer", CAMERA_LAYERS, ids=lambda p: p.stem)
def test_moved_prim_declares_translate_and_orient(layer: Path) -> None:
    # An orient op is required: writing a quaternion to a prim that only has
    # rotateXYZ silently does nothing.
    text = _text(layer)
    body = _prim_block(text, "Xform", "Xform")
    assert body is not None, f"{layer.name}: could not find {MOVED_PRIM}"
    for op in REQUIRED_XFORM_OPS:
        assert op in body, f"{layer.name}: {MOVED_PRIM} lacks {op}"


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
def test_moved_prim_has_no_cesium_globe_anchor(layer: Path) -> None:
    # This test asserted the OPPOSITE until 2026-08-25, because the build sheet wrongly
    # told the author to add a Globe Anchor here. It cost a debugging session: composed
    # under a scene georeferenced elsewhere, the anchor put the camera 9,700 km away.
    # The layer had been authored standalone, where its own CesiumGeoreference had no
    # origin authored, so Cesium fell back to its Denver default and the anchor recorded
    # the Xform as being in Colorado.
    #
    # No anchor is needed: the graph writes local ENU into xformOp:translate every tick
    # and the scene's CesiumGeoreference already maps stage origin to a lat/lon, so a
    # local translate fully describes the camera. An anchor is a second writer fighting
    # the first. The old repo anchored nothing on its camera Xforms either.
    text = _text(layer)
    body = _prim_block(text, "Xform", "Xform")
    assert body is not None, f"{layer.name}: could not find {MOVED_PRIM}"
    anchors = sorted(set(re.findall(r"cesium:anchor:\w+", body)))
    assert not anchors, (
        f"{layer.name}: {MOVED_PRIM} carries a Cesium Globe Anchor ({anchors}). "
        f"Remove it — it fights the pose graph and captures whichever georeference "
        f"origin happened to be active when the layer was authored."
    )


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


@pytest.mark.parametrize("layer", CAMERA_LAYERS, ids=lambda p: p.stem)
def test_writers_target_the_prim_by_relationship_not_by_path(layer: Path) -> None:
    # USD remaps relationship targets when a layer is referenced; a string path does
    # not remap. Using primPath meant /Root/Xform was wrong once the layer was mounted
    # under /World/Environment/drone_0, and hardcoding the composed path instead would
    # pin the layer to one mount point, breaking swarm instances.
    text = _text(layer)
    assert "inputs:usePath = 1" not in text, (
        f"{layer.name}: a writer has usePath=true, so it targets a hardcoded string "
        f"path. Set the `prim` relationship to /Root/Xform and usePath=false."
    )
    assert (
        "inputs:prim = </Root/Xform>" in text or "inputs:prim.connect" in text
    ), f"{layer.name}: no writer targets /Root/Xform by relationship"


@pytest.mark.parametrize("layer", CAMERA_LAYERS, ids=lambda p: p.stem)
def test_camera_sits_at_its_parent_origin(layer: Path) -> None:
    # The graph writes the PARENT's translate; the camera's own translate is never
    # touched, so any offset here is permanent and adds to the commanded altitude.
    text = _text(layer)
    body = _prim_block(text, "Camera", "main_camera_01")
    assert body is not None, f"{layer.name}: could not find main_camera_01"
    translate = re.search(r"xformOp:translate = \(([^)]*)\)", body)
    if translate is None:
        return
    components = [float(v) for v in translate.group(1).split(",")]
    assert all(abs(v) < 1e-6 for v in components), (
        f"{layer.name}: main_camera_01 translate is {components}, expected ~(0,0,0). "
        f"A non-zero offset here doubles the commanded altitude."
    )


@pytest.mark.parametrize("layer", CAMERA_LAYERS, ids=lambda p: p.stem)
def test_camera_looks_along_the_body_forward_axis(layer: Path) -> None:
    # In the parent frame the body axes are +X nose, +Y left wing, +Z up (verified
    # numerically from ned_to_enu: yaw_ned=0 puts +X North, 90 puts it East). A USD
    # camera looks down its own -Z, so the camera rotation must map -Z to +X and +Y to
    # +Z, i.e. rotateXYZ (90, 0, -90). The first build had it pointing 90 degrees off,
    # which is invisible while hovering and only shows up in motion.
    from transforms3d.quaternions import quat2mat

    body_forward = np.array([1.0, 0.0, 0.0])
    body_up = np.array([0.0, 0.0, 1.0])

    body = _prim_block(_text(layer), "Camera", "main_camera_01")
    assert body is not None, f"{layer.name}: could not find main_camera_01"

    orient = re.search(r"xformOp:orient = \(([^)]*)\)", body)
    assert orient is not None, (
        f"{layer.name}: main_camera_01 has no xformOp:orient. The camera needs an "
        f"explicit orientation, since a USD camera at identity looks straight down."
    )

    # USD stores quatd in .usda text as (real, i, j, k) = (w, x, y, z).
    rotation = quat2mat([float(v) for v in orient.group(1).split(",")])
    view = rotation @ np.array([0.0, 0.0, -1.0])
    image_up = rotation @ np.array([0.0, 1.0, 0.0])

    assert np.allclose(view, body_forward, atol=1e-6), (
        f"{layer.name}: camera looks along {np.round(view, 3)}, expected the body "
        f"forward axis {body_forward}. Set rotateXYZ = (90, 0, -90)."
    )
    assert np.allclose(
        image_up, body_up, atol=1e-6
    ), f"{layer.name}: camera image-up is {np.round(image_up, 3)}, expected {body_up}"
