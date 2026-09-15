"""Validate that every OmniGraph node that needs ticking is actually wired to tick.

An unconnected ``execIn`` is the quietest failure mode in OmniGraph: the node exists,
its inputs are set, the graph loads without a warning, and it simply never computes.
This was real -- ``ros2_publisher.execIn`` was unconnected in both camera layers, so
``/isaac_core/global_pose`` never appeared on the ROS graph, and the only symptom was a
topic that "does not appear to be published yet".

Parsed as text so these run with neither Isaac Sim nor ROS 2 present.
"""

from pathlib import Path
import re
from typing import Final

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
LAYERS = sorted((REPO_ROOT / "src" / "isaac_core" / "assets" / "layers").rglob("*.usda"))

# Node types that legitimately have no execIn connection.
#
# ROS2Context is a data-only provider: it hands a context handle to other nodes and is
# never ticked. Anything else with an execIn input is expected to be driven.
DATA_ONLY_NODE_TYPES: Final[frozenset[str]] = frozenset(
    {
        "isaacsim.ros2.bridge.ROS2Context",
        "isaacsim.ros2.bridge.ROS2QoSProfile",
    }
)


def _graphs(text: str) -> list[tuple[str, str]]:
    """Return (graph_name, graph_body) for every OmniGraph in a USD file."""
    return [
        (match.group(1), match.group(2))
        for match in re.finditer(r'def OmniGraph "(\w+)"(.*?)(?=\n    def |\Z)', text, re.DOTALL)
    ]


def _nodes(graph_body: str) -> list[tuple[str, str]]:
    """Return (node_name, node_body) for every node in a graph."""
    return [
        (match.group(1), match.group(2))
        for match in re.finditer(
            r'def OmniGraphNode "(\w+)"(.*?)(?=\n        def OmniGraphNode |\Z)',
            graph_body,
            re.DOTALL,
        )
    ]


def _node_type(node_body: str) -> str:
    """Return a node's declared type, or an empty string."""
    match = re.search(r'node:type = "([^"]+)"', node_body)
    return match.group(1) if match else ""


def test_layers_were_found() -> None:
    # Without this the parametrised test below would pass vacuously.
    assert LAYERS, "no layer USD files found"


@pytest.mark.parametrize("layer", LAYERS, ids=lambda p: p.stem)
def test_every_tickable_node_has_its_exec_input_connected(layer: Path) -> None:
    text = layer.read_text(encoding="utf-8")
    unwired: list[str] = []

    for graph_name, graph_body in _graphs(text):
        for node_name, node_body in _nodes(graph_body):
            if "inputs:execIn" not in node_body:
                continue
            if _node_type(node_body) in DATA_ONLY_NODE_TYPES:
                continue
            if "inputs:execIn.connect" not in node_body:
                unwired.append(f"{graph_name}/{node_name} ({_node_type(node_body)})")

    assert not unwired, (
        f"{layer.name}: these nodes declare an execIn but nothing drives them, so they "
        f"never compute and fail silently:\n  " + "\n  ".join(unwired)
    )


# Timestamp inputs on a published message. Left unconnected they publish as zero, which
# looks like a valid message but carries no usable time.
_STAMP_INPUTS: Final[tuple[str, ...]] = ("inputs:header:stamp:sec", "inputs:header:stamp:nanosec")


@pytest.mark.parametrize("layer", LAYERS, ids=lambda p: p.stem)
def test_published_messages_carry_a_timestamp(layer: Path) -> None:
    # Verified live: /isaac_core/global_pose publishes at ~100 Hz with correct lat/lon/alt
    # but `stamp: {sec: 0, nanosec: 0}`, because these inputs are not connected to a
    # simulation-time source. A consumer cannot order or correlate those messages.
    text = layer.read_text(encoding="utf-8")
    missing: list[str] = []

    for graph_name, graph_body in _graphs(text):
        for node_name, node_body in _nodes(graph_body):
            for stamp_input in _STAMP_INPUTS:
                if stamp_input not in node_body:
                    continue
                if f"{stamp_input}.connect" not in node_body:
                    missing.append(f"{graph_name}/{node_name} {stamp_input}")

    assert not missing, (
        f"{layer.name}: these timestamp inputs are unconnected, so the message publishes "
        f"with a zero stamp:\n  " + "\n  ".join(missing)
    )


def _unconnected_render_product_inputs(text: str) -> list[str]:
    """Return graph nodes that declare `inputs:renderProductPath` without connecting it.

    Args:
        text: The USD layer text.

    Returns:
        Node names with an unconnected render product input.

    """
    offenders: list[str] = []
    for block in re.split(r"\n(?=\s*def OmniGraphNode )", text):
        name_match = re.search(r'def OmniGraphNode "([^"]+)"', block)
        if name_match is None:
            continue
        if "inputs:renderProductPath" not in block:
            continue
        if "inputs:renderProductPath.connect" not in block:
            offenders.append(name_match.group(1))
    return offenders


@pytest.mark.parametrize("layer", LAYERS, ids=lambda p: p.stem)
def test_render_product_inputs_are_connected(layer: Path) -> None:
    # A declared-but-unconnected `renderProductPath` does not error: the node silently falls back
    # to the ACTIVE viewport. With two vehicles that meant both wrote their camera to the same
    # viewport every frame -- the view flipped between aircraft -- while neither named viewport
    # ever received its camera at all. Same failure shape as the unwired `execIn` this file already
    # guards: an input that goes quiet instead of complaining.
    offenders = _unconnected_render_product_inputs(layer.read_text(encoding="utf-8"))

    assert not offenders, (
        f"{layer.name}: {offenders} declare inputs:renderProductPath without connecting it. "
        f"Connect it to isaac_get_viewport_render_product.outputs:renderProductPath, or the node "
        f"silently acts on the active viewport instead of this vehicle's render product."
    )
