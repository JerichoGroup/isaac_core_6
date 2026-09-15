"""Guards that an unconnected OGN input falls back to the same value config would supply.

A binding that silently fails to land leaves the attribute at its ``.ogn`` default, which looks
exactly like success. When that default disagrees with config, the node runs happily on a value no
user ever chose -- which is how the distance sensor came to ship a 100 m default while config said
5000 m, and the schema's own docstring explains that a 100 m ray never reaches the ground and reports
"no detection" forever while looking perfectly healthy.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from isaac_core.config import IsaacCoreConfig
from isaac_core.contracts.ports import DEFAULT_POSE_UDP_PORT

EXTENSIONS = Path(__file__).resolve().parents[3] / "extensions"


def _ogn_inputs(node_suffix: str) -> dict[str, Any]:
    """Return the inputs block of the shipped .ogn whose node name ends with ``node_suffix``."""
    for path in EXTENSIONS.rglob("*.ogn"):
        if "_template" in str(path):
            continue
        for node, body in json.loads(path.read_text(encoding="utf-8")).items():
            if node.endswith(node_suffix):
                return dict(body.get("inputs") or {})
    pytest.fail(f"no shipped .ogn declares a node ending in {node_suffix!r}")
    raise AssertionError  # unreachable, satisfies the type checker


def test_the_pose_node_reference_default_matches_the_config_default() -> None:
    inputs = _ogn_inputs("GlobalPositionToLocalPosition")
    reference = IsaacCoreConfig().geo.enu_reference
    assert inputs["enu_reference"]["default"] == [
        reference.lat_deg,
        reference.lon_deg,
        reference.alt_m,
    ], "an unconnected enu_reference would place the aircraft somewhere config never asked for"


def test_the_udp_node_port_default_matches_the_shared_constant() -> None:
    inputs = _ogn_inputs("UdpToGlobalPosition")
    assert inputs["udp_port"]["default"] == DEFAULT_POSE_UDP_PORT


def test_the_distance_sensor_band_defaults_match_the_config_defaults() -> None:
    inputs = _ogn_inputs("DistanceSensor")
    sensor = IsaacCoreConfig(vehicles={"drone_0": {"cameras": {"eo": {}}}}).vehicles["drone_0"].distance_sensor
    assert inputs["min_range_m"]["default"] == sensor.min_range_m
    assert inputs["max_range_m"]["default"] == sensor.max_range_m, (
        "the .ogn fallback disagrees with config; an unbound input would use a range that may never " "reach the ground"
    )


def test_no_shipped_ogn_still_claims_it_reports_infinities() -> None:
    # The rangefinder saturates at its rated limits instead, because int(inf) raises and crashed
    # exactly when the sensor saw nothing.
    offenders: list[str] = []
    for path in EXTENSIONS.rglob("*.ogn"):
        if "_template" in str(path):
            continue
        text = path.read_text(encoding="utf-8")
        if "+inf" in text or "-inf" in text:
            offenders.append(path.name)
    assert not offenders, f"these .ogn files still document infinity returns: {offenders}"
