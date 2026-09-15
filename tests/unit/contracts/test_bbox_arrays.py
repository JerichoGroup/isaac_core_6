"""Tests for the bbox parallel-array contract.

The message ships sixteen parallel arrays rather than an array of per-object messages, because Isaac
Sim 6 exposes a nested message array as an unusable ``token[]`` that segfaults on write. That makes
index alignment the entire contract: one target appended to fifteen arrays but not the sixteenth
silently shifts every later detection by one.
"""

from __future__ import annotations

import math

import pytest

from isaac_core.contracts.bbox import ARRAY_FIELDS, BboxArrays, BboxDetection


def _detection(name: str, *, visible: bool = True, anchored: bool = True) -> BboxDetection:
    """Build a detection with distinguishable values, for index-alignment checks."""
    index = int(name.rsplit("_", 1)[-1])
    nan = float("nan")
    return BboxDetection(
        target_name=name,
        x1=index * 10,
        y1=index * 10 + 1,
        x2=index * 10 + 2,
        y2=index * 10 + 3,
        in_frame=True,
        is_visible=visible,
        lat=32.0 + index if anchored else nan,
        lon=35.0 + index if anchored else nan,
        alt=500.0 + index if anchored else nan,
        roll=0.01 * index,
        pitch=0.02 * index,
        yaw=0.03 * index,
        distance_x=100.0 * index,
        distance_y=200.0 * index,
        distance_z=300.0 * index,
    )


def test_a_new_container_is_empty_and_aligned() -> None:
    arrays = BboxArrays()
    assert len(arrays) == 0
    assert arrays.aligned()


def test_every_array_grows_together() -> None:
    arrays = BboxArrays()
    for index in range(5):
        arrays.append(_detection(f"cube_{index}"))
    assert len(arrays) == 5
    assert arrays.aligned()
    for name in ARRAY_FIELDS:
        assert len(getattr(arrays, name)) == 5, f"{name} fell out of step"


def test_index_i_is_the_same_object_in_every_array() -> None:
    # The invariant a consumer relies on: read detection i as a slice across all sixteen arrays.
    arrays = BboxArrays()
    detections = [_detection(f"cube_{index}") for index in range(4)]
    for detection in detections:
        arrays.append(detection)
    for index, detection in enumerate(detections):
        for name in ARRAY_FIELDS:
            expected = getattr(detection, name)
            actual = getattr(arrays, name)[index]
            if isinstance(expected, float) and math.isnan(expected):
                assert math.isnan(actual), f"{name}[{index}] lost its NaN"
            else:
                assert actual == expected, f"{name}[{index}] misaligned"


def test_an_invisible_target_still_occupies_its_index() -> None:
    # A target the annotators do not mention must still get a row, or every later detection shifts.
    arrays = BboxArrays()
    arrays.append(_detection("cube_0", visible=True))
    arrays.append(_detection("cube_1", visible=False))
    arrays.append(_detection("cube_2", visible=True))
    assert len(arrays) == 3
    assert arrays.is_visible == [True, False, True]
    assert arrays.target_name == ["cube_0", "cube_1", "cube_2"]


def test_a_target_with_no_geodetic_anchor_still_occupies_its_index() -> None:
    # No anchor means NaN lat/lon/alt, not a skipped row.
    arrays = BboxArrays()
    arrays.append(_detection("cube_0", anchored=True))
    arrays.append(_detection("cube_1", anchored=False))
    assert len(arrays) == 2
    assert arrays.aligned()
    assert not math.isnan(arrays.lat[0])
    assert math.isnan(arrays.lat[1])
    # The box for an anchorless target is still real and still at its own index.
    assert arrays.x1 == [0, 10]


def test_as_outputs_covers_every_declared_field() -> None:
    # The node writes whatever this returns, so a field missing here would never reach the topic.
    arrays = BboxArrays()
    arrays.append(_detection("cube_0"))
    outputs = arrays.as_outputs()
    assert set(outputs) == set(ARRAY_FIELDS)
    assert all(len(values) == 1 for values in outputs.values())


def test_the_field_list_matches_the_detection_shape() -> None:
    # ARRAY_FIELDS and BboxDetection must not drift: append() reads the detection by these names.
    detection_fields = set(BboxDetection.__dataclass_fields__)
    assert set(ARRAY_FIELDS) == detection_fields


def test_the_field_list_matches_the_shipped_message_definition() -> None:
    # The ultimate authority is the .msg file a consumer compiles against.
    from pathlib import Path

    msg = Path(__file__).resolve().parents[3] / "ros2" / "isaac_core_ros2_msgs" / "msg" / "FrameBboxes.msg"
    declared = {
        line.split()[1].strip()
        for line in msg.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#") and len(line.split()) >= 2
    }
    # `count` is a scalar, not one of the parallel arrays.
    assert set(ARRAY_FIELDS) <= declared, f"fields missing from the .msg: {set(ARRAY_FIELDS) - declared}"


@pytest.mark.parametrize("count", [0, 1, 2, 17])
def test_alignment_holds_for_any_number_of_detections(count: int) -> None:
    arrays = BboxArrays()
    for index in range(count):
        arrays.append(_detection(f"cube_{index}"))
    assert len(arrays) == count
    assert arrays.aligned()
