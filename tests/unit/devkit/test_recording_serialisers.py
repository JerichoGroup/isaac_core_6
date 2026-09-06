"""
Tests for the recording serialisers.

Each recorder factory builds a serialiser closure that reads fields off a ROS message. Those
closures had no coverage, so a renamed message field would go uncaught until a live recording
failed. These tests monkeypatch the lazy ROS type loaders (so no ROS install is needed) and
exercise the real closures with duck-typed fake messages, asserting the exact output shape.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
from types import SimpleNamespace

import numpy as np
import pytest

from isaac_core.devkit import recording


@pytest.fixture(autouse=True)
def _stub_ros_types(monkeypatch: pytest.MonkeyPatch) -> None:
    """Replace the lazy ROS message-type loaders so factories construct without ROS."""
    for name in ("_lazy_geopose_type", "_lazy_range_type", "_lazy_framebboxes_type", "_lazy_image_type"):
        monkeypatch.setattr(recording, name, lambda: object)


def test_pose_serialiser_extracts_lla_and_quaternion() -> None:
    recorder = recording.pose_recorder()
    msg = SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(latitude=32.3, longitude=35.25, altitude=1500.0),
            orientation=SimpleNamespace(x=0.1, y=0.2, z=0.3, w=0.9),
        )
    )
    out = recorder._serialiser(msg, 7)
    assert out == {
        "frame": 7,
        "lat": 32.3,
        "lon": 35.25,
        "alt": 1500.0,
        "qx": 0.1,
        "qy": 0.2,
        "qz": 0.3,
        "qw": 0.9,
    }


def test_range_serialiser_extracts_range_fields() -> None:
    recorder = recording.range_recorder()
    msg = SimpleNamespace(range=12.5, min_range=0.1, max_range=100.0)
    assert recorder._serialiser(msg, 3) == {
        "frame": 3,
        "range_m": 12.5,
        "min_range_m": 0.1,
        "max_range_m": 100.0,
    }


def _flat_frame(count: int) -> SimpleNamespace:
    """Build a fake flat FrameBboxes with `count` detections."""
    return SimpleNamespace(
        target_name=[f"t{i}" for i in range(count)],
        in_frame=[True] * count,
        is_visible=[i % 2 == 0 for i in range(count)],
        x1=[10 * i for i in range(count)],
        y1=[20 * i for i in range(count)],
        x2=[10 * i + 5 for i in range(count)],
        y2=[20 * i + 5 for i in range(count)],
        lat=[32.0 + i for i in range(count)],
        lon=[35.0 + i for i in range(count)],
        alt=[500.0 + i for i in range(count)],
        roll=[0.1 * i for i in range(count)],
        pitch=[0.2 * i for i in range(count)],
        yaw=[0.3 * i for i in range(count)],
        distance_x=[1.0 * i for i in range(count)],
        distance_y=[2.0 * i for i in range(count)],
        distance_z=[3.0 * i for i in range(count)],
    )


def test_bbox_serialiser_rezips_parallel_arrays_into_one_dict_per_detection() -> None:
    # FrameBboxes carries parallel arrays, not a Bbox[]. The recorder re-zips them so a reader
    # gets one dict per detection instead of having to index 16 arrays itself.
    recorder = recording.bbox_recorder()
    result = recorder._serialiser(_flat_frame(2), 7)

    assert result["frame"] == 7
    assert [d["target_name"] for d in result["bboxes"]] == ["t0", "t1"]
    assert [d["x1"] for d in result["bboxes"]] == [0, 10]
    assert [d["is_visible"] for d in result["bboxes"]] == [True, False]


def test_bbox_serialiser_captures_all_sixteen_fields() -> None:
    # Guards a real past defect: 9 of 16 fields were silently dropped. Assert the count so a
    # future edit that loses one is caught immediately.
    recorder = recording.bbox_recorder()
    detection = recorder._serialiser(_flat_frame(1), 0)["bboxes"][0]

    assert len(detection) == 16, f"expected 16 fields, got {sorted(detection)}"
    for name in ("lat", "lon", "alt", "roll", "pitch", "yaw", "distance_x", "distance_y", "distance_z"):
        assert name in detection


def test_bbox_serialiser_handles_no_detections() -> None:
    # An empty frame is meaningful -- processed and nothing found -- so it must serialise to an
    # empty list rather than raising or being skipped.
    recorder = recording.bbox_recorder()
    result = recorder._serialiser(_flat_frame(0), 3)

    assert result == {"frame": 3, "bboxes": []}


def test_bbox_serialiser_converts_numpy_scalars_to_json_encodable_values() -> None:
    # ROS array fields deserialise to numpy arrays whose elements json cannot encode. Without
    # conversion, recording fails only at write time, long after the data was captured.
    frame = _flat_frame(1)
    frame.x1 = np.array([42], dtype=np.int32)
    frame.lat = np.array([32.5], dtype=np.float32)
    recorder = recording.bbox_recorder()
    detection = recorder._serialiser(frame, 0)["bboxes"][0]

    assert isinstance(detection["x1"], int)
    assert isinstance(detection["lat"], float)
    json.dumps(detection)


def test_bbox_serialiser_raises_when_a_field_is_missing() -> None:
    # If FrameBboxes loses a field, serialisation must fail loudly rather than silently
    # recording a partial detection.
    frame = _flat_frame(1)
    del frame.distance_z
    recorder = recording.bbox_recorder()
    with pytest.raises(AttributeError):
        recorder._serialiser(frame, 0)


def test_bbox_recorder_imports_our_own_message_package() -> None:
    # The bbox recorder used to import isaac_ros2_messages, which is NVIDIA's package. That
    # only ever resolved because a 2023 workspace happened to be built on the dev machine, and
    # we cannot ship definitions into it: it also carries the .srv files Isaac's ROS bridge
    # needs, so overlaying it would break Isaac. Ours is isaac_core_ros2_msgs.
    source = Path(recording.__file__).read_text(encoding="utf-8")
    assert "isaac_core_ros2_msgs.msg import FrameBboxes" in source
    assert "isaac_ros2_messages" not in source, "recorder must not import NVIDIA's message package"


def test_shipped_message_package_declares_only_what_it_defines() -> None:
    # A rosidl package that lists a .msg it does not contain fails at build time, and one that
    # ships a .msg it does not list silently produces no Python class. The 2023 package had the
    # first defect: its CMakeLists listed five .srv files that were not in the folder.
    pkg = Path(__file__).resolve().parents[3] / "ros2" / "isaac_core_ros2_msgs"
    cmake = (pkg / "CMakeLists.txt").read_text(encoding="utf-8")
    listed = set(re.findall(r'"(msg/[^"]+\.msg)"', cmake))
    present = {f"msg/{p.name}" for p in (pkg / "msg").glob("*.msg")}
    assert listed == present, f"CMakeLists lists {listed} but folder has {present}"
