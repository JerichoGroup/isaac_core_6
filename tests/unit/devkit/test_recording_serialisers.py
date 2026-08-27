"""
Tests for the recording serialisers.

Each recorder factory builds a serialiser closure that reads fields off a ROS message. Those
closures had no coverage, so a renamed message field would go uncaught until a live recording
failed. These tests monkeypatch the lazy ROS type loaders (so no ROS install is needed) and
exercise the real closures with duck-typed fake messages, asserting the exact output shape.
"""

from __future__ import annotations

from types import SimpleNamespace

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


def test_bbox_serialiser_extracts_each_box() -> None:
    recorder = recording.bbox_recorder()
    box = SimpleNamespace(target_name="tower", in_frame=True, is_visible=True, x1=1.0, y1=2.0, x2=3.0, y2=4.0)
    msg = SimpleNamespace(bboxes=[box])
    out = recorder._serialiser(msg, 5)
    assert out["frame"] == 5
    assert out["bboxes"] == [
        {
            "target_name": "tower",
            "in_frame": True,
            "is_visible": True,
            "x1": 1.0,
            "y1": 2.0,
            "x2": 3.0,
            "y2": 4.0,
        }
    ]


def test_bbox_serialiser_handles_no_boxes() -> None:
    recorder = recording.bbox_recorder()
    assert recorder._serialiser(SimpleNamespace(bboxes=[]), 0) == {"frame": 0, "bboxes": []}


def test_a_renamed_field_is_caught() -> None:
    # The whole point: if the pose message loses `.latitude`, serialisation fails loudly
    # rather than silently recording garbage.
    recorder = recording.pose_recorder()
    bad = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(), orientation=SimpleNamespace()))
    with pytest.raises(AttributeError):
        recorder._serialiser(bad, 0)


def test_recorder_topics_default_to_the_namespaced_convention() -> None:
    assert recording.pose_recorder()._topic.endswith("global_pose")
    assert recording.range_recorder()._topic.endswith("distance_sensor")
