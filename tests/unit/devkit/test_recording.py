"""Tests for isaac_core.devkit.recording: generic TopicRecorder."""

from pathlib import Path
import pickle
import sys
from typing import Any

import pytest

from isaac_core.devkit.recording import TopicRecorder

# -- import safety ---------------------------------------------------------- #


def test_import_recording_succeeds_without_rclpy() -> None:
    # Block rclpy via sys.modules to prove import succeeds without it.
    # This is the critical property that keeps CI working.
    sentinel = sys.modules.get("rclpy")
    sys.modules["rclpy"] = None  # type: ignore[assignment]
    try:
        # Force reimport.
        import importlib  # noqa: PLC0415

        import isaac_core.devkit.recording  # noqa: PLC0415

        importlib.reload(isaac_core.devkit.recording)
        # Should succeed.
        assert hasattr(isaac_core.devkit.recording, "TopicRecorder")
        assert hasattr(isaac_core.devkit.recording, "video_recorder")
    finally:
        if sentinel is None:
            del sys.modules["rclpy"]
        else:
            sys.modules["rclpy"] = sentinel


def test_topic_recorder_start_raises_without_rclpy() -> None:
    # Ensure rclpy is blocked.
    sentinel = sys.modules.get("rclpy")
    sys.modules["rclpy"] = None  # type: ignore[assignment]
    try:
        recorder = TopicRecorder(
            topic="/test/topic",
            msg_type=object,
            serialiser=lambda msg, idx: msg,
        )
        with pytest.raises(ImportError, match="rclpy"):
            recorder.start()
    finally:
        if sentinel is None:
            del sys.modules["rclpy"]
        else:
            sys.modules["rclpy"] = sentinel


# -- TopicRecorder with a fake message source (no rclpy needed) ------------- #


def test_topic_recorder_serialises_messages() -> None:
    # Test the recorder's internal logic without actually subscribing.
    recorder = TopicRecorder(
        topic="/test/data",
        msg_type=object,
        serialiser=lambda msg, idx: {"value": msg, "index": idx},
    )
    # Manually invoke the callback (simulating message arrival).
    recorder._recording = True
    recorder._on_message("hello")
    recorder._on_message("world")

    assert recorder.frame_count == 2
    assert recorder._frames[0] == {"value": "hello", "index": 0}
    assert recorder._frames[1] == {"value": "world", "index": 1}


def test_topic_recorder_stop_halts_recording() -> None:
    recorder = TopicRecorder(
        topic="/test/data",
        msg_type=object,
        serialiser=lambda msg, idx: msg,
    )
    recorder._recording = True
    recorder._on_message("before_stop")
    recorder.stop()
    recorder._on_message("after_stop")

    assert recorder.frame_count == 1
    assert recorder._frames[0] == "before_stop"


def test_topic_recorder_save_to(tmp_path: Path) -> None:
    recorder = TopicRecorder(
        topic="/test/data",
        msg_type=object,
        serialiser=lambda msg, idx: {"msg": msg, "idx": idx},
    )
    recorder._recording = True
    for i in range(5):
        recorder._on_message(f"frame_{i}")

    out_path = tmp_path / "output.pkl"
    result = recorder.save_to(out_path)

    assert result == out_path
    assert out_path.exists()

    with out_path.open("rb") as fh:
        loaded = pickle.load(fh)  # noqa: S301

    assert len(loaded) == 5
    assert loaded[0] == {"msg": "frame_0", "idx": 0}
    assert loaded[4] == {"msg": "frame_4", "idx": 4}


def test_topic_recorder_context_manager() -> None:
    recorder = TopicRecorder(
        topic="/test/ctx",
        msg_type=object,
        serialiser=lambda msg, idx: msg,
    )
    recorder._recording = True

    with recorder as rec:
        rec._on_message("in_context")
        assert rec.frame_count == 1

    # After exit, recording should be stopped.
    assert not recorder.recording


def test_topic_recorder_properties() -> None:
    recorder = TopicRecorder(
        topic="/test/props",
        msg_type=object,
        serialiser=lambda msg, idx: msg,
        node_name="custom_node",
    )
    assert recorder.topic == "/test/props"
    assert not recorder.recording
    assert recorder.frame_count == 0


def test_topic_recorder_default_node_name() -> None:
    recorder = TopicRecorder(
        topic="/isaac_core/image_rgb",
        msg_type=object,
        serialiser=lambda msg, idx: msg,
    )
    assert recorder._node_name == "recorder_isaac_core_image_rgb"


def test_topic_recorder_spin_before_start_raises() -> None:
    recorder = TopicRecorder(
        topic="/test/spin",
        msg_type=object,
        serialiser=lambda msg, idx: msg,
    )
    with pytest.raises(RuntimeError, match="start"):
        recorder.spin()


def test_topic_recorder_multiple_serialiser_types() -> None:
    # Verify the generic serialiser can produce different output shapes.
    calls: list[tuple[Any, int]] = []

    def custom_serialiser(msg: object, idx: int) -> list[int]:
        calls.append((msg, idx))
        return [idx, idx * 2]

    recorder = TopicRecorder(
        topic="/test/custom",
        msg_type=object,
        serialiser=custom_serialiser,
    )
    recorder._recording = True
    recorder._on_message("a")
    recorder._on_message("b")

    assert recorder._frames[0] == [0, 0]
    assert recorder._frames[1] == [1, 2]
    assert len(calls) == 2


def test_topic_recorder_high_frame_count(tmp_path: Path) -> None:
    # Stress test: many frames, verify save/load fidelity.
    recorder = TopicRecorder(
        topic="/test/bulk",
        msg_type=object,
        serialiser=lambda msg, idx: idx,
    )
    recorder._recording = True
    for i in range(1000):
        recorder._on_message(i)

    assert recorder.frame_count == 1000

    out_path = tmp_path / "bulk.pkl"
    recorder.save_to(out_path)

    with out_path.open("rb") as fh:
        loaded = pickle.load(fh)  # noqa: S301
    assert len(loaded) == 1000
    assert loaded[999] == 999


def test_topic_recorder_save_creates_parent_dirs(tmp_path: Path) -> None:
    recorder = TopicRecorder(
        topic="/test/nested",
        msg_type=object,
        serialiser=lambda msg, idx: msg,
    )
    recorder._recording = True
    recorder._on_message("data")

    nested = tmp_path / "a" / "b" / "c" / "output.pkl"
    recorder.save_to(nested)
    assert nested.exists()
