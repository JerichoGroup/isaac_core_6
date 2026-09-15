"""Tests for isaac_core.devkit.recording: generic TopicRecorder."""

from pathlib import Path
import pickle
import sys
from typing import Any

import numpy as np
import pytest

from isaac_core.devkit.recording import (
    TopicRecorder,
    measured_fps,
    presentation_times_s,
    video_recorder,
)

# -- import safety ---------------------------------------------------------- #


def test_import_recording_succeeds_without_rclpy() -> None:
    # Block rclpy via sys.modules to prove import succeeds without it.
    # This is the critical property that keeps CI working.
    sentinel = sys.modules.get("rclpy")
    sys.modules["rclpy"] = None  # type: ignore[assignment]
    try:
        # Force reimport.
        import importlib

        import isaac_core.devkit.recording

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
        loaded = pickle.load(fh)

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
        loaded = pickle.load(fh)
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


# -- timing derivation (D22: derive the rate, never ask the user) ----------- #

# Nanoseconds per second, local to the tests.
_NS = 1_000_000_000


def test_measured_fps_from_even_30hz_stamps() -> None:
    stamps = [i * _NS // 30 for i in range(10)]
    assert measured_fps(stamps) == pytest.approx(30.0, rel=1e-6)


def test_measured_fps_ignores_a_dropped_frame_gap() -> None:
    # A single large gap (a dropped frame) must not drag the rate down: the median
    # inter-frame interval is robust to it, unlike a naive mean.
    step = _NS // 50
    stamps = [0, step, 2 * step, 2 * step + 10 * step, 2 * step + 11 * step]
    assert measured_fps(stamps) == pytest.approx(50.0, rel=1e-6)


def test_measured_fps_zero_frames_falls_back() -> None:
    assert measured_fps([]) == 1.0


def test_measured_fps_single_frame_falls_back() -> None:
    # One frame has no meaningful rate; must not divide by zero.
    assert measured_fps([12345]) == 1.0


def test_measured_fps_duplicate_stamps_fall_back() -> None:
    # All-identical timestamps yield only zero deltas -> no positive interval exists.
    assert measured_fps([5, 5, 5, 5]) == 1.0


def test_measured_fps_non_monotonic_uses_positive_deltas_only() -> None:
    # A backwards jump contributes a negative delta which is discarded; the remaining
    # positive intervals still give the right rate.
    step = _NS // 25
    stamps = [0, step, step - 1000, step, 2 * step]
    assert measured_fps(stamps) == pytest.approx(25.0, rel=0.5)


def test_presentation_times_start_at_zero_and_advance() -> None:
    step = _NS // 10
    times = presentation_times_s([100, 100 + step, 100 + 2 * step])
    assert times[0] == 0.0
    assert times[1] == pytest.approx(0.1, rel=1e-6)
    assert times[2] == pytest.approx(0.2, rel=1e-6)


def test_presentation_times_clamp_backwards_jumps() -> None:
    # Non-monotonic input must never make playback time rewind.
    times = presentation_times_s([0, _NS, _NS // 2, 2 * _NS])
    assert times == sorted(times)


def test_presentation_times_empty() -> None:
    assert presentation_times_s([]) == []


# -- save_video (real cv2 round-trip; cv2 is importable here) ---------------- #


def _frame(index: int, stamp_ns: int, *, h: int = 8, w: int = 8) -> dict[str, Any]:
    """Build a recorded-frame dict shaped like the image serialiser's output."""
    return {"frame": index, "stamp_ns": stamp_ns, "image": np.zeros((h, w, 3), dtype=np.uint8)}


def _load_recorder_with_frames(frames: list[dict[str, Any]]) -> TopicRecorder[Any]:
    """Return a video recorder pre-loaded with the given frame dicts."""
    rec = video_recorder()
    rec._frames = {f["frame"]: f for f in frames}
    return rec


def test_save_video_writes_mp4_and_sidecar_at_measured_rate(tmp_path: Path) -> None:
    step = _NS // 30
    frames = [_frame(i, i * step) for i in range(30)]
    rec = _load_recorder_with_frames(frames)

    out = tmp_path / "clip.mp4"
    result = rec.save_video(out)

    assert result == out
    assert out.exists() and out.stat().st_size > 0
    sidecar = tmp_path / "clip.mp4.timestamps.txt"
    assert sidecar.exists()
    lines = sidecar.read_text().strip().splitlines()
    assert len(lines) == 30
    assert float(lines[0]) == 0.0
    assert float(lines[-1]) == pytest.approx(29 / 30, rel=1e-3)


def test_save_video_zero_frames_raises(tmp_path: Path) -> None:
    rec = video_recorder()
    with pytest.raises(RuntimeError, match="no frames"):
        rec.save_video(tmp_path / "empty.mp4")


def test_save_video_single_frame_does_not_crash(tmp_path: Path) -> None:
    # One frame: rate cannot be measured, falls back to a valid writer rate.
    rec = _load_recorder_with_frames([_frame(0, 999)])
    out = rec.save_video(tmp_path / "one.mp4")
    assert out.exists()


def test_save_video_duplicate_timestamps_does_not_crash(tmp_path: Path) -> None:
    frames = [_frame(i, 42) for i in range(5)]
    rec = _load_recorder_with_frames(frames)
    out = rec.save_video(tmp_path / "dup.mp4")
    assert out.exists()


def test_save_video_large_gap_does_not_crash(tmp_path: Path) -> None:
    # A dropped-frame gap in the middle must not crash and must be preserved in the
    # sidecar presentation times.
    step = _NS // 40
    stamps = [0, step, 2 * step, 2 * step + 50 * step, 2 * step + 51 * step]
    frames = [_frame(i, s) for i, s in enumerate(stamps)]
    rec = _load_recorder_with_frames(frames)
    rec.save_video(tmp_path / "gap.mp4")
    sidecar = tmp_path / "gap.mp4.timestamps.txt"
    times = [float(x) for x in sidecar.read_text().strip().splitlines()]
    assert times == sorted(times)
    assert times[-1] > 1.0  # the gap is preserved as real elapsed time


def test_save_video_fps_override_is_used(tmp_path: Path) -> None:
    # An explicit override must actually take effect (not be silently ignored).
    frames = [_frame(i, i * (_NS // 30)) for i in range(10)]
    rec = _load_recorder_with_frames(frames)
    out = rec.save_video(tmp_path / "override.mp4", fps_override=12.0)
    import cv2

    cap = cv2.VideoCapture(str(out))
    try:
        assert cap.get(cv2.CAP_PROP_FPS) == pytest.approx(12.0, rel=1e-3)
    finally:
        cap.release()
