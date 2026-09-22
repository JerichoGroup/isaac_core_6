"""The segmentation recorder, driven by a fake annotator so no Isaac is needed.

The live behaviour these stand in for was measured on an authored house scene: 27 distinct colours with
walls, balcony slabs, roof, ground plane and posts all separated, using `instance_id_segmentation` with
`colorize` and nothing labelled.

Two traps are pinned here because both produce a file that looks written and plays as nothing: an odd
frame dimension, which mp4 cannot encode, and RGB written where the encoder expects BGR, which inverts
exactly the colours that distinguish one prim from another.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest

from isaac_core.sim.segmentation import (
    ANNOTATOR_NAME,
    MIN_FRAMES_FOR_VIDEO,
    SegmentationRecorder,
    measured_fps,
    presentation_times_s,
)


class FakeAnnotator:
    """Returns frames on demand, including the empty array an unwarmed annotator gives."""

    def __init__(self, frames: list[Any], *, wrap_in_dict: bool = True) -> None:
        """Store the frames to hand out, newest call taking the next one."""
        self.frames = list(frames)
        self.wrap_in_dict = wrap_in_dict
        self.reads = 0
        self.attached_to: list[str] = []

    def get_data(self) -> Any:
        """Return the next frame, repeating the last once exhausted."""
        self.reads += 1
        frame = self.frames[min(self.reads - 1, len(self.frames) - 1)]
        return {"data": frame} if self.wrap_in_dict else frame


def _frame(value: int, *, width: int = 8, height: int = 6, channels: int = 4) -> np.ndarray:
    """Build a frame with two regions, the way a real segmented view has.

    Deliberately not uniform: a single-valued frame is what an annotator returns when its buffer is not
    ready, and the recorder now rejects those, so a uniform helper would test a shape that never occurs.

    Args:
        value: Identifies this frame, so tests can tell one from another.
        width: Frame width in pixels.
        height: Frame height in pixels.
        channels: Channel count.

    Returns:
        A frame whose top half is ``value`` and bottom half is a second region.

    """
    frame = np.full((height, width, channels), value, dtype=np.uint8)
    # A second region, so the frame carries more than one value and reads as real content.
    frame[height // 2 :, :, :] = np.uint8((value + 90) % 256)
    return frame


def _recorder(frames: list[Any], **kwargs: Any) -> tuple[SegmentationRecorder, FakeAnnotator]:
    """Return a recorder wired to a fake annotator, plus the fake."""
    fake = FakeAnnotator(frames, **kwargs)

    def factory(render_product: str) -> FakeAnnotator:
        fake.attached_to.append(render_product)
        return fake

    return SegmentationRecorder(annotator_factory=factory), fake


# -- attaching ------------------------------------------------------------------ #


def test_the_annotator_name_is_the_label_free_one() -> None:
    # The two label-based annotators would segment only prims carrying a SemanticsAPI, which is the whole
    # reason this feature does not use the ROS 2 camera helper.
    assert ANNOTATOR_NAME == "instance_id_segmentation"


def test_attach_uses_the_render_product_it_is_given() -> None:
    recorder, fake = _recorder([_frame(1)])
    recorder.attach("/Render/product_a")
    assert fake.attached_to == ["/Render/product_a"]


def test_attaching_twice_to_the_same_product_does_not_reattach() -> None:
    # Re-attaching per recording would pay the annotator's warm-up again and drop the opening frames.
    recorder, fake = _recorder([_frame(1)])
    recorder.attach("/Render/product_a")
    recorder.attach("/Render/product_a")
    assert len(fake.attached_to) == 1


def test_attaching_to_a_different_product_does_reattach() -> None:
    recorder, fake = _recorder([_frame(1)])
    recorder.attach("/Render/product_a")
    recorder.attach("/Render/product_b")
    assert fake.attached_to == ["/Render/product_a", "/Render/product_b"]


# -- capturing ------------------------------------------------------------------ #


def test_nothing_is_captured_before_start() -> None:
    recorder, _ = _recorder([_frame(1)])
    recorder.attach("/Render/p")
    assert recorder.capture() is False
    assert recorder.frames_captured == 0
    assert recorder.is_recording is False


def test_frames_accumulate_once_started() -> None:
    recorder, _ = _recorder([_frame(1), _frame(2), _frame(3)])
    recorder.attach("/Render/p")
    recorder.start()
    for _ in range(3):
        assert recorder.capture() is True
    assert recorder.frames_captured == 3
    assert recorder.is_recording is True


def test_an_unwarmed_annotator_is_skipped_not_recorded() -> None:
    # Replicator returns a zero-size array for the first frames after attaching. Recording that as a
    # frame would write a video whose opening frames are garbage.
    recorder, _ = _recorder([np.zeros((0, 0), dtype=np.uint8), _frame(5), _frame(6)])
    recorder.attach("/Render/p")
    recorder.start()
    assert recorder.capture() is False
    assert recorder.capture() is True
    assert recorder.frames_captured == 1


def test_a_frame_with_too_few_channels_is_skipped() -> None:
    recorder, _ = _recorder([np.zeros((4, 4), dtype=np.uint8), _frame(7)])
    recorder.attach("/Render/p")
    recorder.start()
    assert recorder.capture() is False
    assert recorder.capture() is True


def test_a_raw_array_works_as_well_as_a_dict() -> None:
    # Replicator has returned both shapes across versions, and a wrong guess here reads as "no frames".
    recorder, _ = _recorder([_frame(3)], wrap_in_dict=False)
    recorder.attach("/Render/p")
    recorder.start()
    assert recorder.capture() is True


def test_a_read_that_raises_does_not_stop_the_loop() -> None:
    class Exploding:
        def get_data(self) -> Any:
            raise RuntimeError("annotator exploded")

    recorder = SegmentationRecorder(annotator_factory=lambda _: Exploding())
    recorder.attach("/Render/p")
    recorder.start()
    # The simulation loop calls this every frame; an exception here would stop the simulator.
    assert recorder.capture() is False
    assert recorder.is_recording is True


# -- writing -------------------------------------------------------------------- #


def test_stop_without_start_says_what_to_call(tmp_path: Path) -> None:
    recorder, _ = _recorder([_frame(1)])
    with pytest.raises(RuntimeError, match="start_segmentation_recording"):
        recorder.stop(tmp_path / "out.mp4")


def test_too_few_frames_refuses_rather_than_writing_an_unplayable_file(tmp_path: Path) -> None:
    recorder, _ = _recorder([_frame(1)])
    recorder.attach("/Render/p")
    recorder.start()
    recorder.capture()
    with pytest.raises(RuntimeError, match="warmed up|at least"):
        recorder.stop(tmp_path / "out.mp4")
    assert not (tmp_path / "out.mp4").exists()


def test_a_recording_writes_a_playable_mp4_and_reports_what_it_did(tmp_path: Path) -> None:
    recorder, _ = _recorder([_frame(10), _frame(20), _frame(30), _frame(40)])
    recorder.attach("/Render/p")
    recorder.start()
    for index in range(4):
        recorder.capture(now_ns=index * 33_333_333)
    target = tmp_path / "seg.mp4"
    written = recorder.stop(target)

    assert Path(written["path"]) == target
    assert target.is_file()
    assert target.stat().st_size > 0, "an empty file is not a video"
    assert written["frames"] == 4
    assert written["width"] == 8
    assert written["height"] == 6
    assert written["fps"] == pytest.approx(30.0, abs=0.1)


def test_the_written_video_reads_back_with_the_frames_that_went_in(tmp_path: Path) -> None:
    # The strongest available check without eyes: decode the file and confirm the frame count and size.
    cv2 = pytest.importorskip("cv2")
    recorder, _ = _recorder([_frame(v) for v in (10, 60, 110, 160, 210)])
    recorder.attach("/Render/p")
    recorder.start()
    for index in range(5):
        recorder.capture(now_ns=index * 20_000_000)
    target = tmp_path / "seg.mp4"
    recorder.stop(target)

    capture = cv2.VideoCapture(str(target))
    try:
        assert capture.isOpened(), "the writer produced a file no decoder will open"
        decoded = []
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            decoded.append(frame)
    finally:
        capture.release()
    assert len(decoded) == 5
    assert decoded[0].shape == (6, 8, 3)


def test_colours_survive_the_round_trip_rather_than_being_channel_swapped(tmp_path: Path) -> None:
    # OpenCV writes BGR. Handing it RGB unchanged inverts precisely the colours that tell one prim from
    # another, and the file still plays, so nothing would notice.
    cv2 = pytest.importorskip("cv2")
    red = np.zeros((6, 8, 4), dtype=np.uint8)
    red[:, :, 0] = 255  # pure red in RGB
    recorder, _ = _recorder([red, red, red])
    recorder.attach("/Render/p")
    recorder.start()
    for index in range(3):
        recorder.capture(now_ns=index * 33_000_000)
    target = tmp_path / "red.mp4"
    recorder.stop(target)

    capture = cv2.VideoCapture(str(target))
    try:
        ok, frame = capture.read()
    finally:
        capture.release()
    assert ok
    blue, green, red_channel = (int(frame[3, 4, i]) for i in range(3))
    assert red_channel > 200, f"red did not survive; got BGR=({blue},{green},{red_channel})"
    assert blue < 60, "red arrived in the blue channel, so RGB was written as BGR"


def test_an_odd_sized_frame_is_cropped_to_something_mp4_can_hold(tmp_path: Path) -> None:
    # An odd dimension yields a file that opens in nothing, and the writer reports no error.
    recorder, _ = _recorder([_frame(9, width=9, height=7) for _ in range(3)])
    recorder.attach("/Render/p")
    recorder.start()
    for index in range(3):
        recorder.capture(now_ns=index * 33_000_000)
    written = recorder.stop(tmp_path / "odd.mp4")
    assert written["width"] % 2 == 0
    assert written["height"] % 2 == 0


def test_the_timestamps_sidecar_is_written_beside_the_video(tmp_path: Path) -> None:
    recorder, _ = _recorder([_frame(1), _frame(2), _frame(3)])
    recorder.attach("/Render/p")
    recorder.start()
    for index in range(3):
        recorder.capture(now_ns=index * 50_000_000)
    target = tmp_path / "seg.mp4"
    written = recorder.stop(target)

    sidecar = Path(written["timestamps_path"])
    assert sidecar.name == "seg.mp4.timestamps.txt"
    lines = sidecar.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 3
    assert [float(v) for v in lines] == pytest.approx([0.0, 0.05, 0.10], abs=1e-6)


def test_stopping_ends_the_recording(tmp_path: Path) -> None:
    recorder, _ = _recorder([_frame(1), _frame(2)])
    recorder.attach("/Render/p")
    recorder.start()
    recorder.capture(now_ns=0)
    recorder.capture(now_ns=33_000_000)
    recorder.stop(tmp_path / "a.mp4")
    assert recorder.is_recording is False
    assert recorder.capture() is False


def test_a_second_recording_does_not_inherit_the_first_frames(tmp_path: Path) -> None:
    recorder, _ = _recorder([_frame(v) for v in range(1, 9)])
    recorder.attach("/Render/p")
    recorder.start()
    for index in range(4):
        recorder.capture(now_ns=index * 33_000_000)
    recorder.stop(tmp_path / "first.mp4")

    recorder.start()
    for index in range(2):
        recorder.capture(now_ns=index * 33_000_000)
    written = recorder.stop(tmp_path / "second.mp4")
    assert written["frames"] == 2


def test_cancel_discards_without_writing(tmp_path: Path) -> None:
    recorder, _ = _recorder([_frame(1), _frame(2)])
    recorder.attach("/Render/p")
    recorder.start()
    recorder.capture()
    recorder.cancel()
    assert recorder.is_recording is False
    assert not list(tmp_path.iterdir())


def test_dropped_frames_are_reported_not_hidden(tmp_path: Path) -> None:
    empty = np.zeros((0, 0), dtype=np.uint8)
    recorder, _ = _recorder([empty, empty, _frame(1), _frame(2), _frame(3)])
    recorder.attach("/Render/p")
    recorder.start()
    for index in range(5):
        recorder.capture(now_ns=index * 33_000_000)
    written = recorder.stop(tmp_path / "seg.mp4")
    assert written["dropped"] == 2
    assert written["frames"] == 3


# -- rate maths ----------------------------------------------------------------- #


def test_the_measured_rate_comes_from_the_stamps_not_a_nominal_value() -> None:
    # Terrain streaming alone takes the simulator from 7 to 60 fps, so a nominal rate would misreport the
    # duration of most recordings.
    assert measured_fps([0, 100_000_000, 200_000_000]) == pytest.approx(10.0)
    assert measured_fps([0, 33_333_333, 66_666_666]) == pytest.approx(30.0, abs=0.01)


def test_a_rate_that_cannot_be_measured_falls_back(f: None = None) -> None:
    del f
    assert measured_fps([]) == 30.0
    assert measured_fps([5]) == 30.0
    assert measured_fps([7, 7]) == 30.0
    assert measured_fps([1, 1], fallback=12.0) == 12.0


def test_presentation_times_start_at_zero() -> None:
    assert presentation_times_s([1_000_000_000, 1_500_000_000]) == pytest.approx([0.0, 0.5])
    assert presentation_times_s([]) == []


def test_the_minimum_frame_count_is_more_than_one() -> None:
    assert MIN_FRAMES_FOR_VIDEO >= 2


# -- blank frames, which used to be recorded ------------------------------------ #


def test_an_all_zero_frame_is_skipped_not_recorded(tmp_path: Path) -> None:
    # The bug: an annotator whose buffer is not ready returns the right shape and size filled with zeros.
    # That passed every structural check and encoded as solid black. Measured on a 60-frame sweep, frames
    # 50 to 58 were black and the recording still reported them as captured.
    blank = np.zeros((6, 8, 4), dtype=np.uint8)
    recorder, _ = _recorder([blank, blank, _frame(10), _frame(60), _frame(110)])
    recorder.attach("/Render/p")
    recorder.start()
    for index in range(5):
        recorder.capture(now_ns=index * 33_000_000)
    written = recorder.stop(tmp_path / "seg.mp4")
    assert written["frames"] == 3, "a blank frame was recorded"
    assert written["dropped"] == 2, "blank frames were not reported as dropped"


def test_a_uniform_nonzero_frame_is_also_skipped(tmp_path: Path) -> None:
    # Real geometry always yields more than one value, because even unlabelled background is a colour.
    # A frame of one value is an unready buffer whatever that value is.
    flat = np.full((6, 8, 4), 200, dtype=np.uint8)
    recorder, _ = _recorder([flat, _frame(10), _frame(60), _frame(110)])
    recorder.attach("/Render/p")
    recorder.start()
    for index in range(4):
        recorder.capture(now_ns=index * 33_000_000)
    written = recorder.stop(tmp_path / "seg.mp4")
    assert written["frames"] == 3
    assert written["dropped"] == 1


def test_a_recording_that_goes_entirely_blank_refuses_rather_than_writing_black(tmp_path: Path) -> None:
    # If every frame is blank there is nothing to write, and a file of black frames is worse than an error
    # because it looks like a successful recording.
    blank = np.zeros((6, 8, 4), dtype=np.uint8)
    recorder, _ = _recorder([blank] * 6)
    recorder.attach("/Render/p")
    recorder.start()
    for index in range(6):
        recorder.capture(now_ns=index * 33_000_000)
    with pytest.raises(RuntimeError, match="warmed up|at least"):
        recorder.stop(tmp_path / "seg.mp4")
    assert not (tmp_path / "seg.mp4").exists()
