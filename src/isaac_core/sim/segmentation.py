"""Record the stage's instance segmentation to an mp4.

Replicator's `instance_id_segmentation` annotator keys off prims rather than semantic labels, so every
distinct prim in view gets its own colour with nothing to configure and nothing to label. Its
`colorize` option yields RGB directly, which is what makes a video possible without inventing a palette.

Measured on an authored scene: 27 distinct colours over a house, with walls, each balcony slab, the roof,
the ground plane and individual posts all separated. The two label-based annotators would have shown a
single background colour there, because nothing in that scene carries a `SemanticsAPI`.

Frames are timestamped as they arrive and the mp4 is written at the *measured* rate, because the
simulator's frame rate is not constant -- terrain streaming alone takes it from 7 to 60 fps -- and a
constant-rate container cannot express that. The same `.timestamps.txt` sidecar the topic recorder writes
is written here, so a variable-rate remux is possible later.

This runs inside Isaac: the annotator is only readable in-process, and the deliverable is a file rather
than a topic.

**mp4 is lossy, so treat the video as something to look at rather than something to sample.** Measured on
a recording of an authored house: the annotator produced 27 flat colours, and the decoded mp4 reports over
22,000 because compression stipples the edges of every region. Regions stay obvious to the eye, which is
what a visual check needs, but reading a pixel does not recover the exact instance colour. Extracting
masks would want a lossless container, which this deliberately does not do.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import importlib
import logging
from pathlib import Path
import time
from typing import Any

logger = logging.getLogger(__name__)

# The annotator that keys off prims instead of labels. `instance_segmentation` and
# `semantic_segmentation` both need a SemanticsAPI and would segment only what was labelled.
ANNOTATOR_NAME = "instance_id_segmentation"

# Replicator's capture graph does not run on a bare app update, so a recording that never sees a frame
# is a real possibility worth reporting rather than writing an empty file.
MIN_FRAMES_FOR_VIDEO = 2

# mp4 needs an even frame size; odd dimensions silently produce a file no player will open.
_SIZE_ALIGNMENT = 2

# A colorized frame is (height, width, channels) with at least RGB.
_FRAME_DIMENSIONS = 3
_MIN_CHANNELS = 3

# Two stamps are the fewest that can describe a rate.
_MIN_STAMPS_FOR_RATE = 2


@dataclass
class SegmentationRecording:
    """Frames captured for one recording, with the times they arrived."""

    frames: list[Any] = field(default_factory=list)
    stamps_ns: list[int] = field(default_factory=list)
    dropped: int = 0

    def add(self, frame: Any, stamp_ns: int) -> None:
        """Append a frame and the moment it was read.

        Args:
            frame: The colorized segmentation frame.
            stamp_ns: Arrival time in nanoseconds.

        """
        self.frames.append(frame)
        self.stamps_ns.append(stamp_ns)

    @property
    def count(self) -> int:
        """Return how many frames were captured."""
        return len(self.frames)


def measured_fps(stamps_ns: list[int], *, fallback: float = 30.0) -> float:
    """Return the frame rate the stamps actually describe.

    Args:
        stamps_ns: Arrival times in nanoseconds, in order.
        fallback: Rate to use when the stamps cannot describe one, such as a single frame or a
            zero-length span.

    Returns:
        Frames per second.

    """
    if len(stamps_ns) < _MIN_STAMPS_FOR_RATE:
        return fallback
    span_s = (stamps_ns[-1] - stamps_ns[0]) / 1e9
    if span_s <= 0.0:
        return fallback
    return (len(stamps_ns) - 1) / span_s


def presentation_times_s(stamps_ns: list[int]) -> list[float]:
    """Return per-frame presentation times in seconds, relative to the first frame.

    Args:
        stamps_ns: Arrival times in nanoseconds, in order.

    Returns:
        Seconds since the first frame, one per frame.

    """
    if not stamps_ns:
        return []
    first = stamps_ns[0]
    return [(stamp - first) / 1e9 for stamp in stamps_ns]


class SegmentationRecorder:
    """Attach the segmentation annotator to a render product and record it to an mp4.

    Holds no simulator state beyond the annotator handle, so it can be driven by a fake in tests and
    by Replicator in the simulator without branching on which it is.
    """

    def __init__(self, *, annotator_factory: Any = None) -> None:
        """Initialise the recorder.

        Args:
            annotator_factory: Called with the render product path to produce an object with
                ``get_data()``. Defaults to Replicator, which is only importable inside Isaac; tests
                pass a fake and so never need Isaac.

        """
        self._annotator_factory = annotator_factory or _replicator_annotator
        self._annotator: Any = None
        self._render_product: str | None = None
        self._recording: SegmentationRecording | None = None

    @property
    def is_recording(self) -> bool:
        """Return whether frames are currently being accumulated."""
        return self._recording is not None

    @property
    def frames_captured(self) -> int:
        """Return how many frames the current recording holds, or zero when not recording."""
        return self._recording.count if self._recording is not None else 0

    def attach(self, render_product: str) -> None:
        """Attach the annotator to a render product, if not already attached to it.

        Attaching is separate from recording so the first recorded frame does not pay the annotator's
        warm-up, which produces an empty array for the first few frames.

        Args:
            render_product: The render product path to read.

        """
        if self._annotator is not None and self._render_product == render_product:
            return
        self._annotator = self._annotator_factory(render_product)
        self._render_product = render_product
        logger.info("segmentation annotator attached to %s", render_product)

    def start(self) -> None:
        """Begin accumulating frames, discarding anything from a previous recording."""
        self._recording = SegmentationRecording()
        logger.info("segmentation recording started")

    def capture(self, *, now_ns: int | None = None) -> bool:
        """Read one frame if recording, and keep it.

        Called once per simulation frame. A frame the annotator is not ready to give is skipped rather
        than waited for, because blocking here would stall the simulation loop.

        Args:
            now_ns: Arrival time override, for tests.

        Returns:
            Whether a frame was kept.

        """
        if self._recording is None or self._annotator is None:
            return False
        frame = self._read_frame()
        if frame is None:
            self._recording.dropped += 1
            return False
        self._recording.add(frame, time.monotonic_ns() if now_ns is None else now_ns)
        return True

    def _read_frame(self) -> Any:
        """Return the annotator's current frame, or ``None`` when it has nothing usable.

        Returns:
            An RGB(A) array, or ``None``.

        """
        try:
            data = self._annotator.get_data()
        except Exception:  # a failed read must not stop the simulation loop
            logger.warning("segmentation annotator read failed", exc_info=True)
            return None
        array = data.get("data") if isinstance(data, dict) else data
        if array is None:
            return None
        # An unwarmed annotator returns a zero-size array, which is not an error and not a frame.
        size = getattr(array, "size", None)
        if size == 0:
            return None
        shape: tuple[int, ...] = tuple(getattr(array, "shape", ()))
        if len(shape) != _FRAME_DIMENSIONS or shape[-1] < _MIN_CHANNELS:
            return None
        if _is_blank(array):
            return None
        return array

    def stop(self, path: Path) -> dict[str, Any]:
        """Stop recording and write the mp4.

        Args:
            path: Where to write. The caller confines this under ``output_root``.

        Returns:
            What was written: path, frame count, measured fps, resolution and the sidecar path.

        Raises:
            RuntimeError: If no recording is running, or too few frames were captured to encode.

        """
        if self._recording is None:
            raise RuntimeError("segmentation recording is not running; call start_segmentation_recording first")
        recording, self._recording = self._recording, None
        if recording.count < MIN_FRAMES_FOR_VIDEO:
            raise RuntimeError(
                f"captured {recording.count} segmentation frame(s), need at least {MIN_FRAMES_FOR_VIDEO}; "
                "the annotator may not have warmed up -- record for longer"
            )
        fps = measured_fps(recording.stamps_ns)
        written, encoded_width, encoded_height = _encode_mp4(recording.frames, path, fps)
        sidecar = _write_timestamps(path, recording.stamps_ns)
        logger.info("segmentation recording written to %s (%d frames, %.2f fps)", written, recording.count, fps)
        return {
            "path": str(written),
            "frames": recording.count,
            "dropped": recording.dropped,
            "fps": round(fps, 3),
            # What was encoded, not what arrived: an odd dimension is cropped, and reporting the
            # uncropped size would describe a file that does not exist.
            "width": encoded_width,
            "height": encoded_height,
            "timestamps_path": str(sidecar),
        }

    def cancel(self) -> None:
        """Discard the current recording without writing anything."""
        self._recording = None


def _is_blank(array: Any) -> bool:
    """Return whether a frame carries no content at all.

    An annotator whose buffer is not ready returns an array of the right shape and size filled with
    zeros. That passes every structural check and encodes as a black frame, so a recording could contain
    them without anything noticing: measured on a 60-frame sweep, frames 50 to 58 were solid black and
    the recording reported 59 frames captured and one dropped.

    A frame of real geometry always contains more than one value, because even unlabelled background is
    its own colour. A uniform frame is therefore either an unready buffer or a view of literally nothing,
    and neither is an observation worth keeping.

    Args:
        array: The frame to check.

    Returns:
        Whether every pixel holds the same value.

    """
    try:
        import numpy as np

        view = np.asarray(array)
        if view.size == 0:
            return True
        return bool(view.max() == view.min())
    except Exception:  # a frame we cannot inspect is better skipped than written
        logger.warning("could not inspect a segmentation frame; skipping it", exc_info=True)
        return True


def _replicator_annotator(render_product: str) -> Any:
    """Return a colorized instance-id segmentation annotator attached to a render product.

    Args:
        render_product: The render product path.

    Returns:
        The attached annotator.

    """
    rep = importlib.import_module("omni.replicator.core")
    annotator = rep.AnnotatorRegistry.get_annotator(ANNOTATOR_NAME, init_params={"colorize": True})
    annotator.attach([render_product])
    return annotator


def _encode_mp4(frames: list[Any], path: Path, fps: float) -> tuple[Path, int, int]:
    """Write frames to an mp4 at a given rate.

    Args:
        frames: RGB(A) arrays, all the same shape.
        path: Destination path.
        fps: Frame rate to write.

    Returns:
        The path written and the width and height actually encoded.

    Raises:
        RuntimeError: If OpenCV is unavailable or the writer refuses to open.

    """
    try:
        cv2 = importlib.import_module("cv2")
    except ImportError as exc:  # pragma: no cover - present in Isaac's Python
        raise RuntimeError("writing an mp4 needs opencv; install 'isaac-core[devkit]'") from exc
    import numpy as np

    height, width = int(frames[0].shape[0]), int(frames[0].shape[1])
    # An odd dimension produces a file that opens in nothing, with no error from the writer.
    width -= width % _SIZE_ALIGNMENT
    height -= height % _SIZE_ALIGNMENT
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"could not open an mp4 writer for {path}")
    try:
        for frame in frames:
            array = np.asarray(frame)[:height, :width, :3]
            # Replicator gives RGB; OpenCV writes BGR, and swapping them would invert the colours that
            # make one prim distinguishable from another.
            writer.write(np.ascontiguousarray(array[:, :, ::-1]).astype("uint8"))
    finally:
        writer.release()
    return path, width, height


def _write_timestamps(video_path: Path, stamps_ns: list[int]) -> Path:
    """Write per-frame presentation times beside the video.

    Args:
        video_path: The mp4 that was written.
        stamps_ns: Arrival times in nanoseconds.

    Returns:
        The sidecar path.

    """
    sidecar = video_path.with_suffix(video_path.suffix + ".timestamps.txt")
    sidecar.write_text("\n".join(f"{t:.9f}" for t in presentation_times_s(stamps_ns)) + "\n", encoding="utf-8")
    return sidecar
