"""System tests for segmentation recording: a real simulator, a real mp4, decoded and inspected.

Unit tests drive the recorder with a fake annotator, which proves the accumulation, the channel order and
the rate maths but says nothing about whether Replicator actually hands over frames. That gap is exactly
the one this project keeps falling into, and it hid two real defects here: attaching the annotator from the
control-plane thread hung the call outright, and `Sim.launch(overrides={"sim.scene": ...})` silently loaded
the shipped scene instead of the one asked for.

The strong assertion is on the decoded file: frame count, resolution, that the frames are not all identical,
and that a segmented view contains many distinct regions rather than one flat colour.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from tests.system.conftest import settle

# The scene's reference point, and an altitude above the terrain.
REFERENCE_LAT = 32.22481
REFERENCE_LON = 35.25621
RECORDING_ALTITUDE_M = 1200.0

# Enough frames to encode, with room for the annotator's warm-up.
RECORDING_STEPS = 8
FRAMES_PER_STEP = 10

# A segmented view of real geometry has many regions; one flat colour means nothing was segmented.
MIN_DISTINCT_COLOURS = 3


def _record(session: Any, name: str) -> dict[str, Any]:
    """Record a short segmentation clip while the camera moves, and return what was written.

    Args:
        session: The devkit session.
        name: File name under the output root.

    Returns:
        The report from ``stop_segmentation_recording``.

    """
    session.set_pose(
        lat_deg=REFERENCE_LAT,
        lon_deg=REFERENCE_LON,
        alt_m=RECORDING_ALTITUDE_M,
        pitch_deg=-30.0,
    )
    settle(session)
    session.start_segmentation_recording()
    for index in range(RECORDING_STEPS):
        # Move between frames so the clip is not the same picture repeated, which would still encode.
        session.set_pose(
            lat_deg=REFERENCE_LAT,
            lon_deg=REFERENCE_LON,
            alt_m=RECORDING_ALTITUDE_M,
            pitch_deg=-30.0,
            yaw_deg=index * 5.0,
        )
        session.step(count=FRAMES_PER_STEP)
    written: dict[str, Any] = session.stop_segmentation_recording(name)
    return written


def test_a_recording_writes_an_mp4_that_reports_what_it_contains(segmentation_session: Any) -> None:
    written = _record(segmentation_session, "system_segmentation.mp4")

    path = Path(str(written["path"]))
    assert path.is_file(), f"nothing was written to {path}"
    assert path.stat().st_size > 1000, "the file is too small to be a video"
    assert written["frames"] >= 2, f"only {written['frames']} frame(s) captured"
    assert written["width"] > 0
    assert written["height"] > 0
    assert written["fps"] > 0.0, "a zero frame rate produces a file no player can time"


def test_the_recording_is_confined_to_the_output_root(segmentation_session: Any) -> None:
    # The same confinement capture_frame has: a relative name must land under the configured root.
    written = _record(segmentation_session, "confined_segmentation.mp4")
    root = Path(str(segmentation_session.config.get()["sim"]["control_plane"]["output_root"])).resolve()
    assert Path(str(written["path"])).resolve().is_relative_to(root)


def test_escaping_the_output_root_is_refused(segmentation_session: Any) -> None:
    segmentation_session.start_segmentation_recording()
    with pytest.raises(Exception, match="output_root|confin|escape|outside"):
        segmentation_session.stop_segmentation_recording("../escaped.mp4")


def test_the_timestamps_sidecar_matches_the_frame_count(segmentation_session: Any) -> None:
    # A constant-rate container cannot express a variable frame rate, so the real times ship beside it.
    written = _record(segmentation_session, "sidecar_segmentation.mp4")
    sidecar = Path(str(written["timestamps_path"]))
    assert sidecar.is_file()
    lines = [line for line in sidecar.read_text(encoding="utf-8").splitlines() if line.strip()]
    assert len(lines) == written["frames"]
    times = [float(value) for value in lines]
    assert times[0] == pytest.approx(0.0)
    assert times == sorted(times), "presentation times must not go backwards"


def test_the_decoded_video_holds_moving_segmented_frames(segmentation_session: Any) -> None:
    # The assertion that matters: decode the file and look at it numerically. A recorder that wrote a
    # single flat colour, or the same frame repeatedly, passes everything above and is useless.
    cv2 = pytest.importorskip("cv2")
    import numpy as np

    written = _record(segmentation_session, "decoded_segmentation.mp4")
    capture = cv2.VideoCapture(str(written["path"]))
    try:
        assert capture.isOpened(), "the written file cannot be decoded"
        frames = []
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            frames.append(frame)
    finally:
        capture.release()

    assert len(frames) == written["frames"], f"decoded {len(frames)}, reported {written['frames']}"
    assert frames[0].shape[0] == written["height"]
    assert frames[0].shape[1] == written["width"]

    distinct = len(np.unique(frames[0].reshape(-1, 3), axis=0))
    assert distinct >= MIN_DISTINCT_COLOURS, f"only {distinct} colour(s); nothing appears segmented"

    difference = int(np.abs(frames[0].astype(int) - frames[-1].astype(int)).sum())
    assert difference > 0, "every frame is identical, so the recording did not follow the camera"


def test_stopping_without_starting_is_an_error(segmentation_session: Any) -> None:
    with pytest.raises(Exception, match="not running|start_segmentation_recording"):
        segmentation_session.stop_segmentation_recording("never_started.mp4")


def test_segmentation_reports_itself_as_a_composed_capability(segmentation_session: Any) -> None:
    capabilities = segmentation_session.get_capabilities()
    assert "segmentation" in capabilities["enabled"], capabilities
    assert "segmentation" not in [
        entry.get("id") for entry in capabilities.get("skipped", []) if isinstance(entry, dict)
    ]
