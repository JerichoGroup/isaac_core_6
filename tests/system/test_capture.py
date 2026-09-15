"""System tests for frame capture, which needs a viewport and therefore a GUI.

Every assertion here corresponds to something that broke in this project while the unit suite stayed
green. They are written to measure the observable outcome -- pixels on disk, topics on the wire, the
stage transform -- rather than that a call returned without raising.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from tests.system.conftest import settle

# Metres of tolerance when comparing a commanded altitude against the stage transform.
ALTITUDE_TOLERANCE_M = 1.0

# The scene's reference point, and an altitude comfortably above the terrain.
REFERENCE_LAT = 32.22481
REFERENCE_LON = 35.25621
TEST_ALTITUDE_M = 1500.0


def _reference_altitude(session: Any) -> float:
    """Return the ENU reference altitude the running simulator resolved."""
    return float(session.config.get()["geo"]["enu_reference"]["alt_m"])


def _png_size(path: Path) -> tuple[int, int]:
    """Return a PNG's pixel dimensions by reading its IHDR chunk.

    Deliberately not using an image library: the point is to measure what is on disk without
    depending on Pillow or OpenCV being installed.

    Args:
        path: The PNG file.

    Returns:
        ``(width, height)`` in pixels.

    """
    header = path.read_bytes()[:33]
    assert header[:8] == b"\x89PNG\r\n\x1a\n", f"{path} is not a PNG"
    width = int.from_bytes(header[16:20], "big")
    height = int.from_bytes(header[20:24], "big")
    return width, height


# -- frame capture ----------------------------------------------------------- #


def test_capture_at_the_camera_resolution_writes_that_many_pixels(gui_session: Any) -> None:
    gui_session.set_pose(lat_deg=REFERENCE_LAT, lon_deg=REFERENCE_LON, alt_m=TEST_ALTITUDE_M)
    settle(gui_session)
    result = gui_session.capture_frame("camera_res.png")
    written = Path(str(result["path"]))
    assert written.is_file(), f"capture reported {written} but nothing is there"
    assert _png_size(written) == (
        int(result["width"]),
        int(result["height"]),
    ), "the reported resolution does not match the pixels on disk"


@pytest.mark.parametrize(("width", "height"), [(640, 480), (1920, 1080), (3840, 2160)])
def test_capture_at_a_requested_resolution_writes_that_many_pixels(gui_session: Any, width: int, height: int) -> None:
    # The regression this exists for: capture silently wrote the camera's resolution while reporting
    # the requested one, so a caller asking for 4K received 720p with no way to notice. Measuring the
    # PNG header is the only assertion that could have caught it.
    gui_session.set_pose(lat_deg=REFERENCE_LAT, lon_deg=REFERENCE_LON, alt_m=TEST_ALTITUDE_M)
    settle(gui_session)
    result = gui_session.capture_frame(f"requested_{width}x{height}.png", width=width, height=height)
    written = Path(str(result["path"]))
    assert written.is_file(), f"capture reported {written} but nothing is there"
    assert _png_size(written) == (
        width,
        height,
    ), f"asked for {width}x{height}, the file on disk is {_png_size(written)}"


def test_capture_leaves_the_camera_resolution_unchanged(gui_session: Any) -> None:
    # A capture resizes the render product, which is shared with the image topic and the RTSP stream.
    # Failing to put it back would degrade every consumer until the next launch.
    before = gui_session.capture_frame("before.png")
    gui_session.capture_frame("big.png", width=1920, height=1080)
    settle(gui_session)
    after = gui_session.capture_frame("after.png")
    assert (after["width"], after["height"]) == (
        before["width"],
        before["height"],
    ), "a sized capture left the camera at a different resolution"
