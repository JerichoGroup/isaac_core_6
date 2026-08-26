"""Tests for the RTP video service module."""

from __future__ import annotations

import importlib
import sys
from typing import Any

import pytest

from isaac_core.config import SidecarServiceConfig

# -- caps_string pure function tests ---------------------------------------- #


def test_caps_string_rgb8() -> None:
    from isaac_core.sidecar.rtp import caps_string

    result = caps_string("rgb8", 1280, 720)
    assert result == "video/x-raw,format=RGB,width=1280,height=720,framerate=30/1"


def test_caps_string_bgr8() -> None:
    from isaac_core.sidecar.rtp import caps_string

    result = caps_string("bgr8", 640, 480)
    assert result == "video/x-raw,format=BGR,width=640,height=480,framerate=30/1"


def test_caps_string_mono8() -> None:
    from isaac_core.sidecar.rtp import caps_string

    result = caps_string("mono8", 320, 240, framerate=15)
    assert result == "video/x-raw,format=GRAY8,width=320,height=240,framerate=15/1"


def test_caps_string_rgba8() -> None:
    from isaac_core.sidecar.rtp import caps_string

    result = caps_string("rgba8", 1920, 1080, framerate=60)
    assert result == "video/x-raw,format=RGBA,width=1920,height=1080,framerate=60/1"


def test_caps_string_bgra8() -> None:
    from isaac_core.sidecar.rtp import caps_string

    result = caps_string("bgra8", 100, 100)
    assert result == "video/x-raw,format=BGRA,width=100,height=100,framerate=30/1"


def test_caps_string_unsupported_encoding_returns_none() -> None:
    from isaac_core.sidecar.rtp import caps_string

    assert caps_string("yuv422", 640, 480) is None


def test_caps_string_unknown_encoding_returns_none() -> None:
    from isaac_core.sidecar.rtp import caps_string

    assert caps_string("bayer_rggb8", 640, 480) is None


# -- module import succeeds without rclpy/gi -------------------------------- #


class _BlockingFinder:
    """Meta-path finder that blocks specific modules from being imported."""

    def __init__(self, blocked: frozenset[str]) -> None:
        self._blocked = blocked

    def find_module(self, fullname: str, path: Any = None) -> "_BlockingFinder | None":  # noqa: ANN401
        """Return self if the module should be blocked, else None."""
        for blocked in self._blocked:
            if fullname == blocked or fullname.startswith(blocked + "."):
                return self
        return None

    def load_module(self, fullname: str) -> None:
        """Raise ImportError for blocked modules."""
        msg = f"Blocked for testing: {fullname}"
        raise ImportError(msg)


def test_import_rtp_module_succeeds_without_rclpy_and_gi() -> None:
    # Block rclpy and gi at the meta_path level
    blocked = frozenset({"rclpy", "gi", "sensor_msgs"})
    finder = _BlockingFinder(blocked)
    sys.meta_path.insert(0, finder)  # type: ignore[arg-type]

    # Remove any cached imports of the module
    modules_to_remove = [k for k in sys.modules if k.startswith("isaac_core.sidecar")]
    for mod in modules_to_remove:
        del sys.modules[mod]

    try:
        # This must not raise
        import isaac_core.sidecar.rtp  # noqa: F401

        # And the module should be importable
        assert "isaac_core.sidecar.rtp" in sys.modules
    finally:
        sys.meta_path.remove(finder)  # type: ignore[arg-type]
        # Restore clean state
        modules_to_remove = [k for k in sys.modules if k.startswith("isaac_core.sidecar")]
        for mod in modules_to_remove:
            del sys.modules[mod]
        # Re-import cleanly
        importlib.import_module("isaac_core.sidecar.rtp")


# -- lazy import error messages --------------------------------------------- #


def test_require_rclpy_gives_actionable_error() -> None:
    blocked = frozenset({"rclpy"})
    finder = _BlockingFinder(blocked)
    sys.meta_path.insert(0, finder)  # type: ignore[arg-type]

    # Ensure rclpy is not cached
    rclpy_keys = [k for k in sys.modules if k.startswith("rclpy")]
    saved = {k: sys.modules.pop(k) for k in rclpy_keys}

    try:
        from isaac_core.sidecar.rtp import _require_rclpy

        with pytest.raises(ImportError, match="source /opt/ros/humble"):
            _require_rclpy()
    finally:
        sys.meta_path.remove(finder)  # type: ignore[arg-type]
        sys.modules.update(saved)


def test_require_gi_gives_actionable_error() -> None:
    blocked = frozenset({"gi"})
    finder = _BlockingFinder(blocked)
    sys.meta_path.insert(0, finder)  # type: ignore[arg-type]

    gi_keys = [k for k in sys.modules if k.startswith("gi")]
    saved = {k: sys.modules.pop(k) for k in gi_keys}

    try:
        from isaac_core.sidecar.rtp import _require_gst

        with pytest.raises(ImportError, match="apt install python3-gi"):
            _require_gst()
    finally:
        sys.meta_path.remove(finder)  # type: ignore[arg-type]
        sys.modules.update(saved)


# -- RtpVideoService instantiation ----------------------------------------- #


def test_rtp_service_requires_topic_in_config() -> None:
    from isaac_core.sidecar.rtp import RtpVideoService

    config = SidecarServiceConfig(kind="rtp")
    with pytest.raises(ValueError, match="topic"):
        RtpVideoService(config)


def test_rtp_service_name_reflects_config() -> None:
    from isaac_core.sidecar.rtp import RtpVideoService

    config = SidecarServiceConfig(kind="rtp", topic="/cam/image", host="10.0.0.1", video_port=6000)
    svc = RtpVideoService(config)
    assert "10.0.0.1" in svc.name
    assert "6000" in svc.name
    assert "/cam/image" in svc.name


def test_rtp_service_defaults() -> None:
    from isaac_core.sidecar.rtp import RtpVideoService

    config = SidecarServiceConfig(kind="rtp", topic="/img")
    svc = RtpVideoService(config)
    assert svc._host == "127.0.0.1"  # noqa: SLF001
    assert svc._video_port == 5004  # noqa: SLF001
    assert svc._framerate == 30  # noqa: SLF001
    assert svc._bitrate == 10000  # noqa: SLF001


def test_rtp_service_is_not_healthy_before_start() -> None:
    from isaac_core.sidecar.rtp import RtpVideoService

    config = SidecarServiceConfig(kind="rtp", topic="/img")
    svc = RtpVideoService(config)
    assert not svc.is_healthy()


# -- service registry entry ------------------------------------------------- #


def test_rtp_service_is_registered() -> None:
    from isaac_core.sidecar.service import ServiceRegistry

    factory = ServiceRegistry.get("rtp")
    assert factory is not None


# -- encoding map ----------------------------------------------------------- #


def test_encoding_map_has_expected_entries() -> None:
    from isaac_core.sidecar.rtp import ENCODING_TO_GST_FORMAT

    assert ENCODING_TO_GST_FORMAT["rgb8"] == "RGB"
    assert ENCODING_TO_GST_FORMAT["bgr8"] == "BGR"
    assert ENCODING_TO_GST_FORMAT["mono8"] == "GRAY8"
    assert ENCODING_TO_GST_FORMAT["rgba8"] == "RGBA"
    assert ENCODING_TO_GST_FORMAT["bgra8"] == "BGRA"
    # Only these five
    assert len(ENCODING_TO_GST_FORMAT) == 5
