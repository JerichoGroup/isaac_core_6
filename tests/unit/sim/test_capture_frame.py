"""Tests for the capture_frame control method's parameter handling."""

import inspect
from pathlib import Path

import pytest

from isaac_core.control import InvalidParamsError
from isaac_core.sim.runtime import (
    CAPTURE_RESIZE_SETTLE_FRAMES,
    CAPTURE_SETTLE_FRAMES,
    RENDER_PRODUCT_NODE_NAME,
    SimulationRuntime,
    _optional_positive_int,
)


@pytest.mark.parametrize("value", [0, -1, -1920])
def test_non_positive_dimensions_are_rejected(value: int) -> None:
    # A zero or negative resolution would be accepted by the viewport and produce either a
    # crash or a garbage image, so it is rejected before reaching Isaac.
    with pytest.raises(InvalidParamsError, match="greater than zero"):
        _optional_positive_int({"width": value}, "width")


@pytest.mark.parametrize("value", ["wide", None, [1920]])
def test_non_integer_dimensions_are_rejected_or_absent(value: object) -> None:
    """Reject a non-integer dimension, treating an explicit None as absent."""
    if value is None:
        assert _optional_positive_int({"width": value}, "width") is None
        return
    with pytest.raises(InvalidParamsError, match="must be an integer"):
        _optional_positive_int({"width": value}, "width")


def test_absent_dimension_is_none() -> None:
    """Return None when the caller omits the dimension entirely."""
    assert _optional_positive_int({}, "width") is None


def test_devkit_capture_frame_omits_unset_dimensions() -> None:
    # The handler rejects one dimension without the other, so the devkit must not send a
    # half-populated pair just because the user passed only a path.
    from isaac_core.devkit.session import SimSession

    sent: list[tuple[str, object]] = []

    class _FakeClient:
        def call(self, method: str, params: object = None) -> dict[str, object]:
            sent.append((method, params))
            return {}

    session = SimSession.__new__(SimSession)
    session._client = _FakeClient()  # type: ignore[assignment]
    session.capture_frame(Path("shot.png"))
    assert sent[-1][1] == {"path": "shot.png"}

    session.capture_frame("shot.png", width=3840, height=2160)
    assert sent[-1][1] == {"path": "shot.png", "width": 3840, "height": 2160}


def test_render_product_node_name_exists_in_every_camera_layer() -> None:
    # Guards a real regression: the camera layers were switched from
    # isaac_get_viewport_render_product to isaac_create_render_product, but the runtime kept
    # looking up the old name. og.Controller.node() then raised, the lookup returned None, and
    # capture_frame(width, height) silently fell back to viewport resolution instead of failing.
    layers_root = Path(__file__).resolve().parents[3] / "src" / "isaac_core" / "assets" / "layers"
    for layer in ("camera_udp", "camera_ros"):
        usda = layers_root / layer / f"{layer}.usda"
        assert usda.is_file(), f"missing shipped layer {usda}"
        text = usda.read_text(encoding="utf-8")
        assert (
            f'def OmniGraphNode "{RENDER_PRODUCT_NODE_NAME}"' in text
        ), f"{layer} has no node named {RENDER_PRODUCT_NODE_NAME}; runtime capture would return None"


def test_capture_resolution_is_restored_on_the_error_path() -> None:
    # Guards a real leak: the render product is shared with the image topic and RTSP stream, and
    # capture resizes it. The restore only ran on the success path, so a failed capture left every
    # consumer at capture resolution until the next launch.
    source = Path(inspect.getfile(SimulationRuntime)).read_text(encoding="utf-8")
    marker = 'logger.exception("capture failed")'
    assert marker in source
    tail = source[source.index(marker) : source.index(marker) + 600]
    assert "_restore_capture_resolution" in tail, "the capture error path does not restore resolution"


def test_restore_capture_resolution_is_the_only_place_that_undoes_the_resize() -> None:
    # One implementation, used by both the success and error paths.
    source = Path(inspect.getfile(SimulationRuntime)).read_text(encoding="utf-8")
    assert source.count("def _restore_capture_resolution") == 1
    assert source.count("_restore_capture_resolution(request)") == 2


def test_a_resize_capture_waits_longer_than_a_plain_one() -> None:
    # The bug this guards took five wrong diagnoses. A viewport resize is ASYNCHRONOUS -- the widget
    # carries a `__resize_future` -- so reading the render product 8 frames later returned the old
    # size, and capture wrote the camera's resolution while reporting the requested one. Waiting
    # longer when a resize was asked for is the fix; equal counts would silently reintroduce it.
    assert CAPTURE_RESIZE_SETTLE_FRAMES > CAPTURE_SETTLE_FRAMES


def test_the_capture_result_reports_what_it_measured() -> None:
    # A resize that silently failed used to look like success, because the reported resolution was
    # read back from the render product. The result now carries the render product and both the
    # original and viewport sizes so a caller can see which step did not take.
    source = Path(inspect.getfile(SimulationRuntime)).read_text(encoding="utf-8")
    for key in ('"render_product"', '"original_width"', '"viewport_width"'):
        assert key in source, f"capture result no longer reports {key}"
