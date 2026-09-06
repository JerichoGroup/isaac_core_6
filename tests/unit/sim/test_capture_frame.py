"""Tests for the capture_frame control method's parameter handling."""

from pathlib import Path

import pytest

from isaac_core.control import InvalidParamsError
from isaac_core.sim.runtime import _optional_positive_int


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
