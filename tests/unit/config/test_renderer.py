"""Tests for the ``sim.renderer`` validation and its no-terrain warning."""

import logging

from pydantic import ValidationError
import pytest

from isaac_core.config import IsaacCoreConfig

# The named logger the renderer warning is emitted through. pytest's ``caplog`` does not
# capture this project's warnings, so tests attach a handler to this logger directly.
_RENDERER_LOGGER = "isaac_core.config.schema"


class _Capture(logging.Handler):
    """Collects emitted records so a test can read what the user would see."""

    def __init__(self) -> None:
        """Start with an empty record buffer."""
        super().__init__()
        self.records: list[logging.LogRecord] = []

    def emit(self, record: logging.LogRecord) -> None:
        """Store one record."""
        self.records.append(record)


def _messages_for_renderer(renderer: str) -> list[str]:
    """Build a config with the given renderer and return the warnings it logged."""
    logger = logging.getLogger(_RENDERER_LOGGER)
    handler = _Capture()
    logger.addHandler(handler)
    try:
        IsaacCoreConfig(sim={"renderer": renderer})
    finally:
        logger.removeHandler(handler)
    return [record.getMessage() for record in handler.records]


def test_default_renderer_validates() -> None:
    assert IsaacCoreConfig().sim.renderer == "RaytracedLighting"


def test_default_renderer_produces_no_warning() -> None:
    assert _messages_for_renderer("RaytracedLighting") == []


@pytest.mark.parametrize(
    "renderer",
    ["RaytracedLighting", "PathTracing", "RealTimePathTracing", "MinimalRendering", "Minimal"],
)
def test_accepted_renderer_validates(renderer: str) -> None:
    assert IsaacCoreConfig(sim={"renderer": renderer}).sim.renderer == renderer


@pytest.mark.parametrize("renderer", ["raytracedlighting", "MINIMAL", "PathTRACING"])
def test_matching_is_case_insensitive(renderer: str) -> None:
    # Isaac lower-cases the value before matching, so this validator is case-insensitive
    # too. The stored value is preserved verbatim rather than canonicalised.
    assert IsaacCoreConfig(sim={"renderer": renderer}).sim.renderer == renderer


@pytest.mark.parametrize("renderer", ["Raytraced", "pathtracing ", "", "nonsense"])
def test_unknown_renderer_is_rejected(renderer: str) -> None:
    with pytest.raises(ValidationError) as excinfo:
        IsaacCoreConfig(sim={"renderer": renderer})
    message = str(excinfo.value)
    assert "RaytracedLighting" in message
    assert "PathTracing" in message
    assert "MinimalRendering" in message


@pytest.mark.parametrize("renderer", ["MinimalRendering", "Minimal", "minimal"])
def test_minimal_rendering_validates_but_warns_about_terrain(renderer: str) -> None:
    messages = _messages_for_renderer(renderer)
    assert len(messages) == 1
    assert "terrain" in messages[0].lower()
