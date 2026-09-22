"""Enabling `bbox` on a scene with no target prims must say so.

The capability report says `bbox` is enabled, the topic appears, and every message on it is empty forever.
The reason was logged at debug level only, so the one place a user looks said the feature was fine. On a
scene of your own that is the likely case rather than the exception, because the targets root is a
convention rather than something every scene has -- found by running a newly authored house scene, which
has no `/World/bboxes`.

The OGN node did warn, but from `compute()`, so it fired on every playback tick: 4332 identical lines in
one run, which buried every other message in the log.
"""

from __future__ import annotations

import contextlib
import logging
from typing import Any

from isaac_core.sim.composer import BBOXES_ROOT, _warn_if_bbox_has_no_targets
from isaac_core.sim.manifest import LayerManifest
from isaac_core.sim.planner import FeaturePlan, PlannedLayer


@contextlib.contextmanager
def _captured() -> Any:
    """Capture the composer's log records.

    `caplog` does not see this project's loggers, so attach a handler to the named logger directly, the
    same way `test_tileset_url.py` does.

    Returns:
        A list that fills with the composer's records for the duration of the block.

    """
    records: list[logging.LogRecord] = []
    handler = logging.Handler()
    handler.emit = records.append  # type: ignore[assignment,method-assign]
    logger = logging.getLogger("isaac_core.sim.composer")
    previous = logger.level
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    try:
        yield records
    finally:
        logger.removeHandler(handler)
        logger.setLevel(previous)


def _text(records: list[logging.LogRecord]) -> str:
    """Return the rendered text of captured records."""
    return "\n".join(record.getMessage() for record in records)


def _plan_with(*layer_ids: str) -> FeaturePlan:
    """Return a plan whose enabled layers have the given ids.

    Args:
        *layer_ids: Layer ids to include.

    Returns:
        A feature plan.

    """
    return FeaturePlan(
        enabled=tuple(
            PlannedLayer(
                manifest=LayerManifest(id=layer_id, mount="/World/Environment/{instance}"),
                instance="drone_0",
            )
            for layer_id in layer_ids
        )
    )


def test_bbox_with_no_targets_warns_and_names_the_root() -> None:
    # The message has to be actionable: which prim, and what to put under it.
    with _captured() as records:
        _warn_if_bbox_has_no_targets(_plan_with("camera_udp", "bbox"), labelled=0)
    text = _text(records)
    assert BBOXES_ROOT in text, text
    assert "bbox is enabled" in text
    assert "empty" in text


def test_bbox_with_targets_says_nothing() -> None:
    # The shipped scene has targets, so a normal run must not gain a spurious warning.
    with _captured() as records:
        _warn_if_bbox_has_no_targets(_plan_with("camera_udp", "bbox"), labelled=2)
    assert _text(records) == ""


def test_no_warning_when_bbox_is_not_enabled() -> None:
    # Almost every run has no targets and does not care, because bbox is off.
    with _captured() as records:
        _warn_if_bbox_has_no_targets(_plan_with("camera_udp", "segmentation"), labelled=0)
    assert _text(records) == ""


def test_no_warning_for_an_empty_plan() -> None:
    with _captured() as records:
        _warn_if_bbox_has_no_targets(FeaturePlan(), labelled=0)
    assert _text(records) == ""


def test_the_warning_is_a_warning_not_an_info_line() -> None:
    # It has to survive a default log level, which is what made the previous debug message useless.
    with _captured() as records:
        _warn_if_bbox_has_no_targets(_plan_with("bbox"), labelled=0)
    assert [record.levelno for record in records] == [logging.WARNING]


def test_the_projector_node_warns_once_rather_than_per_frame() -> None:
    # 4332 identical lines in a single run buried everything else. The node's compute() runs per playback
    # tick, so an unchanging condition must be stated once.
    from pathlib import Path

    node = Path(__file__).resolve().parents[3] / (
        "extensions/isaac_core_ogn.sensors/isaac_core_ogn/sensors/nodes/OgnBboxProjector.py"
    )
    source = node.read_text(encoding="utf-8")
    assert "_warn_once" in source, "the node no longer deduplicates its warnings"
    # Every warning reachable from compute() must go through the deduplicating helper.
    compute_body = source[source.index("def compute(") :]
    assert (
        "carb.log_warn(" not in compute_body
    ), "a warning inside compute() bypasses _warn_once and will fire on every frame"


def _load_projector_node() -> Any:
    """Import the OGN node on host Python, stubbing the Isaac-only modules it needs.

    The node itself cannot run outside Isaac, but its warning helper is plain Python and is the part that
    caused the flood, so it is worth exercising rather than only asserted structurally.

    Returns:
        The imported module.

    """
    import importlib.util
    from pathlib import Path
    import sys
    import types

    calls: list[str] = []
    carb = types.ModuleType("carb")
    carb.log_warn = calls.append  # type: ignore[attr-defined]
    database = types.ModuleType("isaac_core_ogn.sensors.ogn.OgnBboxProjectorDatabase")
    database.OgnBboxProjectorDatabase = object  # type: ignore[attr-defined]
    package = types.ModuleType("isaac_core_ogn")
    package.__path__ = []
    sensors = types.ModuleType("isaac_core_ogn.sensors")
    sensors.__path__ = []
    ogn = types.ModuleType("isaac_core_ogn.sensors.ogn")
    ogn.__path__ = []

    saved = {name: sys.modules.get(name) for name in ("carb", "isaac_core_ogn")}
    sys.modules.update(
        {
            "carb": carb,
            "isaac_core_ogn": package,
            "isaac_core_ogn.sensors": sensors,
            "isaac_core_ogn.sensors.ogn": ogn,
            "isaac_core_ogn.sensors.ogn.OgnBboxProjectorDatabase": database,
        }
    )
    try:
        path = Path(__file__).resolve().parents[3] / (
            "extensions/isaac_core_ogn.sensors/isaac_core_ogn/sensors/nodes/OgnBboxProjector.py"
        )
        spec = importlib.util.spec_from_file_location("_probe_OgnBboxProjector", path)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        for name, previous in saved.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous
    module._probe_calls = calls  # type: ignore[attr-defined]
    return module


def test_the_helper_reports_a_repeated_condition_once() -> None:
    module = _load_projector_node()
    calls = module._probe_calls
    for _ in range(50):
        module._warn_once("the same problem")
    assert len(calls) == 1, f"a repeated condition produced {len(calls)} warnings"
    assert "suppressed" in calls[0], "the reader is not told further warnings are hidden"


def test_the_helper_still_reports_a_different_condition() -> None:
    # Deduplicating must not swallow a genuinely new problem.
    module = _load_projector_node()
    calls = module._probe_calls
    module._warn_once("problem one")
    module._warn_once("problem two")
    module._warn_once("problem one")
    assert len(calls) == 2, calls
