"""Tests for isaac_core.sim.planner."""

from __future__ import annotations

import pytest

from isaac_core.sim.capabilities import StageCapabilities, StageCapability
from isaac_core.sim.manifest import LayerManifest
from isaac_core.sim.planner import FeaturePlan, PlanningError, plan_features

# -- Helpers ------------------------------------------------------------------


def _caps(*members: StageCapability) -> StageCapabilities:
    """Build a StageCapabilities with the given members."""
    return StageCapabilities(frozenset(members))


def _manifest(
    layer_id: str,
    requires: tuple[str, ...] = (),
    mount: str | None = None,
    bindings: tuple[dict[str, str], ...] = (),
) -> LayerManifest:
    """Build a minimal LayerManifest for testing."""
    from isaac_core.sim.manifest import Binding

    binding_objs = tuple(
        Binding(
            prim=b["prim"],
            attribute=b["attribute"],
            config=b.get("config"),
            resolve=b.get("resolve"),
        )
        for b in bindings
    )
    return LayerManifest(
        id=layer_id,
        usd=f"{layer_id}.usda",
        mount=mount or f"/Environment/{layer_id}",
        requires=requires,
        bindings=binding_objs,
    )


# -- Satisfied layers are planned ---------------------------------------------


def test_satisfied_layer_is_planned() -> None:
    m = _manifest("sensor", requires=("CAMERA",))
    caps = _caps(StageCapability.CAMERA)

    plan = plan_features(["sensor"], {"sensor": m}, caps)

    assert len(plan.enabled) == 1
    assert plan.enabled[0].manifest.id == "sensor"
    assert len(plan.skipped) == 0


def test_layer_with_no_requirements_is_always_planned() -> None:
    m = _manifest("simple")
    caps = _caps()  # empty capabilities

    plan = plan_features(["simple"], {"simple": m}, caps)
    assert len(plan.enabled) == 1
    assert plan.enabled[0].manifest.id == "simple"


# -- Unmet requirements -------------------------------------------------------


def test_unmet_requirement_skipped_with_reason_mentioning_capability() -> None:
    m = _manifest("bbox", requires=("BBOXES_ROOT",))
    caps = _caps()  # no BBOXES_ROOT

    plan = plan_features(["bbox"], {"bbox": m}, caps)

    assert len(plan.skipped) == 1
    assert plan.skipped[0].id == "bbox"
    assert "BBOXES_ROOT" in plan.skipped[0].reason


def test_strict_mode_raises_on_unmet_requirement() -> None:
    m = _manifest("bbox", requires=("BBOXES_ROOT",))
    caps = _caps()

    with pytest.raises(PlanningError, match="BBOXES_ROOT"):
        plan_features(["bbox"], {"bbox": m}, caps, strict=True)


# -- Unknown layer ids --------------------------------------------------------


def test_unknown_id_skipped_with_reason() -> None:
    plan = plan_features(["nonexistent"], {}, _caps())

    assert len(plan.skipped) == 1
    assert "nonexistent" in plan.skipped[0].reason
    assert plan.skipped[0].id == "nonexistent"


def test_strict_mode_raises_on_unknown_id() -> None:
    with pytest.raises(PlanningError, match="unknown layer id"):
        plan_features(["missing"], {}, _caps(), strict=True)


# -- Deterministic ordering ---------------------------------------------------


def test_ordering_is_deterministic() -> None:
    # Regardless of input order, enabled layers are sorted by id
    m_z = _manifest("zebra")
    m_a = _manifest("alpha")
    m_m = _manifest("middle")
    manifests = {"zebra": m_z, "alpha": m_a, "middle": m_m}
    caps = _caps()

    plan = plan_features(["zebra", "alpha", "middle"], manifests, caps)

    enabled_ids = [p.manifest.id for p in plan.enabled]
    assert enabled_ids == ["alpha", "middle", "zebra"]


def test_duplicate_requested_ids_are_deduplicated() -> None:
    m = _manifest("sensor")
    plan = plan_features(["sensor", "sensor", "sensor"], {"sensor": m}, _caps())
    assert len(plan.enabled) == 1


# -- Binding resolution -------------------------------------------------------


def test_bindings_render_to_expected_prim_paths() -> None:
    bindings = (
        {"prim": "{mount}/ActionGraph/publisher", "attribute": "inputs:topicName", "config": "topics.distance"},
        {"prim": "{mount}/node", "attribute": "inputs:origin", "resolve": "enu_origin"},
    )
    m = _manifest("sensor", mount="/Environment/sensor", bindings=bindings)
    caps = _caps()

    plan = plan_features(["sensor"], {"sensor": m}, caps, instance="drone_0")

    resolved = plan.enabled[0].resolved_bindings
    assert len(resolved) == 2
    assert resolved[0].prim == "/Environment/sensor/ActionGraph/publisher"
    assert resolved[0].attribute == "inputs:topicName"
    assert resolved[0].config == "topics.distance"
    assert resolved[0].resolve is None
    assert resolved[1].prim == "/Environment/sensor/node"
    assert resolved[1].resolve == "enu_origin"


def test_bindings_with_instance_placeholder() -> None:
    bindings = ({"prim": "/Environment/{instance}/sensor/node", "attribute": "inputs:val", "config": "x.y"},)
    m = _manifest("sensor", mount="/Environment/{instance}/sensor", bindings=bindings)
    caps = _caps()

    plan = plan_features(["sensor"], {"sensor": m}, caps, instance="lead")

    resolved = plan.enabled[0].resolved_bindings
    assert resolved[0].prim == "/Environment/lead/sensor/node"


# -- Report rendering ---------------------------------------------------------


def test_report_contains_enabled_and_skipped_entries() -> None:
    m_good = _manifest("sensor")
    m_bad = _manifest("bbox", requires=("BBOXES_ROOT",))
    manifests = {"sensor": m_good, "bbox": m_bad}
    caps = _caps()  # no BBOXES_ROOT

    plan = plan_features(["sensor", "bbox"], manifests, caps)
    report = plan.render_report()

    # Enabled entry has a tick
    assert "\u2713 sensor" in report
    # Skipped entry has circled-slash and the reason
    assert "\u2298 bbox" in report
    assert "BBOXES_ROOT" in report


def test_empty_plan_produces_empty_report() -> None:
    plan = FeaturePlan()
    assert plan.render_report() == ""


# -- Mixed scenarios ----------------------------------------------------------


def test_mix_of_enabled_unknown_and_unmet() -> None:
    m_ok = _manifest("camera", requires=("CAMERA",))
    m_needs = _manifest("bbox", requires=("BBOXES_ROOT",))
    manifests = {"camera": m_ok, "bbox": m_needs}
    caps = _caps(StageCapability.CAMERA)

    plan = plan_features(["camera", "bbox", "phantom"], manifests, caps)

    enabled_ids = [p.manifest.id for p in plan.enabled]
    skipped_ids = [s.id for s in plan.skipped]
    assert enabled_ids == ["camera"]
    assert set(skipped_ids) == {"bbox", "phantom"}


def test_multiple_unmet_requirements_all_listed_in_reason() -> None:
    m = _manifest("fancy", requires=("TILESETS_ROOT", "GEOREFERENCE"))
    caps = _caps()  # nothing

    plan = plan_features(["fancy"], {"fancy": m}, caps)
    reason = plan.skipped[0].reason
    assert "TILESETS_ROOT" in reason
    assert "GEOREFERENCE" in reason
