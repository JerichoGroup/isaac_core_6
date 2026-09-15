"""Test composer helper logic that can be verified without Isaac Sim.

Covers mount path derivation, layer USD file resolution, render report text and
reference ordering. The actual USD stage mounting requires a live Isaac Sim
session.
"""

from pathlib import Path

from isaac_core.sim.composer import layer_usd_path, mount_path_for_layer
from isaac_core.sim.manifest import LayerManifest
from isaac_core.sim.planner import FeaturePlan, PlannedLayer


def _make_planned(
    layer_id: str = "camera_udp",
    mount: str = "/Environment/{instance}",
    usd: str = "camera_udp.usda",
) -> PlannedLayer:
    manifest = LayerManifest(id=layer_id, usd=usd, mount=mount)
    return PlannedLayer(manifest=manifest, resolved_bindings=())


# -- mount_path_for_layer --------------------------------------------------------


def test_mount_path_substitutes_instance() -> None:
    planned = _make_planned(mount="/Environment/{instance}")
    path = mount_path_for_layer(planned, "drone_0")
    assert path == "/Environment/drone_0"


def test_mount_path_without_placeholder() -> None:
    planned = _make_planned(mount="/World/Sensors")
    path = mount_path_for_layer(planned, "drone_0")
    assert path == "/World/Sensors"


def test_mount_path_nested_instance() -> None:
    planned = _make_planned(mount="/World/{instance}/SubPrim")
    path = mount_path_for_layer(planned, "drone_0")
    assert path == "/World/drone_0/SubPrim"


# -- layer_usd_path resolution ---------------------------------------------------


def test_layer_usd_path_found_in_search_dir(tmp_path: Path) -> None:
    # Create a layer directory structure: <search_dir>/<layer_id>/<usd_file>
    layer_dir = tmp_path / "camera_udp"
    layer_dir.mkdir()
    usd_file = layer_dir / "camera_udp.usda"
    usd_file.write_text("dummy")

    planned = _make_planned(layer_id="camera_udp", usd="camera_udp.usda")
    result = layer_usd_path(planned, search_paths=(tmp_path,))
    assert result is not None
    assert result == usd_file.resolve()


def test_layer_usd_path_not_found_returns_none(tmp_path: Path) -> None:
    planned = _make_planned(layer_id="missing_layer", usd="missing.usda")
    result = layer_usd_path(planned, search_paths=(tmp_path,))
    assert result is None


def test_layer_usd_path_searches_in_order(tmp_path: Path) -> None:
    # First search path has the file, second doesn't matter.
    first = tmp_path / "first"
    second = tmp_path / "second"
    first.mkdir()
    second.mkdir()
    (first / "my_layer").mkdir()
    (first / "my_layer" / "layer.usda").write_text("first")
    (second / "my_layer").mkdir()
    (second / "my_layer" / "layer.usda").write_text("second")

    planned = _make_planned(layer_id="my_layer", usd="layer.usda")
    result = layer_usd_path(planned, search_paths=(first, second))
    assert result is not None
    assert result == (first / "my_layer" / "layer.usda").resolve()


def test_layer_usd_path_empty_search_paths() -> None:
    planned = _make_planned(layer_id="any", usd="any.usda")
    result = layer_usd_path(planned, search_paths=())
    assert result is None


# -- render_report text -----------------------------------------------------------


def test_render_report_shows_enabled_and_skipped() -> None:
    from isaac_core.sim.planner import SkippedLayer

    manifest_a = LayerManifest(id="alpha", usd="a.usda", mount="/Env/{instance}")
    plan = FeaturePlan(
        enabled=(PlannedLayer(manifest=manifest_a, resolved_bindings=()),),
        skipped=(SkippedLayer(id="gamma", reason="unmet: TILESETS_ROOT"),),
    )
    report = plan.render_report()
    assert "\u2713 alpha" in report
    assert "\u2298 gamma" in report
    assert "unmet: TILESETS_ROOT" in report


def test_render_report_empty_plan() -> None:
    plan = FeaturePlan(enabled=(), skipped=())
    report = plan.render_report()
    assert report == ""


# -- reference ordering: layers are composed in plan order (sorted by id) ----------


def test_layers_in_plan_are_sorted_by_id() -> None:
    # The planner sorts by id, so mounting respects that same order.
    from isaac_core.sim.capabilities import FakeStageInspector, probe
    from isaac_core.sim.planner import plan_features

    manifests = {
        "zzz": LayerManifest(id="zzz", usd="z.usda", mount="/Env/{instance}"),
        "aaa": LayerManifest(id="aaa", usd="a.usda", mount="/Env/{instance}"),
        "mmm": LayerManifest(id="mmm", usd="m.usda", mount="/Env/{instance}"),
    }
    caps = probe(FakeStageInspector(prims=frozenset()))
    plan = plan_features(
        requested_ids=["zzz", "aaa", "mmm"],
        manifests=manifests,
        capabilities=caps,
    )
    ids = [p.manifest.id for p in plan.enabled]
    assert ids == ["aaa", "mmm", "zzz"]


# -- compose_stage requires Isaac (cannot unit-test fully) -------------------------
# The actual compose_stage function depends on omni.usd.get_context(), which
# is only available inside Isaac Sim. What we CAN test here is all the pure logic
# it delegates to: mount path derivation, layer USD resolution, report generation,
# and the write computation (tested in test_configurator.py). The integration of
# these with a real USD stage can only be verified by launching Isaac Sim.
