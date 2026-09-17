"""Guards for one unnamed camera per vehicle.

Through V2 a vehicle carried ``cameras: dict[str, CameraConfig]`` defaulting to the name ``"eo"``,
while the planner composed only the first entry. The name therefore existed to distinguish cameras
that could not coexist, and it appeared in topics, prim paths, manifest templates and every config
example. V3 removed it.

The trade is deliberate and these tests pin both halves: a vehicle has exactly one camera, and an
extra imaging sensor still works because it arrives as a feature layer mounted under the same vehicle.
"""

from __future__ import annotations

from pathlib import Path
import re
import tempfile
from typing import Final

import pytest

from isaac_core.config import IsaacCoreConfig, load
from isaac_core.config.schema import CameraConfig, VehicleConfig
from isaac_core.sim.capabilities import FakeStageInspector, probe
from isaac_core.sim.configurator import compute_writes
from isaac_core.sim.discovery import discover_layers
from isaac_core.sim.georeference import ResolvedEnuReference
from isaac_core.sim.manifest import KNOWN_PLACEHOLDERS
from isaac_core.sim.planner import plan_features

REPO_ROOT: Final = Path(__file__).resolve().parents[3]
LAYERS_ROOT: Final = REPO_ROOT / "src" / "isaac_core" / "assets" / "layers"

# Pages a new user reads. The migration guide and the engineering log record the old shape on purpose.
_HISTORICAL: Final = frozenset({"migrating_from_2023.md", "development-log.md", "roadmap.md", "v3_plan.md"})


def test_a_vehicle_has_one_camera_and_it_has_no_name() -> None:
    vehicle = IsaacCoreConfig().vehicles["drone_0"]
    assert isinstance(vehicle.camera, CameraConfig)
    assert not hasattr(vehicle, "cameras")


def test_the_old_named_table_is_rejected_by_name() -> None:
    # "Extra inputs are not permitted" would not tell anyone what to write instead.
    with pytest.raises(ValueError, match=r"\[vehicles\.<id>\.camera\]"):
        load(cli_overrides={"vehicles.drone_0.cameras.eo.fov_deg": "90"})


def test_the_rejection_names_the_camera_that_was_found() -> None:
    with pytest.raises(ValueError, match="'thermal'"):
        VehicleConfig(cameras={"thermal": CameraConfig()})


def test_no_topic_carries_a_camera_segment() -> None:
    single = IsaacCoreConfig()
    assert single.topic_resolver("drone_0").camera is None
    assert single.topic_resolver("drone_0").resolve("image_rgb") == "/isaac_core/image_rgb"

    swarm = IsaacCoreConfig(vehicles={"lead": VehicleConfig(), "wing": VehicleConfig()})
    assert swarm.topic_resolver("lead").resolve("image_rgb") == "/isaac_core/lead/image_rgb"


def test_the_camera_placeholder_no_longer_exists() -> None:
    # A manifest using {camera} must fail at load rather than resolve to nothing at compose time.
    assert "camera" not in KNOWN_PLACEHOLDERS


def test_no_shipped_manifest_templates_a_camera_name() -> None:
    offenders: list[str] = []
    for manifest in sorted(LAYERS_ROOT.glob("*/layer.toml")):
        text = manifest.read_text(encoding="utf-8")
        if "{camera}" in text or ".cameras." in text:
            offenders.append(manifest.parent.name)
    assert not offenders, f"these still template a camera name: {offenders}"


def test_no_user_facing_page_shows_the_old_table() -> None:
    offenders: list[str] = []
    pages = [REPO_ROOT / "README.md", *(REPO_ROOT / "docs").rglob("*.md"), *(REPO_ROOT / "config").glob("*.toml")]
    for page in pages:
        if page.name in _HISTORICAL:
            continue
        for number, line in enumerate(page.read_text(encoding="utf-8").splitlines(), start=1):
            if re.search(r"cameras\.[a-z_0-9]+|cameras\s*=|CAMERAS__", line):
                offenders.append(f"{page.relative_to(REPO_ROOT)}:{number}: {line.strip()[:60]}")
    assert not offenders, f"these show the removed named-camera shape: {offenders}"


def test_the_camera_config_still_reaches_a_prim_attribute() -> None:
    # The rename must not quietly stop the camera's settings being applied, which is the whole point
    # of the binding.
    config = load(cli_overrides={"vehicles.drone_0.camera.focal_length_mm": "31.5"})
    manifests = discover_layers((LAYERS_ROOT,))
    plan = plan_features(
        requested_ids=list(config.required_feature_ids()),
        manifests=manifests,
        capabilities=probe(FakeStageInspector(prims=frozenset())),
        instance="drone_0",
    )
    enu = ResolvedEnuReference(reference=config.geo.enu_reference, source="config")
    writes = compute_writes(config, plan, enu, camera_prim="/World/cam", vehicle_id="drone_0")
    focal = [w for w in writes if w.attribute == "focalLength"]
    assert focal, f"no focalLength write among {[w.attribute for w in writes]}"
    assert focal[0].value == pytest.approx(31.5)


def test_a_layer_can_still_add_a_second_camera_to_the_same_vehicle() -> None:
    # This is the trade that makes dropping the name acceptable. A thermal camera arrives as a layer,
    # mounted under the same vehicle so it moves with the airframe, with its own settings under
    # [layers.<id>] reaching its own prim.
    with tempfile.TemporaryDirectory() as directory:
        layer_dir = Path(directory) / "thermal_cam"
        layer_dir.mkdir()
        (layer_dir / "thermal_cam.usda").write_text('#usda 1.0\n(\n    defaultPrim = "Root"\n)\n', encoding="utf-8")
        (layer_dir / "layer.toml").write_text(
            'id = "thermal_cam"\n'
            'usd = "thermal_cam.usda"\n'
            'mount = "/World/Environment/{instance}"\n'
            "\n"
            "[[bindings]]\n"
            'prim = "{mount}/ThermalExport/thermal_camera"\n'
            'attribute = "focalLength"\n'
            'config = "layers.thermal_cam.focal_length_mm"\n',
            encoding="utf-8",
        )
        config = load(
            cli_overrides={
                "assets.layer_search_paths": f'["{directory}"]',
                "features.enabled": '["thermal_cam"]',
                "layers": '{"thermal_cam": {"focal_length_mm": 8.0}}',
            }
        )
        manifests = discover_layers((Path(directory), LAYERS_ROOT))
        assert "thermal_cam" in manifests
        plan = plan_features(
            requested_ids=list(config.required_feature_ids()),
            manifests=manifests,
            capabilities=probe(FakeStageInspector(prims=frozenset())),
            instance="drone_0",
        )
        composed = [planned.manifest.id for planned in plan.enabled]
        assert "thermal_cam" in composed, f"the layer did not compose: {composed}"
        assert "camera_udp" in composed, "the vehicle's own camera layer must still compose alongside it"

        enu = ResolvedEnuReference(reference=config.geo.enu_reference, source="config")
        writes = compute_writes(config, plan, enu, camera_prim="/World/cam", vehicle_id="drone_0")
        thermal = [w for w in writes if "Thermal" in w.prim]
        assert thermal, f"the thermal layer wrote nothing: {[w.prim for w in writes]}"
        assert thermal[0].prim.startswith("/World/Environment/drone_0/"), "it must mount under the vehicle"
        assert thermal[0].value == pytest.approx(8.0)
