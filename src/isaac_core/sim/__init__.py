"""
Feature-layer system: manifest-driven discovery, capability probing and planning.

This package is the pure, testable core of the simulator's feature-layer
composition. It handles discovering layers from disk, validating their manifests,
probing the USD stage for capabilities, and planning which layers to compose.

Everything here is free of ``omni``, ``carb`` and ``pxr`` imports — the Isaac-backed
runtime that actually opens stages and writes prim attributes comes later and
satisfies the narrow :class:`~isaac_core.sim.capabilities.StageInspector` protocol
defined here.
"""

from isaac_core.sim.capabilities import (
    FakeStageInspector,
    StageCapabilities,
    StageCapability,
    StageInspector,
    probe,
)
from isaac_core.sim.discovery import discover_layers
from isaac_core.sim.georeference import (
    ResolvedEnuReference,
    describe_mismatch,
    read_scene_georeference,
    resolve_enu_reference,
)
from isaac_core.sim.manifest import Binding, LayerManifest, load_manifest
from isaac_core.sim.planner import FeaturePlan, plan_features

__all__ = [
    "resolve_enu_reference",
    "read_scene_georeference",
    "describe_mismatch",
    "ResolvedEnuReference",
    "Binding",
    "FakeStageInspector",
    "FeaturePlan",
    "LayerManifest",
    "StageCapabilities",
    "StageCapability",
    "StageInspector",
    "discover_layers",
    "load_manifest",
    "plan_features",
    "probe",
]
