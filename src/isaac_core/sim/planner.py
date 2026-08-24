"""
Feature planner: decide which layers to compose and resolve their bindings.

Given a set of requested layer ids, a manifest registry, and the detected stage
capabilities, the planner produces a :class:`FeaturePlan` listing — in
deterministic order — which layers will be composed and which are skipped, each
with a human-readable reason.

This replaces the ``if "distance_sensor" in self.usds_to_add:`` branches of
the previous generation. The ``SimulationRuntime`` never mentions a feature by
name; it just iterates the plan.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

from isaac_core.contracts.prims import render
from isaac_core.sim.capabilities import StageCapabilities
from isaac_core.sim.manifest import LayerManifest


class PlanningError(Exception):
    """Raised in strict mode when a layer cannot be composed."""


@dataclass(frozen=True, slots=True)
class ResolvedBinding:
    """
    A binding with its prim template rendered to a concrete path.

    Args:
        prim: Concrete absolute prim path after placeholder substitution.
        attribute: USD attribute name.
        config: Dotted config key, or ``None`` if this is a runtime-resolve binding.
        resolve: Named runtime value, or ``None`` if this is a config binding.

    """

    prim: str
    attribute: str
    config: str | None = None
    resolve: str | None = None


@dataclass(frozen=True, slots=True)
class PlannedLayer:
    """
    A layer that will be composed onto the stage.

    Args:
        manifest: The validated layer manifest.
        resolved_bindings: Bindings with concrete prim paths.

    """

    manifest: LayerManifest
    resolved_bindings: tuple[ResolvedBinding, ...] = ()


@dataclass(frozen=True, slots=True)
class SkippedLayer:
    """
    A layer that was requested but cannot be composed.

    Args:
        id: Layer id as requested.
        reason: Human-readable explanation of why it was skipped.

    """

    id: str
    reason: str


@dataclass(frozen=True, slots=True)
class FeaturePlan:
    """
    The complete plan of layers to compose and layers to skip.

    Layers are in a deterministic order (sorted by id). The plan is the single
    input to the compositor — it never needs to re-query capabilities or look up
    manifests.
    """

    enabled: tuple[PlannedLayer, ...] = ()
    skipped: tuple[SkippedLayer, ...] = ()

    def render_report(self) -> str:
        """
        Produce a human-readable startup summary.

        Enabled layers get a tick (\u2713); skipped layers get a circled-slash
        (\u2298) and their reason. This is the report printed at simulator startup
        so operators can see what is active at a glance.

        Returns:
            Multi-line report string.

        """
        lines: list[str] = []
        for planned in self.enabled:
            lines.append(f"  \u2713 {planned.manifest.id}")
        for skipped in self.skipped:
            lines.append(f"  \u2298 {skipped.id} \u2014 {skipped.reason}")
        return "\n".join(lines)


@dataclass(slots=True)
class _PlanBuilder:
    """Mutable accumulator used during planning."""

    enabled: list[PlannedLayer] = field(default_factory=list)
    skipped: list[SkippedLayer] = field(default_factory=list)


def _render_mount(mount_template: str, instance: str) -> str:
    """
    Render a mount template into a concrete prim path segment.

    The mount template may contain ``{instance}`` but not ``{mount}`` (the mount
    IS the mount). Only ``{instance}`` is substituted.

    Args:
        mount_template: Mount path template from the manifest.
        instance: Instance identifier.

    Returns:
        Concrete mount path.

    """
    return mount_template.format(instance=instance)


def _resolve_bindings(
    manifest: LayerManifest,
    mount: str,
    instance: str,
) -> tuple[ResolvedBinding, ...]:
    """
    Render each binding's prim template into a concrete path.

    Args:
        manifest: The layer manifest whose bindings to resolve.
        mount: The concrete mount path for this layer.
        instance: The instance identifier.

    Returns:
        Resolved bindings with concrete prim paths.

    """
    resolved: list[ResolvedBinding] = []
    for binding in manifest.bindings:
        concrete_prim = render(binding.prim, mount=mount, instance=instance)
        resolved.append(
            ResolvedBinding(
                prim=concrete_prim,
                attribute=binding.attribute,
                config=binding.config,
                resolve=binding.resolve,
            )
        )
    return tuple(resolved)


def plan_features(
    requested_ids: Sequence[str],
    manifests: Mapping[str, LayerManifest],
    capabilities: StageCapabilities,
    *,
    strict: bool = False,
    instance: str = "default",
) -> FeaturePlan:
    """
    Decide which layers to compose and which to skip.

    Processing order is deterministic: requested ids are sorted alphabetically.
    For each requested id, if it is unknown or its requirements are unmet, it is
    skipped (or raises in strict mode).

    Args:
        requested_ids: Layer ids the user wants enabled.
        manifests: All discovered manifests, keyed by id.
        capabilities: Stage capabilities detected by
            :func:`~isaac_core.sim.capabilities.probe`.
        strict: If ``True``, raise :class:`PlanningError` instead of skipping.
        instance: Instance identifier for prim path template substitution.

    Returns:
        A complete feature plan.

    Raises:
        PlanningError: In strict mode, if any requested layer cannot be composed.

    """
    builder = _PlanBuilder()

    for layer_id in sorted(set(requested_ids)):
        if layer_id not in manifests:
            reason = f"unknown layer id {layer_id!r}"
            if strict:
                raise PlanningError(reason)
            builder.skipped.append(SkippedLayer(id=layer_id, reason=reason))
            continue

        manifest = manifests[layer_id]

        # Check all required capabilities.
        unmet: list[str] = []
        for req in manifest.requires:
            try:
                if not capabilities.has_named(req):
                    unmet.append(req)
            except KeyError:
                unmet.append(req)

        if unmet:
            reason = f"unmet stage requirement(s): {', '.join(unmet)}"
            if strict:
                raise PlanningError(reason)
            builder.skipped.append(SkippedLayer(id=layer_id, reason=reason))
            continue

        mount = _render_mount(manifest.mount, instance=instance)
        resolved = _resolve_bindings(manifest, mount=mount, instance=instance)
        builder.enabled.append(PlannedLayer(manifest=manifest, resolved_bindings=resolved))

    return FeaturePlan(
        enabled=tuple(builder.enabled),
        skipped=tuple(builder.skipped),
    )


__all__ = [
    "FeaturePlan",
    "PlannedLayer",
    "PlanningError",
    "ResolvedBinding",
    "SkippedLayer",
    "plan_features",
]
