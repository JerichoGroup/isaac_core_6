"""Feature planner: decide which layers to compose and resolve their bindings.

Given a set of requested layer ids, a manifest registry, and the detected stage
capabilities, the planner produces a :class:`FeaturePlan` listing — in
deterministic order — which layers will be composed and which are skipped, each
with a human-readable reason.

This is what removes any ``if "distance_sensor" in ...:`` branch from the runtime. The
``SimulationRuntime`` never mentions a feature by name; it just iterates the plan.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

from isaac_core.contracts.prims import placeholders, render
from isaac_core.sim.capabilities import StageCapabilities
from isaac_core.sim.manifest import CAMERA, INSTANCE, LayerManifest


class PlanningError(Exception):
    """Raised in strict mode when a layer cannot be composed."""


@dataclass(frozen=True, slots=True)
class ResolvedBinding:
    """A binding with its prim template rendered to a concrete path.

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
    """A layer that will be composed onto the stage.

    Args:
        manifest: The validated layer manifest.
        resolved_bindings: Bindings with concrete prim paths.
        instance: The vehicle id this layer was planned for.
        camera: The camera id this layer was planned for, when camera-scoped.

    """

    manifest: LayerManifest
    resolved_bindings: tuple[ResolvedBinding, ...] = ()
    # The identity this layer was planned for. Carried per layer rather than assumed globally so a
    # swarm can compose one camera layer per vehicle and each resolves its own ports and topics.
    instance: str = "default"
    camera: str | None = None


@dataclass(frozen=True, slots=True)
class SkippedLayer:
    """A layer that was requested but cannot be composed.

    Args:
        id: Layer id as requested.
        reason: Human-readable explanation of why it was skipped.

    """

    id: str
    reason: str


@dataclass(frozen=True, slots=True)
class FeaturePlan:
    """The complete plan of layers to compose and layers to skip.

    Layers are in a deterministic order (sorted by id). The plan is the single
    input to the compositor — it never needs to re-query capabilities or look up
    manifests.
    """

    enabled: tuple[PlannedLayer, ...] = ()
    skipped: tuple[SkippedLayer, ...] = ()

    def _is_repeated(self, planned: PlannedLayer) -> bool:
        """Report whether another enabled layer shares this one's id.

        Args:
            planned: The layer being described.

        Returns:
            ``True`` when the same layer id appears more than once, i.e. one copy per vehicle.

        """
        return sum(1 for other in self.enabled if other.manifest.id == planned.manifest.id) > 1

    def render_report(self) -> str:
        """Produce a human-readable startup summary.

        Enabled layers get a tick (\u2713); skipped layers get a circled-slash
        (\u2298) and their reason. This is the report printed at simulator startup
        so operators can see what is active at a glance.

        Returns:
            Multi-line report string.

        """
        lines: list[str] = []
        for planned in self.enabled:
            # Name the vehicle when there is more than one layer for the same id, otherwise a
            # swarm's report is just the same line repeated with no way to tell them apart.
            suffix = f" [{planned.instance}]" if planned.instance != "default" and self._is_repeated(planned) else ""
            lines.append(f"  \u2713 {planned.manifest.id}{suffix}")
        for skipped in self.skipped:
            lines.append(f"  \u2298 {skipped.id} \u2014 {skipped.reason}")
        return "\n".join(lines)


@dataclass(slots=True)
class _PlanBuilder:
    """Mutable accumulator used during planning."""

    enabled: list[PlannedLayer] = field(default_factory=list)
    skipped: list[SkippedLayer] = field(default_factory=list)


def _render_mount(mount_template: str, instance: str) -> str:
    """Render a mount template into a concrete prim path segment.

    The mount template may contain ``{instance}`` but not ``{mount}`` (the mount
    IS the mount). Only ``{instance}`` is substituted.

    Args:
        mount_template: Mount path template from the manifest.
        instance: Instance identifier.

    Returns:
        Concrete mount path.

    """
    return mount_template.format(instance=instance)


def _render_config_key(template: str, *, instance: str, camera: str | None) -> str:
    """Substitute placeholders in a binding's dotted config key.

    Both ``{instance}`` and ``{camera}`` are substituted here, mirroring what
    :func:`~isaac_core.contracts.prims.render` does for prim paths. A leftover
    ``{placeholder}`` raises rather than silently passing through: an unrendered key
    reaches the configurator as a literal like ``vehicles.{camera}.x``, which fails far
    from the manifest with a config-key error that does not name the real mistake.

    Args:
        template: The dotted config key from the binding, possibly templated.
        instance: The instance identifier, normally the vehicle id.
        camera: The camera key to substitute for ``{camera}``, or ``None`` if this
            layer instance serves no specific camera.

    Returns:
        The config key with all supported placeholders substituted.

    Raises:
        PlanningError: If a ``{camera}`` placeholder is present but no camera was
            supplied, or if any unknown placeholder remains.

    """
    found = placeholders(template)
    if CAMERA in found and camera is None:
        msg = (
            f"config key {template!r} uses {{{CAMERA}}} but no camera was supplied to the "
            f"planner; pass a camera id to resolve it"
        )
        raise PlanningError(msg)

    substitutions = {INSTANCE: instance}
    if camera is not None:
        substitutions[CAMERA] = camera

    unknown = found - substitutions.keys()
    if unknown:
        msg = (
            f"config key {template!r} uses unknown placeholder(s): {', '.join(sorted(unknown))}; "
            f"allowed: {', '.join(sorted(substitutions))}"
        )
        raise PlanningError(msg)

    return template.format(**substitutions)


def _resolve_bindings(
    manifest: LayerManifest,
    mount: str,
    instance: str,
    camera: str | None,
) -> tuple[ResolvedBinding, ...]:
    """Render each binding's prim template and config key into concrete strings.

    Both sides need the instance substituted, not just the prim path. A binding like
    ``config = "vehicles.{instance}.rotation_frame"`` is meaningless until
    ``{instance}`` becomes a real vehicle id, and leaving it unrendered produced a
    ``ConfigKeyError`` at compose time complaining that ``vehicles.{instance}`` does not
    exist. "Resolved" has to mean resolved on every axis, or the name lies. The same is
    true of ``{camera}``, which lets a binding template the camera key instead of
    hardcoding one.

    Args:
        manifest: The layer manifest whose bindings to resolve.
        mount: The concrete mount path for this layer.
        instance: The instance identifier, normally the vehicle id.
        camera: The camera key to substitute for ``{camera}``, or ``None`` if this layer
            instance serves no specific camera.

    Returns:
        Resolved bindings with concrete prim paths and config keys.

    """
    render_substitutions = {"mount": mount, "instance": instance}
    if camera is not None:
        render_substitutions["camera"] = camera

    resolved: list[ResolvedBinding] = []
    for binding in manifest.bindings:
        concrete_prim = render(binding.prim, **render_substitutions)
        concrete_config = (
            _render_config_key(binding.config, instance=instance, camera=camera) if binding.config is not None else None
        )
        resolved.append(
            ResolvedBinding(
                prim=concrete_prim,
                attribute=binding.attribute,
                config=concrete_config,
                resolve=binding.resolve,
            )
        )
    return tuple(resolved)


def _capability_names() -> tuple[str, ...]:
    """Return every known stage capability name.

    Returns:
        Enum member names usable in a manifest's ``requires``/``provides``.

    """
    from isaac_core.sim.capabilities import StageCapability

    return tuple(c.name for c in StageCapability)


def plan_features(
    requested_ids: Sequence[str],
    manifests: Mapping[str, LayerManifest],
    capabilities: StageCapabilities,
    *,
    strict: bool = False,
    instance: str = "default",
    camera: str | None = None,
) -> FeaturePlan:
    """Decide which layers to compose and which to skip.

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
        camera: Camera key for ``{camera}`` substitution in prim paths and config keys.
            A layer instance today serves one camera; the caller passes that camera's key
            (for v1 parity, the vehicle's first camera). ``None`` means this plan has no
            camera, which is only valid when no binding references ``{camera}``.

    Returns:
        A complete feature plan.

    Raises:
        PlanningError: In strict mode, if any requested layer cannot be composed, or if
            a binding references ``{camera}`` without a camera being supplied.

    """
    builder = _PlanBuilder()

    pending: list[str] = []
    for layer_id in sorted(set(requested_ids)):
        if layer_id not in manifests:
            reason = f"unknown layer id {layer_id!r}"
            if strict:
                raise PlanningError(reason)
            builder.skipped.append(SkippedLayer(id=layer_id, reason=reason))
            continue
        pending.append(layer_id)

    # A requirement may be satisfied by the scene OR by another layer (bbox requires CAMERA, which
    # the camera layer provides), so capabilities grow as layers are admitted and admission repeats
    # to a fixed point. The resulting order is also the composition order, which matters: a layer
    # referencing another's prim must come after it. Alphabetical put bbox before camera_udp.
    available = {name for name in _capability_names() if capabilities.has_named(name)}

    while pending:
        admitted_this_pass: list[str] = []
        for layer_id in pending:
            manifest = manifests[layer_id]
            if any(req not in available for req in manifest.requires):
                continue

            mount = _render_mount(manifest.mount, instance=instance)
            resolved = _resolve_bindings(manifest, mount=mount, instance=instance, camera=camera)
            builder.enabled.append(
                PlannedLayer(
                    manifest=manifest,
                    resolved_bindings=resolved,
                    instance=instance,
                    camera=camera,
                )
            )
            admitted_this_pass.append(layer_id)

        if not admitted_this_pass:
            break

        for layer_id in admitted_this_pass:
            available.update(manifests[layer_id].provides)
            pending.remove(layer_id)

    # Whatever is still pending cannot be satisfied by the scene or by any layer that was
    # admitted, which also covers a genuine dependency cycle.
    for layer_id in pending:
        unmet = [req for req in manifests[layer_id].requires if req not in available]
        reason = f"unmet stage requirement(s): {', '.join(unmet)}"
        if strict:
            raise PlanningError(reason)
        builder.skipped.append(SkippedLayer(id=layer_id, reason=reason))

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
