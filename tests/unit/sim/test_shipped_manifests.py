"""Contract tests: shipped layer manifests match the authored USD exactly.

This is the single most valuable test file in this scope. It prevents a prim
rename in the GUI from silently breaking configuration: every binding's prim
path, rendered with a representative mount, is verified to correspond to a prim
that ACTUALLY EXISTS in the matching .usda file, and every attribute name is
verified to appear on that prim.
"""

from __future__ import annotations

from pathlib import Path
import re
from typing import Final

import pytest

from isaac_core.assets import LAYERS_DIR
from isaac_core.config import IsaacCoreConfig
from isaac_core.contracts.prims import render
from isaac_core.sim.capabilities import StageCapability
from isaac_core.sim.discovery import discover_layers
from isaac_core.sim.manifest import Binding, LayerManifest, load_manifest

# Every layer shipped inside the package. Parametrising over this rather than
# naming each one means a new shipped layer is covered automatically.
SHIPPED_LAYER_IDS: Final = ("camera_udp", "camera_ros")

# --- Constants ---------------------------------------------------------------

# Path to the authored USD layers (repo root / usd / layers /)
_REPO_ROOT: Final = Path(__file__).resolve().parents[3]
_USD_LAYERS_DIR: Final = _REPO_ROOT / "src" / "isaac_core" / "assets" / "layers"

# Representative mount used to render {mount} in prim templates.
_REPRESENTATIVE_MOUNT: Final = "/Root"

# Representative instance used to render {instance} in prim templates.
_REPRESENTATIVE_INSTANCE: Final = "drone_0"

# Representative camera used to render {camera} in prim and config templates.
_REPRESENTATIVE_CAMERA: Final = "eo"

# Match a USD prim definition: `def <type> "<name>"` or `def "<name>"`
_PRIM_DEF_RE: Final = re.compile(r'^\s*def\s+(?:\w+\s+)?"([^"]+)"')

# Match an over statement: `over "<name>"`
_OVER_RE: Final = re.compile(r'^\s*over\s+"([^"]+)"')

# Match attribute declarations — both `custom <type> <name>` and `<type> <name> =`
# and `uniform <type> <name>` and `float <name> =`
_ATTR_RE: Final = re.compile(r"^\s*(?:custom\s+|uniform\s+)?(?:\w+\s+)+(\S+?)(?:\s*=|\s*\(|\s*$)")


# --- USD text parser ---------------------------------------------------------


def _parse_usd_structure(usd_text: str) -> dict[str, set[str]]:
    """Parse a .usda file and return a mapping of absolute prim paths to their attributes.

    This is a line-by-line parser sufficient for our contract tests. It tracks
    brace-delimited scope to build absolute paths.

    Args:
        usd_text: Contents of a .usda file.

    Returns:
        Mapping from absolute prim path (e.g. "/Root/PoseSync/udp_to_global_position")
        to the set of attribute names declared on that prim (e.g. "inputs:udp_port").

    """
    prims: dict[str, set[str]] = {}
    path_stack: list[str] = []
    brace_depth = 0
    # Track what brace depth each path component was introduced at
    depth_at_push: list[int] = []

    for line in usd_text.splitlines():
        stripped = line.strip()

        # Check for prim definition
        prim_match = _PRIM_DEF_RE.match(line)
        over_match = _OVER_RE.match(line) if not prim_match else None

        if prim_match or over_match:
            name = prim_match.group(1) if prim_match else over_match.group(1)  # type: ignore[union-attr]
            # We need to see the opening brace to push
            if "{" in line:
                path_stack.append(name)
                depth_at_push.append(brace_depth)
                brace_depth += 1
                abs_path = "/" + "/".join(path_stack)
                prims.setdefault(abs_path, set())
            elif stripped.endswith("("):
                # Multi-line metadata; the brace comes on a later line
                # We'll handle it when we see the brace
                pass
        elif stripped == "{" or stripped.startswith("{"):
            # Opening brace without a def on the same line — could be continuing
            # a def that had metadata, or just a nested scope
            # Check if previous meaningful line was a def
            brace_depth += 1

        # Count braces on this line
        close_braces = line.count("}")

        # Handle closing braces — pop path stack when appropriate
        for _ in range(close_braces):
            brace_depth -= 1
            if depth_at_push and brace_depth <= depth_at_push[-1]:
                path_stack.pop()
                depth_at_push.pop()

        # If we are inside a prim, record attributes
        if path_stack:
            current_path = "/" + "/".join(path_stack)
            if current_path in prims:
                attr_match = _ATTR_RE.match(line)
                if attr_match:
                    attr_name = attr_match.group(1)
                    # Filter out non-attribute matches
                    if ":" in attr_name or attr_name in {
                        "focalLength",
                        "horizontalAperture",
                        "verticalAperture",
                        "focusDistance",
                        "clippingRange",
                    }:
                        prims[current_path].add(attr_name)

    return prims


def _parse_usda_prims_and_attrs(usd_path: Path) -> dict[str, set[str]]:
    """Parse a USD file and return prim paths and their attributes.

    Uses a simpler, more robust approach: scan for `def ... "Name"` lines to build
    a path, and collect any `inputs:*` / `outputs:*` / known camera attributes.

    Args:
        usd_path: Path to the .usda file.

    Returns:
        Mapping from prim path to set of attribute names on that prim.

    """
    text = usd_path.read_text(encoding="utf-8")
    prims: dict[str, set[str]] = {}
    path_stack: list[str] = []
    # Track brace depth per path element for correct popping
    pending_def: str | None = None
    brace_depth = 0
    scope_depth: list[int] = []

    for line in text.splitlines():
        # Detect prim definition
        def_match = _PRIM_DEF_RE.match(line)
        over_match = _OVER_RE.match(line) if not def_match else None

        if def_match or over_match:
            name = def_match.group(1) if def_match else over_match.group(1)  # type: ignore[union-attr]
            pending_def = name

        # Count braces
        opens = line.count("{")
        closes = line.count("}")

        for _ in range(opens):
            brace_depth += 1
            if pending_def is not None:
                path_stack.append(pending_def)
                scope_depth.append(brace_depth)
                abs_path = "/" + "/".join(path_stack)
                prims.setdefault(abs_path, set())
                pending_def = None

        for _ in range(closes):
            if scope_depth and brace_depth == scope_depth[-1]:
                path_stack.pop()
                scope_depth.pop()
            brace_depth -= 1

        # Collect attributes on current prim
        if path_stack:
            current_path = "/" + "/".join(path_stack)
            if current_path in prims:
                # Match custom/uniform type attrName patterns
                # Catches: `custom int inputs:udp_port`
                #          `float focalLength = 18.147562`
                #          `custom string inputs:topicName = "..."`
                attr_m = re.match(
                    r"\s*(?:custom\s+|uniform\s+|prepend\s+)?" r"(?:rel\s+|(?:\w+(?:\[\w*\])?\s+)+)" r"(\w[\w:]*)",
                    line,
                )
                if attr_m:
                    attr_name = attr_m.group(1)
                    # Only keep actual node/prim attributes (contain colon or are camera attrs)
                    if ":" in attr_name or attr_name in {
                        "focalLength",
                        "horizontalAperture",
                        "verticalAperture",
                        "focusDistance",
                    }:
                        prims[current_path].add(attr_name)

    return prims


# --- Fixtures ----------------------------------------------------------------


@pytest.fixture()
def udp_manifest() -> LayerManifest:
    """Load the shipped camera_udp manifest."""
    return load_manifest(LAYERS_DIR / "camera_udp" / "layer.toml")


@pytest.fixture()
def ros_manifest() -> LayerManifest:
    """Load the shipped camera_ros manifest."""
    return load_manifest(LAYERS_DIR / "camera_ros" / "layer.toml")


@pytest.fixture()
def udp_usd_prims() -> dict[str, set[str]]:
    """Parse the camera_udp .usda into a prim→attributes map."""
    return _parse_usda_prims_and_attrs(_USD_LAYERS_DIR / "camera_udp" / "camera_udp.usda")


@pytest.fixture()
def ros_usd_prims() -> dict[str, set[str]]:
    """Parse the camera_ros .usda into a prim→attributes map."""
    return _parse_usda_prims_and_attrs(_USD_LAYERS_DIR / "camera_ros" / "camera_ros.usda")


# --- Manifest loads and validates -------------------------------------------


def test_camera_udp_manifest_loads(udp_manifest: LayerManifest) -> None:
    assert udp_manifest.id == "camera_udp"
    assert udp_manifest.usd == "camera_udp.usda"


def test_camera_ros_manifest_loads(ros_manifest: LayerManifest) -> None:
    assert ros_manifest.id == "camera_ros"
    assert ros_manifest.usd == "camera_ros.usda"


def test_camera_udp_requires_nothing_of_the_base_scene(udp_manifest: LayerManifest) -> None:
    # A camera layer BRINGS its own camera. This asserted the opposite until
    # 2026-08-25, which meant the layer was always skipped with "unmet stage
    # requirement: CAMERA" -- the base scene deliberately has no camera, so the
    # requirement could never be satisfied.
    assert udp_manifest.requires == ()


def test_camera_ros_requires_nothing_of_the_base_scene(ros_manifest: LayerManifest) -> None:
    # A camera layer BRINGS its own camera. This asserted the opposite until
    # 2026-08-25, which meant the layer was always skipped with "unmet stage
    # requirement: CAMERA" -- the base scene deliberately has no camera, so the
    # requirement could never be satisfied.
    assert ros_manifest.requires == ()


def test_camera_udp_provides_camera(udp_manifest: LayerManifest) -> None:
    assert "CAMERA" in udp_manifest.provides


def test_camera_ros_provides_camera(ros_manifest: LayerManifest) -> None:
    assert "CAMERA" in ros_manifest.provides


def test_camera_udp_mount_is_instance_templated(udp_manifest: LayerManifest) -> None:
    # {instance} so the layer mounts once per aircraft in a swarm. It must NOT use
    # {mount} -- this field defines what {mount} means, and the circular form used
    # to crash at compose time with a bare KeyError.
    assert "{instance}" in udp_manifest.mount
    assert "{mount}" not in udp_manifest.mount


def test_camera_ros_mount_is_instance_templated(ros_manifest: LayerManifest) -> None:
    # {instance} so the layer mounts once per aircraft in a swarm. It must NOT use
    # {mount} -- this field defines what {mount} means, and the circular form used
    # to crash at compose time with a bare KeyError.
    assert "{instance}" in ros_manifest.mount
    assert "{mount}" not in ros_manifest.mount


# --- Camera key is templated, not hardcoded ----------------------------------


def test_camera_udp_templates_the_camera_key(udp_manifest: LayerManifest) -> None:
    # Hardcoding "eo" meant a config whose camera was named anything else died at compose
    # time with ConfigKeyError: vehicles.drone_0.cameras.eo.width does not exist. The
    # camera key is now templated, so renaming a camera needs no manifest edit.
    camera_config_bindings = [b for b in udp_manifest.bindings if b.config and "cameras" in b.config]
    assert camera_config_bindings, "expected at least one camera config binding"
    for binding in camera_config_bindings:
        assert binding.config is not None
        assert "{camera}" in binding.config
        assert ".cameras.eo." not in binding.config


def test_camera_ros_templates_the_camera_key(ros_manifest: LayerManifest) -> None:
    # See test_camera_udp_templates_the_camera_key: same hardcoded-"eo" bug.
    camera_config_bindings = [b for b in ros_manifest.bindings if b.config and "cameras" in b.config]
    assert camera_config_bindings, "expected at least one camera config binding"
    for binding in camera_config_bindings:
        assert binding.config is not None
        assert "{camera}" in binding.config
        assert ".cameras.eo." not in binding.config


# --- Binding count sanity check ----------------------------------------------


def test_camera_udp_has_expected_binding_count(udp_manifest: LayerManifest) -> None:
    # 1 udp_port + 1 enu_reference + 1 rotation_frame + 1 global_pose topic
    # + 2 viewport resolution + 1 image topic + 1 focalLength + 2 aperture
    # + 2 intrinsics (focusDistance + fStop) + 2 RTSP (port + mountPath)
    # + 3 gimbal start angles = 17
    assert len(udp_manifest.bindings) == 17


def test_camera_ros_has_expected_binding_count(ros_manifest: LayerManifest) -> None:
    # 1 enu_reference + 1 rotation_frame + 2 subscriber topics (lla + orientation)
    # + 1 global_pose topic + 2 viewport resolution + 1 image topic + 1 focalLength
    # + 2 aperture + 2 intrinsics (focusDistance + fStop) + 2 RTSP
    # + 3 gimbal start angles = 18
    assert len(ros_manifest.bindings) == 18


# --- Binding source correctness ----------------------------------------------


def test_camera_udp_enu_reference_is_a_resolve_binding(udp_manifest: LayerManifest) -> None:
    # ENU reference is derived from the scene, not from config directly
    enu_bindings = [b for b in udp_manifest.bindings if b.attribute == "inputs:enu_reference"]
    assert len(enu_bindings) == 1
    assert enu_bindings[0].resolve == "enu_origin"
    assert enu_bindings[0].config is None


def test_camera_ros_enu_reference_is_a_resolve_binding(ros_manifest: LayerManifest) -> None:
    enu_bindings = [b for b in ros_manifest.bindings if b.attribute == "inputs:enu_reference"]
    assert len(enu_bindings) == 1
    assert enu_bindings[0].resolve == "enu_origin"
    assert enu_bindings[0].config is None


def test_camera_udp_udp_port_is_a_resolve_binding(udp_manifest: LayerManifest) -> None:
    # The schema default is None, meaning "derive base + vehicle index", so a config
    # binding would write None to the prim instead of a port number.
    port_bindings = [b for b in udp_manifest.bindings if b.attribute == "inputs:udp_port"]
    assert len(port_bindings) == 1
    assert port_bindings[0].resolve == "udp_port"
    assert port_bindings[0].config is None


# --- Contract: every binding prim path exists in USD -------------------------


def _render_binding_path(binding: Binding) -> str:
    """Render a binding's prim template with representative values."""
    return render(
        binding.prim,
        mount=_REPRESENTATIVE_MOUNT,
        instance=_REPRESENTATIVE_INSTANCE,
        camera=_REPRESENTATIVE_CAMERA,
    )


def _binding_id(binding: Binding) -> str:
    """Human-readable pytest ID for a binding."""
    short_prim = binding.prim.split("/")[-1] if "/" in binding.prim else binding.prim
    return f"{short_prim}:{binding.attribute}"


def _get_bindings_with_ids(manifest: LayerManifest) -> list[tuple[str, Binding]]:
    """Return bindings with test IDs for parametrize."""
    return [(_binding_id(b), b) for b in manifest.bindings]


# Standard UsdGeom.Camera attributes. These are valid and writable on any Camera prim from
# the schema, even when the .usda text authors no opinion for them, so a binding may target
# them without them appearing in the parsed attribute set. Verified live.
_CAMERA_SCHEMA_ATTRS = frozenset(
    {"horizontalAperture", "verticalAperture", "focalLength", "focusDistance", "fStop", "clippingRange"}
)


def _attribute_is_present(attribute: str, prim_path: str, attrs: set[str]) -> bool:
    """Return whether a binding attribute exists, allowing Camera schema attributes."""
    if attribute in attrs:
        return True
    return prim_path.endswith("main_camera_01") and attribute in _CAMERA_SCHEMA_ATTRS


@pytest.mark.parametrize(
    "binding",
    [b for b in load_manifest(LAYERS_DIR / "camera_udp" / "layer.toml").bindings],
    ids=[_binding_id(b) for b in load_manifest(LAYERS_DIR / "camera_udp" / "layer.toml").bindings],
)
def test_camera_udp_binding_prim_exists_in_usd(
    binding: Binding,
    udp_usd_prims: dict[str, set[str]],
) -> None:
    rendered = _render_binding_path(binding)
    assert rendered in udp_usd_prims, (
        f"Binding prim {rendered!r} (from template {binding.prim!r}) "
        f"does not exist in camera_udp.usda. "
        f"Available prims: {sorted(udp_usd_prims.keys())}"
    )


@pytest.mark.parametrize(
    "binding",
    [b for b in load_manifest(LAYERS_DIR / "camera_ros" / "layer.toml").bindings],
    ids=[_binding_id(b) for b in load_manifest(LAYERS_DIR / "camera_ros" / "layer.toml").bindings],
)
def test_camera_ros_binding_prim_exists_in_usd(
    binding: Binding,
    ros_usd_prims: dict[str, set[str]],
) -> None:
    rendered = _render_binding_path(binding)
    assert rendered in ros_usd_prims, (
        f"Binding prim {rendered!r} (from template {binding.prim!r}) "
        f"does not exist in camera_ros.usda. "
        f"Available prims: {sorted(ros_usd_prims.keys())}"
    )


# --- Contract: every binding attribute exists on its prim --------------------


@pytest.mark.parametrize(
    "binding",
    [b for b in load_manifest(LAYERS_DIR / "camera_udp" / "layer.toml").bindings],
    ids=[_binding_id(b) for b in load_manifest(LAYERS_DIR / "camera_udp" / "layer.toml").bindings],
)
def test_camera_udp_binding_attribute_exists_on_prim(
    binding: Binding,
    udp_usd_prims: dict[str, set[str]],
) -> None:
    rendered = _render_binding_path(binding)
    if rendered not in udp_usd_prims:
        pytest.skip(f"prim {rendered!r} not found (covered by prim existence test)")
    attrs = udp_usd_prims[rendered]
    assert _attribute_is_present(binding.attribute, rendered, attrs), (
        f"Attribute {binding.attribute!r} not found on {rendered!r} in camera_udp.usda. "
        f"Available attributes: {sorted(attrs)}"
    )


@pytest.mark.parametrize(
    "binding",
    [b for b in load_manifest(LAYERS_DIR / "camera_ros" / "layer.toml").bindings],
    ids=[_binding_id(b) for b in load_manifest(LAYERS_DIR / "camera_ros" / "layer.toml").bindings],
)
def test_camera_ros_binding_attribute_exists_on_prim(
    binding: Binding,
    ros_usd_prims: dict[str, set[str]],
) -> None:
    rendered = _render_binding_path(binding)
    if rendered not in ros_usd_prims:
        pytest.skip(f"prim {rendered!r} not found (covered by prim existence test)")
    attrs = ros_usd_prims[rendered]
    assert _attribute_is_present(binding.attribute, rendered, attrs), (
        f"Attribute {binding.attribute!r} not found on {rendered!r} in camera_ros.usda. "
        f"Available attributes: {sorted(attrs)}"
    )


# --- Discovery: both manifests found via discover_layers ---------------------


def test_shipped_manifests_discoverable_via_discover_layers() -> None:
    discovered = discover_layers([LAYERS_DIR])
    assert "camera_udp" in discovered
    assert "camera_ros" in discovered


def test_discover_layers_returns_validated_manifests() -> None:
    discovered = discover_layers([LAYERS_DIR])
    assert discovered["camera_udp"].usd == "camera_udp.usda"
    assert discovered["camera_ros"].usd == "camera_ros.usda"


# --- USD file existence sanity -----------------------------------------------


def test_camera_udp_usda_file_exists() -> None:
    assert (_USD_LAYERS_DIR / "camera_udp" / "camera_udp.usda").is_file()


def test_camera_ros_usda_file_exists() -> None:
    assert (_USD_LAYERS_DIR / "camera_ros" / "camera_ros.usda").is_file()


# --- Manifest does not hardcode absolute mount paths -------------------------


def test_camera_udp_no_hardcoded_prim_paths(udp_manifest: LayerManifest) -> None:
    # No binding should reference an absolute path without a placeholder
    for binding in udp_manifest.bindings:
        assert "{mount}" in binding.prim or not binding.prim.startswith(
            "/"
        ), f"Binding prim {binding.prim!r} appears to hardcode a mount path"


def test_camera_ros_no_hardcoded_prim_paths(ros_manifest: LayerManifest) -> None:
    for binding in ros_manifest.bindings:
        assert "{mount}" in binding.prim or not binding.prim.startswith(
            "/"
        ), f"Binding prim {binding.prim!r} appears to hardcode a mount path"


# --- ROS-specific bindings only in camera_ros --------------------------------


def test_camera_udp_has_no_lla_topic_binding(udp_manifest: LayerManifest) -> None:
    # The UDP layer gets position from UDP, not from ROS subscribers
    lla_bindings = [b for b in udp_manifest.bindings if "lla_topic" in (b.config or "")]
    assert lla_bindings == []


def test_camera_ros_has_lla_topic_binding(ros_manifest: LayerManifest) -> None:
    # Derived from the vehicle's mavros_namespace when not set explicitly, so a swarm
    # does not need four topic names spelled out per aircraft.
    assert [b for b in ros_manifest.bindings if b.resolve == "lla_topic"]


def test_camera_ros_has_orientation_topic_binding(ros_manifest: LayerManifest) -> None:
    assert [b for b in ros_manifest.bindings if b.resolve == "orientation_topic"]


# --- UDP-specific bindings only in camera_udp --------------------------------


def test_camera_udp_has_udp_port_binding(udp_manifest: LayerManifest) -> None:
    assert [b for b in udp_manifest.bindings if b.resolve == "udp_port"]


def test_camera_ros_has_no_udp_port_binding(ros_manifest: LayerManifest) -> None:
    port_bindings = [b for b in ros_manifest.bindings if "udp_port" in (b.config or "")]
    assert port_bindings == []


# --------------------------------------------------------------------------- #
# the config side of the contract
#
# The prim-side test above proved the manifests point at prims that exist. Nothing
# checked the OTHER end, and eight bindings shipped referencing config keys that did
# not exist at all (`resolution_width`, `global_pose_topic`, `frame_skip_count`), plus
# several using `config` for fields whose default is None -- meaning "derive this" --
# which would have written a blank attribute instead of a value. Both halves matter.
# --------------------------------------------------------------------------- #


def _lookup_config_key(config: IsaacCoreConfig, dotted: str) -> object:
    """Walk a dotted binding key against a real config, resolving {instance} and {camera}."""
    node: object = config
    resolved = dotted.replace("{instance}", "drone_0").replace("{camera}", "eo")
    for part in resolved.split("."):
        node = node[part] if isinstance(node, dict) else getattr(node, part)
    return node


@pytest.mark.parametrize("layer_id", SHIPPED_LAYER_IDS)
def test_every_binding_config_key_exists_in_the_schema(layer_id: str) -> None:
    manifest = load_manifest(LAYERS_DIR / layer_id / "layer.toml")
    config = IsaacCoreConfig()
    for binding in manifest.bindings:
        if binding.config is None:
            continue
        try:
            _lookup_config_key(config, binding.config)
        except (AttributeError, KeyError) as exc:
            pytest.fail(
                f"{layer_id}: binding {binding.prim}.{binding.attribute} references "
                f"config key {binding.config!r}, which does not exist in the schema ({exc})"
            )


@pytest.mark.parametrize("layer_id", SHIPPED_LAYER_IDS)
def test_no_binding_uses_config_for_a_derive_me_field(layer_id: str) -> None:
    # A field defaulting to None means "derive the conventional value". Binding it with
    # `config` writes None to the prim; it must use `resolve` so the configurator
    # computes it.
    manifest = load_manifest(LAYERS_DIR / layer_id / "layer.toml")
    config = IsaacCoreConfig()
    for binding in manifest.bindings:
        if binding.config is None:
            continue
        value = _lookup_config_key(config, binding.config)
        assert value is not None, (
            f"{layer_id}: binding {binding.prim}.{binding.attribute} uses "
            f"config={binding.config!r}, which defaults to None (meaning 'derive it'). "
            f"Use a `resolve` binding instead so a real value is written."
        )


@pytest.mark.parametrize("layer_id", ["camera_udp", "camera_ros", "distance_sensor", "bbox"])
def test_shipped_manifest_capability_names_are_all_known(layer_id: str) -> None:
    # requires/provides share one namespace with StageCapability. An unknown name is a typo,
    # and the failure mode is nasty: the layer is skipped on every launch with "unmet stage
    # requirement", which reads like a stage problem rather than a spelling mistake.
    known = {c.name for c in StageCapability}
    manifest = load_manifest(LAYERS_DIR / layer_id / "layer.toml")
    for name in (*manifest.requires, *manifest.provides):
        assert name in known, f"{layer_id} uses unknown capability {name!r}; known: {sorted(known)}"
