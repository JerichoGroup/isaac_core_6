"""
USD prim path construction and templating.

Layer manifests declare prim paths as templates containing placeholders, which are
substituted at compose time::

    "{mount}/ActionGraph/ros2_distance_publisher"
    "/Environment/{instance}/thermal"

This is what keeps prim paths out of Python entirely. The previous generation
scattered literals like
``/Environment/distance_sensor/ActionGraph/ros2_distance_publisher`` through its
simulation class, so any rename in the USD silently broke configuration with no
error until runtime. Now a layer author writes the path once in a manifest, and a
contract test asserts every declared path actually exists in the shipped USD.

Rendering is pure string work -- no ``pxr`` import -- so manifests can be validated
in CI without Isaac Sim present.
"""

import re
import string
from typing import Final

# Conventional mount point for composed feature layers. Under /World, because USD
# convention is a single /World default prim and the authored scenes follow it. The
# 2023 repo rooted its scenes at /Environment; mounting there now would create a
# sibling of /World, outside the scene graph.
ENVIRONMENT_ROOT: Final = "/World/Environment"

# Where Cesium tileset prims are expected. Absence is a capability, not an error.
TILESETS_ROOT: Final = "/World/tilesets"

# Where prims eligible for bounding-box reporting are expected.
BBOXES_ROOT: Final = "/World/bboxes"

# Placeholder for a layer's mount point.
MOUNT: Final = "mount"

# Placeholder for a vehicle or layer instance id.
INSTANCE: Final = "instance"

_IDENTIFIER_RE: Final = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def is_valid_prim_path(path: str) -> bool:
    """
    Report whether ``path`` is a well-formed absolute USD prim path.

    Checks the shape only: absolute, no empty or repeated separators, and every
    element a valid USD identifier. It says nothing about whether the prim exists.

    Args:
        path: Candidate prim path.

    Returns:
        ``True`` if the path is well formed.

    """
    if not path.startswith("/") or path == "/":
        return False
    if "//" in path or path.endswith("/"):
        return False
    return all(_IDENTIFIER_RE.match(element) for element in path[1:].split("/"))


def validate_prim_path(path: str) -> str:
    """
    Check that ``path`` is a well-formed absolute USD prim path.

    Args:
        path: Candidate prim path.

    Returns:
        The path unchanged, for convenient inline use.

    Raises:
        ValueError: If the path is not well formed.

    """
    if not is_valid_prim_path(path):
        msg = (
            f"invalid prim path {path!r}: expected an absolute path such as "
            f"'/Environment/drone_0', with identifier-safe elements"
        )
        raise ValueError(msg)
    return path


def placeholders(template: str) -> frozenset[str]:
    """
    Return the set of placeholder names appearing in ``template``.

    Lets the manifest loader reject a template referencing an unknown placeholder
    at load time, rather than producing a malformed path much later.

    Args:
        template: A prim path template, possibly containing ``{name}`` fields.

    Returns:
        The placeholder names found, without braces.

    """
    return frozenset(field for _, field, _, _ in string.Formatter().parse(template) if field)


def render(template: str, **substitutions: str) -> str:
    """
    Substitute placeholders in a prim path template and validate the result.

    Args:
        template: Prim path template, for example ``"{mount}/ActionGraph/node"``.
        **substitutions: Placeholder values, for example ``mount="/Environment/eo"``.

    Returns:
        A validated absolute prim path.

    Raises:
        ValueError: If a placeholder has no value supplied, or if the rendered path
            is not a well-formed prim path.

    """
    missing = placeholders(template) - substitutions.keys()
    if missing:
        msg = f"prim path template {template!r} is missing values for: " f"{', '.join(sorted(missing))}"
        raise ValueError(msg)

    rendered = template.format(**substitutions)
    while "//" in rendered:
        rendered = rendered.replace("//", "/")
    if len(rendered) > 1:
        rendered = rendered.rstrip("/")

    return validate_prim_path(rendered)


def child(parent: str, *elements: str) -> str:
    """
    Return the path of a descendant prim.

    Args:
        parent: Absolute path of the ancestor prim.
        *elements: Path elements to append, outermost first.

    Returns:
        A validated absolute prim path.

    Raises:
        ValueError: If ``parent`` or the result is not well formed.

    """
    validate_prim_path(parent)
    if not elements:
        return parent
    return validate_prim_path(f"{parent.rstrip('/')}/" + "/".join(elements))


__all__ = [
    "BBOXES_ROOT",
    "ENVIRONMENT_ROOT",
    "INSTANCE",
    "MOUNT",
    "TILESETS_ROOT",
    "child",
    "is_valid_prim_path",
    "placeholders",
    "render",
    "validate_prim_path",
]
