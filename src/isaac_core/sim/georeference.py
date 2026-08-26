"""
Derive the local ENU reference from the scene's Cesium georeference.

``geo.enu_reference`` and the scene's ``cesium:georeferenceOrigin`` describe the same
thing: the geodetic point that the stage origin corresponds to. If they disagree, the
aircraft and the terrain disagree about where they are, and nothing reports it -- the
camera simply flies over the wrong ground, or over nothing.

Keeping two copies of one fact in agreement by hand is exactly the class of problem
this repo exists to remove, so composition reads the scene and derives the reference
instead of asking anyone to remember. An explicitly configured value still wins, which
matters for stages that have no Cesium georeference at all.

Why not go further and write latitude/longitude straight onto a Cesium Globe Anchor,
skipping ENU entirely? It was investigated and rejected: Cesium reacts to USD notices
while OmniGraph writers target Fabric, so the write is invisible unless USD is authored
every frame; Cesium ships no OmniGraph nodes, so there is no supported wiring; and it
would couple the pose pipeline to Cesium, breaking non-Cesium stages. Reading the origin
once at composition captures the benefit without any of that.
"""

from dataclasses import dataclass

from isaac_core.config import EnuReference, IsaacCoreConfig
from isaac_core.sim.capabilities import StageInspector

# Conventional path of the Cesium georeference prim. Cesium creates it here, and both
# the scene and each authored layer may carry one.
CESIUM_GEOREFERENCE_PRIM = "/CesiumGeoreference"

# Cesium's georeference origin attribute names.
LATITUDE_ATTRIBUTE = "cesium:georeferenceOrigin:latitude"
LONGITUDE_ATTRIBUTE = "cesium:georeferenceOrigin:longitude"
HEIGHT_ATTRIBUTE = "cesium:georeferenceOrigin:height"

# Provenance labels, mirroring the wording used by the config loader so
# `isaac-core config explain` reads consistently wherever a value came from.
SOURCE_CONFIG = "config (explicit)"
SOURCE_SCENE = f"scene ({CESIUM_GEOREFERENCE_PRIM})"
SOURCE_DEFAULT = "schema default"


@dataclass(frozen=True, slots=True)
class ResolvedEnuReference:
    """
    An ENU reference together with where it came from.

    Args:
        reference: The geodetic anchor for the local ENU frame.
        source: Human-readable provenance, for logging and `config explain`.

    """

    reference: EnuReference
    source: str


def read_scene_georeference(
    inspector: StageInspector,
    prim_path: str = CESIUM_GEOREFERENCE_PRIM,
) -> EnuReference | None:
    """
    Read the Cesium georeference origin from the open stage.

    Args:
        inspector: Stage query interface.
        prim_path: Path of the Cesium georeference prim.

    Returns:
        The origin as an :class:`~isaac_core.config.EnuReference`, or ``None`` if the
        prim is absent or does not carry a complete origin. A partial origin counts as
        absent: guessing the missing component would silently misplace the aircraft.

    """
    if not inspector.prim_exists(prim_path):
        return None

    latitude = inspector.read_double(prim_path, LATITUDE_ATTRIBUTE)
    longitude = inspector.read_double(prim_path, LONGITUDE_ATTRIBUTE)
    height = inspector.read_double(prim_path, HEIGHT_ATTRIBUTE)

    if latitude is None or longitude is None or height is None:
        return None

    return EnuReference(lat_deg=latitude, lon_deg=longitude, alt_m=height)


def resolve_enu_reference(
    config: IsaacCoreConfig,
    inspector: StageInspector,
    prim_path: str = CESIUM_GEOREFERENCE_PRIM,
) -> ResolvedEnuReference:
    """
    Decide which ENU reference a run should use.

    Precedence, highest first:

    1. An ``enu_reference`` explicitly set in config. An explicit value is an
       instruction, and someone overriding it usually has a reason -- a stage with no
       Cesium georeference, or a deliberate offset while debugging.
    2. The scene's Cesium georeference origin.
    3. The schema default.

    Args:
        config: The resolved configuration.
        inspector: Stage query interface.
        prim_path: Path of the Cesium georeference prim.

    Returns:
        The reference to use, with its provenance.

    """
    if "enu_reference" in config.geo.model_fields_set:
        return ResolvedEnuReference(config.geo.enu_reference, SOURCE_CONFIG)

    from_scene = read_scene_georeference(inspector, prim_path)
    if from_scene is not None:
        return ResolvedEnuReference(from_scene, SOURCE_SCENE)

    return ResolvedEnuReference(config.geo.enu_reference, SOURCE_DEFAULT)


def describe_mismatch(
    resolved: ResolvedEnuReference,
    inspector: StageInspector,
    prim_path: str = CESIUM_GEOREFERENCE_PRIM,
    tolerance_deg: float = 1e-6,
) -> str | None:
    """
    Return a warning if an explicitly configured reference disagrees with the scene.

    An explicit override is honoured, but a *silent* disagreement is the failure this
    module exists to prevent, so it is worth saying out loud. Roughly 1e-6 degrees is
    about 0.1 m, well inside anything an operator would set deliberately.

    Args:
        resolved: The outcome of :func:`resolve_enu_reference`.
        inspector: Stage query interface.
        prim_path: Path of the Cesium georeference prim.
        tolerance_deg: Angular difference treated as agreement.

    Returns:
        A human-readable warning, or ``None`` if there is nothing to report.

    """
    if resolved.source != SOURCE_CONFIG:
        return None

    from_scene = read_scene_georeference(inspector, prim_path)
    if from_scene is None:
        return None

    used = resolved.reference
    if (
        abs(used.lat_deg - from_scene.lat_deg) <= tolerance_deg
        and abs(used.lon_deg - from_scene.lon_deg) <= tolerance_deg
    ):
        return None

    return (
        f"geo.enu_reference is set explicitly to "
        f"({used.lat_deg}, {used.lon_deg}, {used.alt_m}) but the scene's "
        f"{prim_path} origin is "
        f"({from_scene.lat_deg}, {from_scene.lon_deg}, {from_scene.alt_m}). "
        f"The explicit value is being used. Unless this is deliberate, the aircraft "
        f"and the terrain will disagree about where they are."
    )


__all__ = [
    "CESIUM_GEOREFERENCE_PRIM",
    "HEIGHT_ATTRIBUTE",
    "LATITUDE_ATTRIBUTE",
    "LONGITUDE_ATTRIBUTE",
    "ResolvedEnuReference",
    "SOURCE_CONFIG",
    "SOURCE_DEFAULT",
    "SOURCE_SCENE",
    "describe_mismatch",
    "read_scene_georeference",
    "resolve_enu_reference",
]
