"""Derive the local ENU reference from the scene's Cesium georeference.

``geo.enu_reference`` and the scene's ``cesium:georeferenceOrigin`` describe the same
thing: the geodetic point that the stage origin corresponds to. If they disagree, the
aircraft and the terrain disagree about where they are, and the camera simply flies over
the wrong ground, or over nothing.

Keeping two copies of one fact in agreement by hand is exactly the class of problem this
repo exists to remove, so composition reads the scene and derives the reference instead
of asking anyone to remember. When config *does* pin the reference explicitly, that value
is authoritative -- but only if it agrees with the scene. A real disagreement is no longer
a warning that scrolls past unread: :func:`resolve_enu_reference` raises and halts
composition, because a mispositioned aircraft over the wrong terrain is a bug that is
almost impossible to trace back to a stale config value.

Why not go further and write latitude/longitude straight onto a Cesium Globe Anchor,
skipping ENU entirely? It was investigated and rejected: Cesium reacts to USD notices
while OmniGraph writers target Fabric, so the write is invisible unless USD is authored
every frame; Cesium ships no OmniGraph nodes, so there is no supported wiring; and it
would couple the pose pipeline to Cesium, breaking non-Cesium stages. Reading the origin
once at composition captures the benefit without any of that.
"""

from dataclasses import dataclass
import math

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

# Horizontal agreement tolerance, in metres. Cesium's origin round-trips through USD and its own
# ellipsoid maths, so bit-exact equality is unrealistic and a sub-metre gap is invisible from
# hundreds of metres up. Anything larger means the two point at different places, and must be loud.
HORIZONTAL_TOLERANCE_M = 1.0

# Mean Earth radius (WGS84 mean), metres. Used only to turn a small angular separation into
# an intuitive horizontal distance for the error message; sub-percent accuracy is ample for
# deciding whether two origins agree.
_EARTH_RADIUS_M = 6_371_008.8


class GeoreferenceMismatchError(RuntimeError):
    """Raise when config and the scene name different geodetic origins.

    Carrying the two references and their separation lets callers log or re-raise with a
    message that says exactly what to change, rather than a bare failure.

    Args:
        configured: The explicitly configured ENU reference.
        scene: The origin read from the scene's Cesium georeference.
        separation_m: Horizontal distance between the two, in metres.
        prim_path: Path of the Cesium georeference prim.

    """

    def __init__(
        self,
        configured: EnuReference,
        scene: EnuReference,
        separation_m: float,
        prim_path: str,
    ) -> None:
        """Build the error and its human-readable message."""
        self.configured = configured
        self.scene = scene
        self.separation_m = separation_m
        self.prim_path = prim_path
        super().__init__(
            f"georeference mismatch: geo.enu_reference is set explicitly to "
            f"lat={configured.lat_deg}, lon={configured.lon_deg} "
            f"but the scene's {prim_path} origin is "
            f"lat={scene.lat_deg}, lon={scene.lon_deg} -- "
            f"{separation_m:.1f} m apart on the ground. "
            f"The aircraft would be positioned relative to one point and the terrain drawn "
            f"around the other. Edit geo.enu_reference to match the scene, re-author the "
            f"scene's {prim_path} origin to match config, or remove geo.enu_reference to "
            f"adopt the scene's origin automatically."
        )


@dataclass(frozen=True, slots=True)
class ResolvedEnuReference:
    """An ENU reference together with where it came from.

    Args:
        reference: The geodetic anchor for the local ENU frame.
        source: Human-readable provenance, for logging and `config explain`.

    """

    reference: EnuReference
    source: str


def horizontal_separation_m(a: EnuReference, b: EnuReference) -> float:
    """Return the great-circle ground distance between two references, in metres.

    Altitude is ignored on purpose: a georeference disagreement is about *where on Earth*
    the origin sits, and reporting the horizontal gap in metres is far more intuitive than
    raw degrees. The haversine formula stays accurate at the sub-metre separations that
    matter here, where a flat-Earth approximation would already do.

    Args:
        a: One reference.
        b: The other reference.

    Returns:
        The horizontal distance in metres.

    """
    lat1 = math.radians(a.lat_deg)
    lat2 = math.radians(b.lat_deg)
    d_lat = lat2 - lat1
    d_lon = math.radians(b.lon_deg - a.lon_deg)
    h = math.sin(d_lat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2) ** 2
    return 2 * _EARTH_RADIUS_M * math.asin(math.sqrt(h))


def read_scene_georeference(
    inspector: StageInspector,
    prim_path: str = CESIUM_GEOREFERENCE_PRIM,
) -> EnuReference | None:
    """Read the Cesium georeference origin from the open stage.

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
    tolerance_m: float = HORIZONTAL_TOLERANCE_M,
) -> ResolvedEnuReference:
    """Decide which ENU reference a run should use, refusing a silent disagreement.

    There is one authoritative source, chosen by an explicit rule:

    1. An ``enu_reference`` explicitly set in config is authoritative. An explicit value
       is an instruction, and someone overriding it usually has a reason -- a stage with no
       Cesium georeference, or a deliberate offset while debugging. But it must *agree*
       with the scene: if the scene also carries a georeference and the two point more than
       ``tolerance_m`` apart on the ground, that is the exact "terrain here, aircraft there"
       bug this module exists to prevent, and :class:`GeoreferenceMismatchError` is raised
       rather than one silently winning.
    2. Otherwise the scene's Cesium georeference origin is used.
    3. Otherwise the schema default is used.

    Args:
        config: The resolved configuration.
        inspector: Stage query interface.
        prim_path: Path of the Cesium georeference prim.
        tolerance_m: Horizontal separation, in metres, treated as agreement.

    Returns:
        The reference to use, with its provenance.

    Raises:
        GeoreferenceMismatchError: When an explicit config reference and the scene's
            georeference disagree by more than ``tolerance_m``.

    """
    from_scene = read_scene_georeference(inspector, prim_path)

    if "enu_reference" in config.geo.model_fields_set:
        configured = config.geo.enu_reference
        if from_scene is not None:
            separation_m = horizontal_separation_m(configured, from_scene)
            if separation_m > tolerance_m:
                raise GeoreferenceMismatchError(configured, from_scene, separation_m, prim_path)
        return ResolvedEnuReference(configured, SOURCE_CONFIG)

    if from_scene is not None:
        return ResolvedEnuReference(from_scene, SOURCE_SCENE)

    return ResolvedEnuReference(config.geo.enu_reference, SOURCE_DEFAULT)


def describe_mismatch(
    resolved: ResolvedEnuReference,
    inspector: StageInspector,
    prim_path: str = CESIUM_GEOREFERENCE_PRIM,
    tolerance_m: float = HORIZONTAL_TOLERANCE_M,
) -> str | None:
    """Return a message when an explicit reference disagrees with the scene.

    Retained so callers can report an in-tolerance-but-nonzero gap if they want to, and so
    the composer's existing warning call site stays valid. A disagreement large enough to
    matter never reaches here: :func:`resolve_enu_reference` has already raised on it. This
    therefore only ever returns ``None`` for a resolved reference produced by that function,
    but it re-checks independently so it is safe to call on any :class:`ResolvedEnuReference`.

    Args:
        resolved: The outcome of :func:`resolve_enu_reference`.
        inspector: Stage query interface.
        prim_path: Path of the Cesium georeference prim.
        tolerance_m: Horizontal separation, in metres, treated as agreement.

    Returns:
        A human-readable message, or ``None`` if there is nothing to report.

    """
    if resolved.source != SOURCE_CONFIG:
        return None

    from_scene = read_scene_georeference(inspector, prim_path)
    if from_scene is None:
        return None

    separation_m = horizontal_separation_m(resolved.reference, from_scene)
    if separation_m <= tolerance_m:
        return None

    used = resolved.reference
    return (
        f"geo.enu_reference is set explicitly to "
        f"({used.lat_deg}, {used.lon_deg}, {used.alt_m}) but the scene's "
        f"{prim_path} origin is "
        f"({from_scene.lat_deg}, {from_scene.lon_deg}, {from_scene.alt_m}), "
        f"{separation_m:.1f} m apart on the ground."
    )


__all__ = [
    "CESIUM_GEOREFERENCE_PRIM",
    "HEIGHT_ATTRIBUTE",
    "HORIZONTAL_TOLERANCE_M",
    "LATITUDE_ATTRIBUTE",
    "LONGITUDE_ATTRIBUTE",
    "GeoreferenceMismatchError",
    "ResolvedEnuReference",
    "SOURCE_CONFIG",
    "SOURCE_DEFAULT",
    "SOURCE_SCENE",
    "describe_mismatch",
    "horizontal_separation_m",
    "read_scene_georeference",
    "resolve_enu_reference",
]
