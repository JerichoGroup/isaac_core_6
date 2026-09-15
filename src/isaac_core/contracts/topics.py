"""ROS 2 topic naming and namespacing.

Topic names are built here and nowhere else. Re-declaring a string like
``/isaac_core/image_rgb`` per consumer means renaming a
topic meant finding every copy. Everything now derives from :class:`TopicResolver`.

Namespacing collapses when there is only one of something, so the common
single-vehicle single-camera case produces exactly the flat names the team already
knows::

    /isaac_core/image_rgb                    one vehicle, one camera
    /isaac_core/eo/image_rgb                 one vehicle, two cameras
    /isaac_core/lead/eo/image_rgb            several vehicles

Whether to collapse a level is decided by the config layer, which knows how many
vehicles and cameras exist; this module just omits any segment given as ``None``.
"""

from dataclasses import dataclass, replace
import re
from typing import Final

# Namespace root for every topic this project owns.
ROOT: Final = "/isaac_core"

IMAGE_RGB: Final = "image_rgb"
GLOBAL_POSE: Final = "global_pose"
DISTANCE_SENSOR: Final = "distance_sensor"
BBOX: Final = "bbox"

# Default MAVROS topics, relative to a vehicle's MAVROS namespace.
# Paths below a vehicle's ``mavros_namespace``. The full topic is derived per vehicle, so the
# leaf is the reusable part -- a hardcoded /mavros/... constant cannot serve a swarm.
MAVROS_LLA_LEAF: Final = "global_position/global"
MAVROS_ORIENTATION_LEAF: Final = "local_position/pose"

_SEGMENT_RE: Final = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def validate_segment(segment: str) -> str:
    """Check that a single topic path segment is a legal ROS 2 name token.

    Args:
        segment: One segment, with no slashes.

    Returns:
        The segment unchanged, for convenient inline use.

    Raises:
        ValueError: If the segment is empty, contains a slash, or does not match
            the ROS 2 token rules (letters, digits and underscores, not starting
            with a digit).

    """
    if not _SEGMENT_RE.match(segment):
        msg = (
            f"invalid topic segment {segment!r}: expected letters, digits and "
            f"underscores, not starting with a digit"
        )
        raise ValueError(msg)
    return segment


def join(root: str, *segments: str | None) -> str:
    """Join a root and any number of segments into a topic name.

    Segments that are ``None`` are omitted, which is how namespace levels collapse
    for the single-vehicle and single-camera cases.

    Args:
        root: Absolute namespace root, beginning with ``/``.
        *segments: Segments to append; ``None`` entries are skipped.

    Returns:
        An absolute topic name.

    Raises:
        ValueError: If ``root`` is not absolute, or a segment is not a legal token.

    """
    if not root.startswith("/"):
        msg = f"topic root must be absolute, got {root!r}"
        raise ValueError(msg)

    present = [validate_segment(segment) for segment in segments if segment is not None]
    if not present:
        return root
    return f"{root.rstrip('/')}/" + "/".join(present)


@dataclass(frozen=True, slots=True)
class TopicResolver:
    """Builds the topic names for one vehicle, and optionally one of its cameras.

    Args:
        root: Namespace root. Rarely changed, but configurable for teams running
            several independent simulators on one ROS domain.
        vehicle: Vehicle segment, or ``None`` to omit it when there is only one
            vehicle.
        camera: Camera segment, or ``None`` to omit it when there is only one
            camera.

    """

    root: str = ROOT
    vehicle: str | None = None
    camera: str | None = None

    def resolve(self, leaf: str) -> str:
        """Return the full topic name for ``leaf`` under this resolver's namespace."""
        return join(self.root, self.vehicle, self.camera, leaf)

    def vehicle_scoped(self, leaf: str) -> str:
        """Return a topic scoped to the vehicle, ignoring any camera segment.

        Used for per-vehicle rather than per-camera data -- pose, range, gimbal --
        which should not sit underneath a camera in the namespace.
        """
        return join(self.root, self.vehicle, leaf)

    def for_camera(self, camera: str | None) -> "TopicResolver":
        """Return a copy of this resolver scoped to ``camera``."""
        return replace(self, camera=camera)


__all__ = [
    "BBOX",
    "DISTANCE_SENSOR",
    "GLOBAL_POSE",
    "IMAGE_RGB",
    "MAVROS_LLA_LEAF",
    "MAVROS_ORIENTATION_LEAF",
    "ROOT",
    "TopicResolver",
    "join",
    "validate_segment",
]
