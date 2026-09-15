"""Coordinate-frame, rotation and pose-source conventions.

Two geographic conventions are in play at once, and conflating them is the single most
common source of orientation bugs here:

``NED`` (North-East-Down)
    The aerospace convention. Used on the wire and in every user-facing API.
    ``+roll`` drops the right wing, ``+pitch`` raises the nose, ``+yaw`` turns the
    nose right.

``ENU`` (East-North-Up)
    What Isaac Sim and Cesium use internally.

All Euler angles in this codebase are **intrinsic XYZ** -- the ``transforms3d``
axes string ``rxyz`` -- and are carried in **radians**. Degrees appear only at the
edges, in user-facing APIs and config, where every such name is suffixed ``_deg``
so the unit is never ambiguous at a call site.
"""

from enum import Enum
from typing import Final

# Euler convention used everywhere: intrinsic (rotating-frame) XYZ.
EULER_AXES: Final = "rxyz"


class Frame(str, Enum):
    """Geographic reference frame that a set of angles or offsets is expressed in."""

    NED = "ned"
    ENU = "enu"


class RotationFrame(str, Enum):
    """Frame in which a roll/pitch/yaw delta is applied to an existing attitude.

    The distinction is which side of the composition the delta lands on, and it is
    directly observable: commanding ``+yaw`` on a vehicle that is banked steeply
    produces visibly different motion under the two settings.

    ``WORLD``
        Extrinsic. The delta is applied about fixed world axes::

            R_result = R_delta @ R_current

        ``+yaw`` always turns about the world vertical, whatever the current
        attitude. The natural choice for a movement command such as a turn.

    ``BODY``
        Intrinsic. The delta is applied about the vehicle's own axes::

            R_result = R_current @ R_delta

        The natural choice for a gimbal, which is physically mounted on the
        airframe and so moves with it (``qmult(q_drone, q_offset)``).

    Both behaviours existed before but neither was selectable; this enum makes the
    choice explicit and configurable.
    """

    WORLD = "world"
    BODY = "body"


class PoseSource(str, Enum):
    """Where a vehicle's pose comes from.

    A single axis rather than a set of mutually-exclusive booleans, so a new source can be
    added without reworking the surface. Each vehicle chooses independently, which means a
    lead aircraft can fly on real physics while its wingmen are placed by script.

    ``UDP``
        Pose packets on a UDP port. Used when physics does not matter and you just
        want the camera placed somewhere, or flown along a path.
    ``ROS``
        ``NavSatFix`` + ``PoseStamped``, typically from MAVROS in front of an
        ArduCopter SITL instance. Used when physics does matter.
    """

    UDP = "udp"
    ROS = "ros"


__all__ = ["EULER_AXES", "Frame", "PoseSource", "RotationFrame"]
