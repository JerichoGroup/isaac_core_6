"""OmniGraph node: convert intrinsic-XYZ Euler angles in radians to a quaternion.

Thin adapter over :func:`isaac_core.geo.euler_to_quaternion`, which is unit tested
without Isaac Sim. This module contains no maths of its own.

Exists because ``GlobalPositionToLocalPosition`` emits ``global_orientation`` as a
``vectord[3]`` of radians, while Isaac's ``isaacsim.ros2.bridge.ROS2Publisher``
expects a ``GeoPoseStamped`` orientation as four separate ``double`` inputs.

A note on the output convention, because it is a deliberate change: the previous
generation published roll, pitch and yaw stuffed directly into the quaternion's x, y
and z fields with w pinned to 1.0, so ``GeoPoseStamped.pose.orientation`` did not
actually contain a quaternion. This node emits a real one. Anything written against
the old convention needs updating -- and wiring roll/pitch/yaw straight into the
publisher's x/y/z reproduces the old behaviour deliberately if that is wanted.
"""

import carb
from isaac_core_ogn.math.ogn.OgnEulerToQuaternionDatabase import OgnEulerToQuaternionDatabase

from isaac_core.geo import euler_to_quaternion

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | E2Q |"

# Number of components in the Euler input vector.
_EULER_COMPONENTS = 3


class OgnEulerToQuaternion:
    """OmniGraph node: intrinsic-XYZ Euler angles in radians to a quaternion."""

    @staticmethod
    def compute(db: OgnEulerToQuaternionDatabase) -> bool:
        """Convert the input Euler angles to a quaternion and write every output form.

        Args:
            db: OmniGraph node database.

        Returns:
            ``True`` always; a malformed input is logged and skipped rather than
            failing the graph, so one bad tick cannot stop the simulation.

        """
        euler = db.inputs.euler_r
        if euler is None or len(euler) != _EULER_COMPONENTS:
            carb.log_warn(f"{_LOG_PREFIX} Expected 3 Euler components, got {euler!r}")
            return True

        roll_r, pitch_r, yaw_r = (float(component) for component in euler)

        try:
            qw, qx, qy, qz = euler_to_quaternion(roll_r, pitch_r, yaw_r)
        except ValueError as exc:
            carb.log_warn(f"{_LOG_PREFIX} Could not convert Euler angles: {exc}")
            return True

        db.outputs.qw = qw
        db.outputs.qx = qx
        db.outputs.qy = qy
        db.outputs.qz = qz

        # Isaac Sim stores quatd attributes in IJKR order [x, y, z, w], not the
        # (w, x, y, z) that transforms3d and the Isaac GUI both display. Verified on
        # Kit 110 against three .ogn files in the install. Same convention as
        # GlobalPositionToLocalPosition.local_orientation.
        db.outputs.quaternion = [qx, qy, qz, qw]
        return True
