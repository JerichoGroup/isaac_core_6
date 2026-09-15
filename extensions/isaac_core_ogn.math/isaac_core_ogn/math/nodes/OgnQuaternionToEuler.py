"""OmniGraph node: convert a quaternion to intrinsic-XYZ Euler angles in radians.

Thin adapter over :func:`isaac_core.geo.quaternion_to_euler`, which is unit tested
without Isaac Sim. This module contains no maths of its own.

Exists because Isaac's ``isaacsim.ros2.bridge.ROS2Subscriber`` exposes a
``PoseStamped`` orientation as four separate ``double`` outputs, while
``GlobalPositionToLocalPosition`` expects ``global_orientation`` as a ``vectord[3]``
of radians. Nothing in either the bridge or OmniGraph's stock nodes bridges that gap.

No frame conversion happens here. MAVROS publishes ``PoseStamped`` orientation
already in ENU, which is what ``GlobalPositionToLocalPosition`` wants. The UDP path
converts NED to ENU inside ``UdpToGlobalPosition``, so neither path converts twice.
"""

import carb
from isaac_core_ogn.math.ogn.OgnQuaternionToEulerDatabase import OgnQuaternionToEulerDatabase

from isaac_core.geo import quaternion_to_euler

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | Q2E |"


class OgnQuaternionToEuler:
    """OmniGraph node: quaternion to intrinsic-XYZ Euler angles in radians."""

    @staticmethod
    def compute(db: OgnQuaternionToEulerDatabase) -> bool:
        """Convert the input quaternion to Euler angles and write every output form.

        Args:
            db: OmniGraph node database.

        Returns:
            ``True`` always; a degenerate quaternion is normalised rather than
            treated as an error, because a subscriber that has not yet received a
            message legitimately reports all zeros.

        """
        qw = float(db.inputs.qw)
        qx = float(db.inputs.qx)
        qy = float(db.inputs.qy)
        qz = float(db.inputs.qz)

        # An all-zero quaternion is not a rotation, and it is exactly what a ROS 2
        # subscriber reports before its first message arrives. Treat it as identity
        # rather than letting it propagate as NaN through the pose pipeline.
        if qw == 0.0 and qx == 0.0 and qy == 0.0 and qz == 0.0:
            qw = 1.0

        try:
            roll_r, pitch_r, yaw_r = quaternion_to_euler(qw, qx, qy, qz)
        except (ValueError, ZeroDivisionError) as exc:
            carb.log_warn(f"{_LOG_PREFIX} Could not convert quaternion: {exc}")
            return True

        db.outputs.euler_r = [roll_r, pitch_r, yaw_r]
        db.outputs.roll_r = roll_r
        db.outputs.pitch_r = pitch_r
        db.outputs.yaw_r = yaw_r
        return True
