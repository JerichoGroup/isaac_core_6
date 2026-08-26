"""
OmniGraph node: split a seconds value into a ROS 2 ``(sec, nanosec)`` timestamp.

Thin adapter over :func:`isaac_core.contracts.stamp.seconds_to_ros_stamp`, which is unit
tested without Isaac Sim. This module contains no arithmetic of its own.

Exists because ``isaacsim.core.nodes.IsaacReadSimulationTime`` emits ``simulationTime`` as
a single ``double`` in seconds, while a ``std_msgs/Header`` needs a signed int32 ``sec``
and an unsigned int32 ``nanosec``. Nothing in the shipped node set bridges those shapes, so
without this the pose topic publishes ``stamp: {sec: 0, nanosec: 0}``.
"""

import carb
from isaac_core_ogn.math.ogn.OgnSecondsToRosStampDatabase import (
    OgnSecondsToRosStampDatabase,
)

from isaac_core.contracts.stamp import seconds_to_ros_stamp

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | STAMP |"


class OgnSecondsToRosStamp:
    """OmniGraph node: seconds as a double to a ROS 2 ``(sec, nanosec)`` pair."""

    @staticmethod
    def compute(db: OgnSecondsToRosStampDatabase) -> bool:
        """
        Convert the input seconds to a ROS 2 timestamp and write both outputs.

        Args:
            db: OmniGraph node database.

        Returns:
            ``True`` always. A bad input is logged and the previous outputs are left
            alone, so one malformed tick cannot stop the simulation -- consistent with
            the other nodes in this extension.

        """
        try:
            sec, nanosec = seconds_to_ros_stamp(float(db.inputs.seconds))
        except (TypeError, ValueError) as exc:
            carb.log_warn(f"{_LOG_PREFIX} Could not convert simulation time: {exc}")
            return True

        db.outputs.sec = sec
        db.outputs.nanosec = nanosec
        return True
