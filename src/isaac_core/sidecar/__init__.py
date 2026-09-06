"""
Supervised out-of-process services for capabilities that cannot live in Isaac's interpreter.

Isaac Sim 6 bundles Python 3.12 while ROS 2 Humble's ``rclpy`` is a CPython 3.10 C
extension; GStreamer's PyGObject likewise cannot cohabit with Isaac's interpreter. The
sidecar hosts services needing those on the system Python 3.10, launched as a peer of
the simulator rather than a fork-and-forget child.

Run with ``python3 -m isaac_core.sidecar --config <path>``.
"""

from isaac_core.sidecar.service import (
    RestartPolicy,
    Service,
    ServiceRegistry,
    ServiceSupervisor,
)

__all__ = [
    "RestartPolicy",
    "Service",
    "ServiceRegistry",
    "ServiceSupervisor",
]
