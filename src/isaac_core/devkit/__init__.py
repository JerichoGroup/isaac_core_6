"""User-facing API for scripting Isaac Sim sessions.

This package is what an operator imports in their mission scripts. It provides:

:class:`~isaac_core.devkit.session.Sim`
    Facade for launching or attaching to a simulation.
:class:`~isaac_core.devkit.session.SimSession`
    Session handle exposing vehicles, features, config, and lifecycle control.
:class:`~isaac_core.devkit.transport.UdpPoseTransport`
    Ships encoded pose packets over UDP at a controlled rate.
:class:`~isaac_core.devkit.transport.FakePoseTransport`
    Records poses for testing without network I/O.
:mod:`~isaac_core.devkit.recording`
    Generic topic recorder replacing the four copy-paste capture classes.

This package requires only stdlib + pydantic + isaac_core kernel packages.
The ``recording`` module additionally needs ``rclpy``/``cv_bridge``/``cv2`` at
*usage time* (not at import time), so ``import isaac_core.devkit.recording``
always succeeds.
"""

from isaac_core.devkit.bot import PoseBot
from isaac_core.devkit.mavlink import MavlinkPoseBridge
from isaac_core.devkit.session import Sim, SimSession
from isaac_core.devkit.transport import (
    FakePoseTransport,
    PoseTransport,
    Ros2PoseTransport,
    UdpPoseTransport,
    pace,
)

__all__ = [
    "FakePoseTransport",
    "MavlinkPoseBridge",
    "PoseBot",
    "PoseTransport",
    "Ros2PoseTransport",
    "Sim",
    "SimSession",
    "UdpPoseTransport",
    "pace",
]
