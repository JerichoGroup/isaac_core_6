"""
Shared contracts: the single source of truth for every cross-boundary name.

Everything in this package is pure Python with no third-party imports at all, so
it can be imported from Isaac Sim's interpreter, a ROS 2 environment, system
python3 or a bare CI venv alike. That matters because these definitions have to be
identical in every one of those processes.

The previous generation had no such layer, and paid for it: topic names, the UDP
header bytes, the packet struct format, default ports and the readiness sentinel
were each re-declared in two to four places that could drift apart independently.

Contents:

:mod:`~isaac_core.contracts.frames`
    Frame and rotation conventions, including the configurable world/body
    rotation frame, and the pose source enum.
:mod:`~isaac_core.contracts.packet`
    UDP pose packet wire specification.
:mod:`~isaac_core.contracts.ports`
    Default ports and per-vehicle port allocation.
:mod:`~isaac_core.contracts.pose`
    Frame-tagged pose value types.
:mod:`~isaac_core.contracts.prims`
    USD prim path templating and validation.
:mod:`~isaac_core.contracts.topics`
    ROS 2 topic naming and namespace collapsing.
"""

from isaac_core.contracts.frames import (
    EULER_AXES,
    Frame,
    PoseSource,
    RotationFrame,
)
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy

__all__ = [
    "EULER_AXES",
    "Frame",
    "GeodeticPose",
    "Lla",
    "PoseSource",
    "RotationFrame",
    "Rpy",
]
