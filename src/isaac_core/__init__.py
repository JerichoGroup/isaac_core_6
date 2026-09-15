"""Team infrastructure for building, running and scripting Isaac Sim 6 simulations.

This top-level module is deliberately lightweight. It must stay importable in an
interpreter that has neither Isaac Sim nor ROS 2 available, so the heavy
subpackages are never imported here:

``isaac_core.sim``
    Requires Isaac Sim's bundled interpreter (``omni``, ``carb``, ``pxr``).
``isaac_core.devkit``
    Requires a sourced ROS 2 environment (``rclpy``).
``isaac_core.sidecar``
    Requires system PyGObject/GStreamer (``gi``).

Import those explicitly, from a process where their environment is available.
Everything else -- :mod:`isaac_core.contracts`, :mod:`isaac_core.config`,
:mod:`isaac_core.geo`, :mod:`isaac_core.protocol` and :mod:`isaac_core.vehicle` --
is pure Python and importable anywhere.
"""

__version__ = "0.1.0"

__all__ = ["__version__"]
