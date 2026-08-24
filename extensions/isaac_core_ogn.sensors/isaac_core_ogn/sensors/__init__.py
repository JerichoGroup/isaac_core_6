"""
Isaac Core OGN sensors extension — ROS 2 publisher and subscriber nodes.

Provides: Ros2GlobalPosePublisher, Ros2RangePublisher, Ros2ImagePublisher, Ros2Gimbal.
"""

import omni.ext


class IsaacCoreOgnSensorsExtension(omni.ext.IExt):
    """Extension lifecycle for isaac_core_ogn.sensors."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the sensors extension."""
        print("[isaac_core_ogn.sensors] Extension startup", flush=True)

    def on_shutdown(self) -> None:
        """Shut down the sensors extension."""
        print("[isaac_core_ogn.sensors] Extension shutdown", flush=True)
