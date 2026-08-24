"""Isaac Core OGN position extension — pose source nodes (UDP and ROS 2)."""

import omni.ext


class IsaacCoreOgnPositionExtension(omni.ext.IExt):
    """Extension lifecycle for isaac_core_ogn.position."""

    def on_startup(self, ext_id: str) -> None:
        """Log extension startup."""
        print("[isaac_core_ogn.position] Extension startup", flush=True)

    def on_shutdown(self) -> None:
        """Log extension shutdown."""
        print("[isaac_core_ogn.position] Extension shutdown", flush=True)
