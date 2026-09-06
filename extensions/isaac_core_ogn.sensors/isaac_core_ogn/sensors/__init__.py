"""Isaac Core OGN sensors extension — physics-based sensing nodes."""

import omni.ext


class IsaacCoreOgnSensorsExtension(omni.ext.IExt):
    """Extension lifecycle for isaac_core_ogn.sensors."""

    def on_startup(self, ext_id: str) -> None:
        """Log extension startup."""
        print("[isaac_core_ogn.sensors] Extension startup", flush=True)

    def on_shutdown(self) -> None:
        """Log extension shutdown."""
        print("[isaac_core_ogn.sensors] Extension shutdown", flush=True)
