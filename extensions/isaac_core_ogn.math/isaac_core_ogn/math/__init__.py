"""Isaac Core OGN math extension — coordinate and rotation nodes."""

import omni.ext


class IsaacCoreOgnMathExtension(omni.ext.IExt):
    """Extension lifecycle for isaac_core_ogn.math."""

    def on_startup(self, ext_id: str) -> None:
        """Log extension startup."""
        print("[isaac_core_ogn.math] Extension startup", flush=True)

    def on_shutdown(self) -> None:
        """Log extension shutdown."""
        print("[isaac_core_ogn.math] Extension shutdown", flush=True)
