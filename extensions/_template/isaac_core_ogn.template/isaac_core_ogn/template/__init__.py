"""Isaac Core OGN template extension — scaffold for new nodes."""

import omni.ext


class IsaacCoreOgnTemplateExtension(omni.ext.IExt):
    """Extension lifecycle for isaac_core_ogn.template."""

    def on_startup(self, ext_id: str) -> None:
        """Log extension startup."""
        print("[isaac_core_ogn.template] Extension startup", flush=True)

    def on_shutdown(self) -> None:
        """Log extension shutdown."""
        print("[isaac_core_ogn.template] Extension shutdown", flush=True)
