"""OmniGraph node: scaffold template that greets the user and counts frames.

Copy this file as a starting point for new nodes. Replace the class names,
internal state, and compute logic with your implementation.
"""

import carb
from isaac_core_ogn.template.ogn.OgnTemplateDatabase import OgnTemplateDatabase

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | TMPL |"


class _InternalState:
    """Persistent per-node state for the template node."""

    def __init__(self) -> None:
        carb.log_info(f"{_LOG_PREFIX} Initializing internal state")
        self.frame_id: int = 0


class OgnTemplate:
    """OmniGraph node: scaffold template."""

    @staticmethod
    def internal_state() -> _InternalState:
        """Return persistent per-node state."""
        return _InternalState()

    @staticmethod
    def compute(db: OgnTemplateDatabase) -> bool:
        """Greet and increment frame counter."""
        carb.log_info(f"{_LOG_PREFIX} compute triggered")

        state: _InternalState = db.per_instance_state
        name = str(db.inputs.name)

        carb.log_info(f"{_LOG_PREFIX} Hello {name}! frame={state.frame_id}")
        db.outputs.frame_id = state.frame_id
        state.frame_id += 1

        return True

    @staticmethod
    def release(node: object) -> None:
        """Release per-node resources."""
        carb.log_info(f"{_LOG_PREFIX} Node release triggered")
        try:
            state = OgnTemplateDatabase.per_instance_internal_state(node)
        except Exception as exc:
            carb.log_error(f"{_LOG_PREFIX} Node release error: {exc}")
            return

        if state is not None:
            carb.log_info(f"{_LOG_PREFIX} Node resources released")
