"""OmniGraph node: convert global LLA + orientation to local ENU position and quaternion.

Thin adapter over isaac_core.geo — all geodesy and rotation maths live there.
"""

import carb
from isaac_core_ogn.math.ogn.OgnGlobalPositionToLocalPositionDatabase import (
    OgnGlobalPositionToLocalPositionDatabase,
)

from isaac_core.contracts.pose import Lla
from isaac_core.geo import (
    EnuConverter,
    compose_local_pose,
    has_position_fix,
    rotation_frame_from_text,
)

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | GPTLP |"


class _InternalState:
    """Persistent per-node state for GlobalPositionToLocalPosition."""

    def __init__(self) -> None:
        carb.log_info(f"{_LOG_PREFIX} Initializing internal state")
        self.geo_reference: tuple[float, float, float] | None = None
        self.converter: EnuConverter | None = None
        self.warned_no_data: bool = False

    def ensure_converter(self, reference: tuple[float, float, float]) -> EnuConverter:
        """Lazily create or recreate the ENU converter if the reference point changed.

        Args:
            reference: ``(lat_deg, lon_deg, alt_m)`` from the node input.

        Returns:
            The current converter instance.

        """
        if self.converter is None or self.geo_reference != reference:
            carb.log_info(f"{_LOG_PREFIX} Creating ENU converter for reference {reference}")
            self.geo_reference = reference
            self.converter = EnuConverter(*reference)
        return self.converter


class OgnGlobalPositionToLocalPosition:
    """OmniGraph node: LLA + ENU orientation to local ENU position and quaternion."""

    @staticmethod
    def internal_state() -> _InternalState:
        """Return persistent per-node state."""
        return _InternalState()

    @staticmethod
    def compute(db: OgnGlobalPositionToLocalPositionDatabase) -> bool:
        """Compute local ENU position and composed quaternion.

        A thin adapter: the maths lives in :func:`isaac_core.geo.compose_local_pose`, where it is
        tested without Isaac Sim.
        """
        state: _InternalState = db.per_instance_state
        global_position = tuple(db.inputs.global_position)
        reference = tuple(db.inputs.enu_reference)

        if not has_position_fix(global_position):
            if not state.warned_no_data:
                carb.log_warn(f"{_LOG_PREFIX} Skipping compute — no valid global position yet")
                state.warned_no_data = True
            return True
        state.warned_no_data = False

        pose = compose_local_pose(
            converter=state.ensure_converter(reference),
            position=Lla(
                lat_deg=global_position[0],
                lon_deg=global_position[1],
                alt_m=global_position[2],
            ),
            attitude_r=tuple(db.inputs.global_orientation),
            rotation_frame=rotation_frame_from_text(str(db.inputs.rotation_frame)),
            gimbal_offset_deg=(
                float(db.inputs.offset_roll_deg),
                float(db.inputs.offset_pitch_deg),
                float(db.inputs.offset_yaw_deg),
            ),
        )

        db.outputs.global_position = list(global_position)
        db.outputs.global_orientation = list(pose.attitude_r)
        db.outputs.local_position = list(pose.local_position)
        db.outputs.local_orientation = list(pose.quaternion_xyzw)

        return True

    @staticmethod
    def release(node: object) -> None:
        """Release per-node resources."""
        carb.log_info(f"{_LOG_PREFIX} Node release triggered")
        try:
            state = OgnGlobalPositionToLocalPositionDatabase.per_instance_internal_state(node)
        except Exception as exc:
            carb.log_error(f"{_LOG_PREFIX} Node release error: {exc}")
            return

        if state is not None:
            carb.log_info(f"{_LOG_PREFIX} Node resources released")
