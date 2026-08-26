"""
OmniGraph node: convert global LLA + orientation to local ENU position and quaternion.

Thin adapter over isaac_core.geo — all geodesy and rotation maths live there.
"""

import math

import carb
from isaac_core_ogn.math.ogn.OgnGlobalPositionToLocalPositionDatabase import (
    OgnGlobalPositionToLocalPositionDatabase,
)
import numpy as np

from isaac_core.contracts.frames import RotationFrame
from isaac_core.contracts.pose import Lla
from isaac_core.geo import EnuConverter, compose_rotation, euler_to_matrix, euler_to_quaternion, matrix_to_euler

# Prefix for all log messages from this node.
_LOG_PREFIX = "SIM | GPTLP |"


def _quaternion_wxyz_to_isaac_xyzw(w: float, x: float, y: float, z: float) -> list[float]:
    """
    Reorder a standard (w, x, y, z) quaternion to Isaac Sim's OmniGraph [x, y, z, w].

    Isaac Sim 6 / Kit 110 OmniGraph ``quatd[4]`` attributes use IJKR order — confirmed
    by reading the installed OGN schemas (e.g. ``OgnUCXPublishOdometry.ogn`` declares
    ``"Orientation as a quaternion (IJKR - x,y,z,w)"`` with default ``[0,0,0,1]``).
    The GUI displays (w,x,y,z) but the attribute storage is (x,y,z,w).

    This matches the previous-generation behaviour. If a future Kit version changes the
    convention, update this single helper.

    Args:
        w: Scalar component.
        x: X vector component.
        y: Y vector component.
        z: Z vector component.

    Returns:
        ``[x, y, z, w]`` for writing to a ``quatd[4]`` OmniGraph attribute.

    """
    return [x, y, z, w]


class _InternalState:
    """Persistent per-node state for GlobalPositionToLocalPosition."""

    def __init__(self) -> None:
        carb.log_info(f"{_LOG_PREFIX} Initializing internal state")
        self.geo_reference: tuple[float, float, float] | None = None
        self.converter: EnuConverter | None = None
        self.warned_no_data: bool = False

    def ensure_converter(self, reference: tuple[float, float, float]) -> EnuConverter:
        """
        Lazily create or recreate the ENU converter if the reference point changed.

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


def _resolve_rotation_frame(value: str) -> RotationFrame:
    """
    Map the string input to the RotationFrame enum, defaulting to BODY.

    Args:
        value: String from the OGN input, expected ``"body"`` or ``"world"``.

    Returns:
        The corresponding :class:`RotationFrame` variant.

    """
    lowered = value.strip().lower()
    if lowered == "world":
        return RotationFrame.WORLD
    return RotationFrame.BODY


class OgnGlobalPositionToLocalPosition:
    """OmniGraph node: LLA + ENU orientation to local ENU position and quaternion."""

    @staticmethod
    def internal_state() -> _InternalState:
        """Return persistent per-node state."""
        return _InternalState()

    @staticmethod
    def compute(db: OgnGlobalPositionToLocalPositionDatabase) -> bool:
        """Compute local ENU position and composed quaternion."""
        state: _InternalState = db.per_instance_state
        global_position = tuple(db.inputs.global_position)
        global_orientation = tuple(db.inputs.global_orientation)
        reference = tuple(db.inputs.enu_reference)

        # Skip if position is all zeros (no data received yet)
        if np.allclose(global_position, [0.0, 0.0, 0.0]):
            if not state.warned_no_data:
                carb.log_warn(f"{_LOG_PREFIX} Skipping compute — no valid global position yet")
                state.warned_no_data = True
            return True
        state.warned_no_data = False

        # ENU converter (lazily recreated if reference changes)
        converter = state.ensure_converter(reference)

        # LLA -> ENU
        lla = Lla(lat_deg=global_position[0], lon_deg=global_position[1], alt_m=global_position[2])
        east, north, up = converter.lla_to_enu(lla)

        # Compose gimbal offset using the selected rotation frame (D14)
        roll_r, pitch_r, yaw_r = global_orientation[0], global_orientation[1], global_orientation[2]
        drone_matrix = euler_to_matrix(roll_r, pitch_r, yaw_r)

        offset_roll_r = math.radians(float(db.inputs.offset_roll_deg))
        offset_pitch_r = math.radians(float(db.inputs.offset_pitch_deg))
        offset_yaw_r = math.radians(float(db.inputs.offset_yaw_deg))
        offset_matrix = euler_to_matrix(offset_roll_r, offset_pitch_r, offset_yaw_r)

        rotation_frame = _resolve_rotation_frame(str(db.inputs.rotation_frame))
        composed = compose_rotation(drone_matrix, offset_matrix, rotation_frame)

        # Extract composed Euler and quaternion
        composed_roll, composed_pitch, composed_yaw = matrix_to_euler(composed)
        qw, qx, qy, qz = euler_to_quaternion(composed_roll, composed_pitch, composed_yaw)

        # Write outputs
        db.outputs.global_position = list(global_position)
        db.outputs.global_orientation = [composed_roll, composed_pitch, composed_yaw]
        db.outputs.local_position = [east, north, up]
        db.outputs.local_orientation = _quaternion_wxyz_to_isaac_xyzw(qw, qx, qy, qz)

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
