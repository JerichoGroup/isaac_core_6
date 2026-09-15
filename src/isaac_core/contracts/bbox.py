"""Parallel-array assembly for ``isaac_core_ros2_msgs/FrameBboxes``.

The message carries sixteen parallel arrays rather than an array of per-object messages, because
Isaac Sim 6 exposes a nested message array as an unusable ``token[]`` that segfaults on write. Index
``i`` is therefore the same object in every array, and that alignment is the whole contract: a target
appended to fifteen arrays but not the sixteenth silently shifts every later detection.

:class:`BboxArrays` exists to make that impossible to get wrong. One :class:`BboxDetection` goes in,
sixteen values come out, and there is no way to append a partial row.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Final

# Field order matches the .msg definition, and is the order the OmniGraph node writes.
ARRAY_FIELDS: Final[tuple[str, ...]] = (
    "target_name",
    "x1",
    "y1",
    "x2",
    "y2",
    "in_frame",
    "is_visible",
    "lat",
    "lon",
    "alt",
    "roll",
    "pitch",
    "yaw",
    "distance_x",
    "distance_y",
    "distance_z",
)


@dataclass(frozen=True, slots=True)
class BboxDetection:
    """One labelled object as seen this frame.

    Attributes:
        target_name: The prim's name.
        x1: Left pixel of the box.
        y1: Top pixel of the box.
        x2: Right pixel of the box.
        y2: Bottom pixel of the box.
        in_frame: Whether the object's extent falls inside the frame at all.
        is_visible: Whether any of it is actually unoccluded.
        lat: Geodetic latitude in degrees, or NaN when the target carries no anchor.
        lon: Geodetic longitude in degrees, or NaN when the target carries no anchor.
        alt: Altitude in metres, or NaN when the target carries no anchor.
        roll: Local roll in radians.
        pitch: Local pitch in radians.
        yaw: Local yaw in radians.
        distance_x: Offset from the camera along stage X, in metres.
        distance_y: Offset from the camera along stage Y, in metres.
        distance_z: Offset from the camera along stage Z, in metres.

    """

    target_name: str
    x1: int
    y1: int
    x2: int
    y2: int
    in_frame: bool
    is_visible: bool
    lat: float
    lon: float
    alt: float
    roll: float
    pitch: float
    yaw: float
    distance_x: float
    distance_y: float
    distance_z: float


@dataclass(slots=True)
class BboxArrays:
    """Sixteen parallel arrays, grown one whole detection at a time."""

    target_name: list[str] = field(default_factory=list)
    x1: list[int] = field(default_factory=list)
    y1: list[int] = field(default_factory=list)
    x2: list[int] = field(default_factory=list)
    y2: list[int] = field(default_factory=list)
    in_frame: list[bool] = field(default_factory=list)
    is_visible: list[bool] = field(default_factory=list)
    lat: list[float] = field(default_factory=list)
    lon: list[float] = field(default_factory=list)
    alt: list[float] = field(default_factory=list)
    roll: list[float] = field(default_factory=list)
    pitch: list[float] = field(default_factory=list)
    yaw: list[float] = field(default_factory=list)
    distance_x: list[float] = field(default_factory=list)
    distance_y: list[float] = field(default_factory=list)
    distance_z: list[float] = field(default_factory=list)

    def append(self, detection: BboxDetection) -> None:
        """Append one detection to every array at once.

        Args:
            detection: The object to record. Every field is written, so the arrays cannot fall out
                of step.

        """
        for name in ARRAY_FIELDS:
            getattr(self, name).append(getattr(detection, name))

    def __len__(self) -> int:
        """Return the number of detections recorded."""
        return len(self.target_name)

    def aligned(self) -> bool:
        """Report whether every array holds the same number of entries.

        Returns:
            ``True`` when the arrays are index-aligned, which :meth:`append` guarantees.

        """
        return len({len(getattr(self, name)) for name in ARRAY_FIELDS}) == 1

    def as_outputs(self) -> dict[str, list[object]]:
        """Return the arrays keyed by output name, for writing to a node database.

        Returns:
            A mapping of every field in :data:`ARRAY_FIELDS` to its list.

        """
        return {name: getattr(self, name) for name in ARRAY_FIELDS}


__all__ = [
    "ARRAY_FIELDS",
    "BboxArrays",
    "BboxDetection",
]
