"""Zoom maths: a 0-1 level, a focal length, and the field of view they mean.

A USD camera's field of view is not a settable attribute. It follows from the focal length and the sensor
aperture:

```
hfov = 2 * atan(horizontal_aperture / (2 * focal_length))
```

So zooming means writing `focalLength` while holding the aperture fixed. Changing the aperture instead
would also change the field of view, but it would move the sensor rather than the lens and would
invalidate every intrinsic derived from it.

**Level is linear in field of view, not in focal length.** Halfway between a 60-degree and a 6-degree lens
is 33 degrees, which is what a user means by "zoom halfway"; in focal length the same midpoint sits at
about 11 degrees, almost fully zoomed in. A real lens is linear in neither, so a per-camera calibration
table is the eventual answer, but linear-in-FOV is the honest approximation and the one that behaves the
way the control looks.

Level 0 is the widest view -- the shortest focal length -- and level 1 the narrowest. That matches how a
zoom control reads rather than how a lens is specified.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

# A lens cannot see a full half-turn or more, and the tangent blows up as it approaches one.
_MAX_HFOV_DEG = 179.0

# Level is a fraction, and clamping is the documented behaviour rather than an error: a zoom control that
# refuses the end of its own range would be surprising.
_MIN_LEVEL = 0.0
_MAX_LEVEL = 1.0

# Below this the min and max focal lengths describe the same lens, so a level cannot be recovered from a
# focal length and the midpoint is the only meaningful answer.
_DEGENERATE_SPAN_DEG = 1e-9


def hfov_deg_for_focal(focal_length_mm: float, horizontal_aperture_mm: float) -> float:
    """Return the horizontal field of view a focal length produces.

    Args:
        focal_length_mm: Lens focal length in millimetres.
        horizontal_aperture_mm: Sensor width in millimetres, held fixed while zooming.

    Returns:
        Horizontal field of view in degrees.

    Raises:
        ValueError: If either value is not positive.

    """
    if focal_length_mm <= 0.0:
        raise ValueError(f"focal_length_mm must be positive, got {focal_length_mm}")
    if horizontal_aperture_mm <= 0.0:
        raise ValueError(f"horizontal_aperture_mm must be positive, got {horizontal_aperture_mm}")
    return math.degrees(2.0 * math.atan(horizontal_aperture_mm / (2.0 * focal_length_mm)))


def focal_for_hfov_deg(hfov_deg: float, horizontal_aperture_mm: float) -> float:
    """Return the focal length that produces a horizontal field of view.

    Args:
        hfov_deg: Desired horizontal field of view in degrees.
        horizontal_aperture_mm: Sensor width in millimetres.

    Returns:
        Focal length in millimetres.

    Raises:
        ValueError: If the field of view is not in ``(0, 179]`` or the aperture is not positive.

    """
    if not 0.0 < hfov_deg <= _MAX_HFOV_DEG:
        raise ValueError(f"hfov_deg must be in (0, {_MAX_HFOV_DEG}], got {hfov_deg}")
    if horizontal_aperture_mm <= 0.0:
        raise ValueError(f"horizontal_aperture_mm must be positive, got {horizontal_aperture_mm}")
    return horizontal_aperture_mm / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))


@dataclass(frozen=True)
class ZoomRange:
    """The focal lengths a camera can reach, and the fields of view they correspond to."""

    focal_min_mm: float
    focal_max_mm: float
    horizontal_aperture_mm: float

    def __post_init__(self) -> None:
        """Reject a range that cannot describe a lens.

        Raises:
            ValueError: If a value is not positive or the minimum exceeds the maximum.

        """
        for name, value in (
            ("focal_min_mm", self.focal_min_mm),
            ("focal_max_mm", self.focal_max_mm),
            ("horizontal_aperture_mm", self.horizontal_aperture_mm),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be positive, got {value}")
        if self.focal_min_mm > self.focal_max_mm:
            raise ValueError(
                f"focal_length_min_mm ({self.focal_min_mm}) must not exceed "
                f"focal_length_max_mm ({self.focal_max_mm})"
            )

    @property
    def widest_hfov_deg(self) -> float:
        """Return the field of view at level 0, which is the shortest focal length."""
        return hfov_deg_for_focal(self.focal_min_mm, self.horizontal_aperture_mm)

    @property
    def narrowest_hfov_deg(self) -> float:
        """Return the field of view at level 1, which is the longest focal length."""
        return hfov_deg_for_focal(self.focal_max_mm, self.horizontal_aperture_mm)

    def hfov_deg_at(self, level: float) -> float:
        """Return the field of view at a zoom level, interpolating linearly in field of view.

        Args:
            level: Zoom level, clamped to ``[0, 1]``.

        Returns:
            Horizontal field of view in degrees.

        """
        fraction = clamp_level(level)
        widest, narrowest = self.widest_hfov_deg, self.narrowest_hfov_deg
        return widest + fraction * (narrowest - widest)

    def focal_mm_at(self, level: float) -> float:
        """Return the focal length to write for a zoom level.

        Args:
            level: Zoom level, clamped to ``[0, 1]``.

        Returns:
            Focal length in millimetres.

        """
        return focal_for_hfov_deg(self.hfov_deg_at(level), self.horizontal_aperture_mm)

    def level_for_focal_mm(self, focal_length_mm: float) -> float:
        """Return the zoom level a focal length corresponds to.

        The inverse of :meth:`focal_mm_at`, so ``set_zoom(focal_mm=...)`` can report the level it
        landed on rather than leaving the two numbers disagreeing.

        Args:
            focal_length_mm: Focal length in millimetres.

        Returns:
            Zoom level in ``[0, 1]``.

        """
        widest, narrowest = self.widest_hfov_deg, self.narrowest_hfov_deg
        span = narrowest - widest
        if abs(span) < _DEGENERATE_SPAN_DEG:
            # A fixed lens: every focal length in range is the same one, so no level is more correct.
            return _MIN_LEVEL
        hfov = hfov_deg_for_focal(focal_length_mm, self.horizontal_aperture_mm)
        return clamp_level((hfov - widest) / span)

    def clamp_focal_mm(self, focal_length_mm: float) -> float:
        """Return a focal length brought inside the range.

        Args:
            focal_length_mm: Requested focal length in millimetres.

        Returns:
            The nearest focal length the camera can reach.

        """
        return min(max(focal_length_mm, self.focal_min_mm), self.focal_max_mm)


def clamp_level(level: float) -> float:
    """Return a zoom level brought inside ``[0, 1]``.

    Args:
        level: Requested level.

    Returns:
        The clamped level.

    Raises:
        ValueError: If the level is not a finite number.

    """
    if not math.isfinite(level):
        raise ValueError(f"zoom level must be finite, got {level}")
    return min(max(level, _MIN_LEVEL), _MAX_LEVEL)


def slew_hfov_deg(current_hfov_deg: float, target_hfov_deg: float, max_rate_deg_s: float, dt_s: float) -> float:
    """Move a field of view toward a target, rate-limited.

    Rate limiting is applied in field of view rather than in focal length, for the same reason level is:
    degrees per second is what a viewer perceives, while millimetres per second would crawl at the wide
    end and race at the long end.

    A ``max_rate_deg_s`` of zero or negative means UNLIMITED and snaps straight to the target, matching
    how the gimbal reads its own rate limit so an unset limit follows instantly rather than freezing.

    Args:
        current_hfov_deg: Where the zoom is now.
        target_hfov_deg: Where it is going.
        max_rate_deg_s: Maximum change in degrees per second, or zero for no limit.
        dt_s: Elapsed time in seconds. Zero or negative returns the current value unchanged.

    Returns:
        The new field of view in degrees.

    """
    if max_rate_deg_s <= 0.0:
        return target_hfov_deg
    if dt_s <= 0.0:
        return current_hfov_deg
    step = max_rate_deg_s * dt_s
    delta = target_hfov_deg - current_hfov_deg
    if abs(delta) <= step:
        return target_hfov_deg
    return current_hfov_deg + math.copysign(step, delta)
