"""Adversarial config sweep: every constrained key, pushed out of bounds.

The hand-written schema tests cover the keys somebody thought to test. This walks the schema instead, so a
key added without a test is still checked, and it asserts the thing that actually matters when a config is
wrong at 2am: **the error names the key**. A rejection that does not say which setting was bad leaves a user
bisecting their own TOML.

Driven through `load(cli_overrides=...)`, which is the path a `--set` flag takes, rather than by constructing
models directly -- so this tests rejection where a user meets it.
"""

from __future__ import annotations

import math
from typing import Any, Final

import pytest

from isaac_core.config import IsaacCoreConfig, load

# Keys whose rejection message legitimately names something other than the leaf itself, with why.
_NAMES_SOMETHING_ELSE: Final[dict[str, str]] = {
    # Port collisions name both vehicles and the key to change, which is more useful than the leaf.
    "vehicles.drone_0.udp_port": "collision messages name both vehicles",
    "vehicles.drone_0.camera.rtsp_port": "collision messages name both vehicles",
}


def _leaf_fields() -> dict[str, Any]:
    """Return dotted key to pydantic FieldInfo for every leaf in the schema.

    Returns:
        A mapping of dotted config key to its field definition.

    """
    found: dict[str, Any] = {}

    def walk(model: Any, prefix: str) -> None:
        for name, field in type(model).model_fields.items():
            value = getattr(model, name)
            dotted = f"{prefix}.{name}" if prefix else name
            if hasattr(type(value), "model_fields"):
                walk(value, dotted)
            elif isinstance(value, dict) and value and hasattr(type(next(iter(value.values()))), "model_fields"):
                for child_key, child in value.items():
                    walk(child, f"{dotted}.{child_key}")
            else:
                found[dotted] = field

    walk(IsaacCoreConfig(), "")
    return found


def _numeric_bounds(field: Any) -> tuple[float | None, float | None]:
    """Return a field's (lower, upper) numeric bounds, if it declares any.

    Args:
        field: A pydantic FieldInfo.

    Returns:
        The lower and upper bound, either of which may be ``None``.

    """
    lower: float | None = None
    upper: float | None = None
    for meta in getattr(field, "metadata", ()):  # annotated_types constraints
        for attribute, assign_lower in (("gt", True), ("ge", True), ("lt", False), ("le", False)):
            bound = getattr(meta, attribute, None)
            if bound is None:
                continue
            if assign_lower:
                lower = float(bound)
            else:
                upper = float(bound)
    return lower, upper


def _constrained_numeric_keys() -> list[tuple[str, float]]:
    """Return every numeric leaf with a bound, paired with a value that violates it.

    Returns:
        Pairs of dotted key and an out-of-bounds value.

    """
    cases: list[tuple[str, float]] = []
    for dotted, field in _leaf_fields().items():
        lower, upper = _numeric_bounds(field)
        if lower is not None:
            # Comfortably below any gt/ge bound, including a bound of zero.
            cases.append((dotted, lower - 1000.0 if lower != 0.0 else -1.0))
        elif upper is not None:
            cases.append((dotted, upper + 1000.0))
    return sorted(cases)


def test_the_sweep_found_constrained_keys() -> None:
    # Without this the parametrised tests below could silently cover nothing.
    cases = _constrained_numeric_keys()
    assert len(cases) > 10, f"only found {len(cases)} constrained keys; the walker is broken"


@pytest.mark.parametrize(("dotted", "bad"), _constrained_numeric_keys(), ids=lambda v: str(v))
def test_an_out_of_bounds_value_is_rejected(dotted: str, bad: float) -> None:
    with pytest.raises((ValueError, TypeError)):
        load(cli_overrides={dotted: str(bad)})


@pytest.mark.parametrize(("dotted", "bad"), _constrained_numeric_keys(), ids=lambda v: str(v))
def test_the_rejection_names_the_key(dotted: str, bad: float) -> None:
    # The property that matters at 2am. A message that does not name the setting leaves a user bisecting
    # their own TOML to find which line broke the load.
    leaf = dotted.rsplit(".", maxsplit=1)[-1]
    with pytest.raises((ValueError, TypeError)) as excinfo:
        load(cli_overrides={dotted: str(bad)})
    message = str(excinfo.value)
    if dotted in _NAMES_SOMETHING_ELSE:
        pytest.skip(f"{dotted}: {_NAMES_SOMETHING_ELSE[dotted]}")
    assert leaf in message, f"rejecting {dotted} produced a message that never mentions {leaf!r}: {message[:300]}"


# -- values that are the wrong shape entirely ----------------------------------- #


@pytest.mark.parametrize(
    "dotted",
    [
        "sim.headless",
        "sim.physics_dt",
        "vehicles.drone_0.camera.fov_deg",
        "vehicles.drone_0.camera.resolution",
        "geo.enu_reference",
        "features.enabled",
    ],
)
def test_a_wrongly_typed_value_is_rejected(dotted: str) -> None:
    # `--set` hands everything over as a string, so the loader has to coerce; a value it cannot coerce
    # must fail rather than silently becoming a default.
    with pytest.raises((ValueError, TypeError)):
        load(cli_overrides={dotted: "definitely not a valid value here"})


@pytest.mark.parametrize("bad", ["nan", "inf", "-inf"])
def test_a_nonfinite_number_is_rejected(bad: str) -> None:
    # NaN compares false against every bound, so a range check alone does not stop it.
    with pytest.raises((ValueError, TypeError)):
        load(cli_overrides={"vehicles.drone_0.camera.fov_deg": bad})


def test_an_unknown_key_is_rejected_rather_than_ignored() -> None:
    # Silently accepting a typo is how a user spends an hour wondering why their setting did nothing.
    with pytest.raises((ValueError, TypeError)) as excinfo:
        load(cli_overrides={"sim.headles": "true"})
    assert "headles" in str(excinfo.value)


def test_an_unknown_nested_section_is_rejected() -> None:
    with pytest.raises((ValueError, TypeError)):
        load(cli_overrides={"no_such_section.no_such_key": "1"})


def test_an_empty_value_is_rejected_where_a_number_is_required() -> None:
    with pytest.raises((ValueError, TypeError)):
        load(cli_overrides={"vehicles.drone_0.camera.fov_deg": ""})


# -- identifiers and paths ------------------------------------------------------ #


@pytest.mark.parametrize("name", ["has space", "has/slash", "has.dot", "", "café"])
def test_an_invalid_vehicle_name_is_rejected(name: str) -> None:
    # Vehicle ids become ROS topic segments and USD prim names, so they cannot be arbitrary text.
    with pytest.raises((ValueError, TypeError)):
        IsaacCoreConfig(vehicles={name: {"camera": {}}})


def test_a_vehicle_name_that_is_a_valid_segment_is_accepted() -> None:
    # The negative tests above are only meaningful if the positive case still works.
    IsaacCoreConfig(vehicles={"drone_0": {"camera": {}}})


@pytest.mark.parametrize("feature", ["has space", "has/slash", ""])
def test_an_invalid_feature_name_is_rejected(feature: str) -> None:
    with pytest.raises((ValueError, TypeError)):
        IsaacCoreConfig(features={"enabled": [feature]})


# -- the boundary itself is accepted -------------------------------------------- #


def test_a_value_exactly_on_an_inclusive_bound_is_accepted() -> None:
    # An off-by-one in a bound would make a documented value unusable, which no out-of-range test catches.
    config = load(cli_overrides={"vehicles.drone_0.camera.f_stop": "0.0"})
    assert config.vehicles["drone_0"].camera.f_stop == pytest.approx(0.0)


def test_the_largest_documented_field_of_view_is_accepted() -> None:
    config = load(cli_overrides={"vehicles.drone_0.camera.fov_deg": "179.0"})
    assert config.vehicles["drone_0"].camera.fov_deg == pytest.approx(179.0)
    assert math.isfinite(config.vehicles["drone_0"].camera.fov_deg)
