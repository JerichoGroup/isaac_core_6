"""Guards for `config/full.toml`, the exhaustive settings reference.

Two properties make that file worth shipping, and both rot silently. It has to list **every** key,
otherwise a reader concludes a setting does not exist. And its values have to match the built-in
defaults, otherwise copying it changes behaviour in ways the reader did not ask for -- the exact trap
that made a fresh-machine run report the HDRI as broken.
"""

from __future__ import annotations

from pathlib import Path
import re
from typing import Any, Final

from isaac_core.config import IsaacCoreConfig, load

FULL_CONFIG: Final = Path(__file__).resolve().parents[3] / "config" / "full.toml"


def _leaf_keys(model: Any, prefix: str = "") -> list[str]:
    """Return every dotted leaf key declared by the schema."""
    found: list[str] = []
    for name in type(model).model_fields:
        value = getattr(model, name)
        dotted = f"{prefix}.{name}" if prefix else name
        if hasattr(type(value), "model_fields"):
            found += _leaf_keys(value, dotted)
        elif isinstance(value, dict) and value and hasattr(type(next(iter(value.values()))), "model_fields"):
            for child_key, child in value.items():
                found += _leaf_keys(child, f"{dotted}.{child_key}")
        else:
            found.append(dotted)
    return found


def _flatten(data: dict[str, Any], prefix: str = "") -> dict[str, Any]:
    """Flatten a nested config dump into dotted keys."""
    flat: dict[str, Any] = {}
    for key, value in data.items():
        dotted = f"{prefix}{key}"
        if isinstance(value, dict):
            flat.update(_flatten(value, f"{dotted}."))
        else:
            flat[dotted] = value
    return flat


def test_the_file_exists_and_loads() -> None:
    assert FULL_CONFIG.is_file(), f"{FULL_CONFIG} is missing"
    load(path=FULL_CONFIG)


def test_every_schema_key_appears() -> None:
    # A key absent from the reference reads as a key that does not exist.
    text = FULL_CONFIG.read_text(encoding="utf-8")
    missing = [key for key in _leaf_keys(IsaacCoreConfig()) if key.rsplit(".", 1)[-1] not in text]
    assert not missing, f"config/full.toml does not mention: {missing}"


def test_copying_it_changes_nothing() -> None:
    # The point of the file is to be a safe starting point, so every uncommented value must equal the
    # built-in default. Anything else means editing one line silently moves several.
    schema = _flatten(IsaacCoreConfig().model_dump(mode="json"))
    from_file = _flatten(load(path=FULL_CONFIG).model_dump(mode="json"))
    divergences = {
        key: (schema.get(key), from_file.get(key))
        for key in sorted(set(schema) | set(from_file))
        if schema.get(key) != from_file.get(key)
    }
    assert not divergences, "config/full.toml does not match the schema defaults for: " + "; ".join(
        f"{k}: default={d!r} file={f!r}" for k, (d, f) in divergences.items()
    )


def test_it_says_it_is_not_loaded_automatically() -> None:
    # Editing a config file that is never read is the single most confusing thing this project does.
    text = FULL_CONFIG.read_text(encoding="utf-8")
    assert "--config config/full.toml" in text
    assert "automatically" in text


def test_it_stays_a_reference_rather_than_an_essay() -> None:
    # It exists because default.toml is comment-heavy: 380 lines for 48 settings. If this one grows
    # the same way it stops being the quick one and there is no reason to ship both.
    lines = FULL_CONFIG.read_text(encoding="utf-8").splitlines()
    settings = sum(1 for line in lines if re.match(r"^#? *[a-z_]+ *=", line))
    assert settings >= 60, f"only {settings} settings are shown; the reference is incomplete"
    assert len(lines) < 220, f"{len(lines)} lines is essay territory; keep the comments to one line each"


def test_no_comment_block_runs_longer_than_two_lines() -> None:
    # One-line comments were the explicit request. A run of three or more means an explanation is
    # growing, and explanations belong in default.toml or the docs.
    lines = FULL_CONFIG.read_text(encoding="utf-8").splitlines()
    # The header explains how to use the file and is allowed to be a paragraph; the rule is about
    # prose growing between the settings.
    first_table = next(i for i, line in enumerate(lines) if line.strip().startswith("["))
    run = 0
    offenders: list[int] = []
    for number, line in enumerate(lines[first_table:], start=first_table + 1):
        stripped = line.strip()
        # A commented-out setting is a setting, not prose.
        if stripped.startswith("#") and not re.match(r"^# *[a-z_]+ *=", stripped) and not stripped.startswith("# ["):
            run += 1
            if run == 3:
                offenders.append(number)
        else:
            run = 0
    assert not offenders, f"comment blocks longer than two lines end at these line numbers: {offenders}"


def test_it_shows_exactly_one_unnamed_camera_table_per_vehicle() -> None:
    # A vehicle has one camera and it has no name. Showing [vehicles.x.cameras.eo] would document a
    # shape the loader now rejects, and showing two would document a feature that does not exist.
    text = FULL_CONFIG.read_text(encoding="utf-8")
    assert "cameras" not in text, "the old named-camera table is still shown"
    tables = re.findall(r"^\s*#?\s*\[vehicles\.([a-z_0-9]+)\.camera\]", text, re.M)
    assert tables, "no camera table found, so this guard is not checking anything"
    assert len(tables) == len(set(tables)), f"a vehicle shows more than one camera table: {tables}"
