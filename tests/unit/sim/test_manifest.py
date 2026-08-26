"""Tests for isaac_core.sim.manifest."""

from __future__ import annotations

from pathlib import Path

from pydantic import ValidationError
import pytest

from isaac_core.sim.manifest import LayerManifest, load_manifest

# -- Helpers ------------------------------------------------------------------


def _write_toml(tmp_path: Path, content: str) -> Path:
    """Write a layer.toml and return its path."""
    toml_file = tmp_path / "layer.toml"
    toml_file.write_text(content, encoding="utf-8")
    return toml_file


# -- Valid manifest -----------------------------------------------------------


def test_valid_manifest_loads(tmp_path: Path) -> None:
    content = """\
id = "distance_sensor"
usd = "distance_sensor.usda"
mount = "/Environment/{instance}/distance_sensor"
requires = ["CAMERA"]
provides = ["range_data"]
provision = false

[[bindings]]
prim = "{mount}/ActionGraph/publisher"
attribute = "inputs:topicName"
config = "topics.distance_sensor"
"""
    manifest = load_manifest(_write_toml(tmp_path, content))
    assert manifest.id == "distance_sensor"
    assert manifest.usd == "distance_sensor.usda"
    assert manifest.mount == "/Environment/{instance}/distance_sensor"
    assert manifest.requires == ("CAMERA",)
    assert manifest.provides == ("range_data",)
    assert manifest.provision is False
    assert len(manifest.bindings) == 1
    assert manifest.bindings[0].config == "topics.distance_sensor"
    assert manifest.bindings[0].resolve is None


def test_valid_manifest_minimal(tmp_path: Path) -> None:
    # Only required fields; defaults apply.
    content = """\
id = "thermal"
usd = "thermal.usda"
mount = "/Environment/thermal"
"""
    manifest = load_manifest(_write_toml(tmp_path, content))
    assert manifest.requires == ()
    assert manifest.provides == ()
    assert manifest.provision is False
    assert manifest.bindings == ()


def test_binding_with_resolve(tmp_path: Path) -> None:
    content = """\
id = "earth"
usd = "earth.usda"
mount = "/Environment/earth"

[[bindings]]
prim = "{mount}/tileset"
attribute = "cesium:url"
resolve = "tilesets_url"
"""
    manifest = load_manifest(_write_toml(tmp_path, content))
    assert manifest.bindings[0].resolve == "tilesets_url"
    assert manifest.bindings[0].config is None


# -- Rejection cases ----------------------------------------------------------


def test_unknown_key_rejected(tmp_path: Path) -> None:
    # extra="forbid" means unrecognised keys are an error
    content = """\
id = "sensor"
usd = "sensor.usda"
mount = "/Environment/sensor"
unknown_field = "boom"
"""
    with pytest.raises(ValueError, match="unknown_field"):
        load_manifest(_write_toml(tmp_path, content))


def test_binding_with_both_config_and_resolve_rejected(tmp_path: Path) -> None:
    content = """\
id = "sensor"
usd = "sensor.usda"
mount = "/Environment/sensor"

[[bindings]]
prim = "{mount}/node"
attribute = "inputs:value"
config = "some.key"
resolve = "some_runtime"
"""
    with pytest.raises(ValueError, match="exactly one"):
        load_manifest(_write_toml(tmp_path, content))


def test_binding_with_neither_config_nor_resolve_rejected(tmp_path: Path) -> None:
    content = """\
id = "sensor"
usd = "sensor.usda"
mount = "/Environment/sensor"

[[bindings]]
prim = "{mount}/node"
attribute = "inputs:value"
"""
    with pytest.raises(ValueError, match="exactly one"):
        load_manifest(_write_toml(tmp_path, content))


def test_unknown_placeholder_in_binding_prim_rejected(tmp_path: Path) -> None:
    # {unknown} is not in KNOWN_PLACEHOLDERS
    content = """\
id = "sensor"
usd = "sensor.usda"
mount = "/Environment/sensor"

[[bindings]]
prim = "{unknown}/node"
attribute = "inputs:value"
config = "some.key"
"""
    with pytest.raises(ValueError, match="unknown placeholder"):
        load_manifest(_write_toml(tmp_path, content))


def test_unknown_placeholder_in_mount_rejected(tmp_path: Path) -> None:
    content = """\
id = "sensor"
usd = "sensor.usda"
mount = "/Environment/{bogus}/sensor"
"""
    with pytest.raises(ValueError, match="unknown placeholder"):
        load_manifest(_write_toml(tmp_path, content))


def test_invalid_id_rejected(tmp_path: Path) -> None:
    # id must be a valid topic segment (letters, digits, underscores, not starting with digit)
    content = """\
id = "123-invalid"
usd = "sensor.usda"
mount = "/Environment/sensor"
"""
    with pytest.raises(ValueError, match="invalid topic segment"):
        load_manifest(_write_toml(tmp_path, content))


def test_id_with_slash_rejected(tmp_path: Path) -> None:
    content = """\
id = "some/path"
usd = "sensor.usda"
mount = "/Environment/sensor"
"""
    with pytest.raises(ValueError, match="invalid topic segment"):
        load_manifest(_write_toml(tmp_path, content))


def test_malformed_toml_raises_with_path(tmp_path: Path) -> None:
    # Invalid TOML syntax
    toml_file = _write_toml(tmp_path, "id = [unclosed")
    with pytest.raises(ValueError, match=str(toml_file)):
        load_manifest(toml_file)


def test_file_not_found_raises(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="layer.toml"):
        load_manifest(tmp_path / "layer.toml")


def test_extra_forbid_on_binding(tmp_path: Path) -> None:
    # Unknown key in a binding entry
    content = """\
id = "sensor"
usd = "sensor.usda"
mount = "/Environment/sensor"

[[bindings]]
prim = "{mount}/node"
attribute = "inputs:value"
config = "x.y"
extra_key = "nope"
"""
    with pytest.raises(ValueError, match="extra_key"):
        load_manifest(_write_toml(tmp_path, content))


def test_layer_manifest_is_frozen() -> None:
    manifest = LayerManifest(
        id="test",
        usd="test.usda",
        mount="/Environment/test",
    )
    with pytest.raises(ValidationError):
        manifest.id = "other"


def test_mount_referencing_itself_is_rejected_at_load(tmp_path: Path) -> None:
    # `mount = "{mount}"` is circular: this field DEFINES what {mount} resolves to.
    # It used to pass validation and then fail at compose time with a bare
    # KeyError: 'mount', which pointed nowhere near the actual mistake.
    manifest = tmp_path / "layer.toml"
    manifest.write_text('id = "x"\nusd = "x.usda"\nmount = "{mount}"\n', encoding="utf-8")
    with pytest.raises(ValueError, match="circular"):
        load_manifest(manifest)


def test_mount_may_use_the_instance_placeholder(tmp_path: Path) -> None:
    manifest = tmp_path / "layer.toml"
    manifest.write_text('id = "x"\nusd = "x.usda"\nmount = "/Environment/{instance}"\n', encoding="utf-8")
    assert load_manifest(manifest).mount == "/Environment/{instance}"
