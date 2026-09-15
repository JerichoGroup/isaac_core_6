"""Contract tests for OmniGraph extensions.

These tests parse .ogn and extension.toml files as data — no Isaac Sim import,
no OmniGraph runtime. They enforce naming conventions, structural contracts, and
prevent the node-type-naming drift that afflicted the old repo.
"""

import json
from pathlib import Path

import pytest

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib

# Root of the extensions directory
_EXTENSIONS_ROOT = Path(__file__).resolve().parents[3] / "extensions"


def _discover_ogn_files() -> list[Path]:
    """Find all .ogn files under extensions/."""
    return sorted(_EXTENSIONS_ROOT.rglob("*.ogn"))


def _discover_extension_tomls() -> list[Path]:
    """Find all config/extension.toml files under extensions/."""
    return sorted(_EXTENSIONS_ROOT.rglob("config/extension.toml"))


# ─── .ogn validity tests ─────────────────────────────────────────────────────

_OGN_FILES = _discover_ogn_files()


@pytest.mark.parametrize("ogn_path", _OGN_FILES, ids=[str(p.relative_to(_EXTENSIONS_ROOT)) for p in _OGN_FILES])
def test_ogn_is_valid_json(ogn_path: Path) -> None:
    # Every .ogn file must be parseable JSON
    text = ogn_path.read_text(encoding="utf-8")
    json.loads(text)


@pytest.mark.parametrize("ogn_path", _OGN_FILES, ids=[str(p.relative_to(_EXTENSIONS_ROOT)) for p in _OGN_FILES])
def test_ogn_has_single_key_matching_filename(ogn_path: Path) -> None:
    # .ogn must have exactly one top-level key == filename minus "Ogn" prefix
    data = json.loads(ogn_path.read_text(encoding="utf-8"))
    keys = list(data.keys())
    assert len(keys) == 1, f"Expected 1 top-level key, got {len(keys)}: {keys}"

    expected_key = ogn_path.stem.removeprefix("Ogn")
    assert keys[0] == expected_key, (
        f"Top-level key {keys[0]!r} does not match expected {expected_key!r} "
        f"(filename {ogn_path.stem} minus 'Ogn' prefix)"
    )


@pytest.mark.parametrize("ogn_path", _OGN_FILES, ids=[str(p.relative_to(_EXTENSIONS_ROOT)) for p in _OGN_FILES])
def test_ogn_key_does_not_contain_ogn_or_sim(ogn_path: Path) -> None:
    # Node type key must never contain "Ogn" or "Sim" (naming rule)
    data = json.loads(ogn_path.read_text(encoding="utf-8"))
    key = list(data.keys())[0]
    assert "Ogn" not in key, f"Key {key!r} contains 'Ogn'"
    assert "Sim" not in key, f"Key {key!r} contains 'Sim'"


@pytest.mark.parametrize("ogn_path", _OGN_FILES, ids=[str(p.relative_to(_EXTENSIONS_ROOT)) for p in _OGN_FILES])
def test_ogn_has_sibling_py(ogn_path: Path) -> None:
    # Every .ogn must have a sibling .py with the same stem
    py_sibling = ogn_path.with_suffix(".py")
    assert py_sibling.exists(), f"Missing sibling .py for {ogn_path.name}"


@pytest.mark.parametrize("ogn_path", _OGN_FILES, ids=[str(p.relative_to(_EXTENSIONS_ROOT)) for p in _OGN_FILES])
def test_ogn_inputs_outputs_have_type_and_description(ogn_path: Path) -> None:
    # Every declared input and output must have "type" and "description"
    data = json.loads(ogn_path.read_text(encoding="utf-8"))
    node_key = list(data.keys())[0]
    node_def = data[node_key]

    for section_name in ("inputs", "outputs"):
        section = node_def.get(section_name, {})
        for port_name, port_def in section.items():
            assert "type" in port_def, f"{ogn_path.name}:{node_key}.{section_name}.{port_name} missing 'type'"
            assert (
                "description" in port_def
            ), f"{ogn_path.name}:{node_key}.{section_name}.{port_name} missing 'description'"


def test_ogn_node_type_keys_are_unique() -> None:
    # Node type keys must be unique across all extensions
    all_keys: dict[str, Path] = {}
    for ogn_path in _OGN_FILES:
        data = json.loads(ogn_path.read_text(encoding="utf-8"))
        key = list(data.keys())[0]
        if key in all_keys:
            pytest.fail(f"Duplicate node type key {key!r} in {ogn_path} and {all_keys[key]}")
        all_keys[key] = ogn_path


# ─── extension.toml validity tests ───────────────────────────────────────────

_TOML_FILES = _discover_extension_tomls()


@pytest.mark.parametrize("toml_path", _TOML_FILES, ids=[str(p.relative_to(_EXTENSIONS_ROOT)) for p in _TOML_FILES])
def test_extension_toml_is_valid(toml_path: Path) -> None:
    # Must be valid TOML
    with toml_path.open("rb") as f:
        tomllib.load(f)


@pytest.mark.parametrize("toml_path", _TOML_FILES, ids=[str(p.relative_to(_EXTENSIONS_ROOT)) for p in _TOML_FILES])
def test_extension_toml_has_python_module_matching_namespace(toml_path: Path) -> None:
    # [[python.module]] name must match the directory's actual package namespace
    with toml_path.open("rb") as f:
        config = tomllib.load(f)

    modules = config.get("python", {}).get("module", [])
    assert modules, f"{toml_path}: no [[python.module]] entry found"

    # The extension root is the parent of `config/`
    ext_root = toml_path.parent.parent
    module_name = modules[0]["name"]

    # The module name uses dots; the directory uses slashes
    module_path = ext_root / module_name.replace(".", "/")
    assert module_path.is_dir(), f"{toml_path}: declared module {module_name!r} does not exist at {module_path}"


@pytest.mark.parametrize("toml_path", _TOML_FILES, ids=[str(p.relative_to(_EXTENSIONS_ROOT)) for p in _TOML_FILES])
def test_extension_toml_ogn_path_exists(toml_path: Path) -> None:
    # [ogn] path must point to a real directory
    with toml_path.open("rb") as f:
        config = tomllib.load(f)

    ogn_section = config.get("ogn")
    if ogn_section is None:
        pytest.skip("No [ogn] section — extension may not provide OGN nodes")

    ogn_path_str = ogn_section.get("path")
    assert ogn_path_str, f"{toml_path}: [ogn] section missing 'path'"

    ext_root = toml_path.parent.parent
    ogn_dir = ext_root / ogn_path_str
    assert ogn_dir.is_dir(), f"{toml_path}: declared ogn path {ogn_path_str!r} does not exist at {ogn_dir}"
