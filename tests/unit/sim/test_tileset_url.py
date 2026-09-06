"""
Tests for repointing Cesium tilesets at a different server from config.

The tile server URL is otherwise baked into the scene USD, so changing it means hand-editing
a `.usda` in the GUI. The previous generation had this feature; this restores and generalises
it: the override supplies only scheme/host/port, and every tileset keeps its own path.

Driven with fakes so no Isaac Sim or USD is required.
"""

from __future__ import annotations

import contextlib
import logging
import sys
import types
from typing import Any

import pytest

from isaac_core.sim.composer import apply_tileset_server_url


class _FakeAttribute:
    """A USD attribute that may or may not exist."""

    def __init__(self, value: str | None, valid: bool = True) -> None:
        """Store the current value and whether the attribute exists."""
        self._value = value
        self._valid = valid

    def IsValid(self) -> bool:  # noqa: N802
        """Report whether the attribute exists on the prim."""
        return self._valid

    def Get(self) -> str | None:  # noqa: N802
        """Return the current value."""
        return self._value

    def Set(self, value: str) -> None:  # noqa: N802
        """Record a new value."""
        self._value = value


class _FakePrim:
    """A prim exposing a single named attribute."""

    def __init__(self, path: str, url: str | None, valid: bool = True, has_url_attr: bool | None = None) -> None:
        """Build a prim carrying a ``cesium:url`` attribute unless told otherwise."""
        self._path = path
        self._valid = valid
        attr_valid = has_url_attr if has_url_attr is not None else url is not None
        self.attribute = _FakeAttribute(url, valid=attr_valid)

    def IsValid(self) -> bool:  # noqa: N802
        """Report whether the prim exists."""
        return self._valid

    def GetAttribute(self, name: str) -> _FakeAttribute:  # noqa: N802
        """Return the attribute if the name matches, otherwise an invalid one."""
        if name == "cesium:url":
            return self.attribute
        return _FakeAttribute(None, valid=False)

    def GetPath(self) -> str:  # noqa: N802
        """Return the prim path."""
        return self._path


class _FakeStage:
    """A stage that resolves one root path to a fixed set of prims."""

    def __init__(self, root_path: str, prims: list[_FakePrim], root_valid: bool = True) -> None:
        """Store the subtree this stage will return for ``root_path``."""
        self._root_path = root_path
        self._prims = prims
        self._root = _FakePrim(root_path, None, valid=root_valid)

    def GetPrimAtPath(self, path: str) -> _FakePrim:  # noqa: N802
        """Return the root prim for the known path, otherwise an invalid prim."""
        if path == self._root_path:
            return self._root
        return _FakePrim(path, None, valid=False)

    def prims(self) -> list[_FakePrim]:
        """Return the prims in this subtree."""
        return self._prims


@pytest.fixture(autouse=True)
def fake_pxr_usd(monkeypatch: pytest.MonkeyPatch) -> None:
    """Install a fake ``pxr.Usd`` whose PrimRange walks our fake stage."""
    module = types.ModuleType("pxr.Usd")

    def prim_range(root: _FakePrim) -> list[_FakePrim]:
        return _CURRENT_STAGE[0].prims() if _CURRENT_STAGE else []

    module.PrimRange = prim_range  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "pxr.Usd", module)


# Set by each test so the fake PrimRange knows which stage to walk.
_CURRENT_STAGE: list[Any] = []


def _stage(prims: list[_FakePrim], root: str = "/World/tilesets", root_valid: bool = True) -> _FakeStage:
    """Build a fake stage and register it for the fake PrimRange."""
    stage = _FakeStage(root, prims, root_valid=root_valid)
    _CURRENT_STAGE.clear()
    _CURRENT_STAGE.append(stage)
    return stage


@contextlib.contextmanager
def _captured_warnings() -> Any:  # noqa: ANN401
    """
    Capture warnings from the composer logger.

    ``caplog`` does not see this project's warnings, so attach a handler directly to the
    named logger and yield the raw records for the test to inspect.

    Returns:
        A list that fills with the composer's log records for the duration of the block.

    """
    records: list[logging.LogRecord] = []
    handler = logging.Handler()
    handler.emit = records.append  # type: ignore[assignment,method-assign]
    logger = logging.getLogger("isaac_core.sim.composer")
    logger.addHandler(handler)
    logger.setLevel(logging.WARNING)
    try:
        yield records
    finally:
        logger.removeHandler(handler)


def test_two_tilesets_each_keep_their_own_path() -> None:
    # The core generalisation: N tilesets share a server but differ by path.
    prims = [
        _FakePrim("/World/tilesets/cityA", "http://oldhost:8000/tiles/cityA/tileset.json"),
        _FakePrim("/World/tilesets/cityB", "http://oldhost:8000/tiles/cityB/tileset.json"),
    ]
    stage = _stage(prims)
    changed = apply_tileset_server_url(stage, "http://newhost:9000", "/World/tilesets")
    assert changed == 2
    assert prims[0].attribute.Get() == "http://newhost:9000/tiles/cityA/tileset.json"
    assert prims[1].attribute.Get() == "http://newhost:9000/tiles/cityB/tileset.json"


def test_only_the_server_is_swapped_not_the_whole_url() -> None:
    # Guards the past bug: the whole URL used to be replaced, collapsing every path.
    prim = _FakePrim("/World/tilesets/a", "https://old:8088/deep/path/tileset.json")
    stage = _stage([prim])
    assert apply_tileset_server_url(stage, "http://new:9000", "/World/tilesets") == 1
    assert prim.attribute.Get() == "http://new:9000/deep/path/tileset.json"


def test_override_with_trailing_slash_is_normalised() -> None:
    prim = _FakePrim("/World/tilesets/a", "http://old:8000/tiles/a/tileset.json")
    stage = _stage([prim])
    assert apply_tileset_server_url(stage, "http://new:9000/", "/World/tilesets") == 1
    assert prim.attribute.Get() == "http://new:9000/tiles/a/tileset.json"


def test_override_without_scheme_defaults_to_http() -> None:
    prim = _FakePrim("/World/tilesets/a", "http://old:8000/tiles/a/tileset.json")
    stage = _stage([prim])
    assert apply_tileset_server_url(stage, "newhost:9000", "/World/tilesets") == 1
    assert prim.attribute.Get() == "http://newhost:9000/tiles/a/tileset.json"


def test_override_without_port_keeps_the_original_path() -> None:
    prim = _FakePrim("/World/tilesets/a", "http://old:8000/tiles/a/tileset.json")
    stage = _stage([prim])
    assert apply_tileset_server_url(stage, "http://newhost", "/World/tilesets") == 1
    assert prim.attribute.Get() == "http://newhost/tiles/a/tileset.json"


def test_ipv6_override_is_parsed_correctly() -> None:
    # Hand-rolled splitting on ':' breaks IPv6 literals; urllib gets it right.
    prim = _FakePrim("/World/tilesets/a", "http://old:8000/tiles/a/tileset.json")
    stage = _stage([prim])
    assert apply_tileset_server_url(stage, "http://[2001:db8::1]:9000", "/World/tilesets") == 1
    assert prim.attribute.Get() == "http://[2001:db8::1]:9000/tiles/a/tileset.json"


def test_query_string_is_preserved() -> None:
    prim = _FakePrim("/World/tilesets/a", "http://old:8000/tiles/a/tileset.json?key=abc123")
    stage = _stage([prim])
    assert apply_tileset_server_url(stage, "http://new:9000", "/World/tilesets") == 1
    assert prim.attribute.Get() == "http://new:9000/tiles/a/tileset.json?key=abc123"


def test_override_path_is_used_as_a_prefix() -> None:
    prim = _FakePrim("/World/tilesets/a", "http://old:8000/cityA/tileset.json")
    stage = _stage([prim])
    assert apply_tileset_server_url(stage, "http://new:9000/mount", "/World/tilesets") == 1
    assert prim.attribute.Get() == "http://new:9000/mount/cityA/tileset.json"


def test_relative_original_url_gains_the_new_server() -> None:
    # A path-only original is treated as a path under the new server, not left server-less.
    prim = _FakePrim("/World/tilesets/a", "tiles/a/tileset.json")
    stage = _stage([prim])
    assert apply_tileset_server_url(stage, "http://new:9000", "/World/tilesets") == 1
    assert prim.attribute.Get() == "http://new:9000/tiles/a/tileset.json"


@pytest.mark.parametrize("url", [None, ""])
def test_no_override_leaves_the_scene_untouched(url: str | None) -> None:
    # The default. A scene that already points at the right server must not be rewritten.
    prim = _FakePrim("/World/tilesets/a", "http://scene:8000/a/tileset.json")
    stage = _stage([prim])
    assert apply_tileset_server_url(stage, url, "/World/tilesets") == 0
    assert prim.attribute.Get() == "http://scene:8000/a/tileset.json"


def test_an_already_matching_server_is_not_rewritten() -> None:
    # Avoids a pointless USD authoring op and a misleading log line on every launch.
    prim = _FakePrim("/World/tilesets/a", "http://same:9000/a/tileset.json")
    stage = _stage([prim])
    assert apply_tileset_server_url(stage, "http://same:9000", "/World/tilesets") == 0


def test_empty_url_warns_and_is_left_alone() -> None:
    # A tileset prim whose cesium:url exists but is empty must not have one fabricated.
    prim = _FakePrim("/World/tilesets/a", "")
    stage = _stage([prim])
    with _captured_warnings() as records:
        assert apply_tileset_server_url(stage, "http://new:9000", "/World/tilesets") == 0
    assert prim.attribute.Get() == ""
    assert any("empty" in r.getMessage() and "/World/tilesets/a" in r.getMessage() for r in records)


def test_present_but_unset_url_warns_and_is_left_alone() -> None:
    # An authored-but-never-set cesium:url attribute: valid attribute, None value.
    prim = _FakePrim("/World/tilesets/a", None, has_url_attr=True)
    stage = _stage([prim])
    with _captured_warnings() as records:
        assert apply_tileset_server_url(stage, "http://new:9000", "/World/tilesets") == 0
    assert prim.attribute.Get() is None
    assert any("empty" in r.getMessage() and "/World/tilesets/a" in r.getMessage() for r in records)


def test_unparseable_override_leaves_everything_untouched() -> None:
    # A bad config value must not silently blank out the terrain.
    prim = _FakePrim("/World/tilesets/a", "http://scene:8000/a/tileset.json")
    stage = _stage([prim])
    with _captured_warnings() as records:
        assert apply_tileset_server_url(stage, "http://", "/World/tilesets") == 0
    assert prim.attribute.Get() == "http://scene:8000/a/tileset.json"
    assert any("no host" in r.getMessage() for r in records)


def test_a_missing_tilesets_root_is_survivable() -> None:
    # A scene without terrain should still run rather than raising.
    stage = _stage([], root_valid=False)
    assert apply_tileset_server_url(stage, "http://new:9000", "/World/tilesets") == 0


def test_prims_without_a_url_attribute_are_skipped() -> None:
    prims = [
        _FakePrim("/World/tilesets/scope", None),
        _FakePrim("/World/tilesets/a", "http://old:8000/a/tileset.json"),
    ]
    stage = _stage(prims)
    assert apply_tileset_server_url(stage, "http://new:9000", "/World/tilesets") == 1
    assert prims[1].attribute.Get() == "http://new:9000/a/tileset.json"
