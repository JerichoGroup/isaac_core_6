"""
Tests for repointing Cesium tilesets at a different server from config.

The tile server URL is otherwise baked into the scene USD, so changing it means hand-editing
a `.usda` in the GUI. The previous generation had this feature; this restores it.

Driven with fakes so no Isaac Sim or USD is required.
"""

from __future__ import annotations

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

    def __init__(self, path: str, url: str | None, valid: bool = True) -> None:
        """Build a prim carrying a ``cesium:url`` attribute unless told otherwise."""
        self._path = path
        self._valid = valid
        self.attribute = _FakeAttribute(url, valid=url is not None)

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


def test_a_tileset_url_is_replaced() -> None:
    prim = _FakePrim("/World/tilesets/Cesium_Tileset", "http://old:8088/a/tileset.json")
    stage = _stage([prim])
    changed = apply_tileset_server_url(stage, "http://new:9000/b/tileset.json", "/World/tilesets")
    assert changed == 1
    assert prim.attribute.Get() == "http://new:9000/b/tileset.json"


def test_every_tileset_in_the_subtree_is_replaced() -> None:
    prims = [
        _FakePrim("/World/tilesets/a", "http://old/a.json"),
        _FakePrim("/World/tilesets/b", "http://old/b.json"),
    ]
    stage = _stage(prims)
    assert apply_tileset_server_url(stage, "http://new/x.json", "/World/tilesets") == 2
    assert all(p.attribute.Get() == "http://new/x.json" for p in prims)


@pytest.mark.parametrize("url", [None, ""])
def test_no_url_leaves_the_scene_untouched(url: str | None) -> None:
    # The default. A scene that already points at the right server must not be rewritten.
    prim = _FakePrim("/World/tilesets/a", "http://scene/a.json")
    stage = _stage([prim])
    assert apply_tileset_server_url(stage, url, "/World/tilesets") == 0
    assert prim.attribute.Get() == "http://scene/a.json"


def test_a_url_that_already_matches_is_not_rewritten() -> None:
    # Avoids a pointless USD authoring op and a misleading log line on every launch.
    prim = _FakePrim("/World/tilesets/a", "http://same/a.json")
    stage = _stage([prim])
    assert apply_tileset_server_url(stage, "http://same/a.json", "/World/tilesets") == 0


def test_a_missing_tilesets_root_is_survivable() -> None:
    # A scene without terrain should still run rather than raising.
    stage = _stage([], root_valid=False)
    assert apply_tileset_server_url(stage, "http://new/a.json", "/World/tilesets") == 0


def test_prims_without_a_url_attribute_are_skipped() -> None:
    prims = [
        _FakePrim("/World/tilesets/scope", None),
        _FakePrim("/World/tilesets/a", "http://old/a.json"),
    ]
    stage = _stage(prims)
    assert apply_tileset_server_url(stage, "http://new/a.json", "/World/tilesets") == 1
