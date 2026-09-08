"""Tests for applying Cesium tile-loading settings to tileset prims."""

from collections.abc import Iterator
from contextlib import contextmanager
import logging
import sys
import types

import pytest

from isaac_core.config import IsaacCoreConfig
from isaac_core.sim import composer


class _FakeAttr:
    """Records the value set on it."""

    def __init__(self, *, valid: bool = True) -> None:
        """Create an attribute, optionally invalid."""
        self._valid = valid
        self.value: object = None

    def IsValid(self) -> bool:  # noqa: N802 - USD API name
        """Report validity."""
        return self._valid

    def Set(self, value: object) -> None:  # noqa: N802 - USD API name
        """Record the value."""
        self.value = value


class _FakePrim:
    """A tileset-like prim that tracks created attributes."""

    def __init__(self, *, is_tileset: bool = True) -> None:
        """Create a prim that either has a cesium:url or does not."""
        self.attrs: dict[str, _FakeAttr] = {}
        if is_tileset:
            self.attrs["cesium:url"] = _FakeAttr()

    def IsValid(self) -> bool:  # noqa: N802 - USD API name
        """Report validity."""
        return True

    def GetAttribute(self, name: str) -> _FakeAttr:  # noqa: N802 - USD API name
        """Return the named attribute, or an invalid placeholder."""
        return self.attrs.get(name, _FakeAttr(valid=False))

    def CreateAttribute(self, name: str, _type: object) -> _FakeAttr:  # noqa: N802 - USD API name
        """Create and return the named attribute."""
        self.attrs[name] = _FakeAttr()
        return self.attrs[name]


class _FakeStage:
    """Returns a fixed root prim."""

    def __init__(self, root: object) -> None:
        """Store the root."""
        self._root = root

    def GetPrimAtPath(self, _path: object) -> object:  # noqa: N802 - USD API name
        """Return the root prim."""
        return self._root


@contextmanager
def _fake_pxr(prims: list[_FakePrim]) -> Iterator[None]:
    """Install fake pxr modules so the composer runs without Isaac Sim."""
    sdf = types.ModuleType("pxr.Sdf")
    sdf.Path = str  # type: ignore[attr-defined]
    sdf.ValueTypeNames = types.SimpleNamespace(UInt="uint", Float="float", UInt64="uint64", Bool="bool")  # type: ignore[attr-defined]
    usd = types.ModuleType("pxr.Usd")
    usd.PrimRange = lambda _root: prims  # type: ignore[attr-defined]
    saved = {name: sys.modules.get(name) for name in ("pxr.Sdf", "pxr.Usd")}
    sys.modules["pxr.Sdf"] = sdf
    sys.modules["pxr.Usd"] = usd
    try:
        yield
    finally:
        for name, module in saved.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


def test_nothing_is_written_when_no_tuning_is_configured() -> None:
    # The default must leave Cesium's own defaults alone. Pinning them would silently freeze
    # values that Cesium may improve in a later release.
    prim = _FakePrim()
    with _fake_pxr([prim]):
        written = composer.apply_tile_tuning(_FakeStage(prim), IsaacCoreConfig().cesium, "/World/tilesets")

    assert written == 0
    assert "cesium:maximumSimultaneousTileLoads" not in prim.attrs


def test_configured_settings_reach_the_tileset() -> None:
    """Write each configured setting onto the tileset prim."""
    prim = _FakePrim()
    cesium = IsaacCoreConfig(
        cesium={"max_simultaneous_tile_loads": 4, "max_cached_bytes": 2147483648, "preload_siblings": False}
    ).cesium
    with _fake_pxr([prim]):
        written = composer.apply_tile_tuning(_FakeStage(prim), cesium, "/World/tilesets")

    assert written == 3
    assert prim.attrs["cesium:maximumSimultaneousTileLoads"].value == 4
    assert prim.attrs["cesium:maximumCachedBytes"].value == 2147483648
    assert prim.attrs["cesium:preloadSiblings"].value is False


def test_non_tileset_prims_are_skipped() -> None:
    # The tilesets scope can hold other prims; writing Cesium attributes onto them would be
    # meaningless clutter.
    tileset, other = _FakePrim(), _FakePrim(is_tileset=False)
    cesium = IsaacCoreConfig(cesium={"max_simultaneous_tile_loads": 8}).cesium
    with _fake_pxr([tileset, other]):
        written = composer.apply_tile_tuning(_FakeStage(tileset), cesium, "/World/tilesets")

    assert written == 1
    assert "cesium:maximumSimultaneousTileLoads" not in other.attrs


@pytest.mark.parametrize("field", ["max_simultaneous_tile_loads", "max_cached_bytes"])
def test_non_positive_values_are_rejected_at_config_load(field: str) -> None:
    """Reject a zero or negative setting rather than passing it to Cesium."""
    with pytest.raises(ValueError, match="greater than"):
        IsaacCoreConfig(cesium={field: 0})


def test_missing_tilesets_root_warns_and_writes_nothing() -> None:
    """Warn rather than raise when the scene has no tilesets scope."""

    class _Absent:
        def IsValid(self) -> bool:  # noqa: N802 - USD API name
            return False

    records: list[logging.LogRecord] = []

    class _Handler(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            records.append(record)

    handler = _Handler()
    logger = logging.getLogger("isaac_core.sim.composer")
    logger.addHandler(handler)
    cesium = IsaacCoreConfig(cesium={"max_simultaneous_tile_loads": 4}).cesium
    try:
        with _fake_pxr([]):
            written = composer.apply_tile_tuning(_FakeStage(_Absent()), cesium, "/World/tilesets")
    finally:
        logger.removeHandler(handler)

    assert written == 0
    assert any("tilesets root" in r.getMessage() for r in records)
