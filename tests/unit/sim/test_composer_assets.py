"""Tests for the two filesystem-facing composer helpers that were reported broken.

`delete_cesium_cache` is fully testable without Isaac. `apply_hdri` needs pxr for the real
path, so only its guard clauses (empty path, missing file) are covered here; the dome-light
creation is verified live.
"""

from __future__ import annotations

from pathlib import Path

from isaac_core.sim.composer import apply_hdri, delete_cesium_cache, resolve_hdri


def test_deletes_all_cesium_cache_files(tmp_path: Path) -> None:
    # The .sqlite plus its -wal and -shm companions, which is what actually holds the GB.
    for suffix in ("", "-wal", "-shm"):
        (tmp_path / f"cesium-request-cache.sqlite{suffix}").write_text("x", encoding="utf-8")
    (tmp_path / "unrelated.txt").write_text("keep", encoding="utf-8")

    removed = delete_cesium_cache(cache_dir=tmp_path)

    assert removed == 3
    assert not list(tmp_path.glob("cesium-request-cache.sqlite*"))
    assert (tmp_path / "unrelated.txt").exists(), "must not touch unrelated files"


def test_deleting_an_absent_cache_is_a_no_op(tmp_path: Path) -> None:
    assert delete_cesium_cache(cache_dir=tmp_path) == 0


def test_deleting_when_the_dir_is_missing_does_not_raise(tmp_path: Path) -> None:
    assert delete_cesium_cache(cache_dir=tmp_path / "nope") == 0


def test_apply_hdri_ignores_empty_path() -> None:
    # The default. A scene with its own lighting must be left alone. Passing None/"" must
    # not touch the stage, so this can assert without any USD at all.
    assert apply_hdri(stage=None, hdri="") is False
    assert apply_hdri(stage=None, hdri=None) is False


def test_apply_hdri_skips_a_missing_file(tmp_path: Path) -> None:
    # A wrong path should warn and no-op, not crash the launch.
    assert apply_hdri(stage=None, hdri=str(tmp_path / "not-here.exr")) is False


def test_a_bare_hdri_name_is_found_on_the_asset_search_paths(tmp_path: Path) -> None:
    # Reported from a fresh machine: setting search_paths and naming the image alone did nothing,
    # because the value went straight to Path() and was resolved against the working directory.
    assets = tmp_path / "assets"
    assets.mkdir()
    image = assets / "sunflowers_puresky_4k.exr"
    image.write_bytes(b"not really an exr")
    assert resolve_hdri("sunflowers_puresky_4k.exr", [assets]) == image.resolve()


def test_an_absolute_hdri_path_is_used_as_given(tmp_path: Path) -> None:
    image = tmp_path / "sky.exr"
    image.write_bytes(b"x")
    assert resolve_hdri(str(image), []) == image.resolve()


def test_a_bare_hdri_name_without_search_paths_is_unresolved() -> None:
    assert resolve_hdri("nowhere-to-look.exr", []) is None


def test_a_missing_absolute_hdri_is_not_searched_for_elsewhere(tmp_path: Path) -> None:
    # An absolute path that does not exist is a mistake to report, not a name to go hunting for.
    assets = tmp_path / "assets"
    assets.mkdir()
    (assets / "sky.exr").write_bytes(b"x")
    assert resolve_hdri("/definitely/not/here/sky.exr", [assets]) is None


def test_a_tilde_hdri_path_is_expanded(tmp_path: Path, monkeypatch: object) -> None:
    image = tmp_path / "home-sky.exr"
    image.write_bytes(b"x")
    import os

    previous = os.environ.get("HOME")
    os.environ["HOME"] = str(tmp_path)
    try:
        assert resolve_hdri("~/home-sky.exr", []) == image.resolve()
    finally:
        if previous is None:
            del os.environ["HOME"]
        else:
            os.environ["HOME"] = previous


def test_apply_hdri_still_refuses_what_cannot_be_resolved(tmp_path: Path) -> None:
    assert apply_hdri(stage=None, hdri="missing.exr", search_paths=[tmp_path]) is False
