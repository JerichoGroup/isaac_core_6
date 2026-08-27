"""
Tests for the two settings that make a GUI run show anything useful.

Both failures they guard against are silent. A `CesiumTilesetPrim` without the Cesium
extension is perfectly valid USD that draws nothing, and a viewport left on Kit's default
perspective camera looks frozen while the aircraft camera tracks correctly behind it. The
symptom in both cases is "blank screen, camera does not respond", with no error anywhere.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from isaac_core.config import IsaacCoreConfig
from isaac_core.sim.runtime import SimulationRuntime


class _BareRuntime(SimulationRuntime):
    """A runtime with the Isaac-dependent constructor bypassed."""

    def __init__(self, config: IsaacCoreConfig) -> None:
        """Store only the config; nothing else is needed for these tests."""
        self._config = config


def test_cesium_is_enabled_at_boot_not_afterwards() -> None:
    # An extension contributing USD schemas must be enabled before the schema registry
    # initialises. Enabling `cesium.omniverse` after startup reported success but left
    # CesiumTilesetPrim unregistered: 5 raw attributes, IsA(Xformable) false, no terrain.
    # At boot: registered schema, 28 attributes, IsA(Xformable) true.
    sim = IsaacCoreConfig().sim
    assert "cesium.omniverse" in sim.boot_extensions
    assert "cesium.usd.plugins" in sim.boot_extensions, "the schema plugin must load too"
    assert "cesium.omniverse" not in sim.extensions, "would double-enable, after boot"


def test_boot_extensions_become_kit_enable_arguments() -> None:
    config = IsaacCoreConfig(sim={"boot_extensions": ("some.ext",), "extension_search_paths": ()})
    args = _BareRuntime(config)._kit_startup_args()
    assert args[:2] == ["--enable", "some.ext"]


def test_the_full_kit_experience_is_the_default() -> None:
    # The minimal `isaacsim.exp.base.python.kit` behaves differently from the editor the
    # team authors USD in, which makes "works in the GUI, not from the CLI" hard to reason
    # about. Keeping them the same avoids a class of confusion.
    assert IsaacCoreConfig().sim.experience == "isaacsim.exp.full.kit"


def test_an_empty_experience_accepts_isaacs_default() -> None:
    config = IsaacCoreConfig(sim={"experience": ""})
    assert _BareRuntime(config)._resolve_experience() == ""


def test_a_missing_absolute_experience_falls_back(tmp_path: Path) -> None:
    # Better to run with Isaac's default than to fail outright on a machine difference.
    config = IsaacCoreConfig(sim={"experience": str(tmp_path / "nope.kit")})
    assert _BareRuntime(config)._resolve_experience() == ""


def test_an_existing_absolute_experience_is_used(tmp_path: Path) -> None:
    kit = tmp_path / "custom.kit"
    kit.write_text("", encoding="utf-8")
    config = IsaacCoreConfig(sim={"experience": str(kit)})
    assert _BareRuntime(config)._resolve_experience() == str(kit)


def test_a_bare_experience_name_resolves_against_exp_path(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    kit = tmp_path / "isaacsim.exp.full.kit"
    kit.write_text("", encoding="utf-8")
    monkeypatch.setenv("EXP_PATH", str(tmp_path))
    config = IsaacCoreConfig(sim={"experience": "isaacsim.exp.full.kit"})
    assert _BareRuntime(config)._resolve_experience() == str(kit)


def test_a_bare_name_without_exp_path_falls_back(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("EXP_PATH", raising=False)
    config = IsaacCoreConfig(sim={"experience": "isaacsim.exp.full.kit"})
    assert _BareRuntime(config)._resolve_experience() == ""


def test_the_omniverse_user_extension_registry_is_searched_by_default() -> None:
    # Cesium is installed through the GUI extension manager, which puts it here rather than
    # under the Isaac install. Isaac's python experience does not search it on its own.
    paths = IsaacCoreConfig().sim.extension_search_paths
    assert any("exts" in path for path in paths), f"no extension registry path configured: {paths}"


def test_extension_folders_become_kit_arguments() -> None:
    # Point at a directory that certainly exists so the filter keeps it.
    config = IsaacCoreConfig(sim={"extension_search_paths": (str(Path(__file__).parent),), "boot_extensions": ()})
    args = _BareRuntime(config)._kit_startup_args()
    assert args[0] == "--ext-folder"
    assert Path(args[1]).is_dir()


def test_missing_extension_folders_are_skipped(tmp_path: Path) -> None:
    # A path that exists on one machine and not another must not make Kit complain on
    # every launch, so this degrades quietly.
    config = IsaacCoreConfig(
        sim={"extension_search_paths": (str(tmp_path / "definitely-not-here"),), "boot_extensions": ()}
    )
    assert "--ext-folder" not in _BareRuntime(config)._kit_startup_args()


def test_a_home_relative_extension_folder_is_expanded() -> None:
    # The default is written with a tilde; leaving it unexpanded would silently never match.
    config = IsaacCoreConfig(sim={"extension_search_paths": ("~",), "boot_extensions": ()})
    args = _BareRuntime(config)._kit_startup_args()
    assert args[:2] == ["--ext-folder", str(Path.home())]


def test_viewport_camera_default_targets_the_vehicle_camera() -> None:
    template = IsaacCoreConfig().sim.viewport_camera
    assert "{instance}" in template, "the path must be per-vehicle for swarm support"
    assert template.startswith("/World/Environment/")


def test_headless_does_not_touch_the_viewport() -> None:
    # There is no viewport to retarget, and importing the viewport utility would be a
    # pointless dependency in a headless run. Reaching Isaac code here would raise.
    config = IsaacCoreConfig(sim={"headless": True})
    _BareRuntime(config)._set_viewport_camera()


def test_an_empty_viewport_camera_leaves_the_viewport_alone() -> None:
    # The documented escape hatch for anyone who wants Kit's default perspective camera.
    config = IsaacCoreConfig(sim={"headless": False, "viewport_camera": ""})
    _BareRuntime(config)._set_viewport_camera()


def test_log_arguments_are_omitted_when_isaac_logs_is_wanted() -> None:
    config = IsaacCoreConfig(sim={"extension_search_paths": (), "boot_extensions": ()}, logging={"isaac_logs": True})
    assert _BareRuntime(config)._kit_startup_args() == []


def test_log_level_arguments_are_added_by_default() -> None:
    config = IsaacCoreConfig(sim={"extension_search_paths": (), "boot_extensions": ()})
    args = _BareRuntime(config)._kit_startup_args()
    assert any(arg.startswith("--/log/level=") for arg in args)


def test_domain_id_none_leaves_the_environment_untouched(monkeypatch: pytest.MonkeyPatch) -> None:
    # The default: the bridge inherits whatever ROS_DOMAIN_ID the shell set.
    monkeypatch.setenv("ROS_DOMAIN_ID", "7")
    config = IsaacCoreConfig(ros2={"domain_id": None})
    _BareRuntime(config)._apply_ros_domain()
    import os

    assert os.environ["ROS_DOMAIN_ID"] == "7", "None must not overwrite the environment"


def test_explicit_domain_id_is_exported(monkeypatch: pytest.MonkeyPatch) -> None:
    # Ofer's ask: a config value must actually reach the bridge, which reads the env var.
    monkeypatch.setenv("ROS_DOMAIN_ID", "7")
    config = IsaacCoreConfig(ros2={"domain_id": 42})
    _BareRuntime(config)._apply_ros_domain()
    import os

    assert os.environ["ROS_DOMAIN_ID"] == "42"


def test_domain_id_mismatch_with_env_warns(monkeypatch: pytest.MonkeyPatch) -> None:
    # Ofer's ask: if config sets a domain that disagrees with $ROS_DOMAIN_ID, say so rather
    # than overriding silently.
    import logging

    monkeypatch.setenv("ROS_DOMAIN_ID", "7")
    config = IsaacCoreConfig(ros2={"domain_id": 42})
    records: list[logging.LogRecord] = []
    handler = logging.Handler()
    handler.emit = records.append  # type: ignore[assignment,method-assign]
    logger = logging.getLogger("isaac_core.sim.runtime")
    logger.addHandler(handler)
    logger.setLevel(logging.WARNING)
    try:
        _BareRuntime(config)._apply_ros_domain()
    finally:
        logger.removeHandler(handler)
    assert any("overrides ROS_DOMAIN_ID" in r.getMessage() for r in records)


def test_domain_id_matching_env_does_not_warn(monkeypatch: pytest.MonkeyPatch) -> None:
    import logging

    monkeypatch.setenv("ROS_DOMAIN_ID", "42")
    config = IsaacCoreConfig(ros2={"domain_id": 42})
    records: list[logging.LogRecord] = []
    handler = logging.Handler()
    handler.emit = records.append  # type: ignore[assignment,method-assign]
    logger = logging.getLogger("isaac_core.sim.runtime")
    logger.addHandler(handler)
    logger.setLevel(logging.WARNING)
    try:
        _BareRuntime(config)._apply_ros_domain()
    finally:
        logger.removeHandler(handler)
    assert not [r for r in records if "overrides ROS_DOMAIN_ID" in r.getMessage()]
