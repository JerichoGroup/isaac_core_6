"""
Verify kernel packages import with no Isaac Sim and no ROS 2 present.

The whole layering exists to guarantee this property: the kernel (contracts,
config, geo, protocol) is pure Python, usable in CI without a GPU, without
Isaac Sim, and without ROS 2.  These tests prove it by importing each package
in a subprocess where omni, carb, pxr, rclpy, isaacsim, and gi are all blocked
via a sys.meta_path finder that raises ImportError.
"""

from pathlib import Path
import subprocess
import sys

import pytest

# The blocker script injects a meta-path finder that makes the named modules
# unimportable, then imports the target package and exits 0 on success.
_BLOCKER_SCRIPT = """\
import sys

class _BlockImports:
    BLOCKED = frozenset({blocked!r})

    def find_module(self, fullname, path=None):
        top = fullname.split(".")[0]
        if top in self.BLOCKED:
            return self

    def load_module(self, fullname):
        raise ImportError(
            f"{{fullname}} is intentionally blocked to verify kernel purity"
        )

sys.meta_path.insert(0, _BlockImports())

import {target}
print(f"OK: {target} imported successfully with blocked modules")
"""

# Modules that must be unimportable during the test.
_BLOCKED_MODULES = ("omni", "carb", "pxr", "rclpy", "gi", "isaacsim")

# Kernel packages that must all import cleanly without the above.
_KERNEL_PACKAGES = (
    "isaac_core.contracts",
    "isaac_core.config",
    "isaac_core.geo",
    "isaac_core.protocol",
    "isaac_core.vehicle",
    "isaac_core.sim",
)

# Non-kernel packages that also must import without Isaac/ROS/GStreamer.
# This is the property that lets the team script a remote simulator from a
# laptop with no Isaac and no ROS installed.
_SCRIPTABLE_PACKAGES = (
    "isaac_core.control",
    "isaac_core.cli",
    "isaac_core.devkit",
    "isaac_core.debug",
)

# The src directory needs to be on PYTHONPATH for the subprocess.
_SRC_DIR = str(Path(__file__).resolve().parents[2] / "src")


@pytest.mark.parametrize("package", _KERNEL_PACKAGES)
def test_kernel_imports_without_sim_or_ros(package: str) -> None:
    # Each kernel package must be importable in a clean subprocess where all
    # Isaac Sim and ROS 2 dependencies are blocked.
    script = _BLOCKER_SCRIPT.format(blocked=_BLOCKED_MODULES, target=package)
    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        env={"PYTHONPATH": _SRC_DIR, "PATH": ""},
        timeout=30,
    )
    assert result.returncode == 0, (
        f"Importing {package} failed with blocked modules.\n" f"stdout: {result.stdout}\n" f"stderr: {result.stderr}"
    )


@pytest.mark.parametrize("package", _SCRIPTABLE_PACKAGES)
def test_scriptable_packages_import_without_sim_or_ros(package: str) -> None:
    # control, cli and devkit must all import cleanly with omni/carb/pxr/rclpy/gi
    # blocked. This guarantees the team can script a remote simulator from a
    # laptop with no Isaac Sim and no ROS 2 installed.
    script = _BLOCKER_SCRIPT.format(blocked=_BLOCKED_MODULES, target=package)
    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        env={"PYTHONPATH": _SRC_DIR, "PATH": ""},
        timeout=30,
    )
    assert result.returncode == 0, (
        f"Importing {package} failed with blocked modules.\n" f"stdout: {result.stdout}\n" f"stderr: {result.stderr}"
    )


def test_debug_imports_without_tkinter() -> None:
    # python3-tk is a separate apt package, so it may be absent on a headless machine or
    # in CI. Importing the module must still work: the inspector needs no GUI at all, and
    # the sender must be able to report a clear "install python3-tk" message rather than
    # dying with an ImportError at import time.
    blocked = (*_BLOCKED_MODULES, "tkinter")
    script = _BLOCKER_SCRIPT.format(blocked=blocked, target="isaac_core.debug.pose_sender_gui")
    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        env={"PYTHONPATH": _SRC_DIR, "PATH": ""},
        timeout=30,
        check=False,
    )
    assert result.returncode == 0, f"import failed with tkinter blocked:\n{result.stderr}"


def test_inspector_imports_without_tkinter() -> None:
    blocked = (*_BLOCKED_MODULES, "tkinter")
    script = _BLOCKER_SCRIPT.format(blocked=blocked, target="isaac_core.debug.inspector")
    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        env={"PYTHONPATH": _SRC_DIR, "PATH": ""},
        timeout=30,
        check=False,
    )
    assert result.returncode == 0, f"import failed with tkinter blocked:\n{result.stderr}"
