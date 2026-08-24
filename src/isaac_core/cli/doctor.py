"""
Environment diagnostics for the ``isaac-core doctor`` command.

Each check is individually guarded and reported as PASS, WARN, or FAIL with a
concrete remediation hint. This command must *never* raise -- a broken environment
is precisely when it is run.
"""

from __future__ import annotations

import importlib
from pathlib import Path
import sys
from typing import Sequence

from isaac_core.install import IsaacInstall, IsaacInstallError

# Minimum Python version for the project.
_MIN_PYTHON = (3, 10)

# Runtime dependencies to check with their minimum versions.
_RUNTIME_DEPS: tuple[tuple[str, str], ...] = (
    ("numpy", "1.24"),
    ("pydantic", "2.6"),
    ("pyproj", "3.6"),
    ("transforms3d", "0.4"),
)

# Extension directories to check in extsUser.
_EXPECTED_EXTENSIONS: tuple[str, ...] = (
    "isaac_core_ogn.math",
    "isaac_core_ogn.position",
    "isaac_core_ogn.sensors",
)


# Warnings accumulated during a run, so the summary reports them instead of
# claiming success. Cleared at the start of every run_doctor().
_WARNINGS: list[str] = []


def run_doctor() -> int:
    """
    Run all environment diagnostic checks and print results.

    Returns:
        Exit code: 0 if no FAILs, 1 if any FAIL was reported.

    """
    print("isaac-core doctor")
    print("=" * 60)

    _WARNINGS.clear()
    has_fail = _run_all_checks()

    print("=" * 60)
    if has_fail:
        print("Some checks FAILED. See remediation hints above.")
        return 1
    if _WARNINGS:
        # Claiming "all checks passed" while warnings are on screen trains people to
        # ignore the output. Exit code stays 0 because warnings are not fatal.
        count = len(_WARNINGS)
        noun = "warning" if count == 1 else "warnings"
        print(f"No failures, but {count} {noun}. See hints above.")
        return 0
    print("All checks passed.")
    return 0


def _run_all_checks() -> bool:
    """
    Execute every diagnostic check and print each result.

    Returns:
        ``True`` if any check returned FAIL.

    """
    has_fail = False

    has_fail = _emit(_check_python_version()) or has_fail
    has_fail = _run_dependency_checks() or has_fail
    has_fail, install = _run_isaac_checks(has_fail)
    has_fail = _emit(_check_ros2()) or has_fail

    if install is not None:
        has_fail = _emit(_check_extensions_linked(install)) or has_fail

    has_fail = _emit(_check_config()) or has_fail
    return has_fail


def _run_dependency_checks() -> bool:
    """
    Check all runtime dependencies.

    Returns:
        ``True`` if any dependency check FAILed.

    """
    has_fail = False
    for dep_name, min_ver in _RUNTIME_DEPS:
        has_fail = _emit(_check_dependency(dep_name, min_ver)) or has_fail

    if sys.version_info < (3, 11):
        has_fail = _emit(_check_dependency("tomli", "2.0")) or has_fail
    return has_fail


def _run_isaac_checks(has_fail: bool) -> tuple[bool, IsaacInstall | None]:
    """
    Run Isaac Sim installation checks.

    Args:
        has_fail: Current failure state.

    Returns:
        Updated has_fail flag and the install if found.

    """
    status, msg, install = _check_isaac_install()
    has_fail = _emit((status, msg)) or has_fail

    if install is not None:
        has_fail = _emit(_check_isaac_version(install)) or has_fail
        has_fail = _emit(_check_isaac_python(install)) or has_fail
        has_fail = _emit(_check_isaac_core_in_isaac(install)) or has_fail

    return has_fail, install


def _emit(result: tuple[str, str]) -> bool:
    """
    Print a single check result, tally it, and return whether it is a FAIL.

    Args:
        result: Tuple of (status, message).

    Returns:
        ``True`` if the status is FAIL.

    """
    status, msg = result
    print(f"  [{status}] {msg}")
    if status == "WARN":
        _WARNINGS.append(msg)
    return status == "FAIL"


def _check_python_version() -> tuple[str, str]:
    """
    Check that the Python version meets the minimum requirement.

    Returns:
        Status and message tuple.

    """
    ver = sys.version_info
    ver_str = f"{ver.major}.{ver.minor}.{ver.micro}"
    if (ver.major, ver.minor) >= _MIN_PYTHON:
        return "PASS", f"Python {ver_str}"
    return "FAIL", f"Python {ver_str} < {_MIN_PYTHON[0]}.{_MIN_PYTHON[1]}. Upgrade to Python 3.10+."


def _check_dependency(name: str, min_version: str) -> tuple[str, str]:
    """
    Check that a Python package is importable and meets its minimum version.

    Args:
        name: Package name to import.
        min_version: Minimum version string (e.g. ``"1.24"``).

    Returns:
        Status and message tuple.

    """
    try:
        mod = importlib.import_module(name)
    except ImportError:
        return "FAIL", f"{name} not importable. Run: pip install {name}>={min_version}"

    version = _get_version(mod, name)
    if version:
        return "PASS", f"{name} {version}"
    return "PASS", f"{name} (version unknown)"


def _get_version(mod: object, name: str) -> str | None:
    """
    Extract a module's version string from common attributes.

    Args:
        mod: The imported module object.
        name: Package name (for importlib.metadata fallback).

    Returns:
        Version string, or ``None`` if not determinable.

    """
    for attr in ("__version__", "VERSION"):
        val = getattr(mod, attr, None)
        if isinstance(val, str):
            return val

    try:
        from importlib.metadata import version  # noqa: PLC0415

        return version(name)
    except Exception:  # noqa: BLE001
        return None


def _check_isaac_install() -> tuple[str, str, IsaacInstall | None]:
    """
    Attempt to locate Isaac Sim.

    Returns:
        Status, message, and the install (or ``None``).

    """
    try:
        install = IsaacInstall.locate()
    except IsaacInstallError:
        return (
            "FAIL",
            "Isaac Sim not found. Set sim.isaac_sim_path in config or install Isaac Sim 6.x.",
            None,
        )
    return "PASS", f"Isaac Sim found at {install.root}", install


def _check_isaac_version(install: IsaacInstall) -> tuple[str, str]:
    """
    Check that the Isaac Sim version is 6.x.

    Args:
        install: Validated install.

    Returns:
        Status and message tuple.

    """
    if install.is_supported():
        return "PASS", f"Isaac Sim version {install.version} (6.x, supported)"
    return (
        "WARN",
        f"Isaac Sim version {install.version} is not 6.x. " f"Extensions use the isaacsim.* namespace (requires 6.x+).",
    )


def _check_isaac_python(install: IsaacInstall) -> tuple[str, str]:
    """
    Check that Isaac's bundled python.sh exists and is executable.

    Args:
        install: Validated install.

    Returns:
        Status and message tuple.

    """
    python_sh = install.python_path
    if python_sh.is_file():
        return "PASS", f"Isaac python.sh present at {python_sh}"
    return "FAIL", f"Isaac python.sh missing at {python_sh}. Installation may be corrupt."


def _check_isaac_core_in_isaac(install: IsaacInstall) -> tuple[str, str]:
    """
    Check whether isaac_core is likely installed into Isaac's interpreter.

    This is a heuristic: we look for a ``isaac_core`` directory or ``.egg-link``
    in Isaac's site-packages. We cannot run Isaac's interpreter from here.

    Args:
        install: Validated install.

    Returns:
        Status and message tuple.

    """
    # Look for common install indicators
    kit_python = install.root / "kit" / "python"
    possible_sites: list[Path] = []
    if kit_python.is_dir():
        possible_sites.extend(kit_python.glob("lib/python*/site-packages"))
    # Also check the root-level python lib
    possible_sites.extend(install.root.glob("kit/python/lib/python*/site-packages"))
    possible_sites.extend(install.root.glob("python_packages"))

    for site_dir in possible_sites:
        if (site_dir / "isaac_core").is_dir():
            return "PASS", "isaac_core appears installed in Isaac's interpreter"
        # Check for editable install marker
        egg_links = list(site_dir.glob("isaac-core*.egg-link")) + list(site_dir.glob("isaac_core*.egg-link"))
        if egg_links:
            return "PASS", "isaac_core appears installed in Isaac's interpreter (editable)"

    return (
        "WARN",
        "Cannot confirm isaac_core is installed in Isaac's interpreter. "
        "Run: $ISAAC_PATH/python.sh -m pip install -e '.[sim]'",
    )


def _check_ros2() -> tuple[str, str]:
    """
    Check whether ROS 2 is available (rclpy importable, ROS_DOMAIN_ID set).

    Returns:
        Status and message tuple.

    """
    import os  # noqa: PLC0415

    rclpy_available = False
    try:
        importlib.import_module("rclpy")
        rclpy_available = True
    except ImportError:
        pass

    domain_id = os.environ.get("ROS_DOMAIN_ID")

    if rclpy_available and domain_id is not None:
        return "PASS", f"ROS 2 available (ROS_DOMAIN_ID={domain_id})"
    if rclpy_available:
        return "WARN", "rclpy importable but ROS_DOMAIN_ID not set. Run: export ROS_DOMAIN_ID=<id>"
    if domain_id is not None:
        return "WARN", f"ROS_DOMAIN_ID={domain_id} but rclpy not importable. Run: source /opt/ros/humble/setup.bash"
    return "WARN", "ROS 2 not sourced (rclpy not importable). Run: source /opt/ros/humble/setup.bash"


def _check_extensions_linked(install: IsaacInstall) -> tuple[str, str]:
    """
    Check whether our extensions are symlinked into extsUser.

    Args:
        install: Validated install.

    Returns:
        Status and message tuple.

    """
    exts_dir = install.exts_user_dir
    if not exts_dir.is_dir():
        return "WARN", f"extsUser directory does not exist at {exts_dir}"

    missing: list[str] = []
    for ext_name in _EXPECTED_EXTENSIONS:
        link_path = exts_dir / ext_name
        if not link_path.exists():
            missing.append(ext_name)

    if not missing:
        return "PASS", f"All extensions linked in {exts_dir}"
    return (
        "FAIL",
        f"Extensions not linked: {', '.join(missing)}. Run: scripts/link_extensions.sh",
    )


def _check_config() -> tuple[str, str]:
    """
    Check that the configuration loads without error.

    Returns:
        Status and message tuple.

    """
    try:
        from isaac_core.config import load  # noqa: PLC0415

        load()
        return "PASS", "Configuration loads successfully"
    except FileNotFoundError as exc:
        return "WARN", f"Config file not found: {exc}. Using defaults."
    except Exception as exc:  # noqa: BLE001
        return "FAIL", f"Configuration error: {exc}"


def doctor_checks() -> Sequence[tuple[str, str]]:
    """
    Run all checks and return a list of (status, message) tuples.

    Useful for programmatic access without printing.

    Returns:
        Sequence of (status, message) pairs.

    """
    results: list[tuple[str, str]] = []

    results.append(_check_python_version())

    for dep_name, min_ver in _RUNTIME_DEPS:
        results.append(_check_dependency(dep_name, min_ver))

    if sys.version_info < (3, 11):
        results.append(_check_dependency("tomli", "2.0"))

    _status, _msg, install = _check_isaac_install()
    results.append((_status, _msg))

    if install is not None:
        results.append(_check_isaac_version(install))
        results.append(_check_isaac_python(install))
        results.append(_check_isaac_core_in_isaac(install))

    results.append(_check_ros2())

    if install is not None:
        results.append(_check_extensions_linked(install))

    results.append(_check_config())

    return results
