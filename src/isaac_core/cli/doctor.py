"""Environment diagnostics for the ``isaac-core doctor`` command.

Each check is individually guarded and reported as PASS, WARN, or FAIL with a
concrete remediation hint. This command must *never* raise -- a broken environment
is precisely when it is run.
"""

from __future__ import annotations

import importlib
from importlib.metadata import version
import os
from pathlib import Path
import subprocess
import sys
from typing import Final, Sequence

from isaac_core.install import IsaacInstall, IsaacInstallError

# Importing isaac_core in Isaac's Python loads no Kit extension, so this is generous.
_ISAAC_IMPORT_TIMEOUT_S: Final = 60.0

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
# Only two extensions ship node definitions. All ROS 2 publish/subscribe work is
# done by Isaac's own C++ isaacsim.ros2.bridge nodes, because Humble's rclpy is a
# Python 3.10 C extension and Isaac Sim 6 runs Python 3.12 -- see docs/ros2_and_python.md.
_EXPECTED_EXTENSIONS: tuple[str, ...] = (
    "isaac_core_ogn.math",
    "isaac_core_ogn.position",
)


# Warnings accumulated during a run, so the summary reports them instead of
# claiming success. Cleared at the start of every run_doctor().
_WARNINGS: list[str] = []


def run_doctor() -> int:
    """Run all environment diagnostic checks and print results.

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
    """Execute every diagnostic check and print each result.

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
    has_fail = _emit(_check_completion()) or has_fail
    return has_fail


def _run_dependency_checks() -> bool:
    """Check all runtime dependencies.

    Returns:
        ``True`` if any dependency check FAILed.

    """
    has_fail = False
    for dep_name, min_ver in _RUNTIME_DEPS:
        has_fail = _emit(_check_dependency(dep_name, min_ver)) or has_fail

    if sys.version_info < (3, 11):
        has_fail = _emit(_check_dependency("tomli", "2.0")) or has_fail
    return has_fail


def _configured_isaac_path() -> str | None:
    """Return ``sim.isaac_sim_path`` from the user's config, or ``None``.

    Read separately from the rest of the doctor so a config that fails to load does not stop the
    install check from running -- the install check is often what explains the config error.

    Returns:
        The configured path, or ``None`` if unset or unreadable.

    """
    try:
        from isaac_core.config import load

        return load().sim.isaac_sim_path
    except Exception:
        return None


def _run_isaac_checks(has_fail: bool) -> tuple[bool, IsaacInstall | None]:
    """Run Isaac Sim installation checks.

    Args:
        has_fail: Current failure state.

    Returns:
        Updated has_fail flag and the install if found.

    """
    status, msg, install = _check_isaac_install(_configured_isaac_path())
    has_fail = _emit((status, msg)) or has_fail

    if install is not None:
        has_fail = _emit(_check_isaac_version(install)) or has_fail
        has_fail = _emit(_check_isaac_python(install)) or has_fail
        has_fail = _emit(_check_isaac_core_in_isaac(install)) or has_fail

    return has_fail, install


def _emit(result: tuple[str, str]) -> bool:
    """Print a single check result, tally it, and return whether it is a FAIL.

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
    """Check that the Python version meets the minimum requirement.

    Returns:
        Status and message tuple.

    """
    ver = sys.version_info
    ver_str = f"{ver.major}.{ver.minor}.{ver.micro}"
    if (ver.major, ver.minor) >= _MIN_PYTHON:
        return "PASS", f"Python {ver_str}"
    return "FAIL", f"Python {ver_str} < {_MIN_PYTHON[0]}.{_MIN_PYTHON[1]}. Upgrade to Python 3.10+."


def _check_dependency(name: str, min_version: str) -> tuple[str, str]:
    """Check that a Python package is importable and meets its minimum version.

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
    """Extract a module's version string from common attributes.

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
        return version(name)
    except Exception:
        return None


def _check_isaac_install(configured_path: str | None = None) -> tuple[str, str, IsaacInstall | None]:
    """Attempt to locate Isaac Sim.

    Args:
        configured_path: ``sim.isaac_sim_path`` from the user's config, if set. Without this the
            doctor hint told people to set a key that it then ignored.

    Returns:
        Status, message, and the install (or ``None``).

    """
    try:
        install = IsaacInstall.locate(config_path=Path(configured_path) if configured_path else None)
    except IsaacInstallError:
        return (
            "FAIL",
            "Isaac Sim not found. Set sim.isaac_sim_path in config or install Isaac Sim 6.x.",
            None,
        )
    return "PASS", f"Isaac Sim found at {install.root}", install


def _check_isaac_version(install: IsaacInstall) -> tuple[str, str]:
    """Check that the Isaac Sim version is 6.x.

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
    """Check that Isaac's bundled python.sh exists and is executable.

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
    """Check whether isaac_core can actually be imported by Isaac's interpreter.

    This used to guess from the filesystem, looking for an ``isaac_core`` directory or an
    ``.egg-link`` in Isaac's site-packages. A modern editable install (PEP 660) creates neither --
    it writes a ``__editable__*.pth`` and a ``dist-info`` -- so a correctly installed package was
    reported as unconfirmed, which is exactly the sort of warning people learn to ignore. Asking the
    interpreter is both definitive and cheap: it costs about 20 ms because no Kit extension loads.

    Args:
        install: Validated install.

    Returns:
        Status and message tuple.

    """
    python_sh = install.python_path
    if not python_sh.is_file():
        return "FAIL", f"Isaac python.sh missing at {python_sh}; cannot check isaac_core there."
    try:
        result = subprocess.run(
            [str(python_sh), "-c", "import isaac_core; print(isaac_core.__file__)"],
            capture_output=True,
            text=True,
            timeout=_ISAAC_IMPORT_TIMEOUT_S,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return "WARN", f"Could not run {python_sh} to check isaac_core ({type(exc).__name__}). Check the install."
    if result.returncode == 0:
        location = result.stdout.strip().splitlines()[-1] if result.stdout.strip() else "unknown location"
        return "PASS", f"isaac_core imports in Isaac's interpreter ({location})"
    return (
        "FAIL",
        "isaac_core is NOT importable in Isaac's interpreter, so `isaac-core run` cannot start. "
        f"Run: {python_sh} -m pip install -e '.[sim]'",
    )


def _check_ros2() -> tuple[str, str]:
    """Check whether ROS 2 is available (rclpy importable, ROS_DOMAIN_ID set).

    Returns:
        Status and message tuple.

    """

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
    """Check whether our extensions are symlinked into extsUser.

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


def _check_completion() -> tuple[str, str]:
    """Report whether shell completion is installed.

    A feature nobody knows about is not shipped. The generator worked from the start while nothing
    installed the script, so pressing TAB completed filenames and the feature was invisible.

    Returns:
        Status and message tuple.

    """
    from isaac_core.cli.completion import detect_shell, is_installed

    shell = detect_shell()
    if shell is None:
        return "WARN", "Cannot tell which shell you use; install completion with: isaac-core completion --install bash"
    if is_installed(shell):
        return "PASS", f"Shell completion installed for {shell}"
    return "WARN", f"Shell completion not installed. Run: isaac-core completion --install {shell}"


def _check_config() -> tuple[str, str]:
    """Check that the configuration loads without error.

    Returns:
        Status and message tuple.

    """
    try:
        from isaac_core.config import load

        load()
        return "PASS", "Configuration loads successfully"
    except FileNotFoundError as exc:
        return "WARN", f"Config file not found: {exc}. Using defaults."
    except Exception as exc:
        return "FAIL", f"Configuration error: {exc}"


def doctor_checks() -> Sequence[tuple[str, str]]:
    """Run all checks and return a list of (status, message) tuples.

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

    _status, _msg, install = _check_isaac_install(_configured_isaac_path())
    results.append((_status, _msg))

    if install is not None:
        results.append(_check_isaac_version(install))
        results.append(_check_isaac_python(install))
        results.append(_check_isaac_core_in_isaac(install))

    results.append(_check_ros2())

    if install is not None:
        results.append(_check_extensions_linked(install))

    results.append(_check_config())
    results.append(_check_completion())

    return results
