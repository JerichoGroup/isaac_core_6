"""
Guard against Python ROS 2 dependencies re-entering the Kit extensions.

Isaac Sim 6 bundles Python 3.12.13. ROS 2 Humble's ``rclpy`` ships a C extension
built for Python 3.10 (``_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so``), and C
extension ABIs are not compatible across Python minor versions. So ``import rclpy``
inside Isaac Sim 6 fails with::

    ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'

and OmniGraph then abandons the whole node file after three retries, taking the
extension's node definitions with it.

This is not a configuration problem and there is no import-order fix. The previous
generation of this tooling could use ``rclpy`` in its nodes only because Isaac Sim
2023.1.1 also bundled Python 3.10.

The resolution: our nodes do pure computation, and **all** ROS 2 publish/subscribe
work is done by Isaac's own ``isaacsim.ros2.bridge`` nodes, which are C++ and
therefore indifferent to the Python ABI. See ``docs/ros2_and_python.md``.

These tests parse source as text -- they never import the node modules, which is
what lets them run with neither Isaac Sim nor ROS 2 present.
"""

from pathlib import Path
import re

REPO_ROOT = Path(__file__).resolve().parents[3]
EXTENSIONS_ROOT = REPO_ROOT / "extensions"

# Modules that only exist in a ROS 2 environment built for the system Python, and so
# can never be imported from inside Isaac Sim 6's bundled 3.12 interpreter.
FORBIDDEN_ROOTS = (
    "rclpy",
    "sensor_msgs",
    "geometry_msgs",
    "geographic_msgs",
    "std_msgs",
    "builtin_interfaces",
    "rosidl_runtime_py",
    "cv_bridge",
)

EXTENSION_PY_FILES = sorted(EXTENSIONS_ROOT.rglob("*.py"))


def _import_roots(source: str) -> set[str]:
    """Return the top-level package names imported by a module's source text."""
    roots: set[str] = set()
    for line in source.splitlines():
        stripped = line.strip()
        match = re.match(r"^(?:import|from)\s+([A-Za-z_][A-Za-z0-9_.]*)", stripped)
        if match:
            roots.add(match.group(1).split(".")[0])
    return roots


def test_extension_python_files_were_discovered() -> None:
    # Guard against the glob matching nothing and every test below passing vacuously.
    assert EXTENSION_PY_FILES, f"no .py files found under {EXTENSIONS_ROOT}"


def test_no_extension_module_imports_a_python_ros2_package() -> None:
    offenders: list[str] = []
    for path in EXTENSION_PY_FILES:
        roots = _import_roots(path.read_text(encoding="utf-8"))
        for forbidden in FORBIDDEN_ROOTS:
            if forbidden in roots:
                offenders.append(f"{path.relative_to(REPO_ROOT)} imports {forbidden}")

    assert not offenders, (
        "Kit extension modules must not import Python ROS 2 packages -- Humble's "
        "rclpy is a Python 3.10 C extension and Isaac Sim 6 runs Python 3.12, so the "
        "import fails at extension load and OmniGraph abandons the node file.\n"
        "Use Isaac's C++ isaacsim.ros2.bridge nodes (ROS2Publisher / ROS2Subscriber) "
        "wired in the graph instead.\n  " + "\n  ".join(offenders)
    )


def test_no_extension_module_calls_rclpy_shutdown() -> None:
    # Retained from the old repo's defect #3: several nodes each calling
    # rclpy.shutdown() in release() tore the shared context out from under the
    # others. Now doubly impossible, but the guard costs nothing.
    offenders = [
        str(path.relative_to(REPO_ROOT))
        for path in EXTENSION_PY_FILES
        if "rclpy.shutdown" in path.read_text(encoding="utf-8")
    ]
    assert not offenders, f"rclpy.shutdown() must never appear in a node: {offenders}"


def test_surviving_nodes_are_the_pure_compute_ones() -> None:
    # The node set shrank deliberately: everything that was a thin wrapper around a
    # ROS publisher/subscriber is now Isaac's own C++ node, leaving only computation
    # that genuinely belongs to us.
    node_files = {path.stem for path in EXTENSIONS_ROOT.rglob("*/nodes/*.py")}
    assert node_files == {
        "OgnGlobalPositionToLocalPosition",
        "OgnQuaternionToEuler",
        "OgnEulerToQuaternion",
        "OgnSecondsToRosStamp",
        "OgnUdpToGlobalPosition",
        "OgnTemplate",
    }, f"unexpected node set: {sorted(node_files)}"
