"""Shared fixtures for the system tests.

These launch a real Isaac Sim. They exist because three regressions in this project shipped past a
fully green unit suite -- the swarm RTSP port collision, a render-product node rename, and a viewport
rename that killed a Kit extension -- and none of them were visible without a running renderer.

Design decisions that make running these routine rather than a chore:

* **One simulator per module, shared by the tests in it.** A launch costs 20-40 seconds, so launching
  per test would make the suite unusable. Module scope rather than session scope matters: a session
  fixture is never released, so three simulators stayed alive at once and the GPU starved until the
  swarm's control plane stopped answering. Tests must leave their simulator usable for the next one.
* **Skipped, never failed, when the environment cannot support them.** No Isaac Sim install, or the
  package missing from Isaac's interpreter, is a machine difference and not a defect.
* **Opt in with ``--system``**, or ``ISAAC_CORE_SYSTEM_TESTS=1``. The default suite stays fast.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
from typing import Any, Final

import pytest

# A launch plus stage composition. Every wait here is bounded, so the worst case for the whole
# suite is about ten minutes rather than open-ended: a launch that will not come up should fail
# fast and say so, not hang a run.
LAUNCH_TIMEOUT_S = 90.0

# Frames to advance after a command so it reaches the stage before anything reads it back.
SETTLE_FRAMES = 40

# Longest we wait for a closed simulator's process to disappear before giving up and letting the
# next module try anyway. Isaac ignores SIGTERM and releases the GPU asynchronously.
TEARDOWN_GRACE_S = 30.0

# How many times a fixture retries a launch that never becomes ready.
LAUNCH_ATTEMPTS = 2

# Rounds of waiting for a launched simulator to report a composed, ready stage, at two seconds each.
# Twenty seconds is deliberate. Raising it to two minutes was tried and made things worse rather than
# better: when a two-vehicle stage in a multi-module run does not compose, it does not compose at all,
# so a longer budget converts a quick, informative skip into a very long hang. Failing fast is the
# more useful behaviour.
READINESS_ATTEMPTS = 10

# Module order, heaviest simulator first. Measured: running the two-vehicle module LAST made it
# fail or skip, while the same module first passed every time -- cumulative pressure from earlier
# simulators is what it struggles with, so it gets the clean machine.
MODULE_ORDER: tuple[str, ...] = (
    "test_swarm",
    "test_capture",
    "test_single_vehicle",
)

# Control plane port for the shared simulator. Deliberately not the default, so a system test run
# cannot silently attach to a simulator someone left running.
SYSTEM_TEST_PORT = 8791

# RTSP base ports, one range per fixture. The simulators run concurrently -- session-scoped
# fixtures are not torn down between tests -- so sharing the default 8554 made the second one
# fail to bind and reported it as a product bug.
SINGLE_RTSP_PORT = 8600
SWARM_RTSP_PORT = 8610
GUI_RTSP_PORT = 8620

# UDP pose ports, one per fixture, for the same reason: `set_pose` sends a real packet, and two
# simulators bound to the same port means delivery goes to whichever answers first. The swarm's
# poses were silently landing in the single-vehicle simulator.
SINGLE_UDP_PORT = 34100
SWARM_UDP_PORT = 34110
GUI_UDP_PORT = 34120


def pytest_addoption(parser: pytest.Parser) -> None:
    """Register the flag that turns the system tests on."""
    parser.addoption(
        "--system",
        action="store_true",
        default=False,
        help="Run the system tests, which launch a real Isaac Sim.",
    )


def _requested(config: pytest.Config) -> bool:
    """Report whether the system tests were asked for."""
    return bool(config.getoption("--system")) or os.environ.get("ISAAC_CORE_SYSTEM_TESTS") == "1"


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    """Skip the system tests unless requested, and run their modules heaviest first."""
    if not _requested(config):
        skip = pytest.mark.skip(reason="system tests need --system or ISAAC_CORE_SYSTEM_TESTS=1")
        for item in items:
            if "system" in str(item.fspath):
                item.add_marker(skip)
        return

    def rank(item: pytest.Item) -> int:
        stem = Path(str(item.fspath)).stem
        return MODULE_ORDER.index(stem) if stem in MODULE_ORDER else len(MODULE_ORDER)

    system_items = [item for item in items if "system" in str(item.fspath)]
    if not system_items:
        return
    others = [item for item in items if "system" not in str(item.fspath)]
    system_items.sort(key=rank)
    items[:] = others + system_items


def _running_simulators() -> list[int]:
    """Return the PIDs of Isaac Sim processes other than this one.

    Scans ``/proc`` rather than shelling out to ``pgrep``, because a pattern broad enough to match
    Isaac also matches the very command doing the matching.

    Returns:
        Process ids, empty when none are running.

    """
    mine = {os.getpid(), os.getppid()}
    found: list[int] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) in mine:
            continue
        try:
            argv = (entry / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        if not argv or not argv[0]:
            continue
        joined = b" ".join(argv).decode(errors="replace")
        if "telemetry" in joined:
            continue
        if "isaac_core.sim" in joined or "kit/kit" in joined:
            found.append(int(entry.name))
    return found


def _await_gpu_release() -> None:
    """Block until no simulator is running, so the next module does not contend for the GPU.

    A fixed sleep was not enough: Isaac ignores SIGTERM and releases the GPU asynchronously, and with
    two prior modules the third simulator's control plane stopped answering. Waiting on the actual
    process list is deterministic where a guessed delay is not.
    """
    deadline = time.monotonic() + TEARDOWN_GRACE_S
    while time.monotonic() < deadline:
        if not _running_simulators():
            # A brief pause even once the processes are gone: the driver frees memory just after exit.
            time.sleep(2.0)
            return
        time.sleep(1.0)


def _isaac_available() -> str | None:
    """Return why Isaac Sim cannot be used, or ``None`` when it can."""
    try:
        from isaac_core.install import IsaacInstall
    except ImportError as exc:  # pragma: no cover - environment specific
        return f"isaac_core is not importable: {exc}"
    try:
        install = IsaacInstall.locate()
    except Exception as exc:  # pragma: no cover - environment specific
        return f"no Isaac Sim install found: {exc}"
    if install is None:
        return "no Isaac Sim install found"
    if not Path(install.python_path).exists():
        return f"{install.python_path} does not exist"
    return None


def _await_responsive(session: Any) -> None:
    """Block until a freshly launched simulator answers and reports a composed stage.

    `Sim.launch` returns once the control plane answers, which is earlier than the point at which the
    stage is fully usable -- a two-vehicle stage in particular. Without this the first test in a module
    could time out on a call that would have succeeded moments later, which looked like a product bug.

    Args:
        session: The devkit session.

    Raises:
        pytest.skip.Exception: If the simulator never becomes usable.

    """
    last_state: object = None
    last_error: str | None = None
    for _ in range(READINESS_ATTEMPTS):
        try:
            state = session.state()
            last_state = state
            if state.get("ready") and state.get("stage_composed"):
                session.step(count=STEP_CHUNK_FRAMES)
                return
        except Exception as exc:
            last_error = f"{type(exc).__name__}: {exc}"
        time.sleep(2.0)
    # Swallowing the reason cost hours once: the simulator's own log showed it had composed and was
    # running, so the failure was on this side of the wire and the message said nothing about it.
    pytest.skip(
        "the simulator launched but never reported a composed, ready stage; "
        f"last state={last_state!r}, last error={last_error}"
    )


def _clear_startup_hazards() -> None:
    """Remove the two things that reliably make a launch fail to reach a composed stage.

    The Cesium request cache's write-ahead log stalls startup badly, and size is not the trigger:
    measured, a two-vehicle launch with a 763 MiB log never became ready across three runs, taking
    over ten minutes each to give up, and passed in 24 seconds with the log removed. The same
    behaviour appeared at 23 GiB. Stale ``/tmp/carb.*`` directories left by a crashed Kit make the
    startup segfault more likely. Both are caches: deleting them while no simulator is running costs
    only re-streamed terrain, which no test asserts on.

    """
    cache = Path.home() / ".cache" / "ov"
    for name in ("cesium-request-cache.sqlite-wal", "cesium-request-cache.sqlite-shm"):
        (cache / name).unlink(missing_ok=True)
    for leftover in Path("/tmp").glob("carb.*"):
        if leftover.is_dir():
            shutil.rmtree(leftover, ignore_errors=True)


# Where each launched simulator's output goes. Kept out of pytest's capture on purpose, and kept on
# disk because it is the only record of why a launch failed.
ISAAC_LOG_DIR: Final = Path(tempfile.gettempdir()) / "isaac_core_system_logs"


def _file_logging_launcher(command: list[str]) -> subprocess.Popen[bytes]:
    """Spawn the simulator with its output going to a file rather than to pytest's capture.

    Isaac prints thousands of lines while starting. Inheriting pytest's captured file descriptors
    meant that output had nowhere to drain, and a two-vehicle launch -- which prints roughly twice as
    much -- blocked partway through startup, so the stage never composed. That surfaced as the swarm
    tests skipping with "never reported a composed, ready stage" in a full run while passing on their
    own, and cost about ten minutes per run in timeouts. Running the same two modules with ``-s``
    passed in 61 seconds, which is what identified the capture as the cause rather than the code.

    Args:
        command: The command to run.

    Returns:
        The spawned process.

    """
    ISAAC_LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = ISAAC_LOG_DIR / f"isaac-{os.getpid()}-{time.monotonic_ns()}.log"
    with log_path.open("wb") as handle:
        return subprocess.Popen(command, start_new_session=True, stdout=handle, stderr=subprocess.STDOUT)


def _launch_with_retry(port: int, overrides: dict[str, Any], *, headless: bool) -> Any:
    """Launch a simulator, retrying if it never becomes ready.

    Isaac segfaults during startup on roughly one launch in three -- a Kit issue that reproduces with
    our extensions disabled -- and under GPU contention a launch can simply never answer. Retrying the
    launch is legitimate; the tests themselves never retry an assertion.

    Args:
        port: Control plane port.
        overrides: Dotted config overrides.
        headless: Whether to run without a window.

    Returns:
        A context manager yielding a connected session.

    Raises:
        pytest.skip.Exception: If no launch came up, since that is an environment problem rather than
            a defect in the code under test.

    """
    from isaac_core.devkit import Sim

    last: Exception | None = None
    for attempt in range(1, LAUNCH_ATTEMPTS + 1):
        _clear_startup_hazards()
        try:
            return Sim.launch(
                headless=headless,
                port=port,
                timeout_s=LAUNCH_TIMEOUT_S,
                overrides=overrides,
                launcher=_file_logging_launcher,
            )
        except (TimeoutError, ConnectionError) as exc:
            last = exc
            _await_gpu_release()
            if attempt < LAUNCH_ATTEMPTS:
                continue
    pytest.skip(f"no simulator came up after {LAUNCH_ATTEMPTS} attempts: {last}")
    raise AssertionError  # unreachable, satisfies the type checker


@pytest.fixture(scope="module")
def output_root(tmp_path_factory: pytest.TempPathFactory) -> Iterator[Path]:
    """Return a directory the simulator may write captures into, cleaned up afterwards."""
    path = tmp_path_factory.mktemp("isaac_core_system")
    yield path
    shutil.rmtree(path, ignore_errors=True)


@pytest.fixture(scope="module")
def sim_session(output_root: Path) -> Iterator[Any]:
    """Launch one headless simulator for the whole session and yield a devkit session.

    Args:
        output_root: Directory the simulator confines captures to.

    Yields:
        A connected ``SimSession``.

    """
    reason = _isaac_available()
    if reason is not None:
        pytest.skip(reason)

    overrides: dict[str, Any] = {
        "sim.control_plane.output_root": str(output_root),
        "vehicles.drone_0.pose_source": "udp",
        "vehicles.drone_0.cameras.eo.rtsp_port": SINGLE_RTSP_PORT,
        "vehicles.drone_0.udp_port": SINGLE_UDP_PORT,
    }
    with _launch_with_retry(SYSTEM_TEST_PORT, overrides, headless=True) as session:
        _await_responsive(session)
        yield session
    _await_gpu_release()


@pytest.fixture(scope="module")
def swarm_session(tmp_path_factory: pytest.TempPathFactory) -> Iterator[Any]:
    """Launch a second simulator configured with two vehicles.

    Separate from :func:`sim_session` because the vehicle set is fixed at launch, and a swarm cannot
    be conjured on a running stage.

    Yields:
        A connected ``SimSession`` with vehicles ``lead`` and ``wing``.

    """
    reason = _isaac_available()
    if reason is not None:
        pytest.skip(reason)

    output = tmp_path_factory.mktemp("isaac_core_swarm")
    overrides: dict[str, Any] = {
        "sim.control_plane.output_root": str(output),
        "vehicles.lead.pose_source": "udp",
        "vehicles.wing.pose_source": "udp",
        # Explicit ports: an explicit value is honoured verbatim, so give each its own rather than
        # relying on the base+index derivation which would land on the other fixture's range.
        "vehicles.lead.cameras.eo.rtsp_port": SWARM_RTSP_PORT,
        "vehicles.wing.cameras.eo.rtsp_port": SWARM_RTSP_PORT + 1,
        "vehicles.lead.udp_port": SWARM_UDP_PORT,
        "vehicles.wing.udp_port": SWARM_UDP_PORT + 1,
    }
    with _launch_with_retry(SYSTEM_TEST_PORT + 1, overrides, headless=True) as session:
        _await_responsive(session)
        yield session
    _await_gpu_release()
    shutil.rmtree(output, ignore_errors=True)


@pytest.fixture(scope="module")
def gui_session(tmp_path_factory: pytest.TempPathFactory) -> Iterator[Any]:
    """Launch a simulator **with a GUI**, for the capabilities that need a viewport.

    Frame capture goes through ``capture_viewport_to_file``, and headless has no colour resource to
    read -- Kit reports "Capture of LdrColor was requested, but no valid resource!" and writes nothing.
    So capture cannot be verified headless, and pretending otherwise would report a working feature as
    broken.

    Skipped when there is no display, which is what a CI machine looks like.

    Yields:
        A connected ``SimSession`` backed by a windowed simulator.

    """
    reason = _isaac_available()
    if reason is not None:
        pytest.skip(reason)
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        pytest.skip("frame capture needs a display; no DISPLAY or WAYLAND_DISPLAY is set")

    output = tmp_path_factory.mktemp("isaac_core_gui")
    overrides: dict[str, Any] = {
        "sim.control_plane.output_root": str(output),
        "vehicles.drone_0.pose_source": "udp",
        "vehicles.drone_0.cameras.eo.rtsp_port": GUI_RTSP_PORT,
        "vehicles.drone_0.udp_port": GUI_UDP_PORT,
    }
    with _launch_with_retry(SYSTEM_TEST_PORT + 2, overrides, headless=False) as session:
        _await_responsive(session)
        yield session
    _await_gpu_release()
    shutil.rmtree(output, ignore_errors=True)


# Frames per `step` call. One long call can exceed the control plane's socket timeout on a busy
# GPU -- a 40-frame step in the swarm session timed out -- and a wedged loop is then indistinguishable
# from a slow one. Several short calls keep each RPC bounded and fail informatively.
STEP_CHUNK_FRAMES = 10


def settle(session: Any, frames: int = SETTLE_FRAMES) -> None:
    """Advance the simulation so a command reaches the stage before it is read back.

    Args:
        session: The devkit session.
        frames: How many frames to advance, in bounded chunks.

    """
    remaining = frames
    while remaining > 0:
        chunk = min(STEP_CHUNK_FRAMES, remaining)
        session.step(count=chunk)
        remaining -= chunk


def wait_for_stage_z(
    session: Any,
    expected_z: float,
    *,
    vehicle: str | None = None,
    tolerance_m: float = 1.0,
    attempts: int = 12,
    resend: Callable[[], None] | None = None,
) -> float:
    """Advance the simulation until a vehicle's stage Z reaches a value, and return what it reached.

    A commanded pose travels over UDP, is decoded by an OmniGraph node and written to a prim, so it
    lands some unpredictable number of frames later. Polling until it converges is honest about that;
    a fixed frame count is a guess that passes on a fast machine and fails on a busy one.

    Args:
        session: The devkit session.
        expected_z: The stage Z the pose should produce.
        vehicle: Vehicle to read, or ``None`` for the default.
        tolerance_m: How close counts as arrived.
        attempts: How many settle-and-check rounds to try.
        resend: Called before each round. ``set_pose`` sends a **single** UDP packet, so one sent
            before the receiving node has bound its socket is simply lost and nothing retries --
            a real sender streams at 30 Hz. Resending each round removes that race.

    Returns:
        The last stage Z observed, whether or not it converged.

    """
    observed = float("nan")
    for _ in range(attempts):
        if resend is not None:
            resend()
        settle(session, STEP_CHUNK_FRAMES)
        pose = session.get_pose(vehicle=vehicle) if vehicle else session.get_pose()
        observed = float(pose["translate"][2])
        if abs(observed - expected_z) < tolerance_m:
            return observed
    return observed
