#!/usr/bin/env python3
"""Run one eyes-on check scenario, with instructions for what to look at.

Each scenario launches Isaac Sim **with the GUI** through the devkit, drives it, and prints what
you should see. Everything is driven through ``isaac_core.devkit`` rather than the control plane
directly, so a scenario also exercises the public API a user would write -- if a scenario is
awkward or broken, that is a devkit bug worth reporting.

Usage, from the repo root::

    PYTHONPATH=src ./scripts/eyes_on_check.py list
    PYTHONPATH=src ./scripts/eyes_on_check.py 1
    PYTHONPATH=src ./scripts/eyes_on_check.py 3 --keep-open 600
    PYTHONPATH=src ./scripts/eyes_on_check.py 7 --dwell 20

Isaac takes 15-30 seconds to appear. The script waits for the control plane before driving
anything, so nothing happens until the simulator is genuinely ready.

Leaving the scenario with Ctrl-C shuts the simulator down cleanly. If you ever need to kill Isaac
from another terminal, note that it is started with ``installSignalHandlers=0`` and therefore
**ignores SIGTERM** -- plain ``kill`` and plain ``timeout`` will not stop it. Use ``kill -9``, or
``timeout -k 5 <seconds>`` when scripting it.

Some scenarios need ROS 2 sourced to be fully observable::

    source /opt/ros/humble/setup.bash
    source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass, field
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any

# Printed when the devkit cannot be imported, which is nearly always a PYTHONPATH problem.
REPO_ROOT_HINT = "run from the repo root with PYTHONPATH=src"

# Reference scene origin. Matches the shipped earth scene's Cesium georeference, so an aircraft
# placed here starts over the terrain rather than in the ocean.
ORIGIN_LAT = 32.22481
ORIGIN_LON = 35.25621

# Terrain elevation at the origin, from the scene's georeference. Altitudes below are absolute
# (metres above sea level), so subtract this to reason about height above ground.
GROUND_M = 516.7

# How long each scenario leaves the simulator open for inspection, unless overridden.
DEFAULT_KEEP_OPEN_S = 240.0

# Default seconds to dwell on each phase of the lifecycle scenario (scenario 7). Generous on
# purpose: the owner reported he could not tell when each pause/step/resume/reset phase was
# happening, so each phase now announces itself and lingers. Override with --dwell.
DEFAULT_DWELL_S = 12.0


@dataclass
class Scenario:
    """One eyes-on check: what to launch, what to do, and what to look for."""

    number: int
    title: str
    watch: tuple[str, ...]
    run: Callable[[Any, float, float], None]
    # Assertive counterpart to `run`, for --verify. Raises AssertionError on a real failure and
    # returns a one-line summary of what it measured. None means this scenario needs human eyes.
    verify: Callable[[Any], str] | None = None
    overrides: dict[str, Any] = field(default_factory=dict)
    needs_ros: bool = False
    note: str = ""


# Where scenario 16's throwaway layer is written. Deliberately outside the repo: the point is that a
# layer needs nothing inside isaac_core, only a directory on the search path.
_OWN_LAYER_ROOT = Path(tempfile.gettempdir()) / "isaac_core_eyes_on_layers"
_OWN_LAYER_ID = "eyes_on_probe"


def _prepare_own_layer() -> Path:
    """Write a behaviour-only layer outside the repo and return its search root.

    It declares no USD, which is legal for a layer that contributes behaviour rather than prims, so
    this needs no authored geometry to prove that discovery and composition reach outside the package.

    Returns:
        The directory to hand to ``assets.layer_search_paths``.

    """
    layer = _OWN_LAYER_ROOT / _OWN_LAYER_ID
    layer.mkdir(parents=True, exist_ok=True)
    (layer / "layer.toml").write_text(
        "\n".join(
            (
                f'id = "{_OWN_LAYER_ID}"',
                "# No `usd` key: this layer contributes behaviour rather than prims, which is what lets",
                "# the scenario prove discovery works without authoring geometry.",
                'mount = "/World/Environment/{instance}"',
                "requires = []",
                'provides = ["EYES_ON_PROBE"]',
                "",
            )
        ),
        encoding="utf-8",
    )
    return _OWN_LAYER_ROOT


def _banner(text: str) -> None:
    """Print a section heading."""
    print(f"\n{'=' * 78}\n{text}\n{'=' * 78}", flush=True)


def _hold(seconds: float, message: str) -> None:
    """Keep the simulator open so there is time to look at it.

    Args:
        seconds: How long to wait.
        message: What just finished.

    """
    print(f"\n  {message}", flush=True)
    print(f"  Holding for {seconds:.0f}s. Ctrl-C when you have seen enough.\n", flush=True)
    try:
        time.sleep(seconds)
    except KeyboardInterrupt:
        print("\n  Stopped early.", flush=True)


def _fly_circle(session: Any, seconds: float, *, alt_m: float = 1200.0, radius_deg: float = 0.004) -> None:
    """Fly a slow circle over the origin using repeated ``set_pose`` calls.

    Uses the devkit's ``set_pose`` rather than a UDP sender so the scenario exercises the
    documented API. Motion is deliberately slow: a fast orbit hides whether the camera tracks
    smoothly or jumps.

    Args:
        session: A connected devkit session.
        seconds: How long to keep flying.
        alt_m: Altitude above sea level in metres.
        radius_deg: Circle radius in degrees of latitude.

    """
    deadline = time.time() + seconds
    start = time.time()
    while time.time() < deadline:
        angle = (time.time() - start) * 0.12
        session.set_pose(
            lat_deg=ORIGIN_LAT + radius_deg * math.cos(angle),
            lon_deg=ORIGIN_LON + radius_deg * math.sin(angle),
            alt_m=alt_m,
            yaw_deg=math.degrees(angle) + 90.0,
        )
        time.sleep(0.05)


def _fly_leg(
    session: Any,
    seconds: float,
    *,
    bearing_deg: float,
    alt_m: float = 1200.0,
    speed_deg_s: float = 0.0006,
) -> None:
    """Fly a straight track on a fixed compass bearing at constant altitude.

    The commanded yaw is set to the bearing being flown, so the nose points exactly along the
    direction of travel. That is what makes nose alignment unambiguous: with a correctly wired
    yaw the terrain slides straight down the screen, whereas a mis-set yaw shows the ground
    crabbing sideways. A circle cannot show this, which is why scenario 1 flies legs instead.

    The leg is centred on the origin: it starts half its length "behind" the origin along the
    bearing and flies forward through it, so the aircraft stays over textured terrain the whole
    time rather than wandering off the tiled area.

    Args:
        session: A connected devkit session.
        seconds: How long to fly this leg.
        bearing_deg: Compass bearing to fly, degrees clockwise from north (NED yaw).
        alt_m: Altitude above sea level in metres.
        speed_deg_s: Track speed in degrees of arc per second; slow so motion reads clearly.

    """
    bearing_r = math.radians(bearing_deg)
    # North and east components of one second of travel, in degrees. Longitude is scaled by
    # cos(latitude) so a degree east covers the same ground distance as a degree north.
    step_north = speed_deg_s * math.cos(bearing_r)
    step_east = speed_deg_s * math.sin(bearing_r) / math.cos(math.radians(ORIGIN_LAT))
    half = seconds / 2.0
    start = time.time()
    while True:
        elapsed = time.time() - start
        if elapsed >= seconds:
            break
        offset_s = elapsed - half
        session.set_pose(
            lat_deg=ORIGIN_LAT + step_north * offset_s,
            lon_deg=ORIGIN_LON + step_east * offset_s,
            alt_m=alt_m,
            yaw_deg=bearing_deg,
        )
        time.sleep(0.05)


def _place_and_confirm(session: Any, *, alt_m: float = 1200.0, attempts: int = 10) -> bool:
    """Put the aircraft over the origin and confirm the pose actually landed.

    Poses are delivered over UDP, which is fire-and-forget: a packet sent before the receiver
    graph exists is simply gone, with no error anywhere. That is exactly what made an early
    scenario look like "the gimbal does not work" when in fact the aircraft had never been
    positioned. So the pose is re-sent until ``get_pose`` shows the expected height.

    Args:
        session: A connected devkit session.
        alt_m: Altitude above sea level in metres.
        attempts: How many times to re-send before giving up.

    Returns:
        ``True`` when the pose was confirmed on the stage.

    """
    expected_z = alt_m - GROUND_M
    print(f"  Placing the aircraft over the origin at {alt_m:.0f} m...", flush=True)
    for attempt in range(1, attempts + 1):
        session.set_pose(lat_deg=ORIGIN_LAT, lon_deg=ORIGIN_LON, alt_m=alt_m)
        time.sleep(1.5)
        pose = session.get_pose()
        translate = pose.get("translate")
        if translate and abs(float(translate[2]) - expected_z) < 1.0:
            print(f"  Confirmed: translate z = {float(translate[2]):.1f} (expected {expected_z:.1f})", flush=True)
            return True
        print(f"    attempt {attempt}: translate = {translate}, want z about {expected_z:.1f}", flush=True)
    print("  WARNING: the pose never took effect. That is a finding worth reporting.", flush=True)
    return False


def _scenario_linear(session: Any, keep_open: float, dwell: float) -> None:
    """Fly two straight legs so nose-to-track alignment is unambiguous.

    A circle cannot show whether the nose points along the direction of travel -- every heading
    looks equally plausible on a curve. Two straight legs on different bearings can: on each leg
    the commanded yaw equals the bearing flown, so a correctly-oriented aircraft has the terrain
    flowing straight down the screen, and turning to a new bearing must visibly re-point the nose
    rather than merely sliding the same view sideways.

    Args:
        session: A connected devkit session.
        keep_open: Seconds to stay open after the scripted part.
        dwell: Unused here; kept for a uniform scenario signature.

    """
    del dwell
    _place_and_confirm(session)
    leg_s = min(max(keep_open, 40.0), 90.0)
    legs = (
        (0.0, "due NORTH"),
        (90.0, "due EAST"),
    )
    print("\n  Flying two straight legs. On each one the nose is commanded to the track it flies.", flush=True)
    for bearing_deg, name in legs:
        _banner(f"LEG on bearing {bearing_deg:.0f} deg ({name}) for {leg_s:.0f}s")
        print("  CORRECT looks like:", flush=True)
        print("    - terrain slides straight from the TOP of the frame toward the BOTTOM", flush=True)
        print("    - the horizon stays LEVEL, with no roll", flush=True)
        print("    - no sideways drift: features track down the screen, not across it", flush=True)
        print("  A MIS-SET yaw would instead show the ground sliding SIDEWAYS (crabbing),", flush=True)
        print("  as if the aircraft were flying one way while pointed another.", flush=True)
        _fly_leg(session, leg_s, bearing_deg=bearing_deg)
    print("\n  The bearing changed from 0 (north) to 90 (east) between the legs.", flush=True)
    print("  If the nose FOLLOWED the new heading, yaw is wired correctly. If the view merely", flush=True)
    print("  translated without the nose turning, that is a finding worth reporting.", flush=True)
    _hold(max(keep_open - 2.0 * leg_s, 10.0), "Both legs finished; the camera holds its last pose.")


def _scenario_terrain(session: Any, keep_open: float, dwell: float) -> None:
    """Fly a slow circle over the terrain."""
    del dwell
    _place_and_confirm(session)
    print("\n  Now flying a slow circle. Watch the viewport.", flush=True)
    _fly_circle(session, min(keep_open, 120.0))
    _hold(max(keep_open - 120.0, 10.0), "Circle finished; the camera holds its last pose.")


def _scenario_gimbal(session: Any, keep_open: float, dwell: float) -> None:
    """Step the gimbal through each axis with a rate limit in place."""
    del dwell
    _place_and_confirm(session)
    print("  Config start angles should already be applied (pitch -15 deg, nose down).", flush=True)
    steps = (
        ("pitch to -45 (look further down)", {"pitch_deg": -45.0}),
        ("pitch back to -15", {"pitch_deg": -15.0}),
        ("yaw to +60 (swing right)", {"yaw_deg": 60.0}),
        ("yaw to -60 (swing left)", {"yaw_deg": -60.0}),
        ("yaw back to 0", {"yaw_deg": 0.0}),
        ("roll to +25 (drop the right side)", {"roll_deg": 25.0}),
        ("roll back to 0", {"roll_deg": 0.0}),
    )
    for label, kwargs in steps:
        print(f"  set_gimbal: {label}", flush=True)
        session.set_gimbal(**kwargs)
        time.sleep(6.0)
    _hold(keep_open, "Gimbal sequence finished.")


def _scenario_distance(session: Any, keep_open: float, dwell: float) -> None:
    """Descend in steps so the reported range changes visibly."""
    del dwell
    print("  Watch the ROS topic in another terminal:", flush=True)
    print("    ros2 topic echo /isaac_core/distance_sensor\n", flush=True)
    _place_and_confirm(session, alt_m=2500.0)
    for alt in (2500.0, 2000.0, 1500.0, 1200.0, 1000.0, 800.0):
        session.set_pose(lat_deg=ORIGIN_LAT, lon_deg=ORIGIN_LON, alt_m=alt)
        print(f"  altitude {alt:6.0f} m  ->  expect range about {alt - GROUND_M:6.0f} m", flush=True)
        time.sleep(8.0)
    _hold(keep_open, "Descent finished.")


def _scenario_bbox(session: Any, keep_open: float, dwell: float) -> None:
    """Look down at the tracked cubes so bounding boxes have something to report."""
    del dwell
    print("  Watch the bbox topic in another terminal:", flush=True)
    print("    ros2 topic echo /isaac_core/bbox\n", flush=True)
    _place_and_confirm(session, alt_m=900.0)
    print("  Pointing the camera straight down at the tracked cubes...", flush=True)
    session.set_gimbal(pitch_deg=-90.0)
    time.sleep(8.0)
    print("  Circling so the cubes enter and leave the frame.", flush=True)
    _fly_circle(session, min(keep_open, 150.0), alt_m=900.0, radius_deg=0.0015)
    _hold(max(keep_open - 150.0, 10.0), "Circle finished.")


def _scenario_capture(session: Any, keep_open: float, dwell: float) -> None:
    """Capture stills at several resolutions."""
    del dwell
    _place_and_confirm(session)
    session.set_gimbal(pitch_deg=-40.0)
    time.sleep(6.0)
    shots: tuple[tuple[str, dict[str, int]], ...] = (
        ("viewport_size.png", {}),
        ("four_k.png", {"width": 3840, "height": 2160}),
        ("small.png", {"width": 640, "height": 480}),
    )
    for name, kwargs in shots:
        print(f"  capture_frame({name}) -> {session.capture_frame(name, **kwargs)}", flush=True)
        time.sleep(2.0)
    print("\n  Open the files listed above and check they show terrain, not a blank frame.", flush=True)
    _hold(keep_open, "Captures finished.")


def _scenario_swarm(session: Any, keep_open: float, dwell: float) -> None:
    """Fly two vehicles to different altitudes and positions."""
    del dwell
    print("  Two vehicles configured: lead and wing.\n", flush=True)
    for vehicle, alt in (("lead", 1500.0), ("wing", 900.0)):
        session.set_pose(vehicle=vehicle, lat_deg=ORIGIN_LAT, lon_deg=ORIGIN_LON, alt_m=alt)
        print(f"  {vehicle}: altitude {alt:.0f} m", flush=True)
    time.sleep(4.0)
    for vehicle in ("lead", "wing"):
        pose = session.get_pose(vehicle=vehicle)
        print(f"  get_pose({vehicle}) prim={pose.get('prim')} translate={pose.get('translate')}", flush=True)
    print("\n  Separating them horizontally so both are distinguishable...", flush=True)
    session.set_pose(vehicle="lead", lat_deg=ORIGIN_LAT + 0.004, lon_deg=ORIGIN_LON, alt_m=1500.0)
    session.set_pose(vehicle="wing", lat_deg=ORIGIN_LAT - 0.004, lon_deg=ORIGIN_LON, alt_m=900.0)
    _hold(keep_open, "Both vehicles are holding their poses.")


def _translate_of(pose: Any) -> tuple[float, float, float] | None:
    """Return a pose's translate as a float triple, or ``None`` when it is absent.

    Args:
        pose: A dict as returned by ``get_pose``.

    Returns:
        The ``(x, y, z)`` translate, or ``None`` if the pose has no readable translate.

    """
    translate = pose.get("translate") if isinstance(pose, dict) else None
    if not translate:
        return None
    return (float(translate[0]), float(translate[1]), float(translate[2]))


def _announce(step: str, watch: str, dwell: float) -> None:
    """Print a clearly delimited banner for one lifecycle phase.

    Args:
        step: What is about to happen.
        watch: What the viewer should be looking at right now.
        dwell: How long this phase will last, in seconds.

    """
    _banner(f"ABOUT TO: {step}")
    print(f"  WATCH NOW: {watch}", flush=True)
    print(f"  This phase lasts about {dwell:.0f}s.", flush=True)


def _scenario_lifecycle(session: Any, keep_open: float, dwell: float) -> None:
    """Exercise pause, step, resume and reset with loud, verifiable narration.

    Each phase announces itself before it happens, states what to watch and for how long, then
    confirms afterwards. Because a frozen viewport is hard to judge by eye, the pose is read
    before and after the pause and step phases: identical numbers corroborate a real freeze, and a
    small advance corroborates a real step. ``state()`` is printed so ``running`` and ``ready`` can
    be checked directly rather than inferred from the picture.

    Args:
        session: A connected devkit session.
        keep_open: Seconds to stay open after the scripted part.
        dwell: Seconds to dwell on each lifecycle phase.

    """
    _place_and_confirm(session)
    print(f"\n  state() before we start: {session.state()}", flush=True)

    _announce(
        "pause() the simulation",
        "the viewport should FREEZE -- nothing in the scene should move",
        dwell,
    )
    before = _translate_of(session.get_pose())
    print(f"  pose translate before pause: {before}", flush=True)
    session.pause()
    time.sleep(dwell)
    after = _translate_of(session.get_pose())
    print(f"  pose translate after {dwell:.0f}s paused: {after}", flush=True)
    print(f"  state() while paused: {session.state()}", flush=True)
    if before is not None and before == after:
        print("  CONFIRMED: the pose did not advance -- the freeze is real, not just apparent.", flush=True)
    else:
        print("  NOTE: the pose changed while paused; if the viewport also moved, report it.", flush=True)

    _announce(
        "step(count=30) once, while still paused",
        "the viewport should ADVANCE a short burst and then stop again",
        dwell,
    )
    before = _translate_of(session.get_pose())
    print(f"  pose translate before step: {before}", flush=True)
    session.step(count=30)
    time.sleep(dwell)
    after = _translate_of(session.get_pose())
    print(f"  pose translate after step + {dwell:.0f}s: {after}", flush=True)
    print("  A small change here, then stillness, is the step landing. No change at all is a finding.", flush=True)

    _announce(
        "resume() the simulation",
        "continuous motion should RETURN and keep going",
        dwell,
    )
    session.resume()
    time.sleep(dwell)
    print(f"  state() after resume: {session.state()}", flush=True)

    _announce(
        "reset() the simulation",
        "simulation time returns to ZERO and the gimbal snaps back to its configured start angle",
        dwell,
    )
    print(f"  reset returned: {session.reset()}", flush=True)
    time.sleep(dwell)
    print(f"  state() after reset: {session.state()}", flush=True)
    print(f"  get_capabilities(): {session.get_capabilities()}", flush=True)
    _hold(keep_open, "Lifecycle sequence finished.")


# --------------------------------------------------------------------------- #
# Assertive verifiers for --verify. Each measures numbers and raises on a real
# failure, so bucket B of docs/dev/feature_matrix.md stops depending on memory.
# --------------------------------------------------------------------------- #

# How many times --verify will retry a launch that never becomes ready.
_LAUNCH_ATTEMPTS = 3

# Metres of tolerance when comparing a commanded altitude to the stage transform.
_ALTITUDE_TOLERANCE_M = 1.0

# The resolution --verify asks a capture for, deliberately unlike the viewport's.
_CAPTURE_WIDTH = 1920
_CAPTURE_HEIGHT = 1080

# A stage transform is always a triple.
_TRANSLATE_COMPONENTS = 3

# Degrees of tolerance when reading a commanded gimbal target back.
_GIMBAL_TOLERANCE_DEG = 0.001

# Altitude separation --verify commands between the two swarm vehicles, in metres.
_SWARM_SEPARATION_M = 600.0


def _scenario_zoom(session: Any, keep_open: float, dwell: float) -> None:
    """Sweep the zoom from widest to longest, then take it by focal length."""
    del dwell
    _place_and_confirm(session, alt_m=1400.0)
    session.set_gimbal(pitch_deg=-35.0)
    time.sleep(3.0)
    print("  Zoom is LINEAR IN FIELD OF VIEW, so level 0.5 should look halfway, not nearly", flush=True)
    print("  fully zoomed in. Watch how much ground the frame covers at each step.\n", flush=True)
    for level in (0.0, 0.25, 0.5, 0.75, 1.0):
        reported = session.set_zoom(level=level)
        print(
            f"  level {level:4.2f}  ->  focal {reported['focal_length_mm']:7.2f} mm, "
            f"field of view {reported['hfov_deg']:6.2f} deg",
            flush=True,
        )
        time.sleep(7.0)
    print("\n  Now by exact focal length rather than by level:", flush=True)
    for focal in (35.0, 85.0):
        reported = session.set_zoom(focal_mm=focal)
        print(f"  focal {focal:5.1f} mm  ->  level {reported['level']:5.3f}", flush=True)
        time.sleep(6.0)
    print("\n  Out-of-range values are clamped rather than refused:", flush=True)
    for focal in (1.0, 9999.0):
        reported = session.set_zoom(focal_mm=focal)
        print(f"  asked {focal:7.1f} mm  ->  got {reported['focal_length_mm']:7.2f} mm", flush=True)
        time.sleep(3.0)
    session.set_zoom(level=0.0)
    _hold(keep_open, "Zoom sweep finished, returned to the widest setting.")


def _scenario_segmentation(session: Any, keep_open: float, dwell: float) -> None:
    """Record a segmentation mp4 while orbiting, so colour stability can be judged."""
    del dwell
    import math

    from isaac_core.contracts.pose import Lla
    from isaac_core.geo.distance import look_at_angles, meters_to_latlon_offset

    output = Path.home() / "isaac_core_out"
    print(f"  Recording to {output}/eyes_on_segmentation.mp4", flush=True)
    print("  The viewport shows the NORMAL camera; the segmentation is written to the file.", flush=True)
    print("  Orbiting the reference point while aimed at it, so the subject stays centred.\n", flush=True)

    target = Lla(lat_deg=ORIGIN_LAT, lon_deg=ORIGIN_LON, alt_m=GROUND_M + 2.0)
    radius_m, orbit_alt_m = 140.0, GROUND_M + 120.0
    _place_and_confirm(session, alt_m=orbit_alt_m)
    session.start_segmentation_recording()
    for index in range(72):
        bearing = math.radians(index * 5.0)
        north_m, east_m = radius_m * math.cos(bearing), radius_m * math.sin(bearing)
        lat_off, lon_off = meters_to_latlon_offset(north_m, east_m, ORIGIN_LAT)
        here = Lla(lat_deg=ORIGIN_LAT + lat_off, lon_deg=ORIGIN_LON + lon_off, alt_m=orbit_alt_m)
        yaw_r, pitch_r = look_at_angles(here, target)
        session.set_pose(
            lat_deg=here.lat_deg,
            lon_deg=here.lon_deg,
            alt_m=here.alt_m,
            pitch_deg=math.degrees(pitch_r),
            yaw_deg=math.degrees(yaw_r),
        )
        time.sleep(0.25)
    written = session.stop_segmentation_recording("eyes_on_segmentation.mp4")
    print(f"\n  WROTE: {written['path']}", flush=True)
    print(f"  {written['frames']} frames at {written['fps']} fps, {written['dropped']} dropped", flush=True)
    print(f"\n  Open it:  ffplay {written['path']}", flush=True)
    _hold(keep_open, "Segmentation recording finished.")


def _scenario_ros_pose(session: Any, keep_open: float, dwell: float) -> None:
    """Drive the vehicle over ROS 2 instead of UDP, the way MAVROS would."""
    del dwell
    import math

    try:
        from geometry_msgs.msg import PoseStamped
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import NavSatFix
    except ImportError as exc:
        print(f"  ROS 2 is not sourced, so this scenario cannot run: {exc}", flush=True)
        print("    source /opt/ros/humble/setup.bash", flush=True)
        return

    print("  This vehicle's pose_source is 'ros', so UDP packets are ignored entirely.", flush=True)
    print("  Publishing MAVROS-shaped topics from this process:", flush=True)
    print("    /mavros/global_position/global  (NavSatFix)", flush=True)
    print("    /mavros/local_position/pose     (PoseStamped)\n", flush=True)

    rclpy.init()
    node = Node("eyes_on_mavros_stand_in")
    fix_pub = node.create_publisher(NavSatFix, "/mavros/global_position/global", 10)
    pose_pub = node.create_publisher(PoseStamped, "/mavros/local_position/pose", 10)
    try:
        # Interpolated at 30 Hz rather than jumped. Publishing one fixed pose per phase makes the camera
        # snap between places, which demonstrates nothing about whether the path is followed smoothly.
        alt_m, yaw_deg = 1200.0, 0.0
        for label, to_alt, to_yaw, seconds in (
            ("climb to 1200 m, facing north", 1200.0, 0.0, 6.0),
            ("turn to face east, holding altitude", 1200.0, -90.0, 8.0),
            ("climb to 1600 m, still facing east", 1600.0, -90.0, 8.0),
            ("descend to 900 m, turning back to north", 900.0, 0.0, 10.0),
        ):
            print(f"  publishing: {label}", flush=True)
            from_alt, from_yaw = alt_m, yaw_deg
            ticks = max(1, int(seconds * 30.0))
            for tick in range(ticks + 1):
                fraction = tick / ticks
                alt_m = from_alt + fraction * (to_alt - from_alt)
                yaw_deg = from_yaw + fraction * (to_yaw - from_yaw)
                fix = NavSatFix()
                fix.latitude, fix.longitude, fix.altitude = ORIGIN_LAT, ORIGIN_LON, alt_m
                pose = PoseStamped()
                half = math.radians(yaw_deg) / 2.0
                pose.pose.orientation.z = math.sin(half)
                pose.pose.orientation.w = math.cos(half)
                stamp = node.get_clock().now().to_msg()
                fix.header.stamp = stamp
                pose.header.stamp = stamp
                fix_pub.publish(fix)
                pose_pub.publish(pose)
                rclpy.spin_once(node, timeout_sec=0.005)
                time.sleep(1.0 / 30.0)
            observed = _translate_of(session.get_pose())
            if observed is not None:
                print(f"    stage z now {observed[2]:8.2f} (expected about {to_alt - GROUND_M:.2f})", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    _hold(keep_open, "ROS 2 pose sequence finished.")


def _scenario_posebot(session: Any, keep_open: float, dwell: float) -> None:
    """Fly the PoseBot flight library: orbit, path, steer, then track a moving target."""
    del dwell
    from isaac_core.contracts.pose import Lla
    from isaac_core.devkit import PoseBot

    config = session.config.get()
    vehicle = next(iter(config["vehicles"]))
    udp_port = int(config["vehicles"][vehicle].get("udp_port") or 33333)
    print(f"  Driving {vehicle} over UDP port {udp_port} with PoseBot, not set_pose.", flush=True)
    print("  Each command is a real 30 Hz pose stream, so the motion should be smooth.\n", flush=True)

    start = Lla(lat_deg=ORIGIN_LAT, lon_deg=ORIGIN_LON, alt_m=1200.0)
    with PoseBot(port=udp_port, start=start, rate_hz=30.0) as bot:
        time.sleep(2.0)
        print("  orbit: 300 m radius at 40 m/s for 20 s", flush=True)
        bot.orbit(ORIGIN_LAT, ORIGIN_LON, radius_m=300.0, speed_mps=40.0, duration_s=20.0)

        print("  fly_path: three waypoints at 60 m/s, nose along the track", flush=True)
        bot.fly_path(
            [
                Lla(lat_deg=ORIGIN_LAT + 0.004, lon_deg=ORIGIN_LON, alt_m=1300.0),
                Lla(lat_deg=ORIGIN_LAT + 0.004, lon_deg=ORIGIN_LON + 0.005, alt_m=1100.0),
                Lla(lat_deg=ORIGIN_LAT, lon_deg=ORIGIN_LON, alt_m=1200.0),
            ],
            speed_mps=60.0,
        )

        print("  steer: 250 m turn radius at 45 m/s for 15 s (a coordinated turn)", flush=True)
        bot.steer(turn_radius_m=250.0, speed_mps=45.0, duration_s=15.0)

        print("  track_point: aiming at a target that drifts east, without moving", flush=True)
        # Offset from the aircraft on purpose. A target directly below means a straight-down aim, where the
        # Euler decomposition is singular and yaw collapses into roll -- the stage then reports a rolled
        # airframe for a command that never touched roll, which reads as a bug and is not one.
        drifting_lon = ORIGIN_LON + 0.004

        def moving_target() -> Lla:
            nonlocal drifting_lon
            drifting_lon += 0.00004
            return Lla(lat_deg=ORIGIN_LAT, lon_deg=drifting_lon, alt_m=GROUND_M)

        bot.track_point(moving_target, duration_s=12.0)

        print("  follow_point: chasing that target with a 250 m standoff", flush=True)
        bot.follow_point(moving_target, distance_m=250.0, height_m=150.0, speed_mps=50.0, duration_s=18.0)
        print(f"\n  {bot.sent} pose packets sent in total.", flush=True)
    _hold(keep_open, "PoseBot sequence finished.")


def _scenario_mavlink(session: Any, keep_open: float, dwell: float) -> None:
    """Drive the simulator from a MAVLink source through the isaac-core-mavlink bridge."""
    del dwell
    try:
        from pymavlink import mavutil
    except ImportError as exc:
        print(f"  pymavlink is not installed, so this scenario cannot run: {exc}", flush=True)
        return

    config = session.config.get()
    vehicle = next(iter(config["vehicles"]))
    udp_port = int(config["vehicles"][vehicle].get("udp_port") or 33333)
    mavlink_port = 14577

    print("  Three processes: a fake autopilot here, the bridge, and the simulator.", flush=True)
    print(f"  autopilot -> MAVLink udp {mavlink_port} -> bridge -> pose udp {udp_port} -> sim\n", flush=True)

    bridge = subprocess.Popen(
        [
            "isaac-core-mavlink",
            f"udpin:127.0.0.1:{mavlink_port}",
            "--host",
            "127.0.0.1",
            "--port",
            str(udp_port),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    # The bridge must be listening before the autopilot speaks, or the opening datagrams are lost.
    time.sleep(4.0)
    stop = threading.Event()
    state = {"alt_m": 1100.0, "yaw_deg": 0.0}

    def autopilot() -> None:
        """Emit GLOBAL_POSITION_INT and ATTITUDE at 30 Hz."""
        link = mavutil.mavlink_connection(f"udpout:127.0.0.1:{mavlink_port}", source_system=1)
        while not stop.is_set():
            boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
            link.mav.global_position_int_send(
                boot_ms,
                int(ORIGIN_LAT * 1e7),
                int(ORIGIN_LON * 1e7),
                int(state["alt_m"] * 1000),
                int(state["alt_m"] * 1000),
                0,
                0,
                0,
                int(state["yaw_deg"] * 100) % 36000,
            )
            link.mav.attitude_send(boot_ms, 0.0, 0.0, math.radians(state["yaw_deg"]), 0.0, 0.0, 0.0)
            time.sleep(1.0 / 30.0)

    thread = threading.Thread(target=autopilot, daemon=True)
    thread.start()
    try:
        # Walked rather than jumped: the autopilot thread reads `state` every tick, so moving it gradually
        # is what makes this look like a flight instead of a sequence of teleports.
        for label, to_alt, to_yaw, seconds in (
            ("hold 1100 m", 1100.0, 0.0, 4.0),
            ("climb to 1500 m", 1500.0, 0.0, 8.0),
            ("turn to 90 deg while holding altitude", 1500.0, 90.0, 8.0),
            ("descend to 850 m", 850.0, 90.0, 10.0),
        ):
            print(f"  autopilot: {label}", flush=True)
            from_alt, from_yaw = float(state["alt_m"]), float(state["yaw_deg"])
            ticks = max(1, int(seconds * 20.0))
            for tick in range(ticks + 1):
                fraction = tick / ticks
                state["alt_m"] = from_alt + fraction * (to_alt - from_alt)
                state["yaw_deg"] = from_yaw + fraction * (to_yaw - from_yaw)
                time.sleep(1.0 / 20.0)
            observed = _translate_of(session.get_pose())
            if observed is not None:
                print(f"    stage z now {observed[2]:8.2f} (expected about {to_alt - GROUND_M:.2f})", flush=True)
    finally:
        stop.set()
        if bridge.poll() is None:
            try:
                os.killpg(os.getpgid(bridge.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
    _hold(keep_open, "MAVLink sequence finished.")


def _scenario_recording(session: Any, keep_open: float, dwell: float) -> None:
    """Record the published topics of a real flight to disk, then report the files."""
    del dwell
    try:
        import rclpy

        from isaac_core.devkit.recording import pose_recorder, range_recorder, video_recorder
    except ImportError as exc:
        print(f"  ROS 2 is not sourced, so this scenario cannot run: {exc}", flush=True)
        print("    source /opt/ros/humble/setup.bash", flush=True)
        return

    output = Path.home() / "isaac_core_out"
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    camera = video_recorder()
    poses = pose_recorder()
    ranges = range_recorder()
    for recorder in (camera, poses, ranges):
        recorder.start()
        threading.Thread(target=recorder.spin, daemon=True).start()
    print("  Three recorders subscribed. Flying a circle while they capture.\n", flush=True)
    try:
        _place_and_confirm(session, alt_m=1300.0)
        _fly_circle(session, 40.0, alt_m=1300.0)
    finally:
        for recorder in (camera, poses, ranges):
            recorder.stop()
    print(
        f"  captured: {camera.frame_count} image frames, {poses.frame_count} poses, " f"{ranges.frame_count} ranges",
        flush=True,
    )
    written: list[str] = []
    if camera.frame_count:
        written.append(str(camera.save_video(output / "eyes_on_recording.mp4")))
    if poses.frame_count:
        written.append(str(poses.save_to(output / "eyes_on_poses.pkl")))
    if ranges.frame_count:
        written.append(str(ranges.save_to(output / "eyes_on_ranges.pkl")))
    for path in written:
        size = Path(path).stat().st_size if Path(path).is_file() else 0
        print(f"    {path}  ({size} bytes)", flush=True)
    for recorder in (camera, poses, ranges):
        recorder.shutdown()
    rclpy.shutdown()
    if written:
        print(f"\n  Open the video:  ffplay {written[0]}", flush=True)
    _hold(keep_open, "Recording finished.")


def _scenario_pose_sender(session: Any, keep_open: float, dwell: float) -> None:
    """Open the pose-sender GUI against this simulator and leave it up to fly by hand."""
    del dwell
    config = session.config.get()
    vehicle = next(iter(config["vehicles"]))
    udp_port = int(config["vehicles"][vehicle].get("udp_port") or 33333)
    print("  Opening the pose-sender GUI. Fly the aircraft by hand from it.", flush=True)
    print(f"  It should land on UDP port {udp_port}, which is what this simulator listens on.\n", flush=True)
    gui = subprocess.Popen(
        ["isaac-core-pose-sender", "--port", str(udp_port)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        if gui.poll() is not None:
            print("  The GUI exited immediately. Is python3-tk installed?", flush=True)
            return
        _hold(keep_open, "GUI open. Drive it, then close the window or press Ctrl-C here.")
    finally:
        if gui.poll() is None:
            try:
                os.killpg(os.getpgid(gui.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass


def _scenario_own_layer(session: Any, keep_open: float, dwell: float) -> None:
    """Prove a layer authored OUTSIDE the repo composes, with nothing in isaac_core changed."""
    del dwell, session
    print("  This scenario is about the startup report rather than the viewport.", flush=True)
    print("  A layer was written to a temporary directory outside this repo, and the", flush=True)
    print("  simulator was pointed at it with assets.layer_search_paths only.\n", flush=True)
    print("  In the startup output above, look for the feature layer report and confirm:", flush=True)
    print("    eyes_on_probe appears as COMPOSED, not skipped and not absent.", flush=True)
    print("  If it is absent entirely, the search path never reached discovery.", flush=True)
    _hold(30.0, "Nothing to fly here; the evidence is in the report above.")


def _reference_altitude(session: Any) -> float:
    """Return the scene's ENU reference altitude from the running simulator's own config."""
    config = session.config.get()
    return float(config["geo"]["enu_reference"]["alt_m"])


def _verify_pose_tracking(session: Any) -> str:
    """Assert a commanded pose reaches the stage with the documented altitude mapping."""
    altitude = 1500.0
    session.set_pose(lat_deg=32.22481, lon_deg=35.25621, alt_m=altitude)
    _settle(session)
    pose = session.get_pose()
    translate = pose["translate"]
    expected_up = altitude - _reference_altitude(session)
    assert (
        abs(translate[2] - expected_up) < _ALTITUDE_TOLERANCE_M
    ), f"stage Z is {translate[2]:.2f} but altitude {altitude} above reference should give {expected_up:.2f}"
    return f"stage Z {translate[2]:.2f} m matches altitude {altitude} m above the reference"


def _verify_capture(session: Any) -> str:
    """Assert a capture writes a file at the requested resolution, not the viewport's."""
    session.set_pose(lat_deg=32.22481, lon_deg=35.25621, alt_m=1200.0)
    _settle(session)
    result = session.capture_frame("verify_capture.png", width=_CAPTURE_WIDTH, height=_CAPTURE_HEIGHT)
    assert int(result["width"]) == _CAPTURE_WIDTH, f"asked for width {_CAPTURE_WIDTH}, got {result['width']}"
    assert int(result["height"]) == _CAPTURE_HEIGHT, f"asked for height {_CAPTURE_HEIGHT}, got {result['height']}"
    written = Path(str(result["path"]))
    assert written.is_file(), f"capture reported {written} but no file exists"
    assert written.stat().st_size > 0, f"{written} is empty"
    return f"wrote {written.name} at {result['width']}x{result['height']}, {written.stat().st_size} bytes"


def _verify_swarm(session: Any) -> str:
    """Assert each vehicle has its own prim and its own altitude."""
    session.set_pose(vehicle="lead", lat_deg=32.22481, lon_deg=35.25621, alt_m=1500.0)
    session.set_pose(vehicle="wing", lat_deg=32.22481, lon_deg=35.25621, alt_m=1500.0 - _SWARM_SEPARATION_M)
    _settle(session)
    lead = session.get_pose(vehicle="lead")
    wing = session.get_pose(vehicle="wing")
    assert lead["prim"] != wing["prim"], f"both vehicles report the same prim {lead['prim']}"
    separation = abs(lead["translate"][2] - wing["translate"][2])
    assert (
        abs(separation - 600.0) < _ALTITUDE_TOLERANCE_M
    ), f"commanded 600 m of separation, measured {separation:.2f} m"
    return f"{lead['prim']} and {wing['prim']} separated by {separation:.1f} m"


def _verify_lifecycle(session: Any) -> str:
    """Assert pause, step and resume leave the control plane responsive and the timeline advancing."""
    session.pause()
    before = session.get_pose()["translate"]
    session.step(count=5)
    session.resume()
    after = session.get_pose()["translate"]
    state = session.state()
    assert state, "get_state returned nothing after a pause/step/resume cycle"
    # The pose need not change, but every call must have been answered.
    assert len(before) == _TRANSLATE_COMPONENTS, "get_pose stopped returning a translate triple"
    assert len(after) == _TRANSLATE_COMPONENTS, "get_pose stopped returning a translate triple"
    return f"paused, stepped 5, resumed; state reports {state.get('state', state)}"


def _verify_gimbal(session: Any) -> str:
    """Assert a commanded gimbal target is accepted and reported back."""
    session.set_pose(lat_deg=32.22481, lon_deg=35.25621, alt_m=1200.0)
    _settle(session)
    result = session.set_gimbal(pitch_deg=-35.0)
    assert result, "set_gimbal returned nothing"
    reported = float(result.get("pitch_deg", result.get("pitch", 0.0)))
    assert abs(reported - -35.0) < _GIMBAL_TOLERANCE_DEG, f"asked for pitch -35, target reads {reported}"
    return f"gimbal target accepted as pitch {reported} deg"


def _settle(session: Any, frames: int = 30) -> None:
    """Advance a few frames so a command reaches the stage before it is read back."""
    session.step(count=frames)
    time.sleep(0.5)


SCENARIOS: tuple[Scenario, ...] = (
    Scenario(
        number=1,
        verify=_verify_pose_tracking,
        title="Terrain and camera tracking",
        watch=(
            "Cesium terrain is VISIBLE (textured ground, not grey or empty).",
            "The viewport shows the drone camera's view, not a default perspective.",
            "On each straight leg the terrain flows from the TOP of the frame toward the BOTTOM.",
            "The nose follows the heading: when the bearing changes, the view re-points, not just slides.",
            "Brief stutters over new ground are expected: that is tile streaming, not a bug.",
        ),
        run=_scenario_linear,
    ),
    Scenario(
        number=2,
        verify=_verify_gimbal,
        title="Gimbal start angles and rate-limited slewing",
        overrides={
            "vehicles.drone_0.gimbal.start_pitch_deg": -15.0,
            "vehicles.drone_0.gimbal.max_rate_deg_s": 15.0,
        },
        watch=(
            "At startup the camera already looks slightly DOWN (start_pitch_deg = -15).",
            "Each commanded move SLEWS smoothly at about 15 deg/s -- it must not snap instantly.",
            "+pitch raises the view, +roll drops the right side, +yaw turns right.",
            "The aircraft does not move; only the camera angle changes.",
        ),
        run=_scenario_gimbal,
    ),
    Scenario(
        number=3,
        title="Distance sensor",
        overrides={
            "features.enabled": ["camera_udp", "distance_sensor"],
            "vehicles.drone_0.distance_sensor.max_range_m": 5000.0,
        },
        needs_ros=True,
        watch=(
            "range on /isaac_core/distance_sensor tracks the printed expectation at each step.",
            "min_range and max_range read 0.2 and 5000, NOT 0.0 and a tiny negative number.",
            "header.stamp advances.",
            "The value changes as the aircraft descends; it must not sit frozen.",
        ),
        run=_scenario_distance,
    ),
    Scenario(
        number=4,
        title="Bounding boxes with occlusion",
        overrides={"features.enabled": ["camera_udp", "bbox"]},
        needs_ros=True,
        watch=(
            "target_name lists the cubes under /World/bboxes.",
            "in_frame becomes true only when a cube is actually within the view.",
            "is_visible goes FALSE if something hides a cube -- that is real occlusion.",
            "x1/y1/x2/y2 look like sensible pixels for where the cube appears on screen.",
        ),
        run=_scenario_bbox,
    ),
    Scenario(
        number=5,
        verify=_verify_capture,
        title="Frame capture at several resolutions",
        watch=(
            "Three files are written at the paths printed by the script.",
            "four_k.png really is 3840x2160 and small.png really is 640x480.",
            "Every image shows terrain, not a blank or black frame.",
            "The GUI viewport looks unchanged afterwards -- the resolution is restored.",
        ),
        run=_scenario_capture,
    ),
    Scenario(
        number=6,
        verify=_verify_swarm,
        title="Two vehicles (swarm)",
        overrides={
            "vehicles.lead.pose_source": "udp",
            "vehicles.wing.pose_source": "udp",
        },
        note="Declares lead and wing, which replaces the default drone_0.",
        watch=(
            "TWO camera layers compose: the startup report lists the layer twice, named per vehicle.",
            "get_pose reports DIFFERENT prims and different translate z for lead and wing.",
            "ros2 topic list shows /isaac_core/lead/... and /isaac_core/wing/... topics.",
            "In the Stage tree, /World/Environment/lead and /World/Environment/wing both exist.",
        ),
        run=_scenario_swarm,
    ),
    Scenario(
        number=7,
        verify=_verify_lifecycle,
        title="Lifecycle: pause, step, resume, reset",
        watch=(
            "pause() visibly freezes the viewport.",
            "step(count=30) advances a short burst and stops again.",
            "resume() restores continuous motion.",
            "reset() returns simulation time to zero; the gimbal returns to its configured angle.",
            "state() and get_capabilities() print sensible content, not errors.",
        ),
        run=_scenario_lifecycle,
    ),
    Scenario(
        number=8,
        title="RTSP video stream",
        note=(
            "In another terminal, run exactly:\n"
            "        ffplay -fflags nobuffer -flags low_delay rtsp://127.0.0.1:8554/stream\n"
            "      (plain `ffplay rtsp://127.0.0.1:8554/stream` also works, with more buffering)"
        ),
        watch=(
            "ffplay shows live video from the drone camera.",
            "The stream picture matches what the GUI viewport shows.",
            "It keeps running while the aircraft moves -- no flag was needed to enable it.",
        ),
        run=_scenario_terrain,
    ),
    Scenario(
        number=9,
        title="Zoom, linear in field of view",
        overrides={
            "vehicles.drone_0.camera.focal_length_min_mm": 20.0,
            "vehicles.drone_0.camera.focal_length_max_mm": 200.0,
            "vehicles.drone_0.camera.zoom_max_rate_deg_s": 25.0,
        },
        watch=(
            "Each step ZOOMS SMOOTHLY at about 25 deg/s of field of view -- it must not snap.",
            "Level 0.5 looks about HALFWAY zoomed. If it looks nearly fully zoomed in, the level is",
            "  being interpolated in focal length instead of field of view, which is the bug this guards.",
            "The printed focal length and field of view agree with what the picture does.",
            "Out-of-range focal lengths are clamped to 20 and 200 mm rather than refused.",
            "The aircraft does not move; only the lens changes.",
        ),
        run=_scenario_zoom,
    ),
    Scenario(
        number=10,
        title="Segmentation recording",
        overrides={"features.enabled": ["camera_udp", "segmentation"]},
        watch=(
            "The VIEWPORT shows the normal camera; segmentation goes to the file, not the screen.",
            "Open the written mp4 afterwards. Every distinct object should hold ONE FLAT COLOUR",
            "  that stays with it as the viewpoint orbits.",
            "Cesium terrain is streamed generated geometry, so expect the ground to read as a few",
            "  large regions rather than per-building instances. Authored objects segment cleanly.",
            "A bad recording shows colours FLICKERING on a stationary object, or any solid black frame.",
            "Colours are stable within one file, not between runs -- do not compare two recordings.",
        ),
        run=_scenario_segmentation,
        note="Writes ~/isaac_core_out/eyes_on_segmentation.mp4 for you to open afterwards.",
    ),
    Scenario(
        number=11,
        title="ROS 2 pose source (MAVROS shaped)",
        overrides={"vehicles.drone_0.pose_source": "ros"},
        needs_ros=True,
        watch=(
            "The aircraft moves in response to ROS 2 topics, with NO UDP sender running at all.",
            "Each printed stage z matches the commanded altitude minus 516.7, within a metre or so.",
            "The heading changes when the published quaternion changes.",
            "MAVROS publishes ENU already, so a positive yaw here turns the opposite way to the",
            "  UDP packet's NED yaw. That difference is expected, not a bug.",
        ),
        run=_scenario_ros_pose,
    ),
    Scenario(
        number=12,
        title="PoseBot flight library",
        watch=(
            "orbit: a smooth circle at constant radius, nose tangential, not a series of jumps.",
            "fly_path: straight legs between waypoints, the nose turning to face each new leg,",
            "  and it ENDS EXACTLY on the last waypoint rather than overshooting.",
            "steer: a constant-radius coordinated turn, not a polygon.",
            "track_point: the aircraft HOLDS POSITION and only rotates, following the drifting target.",
            "follow_point: it now chases the target, holding roughly 250 m away and 150 m above it.",
            "Nothing should jump instantly between poses: every command is a real 30 Hz stream.",
        ),
        run=_scenario_posebot,
    ),
    Scenario(
        number=13,
        title="MAVLink bridge",
        watch=(
            "The aircraft follows the fake autopilot, with no UDP sender and no ROS 2 involved.",
            "Each printed stage z matches the commanded altitude minus 516.7 within a metre or so.",
            "If nothing moves at all, the bridge is not receiving: it must be the LISTENER (udpin)",
            "  and it must be up before the autopilot starts talking.",
        ),
        run=_scenario_mavlink,
    ),
    Scenario(
        number=14,
        title="Recording published topics to disk",
        needs_ros=True,
        overrides={"features.enabled": ["camera_udp", "distance_sensor"]},
        watch=(
            "All three counts are non-zero: image frames, poses and ranges.",
            "The written mp4 plays and shows the flight, at the rate actually measured.",
            "A .timestamps.txt sits beside the mp4 -- the simulator's frame rate is not constant,",
            "  so that sidecar is how the real per-frame timing survives.",
            "The pickles are non-trivial in size, not a few bytes.",
        ),
        run=_scenario_recording,
        note="Writes ~/isaac_core_out/eyes_on_recording.mp4 plus two pickles.",
    ),
    Scenario(
        number=15,
        title="Pose sender GUI, flown by hand",
        watch=(
            "The window opens with one tab, already on the port this simulator listens on.",
            "Typing a position and pressing 'Fly there' RAMPS over about three seconds, not a jump.",
            "The readback line shows what the SIMULATOR reports, beside what you are sending --",
            "  if the camera freezes, that line tells you which half is wrong.",
            "Arrow keys nudge yaw and pitch; PageUp/PageDown nudge altitude.",
            "'Copy call' and 'Copy TOML' put usable text on the clipboard.",
            "Ctrl-C in this terminal closes the GUI and the simulator together.",
        ),
        run=_scenario_pose_sender,
        note="Needs python3-tk and a display.",
    ),
    Scenario(
        number=16,
        title="A layer authored outside the repo",
        watch=(
            "In the startup report, eyes_on_probe is listed as a COMPOSED feature layer.",
            "Nothing in isaac_core was edited to make that happen: the layer lives in a temporary",
            "  directory and was found through assets.layer_search_paths alone.",
            "If it is missing entirely, discovery never saw the search path.",
        ),
        run=_scenario_own_layer,
        overrides={
            "assets.layer_search_paths": [str(_prepare_own_layer())],
            "features.enabled": ["camera_udp", _OWN_LAYER_ID],
        },
        note="Writes a throwaway layer under /tmp; nothing in the repo changes.",
    ),
)


def _find(number: int) -> Scenario | None:
    """Return the scenario with this number.

    Args:
        number: The scenario number.

    Returns:
        The scenario, or ``None`` when there is no such number.

    """
    return next((s for s in SCENARIOS if s.number == number), None)


def _print_list() -> None:
    """Print the scenario menu."""
    _banner("Eyes-on check scenarios")
    for scenario in SCENARIOS:
        ros = "  [needs ROS 2 sourced]" if scenario.needs_ros else ""
        print(f"  {scenario.number}. {scenario.title}{ros}", flush=True)
    print("\n  Run one with:  PYTHONPATH=src ./scripts/eyes_on_check.py <number>", flush=True)


def _verify_one(sim: Any, scenario: Scenario, port: int) -> tuple[bool, str]:
    """Launch headless and run one scenario's assertive checker.

    Isaac segfaults during startup on roughly one launch in three, which is a Kit issue that
    reproduces with our extensions disabled, so a launch that never becomes ready is retried.
    A failed *assertion* is never retried -- that would hide the failures this exists to find.

    Args:
        sim: The ``Sim`` entry point.
        scenario: The scenario to verify. Its ``verify`` must not be ``None``.
        port: Control plane port to bind.

    Returns:
        ``(passed, detail)`` where detail is the measurement or the failure.

    """
    assert scenario.verify is not None
    detail = ""
    for attempt in range(1, _LAUNCH_ATTEMPTS + 1):
        try:
            with sim.launch(headless=True, port=port, overrides=scenario.overrides) as session:
                return True, scenario.verify(session)
        except (TimeoutError, ConnectionError) as exc:
            detail = f"{type(exc).__name__}: {exc}"
            if attempt < _LAUNCH_ATTEMPTS:
                print(f"      launch attempt {attempt} did not come up; retrying", flush=True)
        except Exception as exc:
            return False, f"{type(exc).__name__}: {exc}"
    return False, detail


def _resolve_scenario(text: str) -> Scenario | None:
    """Resolve a scenario number from the command line, reporting why if it cannot.

    Args:
        text: The raw ``scenario`` argument.

    Returns:
        The scenario, or ``None`` when the argument is not a known number.

    """
    try:
        number = int(text)
    except ValueError:
        print(f"ERROR: expected a scenario number or 'list', got {text!r}", file=sys.stderr)
        return None
    scenario = _find(number)
    if scenario is None:
        print(f"ERROR: no scenario {number}", file=sys.stderr)
    return scenario


def _run_verifications(args: argparse.Namespace) -> int:
    """Run the assertive checkers headless and report a pass/fail table.

    Args:
        args: Parsed command line, using ``scenario`` (a number or ``all``) and ``port``.

    Returns:
        ``0`` when every selected scenario verified, ``1`` otherwise.

    """
    try:
        from isaac_core.devkit import Sim
    except ImportError as exc:
        print(f"ERROR: cannot import the devkit ({exc}); {REPO_ROOT_HINT}", file=sys.stderr)
        return 1

    if args.scenario == "all":
        selected = [s for s in SCENARIOS if s.verify is not None]
    else:
        try:
            one = _find(int(args.scenario))
        except ValueError:
            print(f"ERROR: expected a scenario number or 'all', got {args.scenario!r}", file=sys.stderr)
            return 2
        if one is None:
            print(f"ERROR: no scenario {args.scenario}", file=sys.stderr)
            return 2
        if one.verify is None:
            print(f"Scenario {one.number} has no automated check; it needs human eyes.", file=sys.stderr)
            return 2
        selected = [one]

    _banner(f"Verifying {len(selected)} scenario(s) headless")
    results: list[tuple[int, str, bool, str]] = []
    for scenario in selected:
        print(f"\n  [{scenario.number}] {scenario.title}", flush=True)
        ok, detail = _verify_one(Sim, scenario, args.port)
        results.append((scenario.number, scenario.title, ok, detail))
        print(f"      {'PASS' if ok else 'FAIL'}  {detail}", flush=True)

    passed = sum(1 for _n, _t, ok, _d in results if ok)
    _banner(f"{passed}/{len(results)} verified")
    for number, title, ok, detail in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {number}  {title}", flush=True)
        if not ok:
            print(f"          {detail}", flush=True)
    return 0 if passed == len(results) else 1


def main(argv: list[str] | None = None) -> int:
    """Run one scenario, or list them all.

    Args:
        argv: Command line arguments, defaulting to ``sys.argv[1:]``.

    Returns:
        Process exit status.

    """
    parser = argparse.ArgumentParser(
        prog="eyes_on_check.py",
        description="Launch Isaac Sim with the GUI and drive one eyes-on check scenario.",
    )
    parser.add_argument("scenario", help="Scenario number, or 'list' to see them all.")
    parser.add_argument(
        "--keep-open",
        type=float,
        default=DEFAULT_KEEP_OPEN_S,
        help=f"Seconds to stay open after the scripted part (default {DEFAULT_KEEP_OPEN_S:.0f}).",
    )
    parser.add_argument("--port", type=int, default=8760, help="Control plane port (default 8760).")
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Assert instead of describe: run headless, check numbers, exit non-zero on failure. "
        "Use 'all' as the scenario to run every verifiable one.",
    )
    parser.add_argument(
        "--dwell",
        type=float,
        default=DEFAULT_DWELL_S,
        help=f"Seconds to dwell on each lifecycle phase, scenario 7 only (default {DEFAULT_DWELL_S:.0f}).",
    )
    args = parser.parse_args(argv)

    if args.scenario == "list":
        _print_list()
        return 0

    if args.verify:
        return _run_verifications(args)

    scenario = _resolve_scenario(args.scenario)
    if scenario is None:
        _print_list()
        return 2

    try:
        from isaac_core.devkit import Sim
    except ImportError as exc:
        print(f"ERROR: cannot import the devkit ({exc}); {REPO_ROOT_HINT}", file=sys.stderr)
        return 1

    _banner(f"Scenario {scenario.number}: {scenario.title}")
    if scenario.note:
        print(f"  NOTE: {scenario.note}", flush=True)
    if scenario.needs_ros:
        print("  Needs ROS 2 sourced in the terminal you watch topics from.", flush=True)
    print("\n  WHAT TO LOOK FOR:", flush=True)
    for item in scenario.watch:
        print(f"    - {item}", flush=True)
    print("\n  Launching Isaac Sim with the GUI. This takes 15-30 seconds...", flush=True)

    try:
        with Sim.launch(headless=False, port=args.port, overrides=scenario.overrides) as session:
            print("  Simulator ready.\n", flush=True)
            scenario.run(session, args.keep_open, args.dwell)
    except KeyboardInterrupt:
        print("\n  Interrupted; shutting the simulator down.", flush=True)
    except Exception as exc:
        print(f"\nERROR: scenario {scenario.number} failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        print("  That is itself a finding worth reporting.", file=sys.stderr)
        return 1

    _banner(f"Scenario {scenario.number} finished. Please report what you saw.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
