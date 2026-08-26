# isaac_core_6

Team infrastructure for NVIDIA Isaac Sim 6. A plug-and-play sandbox that drives a camera over real Cesium 3D Tiles terrain from LLA + orientation supplied over UDP or ROS 2/MAVROS. Replaces `isaac_core_2023` with a cleaner architecture, stronger typing, full test coverage, and no forking required to add features.

The simulator works end to end: `isaac-core run` launches Isaac Sim 6, composes the stage from layer manifests, a UDP pose packet moves the camera, and both ROS topics publish with real advancing timestamps. 1103 tests pass with no GPU, no Isaac Sim and no ROS 2 installed.

---

## Requirements

- Isaac Sim 6.0.1 (tested on 6.0.1-rc.7) at a known path (e.g. `/home/ofer/isaacsim`)
- Python 3.10+ (system) -- the kernel, CLI and devkit run here
- ROS 2 Humble (optional) -- only needed for ROS pose input and topic publishing

### Two hard environment facts

1. Isaac Sim 6 bundles Python 3.12 while ROS 2 Humble ships C extensions built for 3.10. `rclpy` cannot be imported inside Isaac -- attempting it crashes with a `ModuleNotFoundError` on the pybind11 ABI mismatch. All ROS 2 publish/subscribe is handled by Isaac's C++ `isaacsim.ros2.bridge` nodes instead. See [docs/ros2_and_python.md](docs/ros2_and_python.md) for the full explanation.

2. `$ISAACSIM_PATH` is commonly stale (e.g. still pointing at a 2023.1.1 install). The tooling validates rather than trusts it -- `IsaacInstall` probes known paths and checks the VERSION file before accepting a candidate.

---

## Installation

```bash
./scripts/setup.sh [--isaac-path /path/to/isaacsim]
```

This runs four steps:

1. `pip install --user -r requirements.txt` -- runtime deps into system Python
2. `pip install --user -e .` -- the `isaac-core` package in editable mode
3. `$ISAAC_PATH/python.sh -m pip install -e .` -- the same package into Isaac's bundled Python 3.12
4. `scripts/link_extensions.sh` -- symlinks the OmniGraph extensions into `extsUser`

Then verify:

```bash
isaac-core doctor
```

### Three interpreters, one package

| Interpreter | What runs there | How to install |
|---|---|---|
| System Python 3.10 | CLI, devkit, sidecar, tests | `pip install --user -e .` |
| Isaac's Python 3.12 | `isaac_core.sim`, `isaac_core.geo`, `isaac_core.protocol` (inside the simulator process) | `$ISAAC_PATH/python.sh -m pip install -e .` |
| System Python 3.10 with ROS sourced | `rclpy`-based recording and the `ros2` CLI | same system install, just `source /opt/ros/humble/setup.bash` first |

It is one pip distribution -- `isaac-core` -- importable on both 3.10 and 3.12. Isaac-only imports (`omni`, `carb`, `pxr`) are confined to `isaac_core.sim`; ROS-only imports (`rclpy`) are confined to `isaac_core.devkit.recording` and lazy-loaded.

---

## First flight, start to finish

This walks through launching the sim, flying a trajectory, and verifying the output. The UDP path needs no ROS 2, so it works everywhere.

### 1. Launch the simulator

```bash
isaac-core run
```

With the default config this opens the `earth` scene, composes the `camera_udp` layer, and starts the control plane on `127.0.0.1:8760`. Add `--set sim.headless true` for no GUI.

### 2. Send a pose

In a second terminal:

```bash
PYTHONPATH=src ./scripts/send_test_pose.py hold
```

This holds the camera at the Cesium georeference origin (32.22481N, 35.25621E, 1000 m). The sender runs at 30 Hz until you press Ctrl-C.

### 3. Fly a trajectory

```bash
PYTHONPATH=src ./scripts/send_test_pose.py orbit --radius-m 800 --duration-s 60
PYTHONPATH=src ./scripts/send_test_pose.py path --speed-mps 50
```

Defaults match the scene's Cesium georeference, so the aircraft starts over terrain.

### 4. Watch the live pose from outside

In a third terminal:

```bash
isaac-core-inspect --poll 0.5
```

Expected output (numbers depend on what you sent):

```
    #          Translate (x, y, z)               Quaternion (w, x, y, z)
    1  T=(     0.000,      0.000,    983.300)  Q=(0.6830, -0.1830, 0.1830, 0.6830)
    2  T=(     0.000,      0.000,    983.300)  Q=(0.6830, -0.1830, 0.1830, 0.6830)
```

The translate z value is altitude minus the ENU reference altitude: 1500 - 516.7 = 983.3.

### 5. Verify ROS topics

With ROS 2 sourced (this runs on system Python 3.10, not inside Isaac):

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep isaac_core
ros2 topic echo /isaac_core/global_pose --once
```

Expected fields:

```
header:
  stamp:
    sec: 13
    nanosec: 333333333
pose:
  position:
    latitude: 32.3
    longitude: 35.25621
    altitude: 1500.0
  orientation:
    x: ...
    y: ...
    z: ...
    w: ...
```

Published topics:

| Topic | Type | Rate |
|---|---|---|
| `/isaac_core/global_pose` | `geographic_msgs/msg/GeoPoseStamped` | ~100 Hz |
| `/isaac_core/image_rgb` | `sensor_msgs/msg/Image` | ~115 Hz |

---

## Coordinate conventions

This is the thing newcomers get wrong. Read it once.

### Frames

- Input (on the wire): LLA position + roll/pitch/yaw in NED.
- Internally (in the stage): ENU. Isaac Sim and Cesium both use ENU.
- Body axes: +X nose, +Y left wing, +Z up.

### NED to ENU conversion

The wire carries NED angles. Inside Isaac, `ned_to_enu` converts them:

```
roll_enu  =  roll_ned
pitch_enu = -pitch_ned
yaw_enu   = -yaw_ned + pi/2   (normalised to [-pi, pi])
```

Roll passes through, pitch flips sign, and yaw flips sign and rotates 90 degrees to turn
a compass heading into an ENU bearing.

This deliberately does **not** match the previous generation, which swapped roll and
pitch (`roll_enu = pitch_ned`). That swap made the two axes trade places at the camera:
a pitch input banked the image and a roll input tilted the nose. With the mapping above,
`+pitch` raises the nose, `+roll` drops the right wing, and `+yaw` turns right.
`tests/unit/geo/test_camera_axes.py` asserts that behaviour directly rather than the
formula, so it cannot silently regress again.

### The ENU reference point

`config/default.toml` declares the local ENU origin:

```toml
[geo]
enu_reference = { lat_deg = 32.22481, lon_deg = 35.25621, alt_m = 516.7 }
```

A pose at that exact lat/lon appears at stage origin (0, 0, 0). The Z translate is always `altitude - alt_m`. Example: altitude 1500, reference 516.7, translate Z = 983.3.

---

## UDP packet format

51 bytes, little-endian. Byte-for-byte compatible with the previous generation, so existing senders and scripts remain valid.

| Offset | Size | Field | Type | Notes |
|---|---|---|---|---|
| 0 | 1 | header[0] | uint8 | `0xAC` |
| 1 | 1 | header[1] | uint8 | `0xDC` |
| 2 | 8 | latitude | float64 | degrees |
| 10 | 8 | longitude | float64 | degrees |
| 18 | 8 | altitude | float64 | metres, sea level = 0 |
| 26 | 8 | roll | float64 | radians, NED |
| 34 | 8 | pitch | float64 | radians, NED |
| 42 | 8 | yaw | float64 | radians, NED |
| 50 | 1 | checksum | uint8 | XOR of bytes [2, 50) |

Angles are RADIANS on the wire. The GUI debug tools display degrees and convert internally. This discrepancy tripped people in the previous generation; every angle in this codebase now carries its unit in the name (`roll_r` vs `roll_deg`).

On any malformed packet (wrong size, bad header, checksum mismatch), the receiver holds the last good pose rather than dropping to zero. This freeze-on-bad-data behaviour is deliberate.

Default UDP port: 33333. One port per vehicle; with multiple vehicles, ports are allocated as `base + index`.

The spec lives in `src/isaac_core/contracts/packet.py`; the codec in `src/isaac_core/protocol/`.

---

## Scripting with the devkit

```python
from isaac_core.devkit import Sim

# Connect to a running sim -- even on another machine.
# No repo path, no filesystem knowledge needed.
with Sim.attach(host="192.168.1.50", port=8760) as session:
    state = session.vehicles()
    print(state)

    # Patch configuration at runtime
    session.config.patch(sim={"headless": True})

    # Lifecycle control
    session.pause()
    session.resume()
    session.step(count=10)

    # Feature management
    session.features.enable("distance_sensor")
    session.features.disable("distance_sensor")

    # Query capabilities
    caps = session.get_capabilities()
    print(caps)  # {'enabled': ['camera_udp'], 'skipped': []}
```

`Sim.attach(...)` returns a `SimSession` using only the JSON-RPC control plane, so scripts
are portable between a local and a remote simulator.

To start one from the script instead, use `Sim.launch(...)`. It resolves the Isaac Sim
install, writes the fully resolved config to a temp file, spawns
`python.sh -m isaac_core.sim`, and waits for the control plane to answer:

```python
from isaac_core.devkit import Sim

# Owns the process: leaving the block stops the simulator.
with Sim.launch(headless=True, scene="earth") as session:
    print(session.get_capabilities())
    session.step(count=10)
```

The difference matters: a session from `launch` terminates the simulator when it closes,
while one from `attach` leaves it running because it belongs to whoever started it.

### Sending poses programmatically

```python
from isaac_core.contracts.frames import Frame
from isaac_core.contracts.pose import GeodeticPose, Lla, Rpy
from isaac_core.devkit.transport import UdpPoseTransport, pace
from isaac_core.vehicle import OrbitTrajectory

trajectory = OrbitTrajectory(
    center_lat_deg=32.22481,
    center_lon_deg=35.25621,
    radius_m=800,
    height_m=1000,
    speed_mps=30,
    orbit_duration_s=60,
)

transport = UdpPoseTransport(host="127.0.0.1", port=33333)
sent = pace(trajectory.poses(rate_hz=30.0), transport, rate_hz=30.0)
print(f"sent {sent} packets")
```

---

## Debug tools

Two terminal-launched helpers:

```bash
isaac-core-inspect                      # print state, capabilities, live pose and config
isaac-core-inspect --poll 0.5           # watch the pose change while a sender runs
isaac-core-pose-sender                  # tkinter GUI for driving the camera by hand
isaac-core-pose-sender --check          # validate config and exit, no window
```

`isaac-core-inspect` reads the live prim transform out of the running stage through the control plane, which is the only way to confirm from outside the process that pose input is actually reaching the camera.

The GUI needs `python3-tk` (`sudo apt install python3-tk`). Both tools import without it; only opening the window requires it, and the error says exactly what to install.

---

## Configuration

One file controls everything: [`config/default.toml`](config/default.toml). It is the complete surface -- every value the simulation runs on appears there with inline documentation.

### Layered precedence (later wins)

```
package defaults -> --config <file> / $ISAAC_CORE_CONFIG -> env vars -> --set CLI flags -> runtime patches
```

Environment variables use double-underscore nesting: `ISAAC_CORE__CAMERAS__EO__FOV_DEG=90`.

### Inspect what resolved

```bash
isaac-core config dump          # full resolved config as TOML
isaac-core config explain <key> # which source won a particular value
```

### Layer manifests vs config

Layer manifests (`layer.toml`) declare wiring -- which config key maps to which prim attribute. They never hold values. Your config holds the values, and the compositor resolves the bindings at launch time.

---

## Adding a feature layer without forking

A feature layer is a directory containing a `layer.toml` manifest and (optionally) a `.usda` file:

```
my_layers/thermal_cam/
├── layer.toml
└── thermal_cam.usda
```

Drop it into any directory listed in `layer_search_paths` in your config:

```toml
[assets]
layer_search_paths = ["/home/you/my_layers"]

[features]
enabled = ["thermal_cam"]
```

No code change to `isaac_core` is required. The manifest declares what the layer needs from the stage (`requires`), what it provides (`provides`), and how config flows into prim attributes (`[[bindings]]`).

For layers that need Python logic, declare an entry point under `[project.entry-points."isaac_core.layers"]` in your own package.

---

## Architecture

```
src/isaac_core/
├── contracts/     # types, enums, ports, topics, packet spec -- zero deps
├── config/        # pydantic v2 schema + layered TOML loader
├── geo/           # LLA/ECEF/ENU, NED<->ENU, SLERP, rotations -- pyproj + transforms3d
├── protocol/      # 51-byte UDP pose codec + error hierarchy
├── vehicle/       # kinematics, trajectories, motion limits -- pure generators
├── control/       # JSON-RPC server + client over a local socket
├── cli/           # `isaac-core` entry point (run, doctor, config)
├── install/       # Isaac Sim path resolution and validation
├── devkit/        # user-facing Sim.attach/launch API + transport + recording
├── sim/           # stage composition, runtime, configurator -- ONLY package importing omni/pxr
├── sidecar/       # out-of-process services (RTP streaming) -- PyGObject/GStreamer
└── assets/        # shipped scenes + layer manifests
```

### Why the kernel is pure

Everything above `sim` -- contracts, config, geo, protocol, vehicle, control -- imports nothing from Isaac Sim, ROS 2, or GStreamer. This is not an accident. It means:

- 1103 tests pass with no GPU, no Isaac Sim, no ROS 2. CI runs on any runner.
- Geodesy, codecs, trajectories and config are testable in isolation with sub-second feedback.
- The Isaac-dependent surface is thin and behind interfaces.

### Layering enforcement

`import-linter` checks that the dependency graph never violates the layering:

```
contracts <- config <- geo <- protocol <- vehicle        (pure kernel)
                                  \ devkit             (+rclpy, lazy)
                                  \ sim                (+omni/carb/pxr)
                                  \ sidecar            (+gi)
```

The kernel must not import `sim`, `devkit` or `sidecar`. `sim` must not import `devkit`. This is verified by 23 layering contracts on every `lint-imports` run and cannot rot silently.

---

## Development

### Running tests

```bash
python3 -m pytest -q
```

### Running pre-commit hooks

Files are untracked, so `--all-files` silently skips them. Always pass `--files`:

```bash
pre-commit run ruff        --files src/isaac_core/geo/enu.py
pre-commit run ruff-format --files src/isaac_core/geo/enu.py
pre-commit run mypy        --files src/isaac_core/geo/enu.py
```

### Import-linter

```bash
PYTHONPATH=src lint-imports
```

### System-wide install convention

There is no venv. Tools are installed system-wide with `pip install --user`. Pinned versions in `requirements-dev.txt` match `.pre-commit-config.yaml` exactly, so terminal `ruff`/`mypy` gives the same answer as the commit gate.

### Non-obvious lint rules

- D213 (second-line summaries): multi-line docstrings put the summary on the second line, not the first:
  ```python
  """
  Return the geodetic pose as a tuple.

  Body paragraph.
  """
  ```
- Flat test functions: test classes trip D101/D102. Use module-level functions with `-> None`.
- Annotated parametrize args: `ANN001` applies to `@pytest.mark.parametrize` parameters.
- No attribute docstrings: `check-docstring-first` rejects them. Use `#` comments above assignments.
- pathlib only: `PTH` forbids `os.path`. Use `pathlib.Path`.
- Line length 120, double quotes, 4-space indent.

---

## Troubleshooting

### Blank screen, no terrain, and the camera seems not to respond

Three separate things, all with the same symptom, and all fixed by the shipped defaults:

- **No terrain.** The scene's terrain is Cesium 3D Tiles, which needs the
  `cesium.omniverse` extension. It is installed by the GUI's extension manager into
  `~/.local/share/ov/data/exts/v2`, which Isaac's python experience does **not** search on
  its own. `sim.extension_search_paths` adds that folder and `sim.extensions` enables the
  extension. Without both, a `CesiumTilesetPrim` loads as perfectly valid USD that draws
  nothing, with no error printed.
- **Camera appears frozen.** The main viewport keeps Kit's default perspective camera
  unless told otherwise, so the aircraft camera can be tracking your UDP poses correctly
  while the window shows a static view. `sim.viewport_camera` points the viewport at the
  vehicle camera. Confirm the pose really is arriving with `isaac-core-inspect --poll 0.5`,
  which reads the live prim transform rather than the picture.
- **Nothing moving at all.** `omni.graph.action.OnTick` does not fire unless the timeline
  is playing. The runtime presses play automatically, but if you stopped it in the GUI,
  press play again.

If you have an older config file that predates these settings, either delete it and start
from `config/default.toml` or copy the three keys across.


### Poses seem to be ignored, or snap back to something you did not send

Only one sender can meaningfully own a UDP port. If `isaac-core-pose-sender` is open, or an
earlier `send_test_pose.py` is still running in another terminal, it keeps transmitting at
its own rate and whichever packet arrives last wins. A short burst from a script will simply
lose to a GUI sending at 30 Hz, which looks exactly like the pipeline ignoring you.

Check for other senders before concluding anything:

```bash
pgrep -af 'send_test_pose|pose_sender' || echo "no senders running"
```

Related and by design: when all senders stop, the receiver **holds the last good pose**
rather than snapping to zero. A frozen camera can therefore mean "nothing is sending", not
"something is broken".

### No ROS topics appearing

DDS discovery takes a few seconds after the sim starts. Wait 5-10 seconds before assuming a publisher is broken. Verify the graph is computing with `isaac-core-inspect` first.

### Node not appearing in the GUI after adding an OGN node

Clear the stale generated OmniGraph databases:

```bash
rm -rf ~/.cache/ov/ogn_generated/*/isaac_core_ogn.*
```

Then restart Isaac Sim. Isaac caches generated DBs for nodes that no longer exist; this is also what `scripts/link_extensions.sh` does automatically.

### `OnTick` not firing (all outputs zero)

The `omni.graph.action.OnTick` node does not fire unless the timeline is playing. In the GUI, press Play. In headless mode, the runtime starts playing automatically.

### Missing `python3-tk`

The pose sender GUI requires tkinter. Install it:

```bash
sudo apt install python3-tk
```

Both debug tools import without it. Only opening the window requires it, and the error message says exactly what to install.

### Stale `$ISAACSIM_PATH`

`isaac-core doctor` will find the correct Isaac Sim install even if `$ISAACSIM_PATH` points to an old 2023.1.1 directory. If it cannot find it, pass `--isaac-path` explicitly to `setup.sh`.

### Startup segfault (roughly one in three launches)

Stale `/tmp/carb.*` directories from previous crashes make this worse. Clear them with no Isaac running:

```bash
rm -rf /tmp/carb.*
```

Then relaunch. The segfault is inside Kit's `update_app()` and is not caused by our extensions -- it reproduces with both disabled.

### Cesium cache growing to hundreds of GB

Long sessions grow `~/.cache/ov/cesium-request-cache.sqlite-wal` until Isaac fails to start. Delete it, or set `cesium.delete_cache_on_launch = true` in your config.

---

## What does not work yet

- Sensor layers (distance sensor, bounding-box publishing, satellite imagery) are not yet built.
- RTP/GStreamer streaming exists as a sidecar skeleton only.
- `capture_frame` raises `NotImplementedError`.
- Monotonic frame id cannot reach the image topic with the current node API; `header.stamp` covers most of that need.

---

## Further documentation

- [docs/first_run.md](docs/first_run.md) -- step-by-step guide for the first end-to-end flight
- [docs/usd_build_sheet.md](docs/usd_build_sheet.md) -- instructions for authoring USD in the GUI
- [docs/ros2_and_python.md](docs/ros2_and_python.md) -- why rclpy cannot run inside Isaac Sim 6
- [KIRO.md](KIRO.md) -- engineering log, architecture decisions (D1-D19), and working constraints

---

## License

[Apache License, Version 2.0](LICENSE)
