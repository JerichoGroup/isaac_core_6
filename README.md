# isaac_core_6

Team infrastructure for NVIDIA Isaac Sim 6. A plug-and-play sandbox that drives a camera over real Cesium 3D Tiles terrain from LLA + orientation supplied over UDP or ROS 2/MAVROS. Replaces `isaac_core_2023` with a cleaner architecture, stronger typing, full test coverage, and no forking required to add features.

The simulator works end to end: `isaac-core run` launches Isaac Sim 6, composes the stage
from layer manifests, a UDP pose packet moves the camera, and both ROS topics publish —
`/isaac_core/global_pose` (`geographic_msgs/msg/GeoPoseStamped`, ~100 Hz) and
`/isaac_core/image_rgb` (`sensor_msgs/msg/Image`), both carrying a real advancing
timestamp. Verified numerically against the geodetic math and with `ros2 topic echo`.

---

## Requirements

- **Isaac Sim 6.0.1** (tested on 6.0.1-rc.7) at a known path (e.g. `/home/ofer/isaacsim`)
- **Python 3.10+** (system) — the kernel, CLI and devkit run here
- **ROS 2 Humble** (optional) — only needed for the ROS pose source and topic publishing

### Two hard environment facts

1. **Isaac Sim 6 bundles Python 3.12** while ROS 2 Humble ships C extensions built for 3.10. `rclpy` cannot be imported inside Isaac — attempting it crashes with a `ModuleNotFoundError` on the pybind11 ABI mismatch. All ROS 2 publish/subscribe is handled by Isaac's C++ `isaacsim.ros2.bridge` nodes instead. See [docs/ros2_and_python.md](docs/ros2_and_python.md) for the full explanation.

2. **`$ISAACSIM_PATH` is commonly stale** (e.g. still pointing at a 2023.1.1 install). The tooling validates rather than trusts it — `IsaacInstall` probes known paths and checks the VERSION file before accepting a candidate.

---

## Installation

```bash
./scripts/setup.sh [--isaac-path /path/to/isaacsim]
```

This runs four steps:

1. `pip install --user -r requirements.txt` — runtime deps into system Python
2. `pip install --user -e .` — the `isaac-core` package in editable mode
3. `$ISAAC_PATH/python.sh -m pip install -e .` — the same package into Isaac's bundled Python 3.12
4. `scripts/link_extensions.sh` — symlinks the OmniGraph extensions into `extsUser`

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

It is one pip distribution — `isaac-core` — importable on both 3.10 and 3.12. Isaac-only imports (`omni`, `carb`, `pxr`) are confined to `isaac_core.sim`; ROS-only imports (`rclpy`) are confined to `isaac_core.devkit.recording` and lazy-loaded.

---

## Quick start

### 1. Launch the simulator

```bash
isaac-core run
```

With the default config this opens the `earth` scene, composes the `camera_udp` layer, and starts the control plane on `127.0.0.1:8760`. Add `--set sim.headless true` for no GUI.

### 2. Fly a trajectory

In another terminal:

```bash
PYTHONPATH=src ./scripts/send_test_pose.py hold
PYTHONPATH=src ./scripts/send_test_pose.py orbit --radius-m 800 --duration-s 60
PYTHONPATH=src ./scripts/send_test_pose.py path --speed-mps 50
```

Defaults match the scene's Cesium georeference (32.22°N, 35.26°E, 516.7 m), so the aircraft starts over terrain.

### 3. Query the running sim

```bash
isaac-core config dump
isaac-core config explain sim.headless
```

---

## Scripting with the devkit

```python
from isaac_core.devkit import Sim

# Connect to a sim that is already running — even on another machine.
# No repo path, no filesystem knowledge needed.
with Sim.attach(host="192.168.1.50", port=8760) as session:
    state = session.vehicles()
    print(state)

    session.pause()
    session.config.patch(sim={"headless": True})
    session.resume()

    session.capture_frame("snapshot.png")
```

`Sim.attach(...)` returns a `SimSession` using only the JSON-RPC control plane — the same type that `Sim.launch(...)` returns, so scripts are portable between local and remote.

---

## Debug tools

Two terminal-launched helpers, replacing the old `debugger/` scripts:

```bash
isaac-core-inspect                      # print state, capabilities, live pose and config
isaac-core-inspect --poll 0.5           # watch the pose change while a sender runs
isaac-core-pose-sender                  # tkinter GUI for driving the camera by hand
isaac-core-pose-sender --check          # validate config and exit, no window
```

`isaac-core-inspect` reads the **live prim transform** out of the running stage, which is
the only way to confirm from outside the process that pose input is actually reaching the
camera. It is how the numbers in the quick start were verified:

```
    #          Translate (x, y, z)               Quaternion (w, x, y, z)
    1  T=(     0.000,      0.000,    983.300)  Q=(0.6830, -0.1830, 0.1830, 0.6830)
```

The GUI needs `python3-tk` (`sudo apt install python3-tk`). Both tools import without it;
only opening the window requires it, and the error says exactly what to install.

All GUI state lives in a `PoseSenderController` with no tkinter dependency, so the
behaviour is unit-tested while the widgets stay a thin view.

---

## Configuration

One file controls everything: [`config/default.toml`](config/default.toml). It is the complete surface — every value the simulation runs on appears there with inline documentation.

### Layered precedence (later wins)

```
package defaults → --config <file> / $ISAAC_CORE_CONFIG → env vars → --set CLI flags → runtime patches
```

Environment variables use double-underscore nesting: `ISAAC_CORE__CAMERAS__EO__FOV_DEG=90`.

### Inspect what resolved

```bash
isaac-core config dump          # full resolved config as TOML
isaac-core config explain <key> # which source won a particular value
```

### Layer manifests vs config

Layer manifests (`layer.toml`) declare **wiring** — which config key maps to which prim attribute. They never hold values. Your config holds the values, and the compositor resolves the bindings at launch time.

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
├── contracts/     # types, enums, ports, topics, packet spec — zero deps
├── config/        # pydantic v2 schema + layered TOML loader
├── geo/           # LLA/ECEF/ENU, NED↔ENU, SLERP, rotations — pyproj + transforms3d
├── protocol/      # 51-byte UDP pose codec + error hierarchy
├── vehicle/       # kinematics, trajectories, motion limits — pure generators
├── control/       # JSON-RPC server + client over a local socket
├── cli/           # `isaac-core` entry point (run, doctor, config)
├── install/       # Isaac Sim path resolution and validation
├── devkit/        # user-facing Sim.attach/launch API + transport + recording
├── sim/           # stage composition, runtime, configurator — ONLY package importing omni/pxr
├── sidecar/       # out-of-process services (RTP streaming) — PyGObject/GStreamer
└── assets/        # shipped scenes + layer manifests
```

### Why the kernel is pure

Everything above `sim` — contracts, config, geo, protocol, vehicle, control — imports **nothing from Isaac Sim, ROS 2, or GStreamer**. This is not an accident. It means:

- **980 tests pass with no GPU, no Isaac Sim, no ROS 2.** CI is possible on any runner.
- Geodesy, codecs, trajectories and config are testable in isolation with sub-second feedback.
- The Isaac-dependent surface is thin and behind interfaces.

### Layering enforcement

`import-linter` checks that the dependency graph never violates the layering:

```
contracts ← config ← geo ← protocol ← vehicle        (pure kernel)
                                  ↖ devkit           (+rclpy, lazy)
                                  ↖ sim              (+omni/carb/pxr)
                                  ↖ sidecar          (+gi)
```

The kernel must not import `sim`, `devkit` or `sidecar`. `sim` must not import `devkit`. This is verified on every `lint-imports` run and cannot rot silently.

### Extensions

Two Kit extensions live in `extensions/`, symlinked into `extsUser` by `scripts/link_extensions.sh`:

- `isaac_core_ogn.math` — `GlobalPositionToLocalPosition`, `QuaternionToEuler`,
  `EulerToQuaternion`, `SecondsToRosStamp`
- `isaac_core_ogn.position` — `UdpToGlobalPosition`

All are thin adapters delegating to `isaac_core.geo` and `isaac_core.protocol`. No ROS, no network I/O, no state management beyond what OmniGraph requires.

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

- **D213 (second-line summaries)**: multi-line docstrings put the summary on the second line, not the first:
  ```python
  """
  Return the geodetic pose as a tuple.

  Body paragraph.
  """
  ```
- **Flat test functions**: test classes trip D101/D102. Use module-level functions with `-> None`.
- **Annotated parametrize args**: `ANN001` applies to `@pytest.mark.parametrize` parameters.
- **No attribute docstrings**: `check-docstring-first` rejects them. Use `#` comments above assignments.
- **pathlib only**: `PTH` forbids `os.path`. Use `pathlib.Path`.
- **Line length 120**, double quotes, 4-space indent.

---

## What does not work yet

- **Intermittent segfault on startup.** Roughly one launch in three dies with exit code
  139 just after `simulation running`, inside Kit's `update_app()`. Not caused by our
  extensions — it reproduces with both disabled. Relaunching usually works. Stale
  `/tmp/carb.*` directories from previous crashes make it worse; clearing them with no
  Isaac running helps. Tracked as task 1 in
  [docs/usd_build_sheet.md](docs/usd_build_sheet.md).
- **Sensor layers** (distance sensor, bbox, frame capture) are not yet built.
- **Monotonic frame id** cannot reach the image topic with the current node API. Now that
  `header.stamp` is real and advancing, it covers most of that need.

`isaacsim.ros2.bridge` itself is fine and is enabled by default. An earlier segfault
blamed on it turned out to be a **stale generated OmniGraph database** left behind by a
deleted node; `scripts/link_extensions.sh` now clears those. If you ever see an
unexplained crash on stage open after renaming or removing an OGN node, clear
`~/.cache/ov/ogn_generated/*/isaac_core_ogn.*`.

---

## Further documentation

- [docs/first_run.md](docs/first_run.md) — step-by-step guide for the first end-to-end flight
- [docs/usd_build_sheet.md](docs/usd_build_sheet.md) — instructions for authoring USD in the GUI
- [docs/ros2_and_python.md](docs/ros2_and_python.md) — why rclpy cannot run inside Isaac Sim 6
- [KIRO.md](KIRO.md) — engineering log, architecture decisions (D1–D19), and working constraints

---

## License

[Apache License, Version 2.0](LICENSE)
