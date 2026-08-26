# KIRO.md — isaac_core_6 working memory

Persistent context for building `isaac_core_6`. Not user documentation; this is my
engineering notebook. Read this first at the start of any session.

- **Target**: `isaac_core_6` — a plug & play sandbox environment for the team, on Isaac Sim 6.
- **Base / reference**: `/home/ofer/clones/isaac_core_2023` (read-only, never modify).
- **Mandate**: not a port. Better design, meets the pre-commit standards natively,
  up to date with Isaac Sim 6, more features, much stronger drone support.

---

## 1. Current repo state (isaac_core_6)

Template-only. Nothing project-specific exists yet.

```
LICENSE  README.md  pyproject.toml  .pre-commit-config.yaml  .gitlint  .gitignore
src/dingo_project/{__init__.py,example.py}     <- placeholder, to be replaced
tests/{__init__.py,test_example.py}            <- placeholder
```

### Standards we must satisfy (from `pyproject.toml` / `.pre-commit-config.yaml`)

- Python `>=3.10`, ruff `0.8.2`, mypy `1.14.1`, pytest `8.3.4`, `target-version = py310`.
- ruff lint select: `E W F I N ANN Q ERA PLR A ASYNC PTH FIX TD D`.
  Ignored: `E501`, `D203`, `D202`, `D212`, `PLR0913`.
  - `ANN` ⇒ **every** function needs full annotations (params + return).
  - `D` ⇒ **every** module/class/function needs a docstring.
  - `PTH` ⇒ **no `os.path`**, use `pathlib`. The 2023 repo uses `os.path` everywhere; all
    of that must be rewritten.
  - `ERA` ⇒ no commented-out code. 2023 repo has plenty (e.g. `run_sim.sh` dead flag vars,
    `udp_utils.py` commented alt function name).
  - `A` ⇒ no shadowing builtins.
  - `I` + `force-sort-within-sections = true`, `combine-as-imports = true`.
    The 2023 repo's hand-grouped `# ==== imports ====` blocks are unsorted; ruff will
    reformat them. Don't fight it.
- mypy: `disallow_untyped_defs`, `disallow_incomplete_defs`, `check_untyped_defs`,
  `warn_unused_ignores`, `no_implicit_optional`, `strict_equality`, `warn_return_any`.
  `files = ["src", "tests"]`, `mypy_path = src`.
- Package discovery: `[tool.setuptools.packages.find] where = ["src"]` — every dir under
  `src/` with an `__init__.py` becomes its own top-level package.
- pytest: `pythonpath = ["src"]`, `testpaths = ["tests"]`.
- per-file ignores already set for `__init__.py` (D104/D100) and `tests/**`
  (PLR2004, D103, ANN201).

### ⚠ The big tooling problem to solve early

`omni`, `carb`, `pxr`, `rclpy`, `isaacsim.*` are **not installable into a normal venv**.
They only exist inside Isaac Sim's bundled Python and a sourced ROS 2 environment.
mypy and ruff run in the pre-commit venv, which has none of them. Options (decide with Ofer):

1. Keep all Isaac/ROS-touching code in a directory excluded from mypy via the **top-level
   `exclude:` in `.pre-commit-config.yaml`** (the only exclude that works for the mypy hook —
   `[tool.mypy] exclude` is ignored when pre-commit passes explicit filenames; the template
   README says this explicitly).
2. Add per-module `[[tool.mypy.overrides]] ignore_missing_imports = true` for `omni.*`,
   `carb.*`, `pxr.*`, `rclpy.*`, `isaacsim.*`. Narrow and honest; **preferred**.
3. Ship minimal local stub packages (`stubs/omni/__init__.pyi`, …) + `mypy_path`.
   Best type safety, most upkeep. Possible middle ground for the handful of APIs we use.

Design consequence: **keep the Isaac-dependent surface thin and behind interfaces**, so the
bulk of the codebase (geodesy, path planning, protocol codecs, config, CLI) is pure Python,
fully typed, fully unit-testable without Isaac Sim. This is the single biggest structural
improvement over 2023, where sim logic and Isaac API calls were fused everywhere.

---

## 2. What isaac_core_2023 actually is

> "A tool aimed for simulating camera image view from a real 3dTiles scanning of the relevant
> location from a gps and orientation inputs. This can be used to simulate areal unmaned
> vehicles." — its README

Concretely: **a synthetic aerial-camera rig flying over real photogrammetry terrain**.
You feed it LLA + roll/pitch/yaw over ROS 2 or UDP; it moves a camera prim inside an
Isaac Sim stage whose ground is Cesium 3D Tiles streamed from an internal tile server;
it publishes back the camera image, global pose, a laser range, and 2D bounding boxes.

Built on Isaac Sim **2023.1.1** + ROS 2 **Humble** + **Cesium for Omniverse**.
Scale: ~7,200 lines tracked (960 of which are the README), ~50 real source files.
The 1,178-file count is dominated by two **vendored Cesium extension builds**
(`docker/simulation_docker/cesium_exts_2023/cesium.omniverse-0.24.0+105.1` and `-0.25.0+106.5`)
committed into git, including `.so` binaries and `.whl` files.

Git history: single `main` branch, PR-based (`dev` → `main`), ~33 PRs. Last work was
`udp_bot`. Authors credited in extension.toml: "Mr_Blum, Sinimini and Binchilling".

### Repository map

| Path | Role |
|---|---|
| `simulation/main_sim.py` | entry point; sets `LAUNCH_CONFIG` env, builds `Simulation`, forks `lib_manager.py` |
| `simulation/sim_app.py` | `Simulation` class — the whole sim lifecycle (353 lines) |
| `simulation/sim_utils.py` | argparse, `OPTIONAL_USDS`, `OPTIONAL_SIM_LIBS`, launch config |
| `simulation/consts.py` | all tunables (camera, gimbal, topics, ports, Cesium URL) |
| `simulation/omniverse_utils.py` | thin USD/stage helpers |
| `simulation/lib_manager.py` | out-of-process registry+runner for "sim libs" |
| `simulation/libraries/` | `sim_lib.py` (ABC), `ros_image_to_rtp_lib.py` (GStreamer RTP) |
| `simulation/script_nodes/` | `bbox_node.py`, `sat_node.py`, `sensor_node.py` — OmniGraph ScriptNodes |
| `simulation/ros2_interfaces/` | custom msgs: `Bbox`, `FrameBboxes`, `Gimbal`, `SATOutput` |
| `simulation/dev_kit/` | `isaac_core_dev_kit` pip package — the external automation API |
| `extensions/omni.sim.{math,position,sensors}` | custom OmniGraph node extensions |
| `extensions/template/omni.sim.template` | scaffold for new nodes |
| `usd/` | `cameras/{ros,udp}_camera.usda`, `sensors/{distance_sensor,bbox_publisher,SAT}.usda`, `maps/earth/earth.usda` |
| `debugger/` | tkinter GUIs `ros-sender` / `udp-sender` for manual pose injection |
| `docker/` | two-stage image build (base → simulation) with warm-start `docker commit` |
| `tools/packman/`, `link_app.sh` | stock Omniverse boilerplate for VS Code autocomplete |

---

## 3. Architecture & runtime data flow

### Composition model: USD reference injection driven by CLI flags

This is the core idea, and it's genuinely good — keep it.

`sim_utils.OPTIONAL_USDS` maps a flag to `(usd_path, prim_path, prim_name)`:

```python
"com_udp": ("usd/cameras/udp_camera.usda", "/Environment/udp_camera", "main_camera_01")
"distance_sensor": ("usd/sensors/distance_sensor.usda", "/Environment/distance_sensor", "distance_sensor")
"bbox_publisher": ("usd/sensors/bbox_publisher.usda", "/Environment/bbox_publisher", "bbox_publisher")
"sat": ("usd/sensors/SAT.usda", "/Environment/SAT", "SAT")
```

`Simulation._add_external_usds` then does, per selected flag,
`DefinePrim(prim_path)` + `GetReferences().AddReference(usd_path)`.
So each feature is a **self-contained USD layer carrying its own OmniGraph**, composed onto
the base map stage at launch. Enable/disable = add/skip a reference. Very plug & play.

`Simulation.__init__` order (matters):
`_resolve_camera_key` → `_enable_extensions` → `_configure_settings` → `open_usd_stage`
→ get stage → `_add_external_usds` → `_set_viewport` → `_update_laser_sensor`
→ `_set_cesium_tilesets_url` → `_update_script_node_paths` → `_configure_camera`
→ `_configure_extensions_ros2`.

Then `run_simulation()`: `SimulationContext(stage_units_in_meters=1.0)`,
`initialize_physics()`, `play()`, and a bare `while kit.is_running() and is_playing():
simulation_context.step(render=True)` loop.

### Pose pipeline (the heart of the system)

```
UDP :33333  ──► OgnSimUDPToGlobalPosition ──┐
  (or)                                      │  global_position [lat,lon,alt]
ROS2 NavSatFix + PoseStamped                │  global_orientation [r,p,y] rad ENU
      ──► OgnSimROS2ToGlobalPosition ───────┤
                                            ▼
/isaac_core/gimbal ─► OgnSimROS2Gimbal ─► offset_roll/pitch/yaw (deg)
                                            │
                                            ▼
                        OgnSimGlobalPositionToLocalPosition
                        (pyproj EPSG:4979→4978, then ECEF→ENU rotation)
                                            │
                    ┌───────────────────────┼────────────────────────┐
                    ▼                       ▼                        ▼
        local_position [x,y,z]   local_orientation quat   global_position/orientation
                    │                       │                        │
     WritePrimAttribute            WritePrimAttribute       OgnSimROS2GlobalPosePublisher
      xformOp:translate              xformOp:orient          → /isaac_core/global_pose
              └──────────► /Root/Xform (camera parent) ◄──────────┘
```

All driven by `OnPlaybackTick` in the graph `UDPOdomSync` (or `ROS2OdomSync`).
A parallel graph `ROS2CameraImageExport` handles: `onPhysicsStep` → `createViewport` →
`setViewPortResolution` → `getRenderProduct` → `setCamera` → `cameraHelperInfo`/`cameraHelperDepth`
→ `ros2_image_publisher` (our custom node).

**ENU reference point** is a node input, default `[32.22481, 35.25621, 516.7]`. The camera moves
in a local ENU tangent plane anchored there; Cesium georeferences the tiles to the same anchor.

### The UDP wire protocol — 51 bytes, exactly

Producer: `dev_kit/udp/base_udp_sender.py::_build_packet`. Consumer:
`OgnSimUDPToGlobalPosition::PacketGetter.get_cur_data`. Also reimplemented (duplicated) in
`debugger/udp_sender.py`.

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 0 | header1 | uint8 | `0xAC` |
| 1 | header2 | uint8 | `0xDC` |
| 2–9 | latitude | float64 LE | degrees |
| 10–17 | longitude | float64 LE | degrees |
| 18–25 | altitude | float64 LE | metres, sea level = 0 |
| 26–33 | roll | float64 LE | **radians**, NED |
| 34–41 | pitch | float64 LE | **radians**, NED |
| 42–49 | yaw | float64 LE | **radians**, NED |
| 50 | checksum | uint8 | XOR of bytes 2..49 |

struct format `"<6d"`, default port `33333`, socket binds `0.0.0.0`, non-blocking,
sender may set `SO_BROADCAST`. On any error (size / header / checksum / unpack) the receiver
returns `_last_good_packet` — it holds the last valid pose rather than dropping to zero.
That freeze-on-bad-data behaviour is deliberate and worth preserving.

### Angle conventions — read this twice

- **Wire / user-facing convention: NED, intrinsic XYZ Euler** (`transforms3d` axes `'rxyz'`).
  +roll = right wing down, +pitch = nose up, +yaw = nose right.
- **Internally Isaac Sim + Cesium use ENU.** Conversion lives in
  `OgnSimUDPToGlobalPosition.convert_ned_to_enu`:
  ```python
  roll_enu  =  pitch_ned
  pitch_enu =  roll_ned
  yaw_enu   = -yaw_ned + pi/2      # then normalised to [-pi, pi]
  ```
- MAVROS already publishes ENU, so the ROS 2 path does **no** NED→ENU conversion —
  only `quat2euler(..., axes='rxyz')`.
- Gimbal offsets are applied as a quaternion post-multiply:
  `qmult(q_drone, q_offset)`, offsets given in degrees.
- **Known Isaac Sim quirk, documented in the code**: Isaac presents quaternions as
  `(w,x,y,z)` in the GUI but internally mixes component order, so the node writes
  `local_orientation = [qx, qy, qz, qw]`. Verify whether this still holds in Isaac Sim 6 —
  if fixed upstream, blindly copying this will silently break orientation.

### ROS 2 interface contract

Inputs (subscribed):

| Topic | Type |
|---|---|
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` |
| `/isaac_core/gimbal` | `isaac_ros2_messages/Gimbal` |
| `/isaac_core/sat` | `isaac_ros2_messages/SATOutput` |

Outputs (published):

| Topic | Type | Note |
|---|---|---|
| `/isaac_core/global_pose` | `geographic_msgs/GeoPoseStamped` | **abuses the quaternion**: x=roll, y=pitch, z=yaw (deg), w=1.0 unused |
| `/isaac_core/distance_sensor` | `sensor_msgs/Range` | |
| `/isaac_core/raw_rgb` → `/isaac_core/image_rgb` | `sensor_msgs/Image` | Replicator writer → rate-limiting republisher; `header.frame_id` = monotonic frame counter as a string |
| `/isaac_core/bbox` | `isaac_ros2_messages/FrameBboxes` | |

Custom messages: `Gimbal{float32 roll,pitch,yaw}` (deg),
`SATOutput{string output_path}`,
`Bbox{string target_name; bool in_frame,is_visible; int32 x1,y1,x2,y2; float32 lat,lon,alt,
roll,pitch,yaw,distance_x,distance_y,distance_z}`,
`FrameBboxes{std_msgs/Header header; Bbox[] bboxes}`.

Everything uses `qos_profile_sensor_data` / BEST_EFFORT + KEEP_LAST(10), and
`use_sim_time = True`.

---

## 4. Feature inventory (what to carry forward)

### Simulation flags
`--com-ros` | `--com-udp` (mutually exclusive, required) · `--usd-path` · `--headless` ·
`--distance-sensor` · `--bbox-publisher` · `--sat` · `--image-rtp`

### Custom OmniGraph nodes

| Extension | Node | Purpose |
|---|---|---|
| `omni.sim.position` | `OgnSimUDPToGlobalPosition` | 51-byte UDP → LLA + RPY (ENU rad) |
| `omni.sim.position` | `OgnSimROS2ToGlobalPosition` | NavSatFix + PoseStamped → LLA + RPY |
| `omni.sim.math` | `OgnSimGlobalPositionToLocalPosition` | LLA→ECEF→ENU + gimbal offset compose |
| `omni.sim.sensors` | `OgnSimROS2GlobalPosePublisher` | GeoPoseStamped out, rate-limited |
| `omni.sim.sensors` | `OgnSimROS2RangePublisher` | `sensor_msgs/Range` out |
| `omni.sim.sensors` | `OgnSimROS2ImagePublisher` | Replicator RGB writer + rate-limited republisher |
| `omni.sim.sensors` | `OgnSimROS2Gimbal` | `/isaac_core/gimbal` → roll/pitch/yaw, falls back to `start_*` before first msg |
| `omni.sim.template` | `OgnSimTemplate` | scaffold |

Node code pattern (consistent, and worth keeping as a house style):
`internal_state()` static factory · `compute(db)` · `release(node)` using
`<Node>Database.per_node_internal_state(node)` · lazy resource creation inside `compute`
(subscriber/publisher created on first tick) · `MultiThreadedExecutor` spun on a daemon thread ·
`carb.log_*` with a `SIM | <ABBREV> |` prefix.

### ScriptNodes (`setup`/`compute`/`cleanup` against `db.internal_state`)

- **`bbox_node.py`** — iterates children of `/bboxes`, pulls
  `sd.sensors.get_bounding_box_2d_tight/loose` from the active viewport, reports
  `in_frame` (in tight ∪ loose) vs `is_visible` (in tight), pixel box from *loose*,
  geolocation from `cesium:anchor:{latitude,longitude,height}`, object Euler from its
  `xformOp:orient`, and camera→object ENU delta via `XFormPrim.get_world_pose()`.
  Publishes async via `asyncio.ensure_future` + manual rate gate.
- **`sat_node.py`** — subscribes `/isaac_core/sat`, on a new path does
  `viewport.schedule_capture(FileCapture(path))`, awaits result. Deduplicates by
  `last_captured_path`.
- **`sensor_node.py`** — laser rangefinder. Takes camera world pose, builds
  `rotation_matrix @ (0,0,-1)`, submits `omni.kit.raycast.query` ray, clamps hit to
  `[min_range, max_range]`, returns `max_range` on invalid.

### `isaac_core_dev_kit` — the external automation API (the best idea in the repo)

Lets other projects drive the sim **without forking it**. Keep this concept and make it
first-class in `isaac_core_6`.

- `isaac_manager/` — `HostIsaacManager` and `DockerIsaacManager`, both **context managers**.
  Build a CLI from kwargs via `FLAG_MAP = {f: f"--{f.replace('_','-')}"}`, launch, then
  tail stdout/`docker logs` for the readiness sentinel `"rclpy loaded"` with a spinner and
  a 300 s timeout, then `sleep(5)` for luck. Host variant uses `os.setsid` + `killpg`
  (SIGTERM→SIGKILL) to reap the whole process group. Docker variant reads the compose
  template with `yaml.safe_load`, injects `services.<first>.command`, writes a
  `tempfile.mkstemp` compose file next to the original, `docker compose up -d`,
  and deletes it on exit.
- `core_capture/` — `BaseCapture(ABC)` + `VideoCapture` (cv_bridge→`cv2.VideoWriter` mp4v),
  `PoseCapture`, `DistanceCapture`, `BboxCapture` (pickle dicts keyed by frame index).
  API: `spin()` / `start_capture()` / `stop_capture()` / `save_data_to(path)` / `shutdown()`.
- `udp/` — `BaseUDPSender(ABC)` with `get_next_point(step)` as the single extension point,
  a `perf_counter`-based fixed-rate loop (`next_time += dt`, no drift), and blocking or
  threaded `run()`. Concrete: `OnePointSender`, `OrbitSender` (circle from radius/speed/
  duration, yaw from finite difference), `PathSender` (LLA waypoints, cumulative-distance
  arc-length interpolation at constant speed), `UdpBot`.
- `UdpBot` — the drone-behaviour layer, and the direct ancestor of the drone features Ofer
  wants. Maintains a world rotation matrix; SLERP (with lerp fallback above
  `DOT_THRESHOLD = 0.9995`) between orientations. Methods:
  `move_to_point`, `move_forward_backward`, `move_right_left`, `move_up_down`,
  `turn_roll/pitch/yaw`, `turn_to_point`, `steer(turn_radius_m, speed_ms, duration_s)`.
- `dev_utils.py` — `delete_cesium_cache()`, `safe_rclpy_init/shutdown()`,
  `set_gimbal_angle()`, `save_current_frame_to()`.

### Sim libs (out-of-process side cars)

`main_sim.py` deliberately launches `lib_manager.py` under **`/usr/bin/python3`, not Isaac's
python**, passing the lib set as a JSON argv string, because these libs need packages Isaac's
bundled interpreter can't have. `SIM_LIB_REGISTRY` maps a key to a `SimLibBase` subclass;
each runs `start()` on its own daemon thread.

Only implementation: `ImageRTPStreamer` — subscribes `/isaac_core/image_rgb`, pushes frames
into a GStreamer `appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=10000
speed-preset=superfast ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink`
(port 5004), and side-channels the frame id as `struct.pack("<I", frame_id)` over plain UDP
to port 5005 so a consumer can align video frames with metadata. Caps are recomputed only
when the encoding/resolution string changes.

### Docker strategy

Two stages, each finished with a **`docker commit` warm-start trick**:

1. `build_base_image.sh` → `Dockerfile...core_base` from `nvcr.io/nvidia/isaac-sim:2023.1.1`.
   Installs ROS 2 Humble desktop, GStreamer + PyGObject, clones and `colcon build`s
   `IsaacSim-ros_workspaces`, `pip install pyproj transforms3d` **into Isaac's python**.
   Runs `isaacsim_warmup.sh` (which appends Cesium deps and `useFabricSceneDelegate = true`
   to `omni.isaac.sim.python.kit`, then `runapp.sh --no-window`), waits for
   `"Isaac Sim App is loaded."` in logs, `pkill`, then commits the shader/asset caches
   back into the image.
2. `build_simulation_image.sh` → copies repo + `extensions/*` + vendored `cesium_exts_2023`
   into `/isaac-sim/exts`, rebuilds `isaac_ros2_messages`, boots, waits for `"RTX ready"`,
   commits again.

Both need `runtime: nvidia`, `network_mode: host`, `privileged: true`, X11 socket +
`.Xauthority` mounts, `ROS_DOMAIN_ID=13`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
The base compose sets `build.network: host` specifically so Isaac's python can reach the
internet during build.

### Operational gotcha worth keeping documented

Long Isaac Sim sessions grow `~/.cache/ov/cesium-request-cache.sqlite-wal` to 600–700 GB and
the next launch fails. Fix: delete it. Exposed as `dev_utils.delete_cesium_cache()`.

---

## 5. Defects & design problems found (learn from these)

Verified by reading the source. Ordered roughly by severity.

### Correctness bugs

1. **`UdpBot` violates its base class contract.** `BaseUDPSender.get_next_point(self, step: int)`
   is abstract; `UdpBot.get_next_point(self)` takes no `step`. `_run_loop` calls
   `self.get_next_point(cur_step)` → `TypeError`. So `UdpBot.run()` is broken; the class only
   works through its own imperative methods, which each drive their own send loop.
   The inheritance is wrong — `UdpBot` is not a "sender that yields points", it's a
   *stateful vehicle model*. In v6, separate `PoseSource` (yields poses) from
   `VehicleModel` (integrates commands) from `Transport` (ships bytes).
2. **`OgnSimROS2ImagePublisher` stores state on `db`, not `internal_state`.** It uses
   `hasattr(db, "_republisher_node")` and `db._republisher_node = ...`. `db` is a
   per-compute database wrapper, not guaranteed-stable per-node storage — that's what
   `internal_state` is for. `release(node)` then looks for `node._republisher_node`, a
   *different* object than the `db` it was set on, so the republisher is likely never
   cleaned up. Also its `release` type hint says `node: ROS2ImageRepublisher`, which is wrong.
3. **`rclpy.shutdown()` called from individual node `release()`s.** `OgnSimROS2Gimbal`,
   `OgnSimROS2ToGlobalPosition`, `OgnSimROS2RangePublisher`, `OgnSimROS2GlobalPosePublisher`
   each call `rclpy.shutdown()` on release. With several of these in one graph, the first
   teardown kills the shared rclpy context out from under the others. rclpy lifetime must be
   owned **once**, at process scope. (The dev_kit got this right with
   `safe_rclpy_init/shutdown`; the extensions didn't.)
4. **`sensor_node.py` publishes a one-frame-stale range and references `og` without importing it.**
   `compute` fires `asyncio.ensure_future(get_range_from_ray(...))` then immediately reads
   `db.internal_state.range`, so the value is always from a previous tick. The annotations
   `db: og.Database` have no `import omni.graph.core as og` — it only survives because
   ScriptNode globals inject `og`. Under our lint/type rules this is a non-starter.
5. **`omniverse_utils.get_prim_at_path` raises on an invalid prim, but callers treat it as
   truthy-or-None.** e.g. `if get_prim_at_path(graph_path):` in `_configure_camera` and
   `_update_laser_sensor` — that branch can never be False; it raises instead. And
   `_get_camera_path()` has a `return None` after the `except KeyError` block that is
   unreachable in the success path but reachable on KeyError, so downstream `None` guards
   are inconsistent. There's a separate `is_prim_valid()` that does the right thing and is
   used in only one place. Pick one contract: `get_prim(path) -> Prim` (raises) plus
   `find_prim(path) -> Prim | None`.
6. **`docker_isaac_manager._create_temp_compose` uses `first_service_name` in its own
   `except` handler**, where it may be unbound → `UnboundLocalError` masking the real error.
7. **`debugger/udp_sender.py`'s on-screen packet table lies.** It documents roll/pitch/yaw as
   `[deg]` while `send_packet` transmits `math.radians(...)`. The wire format is radians.

### Design problems

8. **The four `*Capture` classes are ~95% copy-paste.** `PoseCapture`, `DistanceCapture`,
   `BboxCapture` differ only in message type, topic constant, and attribute name. Replace with
   one generic `TopicRecorder[MsgT]` + pluggable serialiser.
   Likewise `HostIsaacManager` and `DockerIsaacManager` duplicate `FLAGS`, `FLAG_MAP`,
   `_wait_with_msg`, and the readiness-sentinel logic verbatim — should be a shared base.
9. **Constants are duplicated across module boundaries with no single source of truth.**
   `/isaac_core/image_rgb`, `/isaac_core/global_pose`, `/isaac_core/bbox`,
   `/isaac_core/distance_sensor`, port `33333`, headers `0xAC/0xDC`, format `"<6d"` and the
   `"rclpy loaded"` sentinel are each re-declared in 2–4 places (`simulation/consts.py`, each
   `core_capture/*.py`, `base_udp_sender.py`, `sat_node.py`, both isaac managers,
   `debugger/udp_sender.py`). The dev_kit is a separate pip package so it *can't* import
   `simulation.consts` — the real fix is a shared, installable contract package.
10. **Readiness detection is a log-string grep** (`"rclpy loaded"`, `"Isaac Sim App is loaded."`,
    `"RTX ready"`) plus an unconditional `sleep(5)`/`sleep(10)`. Fragile across Isaac versions.
    Use an explicit readiness signal we control (a socket, a file, or a ROS 2 topic/service the
    sim publishes when fully initialised).
11. **`main_sim.py` mutates `os.environ["LAUNCH_CONFIG"]` with JSON and relies on import
     order** — `sim_app.py` calls `SimulationApp()` at *module import time* reading that env var,
     so `from sim_app import Simulation` must happen after the env is set. Implicit and easy to
     break. Pass config explicitly to a factory instead.
12. **`sys.path.insert` + flat sibling imports** (`import consts`, `import sim_utils`,
     `from sim_app import ...`). Not a package; can't be installed or imported by tests.
     Incompatible with the `src/` layout we now have.
13. **Two OmniGraph subgraphs poll independently.** `ROS2CameraImageExport` runs off
     `onPhysicsStep` while `UDPOdomSync` runs off `OnPlaybackTick`, and each publisher does its
     own `last_publish_time` rate gating. No shared clock ⇒ image and pose for "the same instant"
     aren't guaranteed aligned. The frame-id side channel is a workaround for exactly this.
     A single timing authority would be better.
14. **`_update_script_node_paths` rewrites absolute host paths into the USD at every launch.**
     Necessary because the `.usda` files store machine-specific `inputs:scriptPath`. Means the
     stage is mutated per machine. Prefer resolving script bodies at runtime, or keep node logic
     in a real extension rather than a ScriptNode with a filesystem path.
15. **Hardcoded prim paths as strings scattered through `sim_app.py`** —
     `/Environment/distance_sensor/ActionGraph/ros2_distance_publisher`,
     `/Environment/bbox_publisher/BboxPublisher/script_node`,
     `/Environment/SAT/takeSAT/script_node`, and `"ROS2OdomSync" if com_ros else "UDPOdomSync"`.
     Any USD rename silently breaks configuration. Needs a typed scene-contract layer.
16. **`isaac_ros2_messages` is hijacked, not owned.** `simulation/ros2_interfaces/CMakeLists.txt`
     declares `srv/IsaacPose.srv`, `srv/GetPrims.srv`, `srv/GetPrimAttributes.srv`,
     `srv/GetPrimAttribute.srv`, `srv/SetPrimAttribute.srv` — **none of which exist in this repo**.
     It only builds because you copy these files *over* NVIDIA's package inside
     `IsaacSim-ros_workspaces` and inherit their `srv/`. Upstream changes break us silently.
     Ship our own `isaac_core_msgs` package.
17. **`dev_kit/pyproject.toml` declares `dependencies = []`** while the code imports `rclpy`,
     `cv2`, `cv_bridge`, `numpy`, `transforms3d`, `yaml`, and `isaac_ros2_messages`. It also
     sets `readme = "README.md"` — and **no `README.md` exists in `simulation/dev_kit/`**, so a
     PEP 621 build should fail on the missing file. `requires-python = ">=3.8"` contradicts the
     `X | None` syntax used in `docker_isaac_manager.py` (3.10+). Version pinned `1.0.0`,
     author `Mr_Blum <gg@gg.gg>`.
18. **Zero tests, zero CI.** No test files anywhere in the repo, no `.github/`. Nothing was
     ever verifiable except by hand.
19. **Internal infrastructure committed to git.** `TILESETS_HTTP_SERVER_URL =
     "http://10.20.15.122:8088"` in `consts.py`, and nine hardcoded `cesium:url` values baked
     into `usd/maps/earth/earth.usda` pointing at the same host (`telem_*`, `Tulkarem_11_22`,
     `nablus`, …). Plain HTTP, no auth. Must be configuration, not source.
20. **Vendored binaries in git** — two full Cesium extension builds (`.so`, `.whl`, images)
     under `docker/simulation_docker/cesium_exts_2023/`. Bloats the clone and pins us to
     Kit 105.1/106.5. Fetch at build time instead.
21. **`run_sim.sh` / `run_sim_in_docker.sh` define `BBOX`, `DISTANCE_SENSOR`, `SAT`, `RTP`
     variables and then don't use them** (`run_sim.sh` passes only `$USD_PATH $COM`), and both
     end with a stray `PID=$!` after a foreground `bash -ic`. Dead code; `ERA` would flag it.
22. **Docs drift.** README documents `HostIsaacManager(..., rtp: bool = False)` but the
     parameter is `image_rtp`. `udp_utils.py` carries a commented-out alternative function
     signature. `UDPOdomSync` graph has `ui:displayName = "ROS2OdomSub"` and dead
     `inputs:value.connect` entries referencing nodes that no longer exist
     (`ros2_odometry_subscribe`, `global_position_to_local_position_01`), plus an orphan
     `ros2_absolute_pose_subscriber` node — leftovers from GUI copy-paste.
23. **`bare except: pass` / `except Exception: pass` used liberally** (`sat_node.cleanup`,
     `base_udp_sender.close`, `bbox_node.setup`'s `rclpy.init()`, several `release()`s).
     Silently swallows real failures.

### Things 2023 got *right* — preserve deliberately

- USD-layer-per-feature composition driven by CLI flags. Genuinely elegant.
- Context-manager lifecycle for the whole simulator (`with HostIsaacManager(...)`).
- A separate installable dev kit so consumers never fork the sim repo.
- Out-of-process sim libs to escape Isaac's bundled interpreter.
- `perf_counter`-with-accumulator fixed-rate loops (drift-free) in `base_udp_sender`.
- Hold-last-good-packet on malformed UDP input.
- Frame-id side channel for aligning video with metadata.
- Docker warm-start `commit` to skip shader compilation on every boot.
- SLERP-based orientation interpolation with a lerp fallback near identity.
- Consistent `internal_state` / `compute` / `release` node idiom and `SIM | XXX |` log prefixes.
- An exceptionally thorough README with collapsible sections — the team clearly used it.

---

## 6. Isaac Sim 6 migration notes

Verified against `docs.isaacsim.omniverse.nvidia.com` (6.0.1 docs, migration guides index).

- **Isaac Sim 4.5 renamed every extension**: `omni.isaac.*` → `isaacsim.*`, with many
  extensions split. So `omni.isaac.kit`, `omni.isaac.core`, `omni.isaac.core_nodes`,
  `omni.isaac.ros2_bridge`, `omni.isaac.core.utils.{extensions,nucleus,rotations}`,
  `omni.isaac.core.prims.XFormPrim` — **all of these 2023 imports are dead**.
  New homes include `isaacsim.core.api`, `isaacsim.core.nodes`, `isaacsim.sensors.*`.
  There is an `isaacsim.core.deprecation_manager` extension that emits mapping warnings —
  useful for mechanically finding replacements.
- **Isaac Sim 6.0 introduces a Core Experimental API** (a rewrite of the Core API) and
  **multiple physics backends: PhysX and Newton** (GPU-accelerated, Warp-based).
  We're a camera/kinematics rig, not a dynamics sim, so Newton is mostly irrelevant to us —
  but `SimulationContext.initialize_physics()` behaviour needs re-checking.
- **6.0 migration guides that touch us**: *Camera Sensors* (camera → experimental RTX),
  *RTX Sensors*, *Physics Sensors*, *PhysX Lidar / Generic / Lightbeam → physics raycast*.
  That last group matters for `sensor_node.py`'s `omni.kit.raycast.query` rangefinder — there
  is now a first-class **physics raycast sensor**, so we may be able to delete our custom
  raycast ScriptNode entirely.
- **6.0 ROS 2 OmniGraph migration only affects `ROS2 Publish Transform Tree` and
  `ROS2 Publish Joint State`** (they now need `Isaac Compute Transform Tree` /
  `Isaac Read Joint State` feeding them instead of resolving prims internally).
  **We use neither**, so this specific guide is a non-issue for us.
- **Multi-tick rendering** is new in 6.0: cameras/RTX lidars can be scheduled at independent
  rates and offsets driven by physics time. This is a direct, better answer to problem #13
  (unsynchronised image/pose publishing) — investigate before reimplementing our own rate gates.
- Also new and relevant: **RTSP camera streaming** is documented natively, and there's an
  **RTX Acoustic Sensor**. Native RTSP may partly replace our GStreamer RTP sim lib.
- Open questions to resolve: is **Cesium for Omniverse** available for Kit 108 / Isaac Sim 6?
  Which ROS 2 distro ships with the 6.0 container (Humble vs Jazzy)? Does the
  `[qx,qy,qz,qw]` quaternion-order quirk still exist? What is the exact
  `nvcr.io/nvidia/isaac-sim` tag for 6.0? Driver minimum?
  **The Cesium answer is a hard dependency — the entire terrain layer rests on it.**

---

## 7. Agreed architecture

Settled with Ofer 2026-08-24. Decisions are in §8; this is the shape they produce.

### Out of scope (explicitly dropped from the 2023 repo)

`docker/` and the whole container workflow · `link_app.sh` · `tools/packman` and
`tools/scripts`. Not migrating them now. No backward compatibility is owed to the
2023 `isaac_core_dev_kit` API — nothing depends on it.

### Repository layout

```
isaac_core_6/
├── config/
│   ├── default.toml               # complete config surface, documented inline
│   └── examples/                  # copy-and-edit scenarios
├── docs/
├── extensions/                    # Kit extensions — symlinked into $ISAACSIM_PATH/exts, never pip
│   ├── isaac_core_ogn.math/
│   ├── isaac_core_ogn.position/
│   ├── isaac_core_ogn.sensors/
│   └── _template/
├── scripts/                       # setup.sh, link_extensions.sh — shims, zero logic
├── src/isaac_core/                # ONE pip distribution
│   ├── __init__.py                # MUST stay lightweight: no omni, no rclpy, no gi
│   ├── contracts/                 # topics, ports, packet spec, prim-path types, enums
│   ├── config/                    # pydantic schema + layered loader
│   ├── geo/                       # LLA/ECEF/ENU, NED↔ENU, geodesics, SLERP
│   ├── protocol/                  # wire codecs
│   ├── vehicle/                   # kinematics, trajectories, missions
│   ├── devkit/                    # user-facing API; needs rclpy, never omni
│   ├── sim/                       # ONLY package allowed to import omni/carb/pxr
│   ├── sidecar/                   # services needing system python3 (GStreamer)
│   ├── debug/                     # GUIs, inspectors
│   └── assets/
│       ├── scenes/                # base stages — small, ship in the wheel
│       └── layers/                # feature layers + manifests — ship in the wheel
├── assets_heavy/                  # large binaries (HDRIs) — repo only, NOT in the wheel
└── tests/{unit,contract,integration}/
```

### Install model — one distribution, three interpreters

```bash
pip install -e .                                    # host/laptop: kernel + devkit
$ISAACSIM_PATH/python.sh -m pip install -e .[sim]   # into Isaac's bundled python
python3 -m pip install -e .[sidecar]                # only if RTP streaming is wanted
```

One package name, one version, no cross-version matching. `omni` comes from Isaac and
`rclpy` from `source /opt/ros/humble/setup.bash` — **neither is ever a pip dependency**.
Actual pip deps are small: numpy, pyproj, transforms3d, pydantic, tomli (py310 has no
`tomllib`), and PyGObject under `[sidecar]` only.

Wrapped by `scripts/setup.sh` (all three installs + extension symlinks + verify) and
`isaac-core doctor` (reports what's missing and how to fix it).

### Layering, enforced by `import-linter` in pre-commit

```
contracts ← config ← geo ← protocol ← vehicle        (pure; no Isaac, no ROS, no I/O)
                                  ↖ devkit           (+rclpy)
                                  ↖ sim              (+omni/carb/pxr)
                                  ↖ sidecar          (+gi)
```

Kernel must not import `sim`/`devkit`/`sidecar`. `sim` must not import `devkit`.
Mechanically checked so it can't rot quietly.

### Two planes — keep them separate

- **Data plane** (high rate, per frame): pose IN via ROS 2/MAVROS (ArduCopter SITL when
  physics matters) or UDP (when it doesn't); image/range/bbox OUT. Selected per vehicle by
  `pose_source`.
- **Control plane** (low rate, commands): JSON-RPC over a **local socket**, not ROS 2.
  `get_state`, `get_capabilities`, `get_config`, `set_config(patch)`,
  `enable_feature`/`disable_feature`, `load_scene`, `reset`/`pause`/`resume`/`step`,
  `capture_frame`, `set_pose`, telemetry stream.
  Fixes the 2023 restart-to-reconfigure problem, needs no rclpy in the caller, and the port
  opening **is** the readiness signal — retires the `"rclpy loaded"` grep and `sleep(5)`.
  ⚠ Bind localhost-only by default; token required for any non-local bind; confine
  `capture_frame` paths under a configured output root.

### Decomposing the god object

`Simulation` (2023: 353 lines, names every feature explicitly) splits into
`SimulationRuntime` (kit lifecycle + step loop, nothing else), `StageComposer`,
`FeatureLayer` (protocol), `Configurator` (applies config via declared bindings),
`ControlPlane`. **`SimulationRuntime` must never mention a feature by name.**
Target: no class over ~150 lines.

### Config

pydantic v2 models, TOML on disk, **library-only — no terminal step**. Layered, later wins:

```
package defaults → config/*.toml → --config / $ISAAC_CORE_CONFIG
  → env (ISAAC_CORE__CAMERAS__EO__FOV_DEG=90) → CLI → runtime patch via control plane
```

Complete surface, not the ~15 values of 2023's `consts.py`: per-camera intrinsics and
resolution, gimbal start pose *and rate limits*, ENU reference, tile server URL, every
topic, every port, QoS, rates, sensor ranges, log levels, feature enable/disable, scene,
asset search paths, output roots. Devkit resolves config in the user's script, writes a
temp file, launches with `--config`; the mutable subset is patchable at runtime.

`pose_source = "udp" | "ros" | "script" | "replay" | "mavlink"` replaces the
mutually-exclusive `--com-ros`/`--com-udp` booleans — one axis, open for new sources.

### Feature layers — manifest-driven, no central registry

```
<layer_dir>/thermal_cam/
├── layer.toml     # id, usd, mount ("/Environment/{instance}/…"), requires,
│                  # provides, provision, and [[bindings]] mapping config keys →
│                  # prim attributes
└── thermal_cam.usda
```

Discovery scans shipped `assets/layers/` plus `layer_search_paths` from config.
Adding a layer = dropping a directory in — **no code change, works from outside the repo.**
Layers needing Python declare an entry point (`[project.entry-points."isaac_core.layers"]`).
The `[[bindings]]` table is what removes every hardcoded prim path from `sim_app.py`
(defect #15), and gives a free contract test: assert declared prim paths exist in the USD.

### Stage capabilities — never crash on a missing prim

`StageProbe` runs after the base stage opens → `StageCapabilities` (tilesets root, bboxes
root, georeference, cameras, semantics). Layers declare `requires`; unmet ⇒ **skipped with a
reason**, not a crash. `--strict-features` flips to fail-fast for CI. Layers may declare
`provision = true` for prims they can safely create. Startup prints an enabled/skipped
report. `get_prim_at_path` becomes `require_prim()` (raises typed `StagePrimMissing`) and
`find_prim() -> Prim | None`, fixing defect #5.

### Devkit — install-not-locate

Never needs a repo path. `Sim.launch()` runs `<isaac_python> -m isaac_core.sim --config …`;
it needs the *Isaac* path (`ISAACSIM_PATH`, config, or probe), never the repo.
`Sim.attach(host=…)` connects to an already-running sim on any machine and returns the
**same `SimSession` type**, so scripts are identical either way. Scenes referenced by
logical name (`scene="earth"`), resolved via asset search path.

### Multi-camera and swarm — N instances from day one

Default 1 vehicle / 1 camera, collapsing to today's flat topic names. Beyond that,
`VehicleSpec` / `CameraSpec` keyed collections; topics and prim paths namespaced by
instance (`/isaac_core/lead/eo/image_rgb`); `{instance}` in layer manifest mounts.
Per-camera position write, frame read, and spec write (resolution, FOV, focal length).
Mixed pose sources per vehicle come free — lead on SITL, wingmen on UDP.

**UDP packet format is unchanged: the exact 51 bytes of 2023.** One UDP port per vehicle,
allocated `base_udp_port + index` (default base 33333, explicitly overridable). No vehicle
id, no version byte — considered and rejected as speculative; per-port isolation is simpler
to debug and keeps the existing debugger GUI and senders working. Discipline kept instead:
**every angle parameter carries its unit in the name** (`roll_deg`, `roll_r`), because the
2023 debugger documented degrees while transmitting radians.

### Sidecar — supervised, not forked

The *reason* was sound (Isaac's python can't host PyGObject/GStreamer); the mechanism wasn't.
Own module, run as `python3 -m isaac_core.sidecar --config <path>`, services declared in
config, with health checks, structured logs, restart policy, graceful-then-forced shutdown.
**The sim process no longer forks children** — the devkit launches sim and sidecar as peers.
Communicates only over documented contracts, so it can run on another machine.

### Assets — what ships in the wheel

Measured in the 2023 repo: `usd/` is 78 MB total, but **72 MB is one file**
(`sunflowers_puresky_4k.exr`). Everything else is ~6 MB, and 5.8 MB of *that* is the two
camera USDs at 2.9 MB each — bloated because they embed the `OmniverseKitViewportCameraMesh`
/ `CameraModel` gizmo mesh. `earth.usda` is only 16 KB because terrain streams from the
Cesium tile server over HTTP.

So: **scenes + layers ship in the wheel** (~6 MB, and less once the camera gizmo mesh is
referenced rather than embedded). **Large binaries stay in `assets_heavy/`**, repo-only,
resolved via `$ISAAC_CORE_ASSET_PATH` / config search path, and **optional** — a scene
missing its HDRI falls back to a default dome light rather than failing, consistent with the
capability-probing philosophy. Scenes live in a personal clone shipped with the repo for now;
S3-style hosting is a later exploration, not now.

### OmniGraph node type names — normalise them

2023 has four naming styles across seven nodes, because the `.ogn` top-level key is
inconsistent with the filename:

| Node type string in USD | style |
|---|---|
| `omni.sim.math.GlobalPositionToLocalPosition` | clean |
| `omni.sim.position.ROS2ToGlobalPosition` | clean |
| `omni.sim.position.UDPToGlobalPosition` | clean |
| `omni.sim.sensors.ROS2Gimbal` | clean |
| `omni.sim.sensors.SimROS2ImagePublisher` | leaked `Sim` |
| `omni.sim.sensors.SimROS2RangePublisher` | leaked `Sim` |
| `omni.sim.sensors.OgnSimROS2GlobalPosePublisher` | leaked `OgnSim` |

v6 rule: file `OgnGlobalPositionToLocalPosition.py/.ogn`, `.ogn` key
`GlobalPositionToLocalPosition`, resulting type
`isaac_core_ogn.math.GlobalPositionToLocalPosition`. No `Ogn`/`Sim` prefix ever leaks into a
node type. Add a contract test asserting `.ogn` key == filename minus `Ogn` prefix.

### Testing

Kernel is pure ⇒ real CI with no GPU and no Isaac: geo round-trips, packet codec
round-trip + fuzz, trajectory generators, config layer precedence, manifest validation,
capability resolution. `FakeSimSession` + in-memory transport for devkit/mission tests.
Contract tests for manifest↔USD and ogn-key↔filename drift. Headless boot smoke test if a
GPU runner exists.

### Why this scales

New feature = new layer directory; no central registry to edit, no god-class branch, no
recurring merge conflict in the same two files (2023's `OPTIONAL_USDS` and
`SIM_LIB_REGISTRY`). New pose source = one `PoseSource` implementation. New vehicle
behaviour = pure `vehicle` code, testable without a GPU. New config knob = one pydantic
field, with layering/env/CLI/validation free. N instances from day one, so swarm work never
needs a topic/prim-path re-plumb.

### Drone features to build beyond 2023

Waypoint missions with loiter/hold · takeoff/land/RTL · velocity- and acceleration-limited
motion (2023 teleports at constant speed, no dynamics) · wind/turbulence · battery and
failsafe modelling · gimbal servo dynamics and rate limits (2023 snaps instantly) · target
tracking / follow-me · geofencing · sensor noise models · MAVLink in the loop · mission
scripting and replay · deterministic seeded scenarios for regression tests.

---

## 8. Decisions log

| # | Decision | Rationale |
|---|---|---|
| D1 | Kit extension namespace `isaac_core_ogn.*` | Ofer's pick — makes clear these are action-graph nodes, not general ROS 2 nodes. Also avoids the `isaac_core.math` collision: Kit puts each extension's python root on `sys.path`, so an extension providing `isaac_core/` would shadow the pip-installed `isaac_core`. Sibling root, no namespace-package config needed. |
| D2 | **One** pip distribution, `[sim]`/`[sidecar]` extras | Four packages was too much friction for a team-facing sandbox. One name, one version. |
| D3 | pydantic v2 for config, library-only | Validation error messages are UX for a whole team. **No terminal command** — validation happens in-process at load. |
| D4 | Control plane = JSON-RPC over local socket, not ROS 2 | No rclpy needed in the caller; available before ROS is up; doubles as the readiness signal. Optional ROS service bridge later if wanted. |
| D5 | CLI is `isaac-core <verb>` | `run`, `doctor`, … |
| D6 | UDP packet format unchanged (51 bytes); one port per vehicle, `base + index` | Vehicle-id + version byte considered and **rejected** as speculative. Per-port isolation is easier to debug and keeps the existing debugger and senders valid. |
| D7 | Swarm + multi-camera supported, default 1/1 | Cheap now, expensive to retrofit into topics and prim paths. |
| D8 | Scenes + layers ship in the wheel; heavy binaries repo-only and optional | 72 of 78 MB is one HDRI. |
| D9 | No back-compat for `isaac_core_dev_kit` | Nothing depends on it; free to design properly. |
| D10 | Drop `docker/`, `link_app.sh`, `tools/` | Ofer's call, for now. |
| D11 | Cesium for Omniverse confirmed working on Isaac Sim 6 | Terrain layer unblocked. |
| D12 | `extensions/` at repo root, not under `src/` | Kit extensions aren't pip packages. Keeps "everything in `src/` is the pip package" true with no exceptions. |
| D13 | All layer settings uniform under `[layers.<layer_id>]` | Lets third-party layers extend the config with zero core changes. Deliberate exception: cameras/gimbals are first-class under `[vehicles.X]` because they're tuned constantly and read far better there. |
| D14 | Rotation frame (world vs body) is configurable | Ofer's requirement. `RotationFrame.WORLD` = extrinsic, `R_delta @ R_current`; `RotationFrame.BODY` = intrinsic, `R_current @ R_delta`. Lives in `contracts.frames` so the math node, the gimbal, and the vehicle model all honour the same setting. 2023 hardcoded body-frame for the gimbal (`qmult(q_drone, q_offset)`) and world-frame for `UdpBot` turns (`delta_R @ R`), with no way to choose. |
| D15 | **Kiro never authors `.usda` files.** | Ofer creates all USD via the Isaac Sim 6 GUI and saves it, which guarantees valid/correct files instead of hand-written multi-thousand-line text. My job is to specify precisely *what* to create and what to put in it, then bind to it via layer manifests. |
| D16 | `GeoPoseStamped` carries a **real quaternion** via `EulerToQuaternion` | Ofer's decision. Replaces the old repo's abuse of stuffing roll/pitch/yaw into the quaternion's x/y/z with w pinned to 1.0. Anything downstream parsing the old convention needs updating. Enforced by `tests/unit/test_usd_layers.py`. |
| D17 | `header.frame_id` is a **coordinate frame name**, not a sequence counter | ROS 2's `std_msgs/Header` has only `stamp` and `frame_id` — ROS 1's `seq` was deliberately removed. `frame_id` is documented as "Transform frame with which this data is associated". The old repo used it as a frame counter, which required a custom rclpy republisher; that is the real reason its image path went `raw_rgb` → `image_rgb`. Correlate frames with `header.stamp` instead. |
| D18 | ENU reference is **derived from the scene's Cesium georeference**, not kept in sync by hand | `geo.enu_reference` and `cesium:georeferenceOrigin` describe the same fact; disagreement silently puts the aircraft over the wrong ground. Composition now reads the scene. Explicit config still wins (needed for non-Cesium stages), and a lat/lon disagreement is reported rather than left silent. |
| D19 | **Rejected**: writing lat/lon/alt directly to a Cesium Globe Anchor instead of computing local ENU | Ofer's idea, investigated properly. Cesium's plugin listens to `UsdNotice::ObjectsChanged` (verified: it has a `UsdNotificationHandler`) while OmniGraph writers target Fabric — `usdWriteBack` is false by default, so the write is invisible. This is almost certainly the 2023 failure Ofer remembered. Making it work needs per-frame USD authoring, the exact thing Fabric exists to avoid. Cesium also ships **zero** OmniGraph nodes, so there is no supported wiring. And it would couple the pose pipeline to Cesium, breaking non-Cesium stages (2023 shipped `full_warehouse.usda`), while moving tested kernel logic into an untestable C++ plugin. The precision argument does not hold either: our ENU is exact geodesy in doubles, and float32 render resolution is 8 mm at 100 km. D18 captures the real benefit without any of the cost. |

### Working constraints

- **Never** `git add` / `git commit` / `git push` in this repo. Read-only git only
  (`status`, `log`, `grep`) — Ofer's explicit instruction.
- `/home/ofer/clones/isaac_core_2023` is **read-only reference**. Never modify.
- **No venv, no conda.** The team installs system-wide. Kiro must **never install
  anything** — instead, add the dependency to `requirements.txt` (runtime) or
  `requirements-dev.txt` (tooling) and tell Ofer what to run.
- `requirements.txt` is the team's install path; `[project.dependencies]` in
  `pyproject.toml` must mirror it. `tests/unit/test_packaging.py` fails if they drift,
  if a dev tool is not `==` pinned, or if a pinned version disagrees with the `rev:`
  in `.pre-commit-config.yaml`.
- Dependencies pip cannot provide (`omni`, `carb`, `pxr`, `rclpy`, `gi`, `tkinter`) are
  documented as comments in `requirements.txt`, never as requirements. This is why the
  kernel packages must not import them.

---

## 9. Session log

- **2026-08-24 (a)** — Read isaac_core_2023 end to end: README (960 lines), all
  `simulation/`, all four extensions and their `.ogn` schemas, the full `dev_kit`, all three
  ScriptNodes, the Docker chain, ROS 2 msg definitions, `debugger/`, and the OmniGraph wiring
  in `usd/cameras/udp_camera.usda`. Verified defects by inspection and `git grep` (no tests,
  no CI, hardcoded `10.20.15.122` in `consts.py` + 9 URLs in `earth.usda`, missing
  `dev_kit/README.md`). Checked Isaac Sim 6.0 migration docs. Wrote §1–§6.
- **2026-08-24 (b)** — Design discussion with Ofer over three rounds. Settled D1–D12.
  Corrected an over-engineered proposal (four pip packages → one) and withdrew the UDP packet
  format change. Measured asset sizes and audited `.ogn` node type naming (found four styles
  across seven nodes). Replaced the earlier unagreed §7 proposal with the agreed
  architecture above.
- **Nothing has been implemented yet.** No source written in isaac_core_6; only `KIRO.md`
  exists beyond the team template.

- **2026-08-24 (c)** — Implemented the first kernel layer. Retargeted `pyproject.toml`
  from the template placeholder to `isaac-core` (one distribution, `[sim]`/`[devkit]`/
  `[sidecar]`/`[debug]`/`[dev]` extras), added narrow mypy overrides for
  `omni`/`carb`/`pxr`/`rclpy`/`gi`/`isaacsim` plus `pyproj`/`transforms3d`, and wrote the
  import-linter layering contracts. Removed the `dingo_project` placeholder. Built
  `isaac_core.contracts` (`frames`, `pose`, `packet`, `topics`, `ports`, `prims`) with 91
  passing tests. **Verified:** all 14 pre-commit hooks PASS at pinned versions
  (ruff 0.8.2, mypy 1.14.1), 91/91 pytest, and `isaac_core.contracts` imports in a bare
  interpreter with neither Isaac Sim nor ROS 2 present.

- **2026-08-24 (d)** — Ofer installed everything system-wide; verified pinned versions now
  match exactly (ruff 0.8.2, mypy 1.14.1, pytest 8.3.4, pre-commit 4.0.1, gitlint 0.19.1,
  import-linter, pydantic 2.13.4, pyproj 3.7.1, transforms3d 0.4.2, tomli 2.2.1).
  Added `requirements.txt` / `requirements-dev.txt` plus `tests/unit/test_packaging.py`,
  which fails if they drift from `pyproject.toml`, if a dev tool is not `==` pinned, or if a
  pin disagrees with a `rev:` in `.pre-commit-config.yaml`. Corrected the `[sidecar]` extra:
  PyGObject comes from apt, not pip.
  Wrote `config/default.toml` (the complete surface) and `isaac_core.config.schema`
  (pydantic v2: `extra="forbid"`, `frozen=True`, cross-reference validation, and derived
  values for udp ports / mount points / topic namespacing).
  **Verified:** 14/14 pre-commit hooks PASS, 149/149 pytest, import-linter 2 contracts KEPT.

### Environment notes (this machine)

- **No venv, no conda** — everything is installed system-wide with `pip install --user`.
  Kiro must never install anything; add to `requirements*.txt` and tell Ofer.
- **No `python3-venv`** (`ensurepip` unavailable). Irrelevant now that tooling is system-wide,
  but it means `pre-commit`'s isolated hook envs (virtualenv-based) are still how the gate
  runs. Terminal tooling now matches those pins, so `ruff`/`mypy` locally agree with the gate.
- Verification recipe: `python3 -m pytest -q`, `PYTHONPATH=src lint-imports`, and
  `pre-commit run <hook> --files <paths>` (files are untracked, so `--all-files` skips them).

- **2026-08-24 (e)** — Ran `geo`, `protocol` and the config `loader` as three parallel
  sub-agents (independent: loader needs only `config`, the other two only `contracts`),
  then an integration stage. Delivered:
  - `isaac_core.geo` — `EnuConverter` (LLA↔ECEF↔ENU, **including the inverse 2023 never
    had**), `ned_to_enu`/`enu_to_ned` as true inverses that reject an already-correctly-tagged
    input, euler/matrix/quaternion helpers on `EULER_AXES`, `compose_rotation` implementing
    D14 in one place, `slerp` with the 0.9995 lerp fallback, haversine distance,
    `meters_to_latlon_offset(north_m, east_m, ref_lat_deg)` — renamed from 2023's ambiguous
    `(dx, dy, ref_lat)` where `dx` meant latitude.
  - `isaac_core.protocol` — `encode`/`decode` for the 51-byte packet, a typed error
    hierarchy (`PacketLengthError`/`HeaderError`/`ChecksumError`/`PayloadError`, which 2023
    lacked entirely — it logged and returned the last good packet for every failure kind),
    and `HoldLastGoodDecoder` preserving 2023's deliberate hold-on-corruption behaviour but
    making failures *observable* via a counter instead of only logging. `encode` refuses a
    non-NED orientation.
  - `isaac_core.config.loader` + `sources` — layered resolution
    (defaults → file → `ISAAC_CORE__*` env → CLI dotted keys), non-mutating `deep_merge`
    where lists replace rather than append, provenance tracking for `config explain`, and
    `dump_toml`.
  - `tests/unit/test_kernel_purity.py` — imports each kernel package in a **subprocess with
    omni/carb/pxr/rclpy/gi/isaacsim forced unimportable**, so the layering's central promise
    is explicitly tested rather than assumed.
  - import-linter widened from 2 to **8 contracts**, including an `independence` contract
    (geo ⊥ protocol, since they are siblings, not a stack) and a `forbidden` contract barring
    every kernel package from importing omni/carb/pxr/rclpy/gi.

  Two things I changed after reviewing the sub-agents' output:
  1. Added `test_encode_matches_bytes_captured_from_the_2023_encoder`, asserting against a
     **hex literal** captured by running 2023's `_build_packet` verbatim. The existing golden
     test built its expectation from the same `contracts.packet` constants `encode()` uses, so
     it could not catch the spec itself drifting. Verified our encoder is byte-identical to
     2023's.
  2. Renamed `euler_to_matrix`/`euler_to_quaternion` parameters to `roll_r`/`pitch_r`/`yaw_r`.
     The integrator had argued the maths layer could skip the unit-suffix convention; I
     disagree — that layer is exactly where a unit error is invisible, and 2023's
     degrees-vs-radians bug lived in precisely such a seam.

  **Verified:** 317/317 pytest, 13/13 pre-commit hooks PASS, import-linter 8 contracts KEPT.

- **2026-08-24 (f)** — Three more parallel tracks (`vehicle`, `extensions`, `sim` layer
  system), then per-extension hardening agents (one each for math / position / sensors,
  at Ofer's suggestion — good granularity, worth repeating).
  - `isaac_core.vehicle` — `VehicleState` (frozen, world rotation matrix), `MotionLimits`
    (speed/accel/turn-rate/climb caps; `None` = unlimited, reproducing the old constant-speed
    behaviour), `Trajectory` protocol with Hold/Orbit/Path implementations, and motion
    primitives. **Purely synchronous generators — no sockets, no threads, no `time.sleep`**,
    which is the fix for defect #1 (the old `UdpBot` fused vehicle model + socket + blocking
    loop, and broke its own base-class contract). Rotations delegate to
    `geo.compose_rotation`/`geo.slerp`; a test asserts no module here imports socket,
    threading or time.
  - `extensions/isaac_core_ogn.{math,position,sensors}` + `_template` — 8 node types, all
    normalised (no `Ogn`/`Sim` leaking into a type string). Nodes are thin adapters over the
    kernel. Old defects verified absent by grep-based regression tests: no `rclpy.shutdown()`
    in any node (#3), no state on `db` (#2), no bare `except: pass` (#23).
  - `isaac_core.sim` — the **pure** layer system: `LayerManifest` (+`Binding` requiring
    exactly one of `config`/`resolve`), manifest discovery with duplicate-id detection,
    `StageCapability`/`StageInspector` Protocol + `FakeStageInspector`, and `plan_features`
    producing an enabled/skipped plan with human-readable reasons plus a startup report.
    No `omni`/`carb`/`pxr` anywhere — the Isaac-backed runtime satisfying `StageInspector`
    comes later.
  - import-linter widened to **12 contracts**; `test_kernel_purity.py` now covers all six
    kernel packages.

  **Verified environment facts** (previously unknown, now confirmed):
  - Isaac Sim **6.0.1-rc.7** at **`/home/ofer/isaacsim`**. `$ISAACSIM_PATH` is **stale** —
    it still points at `isaac_sim-2023.1.1`. `ISAACSIM`/`ISAACSIM_PYTHON` are shell *aliases*,
    so they never reach a non-interactive shell. **Do not trust `$ISAACSIM_PATH`.**
  - Isaac's bundled Python is **3.12.13**; the team's system Python is 3.10.12. The kernel
    imports cleanly on both — verified, and now part of the routine check.
  - Kit **110.1.2**. Verified-present extension deps: `omni.graph` 1.142.5,
    `isaacsim.ros2.bridge` 5.1.2, `isaacsim.core.nodes`, `omni.replicator.core` 1.13.27,
    `omni.syntheticdata` 0.6.15. Cesium **0.29.0+110** present.
  - Isaac 6 `quatd[4]`/`quatf[4]` component order confirmed as **IJKR `[x,y,z,w]`** from
    three independent `.ogn` files in the real install — so the old repo's
    `[qx,qy,qz,qw]` write was right, and it is now isolated behind one named helper.

  **Three things I caught reviewing the agents' work** (all real, none reported by them):
  1. The integrator "fixed" mypy-vs-extensions by adding `extensions/.*` to the
     **top-level** `exclude:` in `.pre-commit-config.yaml`, which silently disabled **every**
     hook on `extensions/` — ruff reported "no files to check" — while its own comment claimed
     "ruff is fine". KIRO.md explicitly warns against exactly this. Replaced with a
     **per-hook `exclude: ^extensions/` on the mypy hook only**; ruff and all hygiene hooks
     now run on extensions again. Verified both directions.
  2. No `data/` directory or `package.icon` in any extension (Ofer spotted this by eye, which
     means the contract tests had a hole). Copied the four icons from the 2023 repo, added
     `icon = "data/*.png"` to each `extension.toml`, and added
     `tests/unit/extensions/test_extension_packaging.py` (27 tests) covering icon presence,
     resolution, PNG validity, readme, and required `[package]` fields.
  3. `sensors_pic.png` was 3000×2250 / 2552 kB. `check-added-large-files` **only inspects
     staged files**, so it passed every local run and would have blocked the first
     `git add`. Downscaled to 512×384 / 208 kB (original still in the 2023 repo). The new test
     asserts no icon exceeds 500 kB *specifically because* the hook cannot catch it while
     untracked.

  **Verified:** 616/616 pytest, 12/12 pre-commit hooks PASS, import-linter 12 contracts KEPT,
  kernel imports OK under Isaac's Python 3.12.

### Sub-agent notes

Parallelising by package, then by individual extension, both worked well. What makes it work:
every agent reads this file first; the non-obvious house style is repeated inline (D213
second-line summaries, flat test functions, `#` comments not attribute docstrings); agents are
forbidden from touching `pyproject.toml` / `KIRO.md` / `requirements*.txt` / `.pre-commit-config.yaml`
so there are no write conflicts; and they must *report* needed dependencies rather than install
them. Give each agent its own test file to avoid collisions on a shared one.

**Always re-verify independently afterwards.** Across two rounds the agents' reports were
honest about what they did, but five genuine defects surfaced only from reading the code and
config myself — including one (the top-level exclude) that silently disabled linting on a whole
directory while the report claimed the opposite. Reports describe intent; only running the
commands and reading the diff establishes fact.

- **2026-08-24 (g)** — Parallel tracks for `control`, `cli`+`install`+`scripts`, and
  `devkit`, then integration.
  - `isaac_core.control` — JSON-RPC over a local socket (D4): newline-delimited JSON,
    registered (not hardcoded) handlers so `sim` can supply Isaac-backed implementations
    without this package importing Isaac, loopback-by-default with a mandatory token off-host,
    and path confinement under `output_root`. `wait_until_ready()` **replaces the old
    log-grep-plus-`sleep(5)` readiness hack** (defect #10) — the port accepting a connection
    *is* the signal. Deviation from §7 worth noting: control lives in its own package rather
    than inside `sim`, because the client must be importable on a laptop with no Isaac while
    the server runs inside Isaac's interpreter.
  - `isaac_core.cli` + `isaac_core.install` + `scripts/` — `isaac-core run|doctor|config
    dump|config explain`, `IsaacInstall` resolution, and `setup.sh` / `link_extensions.sh`.
  - `isaac_core.devkit` — `Sim.attach()` needing **no filesystem knowledge at all** and
    returning the same `SimSession` type as `launch()` (fixes defect #7, the old
    `core_path=` requirement), `UdpPoseTransport` + `pace()` supplying the I/O that
    `vehicle` deliberately lacks, and ONE generic `TopicRecorder` replacing the four ~95%
    copy-paste capture classes (defect #8). `rclpy`/`cv2` imports are lazy so the module
    imports — and the suite runs — on a machine with no ROS. `launch()` deliberately raises
    `NotImplementedError` until the Isaac-side runtime exists rather than faking it.
  - import-linter widened to **18 contracts**; kernel purity test now covers 9 packages.

  **Two things I overrode after review** — the agent reported both as "correct behavior":
  1. `IsaacInstall.locate()` resolved to the **stale 2023.1.1** install because
     `$ISAACSIM_PATH` passes every *structural* check (python.sh, isaac-sim.sh, VERSION all
     present) and structural validity was allowed to win. On this machine that silently
     pointed every downstream path — bundled python, extsUser, extension linking — at a
     version where our extensions cannot load at all. Fixed: a candidate must be structurally
     valid **AND** a supported version to win outright; an unsupported one is kept only as a
     last-resort fallback. An explicit path still wins unconditionally, since that is an
     instruction rather than a guess. Also replaced a hardcoded `/home/ofer/isaacsim` probe
     candidate with home-relative resolution — it would have silently failed for every other
     team member. Added `tests/unit/test_install_resolution.py` pinning all of it.
  2. `doctor` printed "All checks passed." while displaying four warnings, which trains
     people to ignore the output. Now reports the warning count; exit code stays 0.

  `doctor` now correctly finds Isaac Sim 6.0.1 and flags the two real outstanding setup
  steps (install into Isaac's interpreter, link extensions).

  Also wrote **`docs/usd_build_sheet.md`** — staged instructions for authoring the USD in the
  GUI, with exact prim paths, node lists and wiring. Stage 1 (base scene + UDP camera layer)
  is sufficient for a first end-to-end run.

  **Verified:** 789/789 pytest, 14/14 pre-commit hooks PASS, import-linter 18 contracts KEPT,
  kernel imports OK on Isaac's Python 3.12.

- **2026-08-24 (h)** — **Major architectural finding, discovered by Ofer launching the GUI.**
  Our extensions failed to load with
  `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`.

  Root cause, verified on disk: ROS 2 Humble ships
  `_rclpy_pybind11.cpython-**310**-x86_64-linux-gnu.so` while Isaac Sim 6 bundles Python
  **3.12.13**. C extension ABIs do not cross Python minor versions, so **`rclpy` can never
  be imported inside Isaac Sim 6.** There is no import-order or configuration fix. The old
  repo got away with rclpy in its nodes only because Isaac Sim 2023.1.1 also bundled 3.10.

  Isaac Sim 6's own `isaacsim.ros2.bridge` has **zero rclpy imports** — it is C++, and
  therefore indifferent to the Python ABI. It ships generic `ROS2Publisher` /
  `ROS2Subscriber` nodes parameterised by `messagePackage`/`messageSubfolder`/`messageName`
  that expose message fields as dynamic attributes.

  **Resolution — five nodes deleted, all replaced by Isaac's C++ equivalents:**

  | Removed (ours) | Use instead |
  |---|---|
  | `position.Ros2ToGlobalPosition` | 2× `isaacsim.ros2.bridge.ROS2Subscriber` (NavSatFix, PoseStamped) |
  | `sensors.Ros2GlobalPosePublisher` | `ROS2Publisher` (geographic_msgs/GeoPoseStamped) |
  | `sensors.Ros2RangePublisher` | `ROS2Publisher` (sensor_msgs/Range) |
  | `sensors.Ros2Gimbal` | `ROS2Subscriber` (our Gimbal message) |
  | `sensors.Ros2ImagePublisher` | `ROS2CameraHelper` / `ROS2PublishImage` |

  The whole `isaac_core_ogn.sensors` extension is gone. **What remains is exactly two real
  nodes** — `math.GlobalPositionToLocalPosition` and `position.UdpToGlobalPosition` — plus
  the template, both pure computation delegating to `isaac_core.geo` / `.protocol`.

  This is a *better* architecture, arrived at by force: it makes the nodes the thin compute
  adapters they were always meant to be, and deletes an entire class of old-repo defects
  (rclpy lifetime, per-node executors, spin threads inside nodes) by removing the code
  rather than fixing it. It also vindicates the devkit/sidecar split — anything needing
  rclpy runs out-of-process on system Python 3.10 where Humble works fine.

  **Division of labour, now fixed:**
  - Inside Isaac (3.12): pure computation only, no ROS.
  - Isaac's C++ bridge: all ROS 2 publish/subscribe, wired in the graph.
  - Host / devkit / sidecar (3.10): rclpy freely.

  Added `docs/ros2_and_python.md` (the full explanation and the replacement mapping) and
  `tests/unit/extensions/test_no_python_ros2.py`, which fails if any extension module
  imports rclpy or any ROS message package. **I verified that guard bites** by injecting
  `import rclpy` into a node and watching it fail, then restoring. That guard matters
  because this failure only appears once an extension is linked into a real install and
  the GUI is launched — expensive to rediscover.

  Also corrected `docs/usd_build_sheet.md`: Stage 1's `PoseSync` graph drops to five nodes
  and **needs no ROS at all**, so it is still the right starting point.

  **Verified:** 709/709 pytest, 14/14 pre-commit hooks PASS, import-linter 18 contracts KEPT.

  ⚠ **Action for Ofer:** `/home/ofer/isaacsim/extsUser/isaac_core_ogn.sensors` is now a
  dangling symlink. Re-run `scripts/link_extensions.sh` (or delete that one symlink) before
  relaunching, or Kit will keep complaining about a missing extension.

- **2026-08-25** — Ofer authored the USD (Stages 1–3). Reviewed all three files.

  **Quality is good.** 32–36 KB per camera layer versus 2.9 MB in the old repo, because
  the embedded `OmniverseKitViewportCameraMesh` gizmo is absent. `/Root/Xform` in both
  camera layers has `!resetXformStack!, translate, orient, scale` with a Cesium Globe
  Anchor — correct. He also added a frame-id counter (`counter` → `to_string`) into the
  `CameraImageExport` graph, which the sheet had not asked for and is a good addition.

  **Gap he identified, and he was right about the cause.** Isaac's ROS 2 bridge exposes a
  quaternion as **four separate `double` attributes** (`pose:orientation:w/x/y/z`), while
  `GlobalPositionToLocalPosition` speaks `global_orientation` as `vectord[3]` radians.
  Nothing in the bridge or OmniGraph's stock nodes closes that gap. Added two thin
  adapters over the already-tested `isaac_core.geo`:
  - `isaac_core_ogn.math.QuaternionToEuler` — `qw/qx/qy/qz` → `euler_r` (vectord[3]) plus
    scalar `roll_r`/`pitch_r`/`yaw_r`. `qw` defaults to 1.0 and an all-zero quaternion is
    treated as identity, because a subscriber legitimately reports zeros before its first
    message and NaN must not reach the pose pipeline.
  - `isaac_core_ogn.math.EulerToQuaternion` — `euler_r` → `qw/qx/qy/qz` plus a `quatd[4]`
    in Isaac's IJKR order.

  Node set is now five: the two above, `GlobalPositionToLocalPosition`,
  `UdpToGlobalPosition`, and the template.

  **Bug found in `camera_ros.usda`:** two connections still reference
  `udp_to_global_position`, a node that exists only in the UDP layer — a Save-As leftover.
  `inputs:global_orientation` points at it (which is exactly why that input reads as
  unconnected), and `inputs:global_position` lists it *alongside* the correct
  `make_3_vector.outputs:tuple`, leaving an ambiguous two-source input. Ofer to remove
  both in the GUI; documented as Fix 3 in the build sheet.

  **Open decision for Ofer — `GeoPoseStamped` orientation.** The old repo stuffed roll,
  pitch and yaw into the quaternion's `x`/`y`/`z` with `w = 1.0`, so the field never held a
  quaternion; its README documented the abuse. Wiring `EulerToQuaternion` publishes a real
  quaternion, which is what the message type means, but breaks anything downstream parsing
  the old convention. Both paths are documented in the build sheet; awaiting his choice
  before making one the default.

  Also worth noting: `earth.usda` ships `cesium:url = http://127.0.0.1:8088/nablus/...`.
  Harmless as a saved default since composition rewrites it, but `[cesium]
  tileset_server_url` has **no default** in config, so it must be set for a real run.

  Stage 4 (sensor layers) deliberately not yet specified — Ofer reported the instructions
  were unclear, and the 6.0 sensor APIs shifted enough that they should be written against
  a stage that has actually launched.

  **Verified:** 742/742 pytest, all pre-commit hooks PASS, import-linter 18 contracts KEPT.

- **2026-08-25 (b)** — Ofer applied all three USD fixes; validated clean. Added
  `tests/unit/test_usd_layers.py` (19 tests) as a permanent validator: dangling
  connection references, two-source inputs, writers targeting `/Root/Xform`, the required
  `orient` op, Cesium Globe Anchor, no embedded gizmo mesh, layer size, and D16's
  real-quaternion wiring. Hand-authored USD has failure modes nothing else here catches,
  and both of the ones already hit (Save-As leftovers, ambiguous input) were silent.

  **Frame-id investigation — the API is right, our usage was wrong.**
  `OgnROS2CameraHelper.compute` opens with `if state.initialized: return True`, so
  `frameId` is read once into the Replicator writer's `init_params` and never again. The
  node's *output attribute* keeps incrementing (hence the GUI showing it climb) while the
  writer keeps its original copy; stopping calls `custom_reset()`, so replay re-bakes and
  re-freezes. Exactly the reported symptom. `ROS2PublishImage` does take `frameId`
  per-tick but is not hand-wirable — Isaac drives it as a Replicator writer attached to a
  render product, and no `.ogn` in the install outputs the `dataPtr` it needs.

  The deeper finding (D17): ROS 2's `std_msgs/Header` has **only** `stamp` and `frame_id`.
  ROS 1's `seq` was deliberately removed, and `frame_id` is documented as "Transform frame
  with which this data is associated" — a TF frame name. The old repo used it as a frame
  counter, which is *why* its image path needed a custom rclpy republisher
  (`raw_rgb` → `image_rgb`); the earlier catalogue of that pipeline called it "rate
  limiting" and under-documented the real purpose.

  **Ofer still wants a monotonic counter, deferred — and he raised a valid objection to
  the sidecar idea:** a downstream re-stamper counts what *it* received, so it structurally
  cannot detect drops upstream of itself, which is the main reason to want the counter.
  He is right. When we return to this, the promising option is a **custom C++ OmniGraph
  node** (C++ is immune to the Python ABI wall that killed the rclpy nodes) or accepting
  `header.stamp`, which the writer does set per frame. Do not propose the sidecar again
  without addressing the drop-detection objection.

  Added `scripts/send_test_pose.py` — a system-Python sender with `hold`/`orbit`/`path`
  modes, driving `vehicle` → `protocol` → `devkit.transport`, so a moving camera also
  proves that slice of the kernel. **Verified end to end**: spawned it against a loopback
  socket, received 51-byte packets, decoded to the expected LLA/RPY with the NED tag.

  Added `docs/first_run.md`. Two prerequisites found while writing it:
  - Neither camera layer sets a **`defaultPrim`**. A USD reference with no default prim
    pulls in *nothing*, silently. Ofer must set `/Root` as default prim on both.
  - The `.ogn` defaults (`udp_port=33333`, `enu_reference=[32.22481, 35.25621, 516.7]`,
    `rotation_frame=body`) already agree with `earth.usda`'s Cesium georeference origin,
    so the first run needs no node configuration at all.

  **Verified:** 761/761 pytest, 14/14 pre-commit hooks PASS, import-linter 18 contracts KEPT.

- **2026-08-25 (c)** — Kit 110 deprecation warning on every compute:
  `'internal_state' has been deprecated: use 'per_instance_state' or 'shared_state'`.
  Inherited from the 2023 node idiom.

  Authoritative mapping, read from `omni/graph/core/_impl/database.py` in
  omni.graph 1.142.5 (and confirmed against `config/python_api.md`, which marks the old
  ones `[deprecated]`):

  | Deprecated | Current |
  |---|---|
  | `db.internal_state` | `db.per_instance_state` |
  | `Db.per_node_internal_state(node)` | `Db.per_instance_internal_state(node)` |

  **The static `internal_state()` factory on the node class is NOT deprecated** — it is
  the OGN hook that constructs the state object, and Isaac's own `OgnROS2CameraHelper`
  still declares it. Only the accessors changed. Do not "fix" the factory; removing it
  leaves `per_instance_state` empty, which fails at runtime rather than at lint time.

  Updated all three stateful nodes. Added
  `tests/unit/extensions/test_no_deprecated_ogn_api.py` (5 tests) guarding both accessors,
  asserting stateful nodes still declare the factory, and that `release()` uses the
  classmethod form. **Verified the guard bites** by reintroducing `db.internal_state` and
  watching it fail, then restoring.

  Note for the record: this was a `[Warning]`, not an error. The deprecation shim still
  forwards to the new accessor, so the node computed correctly — it is log noise that
  buries real errors, not a functional failure. Worth checking whether anything else
  actually blocked the first run.

  **Verified:** 766/766 pytest, all pre-commit hooks PASS.

- **2026-08-25 (d)** — Camera flew ~9,700 km off. **Root cause was my own bad instruction.**

  Observed `/World/Environment/drone_0/Xform` translate:
  `(-3124516.73, 5473726.35, -7417105.45)` — ECEF-scale, so no reference subtraction had
  happened.

  Diagnosis by reproduction rather than inspection: fed the exact sent pose through
  `isaac_core.geo.EnuConverter` with the `.ogn` default reference and got
  `(0, 0, 483.30)` — correct (same lat/lon, 1000 − 516.7 m up). None of the failure
  hypotheses (zero reference, swapped lat/lon, raw ECEF) reproduced the observation, which
  ruled our node out. Then computed **Denver, Colorado expressed in ENU about the Israeli
  georeference**: `(-3124516.73, 5473724.52, -7417106.79)` — the observed value to within
  2 m. Conclusive.

  Chain:
  1. `docs/usd_build_sheet.md` told Ofer to add a **Cesium Globe Anchor** to `/Root/Xform`.
     That instruction was wrong.
  2. The camera layer was authored standalone, and its own `/CesiumGeoreference` has **no
     `georeferenceOrigin` authored** — so Cesium fell back to its default origin, **Denver
     Colorado** (39.7364, −105.2574, 2733 m).
  3. The Globe Anchor therefore recorded the Xform as being *in Denver*
     (`cesium:anchor:latitude = 39.7364...`).
  4. Standalone it looked fine, because anchor origin and georeference origin agreed.
  5. Composed under `earth.usda`, `cesium:anchor:georeferenceBinding = </CesiumGeoreference>`
     resolves to the *scene's* georeference (Israel), so Cesium computed where Denver is in
     an Israel-centred ENU frame.

  **A Globe Anchor does not belong on the camera Xform.** The graph already writes local
  ENU into `xformOp:translate` every tick, and the scene georeference already maps stage
  origin to a lat/lon, so a local translate is a complete description. An anchor is a
  second writer fighting the first. Confirmed against the old repo: `cesium:anchor` appears
  **zero** times in both of its camera layers — it anchored only the *bbox target objects*,
  which genuinely need pinning to a fixed geographic spot.

  Corrected the build sheet with the full explanation. **Inverted
  `test_moved_prim_has_no_cesium_globe_anchor`** — it previously asserted the anchor must be
  *present*, encoding my wrong belief, which is a good reminder that a test only pins a
  belief and can be confidently wrong. Added
  `test_layer_georeference_origin_is_authored`, since an unauthored origin silently means
  Denver.

  Those 4 tests are **deliberately left failing**: they report a live defect in files only
  Ofer can edit (D15). They go green once the anchors are removed and origins authored. Not
  marked xfail — that would hide a real bug.

  Rest of the suite: 747 passing.

- **2026-08-25 (e)** — **First real launches of `isaac-core run`.** Wired `run` to actually
  spawn `<isaac_python> -m isaac_core.sim`, forwarding the *fully resolved* config via a
  temp TOML so env/CLI/file precedence is understood in exactly one place. Added
  `--dry-run`.

  **Verified live, from a separate process against a running simulator:**
  ```
  ping             -> pong
  get_state        -> {'running': True, 'scene': 'earth', 'headless': True}
  get_capabilities -> {'enabled': ['camera_udp'], 'skipped': []}
  ```
  So: launch, runtime startup, stage composition, and JSON-RPC control over a socket all
  work. Six extensions enable cleanly (`omni.graph.action`, `omni.graph.nodes`,
  `isaacsim.ros2.bridge`, `isaacsim.core.nodes`, and both of ours), and the
  "Could not find node type interface" warnings dropped to **zero**.

  **Six real bugs found by launching, none catchable statically:**
  1. `runtime.run()` used `importlib.import_module("omni.isaac.core")` and
     `SimulationContext`. **Neither exists in Isaac Sim 6** — no `isaacsim.core.api`
     extension, no `SimulationContext` class anywhere in the install. The Core API was
     replaced by `isaacsim.core.experimental.*`. Correct Isaac 6 idiom, taken from
     NVIDIA's own standalone examples: `isaacsim.core.experimental.utils.app` for
     `play`/`pause`/`stop`/`update_app`, and
     `isaacsim.core.simulation_manager.SimulationManager` for `setup_simulation`/
     `initialize_physics`. Because these were lazy import *strings*, ruff, mypy and the
     purity test were all blind. Guarded by `tests/unit/test_no_dead_isaac_api.py`, which
     uses **AST** (text matching false-positived on the docstrings that deliberately
     explain the rename) and includes tests that the guard itself detects each dead form.
  2. The step loop gated on `is_playing()`, inherited from 2023, so pausing terminated the
     process. Now loops while the *application* runs, leaving pause/resume useful.
  3. `features.enabled` was empty by default, so nothing composed. Added
     `IsaacCoreConfig.required_feature_ids()`: the union of explicit features and the
     camera layer implied by each vehicle's `pose_source`. Listing `camera_udp` *and*
     setting `pose_source = "udp"` was a duplicate source of truth that could disagree —
     the same class of problem D18 removed for the georeference. A minimal config now
     composes and flies with nothing under `[features]`.
  4. Both camera manifests declared `requires = ["CAMERA"]`, so they were always skipped
     with "unmet stage requirement: CAMERA". **My spec error** — a camera layer *provides*
     the camera; `earth.usda` deliberately has none. Now `requires = []`,
     `provides = ["CAMERA"]`.
  5. Both manifests had `mount = "{mount}"`, which is circular — that field *defines* what
     `{mount}` means. It passed validation and then crashed at compose with a bare
     `KeyError: 'mount'`. Fixed to `/Environment/{instance}`, and `LayerManifest` now
     rejects a self-referential mount at load with an explanatory message.
  6. `composer.mount_path_for_layer` hardcoded `instance="default"`, mounting every layer
     at `/Environment/default` regardless of vehicle — so no binding's prim path matched,
     and a swarm would have collapsed onto one mount. Instance now threaded from the
     vehicle id.

  **Six agent tests encoded the wrong expectation** and had to be inverted: two asserting
  `requires CAMERA`, two asserting the circular `{mount}`, two asserting `udp_port` was a
  `config` rather than `resolve` binding. Worth remembering that a passing test only pins
  a belief.

  Also: removed the dangling `isaac_core_ogn.sensors` symlink from `extsUser`.

  **Verified:** 936/936 pytest, all pre-commit hooks PASS, import-linter 21 contracts KEPT.

### Open at end of session

- The simulator **exits shortly after composition** without reaching the `simulation
  running` log and without printing a traceback. Extensions enable, the layer mounts, the
  report prints, then `Simulation App Shutting Down`. Suspicion is `SimulationManager
  .setup_simulation()` / `initialize_physics()` raising and the exception being swallowed
  on the way out of the context manager — next step is to stop swallowing it and capture
  stderr separately. An *earlier* run (before extension enabling was added) did reach
  `simulation running` and stayed up for a full 150 s, so the regression is in what was
  added after that point.
- `isaacsim.ros2.bridge` logs `Could not import rclpy` on startup, exactly as
  docs/ros2_and_python.md predicts. It suggests setting `ROS_DISTRO`,
  `RMW_IMPLEMENTATION` and `LD_LIBRARY_PATH` to use its *internal* ROS libraries. Worth
  trying — it may let the C++ bridge publish without any system rclpy at all.
- **Do not use `pkill -f` / `pgrep -f` for the simulator**: the pattern matches the
  agent's own shell command line and kills the session. Enumerate `/proc`, match on the
  Isaac-bundled interpreter path, and exclude own PID and PPID.

- **2026-08-25 (f)** — **FULL END-TO-END VERIFIED, and the segfault is solved.**

  `isaac-core run` launches Isaac Sim 6, composes the stage, runs stably, and a UDP
  packet moves the camera. Measured from outside the process via the control plane:

  | Sent | Prim translate (up) | Expected (alt − 516.7 ENU ref) |
  |---|---|---|
  | nothing | `0.0` | — |
  | `alt=1000` | `483.2999999998` | **483.3** ✓ |
  | `alt=1500` | `983.2999999991` | **983.3** ✓ |

  So the entire chain is proven numerically exact: `scripts/send_test_pose.py` →
  UDP → `UdpToGlobalPosition` → `ned_to_enu` → `GlobalPositionToLocalPosition`
  (ECEF→ENU via pyproj) → `WritePrimAttribute` → `/World/Environment/drone_0/Xform`.
  Runs for 200 s with **zero crash markers**.

  **The segfault root cause** (hardest bug so far; found by a five-way extension bisect):
  `extensions/isaac_core_ogn.position/config/extension.toml` still declared
  `"isaacsim.ros2.bridge" = {}`, left over from the `Ros2ToGlobalPosition` node deleted
  when Python 3.12 made rclpy unimportable. Enabling our position extension therefore
  *transitively* enabled the ROS 2 bridge, which **segfaults this Isaac Sim 6.0.1 install
  during stage open — even with ROS 2 Humble fully sourced** (`ROS_DISTRO`,
  `AMENT_PREFIX_PATH`, `RMW_IMPLEMENTATION` all set, and `LD_LIBRARY_PATH` pointed at
  `/opt/ros/humble/lib` made no difference). Removing the stale dependency fixed it.

  Bisect method worth reusing: a minimal script taking a comma-separated extension list,
  run once per combination. `math` alone passed, `position` alone crashed, which localised
  it immediately. Guessing from stack traces got nowhere — the frames were all in
  `libomni.timeline.plugin.so` with no symbols.

  **Five further bugs fixed on the way there:**
  1. `_resolve_bindings` did not substitute `{instance}` into *config keys*, only into prim
     paths, so every binding raised `ConfigKeyError: vehicles.{instance} does not exist`.
     "Resolved" now means resolved on every axis.
  2. Writes passed plain Python lists to typed USD attributes →
     `Type mismatch: expected 'GfVec3d', got 'vector<VtValue>'`. Added
     `_coerce_for_attribute` in the composer, keyed on the attribute's USD type name. The
     configurator stays pure (plain Python) and the Isaac-coupled writer converts, which
     preserves the pure/impure split.
  3. `ENVIRONMENT_ROOT` was `/Environment`, carried over from the 2023 repo whose scenes
     rooted there. The authored scenes use `/World/Environment`, so layers were mounted as
     a **sibling of `/World`**, outside the scene graph. Now `/World/Environment`.
  4. `features.enabled` was empty by default so nothing composed. Added
     `required_feature_ids()`, deriving each vehicle's camera layer from its `pose_source`
     — same duplicate-source-of-truth removal as D18.
  5. The control plane could not be disabled (`start()` asserted the server existed).
     Added `sim.control_plane.enabled`, which also made the crash bisection possible.

  **New capability:** `get_pose` control method, returning the live translate/orient of a
  vehicle's moved prim. This is what made external verification possible at all — nothing
  else exposes what the graph actually wrote, and checking the viewport by eye does not
  scale. Note `Gf.Quatd` is **not iterable** (it has `GetReal`/`GetImaginary`), so the
  value conversion handles vectors and quaternions separately.

  **`sim.extensions` is now config-driven**, and `isaacsim.ros2.bridge` is deliberately
  **not** in the default list. Without it, ROS nodes in a stage log "Could not find node
  type interface" and do nothing — a survivable degradation instead of a crash. The UDP
  path needs no ROS at all.

  Also cleared stale `~/.cache/ov/ogn_generated/isaac_core_ogn.*` caches, which still held
  generated databases for deleted nodes. Not the crash cause, but correct hygiene:
  deleting a node leaves its generated DB behind.

  **Verified:** 937/937 pytest, all pre-commit hooks PASS, import-linter 21 contracts KEPT,
  live simulator stable with a numerically exact pose pipeline.

- **2026-08-25 (g)** — Cleared the bridge, found the real ROS blocker, and built the
  debug tools.

  **The `isaacsim.ros2.bridge` segfault was misattributed — the bridge is fine.** Ran a
  three-way test (`/tmp/ros1.py`) enabling the bridge on an empty stage, on `earth.usda`,
  and on a stage with the camera layer and its ROS nodes composed. All three survived.
  The full runtime then ran 200s with the bridge in `REQUIRED_EXTENSIONS`: **zero crash
  markers and zero "Could not find node type interface"**, so every ROS node resolves.

  The actual cause of the earlier crash was the **stale generated OmniGraph database** in
  `~/.cache/ov/ogn_generated/`, left behind by the deleted `Ros2ToGlobalPosition` node —
  which I had dismissed at the time as "not the crash cause, but correct hygiene". That
  was wrong, and the lesson is worth keeping: removing an OGN node leaves a generated DB
  that Isaac still registers, and it can take the process down. `link_extensions.sh` now
  clears those caches, and the mechanism is documented in the README and build sheet.

  **A residual intermittent crash exists** inside Kit's `update_app()` on the first frame
  (py-spy shows the control-server thread idle in `select`, so it is not a Python race).
  It reproduced once in roughly six launches; two consecutive 150s runs afterwards were
  clean. Not yet root-caused. It predates this session's changes.

  **The real reason ROS publishing does not work: `ros2_publisher.execIn` is unconnected**
  in both camera layers, so the node never computes. Confirmed live — `ros2 topic list`
  showed no `isaac_core` topics and `ros2 topic echo` reported the topic
  "does not appear to be published yet". An unconnected `execIn` is the quietest failure
  mode in OmniGraph: the node exists, its inputs are set, the graph loads without a
  warning, and it simply never runs. Added `tests/unit/test_usd_graph_wiring.py`, which
  parses every layer and fails on any node declaring an `execIn` that nothing drives
  (`ROS2Context` and `ROS2QoSProfile` are exempt as data-only providers). It currently
  fails for both camera layers, and is task 1 in the build sheet for Ofer.

  Also learned that **rclpy and the `ros2` CLI work fine on the host's Python 3.10** — the
  ABI wall is only inside Isaac's 3.12. That makes `ros2 topic echo` a usable verification
  tool, which is how the unwired publisher was found.

  **Fixed a real thread-safety defect.** `get_pose` read USD, and `step` called
  `update_app()`, directly from the control server's thread. This appeared to work on an
  idle stage and then timed out as soon as pose packets were arriving and OmniGraph was
  writing transforms. Added a main-thread task queue: handlers submit a callable via
  `_on_main_thread` and the step loop drains it each frame, with results and exceptions
  handed back to the caller and a bounded `MAIN_THREAD_TASK_TIMEOUT_S` so a wedged loop
  surfaces as an error rather than a hang. Runs inline when already on the loop thread,
  which avoids deadlocking. Nine tests cover it without needing Isaac.

  **Built `isaac_core/debug/`**, replacing 2023's `debugger/`:
  - `pose_sender_gui.py` — tkinter GUI. All state in `PoseSenderController`, which has no
    tkinter import and is fully unit-tested; the widgets are a thin view. Fixes the old
    GUI's degrees/radians lie (the on-screen table said degrees while the wire carried
    radians), derives the packet table from `contracts.packet` so it cannot drift, and
    delegates all wire work to `protocol` and `devkit.transport` — no struct or checksum
    code. Drift-free pacing via an accumulating `perf_counter` deadline.
  - `inspector.py` — terminal tool printing state, capabilities, live pose and config,
    with `--poll` to watch the pose change. Reading the live prim transform from outside
    the process is the only external proof that pose input reaches the camera.

  `main()` initially ignored `argv` and called `launch_gui()` unconditionally, so `--help`
  opened a window and blocked forever; it hung the session twice before I noticed. Both
  tools now parse arguments before touching tkinter, and `--check` validates without
  opening a window. Regression-tested. **tkinter turned out to already be installed**,
  contrary to my earlier assumption — but the modules still import without it, and a
  purity test proves it.

  **Verified:** 980/980 pytest excluding the two intentional USD-wiring failures, all
  pre-commit hooks PASS, import-linter **23** contracts KEPT (added "Debug does not import
  sim" and "Nothing depends on debug"), Python 3.12 imports OK, three console scripts
  resolve. Live re-verification through the new inspector, with the dispatcher in place:

  | Sent | Prim translate (up) | Expected |
  |---|---|---|
  | alt=1500 | `983.300` | **983.3** ✓ |
  | alt=1000 | `483.300` | **483.3** ✓ |

  Also replaced the template `README.md` and corrected its now-false claim that the bridge
  segfaults.

- **2026-08-26 (a)** — ROS publishing verified working end to end.

  Ofer wired `ros2_publisher.execIn` in both camera layers. `test_usd_graph_wiring.py`
  went green and **both ROS topics now publish**:

  | Topic | Type | Rate |
  |---|---|---|
  | `/isaac_core/global_pose` | `geographic_msgs/msg/GeoPoseStamped` | ~100 Hz |
  | `/isaac_core/image_rgb` | `sensor_msgs/msg/Image` | ~115 Hz |

  `ros2 topic echo` returned `latitude: 32.3, longitude: 35.3, altitude: 1500.0` — exactly
  the values sent over UDP — with a real non-identity quaternion. That closes the last
  unverified link in the chain.

  Two diagnostic lessons from getting there. First, **ROS topics take several seconds to
  appear** after the control plane answers: the publisher needs a couple of graph ticks
  plus DDS discovery. My first check was ~8s in and saw nothing, which looks identical to a
  broken publisher and sent me chasing message-type resolution. Second, when the topic was
  still missing I proved the graph was fine rather than guessing, by probing the live graph
  through `omni.graph.core`: `compute_count=60`, `messageSubfolder` correctly resolving to
  `'msg'` from the `.ogn` default despite being authored with no value, and a valid non-zero
  context handle. Reading `OgnROS2Publisher.cpp` also confirmed it is **C++**, so the
  `Could not import rclpy` warnings the bridge emits are irrelevant to publishing. That
  probe technique is worth reusing: `og.get_node_by_path(...)`, then `get_compute_count()`
  and `og.Controller.get(node.get_attribute(...))`.

  **New gap found: the pose message has no timestamp.** `header:stamp:sec` and
  `header:stamp:nanosec` are unconnected, so it publishes `sec: 0, nanosec: 0`. Closing it
  is not just a wire — `IsaacReadSimulationTime` outputs a **double seconds** value while
  the publisher wants separate **int sec** and **uint nanosec**. Build sheet task 1 offers
  Ofer two options: a four-node math chain in OmniGraph, or one node in
  `isaac_core_ogn.math` (recommended — two connections instead of five nodes, and the
  conversion becomes unit-testable). Awaiting his choice.

  Extended `test_usd_graph_wiring.py` with a timestamp guard, marked `xfail(strict=True)`
  so it flips to a failure the moment the stamp is wired, which is the prompt to delete the
  marker. Also audited `~/isaacsim` at Ofer's request and confirmed **no Isaac source,
  extension, config, `.kit`, `.ogn` or app file was modified**; the only recently-touched
  non-pip file, `apps/isaacsim.exp.full.kit`, is dated 2026-08-16, eight days before this
  work began.

  **Verified:** 982 pass + 2 documented xfail, all pre-commit hooks PASS, import-linter 23
  contracts KEPT.

- **2026-08-26 (b)** — Built `SecondsToRosStamp` to close the timestamp gap.

  Ofer chose option B, one node over a four-node OmniGraph math chain. Added
  `isaac_core.contracts.stamp.seconds_to_ros_stamp`, pure and dependency-free, plus the
  `isaac_core_ogn.math.SecondsToRosStamp` node as a thin adapter over it.

  The arithmetic has three edge cases that a hand-built math chain would likely get wrong,
  which is the main argument for putting it in one tested place:
  - **Rounding can land on exactly 1e9 nanoseconds** (1.9999999999 does), which must carry
    into `sec` or ROS sees a malformed stamp.
  - **Negative inputs need floor, not truncation.** `-0.25` is `(-1, 750000000)`; the
    truncating answer `(0, -250000000)` is unrepresentable because `nanosec` is unsigned.
  - **Non-finite input must be refused.** An unconnected double arriving as NaN should not
    become a timestamp.

  25 unit tests cover those without Isaac, and all five representative cases were then
  confirmed **in a live OmniGraph**, including the carry and the negative floor.

  Named `SecondsToRosStamp`, not `SimulationTimeToRosStamp`, for two reasons: the
  `test_ogn_key_does_not_contain_ogn_or_sim` guard rejects any node key containing "Sim"
  (a 2023-era naming rule worth keeping rather than weakening for a false positive), and
  the node genuinely does not care whether the seconds come from simulation or system time.
  Taking the value as an input rather than reading the timeline internally keeps it a thin
  adapter and leaves Isaac owning the definition of "now".

  Two live-probe lessons worth reusing. `omni.graph.action.OnTick` does **not** fire unless
  the timeline is playing or `inputs:onlyPlayback` is set false — my first compute test
  reported all-zero outputs and looked like a broken node. And a new `.ogn` needs
  `~/.cache/ov/ogn_generated/*/isaac_core_ogn.*` cleared before the type registers.

  Build sheet task 1 now carries exact GUI instructions: two nodes and four connections per
  camera layer.

  **Verified:** 1012 pass + 2 documented xfail, all pre-commit hooks PASS, import-linter 23
  contracts KEPT, node registers in Isaac as `isaac_core_ogn.math.SecondsToRosStamp`.

- **2026-08-26 (c)** — Timestamp wired and verified; the startup segfault is now the top
  open issue.

  Ofer wired `SecondsToRosStamp` into both camera layers. Verified at three levels:
  the wiring guard flipped to XPASS and the `xfail` marker is deleted; a live graph probe
  showed `seconds_to_ros_stamp`, `isaac_read_simulation_time` and `ros2_publisher` all
  computing 80/80 frames with outputs `sec=1, nanosec=366666667`; and two consecutive
  `ros2 topic echo` samples gave `sec: 2, nanosec: 450000000` then
  `sec: 12, nanosec: 16666667` -- non-zero, advancing, `nanosec` in range.

  **A correction worth recording.** Chasing a missing topic, I concluded from a bisect that
  opening `earth.usda` crashed even with no extensions enabled. That was wrong: those
  particular probe runs never started at all, dying on a
  `partially initialized module 'isaacsim'` import error rather than a segfault, and I had
  counted a non-zero exit as a crash without checking which failure it was. Separating the
  two by grepping for `Segmentation fault` versus the import message fixed the picture.
  Lesson: when measuring a crash rate, assert on the specific failure signature, never on
  the exit code alone.

  What the segfault actually is: roughly **one launch in three**, exit code 139 immediately
  after `simulation running`, main thread inside Kit's `update_app()` with the control
  server idle in `select`. Confirmed **not** ours -- it reproduces with both
  `isaac_core_ogn` extensions disabled. Not the tile server either (HTTP 200). A run that
  opens no stage survives every time, so it is tied to stage opening. Clearing the **48
  stale `/tmp/carb.*` directories** Isaac had left behind from earlier crashes measurably
  improved the rate, so crash debris feeds back into further crashes.

  Also worth remembering: `omni.graph.action.OnTick` does not fire unless the timeline is
  playing or `inputs:onlyPlayback` is false. My first node compute test read all-zero
  outputs and looked like a broken node when it was simply not playing.

  **Verified:** 1012 pass + 0 xfail, all pre-commit hooks PASS, import-linter 23 contracts
  KEPT.

### Known remaining issues

- **Intermittent startup segfault**, about one launch in three, inside Kit's
  `update_app()` just after play. Not ours -- reproduces with our extensions disabled.
  Build sheet task 1 records everything ruled out so far.
- **Intermittent segfault** inside Kit's `update_app()` on the first frame, roughly one
  launch in six, not root-caused and not obviously ours. Worth watching for a pattern
  before spending more on it.

### Pending

- **Sensor layers** — `distance_sensor`, `bbox_publisher`, `sat`. Isaac Sim 6 ships a
  native physics raycast sensor that likely replaces 2023's custom raycast script node.
  Globe Anchors genuinely belong on `bbox_publisher` targets, unlike the camera.
- **Monotonic frame id** — deferred with Ofer. `ROS2CameraHelper` bakes `frameId` once at
  writer init, so a counter cannot reach the topic. Downstream re-stamping is ruled out
  (it cannot detect upstream drops). Likely answer is a custom **C++** OmniGraph node.
- **`capture_frame`** raises `NotImplementedError` — needs Isaac's viewport capture API.
  Route it through `_on_main_thread` when implemented.
- **Eyes-on check for Ofer**: run non-headless and confirm the camera looks along the
  nose. The quaternion is numerically right; nobody has looked at it.
- **Wire `import-linter` as a pre-commit hook** (`repo: local`, `language: system`). The
  config is complete and passing; the hook was never added.


### Template findings (worth telling the team)

- The template ignores ruff `D212` but leaves **`D213` active**, so multi-line docstrings
  must put the summary on the **second** line:
  ```python
  """
  Summary here.

  Body.
  """
  ```
  The template never exercised this -- every docstring it ships is single-line -- so this is
  the first time the convention is visible. All source here follows it.
- `[tool.ruff.lint.per-file-ignores]` for `tests/**` exempts `D103` (function docstring) and
  `ANN201` (return annotation), which assumes **flat test functions**. Test *classes* trip
  `D101`/`D102`, and parametrised arguments still trip `ANN001`. Tests here are therefore
  flat functions with annotated parameters and explicit `-> None` (mypy's
  `disallow_untyped_defs` still requires the return annotation, matching the template's own
  `def test_add() -> None`).
- `check-docstring-first` rejects PEP 258 **attribute docstrings** (a bare string after a
  module-level constant), reporting "Multiple module docstrings". Document constants with
  `#` comments above the assignment instead.

### Next step

Continue in dependency order: **`config`** (pydantic schema + layered loader + the full
`config/default.toml` surface), then `geo`, `protocol`, `vehicle`. Only then `sim` +
`extensions`, then `devkit`, then `sidecar` and `debug`.

Still outstanding: `README.md` is still the template's and needs replacing; no `config/`,
`docs/`, `scripts/`, `extensions/` or `assets/` directories exist yet.
