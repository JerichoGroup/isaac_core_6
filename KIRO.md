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

### Pending

- **Widen the import-linter contracts** in `pyproject.toml` as each package lands. They
  currently cover only `contracts` and `config`, because import-linter errors on a module
  that does not exist. There is an inline NOTE in `pyproject.toml` saying exactly what to add.
- `README.md` is still the template's.
- Not yet created: `docs/`, `scripts/`, `extensions/`, `src/isaac_core/assets/`.


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
