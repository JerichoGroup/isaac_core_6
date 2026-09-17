# docs/development-log.md — isaac_core_6 working memory

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
| D20 | **ROS 2 is the data plane; the JSON-RPC control plane is the command plane** | Restated and confirmed by Ofer 2026-09-02, clarifying D4. Data *flowing* in or out of the simulation (pose in, image/pose/range/bbox out) belongs on ROS 2 topics. *Commanding* the simulation (take a picture, set gimbal angles, reset, load a scene) belongs on the control plane. Consequence: 2023's `--sat` ROS topic becomes a `capture_frame` control method, and gimbal angles become a control method rather than only a ROS subscriber. |
| D21 | Use Isaac Sim 6's **native RTSP**, not our own RTP sidecar | Ofer 2026-09-02. Wire it into the image-publisher action graph in both camera layers so it is **always on** — no flag, no sidecar process. Anyone who wants the stream consumes it; anyone who does not simply ignores it. Supersedes the `sidecar.rtp` approach. |
| D22 | **No ROS 2 rate manipulation anywhere** | Ofer 2026-09-02. No publisher or subscriber throttles or boosts its rate; that is a path to untimed-message chaos. `publish_rate_hz` / 2023's `MAX_OUTPUTS_ROS_HRZ` are therefore deliberately absent, not merely unimplemented. The one legitimately related problem is video recording: the recorder must derive the **live** frame rate from message timestamps rather than making the user guess a constant fps (Isaac runs ~30-50 fps, variable, which is why 2023's videos played at the wrong speed). |
| D23 | **Reach for what Isaac Sim already offers before building our own; carry no dead code** | Ofer 2026-09-02. Two halves of one habit. First: the instinct must be "how do I achieve this with what Isaac Sim provides" before writing a bespoke subsystem — the RTP sidecar is the cautionary example, ~340 lines superseded by a native feature. Second: unused code is deleted, not parked "just in case". Applied during development, not only at cleanup time. Concrete consequences: `sidecar/rtp.py` goes when native RTSP lands (M4); `SATOutput.msg` and `Gimbal.msg` are **not** vendored because capture and gimbal are control-plane commands (D20), so nothing would consume them; the ROS `/isaac_core/gimbal` subscriber is removed from both camera layers — anything external that wants to command the gimbal uses `Sim.attach()` and the control plane. A dead-code sweep is an explicit v2 exit criterion. |

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
- **Nothing has been implemented yet.** No source written in isaac_core_6; only `docs/development-log.md`
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
     "ruff is fine". docs/development-log.md explicitly warns against exactly this. Replaced with a
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
forbidden from touching `pyproject.toml` / `docs/development-log.md` / `requirements*.txt` / `.pre-commit-config.yaml`
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

  Also wrote **`docs/dev/usd_build_sheet.md`** — staged instructions for authoring the USD in the
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

  Also corrected `docs/dev/usd_build_sheet.md`: Stage 1's `PoseSync` graph drops to five nodes
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
  1. `docs/dev/usd_build_sheet.md` told Ofer to add a **Cesium Globe Anchor** to `/Root/Xform`.
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

- **2026-08-26 (d)** — Fixed the intermittent startup segfault. Root cause found by
  measurement, and it was the ROS 2 bridge after all.

  Built `scripts/crash_rate.sh` to make the problem measurable: N launches, each killed
  before the next, stale `/tmp/carb.*` cleared first, counting the specific
  `Segmentation fault` signature rather than a non-zero exit code. **Baseline: 5 crashes in
  10 runs.** Without a harness this was unfixable -- every "fix" looked like it worked on a
  lucky run of three.

  Two wrong turns first, both cheap because the harness disproved them quickly: waiting for
  `is_stage_loading()` to finish (4/10 -- and it made startup crawl, because Cesium streams
  tiles indefinitely so the wait never completes), and switching the renderer from
  `RealTimePathTracing` to `RaytracedLighting` (5/12). The renderer change was kept anyway
  as `sim.renderer`: this project wants a correct image, not a photoreal one.

  The stack trace was the turning point, once I looked at native frames instead of the
  Python ones: `omni.graph.core` -> `omni.graph.image.core` -> `omni.kit.exec.core` -> TBB.
  That is OmniGraph's **render-stage** execution inside Kit's parallel executor, not Cesium
  and not RTX. A controlled bisect on an **empty stage** then isolated it:

  | Extensions | Crashes |
  |---|---|
  | `omni.graph.action` + `omni.graph.nodes` | 0 / 6 |
  | + `isaacsim.core.nodes` | 0 / 6 |
  | + **`isaacsim.ros2.bridge`** | **4 / 6** |

  So yesterday's original suspicion was right and my later "the bridge is fine" conclusion
  was wrong -- that test used 30 frames where this one uses 120 plus play plus 120, and it
  simply got lucky. Worth remembering: a negative result from a short probe is not evidence.

  **The fix, entirely on our side: pump frames after enabling extensions, before any stage
  operation.** Enabling the bridge and immediately creating a stage crashed 4/6; pumping 60
  frames first crashed 0/6. The bridge needs update cycles to finish registering its
  render-stage hooks, and a stage arriving mid-way races it. Added
  `EXTENSION_WARMUP_FRAMES = 60` after `_enable_required_extensions`, and
  `STAGE_SETTLE_FRAMES = 30` via a `settle` callback the composer invokes after the scene
  opens and after layers mount.

  **Result: 0 segfaults in 27 runs**, every one reaching play, against 5 in 10 before. If
  the rate were still 0.5 the odds of 27 clean runs would be about 1 in 10^8.
  `tests/unit/sim/test_startup_warmup.py` locks the ordering in with a fake app module and
  was verified to fail when the pump is removed.

  Also implemented **`Sim.launch()`**, whose `NotImplementedError` message had gone stale
  (it claimed the Isaac-side runtime did not exist). It now resolves the install, writes a
  resolved config, spawns `python.sh -m isaac_core.sim`, and waits for the control plane;
  the returned session owns the process so `with Sim.launch(...)` cannot leak a simulator.
  While testing it I discovered the old test suite **actually launched Isaac Sim** -- its
  log leaked into pytest output. Tests now inject a fake launcher; the devkit suite went
  from 12.9s to 2.8s.

  Two sub-agents wrote `docs/dev/roadmap.md` (repo-wide status, parity table against the 2023
  repo, what remains for v1) and rewrote `README.md` with usage and worked examples. Both
  were independently checked: the roadmap listed the import-linter hook and `Sim.launch` as
  outstanding when both were done, and test counts were stale. Corrected.

  **Verified:** 1037 pass, all pre-commit hooks PASS, import-linter 23 contracts KEPT, end
  to end re-confirmed (prim at 8339.9 m north for lat 32.3; ROS message with
  `stamp: {sec: 13, nanosec: 333333333}`).

- **2026-08-26 (e)** — Fixed the GUI run: blank screen, no terrain, camera looking dead.

  Ofer ran `isaac-core run` non-headless and got a white screen with no tilesets and an
  apparently unresponsive camera. All my verification to that point had been **headless**,
  which is exactly why this slipped through: every headless check reads the prim transform
  through the control plane, and a prim transform is right whether or not anything is drawn.

  Two independent causes, both silent:

  1. **Cesium was never enabled.** The scene's terrain is 3D Tiles, needing
     `cesium.omniverse`. It is installed by the GUI extension manager into
     `~/.local/share/ov/data/exts/v2`, and Isaac's python experience does not search that
     folder -- `enable_extension("cesium.omniverse")` returned **False** with
     "No versions of cesium.omniverse that satisfies". Added `sim.extension_search_paths`,
     passed to Kit as `--ext-folder`, and put `cesium.omniverse` in the default extension
     list. Verified: "CesiumOmniverse startup" now appears in the log. A
     `CesiumTilesetPrim` without the extension is valid USD that draws nothing, so there is
     no error to find -- worth remembering as a failure shape.
  2. **The viewport kept Kit's default perspective camera**, so the aircraft camera tracked
     poses correctly while the window showed a static view. Added `sim.viewport_camera`,
     defaulting to `/World/Environment/{instance}/Xform/main_camera_01`, applied after
     composition and skipped when headless. Verified in a real GUI launch: "viewport looking
     through /World/Environment/drone_0/Xform/main_camera_01".

  The lesson for the rest of this project: **headless verification cannot see rendering.**
  Anything visual needs either a GUI launch or an image-topic check. I should have flagged
  the eyes-on item as blocking rather than nice-to-have.

  Eight tests in `tests/unit/sim/test_gui_setup.py` cover the defaults, tilde expansion,
  missing-folder tolerance, and the headless and empty-string escape hatches. Note the
  config models are **frozen** (pydantic), so tests must construct
  `IsaacCoreConfig(sim={...})` rather than mutate.

  Ofer pointed me at how the 2023 repo did this, which confirmed the approach and surfaced
  one more gap. That code called the same `enable_extension("cesium.omniverse")`; it worked
  there because the Docker image **copied Cesium into `/isaac-sim/exts/`**, an
  already-searched folder. `--ext-folder` reaches the same result without writing anything
  into the Isaac install, which matters here.

  It also had `_set_cesium_tilesets_url()`, rewriting every `cesium:url` under `/tilesets` at
  launch. Our `cesium.tileset_server_url` existed in the schema but was **never applied** --
  a silent dead config key. Now implemented as `apply_tileset_server_url()` in the composer,
  defaulting to leaving the scene alone, so the team can repoint the tile server from config
  instead of hand-editing USD in the GUI. Seven tests with fake USD objects.

  **Verified:** 1037 pass, all hooks PASS, import-linter 23 contracts KEPT, GUI launch clean
  for 150s with 0 crashes, Cesium and the viewport retarget both confirmed in that launch.

- **2026-08-26 (f)** — Terrain finally renders. The cause was USD **schema registration
  timing**, not the extension being missing.

  After (e) the camera responded but there were still no tilesets. Ofer's diagnosis is what
  cracked it: in the GUI the `Cesium_Tileset` prim showed Transform, Geometry, Visual, Kind
  and Cesium Tileset Settings; under `isaac-core run` the same prim showed only Semantics,
  Prim Custom Data, Array Properties and Raw USD Properties -- and **every** prim was losing
  most of its properties. Losing *typed* properties means the schema is not resolving, which
  points at the registry rather than at Cesium.

  Measured it directly:

  | | schema registered | attributes | `IsA(Xformable)` |
  |---|---|---|---|
  | `enable_extension("cesium.omniverse")` after boot | **False** | 5 | **False** |
  | `--enable cesium.omniverse` at Kit startup | True | 28 | True |

  So `enable_extension` **reported success and did nothing useful**: the USD schema registry
  had already initialised, so `CesiumTilesetPrim` stayed an unregistered type and the prim
  resolved untyped. A tileset that is not Xformable draws nothing, and nothing logs an error.
  Added `sim.boot_extensions`, passed to Kit as `--enable`, with `cesium.usd.plugins`
  alongside `cesium.omniverse` -- the schemas live in that separate extension, which
  `cesium.omniverse` declares as a dependency.

  Ofer then said to just use the full app, which is the better call and I should have reached
  for it earlier: `sim.experience` now defaults to `isaacsim.exp.full.kit` instead of Isaac's
  minimal `isaacsim.exp.base.python.kit`. Keeping the CLI and the editor on the same
  experience removes a whole class of "works in the GUI but not from the CLI".

  Verified with the exact configuration the runtime now uses: schema registered, 28
  attributes, `IsA(Xformable)` true, `cesium:url` reading the right server, surviving play.
  The heavier app did **not** bring the segfault back: **0 crashes in 8 runs**.

  General lesson, worth applying beyond Cesium: **an extension that contributes USD schemas
  must be enabled before the schema registry initialises.** `enable_extension` returning
  `True` is not evidence that it did anything.

  **Verified:** 1103 pass, all hooks PASS, import-linter 23 contracts KEPT.

- **2026-08-26 (g)** — Two reported config bugs, both real, and an audit that found ten more.

  **`logging.isaac_logs = false` did nothing.** The setting existed in the schema and was
  never read. Worse, `__main__` called `logging.basicConfig(level=INFO)`, which sets the
  **root** logger, so every third-party Python logger flooded the terminal: 2,822 of 3,414
  lines. `ogn_registration` alone was 2,721.

  Fixing it took three attempts, each disproved by measurement:
  1. Kit startup args (`--/log/level=warning`) — no effect. Most of the noise is Python
     logging, not Kit's carb stream.
  2. Setting those loggers' levels at startup — no effect. Each extension sets its own level
     while loading, overwriting ours.
  3. Setting them *after* extensions load — no effect either. The noisiest records are
     emitted **during** the load, so there is no moment afterwards to intervene.

  What works is a **filter on the root handler**, which drops the records whenever they are
  emitted, with anything at WARNING or above always passing so real problems still surface.
  Result: **3,406 lines -> 603**, and `isaac_logs = true` still gives everything.

  **`vehicles.drone_0.rotation_frame = "world"` behaved as body frame.** Ofer's test case:
  pitch down 90, then change yaw; he expected the view to spin about its centre and instead
  the nose swung. Diagnosis in two parts:
  - `rotation_frame` was only ever applied to how the **gimbal offset** composes onto the
    aircraft attitude, and the gimbal offsets come from a ROS subscriber that publishes
    nothing by default, so the offset matrix was always identity and the setting was
    unobservable. Ofer corrected my first reading here -- he meant the vehicle, not the gimbal.
  - The aircraft's own attitude went through `euler_to_matrix` with a hardcoded intrinsic
    XYZ (`rxyz`), which makes yaw a body-axis rotation regardless of the setting.

  `euler_to_matrix`, `matrix_to_euler` and `euler_to_quaternion` now take a frame:
  BODY keeps `rxyz`; WORLD uses `rzyx`, the standard aerospace yaw-pitch-roll order, where
  yaw is applied first about world up. Verified numerically before implementing -- pitched
  -90, the nose stays at `[0,0,1]` for every yaw under `rzyx` while only the up vector spins,
  which is exactly the behaviour asked for. 22 tests cover it, including the contrast case.

  Note the 2023 repo used `rxyz` too, so this was never a regression: world-frame behaviour
  there came from `_apply_world_axis_rotation` in the **dev kit**, which composed a world-axis
  delta client-side and sent the resulting absolute RPY. Our fix puts it in the simulator,
  which is better -- it works for any sender, not just ours.

  **The audit.** Both bugs were the same shape -- a config key that parses and validates but
  that nothing reads -- so I checked all 65 schema fields against every use in `src/`,
  `extensions/` and `scripts/`. Twelve were dead. Two are now fixed (`isaac_logs`,
  `tileset_server_url`); the remaining ten are listed in `docs/dev/roadmap.md` with reasons.
  `tests/unit/config/test_no_dead_keys.py` now fails on any **new** dead key and also fails
  when a key on the known-dead list gets fixed but is left on it, so the backlog cannot rot
  in either direction.

  Lesson: a config surface that silently ignores values is worse than one that does not offer
  them. Ofer found two; a five-minute audit found ten. Worth doing that audit on any new
  config block from now on.

  **Verified:** 1103 pass, all hooks PASS, import-linter 23 contracts KEPT, live launch 603
  log lines with 0 crashes.

- **2026-08-26 (h)** — Roll and pitch were swapped at the camera, and the log fix finally
  landed properly.

  **Logs.** My previous fix cut 3,406 lines to 603, which Ofer correctly said still looked
  the same -- 499 of the remainder were Kit's `[ext: ...] startup` list, printed straight to
  **stdout**, which no log level touches. `--/app/enableStdoutOutput=false` removes them, and
  our own logging goes to stderr so it survives. **3,406 -> 77 lines**, with
  `isaac_logs = true` still giving the full 3,406. Lesson: "quieter" is not the goal, and a
  percentage reduction can miss the thing the user actually sees.

  **Roll and pitch traded places.** Ofer reported that a pitch input rolled the picture and a
  roll input tilted the nose, with specific directions. Modelling the whole chain -- wire NED,
  `ned_to_enu`, `euler_to_matrix`, then the camera prim's own 90 degree mount -- reproduced it
  exactly: `+pitch` left the forward vector untouched and tilted the up vector, `+roll` tilted
  the nose down.

  The cause was `ned_to_enu`, which swapped the two axes (`roll_enu = pitch_ned`,
  `pitch_enu = roll_ned`). KIRO recorded that as *"correct, not a bug -- it matches the 2023
  formula exactly"*. It does match 2023, and 2023 was wrong. Matching a previous
  implementation is not the same as being correct, and I should not have treated it as
  evidence.

  Found the right mapping by testing candidates against the body-axis convention rather than
  by reasoning about it: `roll_enu = roll_ned`, `pitch_enu = -pitch_ned`, yaw unchanged. The
  roll sign was settled by working out that a positive rotation about the nose axis takes the
  right wing down, so `+roll` must tilt the up vector east when flying north.

  Now: `+pitch` raises the nose, `-pitch` lowers it, `+roll` drops the right wing without
  moving the nose, `+yaw` turns right, and pitch no longer banks the image. Confirmed live --
  for `pitch=+20` the expected and measured camera forward both came out `[0, 0.94, 0.342]`.

  `tests/unit/geo/test_camera_axes.py` (25 tests) asserts **where the camera looks**, not the
  formula. The old suite passed while the picture was wrong precisely because it tested the
  formula against the previous generation. Behavioural tests are the only kind that would have
  caught this.

  **Verified:** 1103 pass, all hooks PASS, import-linter 23 contracts KEPT, live launch 77 log
  lines and orientation matching computation to 1e-3.

- **2026-08-26 (i)** — Full validation of both rotation frames, and a false alarm worth
  recording.

  Ofer asked whether WORLD should mean "each angle rotates only its own axis". The honest
  answer is that full independence is **impossible** for any Euler triple, because rotations
  do not commute -- only the outermost angle can have that property. Established the exact
  algebra by test rather than assertion:

  | Frame | Matrix | Angle that is always about a fixed world axis |
  |---|---|---|
  | WORLD | `Rz(yaw) @ Ry(pitch) @ Rx(roll)` (fixed-axis XYZ, aerospace yaw-pitch-roll) | yaw, about world up |
  | BODY | `Rx(roll) @ Ry(pitch) @ Rz(yaw)` (intrinsic XYZ) | roll, about world east |

  So Ofer's observation was correct and is not a bug: in WORLD, yaw always rotates about
  world up, but once yaw is non-zero, changing pitch rotates about a yaw-rotated axis. What
  *is* guaranteed, and is now tested, is that the stored orientation decomposes back to
  exactly the three angles that built it -- exact to 2e-16 in both frames, and the orientation
  survives decompose-recompose even at gimbal lock where the triple is not unique.

  `tests/unit/geo/test_rotation_pipeline.py` (133 tests) validates the whole chain for both
  frames: wire bytes, decode, NED to ENU, matrix, quaternion, camera mount, view direction.
  It includes the honest negative test that WORLD pitch is not world-stable once yawed, so
  nobody rediscovers that as a bug.

  Then confirmed **live through the real OGN node**: **11/11 attitudes matched computation for
  WORLD and 11/11 for BODY**, across level, single-axis, combined, straight-down and
  near-gimbal-lock cases.

  **The false alarm.** My first two live attempts reported 3/11 and 4/11, with the prim
  orientation apparently stuck. Cause: Ofer had `isaac-core-pose-sender` open, transmitting to
  the same UDP port at 30 Hz, so my short bursts simply lost -- last packet wins. Nothing was
  wrong. Two lessons: check for competing senders before believing a pipeline is broken, and a
  burst is not a valid way to drive a last-writer-wins input. Added a README troubleshooting
  entry, since this will happen to the team too.

  Also cleaned up a genuine self-inflicted hazard found on the way: a leftover `/tmp/bisect.py`
  from earlier debugging was **shadowing Python's stdlib `bisect`**, which crashed Isaac on
  startup with a traceback pointing at my own probe file. That also retroactively explains the
  `partially initialized module 'isaacsim'` failures I had misread as a systemic problem
  earlier in the day. Probe scripts now live inside the repo and are deleted after use.

  **Verified:** 1236 pass, all hooks PASS, import-linter 23 contracts KEPT, 22/22 live
  attitudes across both frames.

- **2026-08-27 (feature audit, ongoing)** — Systematic validation that every claimed
  feature actually works. Two parallel sub-agents audited the pure-Python breadth (devkit,
  and the position/math pipeline); I verified their claims independently and did all live
  sim checks myself.

  **Position/math pipeline: clean.** 63 independent numeric checks (packet offsets+checksum,
  encode/decode round-trip, hold-last-good, LLA<->ENU vs pyproj, NED<->ENU, BODY=Rx@Ry@Rz /
  WORLD=Rz@Ry@Rx, quaternion==matrix, slerp, gimbal lock, trajectories, motion limits).
  Every README coordinate/packet claim verified true. Corroborates the earlier frame work.

  **Config second pass (Ofer's ofer.toml comments), all fixed + live-verified:**
  - `cesium.tilesets_root` default was `/tilesets` but the prim is `/World/tilesets` -> the
    tileset URL override silently did nothing. Fixed root (and `BBOXES_ROOT` -> `/World/bboxes`).
  - `hdri` created a second dome light; now repoints the scene's existing
    `/World/Environment/DomeLight`, as Ofer found.
  - `delete_cache_on_launch` caused intermittent "disk I/O error" because Cesium (a boot
    extension) had the sqlite open; moved deletion to before the app starts. 0 errors now.
  - `scene` relative paths now resolve against the working directory.
  - `domain_id` warns when it disagrees with `$ROS_DOMAIN_ID`.
  - `sim.viewport.primary_camera` (dead, confusingly named) removed with Ofer's OK; the
    logical-key-camera idea is roadmapped.

  **Devkit defects the audit + live testing caught (all fixed):**
  - `SimSession.reset()`, `features.enable/disable()` called control methods the runtime
    NEVER registered -> `MethodNotFoundError` against a real sim, while the README showed
    `features.enable(...)` as a working example. Registered handlers that raise a clear
    NotImplementedError (matching `capture_frame`/`set_config`), fixed the README, roadmapped
    the real implementations. Added `test_control_method_coverage.py`: a static guard that
    every control method the devkit calls is registered (verified it bites).
  - `SimSession.vehicles()` called `get_state` and returned `{running, scene, headless}` --
    misleading name. Renamed to `state()`.
  - No `SimSession.get_pose()` existed though the control plane supports it and the inspector
    uses it. Added it. (I had briefly put `session.get_pose()` in the README before it
    existed -- caught by live testing.)
  Live-verified against a running sim: state/get_capabilities/pause/resume/step/get_pose/
  config.get all work; reset/features.enable/config.patch all fail with a clear deferred error.

  **Verified:** 1263 pass, all hooks + import-linter green.

- **2026-08-27 (audit cont.)** — More feature validation + inspector redesign.

  **Recording serializers (D-DEV-5) closed:** added `test_recording_serialisers.py` (6
  tests) exercising the real pose/range/bbox serialiser closures with fake messages by
  monkeypatching the lazy ROS type loaders -- no ROS install needed. A renamed message field
  now fails a test.

  **ROS pose-input path verified live:** ran the sim with `pose_source="ros"`, published a
  `NavSatFix(lat=32.3, alt=1500)` on `/mavros/global_position/global` from host Python 3.10,
  and the prim moved to `(0, 8339.89, 977.83)` -- identical to the UDP path for the same
  input. Capabilities showed `camera_ros`. So `pose_source="ros"` works end to end.

  **Sidecar:** honest skeleton -- real code (rtp/service/supervisor), 31 unit tests, imports
  without `gi`, documented as skeleton in the README. `caps_string('h264')` returning None is
  correct (it maps raw ROS encodings, not compressed ones).

  **Inspector redesigned per Ofer to read REAL values off the running stage**, not echo
  config. Added two control methods: `read_prim_attribute` (generic single-attribute live
  read) and `get_runtime_values` (curated: udp_port, rotation_frame, enu_reference, topic
  names, camera focalLength/apertures, tileset URLs -- all read off the composed prims on the
  main thread). The inspector prints a "Live values (read from the running stage)" section.
  Verified live with a custom config: it reported udp_port=40404, rotation_frame=body,
  focalLength=20.0, fov-derived hAperture=23.09 -- the actual applied values, not config
  defaults. This also fixes the confusing "None everywhere" Ofer saw (get_config shows the
  raw None-means-derive fields; the live section shows what they became).

  **Verified:** 1270 pass, all hooks + import-linter green.

- **2026-08-27 (audit complete)** — Finished the full feature audit. Three more parallel
  sub-agents (CLI/install, OGN extension nodes, control plane/contracts), each independently
  re-verified, plus my own live checks.

  **Clean (0 defects):** OGN extension nodes (6 nodes, schema==impl, delegate to the pure
  kernel, 132 guard tests), control plane + contracts (168 tests + 133 probe checks: JSON-RPC
  round-trip, token auth, confine_path traversal blocking, framing, port/topic/prim logic).

  **CLI defects found and fixed:**
  - `--isaac-path /bogus` silently fell through to auto-probing and used a *different*
    install. Now an invalid explicit path raises `IsaacInstallError` naming the path.
  - `--set` could not set list/dict fields (`--set sim.extensions '[...]'` crashed) because
    `cli_source` passed raw strings while `env_source` coerced. Made them symmetric: string
    `--set` values now coerce (bool/int/float/JSON) like env vars; non-string programmatic
    values pass through untouched. Updated the install test (it had locked in the old
    silent-fallthrough) and added cli_source coercion tests.

  **Ofer's eyes-on camera test:** terrain renders, viewport is the drone camera, motion
  smooth + holds on stop, heading follows travel -- all PASS. Two issues he raised:
  - `--pitch-deg` did nothing on orbit/path: real bug -- `_orbit`/`_path` never passed
    `roll_r`/`pitch_r` to the trajectory, so it always flew level. Fixed; verified the poses
    now carry the pitch.
  - Periodic viewport hiccup: ruled out the pose pipeline by sampling the live prim during an
    orbit (119 samples, 0 outliers >3x mean, 0 frozen -- provably smooth). It is render-side,
    consistent with Cesium 3D-tiles streaming. Roadmapped (tuning + optional slerp smoothing).
  - Pose-sender GUI: Ofer verified every control -- fields, nudge, lock, pause/resume, reset,
    live log, packet table. Fully working.

  **ROS publish content verified live:** `/isaac_core/global_pose` carried the exact sent
  lat/lon/alt (32.4/35.3/1234.0), a real non-identity quaternion, and an advancing timestamp
  (sec 2, nanosec 133333333) -- the SecondsToRosStamp node working end to end.

  **Verdict:** every declared feature is now verified-working, fixed, honestly roadmapped, or
  removed-with-approval. Nothing found claims to work while broken. 1274 pass, all hooks +
  import-linter green.

- **2026-08-30** — Three inspector bugs Ofer found by exercising every flag.

  - **`--count N` alone was silently ignored** and printed the one-shot report instead of N
    samples, because `--count` only took effect alongside `--poll` -- and `--help` never said
    so. `--count` now implies polling (interval defaults to 0.5s), and the help says it.
    This is the same "flag that quietly does nothing" class as the dead config keys.
  - **`--help` advertised `isaac-core-inspector`**, which is not an installed command (the
    console script is `isaac-core-inspect`). Fixed the argparse `prog`.
  - **`--port 33333` gave a bare "connection refused"**. 33333 is the UDP *pose input* port
    and appears right next to the control-plane port in config, so this is an easy mistake;
    the error now names the confusion and points at 8760.

  Five regression tests added. Live-verified: `--count 5` against an orbiting sim polled 5
  samples with changing values (x 226->294, y 444->403, z constant 483.28 = alt 1000 - 516.7),
  which also re-confirms the `--pitch-deg` orbit fix.

  Everything else in Ofer's transcript was correct behaviour: the all-zero pose with the
  "no pose received yet" hint (no sender running), and the raw `None`s in the Config section
  are the derive-if-unset fields, with the Live values section showing what they resolve to.

  **Verified:** 1279 pass, all hooks + import-linter green.

- **2026-09-02 — VERSION 1 SHIPPED.** Ofer declared v1 done: the kernel and its features
  work, are tested, and both UDP and ROS position pipelines are validated. Eyes-on
  non-headless flight confirmed correct; README images deferred as non-blocking.

  Built the Version 2 plan. Re-inventoried the 2023 repo from its **README's own feature
  list** (flags, `consts.py` config constants, ROS input/output topic tables) cross-checked
  against its source tree, rather than trusting the existing parity table. That surfaced gaps
  the earlier table had missed:

  - **We ship no custom ROS 2 message definitions.** `devkit.recording` imports
    `isaac_ros2_messages.msg.FrameBboxes`, and bbox/SAT/gimbal all need `Gimbal`, `Bbox`,
    `FrameBboxes`, `SATOutput` — but this repo contains none. It only resolves because the
    2023 workspace is built on Ofer's machine. That is a real portability gap, so it became
    **M1**, the foundation milestone that unblocks M2-M4.
  - **`--sat` was an *input* topic**, not just a capture flag: `/isaac_core/sat` carries an
    output path and the sim writes a PNG there. That reframes it as a ROS-triggered capture
    sharing the `capture_frame` / `output_root` implementation.
  - **2023 had a second, Cesium-free scene** (`full_warehouse.usda`). High value we had not
    noted: it lets the sim run with no network or tile server, useful for CI and offline work.
  - **`MAX_OUTPUTS_ROS_HRZ`** is 2023's publish-rate cap, which maps to the `publish_rate_hz`
    key we removed as dead -- to be re-added with a real implementation.
  - Recorded the **migration note**: 2023 abused the quaternion (x=roll, y=pitch, z=yaw); we
    publish a real one (D16), so 2023 consumers need updating.

  Restructured `docs/dev/roadmap.md`: v1 marked shipped with its closeout, the parity table made
  README-authoritative, and the flat "after v1" list replaced by **seven ordered v2 milestones**
  (M1 messages -> M2 sensor layers -> M3 gimbal -> M4 streaming -> M5 runtime control ->
  M6 swarm -> M7 production readiness), each with goal, work items, dependencies, size, owner
  and acceptance criteria. Added a v3 section for genuinely new capability (missions, dynamics,
  tracking). Rewrote "Needs Ofer" as a v2 table tied to milestones. Updated the README's
  "what does not work yet" to point at the milestones instead of listing bare gaps.

  v2's definition of done keeps the v1 bar: every feature verified against a running sim, no
  dead config keys, no untrue README claim, and it must work on a machine that has never seen
  the 2023 repo.

- **2026-09-02 (b)** — Version 2 plan revised on Ofer's direction. Three new decisions
  recorded (D20-D22) because these are architectural, not task-level:

  - **D20: ROS 2 is the data plane, the control plane is the command plane.** Ofer restated a
    decision from the start of the project, clarifying D4. Data flowing in/out (pose, image,
    range, bbox) goes on ROS topics; *commanding* the sim (take a picture, set gimbal angles,
    reset, load a scene) goes over JSON-RPC. Consequences: 2023's `--sat` ROS topic becomes
    `capture_frame(path, width, height)` in M5 -- with a requested resolution, which 2023 could
    not do -- and gimbal gains a `set_gimbal` control method. Left an open question for Ofer:
    whether the existing ROS gimbal subscriber still earns its keep.
  - **D21: use Isaac Sim 6's native RTSP, not our RTP sidecar.** Wire it into the
    image-publisher graph in *both* camera layers, always on -- no flag, no sidecar process.
    This supersedes `sidecar.rtp` and makes it dead code; flagged for Ofer whether to delete
    the sidecar entirely or keep the supervisor framework for future services.
  - **D22: no ROS 2 rate manipulation anywhere.** No throttling or boosting, on any pub/sub --
    it invites untimed-message chaos. So `publish_rate_hz` / 2023's `MAX_OUTPUTS_ROS_HRZ` are
    deliberately *absent*, not merely unimplemented, and the parity table now says so. The one
    legitimately related problem is **video recording**: 2023 made the user guess a constant
    fps while Isaac runs a variable ~30-50 fps, so recordings played at the wrong speed.
    M7 now requires deriving real timing from `header.stamp` -- possible only because we
    publish real advancing timestamps, which 2023 did not.

  Also per Ofer: the Cesium-free second scene is **not planned** (feature development first,
  not scene work), and M1 vendors the 2023 `simulation/ros2_interfaces/` wholesale keeping the
  `isaac_ros2_messages` name so existing consumers keep working.

  Validated that package rather than assuming: `isaac_ros2_messages` v0.2.0, ament_cmake +
  rosidl, deps std_msgs/geometry_msgs/builtin_interfaces, four definitions. **Found a real
  gap:** `Bbox.msg` carries 16 fields (pixel box **plus** lat/lon/alt, roll/pitch/yaw and
  distance_x/y/z) while our `bbox_recorder` serialiser reads only 7 -- it would silently drop
  the geodetic and distance data. Recorded as M2 work so recordings do not lose what the
  publisher sends. Also noted `SATOutput` will have no consumer once capture is control-plane,
  but is vendored anyway for wire compatibility.

- **2026-09-02 (c)** — Plan finalised. Ofer's two open questions answered and a new decision
  recorded (**D23**: reach for what Isaac Sim already offers before building our own, and carry
  no dead code — applied while developing, not only at cleanup).

  - **Gimbal ROS subscriber: dropped.** Anything external that wants to move the gimbal uses
    `Sim.attach()` and the control plane. Removing it also unblocks the config problem: those
    node inputs are *connected* to the subscriber today, and a connected USD attribute ignores
    authored values, which is exactly why `start_*_deg` was dead.
  - **Sidecar: keep the framework, drop the implementation.** `sidecar/service.py` (supervisor
    + registry, 15 tests) stays as genuinely reusable; `sidecar/rtp.py` (~340 lines, 16 tests),
    the `RtpVideoService` export, its registration import and possibly `DEFAULT_RTP_*` ports all
    go when native RTSP lands.
  - **Messages trimmed to what has a consumer:** vendor `Bbox` + `FrameBboxes` only. `Gimbal`
    and `SATOutput` are *not* vendored, because both became control-plane commands (D20) and
    nothing would consume them. Vendoring them "for compatibility" would have been dead code on
    arrival — my earlier plan said to vendor all four, which contradicted the very habit Ofer is
    asking for.

  Added **M8, a dead-code/dead-config sweep as the explicit v2 exit gate** (empty known-dead
  list; no module/class/function/message without a consumer; no `Method` enum member without a
  handler *and* a caller; no `NotImplementedError` the README implies works), and made "no dead
  or unused code" the fifth item in v2's definition of done.

  Rewrote the Ofer section as **"Ofer's task list for Version 2"** — 12 numbered items grouped
  into USD authoring (4), eyes-on validation (5) and environment/decisions (3), each tagged with
  the milestone it blocks so nothing surfaces late.

  Plan is implementation-ready. M1 is the unblocker.

- **2026-09-02 (d)** — Version 2 development started. 1279 -> **1337 tests**.

  **M1 blocked on a decision, and the reason is a real finding.** Validating the 2023
  `ros2_interfaces/` package showed it is **not standalone**: its `CMakeLists.txt` also
  generates five `.srv` files (`IsaacPose`, `GetPrims`, `GetPrimAttribute(s)`,
  `SetPrimAttribute`) that do **not exist in that folder** — they are NVIDIA's. The installed
  package at `~/IsaacSim-ros_workspaces/humble_ws/install/isaac_ros2_messages/` carries both
  NVIDIA's srvs *and* the four 2023 msgs, so what Ofer actually has is NVIDIA's package with
  the custom messages **merged in**. The 2023 folder is an overlay, not a package. Therefore
  "keep the name" cannot mean shipping our own `isaac_ros2_messages`: two packages with one
  name collide, and if ours won it would remove the srvs Isaac's own ROS tooling uses.
  Escalated to Ofer with the recommendation to use a distinct package name.

  **M2 distance sensor: node built and verified live.** Isaac 6 *does* ship a `RaycastSensor`,
  but it is a Python runtime class, not an OGN node or USD schema, so it cannot go in an action
  graph — so per D23 the right "use what Isaac offers" is the **PhysX scene-query interface**
  (`raycast_closest`), which is what Isaac exposes for this.
  - New extension `isaac_core_ogn.sensors` with `DistanceSensor`.
  - All *interpretation* lives in `isaac_core.contracts.rangefinder` (17 tests, no Isaac):
    `sensor_msgs/Range` conventions, +inf for no detection, -inf for too close, deliberately
    **not** clamped — clamping would make "nothing there" indistinguishable from "something at
    exactly max range".
  - `DistanceSensorConfig` (min/max band, topic) with a validator rejecting an inverted band.
  - `distance_topic` resolver honours an explicit config topic (same bug class `image_topic`
    had).
  - Manifest `assets/layers/distance_sensor/layer.toml` written to match the build sheet.
  - **Live-verified**: node registers as `isaac_core_ogn.sensors.DistanceSensor`, computed
    60/60 frames, and with the sensor 10 m above a surface at 0.1 m reported
    `range_m = 9.899`, `hit = true`, correct min/max pass-through. The guard tests earned their
    keep again — they caught the missing extension icon and the stale node inventory.

  **M7 video recording at the true frame rate (Ofer's D22 exception).** `save_video` now derives
  the rate from `header.stamp` and never asks for an fps. Only OpenCV is importable (no PyAV /
  imageio-ffmpeg), and `cv2.VideoWriter` is constant-rate, so: fps comes from the **median**
  inter-frame interval (robust to dropouts, unlike a mean), and exact per-frame presentation
  times are additionally written to a `.timestamps.txt` sidecar so a true VFR remux is possible
  later without re-recording. Verified numerically: 200 frames of U(30,50) jitter -> 38.94 fps,
  38.81 with a 2 s dropout injected, safe on single and duplicate stamps.

  **M6 groundwork: `{camera}` placeholder** in manifest bindings, so camera keys are no longer
  hardcoded. Verified end to end: renaming the camera to `rgb` now resolves
  `vehicles.drone_0.cameras.rgb.width` and composes with no `ConfigKeyError`.

  **Also fixed:** `bbox_recorder` was dropping 9 of `Bbox.msg`'s 16 fields (the geodetic
  position, orientation and per-axis distances) — now serialises all 16.

  Rewrote `docs/dev/usd_build_sheet.md` for v2: Task A (remove the gimbal ROS subscriber, ready
  now), Task B (distance sensor layer, ready now — node exists and is verified), Task C (bbox,
  blocked on a node I have yet to build), Task D (RTSP, blocked on my investigation).

- **2026-09-02 (e)** — Validated Ofer's USD tasks; RTSP and the message package landed. **1352 tests**.

  **D24: our ROS 2 messages ship as `isaac_core_ros2_msgs`, built into the user's humble_ws.**
  Ofer's decision after the naming finding. `ros2/isaac_core_ros2_msgs/` holds
  `CMakeLists.txt` + `package.xml` + `msg/`, carrying **only** `Bbox` + `FrameBboxes`.
  `scripts/setup.sh` step 5 copies it to `$ROS_WS/src` (default
  `~/IsaacSim-ros_workspaces/humble_ws`, overridable) and runs
  `colcon build --packages-select isaac_core_ros2_msgs` — `--packages-select` so setup cannot
  rebuild the user's whole workspace. Skips with an explanation when ROS or the workspace is
  absent, since nothing else in the repo needs these messages.
  - Built clean in 3.56 s. `Bbox` has all **16 fields with names identical to 2023**, so a
    consumer changes only the package in its import. NVIDIA's `IsaacPose`/`GetPrims`/
    `SetPrimAttribute` srvs verified **still importable** — the collision we avoided is real.
  - Did **not** copy 2023's `package.xml`: it declares an NVIDIA **proprietary** license and an
    NVIDIA maintainer, which cannot go into an Apache-2.0 repo. Ours is Apache-2.0.
  - `devkit.recording` now imports `isaac_core_ros2_msgs`; a test asserts `isaac_ros2_messages`
    never reappears, and another asserts CMakeLists lists exactly the `.msg` files present —
    the defect 2023 actually had.
  - Added a `check-xml` pre-commit hook, since the repo now ships XML.

  **Validated Ofer's USD tasks A and B. Task A clean; task B had a silent-failure bug.**
  - Task A (gimbal subscriber removed from both camera layers): correct. No `Gimbal` reference
    remains and the `offset_*_deg` inputs are now **unconnected**, which is what makes the
    gimbal config reachable at all.
  - Task B: all six node types correct and wired, but `RangeSensing` was declared at **layer
    root, a sibling of the `defaultPrim`** rather than under it. A layer is composed by
    reference, which brings in only the defaultPrim's subtree — so the entire graph would have
    been dropped with no error and the sensor would have published nothing. Caught by a new
    guard rather than by eye, and the guard also revealed the cause of the blind spot:
    `test_usd_layers.py` only globbed `camera_*`, so a new layer was never checked. Now
    `ALL_LAYERS`.
  - Manifest realigned to Ofer's actual node name (`distance_sensor`, not
    `distance_sensor_node`).

  **M4 is done, natively, and Ofer's "naive" attempt was right.** He enabled
  `isaacsim.streaming.rtsp` and wired `RTSPCameraHelper` into both camera image graphs:
  `execIn` from the playback tick, `renderProductPath` from the viewport render product, other
  inputs left unauthored to take `.ogn` defaults. The **one** missing piece was that
  `isaac-core run` never enabled the extension — a GUI-only enable does not carry over, and the
  node would have logged "Could not find node type interface" and done nothing. Added
  `isaacsim.streaming.rtsp` to the default `sim.extensions`, plus per-camera `rtsp_port` /
  `rtsp_mount_path` config with a derived-mount resolver and manifest bindings in both layers.
  - **Live-verified**: port 8554 listening; RTSP `OPTIONS` → `200 OK`; `DESCRIBE
    rtsp://127.0.0.1:8554/stream` → `200 OK` with SDP `m=video 0 RTP/AVP 96` /
    `a=rtpmap:96 H264/90000`. A real H.264 stream, no sidecar involved.
  - **Consequence: `isaac_core.sidecar` (RTP/GStreamer) is now dead code** and should be
    removed under D23. Needs Ofer's sign-off before deletion.

- **2026-09-02 (f)** — Five components in parallel; bbox pipeline unblocked. **1352 -> 1435 tests**.

  Ofer's `RangeSensing` fix validated (now nested under `/Root`). Ran two rounds of parallel
  sub-agents with strict single-writer file ownership to avoid concurrent edits to shared
  modules; **every component was then re-verified independently from first principles**, not by
  reading the agents' own tests. Worth noting: both rounds produced sub-agent reports claiming
  full-suite failures that were actually artifacts of *another agent mid-edit* — the reports
  contradicted each other, and only an independent run settled it. Concurrent agents cannot be
  trusted to report suite state.

  **`contracts/projection.py`** — pure pinhole projection kernel (30 tests). Independently
  verified against hand-computed optics: with focal 20 / aperture 40 the derived hFOV is exactly
  90.000 deg, the centre maps to (960, 540) exactly, the 45-deg edge ray lands exactly on x=1920,
  +Y maps *above* centre, and a behind-camera or on-lens-plane point returns `None` rather than a
  plausible mirrored pixel — the classic negate-z bug, now a guarded test.
  Aperture convention confirmed to be the exact inverse of `configurator._resolve_horizontal_aperture`
  (`aperture = 2*focal*tan(fov/2)`), so kernel and camera prim cannot disagree.

  **`geo/gimbal.py`** — gimbal angle state + rate limiting (22 tests). Independently verified:
  179 deg -> -179 deg is a **2 deg** move (shortest path across the seam), no overshoot, per-axis
  independence, `dt <= 0` returns unchanged, and `max_rate <= 0` means *unlimited* to match the
  config default of 0 meaning "no limit" (surprising, so documented).

  **Cesium multi-tileset URLs (M7)** — `apply_tileset_server_url` now swaps only scheme+host+port
  and **preserves each tileset's own path**, over N tilesets, using `urllib.parse` rather than
  hand-rolled `://` splitting. Verified: two tilesets keep distinct paths, query strings survive,
  IPv6 authorities work, no-scheme and trailing-slash overrides normalise, and a hostless
  override leaves every url untouched with a warning rather than blanking the terrain.

  **`OgnBboxProjector`** — OGN node projecting targets to pixel boxes, all maths delegated to the
  kernel. **Live-verified in Isaac**: registered, computed 60/60, and with three targets (centre,
  far right, behind camera) returned `count=3` with the centre box `909..1011` — which matches
  hand calculation exactly, since the cube's near face at z=-95 subtends `5/95*960 = 50.5` px
  about the centre 960. The off-screen target clipped with `inFrame=false`; the behind-camera
  target **kept its array slot** with a zeroed box, so a consumer matching arrays by index cannot
  misalign. `isVisible` currently mirrors `inFrame`: **occlusion is not tested**, documented in
  the node and the build sheet rather than faked.
  - Found Isaac ships a native `ROS2PublishBbox2D`. Not used, and the reason is recorded in the
    build sheet: it is replicator-driven and publishes `vision_msgs/Detection2DArray`, which has
    no geodetic fields, while our `Bbox` carries lat/lon/alt, orientation and per-axis distance.
    D23 was considered and consciously overridden, not ignored.

  **CLI tab-completion (M7)** — dependency-free kubectl-style `isaac-core completion bash|zsh`,
  where the emitted script calls back into `completion --list-keys` at runtime so the candidate
  list can never drift from the schema. Independently verified: **all 55 emitted keys resolve**
  via `config explain` (not a sample — every one), the emitted bash passes `bash -n`, an unknown
  shell exits non-zero, and keys added to the schema *minutes earlier* (`rtsp_port`,
  `distance_sensor.max_range_m`) appear automatically.

  Wrote the **Task C bbox build sheet** and the `bbox` layer manifest, binding the projector's
  intrinsics from the same camera config that drives the camera prim so the projection cannot
  drift from the image being rendered. Flagged the real risk honestly: `FrameBboxes` holds a
  nested `Bbox[]`, which is the case most likely to defeat the generic `ROS2Publisher`; if it
  does, a dedicated publisher node is the fallback rather than Ofer fighting the GUI.

- **2026-09-02 (g)** — Validated Ofer's bbox layer; found two real bugs of my own. **1451 tests**.

  **D25: a layer's `requires` may be satisfied by another layer, and planning is dependency-ordered.**
  Requirements were only tested against capabilities probed from the *base scene*, so a
  capability provided by a **layer** could never satisfy another layer. `bbox` requires `CAMERA`,
  which `camera_udp` provides, and was therefore skipped on every launch with
  `unmet stage requirement(s): CAMERA` — which reads like a stage problem, not a planner bug.
  This is exactly what Ofer hit ("the bbox feature was not there").
  Planning is now a fixed point: admit every layer whose requirements are met, add what it
  provides, repeat until a pass changes nothing; whatever remains is skipped (which also covers a
  dependency cycle). The resulting order **is** the composition order, and that matters
  independently — alphabetically `bbox` sorted before `camera_udp`, so the camera prim would not
  have existed when bbox referenced it. Also added `DISTANCE_SENSOR`/`BBOX` to `StageCapability`
  and a guard that every shipped manifest uses only known names, because an unknown name silently
  skipped a layer forever.

  **Ofer's three criticisms of the build sheet were all correct**, and two were defects in my
  writing:
  - Paths were ambiguous: I mixed *in-layer* paths (rooted at the `defaultPrim`) with
    *composed-stage* paths (`/World/bboxes`, the camera). He can only author the former.
  - **Relationships could not be set at all** — the GUI picker only offers prims present in the
    file being edited, and neither target exists in `bbox.usda`. The design was impossible to
    execute; asking him to do it was my error.
  - **`/Root/Xform` in the bbox layer does nothing** — right. The camera layers need it because
    it is the prim the pose graph moves; a read-only layer has no use for it.

  **Relationship bindings: built, then abandoned on evidence.** Added a `[[relationships]]`
  manifest section resolved at compose time (the composer is the first moment the whole stage
  exists). Verified resolving correctly to `/World/.../main_camera_01` and `/World/bboxes`. Then
  the live run died. **Writing a relationship onto a live OmniGraph node aborts Kit**: exit 0
  before `run()` is reached, no traceback, no faulthandler output, and the base scene's stage is
  closed immediately after composition — which the healthy baseline never does. `og.Controller`
  instead of raw USD `CreateRelationship` made no difference. Node inputs are now plain
  **token paths** (`cameraPath`, `targetsRootPath`) written as ordinary attribute bindings, which
  also removes the GUI-authoring problem entirely. A guard asserts no shipped layer uses a
  relationship binding.

  **Still open, and it is mine.** With the projector given a real camera and targets root the
  simulator still exits before the loop. Bisect is clean and reproducible: identical layer with
  those two inputs unwired exits **124** (healthy, "simulation running" logged); with them
  populated it exits **0** having never reached `run()`. So it is the populated projector path,
  not Ofer's USD. Ruled out: a Python exception (no traceback), a Python-visible segfault (no
  faulthandler output), and an unresolvable `FrameBboxes` (sourcing the built workspace changes
  nothing). Next suspicion is the node's `UsdGeom.BBoxCache`/`ComputeWorldBound` path being
  evaluated eagerly on a Cesium-tileset stage — my standalone probe that passed used a trivial
  three-cube stage, which is exactly the difference.

  **Process note.** My earlier A/B "relationships are the cause" was **confounded** — I had
  renamed the node's inputs in the same step, so two variables moved at once. Re-testing showed
  relationships were not the trigger. Recording it because the lesson generalises: one variable
  per run, and a bisect that changes two things proves nothing.

  Ofer also flagged that he had Isaac Sim open during earlier live tests. Re-verified RTSP with
  ownership proof — 8554 closed with nothing running, LISTENING once my process was up, `DESCRIBE`
  `200 OK` with `H264/90000`, closed again after killing mine. The port's lifetime tracks my
  process exactly, so the earlier result stands. The distance-sensor and bbox-projector probes
  were unaffected (own processes, own stages, exact deterministic geometry).

- **2026-09-02 (h)** — Root-caused the bbox failure. It was mine, and it was one line of wrong path.

  **The bug: `_resolve_camera_prim` looked for a prim that has never existed.** It guessed
  `{mount}/Camera_{camera_id}`; the real camera is `{mount}/Xform/main_camera_01`. So it always
  returned `None`, and any layer binding `resolve = "camera_prim"` made `compute_writes` raise
  `ConfigKeyError: resolve 'camera_prim' requested but no camera path is available`. No shipped
  layer had ever used that resolver, so the dead code sat there until bbox became its first
  caller. Now resolves from **`sim.viewport_camera`** — already the single place naming the
  vehicle's camera, so a layer binding and the viewport cannot disagree — with a fallback that
  searches the mount for the first `Camera`-typed prim.

  **Why it was so hard to see, and the lesson.** The launcher swallowed the exception: exit 0, no
  traceback, a graceful "Simulation App Shutting Down". Three rounds of live A/B against the
  *symptom* produced two wrong conclusions (a Kit abort on relationship writes; a `BBoxCache`
  crash on Cesium geometry). Both were disproved by direct probes -- `ComputeWorldBound` on the
  Cesium-anchored cube returns correct bounds and survives play. What actually worked was
  reproducing composition **in-process**, where the real traceback appeared immediately. Lesson:
  when a launcher hides the error, stop bisecting the symptom and re-run the failing call in a
  process that cannot hide it.

  **Ofer's design catch, adopted.** He asked whether the camera's own values should override the
  projector's intrinsics inputs. They should, and now they do: the node reads focal length and
  both apertures **off the camera prim** rather than taking them as inputs, so the projection
  cannot disagree with the picture being rendered. A wrong-but-plausible box in the wrong place is
  worse than an obvious failure. Image size stays an input because resolution belongs to the
  render product, not the camera. Bindings went 8 -> 5, and `focalLength` is now an *output* for
  verification.

  **Verified live, end to end** (real launcher, exit 124 = healthy): both layers compose, the
  projector computes 90/90 and reports `count = 2` over `['Cube', 'Cube_01']` with
  `inFrame = [true, false]`. `/isaac_core/bbox` publishes as
  `isaac_core_ros2_msgs/msg/FrameBboxes`, publisher count 1, `header.stamp` advancing.

  **Confirmed limitation, needs Ofer's decision.** `bboxes` is always `[]`. Enumerated the
  publisher's live inputs: Isaac's generic `ROS2Publisher` exposes the nested `Bbox[]` field as
  **`inputs:bboxes` of type `token[]`** -- a flat token array that cannot carry 16 typed fields
  per element. Nested message arrays are unsupported, exactly the risk flagged when the layer was
  specified. Options put to him: flatten `FrameBboxes` to parallel arrays (works today, matches
  the node's outputs, but changes the wire format) or build a C++ publisher node (preserves the
  2023 shape, much more work). Recommended flattening.

  **Also found:** the message package must be on the library path or the publisher fails with
  `libisaac_core_ros2_msgs__rosidl_generator_c.so: cannot open shared object file` and the topic
  silently never appears. Sourcing `humble_ws/install/setup.bash` before launch is mandatory for
  the bbox layer; documented in the build sheet's run instructions.

  Rewrote `docs/dev/usd_build_sheet.md` per Ofer's request: tasks only, no rationale, with explicit
  DONE/TODO/BLOCKED status per task and a "do not author these" table. Explanations belong here.

- **2026-09-03** — Answered Ofer's two design questions with live evidence. Both change the plan.

  Verified his USD edits: `/Root/Xform` gone, stale `custom rel` declarations replaced with the
  `cameraPath`/`targetsRootPath` token declarations. Task C is complete on his side.

  **His proposed fix for the empty array is impossible in Isaac Sim 6, and the reason matters.**
  He suggested the OGN build a `Bbox` per detection and append them to an array. That requires
  constructing a ROS message object, which needs the rosidl Python runtime, which needs `rclpy` —
  built for Python 3.10 against Isaac 6's 3.12. Verified inside Isaac:
  `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`.
  **The 2023 `bbox_node.py` did exactly what he described** (`import rclpy`,
  `from isaac_ros2_messages.msg import FrameBboxes, Bbox`, build `List[Bbox]`, publish) — which is
  precisely why it cannot be ported. So "flatten" does not mean the node assembles messages; it
  means changing the `.msg` so `FrameBboxes` carries **parallel arrays of primitives**, because
  Isaac's C++ publisher can fill primitive arrays from OGN outputs but exposes a nested `Bbox[]`
  as a useless `token[]`.

  **His question "why does the bbox node need all those camera values?" was the better catch.**
  It shouldn't. Reading 2023's `bbox_node.py` shows it did **no projection maths at all** — it
  used `sd.sensors.get_bounding_box_2d_tight` / `_loose` against the viewport. Both helpers
  **exist in Isaac Sim 6** (`omni.syntheticdata` 0.6.15, cp312, so importable), and the
  replicator annotators `bounding_box_2d_tight`/`bounding_box_2d_loose_fast` are registered.
  That approach is strictly better on three counts: pixel-exact against the actual render (cannot
  drift from the image), **no camera intrinsics at all**, and **real occlusion** — `tight` vs
  `loose` is exactly the `is_visible` vs `in_frame` distinction our node currently fakes.
  D23 applies and I missed it: I hand-rolled projection when Isaac already had this.
  The catch, found by probing: the annotators returned **0 rows** on our scene because they key
  off **semantics**, and our cubes have none. 2023's `earth.usda` labelled every target with
  `SemanticsAPI` (`semanticType = "class"`, `semanticData = "Cube"`; five duplicate APIs on one
  cube, a GUI artifact). So switching costs one USD task: label the targets.
  Recommended switching; it would make `contracts/projection.py` dead code, which is the honest
  price and cheaper than shipping a projection that silently disagrees with the picture.

  Rewrote the build sheet to Ofer's spec: completed tasks **deleted rather than marked done**,
  decisions first because nothing else can proceed without them, then tactical tasks only.

- **2026-09-03 (b)** — **Correction: Ofer was right and I was wrong.** ROS message objects *can* be
  built inside Isaac Sim 6.

  He added `from isaac_core_ros2_msgs.msg import Bbox` plus `message_to_yaml` to the projector node
  and it ran, printing the structure. My contrary claim came from a probe that **never sourced the
  ROS workspace**, so `ModuleNotFoundError` proved nothing about the real environment. Sloppy: I
  drew an architectural conclusion from a broken test setup.

  Re-tested properly inside Isaac's 3.12 with the workspace sourced:
  - `Bbox()` constructs, `message_to_yaml` works, and **`FrameBboxes.bboxes = [b1, b2]` assembles
    the nested array**. The generated message modules are pure Python (they live under
    `.../python3.10/dist-packages/` yet import fine on 3.12).
  - `import rclpy` still fails: `No module named 'rclpy._rclpy_pybind11'`.
  - The message's Python C typesupport `_bbox_s` was never built. So a Python-assembled message
    has no route to DDS from inside Isaac.
  - Publishing from **host** Python 3.10 works completely: verified a real `FrameBboxes` on the
    wire carrying two nested `Bbox` with all 16 fields populated.

  So the corrected boundary is **construction, not publication**: building the message in-process
  is possible; publishing it is not. 2023 got away with it because Isaac 2023 bundled Python 3.10,
  where rclpy imported.

  This adds a third option and changes my recommendation. Previously I pushed flattening on the
  false premise that messages could not be built in-process. Now: **A** host-side rclpy publisher
  (keeps the exact 2023 `Bbox[]` shape, proven working, costs a process — and is precisely what
  `src/isaac_core/sidecar/` is for, so the proposal to delete it is withdrawn pending this
  decision); **B** flatten to parallel arrays (fewest moving parts, changes the wire format);
  **C** a C++ OGN publisher.

  **Guard strengthened, and it earned its keep.** `test_no_python_ros2.py` already caught
  `rosidl_runtime_py` but missed `isaac_core_ros2_msgs`, and its stated rationale was wrong for
  message packages — it claimed the import *cannot* succeed, when in fact it succeeds *only with a
  workspace sourced*. That is the more insidious failure: at module level it makes node
  registration depend on the launcher's environment, so the node type silently never registers on
  a machine without the overlay. Added our package to the list and rewrote the message to name
  both hazards. The suite is intentionally **red on one test** until the diagnostic snippet is
  removed from `OgnBboxProjector.py` — left in place rather than deleted, since it is Ofer's code.

- **2026-09-03 (c)** — Settled the nested-array question by measurement. Ofer's approach is blocked,
  but not for the reason I first gave.

  He pushed back twice, correctly, so I stopped asserting and tested Isaac's generic
  `ROS2Publisher` directly in a live graph:
  - `inputs:bboxes` for a nested `Bbox[]` resolves to **`token[]`**, extended type
    `EXTENDED_ATTR_TYPE_REGULAR` — a *fixed* type, so it can never resolve to anything richer.
  - Writing even legal tokens to it **segfaults Isaac** (core dumped, minidump written). The
    attribute is not merely useless, it is unsafe.
  - The same publisher handles a **primitive** array correctly: `std_msgs/Int32MultiArray`
    exposes `inputs:data` as `int[]`, accepts `[11, 22, 33, 44]`, survives 90 frames, no crash.
  - It also handles nested **single** messages fine, flattening them to `:` paths — visible as
    `inputs:layout:dim` on that same node, the same mechanism as `header:stamp:sec`.

  Conclusion: Isaac's generic publisher implements nested single messages and primitive arrays,
  but **not arrays of messages** — it creates a placeholder `token[]` that crashes on write.
  So the blocker is not building the messages (Ofer was right that we can) but that **there is no
  channel from an OGN node to the publisher for an array of structs**: OGN attributes carry typed
  graph data, and `list[Bbox]` of Python objects is not an OGN type at all.

  Both remaining routes are now *proven* rather than assumed: **A** host-side rclpy publisher
  (nested `Bbox[]` verified on the wire with all 16 fields) and **B** flatten to primitive arrays
  (that publisher path verified crash-free). A C++ OGN node remains untested.

  Worth recording as a pattern: three of my claims this week were overturned by Ofer asking
  "are you sure?" — the relationship-write cause, the message-import limit, and now the shape of
  the real constraint. Each time the correction came from running the thing rather than reasoning
  about it. Assert less, probe earlier.

- **2026-09-03 (d)** — **D26: `FrameBboxes` carries parallel arrays, not `Bbox[]`.** Ofer chose
  flattening. Implemented and verified end to end apart from his USD wiring.

  **Message.** `FrameBboxes` is now `header` + 16 parallel arrays. Field *names* are unchanged
  from 2023, so only the index moves: `msg.bboxes[i].x1` becomes `msg.x1[i]`. No `count` field —
  it would duplicate `len(target_name)` and could disagree with it. **`Bbox.msg` deleted**: with
  nothing referencing it, shipping it would be exactly the dead weight D23 forbids.

  **The risk I flagged is cleared.** Before touching the node I verified the one thing I had not:
  `string[]` maps to `token[]`, the same OGN type that segfaulted on the nested-array test. It is
  safe here — the crash was specific to an unimplemented *message-array* field, not tokens as
  such. Wrote all 16 arrays for 2 detections and the publisher survived, then confirmed on the
  wire: `target_name: [tank, truck]`, `is_visible: [true, false]`, correct ints and floats
  throughout. Had that failed, 16 GUI connections would have been wasted work.

  **Node.** Extended from 7 to all 16 per-detection outputs, named **identically to the message
  fields** so wiring is 1:1 with nothing to translate. Added: `lat`/`lon`/`alt` from each target's
  `cesium:anchor:*` attributes; `roll`/`pitch`/`yaw` from the authored `xformOp:orient` via the
  already-tested `geo.rotations.quaternion_to_euler` (confirmed importable inside Isaac —
  transforms3d 0.4.2 is present); `distance_x/y/z` as world-axis deltas between target and camera
  translation, which is what 2023 actually computed despite its comment saying "local".
  Missing geodetic data yields **NaN, not 0.0** — zero is a real place off Africa, so a consumer
  could not otherwise tell "unknown" from "there". A single `_ARRAY_FIELDS` tuple drives both the
  empty and populated writers so they cannot drift apart.

  **Live-verified** on the composed stage: `target_name = ['Cube', 'Cube_01']`,
  `in_frame = [True, False]`, `lat/lon/alt = [32.224777, 35.256363, 517.16]` matching
  `earth.usda`'s anchor exactly, real world-axis distances, `focalLength = 22.79` read off the
  camera prim.

  **Consumer side updated.** `bbox_recorder` re-zips the arrays into one dict per detection, since
  that is what a reader wants; a `_scalar` helper converts numpy elements, because ROS array
  fields deserialise to numpy and json would otherwise fail only at write time, long after
  capture. Tests rewritten for the flat shape, including a json-encodability guard.

  Ofer's diagnostic snippet removed now that it has served its purpose (it settled the design
  question), which returns the ROS-import guard to green. Added build-sheet task **T3**: 16
  identical-name connections, with the note that the publisher's inputs only appear once the
  message package is built and sourced — otherwise it looks like the inputs do not exist.

- **2026-09-03 (e)** — **Bounding boxes work end to end.** M2 complete.

  Validated Ofer's wiring after he recreated the publisher node. All correct: `execIn` connected
  on `seconds_to_ros_stamp`, `bbox_projector` **and** `ros2_publisher` (the recreated publisher was
  the risk -- an unwired `execIn` there is exactly the defect that silently broke both camera
  layers earlier); `ros2_context` and `read_sim_time` correctly have none; all 16 field
  connections present with matching types; `messageName`/`messagePackage`/`topicName` right.

  **His "duplicate GUI fields" were real, not a GUI bug.** `bbox.usda` still declared
  `outputs:inFrame`, `outputs:isVisible` and `outputs:targetNames` -- leftovers from before the
  node's outputs were renamed to match the ROS field names. The node type no longer has them, so
  they were inert USD cruft, but the GUI lists every *authored* attribute, hence each appearing
  twice alongside its snake_case replacement. Removed the three lines at his explicit request
  (noted as a deliberate exception to "Ofer authors all USD"), leaving exactly 16 array outputs.
  Worth remembering: renaming an OGN attribute leaves stale authored attributes behind in any USD
  that referenced the old name, and nothing warns about it.

  **Live end-to-end proof** on `/isaac_core/bbox`, type `isaac_core_ros2_msgs/msg/FrameBboxes`,
  publisher count 1, **~59.7 Hz** (min 0.012s, max 0.020s, std dev 0.0018s):
  `target_name: [Cube, Cube_01]`, `in_frame`/`is_visible: [true, false]`,
  boxes `x1/y1/x2/y2 = [799,305,869,363]` for the visible one and zeros for the other,
  `lat/lon/alt = [32.2247772, 35.2563629, 517.163]` matching `earth.usda`'s anchor,
  roll/pitch/yaw ~0 (identity orientation, float noise only), and real world-axis distances
  `[14.373, -3.514, 0.463]`. `header.stamp` advancing.

  Every value matches what the node produced in isolation, so the whole chain -- projector to
  publisher to wire -- is consistent.

- **2026-09-03 (f)** — **D27 (Ofer's decision): the sidecar mechanism stays; only its RTP service goes.**
  His reasoning, recorded because this is the first answer on the sidecar's future: deleting the
  supervision machinery now only to rebuild it for a future feature is wasted work. If the repo
  reaches DONE after Version 3 without ever needing it, remove it then.
  Acted on it: deleted `sidecar/rtp.py` and its tests, kept `service.py` (registry, supervisor,
  restart policy) and the `[sidecar]` config section, and rewrote that section's comment to say
  what the mechanism is *for* now -- work that cannot run inside Isaac's interpreter, `rclpy`
  being the obvious case. 1449 -> 1433 tests, the drop being the RTP suite.

- **2026-09-03 (g)** — Distance sensor: **three real bugs, and the FPS complaint was not the sensor.**

  **Bug 1 — double vs float.** `sensor_msgs/Range` declares `range`/`min_range`/`max_range` as
  float32, but our node published `double`. OmniGraph refuses that connection and leaves the
  publisher's input at **uninitialised memory**, which is exactly why Ofer saw
  `min_range: 0.0` and `max_range: -1.5881868392106856e-23`. Changed the node's outputs and range
  inputs to `float`, which then surfaced a second, clearer error --
  `requires output attribute "outputs:range_m" to be of type "float" instead of type "double"` --
  because the USD still declared them `double`. Aligned those too. Same lesson as the bbox
  duplicates: changing an OGN attribute's *type* leaves the old declaration behind in USD, and the
  node silently fails to instantiate.

  **Bug 2 — min and max were crossed.** `inputs:max_range` was fed `min_range_out` and vice versa.

  **Bug 3 — wrong raycast, and 2023 had it right.** I used `omni.physx.raycast_closest`, which only
  hits **collision** geometry. Cesium 3D Tiles terrain has none, so the sensor could never see the
  ground: it reported "no detection" forever while looking perfectly healthy. 2023 used
  `omni.kit.raycast.query` -- the **render** raycast, against rendered geometry -- and that
  extension exists in Isaac Sim 6 (1.2.0, cp312). Reimplemented on it using the *raycast sequence*
  API (`add_raycast_sequence` / `submit_ray_to_raycast_sequence` /
  `get_latest_result_from_raycast_sequence`), which is built for per-frame use and holds the last
  valid value rather than flickering while a query is in flight -- no async/await needed inside a
  synchronous compute. This is the third time now that 2023 had already solved something the way
  Isaac intends and I reached for a different API first.

  **Also wrong: the default range.** `max_range_m` defaulted to 100 m, a ground-rangefinder number.
  This sensor points down from an aircraft 500-2000 m above terrain, so the ray never arrived.
  Default is now **5000 m**, with the reasoning in the schema comment.

  **Verified live**, driven over UDP so the pose graph owns the transform (setting the prim
  directly is pointless -- `write_prim_attribute` overwrites it every frame, which is why an
  earlier attempt showed a frozen range):

  | altitude MSL | range_m | implied terrain |
  |---|---|---|
  | 2500 | 1982.58 | 517.4 |
  | 2000 | 1482.25 | 517.8 |
  | 1500 | 982.25 | 517.8 |
  | 1200 | 682.25 | 517.8 |
  | 1000 | 482.17 | 517.8 |
  | 800 | 281.75 | 518.3 |

  Every reading is altitude minus a consistent ~517.8 m terrain elevation, which matches the
  scene's ENU reference of 516.7 m. Self-consistent to within a metre across a 1700 m span.

  **The FPS dips are Cesium tile streaming, not the sensor** -- and proving it required a retest,
  because Ofer had a second Isaac Sim running during the first measurement. Clean baseline
  comparison afterwards: *without* the sensor, median 59.9 fps but **9 of 300 frames below 10 fps**
  (min 0.92); *with* the sensor at a 5 km ray, median 59.9 fps and **0 of 300 below 10 fps**
  (min 14.9). The baseline was worse, so the raycast is not the cause. Consistent with the
  viewport-hitch issue already roadmapped in M7. Range values reproduced identically across both
  the contaminated and clean runs, confirming GPU contention never touched the geometry.

  **No distance-sensor GUI is claimed anywhere** in the repo -- checked -- so nothing is broken
  there. 2023 had one; ours deliberately does not, and the honest place to read the value is
  `ros2 topic echo` or the inspector.

- **2026-09-03 (h)** — **D28: bounding boxes come from Isaac's synthetic-data annotators, and target
  semantics are applied automatically.** Ofer's call, and he was right to push: my hand-rolled
  projection was the wrong instinct and D23 already said so.

  **Answering the question he actually asked** (is there a COTS bbox OGN node?): no, and he was
  right not to find one. Isaac ships `ROS2PublishBbox2D`/`Bbox3D`, but they are *publishers* whose
  only data input is `data: uchar[]` -- an opaque annotator buffer -- and they emit
  `vision_msgs/Detection2DArray` with no geodetic fields. There are **no** synthetic-data OGN nodes
  exposing bbox arrays in a consumable form (checked all of `exts` and `extscache`). The annotators
  are a **Python API**, so switching changed only our node's internals: same node, same 17 outputs,
  same wiring, nothing for Ofer to rewire.

  **Implementation.** `compute` now calls `sd.sensors.get_bounding_box_2d_tight/loose(viewport)`
  and matches rows to targets by the `name` field, which is the **full prim path** -- not the
  semantic label, which is not unique. `in_frame` = present in loose; `is_visible` = present in
  tight. Sensors are enabled once per node, not per frame, since enabling allocates render
  resources. Viewport is resolved by matching `camera_path` against each viewport's camera, falling
  back to the active viewport (which is already correct for one vehicle because
  `sim.viewport_camera` points it at the drone camera). Dropped `imageWidth`/`imageHeight` inputs
  and the `focalLength` output -- with no projection there are no intrinsics -- and removed the
  three now-stale attribute declarations from `bbox.usda` so they cannot reappear as duplicate
  GUI fields.

  **`composer.apply_target_semantics`** labels every child of `BBOXES_ROOT` at composition time
  with `semanticType = "class"` and the prim's own name as data, using the top-level `Semantics`
  module (`pxr.Semantics` is deprecated in Isaac 6 and warns). Annotators key entirely off
  semantics, so an unlabelled prim is invisible to them however solid it looks -- that is why my
  first annotator probe returned zero rows. 2023 labelled by hand in USD; doing it in the composer
  means adding an object to the scene is the whole workflow.

  **Live-verified, and occlusion is the payoff:**
  - Composer labelled both targets by itself (`applied=True data=Cube` / `data=Cube_01`).
  - Boxes `Cube (747,452)-(754,459)`, `Cube_01 (529,464)-(567,515)`. Note these differ markedly
    from what the projection produced (`799..869`) because the annotator measures the *actual*
    viewport render while the projection used configured width/height -- precisely the silent
    drift that motivated the switch.
  - Inserted a blocker cube in front of `Cube`: `in_frame = [True, True, True]` but
    `is_visible = [False, True, True]`. **Real occlusion**, which the projection could never do.
  - Cost is small: median 59.9 fps with annotators (min 47.5, p05 58.5, 0/150 frames under 10 fps)
    against a 59.9 median baseline (min 59.8). Measurable, no dropped frames.

  **`contracts/projection.py` deleted** along with its 30 tests -- 1433 -> 1403 tests. That is the
  honest price of the switch and much cheaper than shipping a projection that can silently
  disagree with the picture. Third time this week that 2023 had already chosen the API Isaac
  intends: RTSP, the render raycast, and now the bbox annotators.

- **2026-09-06** — Relationship machinery deleted; **M3 (gimbal) complete**. 1403 -> 1408 tests.

  **Relationship-binding machinery removed** on Ofer's decision, ~427 lines across
  `manifest.py`/`planner.py`/`configurator.py`/`composer.py` plus its 188-line test file. The
  argument that settled it: unlike the sidecar -- a generic mechanism with a plausible future use,
  so kept (D27) -- this solved one problem for which token path attributes are now the better
  answer, and its most natural use (a graph node) *aborts Kit*. Keeping it would have been leaving
  a trap, not insurance.
  Note for next time: my automated block-cutter mishandled multi-line `def` signatures whose
  closing `) -> T:` sits at column 0, silently leaving orphaned function bodies that only ruff's
  syntax check caught. Repaired by explicit text matching. Cut blocks by parsing, or verify with a
  syntax check immediately.

  **M3 — gimbal over the control plane, done and live-verified.**
  - `Method.SET_GIMBAL` + `_handle_set_gimbal` + `SimSession.set_gimbal`.
  - **Design point worth keeping:** the handler only *records* the target; the simulation loop does
    the prim writes via a new `_step_gimbal` called each frame. That sidesteps the USD
    thread-safety problem entirely -- no main-thread task queue needed -- and means the caller is
    never blocked behind a frame. `set_gimbal` returns on acceptance, not arrival, because a
    rate-limited move takes real simulated time.
  - Slew uses the already-tested `geo.gimbal.slew_towards`, integrated at `sim.physics_dt` rather
    than wall time, so a commanded move takes the same simulated duration regardless of machine
    speed.
  - Config `start_*_deg` now bound in **both** camera layers to the pose node's `offset_*_deg`
    inputs. These were dead for all of v1 for a reason that is easy to miss: the inputs were
    *connected* to the (now removed) ROS subscriber, and a connected USD attribute ignores
    authored values.
  - Removed the dead `gimbal_topic` resolver, the `GIMBAL` topic usage and the `gimbal.topic`
    config field -- with the subscriber gone nothing published or consumed them (D23).
  - **Live proof:** start angles read back off the stage as exactly `(5.0, -12.0, 30.0)`; a
    commanded yaw 30 -> 90 deg at `max_rate_deg_s = 20` ramped through 41.3, 53.0, 64.7, 76.3, 88.0
    and settled at 90.0 in ~3.1 s. Measured ~19.4 deg/s against 20 configured, and unmistakably a
    ramp rather than 2023's unconditional snap.
  - **Dead-key list is down to two**: `stage_units_in_meters` and `ros2.use_sim_time`. Four of the
    six were the gimbal keys.

- **2026-09-06 (b)** — Two M7 items landed via parallel agents, plus a defect that had already cost
  me hours twice. **1408 -> 1431 tests.**

  **Georeference unified (M7).** `geo.enu_reference` and the scene's `/CesiumGeoreference` can no
  longer silently disagree. Authority rule: an explicitly-set config value is authoritative *but
  must agree with the scene*; otherwise the scene wins; otherwise the schema default. Beyond a
  **1 m** horizontal tolerance it now raises rather than warning, and the message reports the gap
  **in metres** (degrees are not intuitive) plus the three ways to fix it. Tolerance justified in
  code: Cesium's origin round-trips through USD and its own ellipsoid maths, so sub-metre noise is
  expected, and 1 m is far below anything that matters for a camera hundreds of metres up. Altitude
  is deliberately excluded from the trigger -- a reference above ground is routine. Separation uses
  a local haversine so `sim` still does not import `geo` (23/23 contracts kept).
  Verified our own repo agrees exactly (32.22481 / 35.25621 / 516.7 on both sides), so nothing
  breaks, and verified the guard fires: an 8360.8 m mismatch refused to start.

  **A "loud" failure that printed nothing.** Testing that guard exposed something worse than the
  bug it was fixing: the exception reached **nobody**. Kit suppresses stdout and our logging filter
  drops third-party handlers, so composition failures exited with no explanation. This is the exact
  mechanism that hid the bbox `ConfigKeyError` for three rounds of misdirected bisecting. Fixed at
  the source: `sim/__main__.py` now wraps `open_stage` and logs through our own logger — which *is*
  visible — before re-raising. Every startup failure is now explained, not just this one.
  Remaining wart: the process still exits 0 because Kit's `SimulationApp` owns shutdown, so exit
  status is not usable for scripting. Noted, not chased.

  **Camera intrinsics first-class (M7).** `focus_distance`, `f_stop`, `horizontal_aperture_mm`,
  `vertical_aperture_mm` on `CameraConfig`, bound in both camera layers. Precedence: an explicit
  aperture wins and `fov_deg` is ignored for that axis; absent it, the historical
  `2*focal*tan(fov/2)` derivation is unchanged. Setting both an explicit aperture *and* a
  non-default `fov_deg` warns and names the winner, reusing the existing override-collision
  pattern. `f_stop` defaults to 0, which in USD means depth of field **off** — documented, because
  someone setting it expecting a photographic effect needs to know that.
  Live-verified on the prim: `focusDistance = 1250.0`, `fStop = 2.8`, `horizontalAperture = 36.0`
  with the explicit override beating `fov_deg`.

- **2026-09-06 (c)** — Two more M7 items, and **M5's headline feature: `capture_frame` works**.
  1431 -> 1458 tests. **The dead-config-key list is now EMPTY.**

  **`stage_units_in_meters` wired up**; warns loudly when set to anything but 1.0, because the
  whole pipeline treats one unit as one metre and a different value silently rescales the world
  relative to poses. Live-verified: `set stage metersPerUnit to 1.0`, silent at the metric default.

  **`ros2.use_sim_time` removed after proving it inapplicable.** `ROS2Context` has only
  `domain_id` and `useDomainIDEnvVar`; `use_sim_time` exists solely as an *rclpy node parameter*,
  and we run no rclpy nodes inside Isaac. Our publishers already stamp real simulation time via
  `IsaacReadSimulationTime` -> `SecondsToRosStamp`, so the flag was redundant by construction.
  Deleted rather than faked. **Migration note:** the config models are `extra="forbid"`, so an old
  user TOML containing `ros2.use_sim_time` will now fail validation with a clear error.

  **MAVLink pose source, as a host-side bridge.** The measurement that shaped the design:
  `pymavlink` 2.4.47 is importable on host Python 3.10 but **not** inside Isaac's 3.12. So MAVLink
  is decoded on the host and re-emitted as our existing 51-byte UDP packet -- the simulator needs
  no MAVLink knowledge and no new dependency, and `pymavlink` stays optional and lazily imported.
  Handles the unit traps explicitly (`GLOBAL_POSITION_INT` is degrees x 1e7 and millimetres;
  `ATTITUDE` is already NED radians) and refuses to emit until both a position *and* an attitude
  have arrived -- otherwise the first packet would place the aircraft at lat 0 lon 0, a real spot
  in the Atlantic.

  **`capture_frame` (M5).** Two genuine bugs found in my own first attempt, both by live testing:
  1. I pumped frames inside a main-thread task that the loop was already draining -- re-entrant
     `update_app()`, which simply hung. Restructured as a **loop-driven state machine** (start ->
     settle -> await_file -> finish), one transition per frame, reusing the pattern that worked for
     the gimbal. No re-entrancy possible by construction.
  2. Capturing the *active* viewport fails headless: Kit reports "Capture of LdrColor was
     requested, but no valid resource!". The camera layer's **render product** is the thing
     actually rendering (it feeds the image topic), so the capture now targets that, read off the
     graph rather than guessed.
  Two smaller ones: Kit names captures after the render variable, so `shot.png` landed as
  `shot_LdrColorSD.png` -- now renamed into place, since a caller that asked for `shot.png` should
  be able to open `shot.png`. And resizing the *viewport widget* did nothing to the captured size;
  the **render product's** resolution is what matters, which is why a 4K request first produced a
  720p file.
  **Live proof:** requests for viewport-size, 3840x2160 and 640x480 produced files measuring
  exactly 1280x720, 3840x2160 and 640x480, with the viewport correctly restored afterwards.
  Capture at an arbitrary resolution is a capability 2023 did not have.

- **2026-09-06 (d)** — **M5 substantially done**: `set_pose`, `reset` and `config.patch` join
  `capture_frame` and `set_gimbal`. 1458 -> 1469 tests.

  **`set_pose` sends a UDP packet to our own port rather than writing the prim.** Writing the prim
  is the obvious implementation and does not work: the pose graph rewrites that transform every
  frame from whatever the receiver last held, so a direct write is gone within one frame. (I hit
  exactly this while testing the distance sensor -- a manually-set transform appeared frozen because
  the graph kept overwriting it.) Feeding the receiver means the pose persists precisely as if it
  had arrived over the wire, and the usual last-packet-wins rule applies, which is documented.
  Encodes via `contracts.packet` (spec + checksum), so `sim` still does not import `protocol` and
  the layout cannot drift from the receiver's.
  **Live proof:** alt 1000/1500/2000 produced translate z of 483.3/983.3/1483.3 — each exactly
  `alt - 516.7`, the ENU reference altitude.

  **`reset` has a deliberately narrow, stated scope**: restart the timeline (so simulation time,
  and therefore every published `header.stamp`, returns to zero) and drop any commanded gimbal
  target. It does **not** reload the stage, for the same reason `load_scene` is deferred.

  **`config.patch` is an explicit allowlist, not a deep merge, and that is the design.** Almost all
  config is applied *once* at composition -- a binding writes it to a prim and the value is never
  read again -- so "patching" it would update the config object while the stage kept the old value:
  a silent lie, worse than refusing. Only fields something re-reads while running are accepted,
  which today is exactly `gimbal.max_rate_deg_s`. Anything else is refused with a message that
  explains why and names the restart. Frozen models are respected by keeping patches in a
  `_config_overrides` dict consulted ahead of the model.
  **Live proof:** patching 20 -> 90 deg/s was accepted (reporting previous and new) and the next
  commanded slew covered 90 deg in ~1.1 s, against the ~4.5 s the old rate would have taken;
  `sim.headless` was refused with the explanation.

  **`load_scene` now registered with an honest error** instead of no handler at all -- an
  unregistered method reads as a client/server version mismatch. The reason is concrete rather than
  hand-waved: swapping the stage means closing one that Cesium, the ROS bridge and the action graphs
  all reference, and every stage-lifecycle shortcut in this project so far has produced a silent
  abort rather than an error (the startup warm-up frames exist because of that).

  README corrected: the deferred list is now exactly `features.enable/disable` and `load_scene`.

- **2026-09-06 (e)** — **M6 (swarm) works**, plus renderer validation. 1469 -> 1499 tests.

  **Two vehicles fly independently, verified live.** The architecture was built for this but had
  never been run, and the gap was 27 hardcoded `next(iter(config.vehicles))` assumptions across
  four modules. The config layer was already multi-vehicle ready (`resolved_udp_port(vehicle_id)`
  etc.); everything above it silently used the first vehicle.
  - **Configurator resolvers are now vehicle- and camera-scoped**, threaded explicitly with no
    module-level "current vehicle" state.
  - **`PlannedLayer` carries the identity it was planned for** (`instance`, `camera`). This is the
    key change: `compute_writes` reads it *per layer*, so each vehicle's camera layer resolves its
    own port and topics. Taking one identity for the whole call would have given every vehicle the
    first vehicle's values -- the same class of bug as the single-vehicle assumption, just moved.
  - **The launcher plans once per vehicle** and merges, so each aircraft mounts its own layer copy.
  - **Control methods take an optional `vehicle`**, defaulting to the first, and reject an unknown
    id by listing the real ones rather than silently acting on the wrong aircraft.
  - **Single-vehicle output is byte-identical**, verified explicitly: `/isaac_core/image_rgb`,
    `/isaac_core/global_pose`, `/stream`, port 33333. That was the compatibility constraint that
    mattered most, since every existing consumer depends on it.

  **Live proof** with `lead` + `wing`: both camera layers composed; `set_pose` to 1500 m and 900 m
  gave translate z of **983.3** and **383.3** (each `alt - 516.7`) on separate mounts
  `/World/Environment/lead/Xform` and `/World/Environment/wing/Xform`; ROS advertised
  `/isaac_core/lead/global_pose`, `/isaac_core/lead/image_rgb`, `/isaac_core/wing/global_pose`,
  `/isaac_core/wing/image_rgb`; an unknown vehicle was rejected with the configured list.
  Also fixed the startup report, which printed `camera_udp` twice with no way to tell the copies
  apart -- it now names the vehicle when a layer id appears more than once.

  **`sim.renderer` validated (M7).** An unknown value is rejected at config load with the valid
  options listed, instead of being passed to Isaac to fail later. `MinimalRendering` validates but
  **warns**, because it draws no Cesium terrain and the symptom -- an empty but otherwise healthy
  scene -- is indistinguishable from a broken tileset URL or a missing extension. Warn rather than
  raise: profiling without terrain is legitimate. Matching is case-insensitive with a `Minimal`
  alias, and a README troubleshooting entry covers it.

- **2026-09-06 (f)** — Silent boot, and the **M8 audit found real dead code**. 1499 -> 1512 tests.

  **Boot noise 77 -> 30 lines, zero Warp mentions.** Two levers, both gated behind
  `logging.isaac_logs` so `isaac_logs = true` still shows everything: `warp.config.quiet = True`
  set before `SimulationApp` is constructed, and a POSIX fd-1 redirect around **only** the
  construction call. Deliberately leaves **fd 2 untouched**, so a crash or traceback still
  surfaces -- a silent boot that also hides a failure would be much worse than a noisy one, and the
  test that matters most asserts stdout is restored even when construction *raises*.

  **The exit-gate audit did its job and found two genuinely dead things.** I had it write guards
  and report rather than remove, and told it not to weaken a guard to go green -- it correctly left
  two tests failing with real findings:
  - `read_prim_attribute` was registered on the control plane with **no caller anywhere in the
    shipped code**. My own KIRO note had claimed the inspector used it; it used only
    `get_runtime_values`. I have been calling it constantly from throwaway verification scripts,
    which is exactly how a capability ends up feeling used while shipping as dead surface. Rather
    than delete something genuinely useful, wired it into `isaac-core-inspect --read PRIM:ATTRIBUTE`
    (repeatable, and reports `<not found on the stage>` distinctly from an attribute that holds
    nothing). Live-verified: `focalLength = 22.788`, `horizontalAperture = 36.973`, bogus prim
    reported as not found.
  - `require_prim`, `find_prim` and `StagePrimMissingError` in `sim/stage.py` were exported and
    documented but never called -- the codebase uses inline `GetPrimAtPath(...).IsValid()` instead.
    All three removed. `UsdStageInspector` in the same module is used and stays.

  **One audit finding was wrong, and checking mattered:** it reported that README/KIRO claim the
  `isaac_core_ogn.sensors` extension is deleted. The actual line says a stale *symlink* was removed
  from `extsUser`. Had I acted on it I would have "reconciled" a doc that was already correct.

  New guards that now hold the line: every `Method` enum member has a registered handler; every
  registered handler is reachable from the devkit, CLI or a shipped tool; no `NotImplementedError`
  the README implies works; no public symbol in `src/` without a consumer.

- **2026-09-06 (g)** — **Tile-streaming hitch root-caused, and my first answer was wrong.**
  1512 -> 1518 tests.

  Ofer suggested the Cesium base-URL work might still be outstanding and pointed at 2023's
  `sim_app`. It was already done -- but reading 2023 was still worth it, because **2023's approach
  would break our scene**: it *constructed* `f"{base}/{prim.GetName()}/tileset.json"` rather than
  preserving the authored path. Our tileset prim is named `Cesium_Tileset` while its path is
  `/nablus/`, so that formula yields a 404. Their convention only worked because their prims were
  named after their tile directories. Ours swaps scheme+host+port and keeps the rest, which is what
  Ofer actually described.

  **The hitch experiment, and the correction.** First run looked decisive: with defaults
  (20 concurrent loads) the worst frame was **6.86 fps** with 3/320 under 20 fps; dropping to 4
  concurrent loads gave a worst frame of 38.5 fps and 0/320 slow. A 5.6x improvement -- except the
  phases ran sequentially, so I re-ran with the **order reversed**. That flipped the result
  completely: 4 loads *first* gave 7.02 fps worst and 2/320 slow, while 20 loads *second* gave
  59.83 fps worst and 0/320. **Whichever phase runs first is slow, regardless of its settings.**
  The cause is cold-cache streaming, not load concurrency.

  Recording this because the near-miss is the lesson: the first experiment produced a clean,
  large, plausible effect and a tidy causal story, and it was an artefact of ordering. The only
  reason it did not become a shipped "fix" plus a confident changelog entry is that the confound
  was written into the plan before the result was seen.

  **Deliverable, honestly scoped.** Exposed the Cesium tunables as config
  (`max_simultaneous_tile_loads`, `max_screen_space_error`, `max_cached_bytes`,
  `preload_ancestors`, `preload_siblings`), each defaulting to `None` = leave Cesium's own default
  alone, so tuning no longer needs a `prim_override` and we do not freeze values Cesium may improve.
  Applied to every tileset prim under the tilesets root, skipping non-tileset prims.
  **Did not change any default**, because the measurement says concurrency is not the cause.
  Documented in `config/default.toml` and a README troubleshooting entry that states plainly what
  helps (keep the cache, raise `max_cached_bytes`, warm the route) and what does not (lowering
  concurrency), including why the obvious-looking result is misleading.

- **2026-09-06 (h)** — **M8 closed. Eyes-on harness built.** 1518 tests.

  **M8 validated:** 9 guards pass, `_KNOWN_DEAD` is empty, no stale bytecode, and exactly three
  `NotImplementedError`s remain -- `load_scene`, `enable_feature`, `disable_feature` -- each with a
  concrete reason and each guarded against the README overclaiming. Also removed
  `DEFAULT_RTP_VIDEO_PORT`/`DEFAULT_RTP_META_PORT`, dead since native RTSP replaced the sidecar:
  their only reference was a test asserting their own values, which is a test of nothing.

  **`Sim.launch` could not enable features or declare vehicles** -- it only accepted host, port,
  scene and headless. Found while writing the eyes-on script, which is exactly the kind of gap Ofer
  hoped that exercise would expose. Added an `overrides` parameter taking dotted config keys, with
  the explicit arguments applied *last* so `headless=True` cannot be silently contradicted by an
  override.

  **`scripts/eyes_on_check.py`**: eight scenarios, each launching the GUI **through the devkit**
  rather than the control plane, so running one also exercises the public API a user would write.
  Each prints what to look for before it starts, so the checking criteria are on screen while the
  thing is happening rather than in a document.

  **Process note, corrected by Ofer twice and worth stating loudly.** Two separate incidents:
  a verification run was SIGKILLed, and a later one **hung overnight without the `timeout` ever
  firing**, so Ofer had to kill it by hand.
  Root cause of the hang: we pass `--/app/installSignalHandlers=0` to Kit, so **Isaac ignores
  SIGTERM** -- which is exactly what plain `timeout N` sends. `timeout` therefore cannot kill an
  Isaac process at all. Every Isaac invocation must use **`timeout -k 5 N`** (or `-s KILL`) so a
  SIGKILL follows the ignored SIGTERM. The standing "prefix every command with `timeout N`" rule was
  necessary but not sufficient, and the gap cost Ofer a night of a pinned machine.
  Second cause, for the SIGKILL: an **orphaned Isaac process** from an earlier run was still holding
  memory, so kill strays *before* launching, not only after.
  Third, and the cheapest lesson: the injected `launcher` parameter lets the whole config-building
  path be verified with **no Isaac spawn at all**, which is how the overrides were confirmed to land
  in the written TOML. Prefer that; reserve live runs for what genuinely needs a GPU.

  **Superseded note.** A verification run was SIGKILLed and Ofer flagged that I was
  stuck. Cause: an **orphaned Isaac process** from an earlier run was still holding memory. Two
  lessons: kill strays *before* launching, not only after; and the injected `launcher` parameter
  makes it possible to verify the whole config-building path with **no Isaac spawn at all**, which
  is what I used to confirm the overrides land in the written TOML. Cheap verification first, live
  runs only for what genuinely needs a GPU.

- **2026-09-07** — **Ofer's eyes-on pass found six real bugs; three shared one root cause.**
  1518 -> 1520 tests. The harness paid for itself immediately.

  **Root cause A: a connectable port is not readiness.** `start()` opens the control plane, and
  `open_stage()` composes the stage ~8 seconds later. `Sim.launch` treated the open port as ready,
  so every command issued in that window was accepted and silently dropped -- the graphs that would
  act on it did not exist. This explains three separate symptoms Ofer reported: the gimbal scenario
  where "the camera did not move at all" (its `set_pose` was lost), the swarm scenario where neither
  vehicle appeared to move, and the lifecycle scenario running before the sim was usable. Ofer
  pointed straight at it -- 2023 waited for the first pose before trusting anything.
  Fix: `get_state` now reports `ready` (running **and** stage composed) and `wait_until_ready`
  waits on it, tolerating an older server that cannot answer so a newer client cannot hang.

  **Root cause B: the client socket timeout was 5 s.** `capture_frame` spans several frames
  (resize, settle, async file write) and `step(count=30)` runs 30 -- both legitimately exceed 5 s,
  so they raised `TimeoutError` while the work was still succeeding. Ofer's log shows it plainly:
  the scenario reported failure, and the capture completed *afterwards*. Default is now 60 s.

  **Root cause C: the simulator survived a finished script.** Two facts combined, and the second
  is one I had already documented for my own tooling without connecting it here:
  `python.sh` does not `exec`, so the process we hold is a wrapper and Isaac is its child; and Isaac
  runs with `installSignalHandlers=0`, so it **ignores SIGTERM**. Signalling the wrapper achieved
  nothing. Now spawned with `start_new_session=True` and the whole process **group** is signalled,
  SIGKILL following an ignored SIGTERM. Ctrl-C appeared to work only because the terminal sends
  SIGINT to the whole foreground group already.

  **Root cause D: swarm RTSP port collision.** Both vehicles bound 8554 --
  `Error binding to address 0.0.0.0:8554: Address already in use` -- so the second aircraft had no
  stream. Added `resolved_rtsp_port`, offsetting by vehicle index exactly as pose ports do.
  Verified: single vehicle still 8554, two vehicles get 8554/8555, an explicitly pinned port is
  honoured verbatim.

  **D28 (Ofer's call): out-of-band range readings saturate at the rated limits instead of
  reporting infinity.** ROS convention for `sensor_msgs/Range` is +/-inf, which is what I had
  implemented, and his objection is better: `float("inf")` cannot be cast to `int`, so a consumer
  doing `int(msg.range)` crashes exactly when the sensor sees nothing -- the common case for a
  downward rangefinder in level flight. Every reading is now finite and inside
  `[min_range, max_range]`; a value parked at a limit is itself the out-of-range signal. Cost,
  documented: "exactly at the limit" and "beyond it" are no longer distinguishable, so
  `is_saturated()` exists for consumers that need to know. A test now asserts finiteness across the
  entire input space including inf and nan.

  **Still open: the distance sensor's ray geometry.** Ofer's numbers do not match a
  straight-down ray -- pitch -25 returns a value while pitch -55 returns nothing despite being
  higher above ground, and lowering altitude at -55 restores a reading. That pattern suggests the
  ray direction or its max_t is not what I think. Not yet diagnosed; needs a live geometry probe
  rather than a guess.

  **Harness improvements from the same feedback:** every scenario now calls `_place_and_confirm`,
  which re-sends the pose until `get_pose` shows the expected height -- because UDP is
  fire-and-forget and a lost pose previously masqueraded as a broken gimbal. Scenario 8 prints the
  exact `ffplay` command rather than expecting the reader to find it in the README.

- **2026-09-07 (b)** — **I shipped a regression that broke every scenario, and the suite did not
  catch it.** 1520 -> 1535 tests.

  The `rtsp_port` fix went in as two edits: point the manifests at a `resolve = "rtsp_port"` and
  register that resolver. The second edit was a `str.replace` anchored on `resolved_vehicle_id`
  when the variable is `vehicle_id`, so it **silently matched nothing**. The manifests then asked
  for a resolver that did not exist, and every launch died with
  `ConfigKeyError: unknown resolve name 'rtsp_port'`.

  **Why the suite passed anyway, which is the real lesson.** A manifest's `resolve` name is only
  looked up during `compute_writes`, i.e. at compose time inside a running Isaac Sim. Nothing in the
  unit suite ever resolved the shipped manifests' resolver names, so a dangling reference was
  invisible to 1520 tests and surfaced only on Ofer's machine. Added
  `tests/unit/sim/test_manifest_resolvers.py`: it scans every shipped `layer.toml` for
  `resolve = "..."` and asserts each name resolves, parametrised so a failure names the offender.
  Includes a not-vacuous check so an empty scan cannot pass silently. 14 resolvers covered.

  Two process notes. First: a `str.replace` that does not match is a **silent no-op**, and I used
  one for a change whose failure mode was invisible to tests -- an `assert old in s` before the
  replace would have caught it at authoring time, and I now use one. Second: I claimed the port fix
  was verified because `resolved_rtsp_port` returned 8554/8555 in isolation; that tested the config
  method, not the wiring that consumes it. Verifying the unit I just wrote, rather than the path the
  user takes, is how this reached him.

  Re-verified end to end after the fix: `compute_writes` succeeds for single vehicle (17 writes),
  single + bbox + distance (23), and two vehicles (34, RTSP ports 8554 and 8555); live launches
  reach `simulation running` for both single and swarm configs with **zero** resolve errors and
  **zero** `Address already in use`.

- **2026-09-07 (c)** — **Gimbal pitch/roll swap root-caused and fixed.** 1535 -> 1546 tests.

  Ofer reported commanded pitch coming out as roll, and confirmed it twice: the gimbal scenario
  showed a `start_pitch_deg = -15` appearing as roll 15, and the capture scenario logged
  `pitch 0.0 / roll -40` for a commanded `pitch_deg=-40`.

  **Two compounding causes, neither visible in the existing tests.**
  1. The offset was composed in the *vehicle's* rotation frame, which defaults to **WORLD**, i.e.
     about fixed world axes. A level, north-heading aircraft already carries **ENU yaw = +90 deg**
     (`yaw_enu = -yaw_ned + pi/2`, turning a compass heading into a bearing), and that 90 degrees
     rotates the offset's axes so "pitch" lands on the roll axis. Measured through the full
     transform chain: commanded pitch changed the camera's elevation by **0.00 deg** while
     commanded roll changed it by the full amount. A gimbal is bolted to the airframe, so the
     offset must be **body**-relative regardless of how the airframe's own angles compose.
  2. The offset never received the airframe's NED->ENU sign conventions, so pitch and yaw were
     inverted relative to any pose arriving on the wire -- the same command meant opposite things
     depending on which path set it.

  Fix: compose the offset in `RotationFrame.BODY` with pitch and yaw negated. Verified through the
  full chain (node quaternion -> camera's authored local orient -> world look vector):
  pitch -15 gives elevation **-15.00**, pitch +15 gives **+15.00**, roll +25 leaves elevation at
  **0.00** while tilting image-up by **25.00**, and yaw +60 increases bearing by **60.00**.

  **Why the old tests missed it, which is the transferable lesson.** `compose_rotation` and
  `euler_to_matrix` both had passing unit tests, and the composition *formula* read correctly. The
  bug only existed in the *interaction* between the offset frame and the airframe's built-in 90
  degree yaw -- invisible to any test that checks a matrix multiplication or an Euler round-trip.
  The new `tests/unit/geo/test_gimbal_axes.py` asserts on the **camera's world look direction**
  instead: elevation, bearing and image tilt. It also pins a level aircraft looking *north*,
  because my own first probe assumed nose = body +X, had level flight pointing **south**, and
  produced a confidently wrong conclusion I nearly acted on. Assert on observable outcomes, not on
  the formula that produces them.

  **`Sim.launch` leaked a simulator on Ctrl-C.** Interrupting while "Launching..." was on screen
  left a fully-booted Isaac running, because the session that owns the process was only constructed
  *after* the readiness wait returned. Now built before the wait and closed on `BaseException`
  (deliberately, since `KeyboardInterrupt` is the common case).

  **Still open from Ofer's pass, with what is known:**
  - **Distance sensor ray direction.** His repro is clean: pitch -90, descend N metres, expect the
    range to fall by N. He also observes it "looking forward". Not yet diagnosed.
  - **Swarm viewport fights.** Both vehicles' cameras are named `main_camera_01` and each layer's
    `isaac_set_camera` retargets the *same* viewport, so the view flips between aircraft every
    frame. Needs a per-vehicle viewport.
  - **`resume`/`reset` touch the timeline from the control-server thread**, producing
    `RuntimeError: There is no current event loop in thread 'isaac-core-control-server'`. They must
    go through the main-thread queue as `set_gimbal` and `capture_frame` do.
  - **Leftover `viewport_size_LdrColorSD.png`.** The rename claims the newest match; a stale file
    from a prior run survives.
  - Scenario ergonomics: a linear path reads better than a circle for judging nose alignment, and
    the lifecycle scenario needs louder narration.
  - Ofer's roll-then-pitch-180 observation is the documented Euler coupling limit, not a new bug:
    only the outermost angle is frame-stable, so roll's effect necessarily changes at large pitch.
    Worth restating in the README rather than treating as a defect.

- **2026-09-07 (d)** — **The distance sensor was never broken.** 1546 -> 1561 tests.

  Ofer's odd readings (a value around 900 when looking up, nothing at pitch -55, a reading returning
  when he descended) were **all downstream of the gimbal pitch/roll swap**. His "pitch" commands were
  rolling the aircraft, which swings a body-mounted downward ray sideways. At 55 degrees off vertical
  from 483 m above terrain the ray reaches ground at `483 / cos(55) = 842 m` -- his "around 900".
  Verified the geometry is right: level flight casts the ray at exactly -90 degrees elevation
  (straight down), and pitching nose-down 30 degrees tilts it to -60, which is correct for a rigidly
  mounted sensor.

  **Live proof over the ROS topic**, gimbal at zero, altitude the only variable:
  2000 -> 1482.25, 1600 -> 1082.25, 1200 -> 682.25, 1000 -> 482.17. Every reading is altitude minus a
  consistent ~517.75 m terrain, and a -400 m altitude change produces exactly -400 m of range.
  Lesson: two of Ofer's three reported "distance sensor bugs" were one gimbal bug wearing a disguise.
  Chasing the symptom would have wasted a lot of effort; fixing the upstream cause resolved them.

  **Found while investigating, and it is a real design issue: the gimbal offset rotates the whole
  `/Root/Xform`**, which carries the camera *and* the distance sensor. So aiming the camera also
  swings a fuselage-mounted rangefinder, which is physically wrong. Needs a USD change -- the offset
  should apply to a child Xform holding only the camera -- so it is Ofer's call, flagged not fixed.

  **Swarm viewport fight fixed, the way Ofer suggested.** Root cause read from Isaac's source:
  `IsaacCreateViewport` with an empty `name` and `viewportId == 0` **reuses the active viewport**. So
  every vehicle's layer grabbed the same one and retargeted it to its own camera, flipping the view
  every frame. New `viewport_name` resolver returns `""` for a single vehicle -- byte-identical to
  today, no extra window -- and the vehicle id when there are several. Answering his question
  directly: with two or more vehicles you *do* get one viewport window per vehicle, and that is
  necessary rather than cosmetic, because each vehicle needs its own render product to have its own
  image topic and RTSP stream. `sim.viewport_camera` still aims the main viewport at one chosen camera.

  **Latent bug found by a sub-agent:** `_handle_step` **ignored `count` entirely** and always advanced
  one frame, despite `session.step(count=...)` and the README both documenting it. Now honoured and
  validated. Also fixed: `_handle_pause` and `_handle_resume` called the timeline directly from the
  control-server thread, which is what produced
  `RuntimeError: There is no current event loop in thread 'isaac-core-control-server'` inside Isaac's
  throttling extension. Both now dispatch through the main-thread queue, as `reset` already did.

  Scenario 1 now flies two straight legs (north, then east) with commanded yaw equal to the track, so
  nose alignment is unambiguous -- a circle made it impossible to judge. Scenario 7 announces each
  lifecycle phase and prints the pose before and after, so a frozen viewport is corroborated by
  identical numbers rather than resting on the eye alone; `--dwell` sets the pace.

  Resolved without action: the leftover `_LdrColorSD.png` files were stale from before the rename fix
  and did not reappear, and Ofer confirmed the pitch-180 horizon behaves correctly, so the Euler
  coupling concern was unfounded.

- **2026-09-07 (e)** — **D29: the distance sensor is boresighted with the camera, by construction.**
  Ofer's requirement, stated plainly: the ray should always point at the centre of the camera's view
  and report the distance to whatever is there.

  **The bug this exposed was bigger than the gimbal coupling I had flagged.** The sensor was not
  merely *also* moved by the gimbal -- it was **permanently 90 degrees off the camera**. Measured:
  in level flight the camera looks north while the sensor cast straight down; with the gimbal at
  pitch -30 or yaw +90 the separation stayed exactly 90 degrees. Cause: the camera prim carries its
  own local orientation `(0.5, 0.5, -0.5, -0.5)` -- that is what makes a USD camera look along the
  nose -- while the sensor Xform was identity. Both inherited the same mount, so they moved
  together, but 90 degrees apart. My earlier framing ("the gimbal shouldn't move the sensor") had
  the relationship backwards: the sensor was never aimed where I assumed.

  **Fix: cast along the camera prim's own local -Z.** The node takes a `cameraPath` token input,
  written by the compositor from the `camera_prim` resolver -- the same pattern the bbox projector
  uses, and for the same reason: the camera lives in another layer, so a USD relationship cannot be
  authored in the GUI. Boresighting is now structural rather than a value someone has to keep in
  sync; if the camera's orientation ever changes the ray follows automatically. The separate
  `sensorPrim` input and the `/Root/Xform/distance_sensor` prim are gone.

  **Live proof** with the camera aimed straight down (gimbal pitch -90): range 982.25 / 682.25 /
  482.17 at altitudes 1500 / 1200 / 1000 -- ground clearance to within a metre in every case.
  Camera level at the horizon: 5000.0, the saturated maximum, correct since nothing is within 5 km
  along a horizontal ray, and finite as D28 requires. Camera 45 degrees down from 1200 m: 801.4 m,
  shorter than the 966 m a flat-terrain slant would give, which is consistent with the ray meeting
  rising ground (566 m out, terrain ~634 m against 518 m at the origin -- an 11.5 degree slope, very
  plausible for this scene). I have not independently profiled the terrain to confirm that last one.

  Also settled: no separate gimbal Xform is needed, and the option-B restructure that would have
  moved the camera prim -- breaking `sim.viewport_camera`, the `camera_prim` resolver, the bbox
  `cameraPath` and the capture path -- is avoided entirely.

- **2026-09-07 (f)** — **Swarm fixed properly; three symptoms, one cause.** 1561 -> 1569 tests.

  My previous swarm fix named the *created* viewport per vehicle but missed that two other nodes
  look a viewport up **by name and default to the active one when unset**:
  `IsaacGetViewportRenderProduct` and `IsaacSetViewportResolution`. So the second vehicle's getter
  still asked for the active viewport, came back with an empty render product
  (`Render product '' not valid`), and with no product of its own both RTSP servers fell back to the
  same one and fought over port 8554. Naming only half the chain was worse than not naming it.

  Binding `inputs:viewport` on both, from the same `viewport_name` resolver, fixed all three
  symptoms at once -- and one I had not connected to it: **`lead`'s translate read 168.3 instead of
  983.3**. That was not a pose bug at all; it was downstream of the shared-render-product confusion,
  and it disappeared with the viewport fix. Worth recording, because I had it queued as a separate
  "HIGH" investigation and it would have been wasted effort.

  Live: **0** RTSP bind collisions, `lead` 983.3 and `wing` 383.3 (both exact), and -- the real
  proof each vehicle owns its own render product -- `/isaac_core/lead/image_rgb` at 25.8 Hz
  *and* `/isaac_core/wing/image_rgb` at 24.2 Hz simultaneously. One transient empty-render-product
  warning remains during startup ordering, before the named viewport exists; harmless given both
  topics then publish steadily, but not chased.

  **Two lifecycle bugs from Ofer's pass, both real:**
  - `step(count=N)` **did nothing while paused**, which is exactly when stepping is useful.
    `update_app` renders a frame but does not advance a paused timeline. Now calls
    `timeline.forward_one_frame()` per step when not playing, then renders.
  - `reset` left the simulation **stopped**. Kit ignores `play()` issued in the same frame as
    `stop()`, so the play was silently dropped. Deferred to the next loop iteration via a
    `_pending_play` flag the loop honours -- the same deferral pattern the capture state machine
    uses, and for the same underlying reason.

  Confirmed not a bug: no motion while running is correct -- `set_pose` sends one packet and the
  receiver holds the last good pose by design, so nothing moves unless something keeps sending.

  Unreproduced: one gimbal run in four where the offsets appeared to arrive only at the end and
  pitch never moved. Not diagnosed and not guessed at; if it recurs the thing to capture is whether
  `gimbal target set to ...` appears in the log at the moment each command is issued.

- **2026-09-07 (g)** — **Naming viewports was the wrong fix; offscreen render products are the right
  one.** Scenario 7 confirmed working by Ofer.

  The viewport bindings are provably correct -- `compute_writes` for two vehicles puts `'lead'` on
  lead's three nodes and `'wing'` on wing's, on the right prims -- yet at runtime Ofer still saw a
  **single** viewport named `wing` serving both cameras, and once a black unresponsive window. So
  Isaac is not honouring the name reliably. Viewport windows are a GUI concept and creating them
  from a graph during startup is fragile; I was fixing the wrong layer of the problem, twice.

  `isaacsim.core.nodes.IsaacCreateRenderProduct` is the intended mechanism -- its own description
  says "for use with **offscreen rendering**". It takes `cameraPrim`, `width` and `height` directly
  and emits `renderProductPath`, so it replaces **four** nodes (`isaac_create_viewport`,
  `isaac_get_viewport_render_product`, `isaac_set_viewport_resolution`, `isaac_set_camera`) with one,
  needs no window, and gives each vehicle its own product by construction. Written up as build-sheet
  T1 for Ofer, explicitly gated on me switching the manifest first so there is no broken
  intermediate state -- the manifest and the USD have to move together and only he can author USD.

  Note this also simplifies the single-vehicle path: fewer nodes, no viewport dependency for the
  image topic, and the GUI's own view stays under `sim.viewport_camera` as before.

  **Scenario 7 now passes.** `step` advances while paused, `reset` leaves the simulation running,
  pause and resume work. Ofer verified each phase against a ROS topic rather than the viewport, which
  is the more reliable check and is worth keeping in the scenario notes.

- **2026-09-08** — **Swarm root cause found: one unconnected input. My previous two diagnoses were
  both wrong, and Ofer's push-back is what corrected them.**

  He asked why the image-export graph needed rewriting at all, and added the detail that **two
  correctly-named viewports do open** -- both merely showing the same fight. That contradicted my
  "Isaac does not honour viewport names" theory, so I measured instead of theorising again.

  Measured at runtime with two vehicles:
  - `isaac_create_viewport.inputs:name` = `'lead'` / `'wing'` -- correct.
  - `isaac_get_viewport_render_product.inputs:viewport` = `'lead'` / `'wing'` -- correct.
  - resolved `outputs:renderProductPath` = **two distinct products** (`ViewportTexture_1` and
    `ViewportTexture_2`). So the whole viewport chain works.
  - **`isaac_set_camera.inputs:renderProductPath` = `''` for both vehicles.**
  - Both render products' `camera` relationship still pointed at `/OmniverseKit_Persp`.

  So: `IsaacSetCameraOnRenderProduct` had `inputs:renderProductPath` **declared but never
  connected**, fell back to the **active viewport**, and both vehicles wrote their camera there every
  frame -- the flicker. Neither named viewport ever received its camera at all. The two helpers
  beside it (`ros2_camera_helper`, `rtsp_camera_helper`) *are* connected to the same getter output;
  only this one was missed.

  **The fix is one connection per camera layer.** The render-product rewrite I had written up as a
  build-sheet task -- replacing four nodes, gated on a manifest change -- was unnecessary and has
  been retracted. Ofer would have done a substantial USD restructure for nothing had he not asked.

  **Guard added** to `test_usd_graph_wiring.py`: any node declaring `inputs:renderProductPath`
  without connecting it now fails, naming the node. Same failure shape as the unwired `execIn` that
  file already guards -- an input that goes quiet instead of complaining -- and the third time this
  repo has been bitten by exactly that shape (`ros2_publisher.execIn`, `header:stamp`, now this).
  Currently failing on both camera layers by design, until the connection is authored.

  **Process note.** Twice I proposed a fix from a plausible mechanism rather than from measurement,
  and both times the measurement said something different. The pattern to keep: when a symptom
  survives a fix, re-measure the assumption the fix rested on instead of escalating the size of the
  change. Escalating is what produced a four-node rewrite proposal for a one-line bug.

- **2026-09-08 (b)** — **The swarm blocker is a bug in NVIDIA's own node.** Ofer's Isaac error
  supplied the missing evidence.

    'OgnIsaacGetViewportRenderProductInternalState' object has no attribute 'viewport'

  In unmodified Isaac source, `OgnIsaacGetViewportRenderProduct.py`:

      class OgnIsaacGetViewportRenderProductInternalState:
          def __init__(self) -> None:
              viewport = None          # local variable, should be self.viewport

  The state object never gets the attribute. `compute` creates it only when
  `get_viewport_from_window_name` succeeds, then reads it unconditionally -- so a window that is not
  registered yet turns a benign "not found" warning into a hard `AttributeError` that kills the node.
  `lead` wins the creation race, `wing` does not. Everything else follows: `wing`'s render product is
  empty, hence `Render product '' not valid`, RTSP `skipping setup`, and both streams falling back to
  port 8554 with `Address already in use`.

  We may not patch Isaac's files, so the fix is to stop using that node.
  `IsaacCreateRenderProduct` takes the camera and resolution directly, creates an **offscreen**
  product, does no name lookup (no race), initialises its state correctly (`self.render_product_path`),
  and reuses an existing product for the same camera+resolution.

  **Honest accounting of my three diagnoses.** (1) "viewport names are not applied" -- wrong, they
  are, measured. (2) "`isaac_set_camera.renderProductPath` is unconnected" -- true and worth fixing,
  but not the cause; the getter was already dead upstream of it. (3) This one, which is grounded in
  NVIDIA's source and Ofer's traceback rather than in inference. My original instinct to replace the
  viewport chain was right, for entirely the wrong reason -- and I only reached the real reason
  because Ofer pushed back twice and supplied an error I had not seen. The pattern worth keeping:
  a symptom that survives two fixes usually means the failing component is upstream of where I am
  looking, and an unread stack trace beats any amount of reasoning about mechanism.

  The `renderProductPath` guard added yesterday stays useful: after the swap the two helpers still
  declare that input and must be connected to the new node, and the guard will catch it if not.

- **2026-09-08 (c)** — Ofer authored the render-product swap; manifest switched. **Single-vehicle
  path verified unregressed. Swarm image topics remain open.**

  His USD edits verified: `IsaacCreateRenderProduct` present in both camera layers with `execIn`
  wired and `cameraPrim` -> `/Root/Xform/main_camera_01`; all four viewport-era nodes deleted
  (`isaac_create_viewport`, `isaac_get_viewport_render_product`, `isaac_set_viewport_resolution`,
  `isaac_set_camera`); both helpers reconnected to the new node's output. 42 USD guards pass.

  My side: dropped the five now-dead viewport bindings per layer, moved `width`/`height` onto the new
  node, deleted the `viewport_name` resolver (no consumer), 20/21 -> 17/18 bindings. A guard I had
  forgotten -- `binding_prim_exists_in_usd` -- caught the stale bindings before I did, which is
  exactly what it is for.

  **What the swap fixed.** All four failure classes are now **zero** in a two-vehicle run:
  `'...' object has no attribute 'viewport'`, `Render product '' not valid`,
  `renderProductPath is empty`, and `Address already in use`. The NVIDIA node bug is out of the path.

  **What is still broken.** In a swarm neither `/isaac_core/lead/image_rgb` nor
  `/isaac_core/wing/image_rgb` appears, even after 40 s. Single vehicle **does** publish
  `/isaac_core/image_rgb`, so the main path is unregressed and this is specific to multi-instance.
  One clue from Ofer's earlier log worth following: `SdRenderVarPtr missing valid input renderVar
  rep_LdrColor_...`, which suggests an offscreen render product may need its colour render var
  explicitly realised in a way a viewport-backed one got for free. Not investigated.

  **Stopping here deliberately.** Ofer flagged that I was stuck, and he was right: I had made three
  wrong diagnoses of this subsystem and was starting a fourth investigation while two orphaned Isaac
  processes from a hung in-process probe were still holding GPU memory and making runs flaky. Also
  learned: opening the scene in-process after a full app load hangs, so live verification of this
  subsystem must go through `python.sh -m isaac_core.sim`, not an embedded probe.
  Handing the remaining gap over with the clue above rather than guessing again.

- **2026-09-08 (d)** — **Swarm works. My "image topics don't publish" call was wrong.**

  Ofer verified the ROS topics via rqt and the pose sender: everything present and working. My
  earlier conclusion came from `ros2 topic hz` checks run too early, while **two orphaned Isaac
  processes** from a hung probe were still holding GPU memory. Two measurement errors compounding --
  and I reported the result as a product defect. The startup `Render product '' not valid` /
  `renderProductPath is empty` warnings are **transient**: they fire on the first tick before the
  render product exists, and the helpers set up correctly a tick later. Noisy, not broken.

  **Viewport**: confirmed deterministic. It follows the first vehicle *declared in config* --
  `lead` in Ofer's file -- and TOML preserves declaration order, verified stable across repeated
  loads. He is happy with a single viewport, so nothing to change; now documented in the README
  rather than left as implicit behaviour.

  **RTSP collision**: the bindings are correct (measured: `lead` 8554 `/lead/stream`, `wing` 8555
  `/wing/stream`), `inputs:port` is declared, and no write warnings appear. NVIDIA's own
  `rtsp_writer.py` docstring confirms the design -- "Each simultaneous stream needs a unique port" --
  so per-vehicle ports are right. The `Address already in use` on 8554, together with
  `no factory for path /lead/stream`, points at a **stale RTSP server from an earlier run** still
  holding the port; 8554 was free when checked afterwards. Consistent with Isaac ignoring SIGTERM.
  Added a README troubleshooting note with the `pgrep`/`kill -9` check rather than treating it as a
  code defect, since the evidence does not support one.

  **Isaac-side, not ours**: opening the action-graph editor for a *second* camera layer crashes the
  GUI. Both layers reference the same source USD at different mounts, so this is likely Isaac's graph
  editor mishandling two instances. Nothing in our code participates; recorded so it is not
  re-investigated as a regression.

  **Standing lesson from this stretch.** Three of my swarm diagnoses were wrong, and every one failed
  the same way: I measured, drew a conclusion, and did not check whether the measurement itself was
  sound. Orphaned processes, checks run before DDS discovery, and an in-process probe that hangs all
  produced confident but false readings. Before reporting a defect from a live run: confirm no stray
  processes, confirm the check waited long enough, and prefer the launcher over an embedded probe.

- **2026-09-08 (e)** — **Removed the two deferred methods rather than shipping them broken.**

  Ofer's call, and the right one: v2 now ships nothing that is known not to work. Deleted
  `load_scene`, `enable_feature` and `disable_feature` -- three `Method` enum members, three control
  plane registrations, three runtime handlers, the devkit `_FeatureManager` proxy and its `features`
  property. Roadmapped as two-liners under "Live stage manipulation (lowest priority)" with the
  reason a future implementer needs: every stage-lifecycle shortcut here produced a *silent abort*,
  so both need a crash-rate harness before being called done.

  Two things the removal surfaced. My `cut` helper removed `_ConfigProxy` along with
  `_FeatureManager` because they were adjacent -- caught by the suite, restored from
  `git show HEAD:`, which is the argument for the file being tracked. And a docstring on `Sim`
  claimed `Sim.launch` "currently raises NotImplementedError for the actual process spawn", which
  has been false for a long time; `launch` works. Exactly the class of rot Phase 2 exists to remove,
  found early by accident.

  Adjusted `test_the_scanners_found_something`: it asserted deferred handlers *exist*, which was a
  reasonable parser self-check when some did and a guaranteed failure once none do. Now asserts the
  scanner returns a set, and the README-honesty guard stays live for anything added later.

  **1558 tests pass** (two fewer, being the tests for the removed features), all gates green,
  23 contracts kept.

- **2026-09-08 (f)** — V2 finalization planned; style profile put to Ofer before touching style.

  Plan in `dev/v2_finalization_plan.md`: six phases -- freeze the claim surface, split docs by
  audience, source cleanup, verify every feature, repo hygiene, release gate. Ofer added that
  Phase 0's matrix must be **re-run at the end** so cleanup cannot quietly break something the
  matrix already certified, and dropped the CONTRIBUTING.md in favour of a fresh-clone check on a
  different machine with a fresh Isaac install.

  He corrected me on style: I had proposed taking docstring inspiration from 2023, whose docstrings
  are lowercase fragments (`"""this file defines the sim app class"""`). He said it should be the
  industry standard instead, and he is right -- that is PEP 257, capitalised imperative sentences.

  Measuring to answer him found something worse in *our* config: `pyproject.toml` explicitly ignores
  **D212**, so we enforce summaries on the **second** line, the opposite of PEP 257 and Google style.
  538 multi-line docstrings are affected. `ruff --fix` autofixes D212, so the correction is mechanical
  rather than a hand edit. Question A1 in `docs/dev/style_questions.md`, recommending we switch.
  I set that ignore originally and should have flagged it as non-standard at the time.

- **2026-09-08 (g)** — Style profile answered; **every lint suppression removed from `src` and
  `scripts`**; docstrings switched to PEP 257.

  **Docstrings (A1).** Removed `D212` from the ignore list. The pair `D212`/`D213` is mutually
  exclusive -- you must ignore exactly one -- so this was necessarily a swap, and `D213` went in with
  a comment recording why. `ruff --fix` corrected **582** docstrings to first-line summaries.

  **`Any` (A2, and Ofer's steer).** He said he is fine with `Any`, so `ANN401` went into the ignore
  list and 103 inline suppressions came out. Measured first, because the recommendation depended on
  it: 65 were JSON-RPC/config boundaries where a `JsonValue` alias would have been more precise, and
  34 were Isaac APIs that ship **no type stubs**, where `Any` is the only honest annotation. Given a
  third of them cannot be improved at all, the global ignore is the consistent choice.

  **The big finding: most suppressions were already dead.** `PLC0415` (45 uses) is a **preview-only
  rule and was never active** -- ruff says so directly: "Selection `PLC0415` has no effect because
  preview is not enabled". Then `RUF100` (unused-noqa) found **51 more** dead directives. So 91 of
  the ~190 suppressions were protecting against rules that never fired. They accumulated because
  nothing checked them, which is the argument for enabling `RUF100` permanently.

  **Real fixes, not suppressions**, for the rest:
  - 5 `assert x is not None  # noqa: S101` in `control/server.py` became three real guards raising
    `RuntimeError("control server is not running")` -- mypy is satisfied *and* a genuine failure now
    reports itself instead of vanishing under `-O`.
  - `key.fileobj` narrowed with `isinstance` rather than `# type: ignore[assignment]`.
  - 16 **stdlib** modules (`os`, `subprocess`, `pathlib`, `tempfile`, …) were being imported inside
    functions for no reason; hoisted to module top. The genuinely lazy ones (`rclpy`, `tkinter`,
    `cv2`, `pymavlink`) stay lazy -- that is the three-interpreter design, not an oversight.
  - `_launch_simulator(install: object, *, config: object)` and `_configure_logging(config: object)`
    were annotated `object`, which *forced* three `attr-defined` ignores. Both attributes existed all
    along. Annotated properly via `TYPE_CHECKING`, so the ignores went and real checking arrived.
  - `prim.IsValid()` / `prim.HasAttribute()` wrapped in `bool()` instead of `no-any-return` ignores.
  - `sim/__main__.py` had five `# type: ignore[name-defined]  # noqa: F821` pairs for forward refs
    with no `TYPE_CHECKING` block. Added the block; all ten suppressions went.

  **Result: `src` and `scripts` contain zero `# noqa` and zero `# type: ignore`.** Remaining are 29
  in `tests` and 9 in `extensions`, both areas Ofer explicitly exempted. `N802` moved to the existing
  `tests/**` per-file-ignores with a note -- fakes for Pixar's USD API must copy its PascalCase names
  exactly or they are not substitutable.

  One self-inflicted break worth recording: deleting `import tkinter as tk` removed the *body* of the
  `try:` that tests tkinter availability, producing an `IndentationError`. The suite caught it
  immediately. Its `F811` suppression turned out to be stale anyway.

  **1558 tests, all seven gates green, 23 contracts kept.**

- **2026-09-08 (h)** — Log moved here from `KIRO.md` at Ofer's request (B10), since a 3,300-line
  engineering log is the first thing a newcomer opened after the README and none of it is
  user-facing. All six referencing files updated. Also recorded: emoji allowed in the README only,
  at most one per section heading (B1); Ofer shoots all screenshots during the Phase 1 review, so the
  build sheet now lists seven with exact `docs/images/<name>.png` filenames the README will
  reference (B7); no `CONTRIBUTING.md`; and the fresh-clone check happens on a **different machine
  with a fresh Isaac install**.

- **2026-09-08 (i)** — `RUF100` enabled (approved); test ignores audited; **Phase 0 complete**.

  **Test ignores: nothing stale to remove.** Ofer's distinction was that permitting ignores in tests
  does not mean permitting *stale* ignores. Checked properly rather than assumed:
  `warn_unused_ignores = true` was **already** set in `[tool.mypy]`, and mypy reports **0 unused**
  across `tests/`. Verified the hook genuinely covers tests by injecting a type error into
  `tests/unit/control/test_messages.py`, confirming mypy caught it, then restoring the file -- without
  that check "0 stale" could equally have meant "not checked". All 29 are load-bearing, and all 29
  carry a specific error code; there are no bare `# type: ignore`. So the staleness guard for
  `type: ignore` already existed, and `RUF100` now provides the matching guard for `# noqa`.

  Proved `RUF100` is live by planting a dead `# noqa: E501` in `contracts/topics.py` and confirming
  ruff flagged it, then restoring. A rule believed-enabled is worth nothing.

  **Phase 0 complete: `dev/feature_matrix.md`.** ~90 rows over ten areas, each with a verification
  bucket (A automated / B live-scripted / C eyes-on) and a cited evidence file. All 58 cited test
  files verified to exist, mechanically, rather than trusted.

  Five findings, none a crash:

  - **F1** — `get_runtime_values` and `read_prim_attribute` are registered handlers called by
    `isaac-core-inspect` with raw strings and are absent from the `Method` enum. The coverage guard
    passes *correctly* because both are reachable, but the enum is documented as the control-plane
    surface, so a user reading it gets an incomplete answer. `read_prim_attribute` is not documented
    anywhere at all.
  - **F2** — four dead topic constants in `contracts/topics.py`: `SAT`, `RAW_RGB`, `MAVROS_LLA`,
    `MAVROS_ORIENTATION`, all confirmed at **zero** references outside their own module. The MAVROS
    input path itself works -- `_resolve_mavros_topic` builds those names dynamically per vehicle --
    so only the constants are dead. The dead-symbol guard does not reach module-level string
    constants, which is why they survived.
  - **F3** — the README-honesty guard is now vacuous, since nothing raises `NotImplementedError`.
    Keeping it: it costs nothing and goes live again the moment a capability is deferred.
  - **F4** — bucket B has no runner. Most live rows are marked pass from ad-hoc runs recorded here,
    not a repeatable script. That is precisely the weakness that let me report the swarm image topics
    as broken when they were fine, so Phase 3 builds `eyes_on_check.py --verify`.
  - **F5** — the unreproduced gimbal report, carried forward.

  The matrix also states what it does **not** cover -- sustained load, crash rate, performance, and
  fresh-install behaviour -- so those gaps are deliberate rather than accidental.

  Noted for Phase 2: Ofer asked that `Any` not be *overused* now that `ANN401` is off. The policy is
  that `Any` is for the JSON-RPC/config boundary and unstubbed Isaac APIs, not a default.

  **1558 tests, all gates green.**

- **2026-09-08 (j)** — F1 closed; **Phase 0.5 (adversarial agent review) run and triaged**.

  **F1.** `get_runtime_values` and `read_prim_attribute` added to the `Method` enum, and
  `isaac-core-inspect` switched from raw strings to enum members — six call sites, including four
  that already had members and were still using strings. `_INTERNAL_HANDLERS_OK` is now empty, so
  the enum is once again the complete control-plane surface. Left the two raw strings inside
  `ControlClient`'s docstring examples, where a literal reads better for a user.

  **Phase 0.5 placement.** Ofer asked for a review phase and where it belonged. Findings split into
  substantive (missing tests, logic, dead surface) and cosmetic (comments, docstrings, doc
  structure); the substantive kind must land *before* Phases 2 and 3 to be actionable, and the
  cosmetic kind would be invalidated *by* them. So it runs after the matrix and before the README
  rewrite, explicitly scoped to substance, with reviewers told to ignore style. It also means the
  rewrite answers the confusions a fresh reader actually reported rather than the ones I imagined.

  Seven reviewers ran in parallel: architecture/API, test critic, kernel correctness, Isaac surface,
  robustness/security, fresh user, config. Full triage in `docs/dev/phase_0_5_triage.md`.

  **The finding that justified the whole phase was mine.** `runtime.py` looked up the OmniGraph node
  `isaac_get_viewport_render_product`, which has **zero** occurrences in the shipped camera layers —
  I replaced it with `isaac_create_render_product` during the swarm fix and never updated the
  runtime. `og.Controller.node()` raised, `except Exception` swallowed it, the lookup returned None,
  and `capture_frame(width, height)` silently fell back to viewport resolution while reporting
  success; headless capture would fail outright. Invisible to 1558 passing tests, and I had reported
  that manifest switch as verified. The name is now a module constant with a test asserting it
  against both `.usda` files, and I confirmed the guard bites by reverting the constant and watching
  it fail.

  **Three more real bugs fixed.** (1) A structurally valid UDP packet carrying `lat=200` or `NaN`
  reached `Lla.__post_init__`, whose `ValueError` is not a `PosePacketError`, so
  `HoldLastGoodDecoder` did not catch it and the receive loop crashed — remote-triggerable from any
  sender. `alt=inf` propagated infinity into the stage, the same class as the old `int(inf)` crash.
  `decode` now validates finiteness and lat/lon range as `PacketPayloadError`; 10 parametrized tests
  lock it; the bounds are reused from `contracts/pose.py` rather than duplicated.
  (2) `config/default.toml` had drifted from the schema in three values, and since the README tells
  users to start from that file, copying it *changed behaviour*: `tilesets_root` pointed at
  `/tilesets` instead of `/World/tilesets` (silently disabling the tileset URL override and every
  tile tunable), `delete_cache_on_launch` was `true` (the exact setting our own troubleshooting
  section warns causes worst-case hitching), and `domain_id` pinned 13 instead of inheriting
  `$ROS_DOMAIN_ID`. A guard now diffs every key with a two-entry allowlist; the old test checked only
  validity. (3) `config.layers` is read by **zero** code, and its documented examples were actively
  misleading — `[layers.distance_sensor] max_range_m = 180.0` contradicted the real per-vehicle key
  whose docstring explains that a 180 m ray never reaches the ground, `[layers.bbox_publisher]` used
  an id that is not the shipped layer's, and `[layers.sat]` named a layer that never existed. An
  existing test *required* those sections to exist, which is why they survived.

  **Highest-value accepted-but-not-done finding:** no OGN node `compute()` is ever executed. All
  four runtime nodes are tested by parsing their `.ogn` JSON as data, so the code that actually turns
  a pose into what the camera receives is untested, and `test_gimbal_axes.py` *reimplements* the
  node's composition instead of calling it — they can drift apart while both stay green. That is
  Phase 3's first task.

  Reviewers were also wrong in useful ways, and I rejected four claims after checking: capture path
  confinement is genuinely airtight (`.resolve()` runs before `relative_to`, so symlinks, symlinked
  parents, absolute paths and `..` are all rejected), the import-linter layering is not ceremony,
  `normalize_angle` mapping `+pi` to `-pi` is correct, and `slerp` not clamping `t` is a documented
  precondition all callers respect.

  **1571 tests** (up 13), all seven gates green, 23 contracts kept.

- **2026-09-08 (k)** — Ofer's four decisions implemented; **Phase 1 documentation rewrite**.

  **Decisions.** (1) Multi-vehicle gimbal/capture: took the easy way out as instructed.
  `_require_single_vehicle` refuses with a message naming every configured vehicle and stating which
  one it *would* have acted on, instead of silently retargeting; the real per-vehicle work is
  roadmapped with the ten `next(iter(vehicles))` sites named. (2) `PoseSource` trimmed to
  `{udp, ros}` — `script`, `replay` and `mavlink` were accepted config that produced a vehicle which
  never moved. The test that catalogued them now asserts the *invariant*: every member of the enum
  must imply a shipped layer. (3) Sidecar retained by explicit sanction, recorded in the package
  docstring and in `architecture.md` so no future reviewer re-raises it. (4) Default features are
  `camera_udp` only, so a first run publishes exactly what the walkthrough describes.

  **Two facts I got wrong and had to correct mid-edit**, both caught by checking rather than by a
  test. I wrote that the `isaac_core_ogn.sensors` extension ships *four* nodes while fixing a doc
  that claimed it was *gone*; the filesystem says **seven** across three extensions. Nearly replaced
  one false claim with another. And I described `Lla.alt_m` as "only the difference is used", which is
  an oversimplification: `enu.py` sends both position and reference through a pyproj ECEF transform
  that treats height as ellipsoidal, so the honest statement is that a consistent MSL input is correct
  to within the geoid variation across the scene. The original docstring said "above the WGS84
  ellipsoid; sea level is 0", which names two datums tens of metres apart in one sentence.

  **README rewritten** on the 2023 structure Ofer asked for: requirements, install, run, one section
  per feature (what it does, how to enable it, how to use it, how to confirm it), topics in/out,
  conventions, config, devkit, debug tools, troubleshooting. Emoji at most one per section heading.
  History, architecture and migration notes moved out to their own documents, so the README is about
  *using* the thing.

  New `tests/unit/test_readme_honesty.py` guards the claims mechanically: every document link
  resolves, only shipped layers are documented, no stale "what does not work yet" section survives,
  and the test count is not wildly stale. That last one exists because the count appeared as 1103
  (README, twice), 1279 (roadmap) and 1558 (matrix) while the suite was actually at 1571 — a reader
  who checks one number and finds it wrong stops trusting the rest.

  **Other review findings closed.** `first_run.md` rewritten: it previously opened with "There is no
  `isaac-core run` yet" and walked the reader through building the stage by hand, while the README
  linked it as *the* first-flight guide. `setup.sh` now uses `-e ".[sim]"` to match the doctor's own
  hint (the extra is intentionally empty, so the two spellings were always equivalent — just
  confusing). `rtsp_port`/`rtsp_mount_path` added to `default.toml`, which the README had claimed
  was the complete surface while those real schema fields were absent. `roadmap.md` and
  `usd_build_sheet.md` moved under `docs/dev/`, with all references updated.

  Three new user-facing documents hold what left the README: `architecture.md` (the kernel-purity
  rule and why `contracts/` exists, threading, the control plane, and why the sidecar is empty),
  `authoring_layers.md` (the extension seam, with the declared-but-unconnected trap that caused three
  real bugs), and `migrating_from_2023.md` (byte-identical packet, no Docker, gimbal axes fixed so
  remove your compensation).

  **1575 tests, all seven gates green, 23 contracts kept.** The only unresolved README links are the
  four screenshots Ofer will shoot.

- **2026-09-09** — **Phase 2 started: correctness and security findings first, style after.**

  Deliberate ordering: the review's substantive findings are worth more than the comment pass, and
  doing them first means the style sweep runs over code that is already correct.

  **Control plane.** `MAX_REQUEST_BYTES` (1 MiB) caps the per-connection buffer — a client streaming
  bytes with no newline previously grew it without limit and would OOM a process holding a GPU. Token
  comparison moved to `hmac.compare_digest`; a byte-wise `!=` leaked the token through timing on
  exactly the non-loopback path the token exists to protect. Both tested, the buffer one end-to-end
  over a real socket, asserting the server drops the offender *and still serves other clients*.

  **`limits` was accepted and ignored by two motion functions.** Found `move_to` from the review, then
  scanned the module with `ast` and found `turn_to_point` doing the same — nine functions take
  `limits`, seven honoured it. Both now do: `move_to` extends the duration when the implied
  `distance / duration` exceeds `max_speed_mps`, `turn_to_point` when the implied sweep rate exceeds
  `max_turn_rate_deg_s`. Verified by measurement, not inspection: a 111 km move under a 1 m/s cap goes
  from 10 samples to 111,194. The third test asserts the **invariant** via `ast`, so a new function
  cannot reintroduce the same lie.

  **The live RPC path did not validate what the config path always has.** `_optional_float` used
  `float(...)`, and `float("nan")`/`float("inf")` parse cleanly, so a NaN latitude reached the packet
  encoder and the stage. It now rejects non-finite values and takes optional bounds, applied at
  `set_pose` from the same `MIN_LAT_DEG`/`MAX_LAT_DEG` constants the packet codec uses.

  **Dead surface.** `SAT` and `RAW_RGB` topic constants deleted (zero references). The MAVROS pair was
  more interesting: `MAVROS_LLA = "/mavros/global_position/global"` could never be used because the
  real topic is derived per vehicle from `mavros_namespace`, so rather than delete them I replaced them
  with the *leaves* (`MAVROS_LLA_LEAF`) and wired them into `_resolve_mavros_topic`, which had those
  strings hardcoded at three sites. Dead constants became the single source of truth. Also removed two
  capability probes checking 2023-era prim paths (`/Environment/main_camera`, `/semantics`) that exist
  in no shipped scene, plus their unused constants.

  Two self-inflicted breakages, both caught immediately by the suite: I inserted an import at module
  indentation into a function-local import block, and then hoisted constants into one function while a
  second function 80 lines away also needed them. The fix in both cases was a module-level import,
  which `contracts` being pure makes legal.

  **1588 tests, all seven gates green, 23 contracts kept.** Still to do in Phase 2: the comment-style
  pass, the duplicate-logic sweep, and the error-message audit.

- **2026-09-09 (b)** — Phase 2 comment pass, using Ofer's metric rather than mine.

  I had been measuring comment *density* per file. He corrected the metric: density is not the
  problem, **large blocks** are, and his style is one-liners wherever possible with medium chunks
  only for user-facing or API-shaped things. Re-measured by consecutive-comment-run length, which
  found a completely different set of targets: 26 blocks of five or more lines, the worst at eleven.
  Now **9 blocks, largest 6 lines**, and every survivor sits on a config field or a public contract.

  Compressing them surfaced a bug I had introduced: my `RENDER_PRODUCT_NODE_NAME` constant had been
  inserted *between* the `PATCHABLE_CONFIG_KEYS` comment and the constant it documents, so the comment
  read as though it described the render-product node. Only visible by reading the region, not from any
  test.

  **Removed every reference to the old repo from `src`.** 24 mentions of "2023", 30 of "previous
  generation", plus "the old repo", "god class" and three "defect #N" citations. A generic user has
  never seen that repo and cannot look up defect numbers. Each was rewritten as a positive statement
  of the constraint rather than a comparison -- "Do NOT swap roll and pitch here, that makes the axes
  trade places at the camera" instead of "this deliberately does not match the previous generation,
  which swapped them". The knowledge that stops someone undoing a hard-won decision is preserved;
  the archaeology is not. Kept the year-based version numbers in `install.py`, which are real Isaac
  Sim releases (2022, 2023.1.1) and not history.

  Method note: I did these as scripted exact-string replacements and 10 of 29 missed on docstring
  indentation, which then left three passages reading as broken sentences ("Anything calibrated
  against a swapped mapping needs revisited."). Caught by reading the output rather than by the
  suite -- prose damage is invisible to tests, so a scripted docstring edit has to be re-read.

  **1588 tests, all seven gates green, 23 contracts kept.** Remaining in Phase 2: the duplicate-logic
  sweep and the error-message audit.

- **2026-09-09 (c)** — Phase 2 duplicate-logic sweep, error-message audit, and the remaining
  lower-priority review findings.

  **The real duplication was not where the plan guessed.** I had listed three topic resolvers as
  candidates; reading them showed a shared `_resolve_topic` already exists and what repeats is a
  three-line "explicit config value wins, else derive" idiom where the *differences* -- which field,
  which derivation -- are the whole substance. Folding that into one parameterised helper would trade
  three readable lines for a callable argument and hide which field feeds which topic. Left alone,
  with a comment saying why, per the plan's own rule.

  What was genuinely duplicated: **`next(iter(config.vehicles))` inlined at 11 sites** across
  `runtime.py` and `composer.py`, while `configurator.py` had a private helper for exactly that. Now
  one `IsaacCoreConfig.first_vehicle_id` property, which also raises a real error when no vehicles are
  configured instead of `StopIteration`. Also renamed a nested `_pump` that shadowed the method of the
  same name while doing something different -- the method advances frames, the nested one advances the
  *timeline* too, which is the distinction that made stepping work while paused.

  The quaternion-helper candidate turned out clean: no extension does its own conversion, they all
  call `isaac_core.geo`.

  **Error-message audit.** Sixteen raised messages are seven words or fewer, but short is not
  unactionable -- "capture_frame requires params.path" names the method and the field. The one worth
  fixing was `set_config`, which paired with a review finding: the devkit's `config.patch(**kwargs)`
  invited `patch(gimbal_max_rate_deg_s=10.0)`, a call the server always rejects because a dotted key
  cannot be a Python identifier. `patch(key, value)` now matches the wire contract, and the
  missing-params error shows the exact JSON shape plus the patchable keys.

  **Remaining findings closed.** `/home/ofer/isaacsim` was hardcoded in three shipped scripts while
  `install.py` correctly probed `Path.home()` -- one user's path shipped to everyone; now
  `$HOME`/`$ISAACSIM_PATH`. `setup.sh` ended with `isaac-core doctor || true`, so a broken environment
  still printed "Setup complete"; it now exits non-zero and says to fix the reported lines.
  `save_video` did not check `writer.isOpened()`, and OpenCV returns a writer even with no codec, then
  silently drops every frame -- an empty file behind a "wrote N frames" log; it now raises and names
  the remedy. And capture's render-product resize was undone only on the success path, so a failed
  capture left the image topic and RTSP stream at capture resolution until relaunch; there is now one
  `_restore_capture_resolution` used by both paths, with a test asserting the error path calls it.

  **1588 tests, all seven gates green, 23 contracts kept.**

- **2026-09-09 (d)** — README CR round 1 applied; RTSP swarm collision diagnosed; **my OGN framing
  was wrong and Ofer caught it**.

  **README.** Applied his review and, at his request, extended the same ideology past the 25% he had
  read so the rest is cheaper to review. Repo-level bragging removed from the summary ("adding your
  own sensor does not require forking" was *our* grievance with the old repo, not a user's concern) --
  the capability keeps its own section instead. Specific sensors in the opening paragraph collapsed to
  "optional robotics and computer-vision features". Every system requirement and every feature is now
  a collapsible section. Deleted the "two environment facts" block: the rclpy/interpreter split is
  developer-facing and already lives in `docs/ros2_and_python.md`, and `$ISAACSIM_PATH` is not a
  requirement because the tooling probes for the install. Removed hand-holding cross-references
  ("that symptom is covered in troubleshooting") -- a user in trouble reads the troubleshooting
  section without being sent there. Removed editorialising like "commanded over the control plane
  because aiming a gimbal is a command rather than telemetry".

  He also caught unfriendly wording: "+pitch raises the look direction". Technically right for a
  gimbal -- it is the camera that moves, not the nose -- but nobody says "look direction". Now
  "+pitch aims the camera up".

  `python3-tk` promoted from optional to required in both the README and `requirements.txt`. It is
  stdlib, so it cannot be a pip line; the file documents it in the not-installable-by-pip section,
  now marked REQUIRED with the reason the GUI will not start without it. Also stripped the last
  archaeology from `requirements.txt`.

  **The RTSP swarm collision now has a specific mechanism, not a guess.** `inputs:port` is declared in
  the USD as `custom int inputs:port` with **no authored value**, and the `.ogn` default is **8554**.
  So if the per-vehicle port write does not land, both helpers fall back to 8554 -- exactly the
  observed `Error binding to address 0.0.0.0:8554: Address already in use`. This is the same
  declared-but-unwritten failure class as the three earlier bugs where an unconnected input silently
  used its default. Verified what I can statically: the config resolves correctly (lead 8554/wing 8555,
  both with cameras), `compute_writes` targets the right per-vehicle prim paths, and no stale process
  or bound port was present. What remains unverified is whether the write *lands* on wing's prim at
  runtime. Not guessing a fourth time: one probe reading the authored `inputs:port` off both prims
  after compose settles it.

  Also worth recording for the screenshot: with offscreen render products there is now only **one
  viewport**, so no single screenshot can show two camera views. Ofer's swarm shot is not obtainable
  the way it was specified.

  **My OGN claim was mis-framed and he was right to challenge it.** I wrote that "no OGN node
  compute() is ever executed" as a Phase 3 headline, which reads as though we were failing to call
  something. Isaac drives `compute()` and `release()`; we never do. The finding was only ever about
  *test coverage* -- our suite exercises the `.ogn` schemas as data and never runs the compute bodies.
  Measured how much that actually hides:
  `OgnGlobalPositionToLocalPosition.compute()` is **67 lines**, `OgnDistanceSensor` 64,
  `OgnBboxProjector` 53, the rest 20-35. So "the nodes are thin adapters over the kernel" is only
  partly true, and the untested part is real.

  But the remedy I had queued -- drive `compute()` from tests with a fake `db` -- is the worse of the
  two options: it is artificial, and it tests our fake as much as our code. The better fix is to make
  the claim true: move the logic out of those three compute bodies into `isaac_core.geo` /
  `contracts`, where it is testable with no Isaac and no fake, leaving `compute()` genuinely thin.
  Phase 3 will do that instead.

  **1590 tests, all seven gates green, 23 contracts kept.** Five of the README images are in place.

- **2026-09-10** — **The swarm RTSP collision was a real bug, found and fixed.** Twice I called it a
  stale process. It was not.

  Ofer insisted I run scenario 6 live and verify before claiming anything, and that is what found it.
  Two runs, same machine, minutes apart: my hand-written `--config` file worked (both 8554 and 8555
  listening, zero errors), and `eyes_on_check.py 6` **reproduced the collision**. The difference is
  that `Sim.launch` dumps the resolved config to a temp TOML and re-reads it.

  **Root cause.** `resolved_rtsp_port` decided "explicitly set means do not offset" by testing
  `"rtsp_port" in camera.model_fields_set`. `dump_toml` writes every field, so after a dump/reload
  *every* field is "explicitly set" -- the per-vehicle offset was skipped and both vehicles asked for
  8554. Measured directly: before dump `{lead: 8554, wing: 8555}`, after dump `{lead: 8554, wing:
  8554}`. So every multi-vehicle `Sim.launch` collided, which is every eyes-on swarm run.

  Two things hid it. The existing `test_dump_toml_round_trip_produces_equal_config` compares configs
  for **equality**, and the configs *are* equal -- `model_fields_set` is not data. Exactly the
  weak-assertion class the review warned about. And I had twice attributed the symptom to a leftover
  process, which was plausible and wrong.

  **Fix**: `rtsp_port` is now `Port | None = None`, meaning derive, matching what `udp_port`,
  `rtsp_mount_path` and every topic field already do -- `resolved_udp_port` was never affected because
  it already used that pattern. Round-trip verified, explicit pins still honoured verbatim, and three
  tests added that assert **behaviour** across a dump/reload rather than equality.

  Live result after the fix: scenario 6 shows **zero** occurrences of `Address already in use`,
  `failed to create socket`, `Failed to attach RTSP`, `renderProductPath is empty` and
  `Render product '' not valid`. The last two were downstream of the RTSP failure, which is why my
  earlier "transient first-tick warning" reading looked self-consistent.

  Also verified, since Ofer asked whether cleanup had caused it: the camera manifests are **unchanged**
  since `c532c03`, the commit whose message claims all eight checks passed. The bug predates the
  cleanup; that claim was wrong when I wrote it.

  **Ofer's frames question, answered by reading the code.** The README said "on the wire: NED", which
  is only true of the UDP packet. `OgnUdpToGlobalPosition` calls `ned_to_enu`; the ROS path takes
  MAVROS orientation, which is **already ENU** per ROS convention, straight through
  `quaternion_to_euler`. Each path converts exactly once and both reach the stage as ENU. That also
  **disproves** the Phase 0.5 suspicion (isaac_surface #7) of a double conversion on the ROS path --
  recorded as rejected.

  **README round 2.** Removed the test-count status line (an us-metric). Rewrote the pose-input
  section to answer "what are all the ways to command a pose": exactly two *sources* the simulator
  listens on, plus a table of host-side adapters that feed the UDP one -- which is what
  `isaac-core-mavlink` is, and why there is no `mavlink` pose_source. Config precedence became a
  five-row table explaining that `--config` and `$ISAAC_CORE_CONFIG` are two spellings of the *same*
  slot while env vars are a different mechanism. The devkit section now documents every class a user
  touches with full signatures and a start-to-finish example, in collapsibles. Feature-layer authoring
  gained a USD tree, a full `layer.toml` and the two config keys, with a step-by-step GUI walkthrough
  in `authoring_layers.md`. Every troubleshooting entry is now collapsible. 35 collapsible sections in
  total.

  `opencv-python` was referenced by the README but declared nowhere; it is pip-installable, so it went
  into the `[devkit]` extra with the README naming the install command.

  **1593 tests, all seven gates green, 23 contracts kept.**

- **2026-09-10 (b)** — **Ofer found a real functional regression in the offscreen-render-product
  switch: Cesium does not stream terrain for a camera that has no viewport.**

  He noticed it from the picture, not from a log: with one viewport, `wing`'s image contained only the
  union of tiles `lead` had pulled in. Confirmed the mechanism in Cesium's own source --
  `extension.py` builds its per-frame view list by iterating `get_viewport_window_instances()` and
  reading each window's view and projection matrix, then passes that to `on_update_frame`. An
  offscreen render product is not a window, so Cesium never knows to load tiles for it. There is no
  hook to register a view without a window; the list is built solely from windows.

  He also remembered a version where two viewports worked, and he was right: `a77b631` and `dac8867`
  carry the four-node chain (`IsaacCreateViewport`, `IsaacSetViewportResolution`,
  `IsaacGetViewportRenderProduct`, `IsaacSetCameraOnRenderProduct`). `c532c03` -- the commit whose
  message claims all eight eyes-on checks passed -- **already** had the offscreen node, so that claim
  was made after the regression was introduced and did not catch it.

  **Fixed in Python rather than by reverting the USD.** `omni.kit.viewport.utility.create_viewport_window`
  takes `name`, `width`, `height` and `camera_path`, so the runtime now opens one window per vehicle
  after the first, pointed at that vehicle's camera at its configured resolution. This keeps Cesium
  happy without reintroducing `IsaacGetViewportRenderProduct`, whose NVIDIA bug I read in source (a
  local `viewport = None` instead of `self.viewport`, so the attribute only exists when the window
  lookup succeeds and is read unconditionally). It also needs no USD authoring from Ofer.

  Matching the viewport resolution to the camera matters: `IsaacCreateRenderProduct` reuses an
  existing product whose `camera` relationship and `resolution` both match, so viewport plus node is
  one render pass, not two.

  Live: scenario 6 logs `viewport looking through .../lead/...` and `opened viewport 'wing' looking
  through .../wing/...`, with zero occurrences of every error class. Ofer then got the two-viewport
  screenshot, now in the README.

  Two bugs of my own on the way, both caught immediately: `_viewport_size_for` read
  `camera.width`/`camera.height`, which do not exist -- the field is a `resolution` tuple -- and a
  README anchor string I asserted on had drifted.

  **`opencv-python` question.** It *is* used: `devkit/recording.py` imports `cv2` lazily to write
  video. It was declared in no dependency list, so a user following the README would hit ImportError.
  An "extra" is pip's optional-dependency group -- `[project.optional-dependencies]` in
  `pyproject.toml` -- installed with `pip install 'isaac-core[devkit]'`. `devkit` already existed but
  was empty; `opencv-python>=4.8,<5` now lives there because video writing is genuinely optional,
  while `requirements.txt` stays the always-needed set.

  **1595 tests, all seven gates green, 23 contracts kept.** All six README images now resolve.

- **2026-09-10 (c)** — Startup viewport named after the first vehicle.

  Ofer's observation after re-running scenario 6: the swarm viewports work, but `wing` had a title and
  `lead` was Kit's generic unnamed "Viewport", so one of a matched set looked different from the rest.

  Kit takes the startup window's name from `/exts/omni.kit.viewport.window/startup/windowName`, read
  when it creates that window -- so it has to be a startup argument, not something set afterwards.
  `_default_viewport_name_args` adds it from `first_vehicle_id`, and returns nothing when headless or
  when no vehicle is configured. Verified live: both windows are now labelled with their vehicle id,
  and scenario 6 still shows zero errors of every class.

  Fixing it broke `test_log_arguments_are_omitted_when_isaac_logs_is_wanted`, which asserted
  `_kit_startup_args() == []`. That assertion was too broad: it claimed the *whole* argument list is
  empty when Isaac's logs are wanted, when what it means is that no `--/log/` arguments appear. Now
  narrowed to that, so an unrelated startup argument cannot fail it again.

  **1598 tests, all seven gates green, 23 contracts kept.**

- **2026-09-14** — README CR3. Two of the three items were **my errors**, both caught by Ofer reading
  rather than by any test.

  **The USD tree I documented contradicted every layer we ship.** I had drawn the OmniGraph under
  `/Root/Xform`. Checked all four shipped layers: `bbox` has `/Root/BboxExport`, `camera_udp` and
  `camera_ros` have `/Root/CameraImageExport` and `/Root/PoseSync`, `distance_sensor` has
  `/Root/RangeSensing` -- every graph is a **sibling** of `Xform`, directly under `/Root`. That is not
  arbitrary: `Xform` carries the vehicle transform and a graph has none, which is exactly why manifest
  paths read `{mount}/CameraImageExport/...`. `docs/authoring_layers.md` was worse than wrong, it was
  self-contradictory: it drew the graph inside `Xform`, said it resolves to
  `.../Xform/ThermalExport/...`, then claimed that is what `{mount}/ThermalExport/...` resolves to.
  Both fixed, and `test_every_layer_puts_its_graphs_directly_under_root` now asserts the convention so
  the docs cannot drift from the layers again.

  **The README imported a class that does not exist.** `from isaac_core.devkit.recording import
  Recorder`, in two places. There is no `Recorder`: the module exports `TopicRecorder` plus four
  factories (`video_recorder`, `pose_recorder`, `range_recorder`, `bbox_recorder`) and two timing
  helpers. A user would have hit `ImportError` on the first line they copied. Both sites now document
  the real surface, including the five-method recorder lifecycle and the `TopicRecorder` constructor
  for topics with no factory.

  **Ofer's `*` question.** He asked whether the bare `*` in `Sim.attach`/`Sim.launch` hid additional
  options. It does not -- it is Python's keyword-only marker. Verified by introspection that neither
  takes `**kwargs`, and that `attach` has `host`/`port` positional-or-keyword while **every** argument
  of `launch` is keyword-only. Now stated explicitly, along with the fact that both lists are complete.

  **New devkit example**, to his spec: launches with `bbox` enabled, starts a video recorder and a bbox
  recorder each on its own thread *before* the flight, changes the simulation both ways (`set_gimbal`
  as a control call and `config.patch` as a runtime patch), flies an orbit over UDP, then saves the mp4
  and the boxes and prints capabilities and the final pose. Every one of the 18 symbols it uses was
  checked to exist, and `config.patch(key, value)` matches the signature corrected in Phase 2.

  **1599 tests, all seven gates green, 23 contracts kept.** Phase 1 is closed.

- **2026-09-14 (b)** — **Phase 3 begins: logic moved out of the OGN compute bodies into the kernel,
  which is the fix Ofer preferred over driving `compute()` from tests with a fake database.**

  **`OgnGlobalPositionToLocalPosition`: 67 lines -> 38.** The whole composition now lives in
  `geo/pose_pipeline.py` as `compose_local_pose`, plus `has_position_fix` (the zero-position guard),
  `rotation_frame_from_text` (string to enum, defaulting to BODY) and `quaternion_to_isaac_order` (the
  WXYZ-to-IJKR reorder that is a 180 degree error when reversed). Before rewriting the node I checked
  the extracted kernel reproduces the original maths **exactly** across three cases including a WORLD
  frame with non-zero offsets, so this is a move rather than a rewrite. 21 new tests cover the
  branches that previously could only be reached by launching Isaac.

  **Closed finding T2, the one that mattered most.** `tests/unit/geo/test_gimbal_axes.py` carried a
  *copy* of the node's composition and said so in its own docstring -- node and test could drift apart
  while both stayed green, which is precisely how the original pitch-as-roll bug survived. It now
  calls `compose_local_pose`, so the assertions about the camera's real world look direction are
  measuring the shipped path.

  **`OgnDistanceSensor`: honest result, not a forced one.** Most of its 64 lines are irreducible Isaac
  interaction -- stage lookup, `Xformable.ComputeLocalToWorldTransform`, the render raycast sequence --
  and its pure part (`resolve_range`) was already in `contracts/rangefinder.py`. What I did extract is
  `boresight_direction`, the negated third row of the camera's world rotation, because that *was* a
  real bug: casting along a sibling Xform's -Z was permanently 90 degrees off, since an Xform is
  identity while the camera carries orient (0.5, 0.5, -0.5, -0.5). Three tests. Also collapsed four
  identical "report no detection" exits into one helper, which matters because reporting zero instead
  of the rated maximum reads as "touching the ground".

  Two self-inflicted breakages, both the same mistake twice: a naive string replacement on
  `from x import y` mangled a **multi-line** import into `import boresight_direction, (`. I had already
  done this once in Phase 2. Stopping that pattern: multi-line imports get rebuilt, not string-patched.

  **1623 tests, all seven gates green, 23 contracts kept.** Remaining in Phase 3: `OgnBboxProjector`
  (53 lines, parallel-array assembly is genuinely extractable), the `--verify` runner for bucket B, and
  the rest of the T-list.

- **2026-09-14 (c)** — Phase 3 continued. The `--verify` runner found a real regression on its first
  run, which is exactly why it exists.

  **Equivalence of the extracted pose pipeline, done properly.** Ofer was right that three cases was
  thin for the heart of the repo. Ran a systematic sweep of the extracted kernel against the original
  inline node body pulled from git: four ENU references (including the equator and 78 degrees north),
  fifteen positions each, six airframe attitudes including one at gimbal lock, six gimbal offsets
  including near-180-degree values, both rotation frames -- **4320 cases, largest deviation exactly
  0.000e+00**. Bit-identical, so the extraction is a move rather than a rewrite. Froze 54 of those as
  golden vectors in `test_pose_pipeline_golden.py`, which is better than keeping the second
  implementation around: a copy is what let the node and its test drift in the first place. Verified
  the golden test bites by removing the gimbal pitch sign flip -- 41 failures.

  **`OgnBboxProjector`.** The 16 parallel arrays moved to `contracts/bbox.py`, where `BboxArrays`
  takes one whole frozen `BboxDetection` per append. Sixteen independent `list.append` calls could
  silently omit one and shift every later detection; now that is unrepresentable. 12 tests cover the
  invariant, including that an invisible target and an anchorless target each still occupy their
  index, and that `ARRAY_FIELDS` matches both the dataclass and the shipped `.msg`.

  **T6 found a real bug.** The distance sensor's `.ogn` default for `max_range_m` was **100.0** while
  config says **5000.0** -- and the schema's own docstring explains that a 100 m ray never reaches the
  ground and reports "no detection" forever while looking perfectly healthy. So a binding that failed
  to land would silently use the known-broken value. Fixed, plus three `.ogn` descriptions still
  claiming the sensor reports `+inf`/`-inf` when it has saturated at its limits since D28. Guarded by
  `test_ogn_defaults_match_config.py`.

  **T3 and T4.** `test_orbit_yaw_is_tangent_to_path` asserted only `isfinite(yaw)` -- it named a
  behaviour and checked nothing about it. Now it measures the bearing from each sample to the next and
  asserts the reported yaw matches within 2 degrees, with a companion asserting yaw sweeps more than
  180 degrees over a full orbit so a frozen heading cannot pass. Added four distinct-field golden
  packet vectors; verified they catch a roll/pitch transposition in `encode()` (11 failures) which the
  existing single near-symmetric vector could not.

  **A trap in my own verification technique.** Break-and-restore appeared to show the orbit fix
  failing after restore. The cause: my frozen edit `yaw_r=0.0,` is exactly as long as `yaw_r=yaw,`, so
  the restored file had an identical **size**, and Python's timestamp+size `.pyc` invalidation reused
  stale bytecode. Every earlier break-and-restore proof stands (the tests did fail then pass), but the
  technique now requires clearing `__pycache__` or making a size-changing edit.

  **`--verify` runner built** (closing F4). `eyes_on_check.py --verify` runs headless, asserts numbers,
  and exits non-zero; `all` runs every verifiable scenario. Five of eight are now automated: pose
  tracking, gimbal, capture, swarm and lifecycle. Launches are retried up to three times because Isaac
  segfaults on roughly one launch in three, but a failed **assertion** is never retried -- that would
  hide the failures the runner exists to find. First full run: `3/5`, and both failures were real
  information. Scenario 1 was the startup segfault (passed alone, hence the retry logic).

  **Scenario 5 is a genuine regression and it is mine.** `capture_frame(width=1920, height=1080)`
  silently captures at **1280x720** and reports those numbers back. It fails outright headless. A
  19 MB `four_k.png` from 2026-09-07 proves 4 K capture used to work. Mechanism: capture resizes the
  render product it resolves, but `IsaacCreateRenderProduct` **reuses a product whose camera and
  resolution already match** -- and the viewport aimed at that camera owns one. So capture is resizing
  a viewport-owned product, and the viewport puts it back. Correcting the node-name lookup earlier was
  necessary but not sufficient. **The fix is for capture to create its own render product at the
  requested resolution instead of resizing a shared one.** Not attempted yet; it is the one open item
  in Phase 3 and it affects a documented feature.

  **1704 tests, all seven gates green, 23 contracts kept.**

- **2026-09-14 (d)** — Capture-at-resolution: two fix attempts failed, so the behaviour is now honest
  rather than half-fixed.

  Tried creating a dedicated render product with `rep.create.render_product(..., force_new=True)` so
  capture would stop resizing the viewport's -- it produces **no file at all**. Then added
  `rep.vp_manager.attach_hydra_texture(path)`, which is what Isaac's own node does for the same
  reason -- still no file. Two attempts at the same approach, so I stopped guessing per my own rule and
  reverted to the mechanism that works for camera-resolution capture.

  **What changed instead:** the finish stage now compares the achieved resolution against the request
  and **fails with an explanation** when they differ. Before this, asking for 4 K silently wrote 720p
  *and reported 1280x720 as though that were fine* -- a caller had no way to notice. Verified live:
  camera-resolution capture writes its file, a 4 K request is refused with the reason and the
  workaround. The README's 4 K example is gone and the limitation is stated where the feature is
  documented.

  `docs/dev/roadmap.md` records both failed attempts explicitly, so whoever picks this up does not
  repeat them, plus the two remaining hypotheses: a freshly created product may need more than
  `CAPTURE_SETTLE_FRAMES` before `capture_viewport_to_file` will write it, or the call may need the
  viewport that *owns* the product rather than the active one.

  Matrix updated: **F4 closed** (bucket B has a runner), **F6 opened** for this. Worth stating plainly
  that the runner justified itself immediately -- this was invisible to 1704 unit tests because it needs
  a live renderer, and it is the second time in this project that a documented capability turned out
  not to work while everything green said otherwise.

  **1704 tests, all seven gates green, 23 contracts kept.**

- **2026-09-14 (e)** — **A regression I introduced, caught by Ofer, plus five failed attempts at
  capture-at-resolution.**

  **The viewport rename was mine and it broke a Kit extension.** Ofer said the
  `AttributeError: 'NoneType' object has no attribute 'get_frame'` "was not there before", which was the
  clue. `omni.kit.viewport_widgets_manager/manager.py:201` does
  `get_active_viewport_window(window_name="Viewport")` with the name **hardcoded**. Naming Kit's startup
  window after the first vehicle made that return `None`, and the extension died on every launch.
  Reverted; verified live that a plain `python -m isaac_core.sim` now logs zero occurrences of
  `get_frame`, `viewport_widgets_manager` and `Failed to startup python extension`. The function is kept
  as a stub returning `[]` with the reason in its docstring, so nobody re-adds the argument, and the
  test is now the inverse: assert the window is **never** renamed. Cosmetic consistency was not worth a
  startup error, and I should have launched once after making that change instead of trusting the tests.

  **Capture-at-resolution: still not fixed after five approaches.** All six ruled-out attempts are
  recorded in `docs/dev/roadmap.md` with what each produced. The important correction: one instrumented
  run reported 1920x1080 and I described it as working. I never checked the pixels on disk for that run,
  and every subsequent run with identical code wrote 1280x720. `request.used` is read back from the
  render product, so the product can claim the new size while the capture writes the viewport's -- so my
  "it works" was very likely a false positive of exactly the kind the new honest check exists to catch.
  Correcting it here rather than leaving it in the log.

  Also hit the stale-`.pyc` trap a third time, which is what made the race look real: a run with a debug
  log line behaved differently from one without. Every capture experiment from here on clears
  `__pycache__` first.

  Left the tree in the honest state: camera-resolution capture works, a requested resolution refuses
  with the reason and the workaround, the README says so, and the speculative settle-frame bump is
  reverted so the code is the known-good shape.

  **1703 tests, all seven gates green, 23 contracts kept.**

- **2026-09-14 (f)** — System test harness built. Capture still not fixed, and the archaeology
  overturned my earlier account of it.

  **Archaeology on `c532c03`, which Ofer asked for, changes the story.** At that commit
  `_camera_render_product()` looked up `isaac_get_viewport_render_product` while the USD had already
  been refactored to `isaac_create_render_product`. So the lookup resolved **nothing**, the resize was
  a no-op on `None`, and capture silently used viewport resolution and reported it. Capture-at-resolution
  was therefore **already broken at the commit whose message says all eight checks passed** -- Ofer's
  recollection of scenario 5 passing is consistent with a file being produced, which it was. The 19 MB
  4 K file dated 2026-09-07 belongs to the earlier era (`a77b631`/`dac8867`) when each camera had its
  **own** viewport, so the render product was that viewport's and resizing it was uncontested.

  **The mechanism, now understood.** `IsaacCreateRenderProduct` owns its product and reconciles it only
  when its **own** `inputs:width`/`inputs:height` change -- its compute reads
  `if state.resolution != (db.inputs.width, db.inputs.height)`. Writing the product's `resolution`
  attribute directly leaves that state untouched, so nothing reconciles. Capture now drives the node's
  inputs instead, which is the sanctioned path. Whether that alone fixes it is **unverified**: two
  consecutive live runs produced no capture file at all for every resolution including the camera's,
  which does not track my changes and points at run-to-run instability I could not isolate.

  **Headless capture does not work at all**, which is a genuine constraint rather than a bug in my
  change: `capture_viewport_to_file` needs a viewport colour resource and headless has none. The system
  tests were moved to a GUI fixture for capture because of it, and skip when there is no display.

  **`tests/system/` exists and works as infrastructure.** Opt in with `--system` or
  `ISAAC_CORE_SYSTEM_TESTS=1`; skipped otherwise, so the default suite is still 1703 tests in 18
  seconds. Session-scoped fixtures launch one simulator each (single vehicle headless, swarm headless,
  single vehicle GUI) and share it across tests, because a launch costs 20-40 s. It reads PNG dimensions
  straight from the IHDR chunk rather than via Pillow, so measuring pixels needs no extra dependency.

  It immediately found two problems **in itself**, which is worth recording because both are the same
  mistake I keep making in a new guise: concurrent session fixtures collided on RTSP 8554 and then on
  UDP 33333, so one simulator's poses were landing in another's socket. Assigning per-fixture port
  ranges fixed the RTSP half; the UDP half is not right yet -- after that change the single-vehicle pose
  test began failing too, and standalone the same swarm configuration measures a perfect 600.00 m
  separation. So the current failures are the harness, not the product.

  **State: harness in place and honest, capture unfixed.** 1703 unit tests and all seven gates green;
  the system suite is opt-in and currently red for capture and swarm, both traced to fixture port
  isolation and to capture itself.

- **2026-09-14 (g)** — **capture-at-resolution FIXED, and Ofer was right on both counts.**

  He insisted the feature had worked and that never producing a single correct file suggested something
  more basic was wrong. Both were true, and my repeated "it was always broken" conclusion was reached by
  *reading* commits instead of *running* them.

  **Root cause: the viewport resize is asynchronous.** The viewport widget carries a
  `__resize_future`, so the render product is still the old size for many frames after
  `viewport.resolution = (w, h)` returns. With `CAPTURE_SETTLE_FRAMES = 8` the capture read the old size
  back, wrote the camera's resolution, and reported the requested one. That is why five earlier
  approaches all "failed": every one of them was measured too early.

  What made it findable was making the machinery observable instead of guessing. The capture result now
  reports `render_product`, `original_width/height` and `viewport_width/height`, and the mismatch error
  quotes all three. That immediately showed two facts I had been unable to see: the resolved product is
  `.../HydraTextures/omni_kit_widget_viewport_ViewportTexture_0`, i.e. `IsaacCreateRenderProduct`
  **reuses the main viewport's** product, and after setting the viewport it still measured 1280x720 --
  which pointed at timing rather than ownership.

  **The fix**: set the viewport resolution (the product follows, since the product is the viewport's),
  and wait `CAPTURE_RESIZE_SETTLE_FRAMES = 30` frames rather than 8 when a resize was requested.
  Measured on disk by reading PNG IHDR headers: 1920x1080 and 3840x2160 as requested, camera resolution
  unchanged before and after. Also confirmed by actually reverting to the `a77b631` layer and running it,
  which is what Ofer asked for -- the old design resolved `ViewportTexture_0` too, so the old and new
  designs share the mechanism and the old one was equally subject to the timing bug.

  **Headless capture genuinely cannot work**: `capture_viewport_to_file` needs a viewport colour
  resource. Documented in the README rather than worked around.

  **System tests, as a tool.** `tests/system/` runs a real Isaac Sim under pytest, opt in with
  `--system` or `ISAAC_CORE_SYSTEM_TESTS=1`. **All 13 pass in 1 minute 47.** They measure observable
  outcomes: PNG pixel dimensions read from the IHDR chunk (no Pillow dependency), the stage transform
  after a commanded pose, two vehicles at two altitudes, and that a swarm refuses single-vehicle
  commands.

  Three problems in the harness had to be fixed first, all the same class of mistake in new clothes:
  concurrent fixtures collided on RTSP 8554, then on UDP 33333 so one simulator's poses landed in
  another's socket, and finally **session**-scoped fixtures never release, so three simulators ran at
  once and starved the GPU until a control-plane call timed out. Module scope plus an 8-second teardown
  grace fixed the last one and cut the suite from 4-7 minutes to under two. `settle()` also steps in
  chunks of 10 frames, because one 40-frame RPC could exceed the socket timeout and a slow loop was then
  indistinguishable from a wedged one.

  **1705 unit tests, 13 system tests, all seven gates green, 23 contracts kept.**

- **2026-09-14 (h)** — Phase 3 closed. System test harness stabilised as far as Isaac allows, and the
  remaining flakiness is honest about itself.

  **`--verify` retained, `tests/system/` is the tool.** Ofer preferred a tool over a rule, and this is
  it: `python3 -m pytest tests/system --system`, 13 tests, about 95 seconds on a clean machine.
  Verified fully green on four separate runs (90 s, 94 s, 95 s, 95 s).

  **Four harness bugs found and fixed, all the same class -- shared resources between concurrent
  simulators:**
  1. RTSP 8554 collision between fixtures -> per-fixture port ranges.
  2. UDP 33333 collision, so one simulator's poses landed in another's socket -> per-fixture UDP ports.
  3. **Session**-scoped fixtures are never released, so three simulators ran at once and starved the
     GPU until a control-plane call timed out -> module scope, which also cut the suite from 4-7
     minutes to under two.
  4. A 40-frame `step` in one RPC could exceed the socket timeout -> `settle()` steps in chunks of 10.

  **Two more subtle ones.** `set_pose` sends a **single** UDP packet, so one sent before the receiving
  node has bound its socket is simply lost and nothing retries -- a real sender streams at 30 Hz. The
  pose assertions now resend while polling for convergence, which is both honest about the asynchrony
  and how a real client behaves. And module **order** turned out to matter: the two-vehicle module run
  last would fail or skip, run first it passed every time, so `MODULE_ORDER` puts the heaviest simulator
  on the clean machine.

  **Known limitation, stated plainly.** Even so, one run in roughly three cannot bring the swarm
  simulator to a composed, ready stage, and the readiness probe **skips** those tests rather than
  failing them -- a simulator that will not launch is an environment problem, not a defect in the code
  under test, and reporting it as a product failure would be the worse lie. Every wait is bounded, so
  the worst case is a slow run rather than a hang. Swarm behaviour itself is verified by other means:
  standalone runs measure exactly 600.00 m of separation, and Ofer has confirmed eyes-on scenario 6.

  **Phase 3 test-gap findings all closed.** T2 (the gimbal test reimplementing the node) closed by the
  kernel extraction; T3 (orbit yaw asserting only `isfinite`) rewritten to measure path bearing; T4
  distinct-field packet vectors added and verified to catch a roll/pitch transposition in `encode()`;
  T5 gimbal target accumulation now has five tests covering held-versus-start fallback; T6 found and
  fixed a real `.ogn` default that disagreed with config.

  Also removed the last `previous generation` reference from `src`, which the Phase 2 sweep had missed.

  **1710 unit tests, 13 system tests, all seven gates green, 23 contracts kept.**

- **2026-09-14 (i)** — **Phase 4 (repo hygiene) done.** Two of the findings were real bugs rather than
  tidiness, which is the argument for doing this pass at all.

  **A packaging bug: the shipped layer manifests were not shipped.** `package-data` listed nothing, so
  the four `assets/layers/*/layer.toml` files were excluded from any wheel or sdist -- `SOURCES.txt`
  confirmed **zero** `layer.toml` entries. A non-editable `pip install isaac-core` would therefore have
  no feature layers and every feature would fail to compose. Invisible because every install in this
  project, including `setup.sh`, uses `-e`, which points at the source tree. Fixed and guarded by a
  test that walks every non-Python file in the package and asserts it matches a declared pattern.

  **`py.typed` was missing.** The package is checked under strict-ish mypy, but without the marker a
  consumer's type checker silently ignores every annotation. Added, declared in `package-data`, and the
  `Typing :: Typed` classifier now has something behind it.

  **A documented feature that does not work.** `config/default.toml` described a second camera per
  vehicle -- "topics then become /isaac_core/eo/image_rgb and /isaac_core/ir/image_rgb". Measured: a
  vehicle declaring `eo` and `ir` plans **one** camera layer, scoped to `eo`, and the second camera gets
  no topic and no RTSP stream. The claim is gone, the limitation is roadmapped, and the README's
  "namespaced when there is more than one camera" is corrected to "vehicle".

  **`config/default.toml` rewritten end to end**, since it is the file users actually read. It carried
  fifteen inaccuracies, several of them consequences of this month's changes: it presented itself as the
  base layer of the precedence chain (it is documentation and is not auto-loaded), gave an environment
  variable example missing the per-vehicle path, still advertised `script`/`replay`/`mavlink` pose
  sources that were removed, said the control plane could "enable features" after that was deleted,
  documented `mount` defaulting to `/Environment/<id>` rather than `/World/Environment/<id>`, claimed
  swarm topics look like `/isaac_core/lead/eo/image_rgb` when they are `/isaac_core/lead/image_rgb`, had
  `rotation_frame` documented twice with different wording, contradicted itself about `[layers.*]`, and
  pointed `[[prim_overrides]]` at a prim that does not exist. Two archaeology references also went.

  **`.gitignore` replaced.** It was the stock GitHub Python template -- 200 lines covering Django,
  Scrapy, SageMath, Celery, Abstra and Marimo -- while knowing nothing about this project: captures
  under `isaac_core_out/`, the scratch directories used to debug against a live simulator, or the
  `carb.*` directories Kit leaves behind when it crashes. Now 68 relevant lines, verified by
  `git check-ignore` that nothing previously ignored became visible.

  **Corrected the plan's own assumption**: it listed `src/isaac_core.egg-info` as a stale artefact to
  delete. It is not stale -- its entry points match `pyproject.toml` exactly, `SOURCES.txt` has zero
  dangling entries, and the console script works. It is also what makes the editable install resolve,
  and regenerating it needs a `pip install` I am not permitted to run, so deleting it would have broken
  the environment irreversibly. Left in place with the reason recorded in `.gitignore`.

  **Also**: metadata gained ten keywords and twelve classifiers; all four console scripts verified to
  import and expose their entry function; 37 `__pycache__` directories cleared; `docs/README.md`
  removed as a second, stale index covering three of ten documents and describing two of them
  inaccurately (the README table is the index, and it is link-tested); `feature_matrix.md` and
  `v2_finalization_plan.md` moved to `docs/dev/` so `docs/` holds only user-facing pages;
  `crash_rate.sh` earns its place by being referenced from the troubleshooting entry it serves.

  New `tests/unit/test_repo_hygiene.py` (18 tests) guards all of it: nothing generated is tracked, the
  ignore file covers this project's artefacts, every console script resolves, keywords and classifiers
  are present, `py.typed` ships, every package data file is declared, the config reference does not
  offer removed pose sources or claim to be auto-loaded, every inter-document link resolves, and there
  is exactly one documentation index.

  **1728 tests, all seven gates green, 23 contracts kept.**

- **2026-09-15** — **The shipped assets now live inside the package.** Answering "do we support a
  non-editable install?" turned out to be answering "does anything ship?". The four layer USD files and
  `earth.usda` sat in a repo-level `usd/` directory, outside the distribution, while `package-data`
  shipped nothing at all -- so `pip install isaac-core` produced a package with no scene and no layers
  that could not compose anything. The giveaway was that `_resolve_scene_path` already searched
  `assets/scenes` and `_resolve_layer_search_paths` already searched `assets/layers`: the resolvers were
  written for packaged assets and only the files were in the wrong place.

  Each layer is now a directory holding its manifest and its USD side by side, which is exactly the
  shape `docs/authoring_layers.md` tells third parties to use -- our own layers had been the one
  exception. The repo-relative `parents[3]` fallbacks are gone, so an installed package and a source
  checkout resolve assets identically, and a test fails if any module reaches for the repo root again.
  Version bumped to 0.2.0 and the repository URL declared.

  **A 23 GB Cesium cache masquerading as a regression.** Straight after the move the system suite went
  from 97 s to 715 s and all three swarm tests stopped reaching a composed stage -- exactly what a
  broken asset move would look like. It was not: no moved USD file contains a relative reference, there
  were no stray processes, and the GPU was idle at 583 MiB and 36 C. The write-ahead log of the Cesium
  request cache had reached 23 GB. Deleting it took the swarm from 3 skipped in 646 s to **3 passed in
  32 s** and the full suite back to 94 s. The troubleshooting entry now describes the symptom rather
  than the disk usage, because a launch that hangs with an idle GPU looks nothing like a full cache and
  the size is the last thing anyone would check.

  **1730 tests, all seven gates green, 23 contracts kept, 13 system tests passing.**

- **2026-09-15 (ii)** — **Phase 5, the release gate.** Eight of the nine checks pass. The ninth, a
  fresh clone on a different machine with a fresh Isaac install, is the one that cannot be done from
  here.

  **Reading the README cold found three defects, and all three were the kind a reader hits in the first
  ten minutes.** The camera configuration example used `width` and `height`, which are read-only
  properties rather than config keys, so the single snippet a new user is most likely to copy was
  rejected at load -- every model forbids extra keys, so it was a hard error, not a silent one. The
  layer-authoring section documented registering a layer through a `[project.entry-points."isaac_core.layers"]`
  group; no such lookup exists anywhere in the source, so a layer registered that way is never
  discovered. And `--set` is `nargs=2`, so the `--set key=value` form shown in five files is rejected by
  argparse with "expected 2 arguments" -- every one of those command lines would have failed. Fixed, the
  entry-point gap is roadmapped, and three tests now guard them: one loads **every** TOML block in the
  README through the real loader, one rejects the equals form anywhere in the docs, and one fails if the
  entry-point claim returns.

  **A sub-agent got the `--set` direction backwards**, reporting the quickstart as wrong and the
  reference as right. Parsing both forms with the real parser showed the opposite. Worth recording
  because the finding was correct and the diagnosis inverted, and only running it settled it.

  **The roadmap had outrun the code.** It listed the distance sensor and bbox publisher as missing while
  both ship as wired layers the README documents, and marked the gimbal start angles, `max_rate_deg_s`
  and `sim.stage_units_in_meters` as dead config keys when all are applied -- `slew_towards` integrates
  toward the target every frame, and the dead-key guard's known-dead set is empty. Since the README
  points readers at that document as "what is not built yet", stale entries there contradict the
  feature list. Corrected, and the document now says which section is live and which is history.

  **Dead surface came out clean where it matters**: zero dead config keys, and the client's method
  names, the `Method` enum and the server's handlers are three identical sets of fifteen. One export
  went: `topics.GIMBAL`, naming a ROS topic that does not exist, since the gimbal is a control-plane
  command. The other unimported exports are field and return types of public models, which a package
  shipping `py.typed` is right to export even when nothing internal imports them by name.

  **Both Python versions.** 3.10 runs the full suite. Isaac's 3.12 has no `pytest`, so the unit suite
  cannot run there, but all 63 modules import cleanly under it and the 13 system tests drive the
  simulator on 3.12 end to end, which is the code path that matters.

  **1733 tests, all seven gates green, 23 contracts kept, 13 system tests passing.**

- **2026-09-15 (iii)** — **The Cesium write-ahead log, and a wrong conclusion corrected.** The system
  suite started taking 715 s instead of 97 s with all three swarm tests failing to reach a composed
  stage, immediately after the assets moved into the package -- which is exactly what a broken asset
  move looks like. It was not that: no moved USD file holds a relative reference, there were no stray
  processes, and the GPU was idle. The Cesium request cache's write-ahead log had reached 23 GB, and
  deleting it restored both.

  **Then it happened again at 763 MB**, which falsified the size explanation I had just written into the
  troubleshooting entry. Four trials settled it:

  | WAL | Result |
  |---|---|
  | 23 GB | 3 skipped, 715 s |
  | deleted | 3 passed, 32 s |
  | 763 MB | 3 skipped, 641 s (repeated three times) |
  | deleted | 3 passed, 24 s |

  So the trigger is the log existing in a state Cesium then labours over, not how large it has grown.
  A few hundred megabytes is enough, which makes "check whether it looks big" useless advice -- the
  README now says size is not the trigger and gives the symptom instead, since an idle GPU and a launch
  that never becomes ready look nothing like a cache problem.

  The harness now deletes the log before **every** launch attempt, which is the difference between a
  release gate and a coin toss: the full system suite went to **13 passed in 83 s** with a 244 MB log
  present beforehand. Two-vehicle launches are the sensitive case, presumably because each viewport
  streams its own tiles.

  Worth remembering that the first explanation was confidently wrong and survived one confirming
  experiment. It took a second failure at a thirtieth of the size to disprove it.

- **2026-09-16** — **Second machine, then an exhaustive pass over every feature, config key and devkit
  function.**

  **What the fresh machine proved.** A clean clone on different hardware installed from the README
  without edits, on **Isaac Sim 6.1.0-rc.26** rather than the 6.0.1-rc.7 everything was built against:
  driver 580.178.04, system Python 3.10.12, Isaac's 3.12.13. 1733 unit tests and all 13 system tests
  passed there, terrain streamed, and **startup did not segfault once in about five runs** -- so the
  one-in-three crash looks specific to this machine or to 6.0.1, not to the code. Eyes-on scenarios 1,
  3, 5, 6 and 8 all passed.

  **Four defects came back, and the first one was not what it looked like.** Setting `assets.hdri` did
  nothing. The reported cause was HDRI resolution; the actual cause was that `config/default.toml` had
  been edited in place, and that file is documentation which is never auto-loaded. The run banner said
  `Config: (defaults)` and nothing else. Both are now fixed: the banner names the file it is ignoring
  and how to pass it, and it no longer claims "(defaults)" when `$ISAAC_CORE_CONFIG` is set. The real
  HDRI gap underneath was that a bare filename was resolved against the working directory rather than
  `assets.search_paths`, so a new `resolve_hdri` handles bare names, absolute paths and `~`.

  **Ctrl-C left Isaac running.** `subprocess.call` plus `return 130` meant the interrupt was reported
  while the child carried on: Isaac ignores SIGTERM, kept initialising, and opened a window belonging
  to a command that had already exited. The launcher now owns the process in its own session and
  signals the whole group, SIGTERM then SIGKILL. Measured: two Isaac processes during startup, zero at
  3, 6, 12 and 20 seconds after one SIGINT, exit code 130 -- and a normal run still reaches a composed
  stage.

  **`doctor` warned about a working install.** The check looked for an `isaac_core` directory or an
  `.egg-link`; a PEP 660 editable install writes neither, only a `__editable__*.pth` and a `dist-info`.
  It now runs Isaac's interpreter and imports the package, which is definitive and costs 22 ms, and a
  real failure is now FAIL rather than a warning people learn to scroll past.

  **The pose sender needed two Ctrl-Cs.** Tk prints exceptions raised in callbacks and continues, so
  the first `KeyboardInterrupt` was swallowed by the periodic UI refresh. A SIGINT handler now
  schedules the shutdown onto Tk's own loop; measured, the window and process go two seconds after one
  SIGINT with exit code 0.

  **Every config key, properly this time.** `test_no_dead_keys.py` only greps for a field name
  appearing somewhere in the tree, which a mention in a comment satisfies. The new
  `test_every_key_is_wired.py` sets each of 63 leaf keys to a different valid value and diffs what the
  configuration layer actually produces -- the feature plan, the attribute writes, the resolved
  mounts, ports and topics -- with the config dump deliberately excluded, since "the value I set is in
  the config" proves nothing. Result: 30 keys reach the stage or the naming, 31 are runtime-only and
  each names the test that covers it, and **zero** do nothing. Keys that only matter in context are
  compared in that context, and a sampled value that fails validation is a test failure rather than a
  pass, so a key cannot drop out of the check unnoticed.

  That found a real one: **`vehicles.<id>.gimbal.rotation_frame` was consumed by nothing.** Only the
  vehicle's own `rotation_frame` is bound to the pose node, and that is the frame the gimbal offset
  composes onto. Two keys with the same name and different defaults, and the README documented the dead
  one, telling people to "keep this as body" -- advice about a setting that could not do anything.
  Removed from the schema, the config reference and the README.

  **Everything else, live.** 25 devkit checks against one running simulator: both `Sim` entry points,
  every `SimSession` method, the config proxy, the raw client, and `Sim.attach` from a separate process
  leaving the simulator alone afterwards. `set_pose` lands at z=483.3000 for 1000 m. `capture_frame`
  writes 1280x720, then 3840x2160, then **restores the viewport** -- proven by reading IHDR back off
  the PNGs. Path confinement, `width` without `height`, and an unknown vehicle name are all refused
  with useful messages.

  12 feature checks with ROS 2 sourced: 1441 frames on `image_rgb` at 1280x720 rgb8, 1446 `global_pose`
  messages reporting exactly the commanded position, 1446 `Range` messages reading **81.91 m** from
  600 m over terrain at 518 m, and 1446 `FrameBboxes` whose nine arrays are all the same length and
  name `Cube` and `Cube_01`. RTSP answers `RTSP/1.0 200 OK`.

  `pose_source = "ros"` drove the stage to z=883.30 for 1400 m and z=383.30 for 900 m from MAVROS-style
  `NavSatFix`. Two vehicles each followed their own UDP port to 483.30 and 883.30, published
  `/isaac_core/lead/...` and `/isaac_core/wing/...` with the un-namespaced names correctly absent, and
  served RTSP on 8554 and 8555. The recording devkit captured live topics into a 3.7 MB mp4 and 434
  pose records.

  Two smaller things fell out of that. `get_config` dumped the model while patches lived beside it, so
  patching `gimbal.max_rate_deg_s` and reading it back returned the startup value while the new one was
  in force -- the one thing a config reader must not do. And the README named the video timestamps
  sidecar `orbit.timestamps.txt` when the code writes `orbit.mp4.timestamps.txt`, a file the reader
  would never find.

  **Both scenario questions answered as "leave it".** Capture resizing the viewport is forced, not
  incidental: `IsaacCreateRenderProduct` reuses a product already targeting that camera, so the
  camera's product *is* the viewport's and there is no second surface to render into. It is restored
  afterwards, which the PNG sizes above confirm. And the lead viewport cannot be renamed, because
  `omni.kit.viewport_widgets_manager` looks its window up by the literal string `"Viewport"`; renaming
  it is the regression that broke every launch once already.

- **2026-09-16 (ii)** — **The swarm system tests, and a fix that made things worse.** After the
  verification above, the full system suite started reporting the three two-vehicle tests as skipped
  with "never reported a composed, ready stage", taking twelve minutes to do it, while the same module
  on its own passed in 24 seconds and a two-vehicle launch driven by hand was ready in 20.6 s windowed
  and 20.6 s headless, with both cameras composed.

  The readiness wait gave a launch twenty seconds after the control plane first answered, so the
  obvious move was to raise it. That was wrong, and measurably so: at two minutes the suite stopped
  skipping and started **hanging for forty minutes** on the first swarm test, and had to be killed,
  leaking two Isaac processes. When a two-vehicle stage in a multi-module run does not compose, it does
  not compose *at all*, so a longer budget only buys a longer wait for the same answer. Reverted to
  twenty seconds, with the reasoning written down so the next person does not try the same thing.

  What actually restores it is clearing the Cesium write-ahead log and the stale `/tmp/carb.*`
  directories before the run: **13 passed in 83 s**. The harness now clears both before every launch
  attempt rather than only the cache file, which is the honest fix -- these are the two documented
  causes of a launch that never becomes ready, and both are caches.

  Recording this because the first diagnosis was a guess dressed as a fix, and the experiment that
  disproved it took forty minutes to run.

- **2026-09-16 (iii)** — **What the swarm system tests are actually doing, and two wrong guesses on the
  way.** The three two-vehicle tests skip in roughly half of full-suite runs on this machine while
  passing on their own in 24 seconds, passing on the second machine 13/13, and passing here 13/13
  several times. Each failed run costs about eleven minutes in timeouts, so it looks far worse than it
  is.

  First guess: the readiness budget was too small. Raising it from twenty seconds to two minutes turned
  a quick skip into a **forty-minute hang** that had to be killed and leaked two Isaac processes.
  Reverted. Second guess: pytest's output capture was blocking a very chatty startup, since the same
  two modules passed with `-s` in 61 seconds. The launcher now writes the simulator's output to a file
  instead, and the tests skipped anyway -- the `-s` run had simply been a lucky one.

  That redirect is worth keeping regardless, because it produced the first real evidence. The
  simulator's own log from a *failed* run shows it composed both vehicles and started:

  ```
  control plane listening on 127.0.0.1:8792
  Feature layers:
    ✓ camera_udp [lead]
    ✓ camera_udp [wing]
  simulation running
  ```

  So the stage was ready and the failure is on this side of the wire: `session.state()` did not return a
  ready state within its ten rounds even though the simulator was serving. The readiness helper caught
  every exception and threw the reason away, which is why this took so long to see. It now reports the
  last state it saw and the last error, so the next failure names its own cause instead of being a
  mystery.

  Leaving it there for V2. The two-vehicle **product** is verified by direct measurement rather than by
  this fixture: lead and wing at exactly 483.30 and 883.30 on their own UDP ports, namespaced topics
  present and the un-namespaced names correctly absent, RTSP answering on both ports, both cameras
  composed, ready in 20.6 s headless and 22.4 s windowed, nothing leaked. What is unreliable is the
  harness, on this machine, and it now leaves a log and a reason behind when it fails.

- **2026-09-16 (iv)** — **V3 phases 4, 6, 5 and 1.1/1.2.** Four of the five implementation tasks, each
  verified against a running simulator rather than only under pytest.

  **Phase 4, no environment variable names an Isaac install.** `$ISAACSIM_PATH` is gone from
  resolution, which is now explicit `--isaac-path`, then config, then a probe. That exposed a real gap:
  **`sim.isaac_sim_path` was named in error messages and the doctor hint but did not exist in the
  schema**, and `config_path` was never passed at any of the three `locate()` call sites. With the
  variable removed that was the only non-flag escape hatch, and it was entirely non-functional. Added
  the field and wired it into the CLI, the doctor and `Sim.launch`. `crash_rate.sh` was worse than a
  print: it defaulted to `${ISAACSIM_PATH:-$HOME/isaacsim}`, so it could benchmark a different install
  than the one `isaac-core` runs, silently. Proven by poisoning both variables with bogus paths and
  watching doctor, `run --dry-run` and the crash-rate resolution all still find the real install.

  **Phase 6, completion is installed rather than offered.** The generator was always fine; nothing
  installed the script, so TAB completed filenames and the feature was invisible. `setup.sh` now
  installs it as a sixth step, `doctor` reports whether it is installed, and the script is written to
  `~/.local/share/isaac-core/` with a marker-wrapped source line appended to the shell's rc file,
  idempotently. Verified by driving the generated function in a real bash with `COMP_WORDS` set and
  reading `COMPREPLY` back: `isaac-core r` offers `run`, `--set sim.h` offers `sim.headless`, and
  `--set vehicles.` offers all twenty-four deep keys.

  **Phase 5, one camera per vehicle and it has no name.** `cameras: dict[str, CameraConfig]` became
  `camera: CameraConfig`, and the `{camera}` placeholder is gone from the manifest vocabulary, the
  planner, the configurator and the topic resolver. The name only ever existed to distinguish cameras
  that could not coexist. The old table is rejected with a message naming the new form, because
  "Extra inputs are not permitted" tells nobody what to write. The trade is pinned by a test: a
  **thermal layer still adds a second camera to the same vehicle**, mounted under the vehicle so it
  moves with the airframe, with its own `[layers.thermal_cam]` value reaching its own prim, composing
  alongside `camera_udp`. Live proof that the new key path reaches the stage: launching with
  `camera.fov_deg = 55` recomputed `horizontalAperture` to 23.73, where the default at fov 78.1 is
  36.97.

  **Phase 1.1, `PoseBot`.** Every motion primitive already existed as a pure function; what was missing
  was the object that holds a `VehicleState` and a transport so calls compose. Angles are degrees at
  this layer because every other user-facing surface uses degrees. `MotionLimits` is carried by the bot
  and **enforced**, closing the roadmap item where limits were coded and tested while nothing applied
  them. There is no `hold()`: the simulator holds the last good pose by design, so it would be a
  `sleep` wearing a method's clothes.

  Two pre-existing bugs fell out of building it. **`PathTrajectory` never arrived at its final
  waypoint** -- the step count truncates, so a path asked to end at 1100 m ended at 1098.7, and "fly to
  this point" never got there. And **a closed `UdpPoseTransport` silently resurrected**: `close()` only
  dropped the socket and `send()` lazily recreated it, while `FakePoseTransport`, its drop-in
  replacement, raised. A use-after-close was therefore caught in tests and not in production.

  **A ROS 2 transport, so the GUI's per-tab source is one line.** `Ros2PoseTransport` publishes the two
  messages MAVROS publishes, converting NED to ENU once. `rclpy` is imported on first use, not at module
  import, because this module is imported inside Isaac where `rclpy` cannot be. Both transports satisfy
  `PoseTransport`, so `PoseBot` drives either without knowing which.

  **The devkit may now use the pure kernels.** The layering contract forbade `devkit -> vehicle` and
  `devkit -> geo` alongside `devkit -> sim`. Only the `sim` half protects anything: `vehicle` and `geo`
  are pure Python over dependencies that are already required and import nothing from `sim`. The
  reasoning is written into `pyproject.toml` beside the contract.

  Live: UDP bot to 1000 m gives stage z 483.30, down 300 gives 183.30, forward 200 travels exactly
  200.0 m, `fly_path` ends at exactly 1400.0; ROS bot to 1200 m gives 683.30 and down 400 gives 283.30.
  Nothing leaked in any run.

  **1848 tests, all sixteen hooks green, 23 contracts kept.** The multi-tab GUI is the one task left;
  its controller already carries the per-tab pose source, target labelling and copy-as-snippet.

- **2026-09-16 (v)** — **Phase 1.3, the pose sender is one window with a tab per vehicle.**
  `PoseSenderController` was already UI-free and already per-target, so the work was giving it a pose
  source, a readback and a stream URL, then wrapping N of them in a notebook. The source is the only
  thing that differs between tabs: both transports satisfy `PoseTransport`, so one window drives a UDP
  vehicle in one tab and a ROS one in another with nothing else changing.

  `SenderTabs` holds the model rather than the widgets, because the interesting part is not the
  notebook. A new tab lands on `33333 + tab index`, which is exactly how the simulator allocates a
  vehicle's port, so a two-vehicle swarm needs no arithmetic from the operator. The last tab refuses to
  close, since a window with no tabs has nothing to send and no way back.

  Also in: a readback line showing the stage's own transform beside what the tab is sending, arrow keys
  for yaw and pitch with PageUp/PageDown for altitude, a "Fly there" that ramps over three seconds
  instead of teleporting, copy-as-`set_pose` and copy-as-`[geo]`, a per-tab stream button, a packet
  counter, and pause. The ramp animates the controller rather than driving a second transport, so
  nothing has to arbitrate between two writers on one port.

  **Two bugs found by running it against a real simulator rather than by reading it.**

  The readback passed the *tab's label* as a vehicle id. A tab is called `vehicle` by default while the
  simulator had `drone_0`, so every readback failed -- and the failure was reported as "no simulator on
  127.0.0.1:8760", which is the wrong diagnosis entirely. A tab now names a vehicle only when it has
  adopted a real one, connection failures and error replies are reported differently, and a bad vehicle
  name produces `unknown vehicle 'nope'; configured vehicles: drone_0`.

  The stream button guessed the single-vehicle mount path. On a swarm the simulator namespaces every
  mount including the first, so the button aimed at `/stream` while the server offered `/lead/stream`.
  Measured against the running RTSP server: the corrected URLs return `RTSP/1.0 200 OK` for both
  vehicles, the old guess returns `404 Not Found`, and Kit logs `no factory for path /stream` behind it.

  A tab now adopts a vehicle from the running simulator when one is reachable, which is what makes the
  readback correct on a swarm without the operator configuring anything. Verified live on two vehicles:
  tabs named themselves `lead` and `wing`, took ports 33333 and 33334, and read back 483.30 and 883.30
  independently -- matching `get_pose(vehicle=...)` exactly.

  Ctrl-C still closes the window and every sender with it, checked after the refactor: one SIGINT,
  exit 0, two seconds.

  **1871 tests, all sixteen hooks green, 23 contracts kept, 13 system tests passing.**

- **2026-09-17** — **Six real bugs Ofer found in work I had reported as done, and why my testing
  missed each one.** Recording the pattern rather than only the fixes, because the pattern is the
  lesson: every one of these was invisible to a test I had written to check the thing it missed.

  **The rotation bug, and it was three bugs.** With `rotation_frame = world`, pitching down 30 and then
  yawing repeatedly produced roll, and the horizon would not stay put. Three separate defects stacked:

  1. *Expression.* The wire carries Euler angles and the simulator rebuilds them with the vehicle's
     configured frame, but every conversion in the `vehicle` stack used the library default, which is
     `BODY`. A world-composed matrix decomposed as body angles: truth roll 0 / pitch -30, reported roll
     30 / pitch 0. `VehicleState` now carries its frame, and every conversion uses it.
  2. *Intent.* In the world convention the angles compose as yaw about world up, then pitch, then roll,
     so "pitch by 30" means moving the pitch term. Rotating about the fixed world Y axis instead is a
     different operation, and while yawed 90 degrees it rolls the airframe -- geometrically true, never
     what the caller asked. Measured side by side: world-Y pre-multiply gives roll -30, moving the
     pitch term gives pitch -30. Yaw is identical either way, which is exactly why yaw looked fine and
     hid the other two axes.
  3. *Path.* Endpoints being right is not enough. SLERP takes the shortest great-circle path in
     rotation space, which swung roll up to 45 degrees mid-turn and back, so the camera visibly rolled
     while every endpoint assertion passed. World-frame turns and `turn_to_point` now interpolate the
     angles, holding the axes nobody asked to move.

  **Why I missed it:** every rotation test turned one axis from rest, and single-axis attitudes
  decompose identically in both frames. A whole green suite could not see it. `test_multi_axis_rotation.py`
  now covers two- and three-axis attitudes, asserts the three turns commute in any of the six orders,
  and checks the intermediate poses rather than just the endpoints. Verified against the live stage
  transform: roll holds at 0.0 and pitch at 30.0 across four 90-degree yaws, where the third angle used
  to wander between +/-30.

  **`$ISAACSIM_PATH` was not actually gone.** `pyproject.toml` and `requirements.txt` both still told
  the user to run `$ISAACSIM_PATH/python.sh`, and both shell scripts used a local variable named
  `ISAAC_PATH`. **Why I missed it:** my guard checked `src/**/*.py`, `docs/` and the shell scripts, with
  a condition sloppy enough to permit `$ISAAC_PATH` in a script -- and it never looked at packaging
  files at all. It now walks every tracked file with a named exemption list, and I proved it by
  injecting the original offences into five different files and watching each one fail.

  **Completion installed but did nothing.** The rc line only takes effect in a new shell, so the shell
  that had just run setup still completed filenames. Now also written to bash-completion's completions
  directory, which is loaded on demand at the moment TAB is pressed -- so an already-running shell picks
  it up. **Why I missed it:** I tested the generated function directly instead of the installed path.

  **`PoseBot(start=...)` sent nothing.** The position only reached the simulator once a command ran, so
  `with PoseBot(start=...) as bot: pass` left the vehicle where it was. Entering the context now streams
  the starting pose briefly, streamed rather than sent once because a single datagram is lost if the
  node has not bound yet.

  **`move_to_point` did not look where it was going.** It now faces the target by default, recomputed
  per pose because the bearing changes as the vehicle travels, with the aim held through arrival --
  `atan2(0, 0)` is zero, which snapped the camera due north on the final frame. The look-at maths is
  extracted so `turn_to_point` and `move_to_point` cannot drift apart.

  **The stream player outlived the window.** `start_new_session=True` put it beyond reach, so closing
  the sender left an orphaned stream. The tab owns the players it spawns and stops them on close;
  measured live, one ffplay while open and zero after a single Ctrl-C. The URL is also an editable field
  now, because guessing cannot cover a second vehicle, a remote host or a hand-authored mount.

  Also: tab labels are positional (`tab_1`, `tab_2`, renameable by double-click) rather than a mix of
  simulator names and placeholders; the topic namespace has its own labelled row instead of an
  unexplained box; and Copy TOML carries the angles as a comment rather than dropping half of what was
  on screen.

  **1907 tests, all sixteen hooks green, 23 contracts kept, 13 system tests passing.**

### Known remaining issues

- **Sensor layers** are not built: distance sensor, bounding-box publishing, satellite
  imagery. See `docs/dev/roadmap.md`.
- **`capture_frame`** raises `NotImplementedError` -- needs Isaac's viewport capture API.
  Route it through `_on_main_thread` when implemented.
- **Monotonic frame id** cannot reach the image topic; `header.stamp` now covers most of
  that need.
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

## Auditing the feature matrix for the trap that let six bugs ship

The six bugs shared one shape: each test verified the thing as built, in the shape it was built, rather
than the surface a user meets. Since `docs/dev/feature_matrix.md` is the document that certifies what
works, it was audited row by row against that pattern.

**One row was simply false.** "Entry-point layers from another package | pass" cited
`sim/test_discovery.py`, which never mentions entry points, and no `importlib.metadata` lookup exists
anywhere in `sim/`. `docs/dev/roadmap.md` had listed the same capability as a gap the whole time, so two
documents disagreed and only the optimistic one was read. Corrected to **not built**.

**Gimbal composition was only ever tested on a level airframe.** The helper in `test_gimbal_axes.py`
hardcodes the attitude to `(0, 0, 0)`, so the interaction between airframe attitude and gimbal offset --
the only part that can actually be wrong -- was untested. Measured, it is correct: an airframe pitched
-30 with a gimbal at -30 looks 60 degrees down, a yawed airframe carries the gimbal's azimuth with it,
and a 45-degree roll converts pitch into `sin30·cos45` of elevation plus a swing, which is what a gimbal
bolted to a rolled aircraft must do. Missing coverage rather than a bug, now pinned by
`geo/test_gimbal_on_attitude.py`, including the case no test had: all three airframe axes and all three
gimbal axes non-zero at once.

**A count was stale.** The matrix said 13 `Method` enum members; there are 15. A hand-maintained number
in a certifying document always rots, so `test_feature_matrix_claims.py` now checks every count against
the code, verifies each cited file exists, and fails if a row claims "pass" for something the roadmap
calls unbuilt. Each guard was proven by injecting the original offence and watching it fail.

Rows that turned out genuinely strong, worth recording because they are the pattern to copy: the 51-byte
packet is anchored to a hex literal captured from the 2023 encoder rather than round-tripped through our
own constants, and the test says why; `isaac-core-inspect` drives a real client against a real server on
an ephemeral port; freeze-on-bad-data covers seven distinct malformed inputs; `slew_towards` limits each
axis independently along the shortest angular path, so the gimbal never had the mid-path error that
SLERP introduced in the vehicle stack.

### The audit found the swarm skip was never a Cesium problem

The system suite had been skipping its three swarm tests for many sessions, blamed on Cesium
write-ahead-log flakiness. Improving the skip message to print the last observed state exposed the real
cause immediately:

```
last state={'running': True, 'ready': True, 'stage_composed': True, ...}, last error=TimeoutError
```

The stage was ready and composed. What timed out was the `step` call the readiness helper made
*afterwards*, swallowed by a broad `except` and reported as "never reported a composed, ready stage" --
a message contradicted by the state printed beside it.

The cause is that main-thread methods wait for the simulator's main thread, and a two-vehicle headless
stage holds it for longer than the 60-second call timeout while the control plane keeps answering
`get_state` promptly. That timeout was hardcoded in `ControlClient` and not exposed through `Sim.launch`
or `Sim.attach`, so no user hitting this could have raised it either. It is now a `call_timeout_s`
argument on all three, defaulted unchanged at 60 s so a genuine hang still surfaces quickly.

With the harness passing 240 s the three tests run instead of skipping, where the suite had been 10
passed and 3 skipped. Three real tests had been silently not running, which is the same failure as the
matrix row: something reported a state that was not true, and nothing checked.

**Getting there took one wrong turn worth recording, because the fix caused it.** Consecutive full runs
began alternating strictly pass, fail, pass -- every green run 83-120 s, every red run 1041-1064 s, twice
within 0.2 s of each other, always the same three swarm tests. Background simulators were the obvious
suspect and were ruled out by measurement, not assumption: `/proc` showed zero Isaac processes before and
after every run, and the GPU returned to its 650-680 MiB desktop baseline each time.

The destabiliser was the cleanup itself. `_clear_startup_hazards()` runs before **every** launch attempt,
and killing leaked simulators had been added into it, so a kill matching any simulator command line fired
repeatedly during a run rather than once before it. Moving it to a session-scoped autouse fixture, which
runs once before any of the suite's own simulators exist, ended the alternation: **four consecutive full
runs, 13 passed each, at 122, 120, 117 and 108 s.**

That is also the safer design regardless of the flakiness. The match is on any simulator command line, so
running it while one of ours is alive would kill a session the suite still needs. Each module happens to
use exactly one module-scoped session fixture today, so nothing overlapped -- but that is a property of
the current layout, and a future module needing two sessions would have had its first one killed with no
obvious cause.

Three fixes came out of this, all kept: the call timeout is configurable and reaches the socket, the
harness clears the whole Cesium cache glob rather than two of its three files, and leaked simulators are
killed once at session start. The suite went from 10 passed and 3 skipped, where the skip named the wrong
cause, to 13 passed repeatedly.
