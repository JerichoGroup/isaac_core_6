# Project status — isaac_core_6

## Per-vehicle gimbal and frame capture

`set_gimbal` and `capture_frame` act on a single vehicle. With more than one configured they now
**refuse** with a message naming the vehicles, rather than silently acting on the first one and
reporting success — which is what they used to do.

To lift the restriction: give `set_gimbal` and `capture_frame` a `vehicle` argument (and
`capture_frame` a `camera` argument), key the gimbal target and current angles per vehicle instead of
one runtime-wide pair, and route both through the same `_vehicle_from` resolution `set_pose` and
`get_pose` already use. Around ten call sites resolve `next(iter(vehicles))` today.

Everything else is already per-vehicle: ports, topics, mounts, render products and RTSP streams. Only
these two commands are not.

## Live stage manipulation (lowest priority)

Both items below were removed from the shipped surface rather than left raising
`NotImplementedError`, so v2 ships nothing that is known not to work. Neither blocks any workflow
and neither existed in the 2023 repo. **These are the least important things on this roadmap.**

- **Runtime feature toggling** (`features.enable/disable`) — compose or remove a feature layer on a
  live stage. Today: list features in config before launch.
- **Runtime scene swap** (`load_scene`) — open a different scene without restarting. Today: set
  `sim.scene` and restart, which takes about fifteen seconds.

Whoever picks these up should know why they were deferred: every stage-lifecycle shortcut tried in
this project produced a *silent abort* rather than an error, so both need a crash-rate harness
before being considered done.

## Version 1 — shipped (2026-09-02)

The core sandbox is done, tested, and validated by hand as well as by suite.

`isaac-core run` launches Isaac Sim 6.0.1, composes the stage from layer manifests, and a
pose from **either UDP or ROS 2/MAVROS** drives the camera over real Cesium 3D Tiles
terrain. Both ROS 2 topics publish with real advancing timestamps
(`/isaac_core/global_pose` ~100 Hz, `/isaac_core/image_rgb` ~115 Hz).

- **1279 tests** pass in ~12 s with no GPU, no Isaac Sim and no ROS 2 installed.
- **23 import-linter contracts** enforce the layering and cannot silently rot.
- The intermittent startup segfault is fixed (0 in 27 launches, from 5 in 10).
- Eyes-on non-headless flight confirmed by Ofer: terrain renders, the viewport tracks the
  drone camera, motion is smooth and holds on stop, heading follows travel.
- Pose-sender GUI confirmed by Ofer: every control works.
- A feature audit went through the config surface, position/math pipeline, devkit, control
  plane, CLI/install, OGN nodes, contracts and debug tools. Everything now either works,
  is fixed, is honestly roadmapped, or was removed with approval. Nothing ships claiming to
  work while broken.

What v1 deliberately does **not** include: the sensor layers (distance sensor, bbox, SAT),
live-tested streaming, runtime scene/feature mutation, and swarm. Those are Version 2.

### v1 closeout

| Item | Status |
|---|---|
| Eyes-on non-headless flight | Done — confirmed correct by Ofer |
| README images | Deferred by Ofer to a later pass (not blocking) |

---

## Done

### Kernel (pure Python, no Isaac/ROS/GPU)

- `contracts` — packet spec, topics, ports, frames, pose types, prim paths, stamp conversion. Single source of truth for the wire format and all shared constants.
- `config` — pydantic v2 schema (`extra="forbid"`, frozen), layered TOML loader (defaults → file → env → CLI → runtime patch), provenance tracking (`config explain`), `dump_toml`.
- `geo` — `EnuConverter` (LLA↔ECEF↔ENU both directions), NED↔ENU, euler/matrix/quaternion helpers, `compose_rotation` honouring D14 (body vs world frame), SLERP with lerp fallback, haversine distance.
- `protocol` — 51-byte UDP encode/decode (byte-identical to the 2023 encoder, verified against a captured hex literal), typed error hierarchy, `HoldLastGoodDecoder` preserving hold-on-corruption.
- `vehicle` — `VehicleState`, `MotionLimits` (speed/accel/turn-rate/climb caps), `Trajectory` protocol with Hold/Orbit/Path, motion primitives. Pure generators — no sockets, no threads, no time.
- `control` — JSON-RPC over local socket, registered handlers, loopback-by-default with token off-host, path confinement. Port opening is the readiness signal (replaces log-grep + sleep).

### Simulator integration

- `sim.runtime` — Isaac Sim 6 lifecycle (enable extensions, warm up frames, open stage, play, step loop). Never mentions a feature by name.
- `sim.composer` — mounts USD layers via references, applies config bindings to prim attributes with type coercion.
- `sim.planner` — resolves `required_feature_ids()` from config, checks stage capabilities, produces enabled/skipped plan with reasons.
- `sim.configurator` — declares binding resolution (config key or derivation), thread-safe main-thread dispatch.
- `sim.manifest` — `LayerManifest` + `Binding`, discovery with duplicate-id detection, self-referential mount rejection.
- `sim.georeference` — derives ENU reference from the scene's Cesium georeference (D18).
- `sim.capabilities` — `StageInspector` Protocol + `FakeStageInspector` for tests.
- Warm-up fix: 60 frames pumped after enabling extensions, before any stage operation. Measured: 0 segfaults in 27 consecutive launches vs 5 in 10 before.
- `get_pose` control handler reads live prim transform from a running stage (verified: alt=1000 → translate 483.3, alt=1500 → 983.3).

### Control plane

- JSON-RPC methods: `ping`, `get_state`, `get_capabilities`, `get_config`, `set_config`, `get_pose`, `pause`, `resume`.
- Client usable from any machine with no Isaac. `wait_until_ready()` replaces log-grep.
- Main-thread task queue: handlers submit work; step loop drains it each frame with a bounded timeout.

### Devkit

- `Sim.attach(host, port)` — connects to a running sim anywhere, returns a `SimSession`. No filesystem knowledge needed.
- `Sim.launch()` — resolves the Isaac install, writes a resolved config, spawns
  `python.sh -m isaac_core.sim`, and waits for the control plane. The returned session
  owns the process, so closing it stops the simulator. Tested with an injected launcher,
  so the suite never starts Isaac Sim.
- `UdpPoseTransport` + `pace()` — I/O that the vehicle layer deliberately lacks.
- `TopicRecorder` — one generic recorder replacing four ~95% copy-paste capture classes.

### CLI and install

- `isaac-core run` — spawns `<isaac_python> -m isaac_core.sim`, forwards fully resolved config. `--dry-run` available.
- `isaac-core doctor` — finds Isaac Sim 6, checks version, flags missing steps.
- `isaac-core config dump` / `isaac-core config explain <key>`.
- `IsaacInstall` — probes known paths, validates VERSION file, rejects stale installs.
- `scripts/setup.sh` — three-interpreter install + extension link.
- `scripts/link_extensions.sh` — symlinks extensions into `extsUser`, clears stale OGN caches.

### Debug tools

- `isaac-core-pose-sender` — tkinter GUI, all state in `PoseSenderController` (unit-tested without tkinter). Fixes the old GUI's degrees/radians lie.
- `isaac-core-inspect` — terminal tool printing state, capabilities, live pose and config; `--poll` watches the pose change in real time.
- `scripts/send_test_pose.py` — `hold`/`orbit`/`path` modes driving `vehicle` → `protocol` → `devkit.transport`.

### Extensions (OmniGraph nodes)

Five nodes across two extensions (plus the template):

| Extension | Node | Purpose |
|---|---|---|
| `isaac_core_ogn.math` | `GlobalPositionToLocalPosition` | LLA→ECEF→ENU + gimbal compose |
| `isaac_core_ogn.math` | `QuaternionToEuler` | bridges Isaac's per-field quaternion to euler |
| `isaac_core_ogn.math` | `EulerToQuaternion` | euler → quaternion for GeoPoseStamped |
| `isaac_core_ogn.math` | `SecondsToRosStamp` | double seconds → int sec + uint nanosec |
| `isaac_core_ogn.position` | `UdpToGlobalPosition` | 51-byte UDP → LLA + RPY (ENU) |

All are thin adapters delegating to `isaac_core.geo`/`.protocol`/`.contracts`. No rclpy anywhere (guard test enforces it). The old `isaac_core_ogn.sensors` extension is deleted — all ROS publish/subscribe is done by Isaac's C++ `isaacsim.ros2.bridge`.

### USD

- `earth.usda` base scene with Cesium georeference (32.22°N, 35.26°E, 516.7 m).
- `camera_udp.usda` — full OmniGraph: UDP receive → NED→ENU → ECEF→ENU → write translate/orient, plus image publish and timestamped pose publish.
- `camera_ros.usda` — same output path with ROS 2 subscribers on MAVROS topics as input.
- Layer manifests (`layer.toml`) for both, declaring requires/provides/bindings.

### Testing

- 1279 tests passing (`python3 -m pytest -q` → `1279 passed in ~12s`).
- Guard tests: kernel purity (imports in subprocess with omni/rclpy/gi forced unimportable), no-dead-Isaac-API (AST scan), no-Python-ROS2-in-extensions, no-deprecated-OGN-API, USD layer validation (dangling connections, two-source inputs, missing orient op, Globe Anchor presence, layer size), graph wiring (unconnected `execIn` detection), extension packaging, OGN key naming.
- 23 import-linter contracts KEPT (verified this session).

### Tooling

- ruff 0.8.2, mypy 1.14.1, pytest 8.3.4, pre-commit 4.0.1 — all pinned and verified matching.
- `test_packaging.py` fails if `requirements.txt` drifts from `pyproject.toml` or dev tool pins disagree with `.pre-commit-config.yaml`.

---

## Parity with isaac_core_2023

Built from the 2023 README's own feature list (its flags, config constants, and ROS topic
tables), cross-checked against its source tree. This is the definitive gap list driving
Version 2.


| Capability | 2023 | isaac_core_6 | Notes |
|---|---|---|---|
| UDP pose pipeline (51-byte packet) | Yes | Done | Byte-identical wire format, verified against captured hex |
| ROS 2 pose input (MAVROS) | Yes (rclpy in-node) | Done | Isaac's C++ bridge subscribers, no rclpy needed |
| Camera image publish | Yes (`/isaac_core/image_rgb`) | Done (~115 Hz) | Via `ROS2CameraHelper`, real timestamp |
| Global pose publish | Yes (abused quaternion field) | Done (real quaternion, D16) | `GeoPoseStamped` with correct orientation |
| Timestamp on messages | Fake (sim time, no advancing stamp) | Done (real advancing stamp) | `SecondsToRosStamp` from `IsaacReadSimulationTime` |
| Distance sensor | Yes (custom raycast script node) | Not yet | Parked; Isaac 6 ships a native physics raycast sensor |
| Bounding-box publish | Yes (script node + custom msgs) | Not yet | Parked; needs sensor layer USD |
| SAT (screenshot capture) | Yes (script node) | Not yet | Lower priority; `capture_frame` control method is the planned route |
| RTP/GStreamer streaming | Yes (out-of-process sim lib) | Coded, not live-tested | `sidecar.rtp` exists; sidecar disabled by default |
| Gimbal control | Yes (ROS topic, snaps instantly) | Config + rate limits coded | `max_rate_deg_s` in config; needs the ROS subscriber wired in USD |
| Dev kit — launch manager | Yes (`HostIsaacManager`, `DockerIsaacManager`) | Done: `Sim.attach()` and `Sim.launch()` | One API for local and remote; Docker path dropped for now |
| Dev kit — capture/recording | Yes (4 copy-paste classes) | 1 generic `TopicRecorder` | Needs rclpy (host-side), tested without ROS |
| Dev kit — UDP senders (orbit, path, bot) | Yes | `vehicle` + `send_test_pose.py` | Cleaner: generators, not socket-bound classes |
| Debug GUI (tkinter) | Yes (udp_sender, ros_sender) | Done (`isaac-core-pose-sender`) | Fixes degrees/radians lie |
| Cesium 3D Tiles terrain | Yes (hardcoded internal IP) | Done (config-driven URL) | No hardcoded IPs in source |
| Docker workflow | Yes (two-stage, warm-start commit) | Deliberately dropped (D10) | Can revisit post-v1 |
| VS Code autocomplete (`link_app.sh`) | Yes | Dropped | Isaac 6 installs differently; not needed with the type stubs approach |
| Custom ROS 2 messages (Gimbal, Bbox, SAT) | Yes (`isaac_ros2_messages`) | Not needed for camera pipeline | Bbox/SAT layers will need their own msgs |
| Monotonic frame id in image header | Attempted (broken: baked at init) | Not yet | `header.stamp` covers most of the need; C++ node likely required |
| `--distance-sensor` → `/isaac_core/distance_sensor` (`sensor_msgs/Range`) | Yes | **Missing** | Needs USD layer + publisher; Isaac 6 has a native raycast sensor. Also `LASER_MIN/MAX_RANGE` config |
| `--bbox-publisher` → `/isaac_core/bbox` (`FrameBboxes`) | Yes | **Missing** | Needs USD layer, custom msgs, per-object Globe Anchors |
| `--sat` frame capture | Yes (ROS topic in) | **Missing**, and moving to the control plane (D20) | Capture is a *command*, not data, so it becomes `capture_frame(path, width, height)` in M5 — with a requested resolution, which 2023 could not do |
| `--image-rtp` → RTP video stream | Yes (custom sidecar) | Superseded by **native RTSP** (D21) | Isaac 6's own RTSP, wired into both camera graphs and always on — no flag, no sidecar. M4 |
| Live gimbal control | Yes (ROS topic) | Moving to the **control plane** (D20), M3 | The ROS subscriber is removed; `set_gimbal` replaces it, and external callers use `Sim.attach()`. Config start angles and `max_rate_deg_s` become live at the same time |
| Custom ROS 2 messages | Yes: `Gimbal`, `Bbox`, `FrameBboxes`, `SATOutput` | Vendoring **only `Bbox` + `FrameBboxes`** (M1) | We *import* `isaac_ros2_messages` but ship no definitions — it only resolves because the 2023 workspace is built on this machine. `Gimbal`/`SATOutput` are not vendored: both became control-plane commands (D20), so nothing would consume them (D23) |
| Second scene (`full_warehouse.usda`, no Cesium) | Yes | Not planned | Ofer's call: feature development first, not scene work. Revisit if offline/CI runs are ever needed |
| `MAX_OUTPUTS_ROS_HRZ` (publish rate cap) | Yes | **Deliberately not carried over** (D22) | No ROS 2 rate manipulation anywhere — it invites untimed-message chaos. The related real problem, correct video frame timing, is handled in M7 via message timestamps |
| Global pose orientation semantics | Abused quaternion (x=roll, y=pitch, z=yaw) | Real quaternion (D16) | **Migration note:** any 2023 consumer reading RPY out of the quaternion fields must be updated |

---

## Version 2 — plan

**Goal:** everything the 2023 repo could do, plus the features already designed for but not
built, and the whole thing production ready.

**Definition of done for v2** (the v1 bar, plus one):
1. Every feature verified against a running simulator, not just unit-tested.
2. No config key that parses but does nothing (`test_no_dead_keys.py` stays green with an
   empty known-dead list).
3. No claim in the README that is not true.
4. Runs on a machine that has never seen the 2023 repo.
5. **No dead or unused code** (D23). A sweep before sign-off is the backstop, not the method —
   the habit is to delete on the way past and to prefer what Isaac Sim already offers over
   building our own. The RTP sidecar is the cautionary tale: ~340 lines superseded by a native
   feature.

Milestones are ordered by dependency, then by value. M1 unblocks M2-M4.

### M1 — Vendor the ROS 2 messages we actually use (foundation)

The portability gap. `devkit.recording` imports `isaac_ros2_messages.msg.FrameBboxes`, but this
repo ships **no** message definitions. It resolves today only because the 2023 workspace happens
to be built on Ofer's machine — a fresh clone would fail.

**Decided (Ofer):** take the 2023 `simulation/ros2_interfaces/` package and keep the name
`isaac_ros2_messages`, so existing consumers keep working — but vendor **only what has a
consumer** (D23, no dead code):

| Message | Vendor? | Why |
|---|---|---|
| `Bbox.msg` | Yes | Needed by the bbox publisher (M2) |
| `FrameBboxes.msg` | Yes | The published wrapper (`Header` + `Bbox[]`) |
| `Gimbal.msg` | **No** | Gimbal becomes a control-plane command (D20) and the ROS subscriber is being removed (M3) — nothing would publish or consume it |
| `SATOutput.msg` | **No** | Frame capture becomes a control-plane command (D20) — nothing would consume it |

Already inspected (the "validate it" part): package `isaac_ros2_messages` **v0.2.0**,
`ament_cmake` + `rosidl_default_generators`, depends on `std_msgs`, `geometry_msgs`,
`builtin_interfaces`. `Bbox.msg` carries **16 fields** — name, in_frame, is_visible, pixel box
(x1,y1,x2,y2), **lat/lon/alt, roll/pitch/yaw, distance_x/y/z**.

**Finding to fix in M2:** our `bbox_recorder` serialiser reads only **7 of those 16 fields** —
it silently drops the geodetic position, orientation and per-axis distances the publisher sends.

Work: vendor the trimmed package; build instructions in `scripts/setup.sh`; a `doctor` check
that the messages are importable; a guard test so the recorders fail with an actionable message
rather than an opaque `ModuleNotFoundError` when the package is missing.

Acceptance: a machine with only this repo + ROS 2 Humble can build the messages, and
`bbox_recorder()` constructs without the 2023 repo present. Size: small-medium. Owner: Kiro.

### M2 — Sensor layers (the biggest parity gap)

Two features, one pattern each: a USD layer Ofer authors, a `layer.toml` manifest, a config
block, tests, and live verification. Both are **data out**, so both are ROS topics (D20).

- **Distance sensor** → `/isaac_core/distance_sensor` (`sensor_msgs/Range`). Isaac 6 ships a
  native physics raycast sensor that likely replaces 2023's custom script node entirely —
  evaluate that first. Needs `min_range`/`max_range` config (2023's `LASER_MIN/MAX_RANGE`).
- **Bbox publisher** → `/isaac_core/bbox` (`FrameBboxes`). This is where Cesium Globe Anchors
  genuinely belong: each target pinned to a fixed geographic spot, unlike the camera. Depends
  on M1.

Frame capture (2023's `--sat`) is deliberately **not here** — per D20 it is a command, so it
lives on the control plane in M5, not on a ROS topic.

Acceptance: each publishes correct values against a running sim, verified the way the camera
pipeline was (real numbers on the topic, not just "a topic exists"). Size: medium-large.
Owner: Kiro for code/manifests/tests, Ofer for USD authoring (D15).

### M3 — Gimbal over the control plane

Half-built today: the USD has a gimbal ROS subscriber, the config is dead, and nothing limits
the slew rate.

**Decided (Ofer):** commanding the gimbal is control-plane work (D20), and the ROS
`/isaac_core/gimbal` subscriber is **removed** — anything external that wants to move the gimbal
uses `Sim.attach()` and commands it from there. That also removes the need for `Gimbal.msg`.

- Add a **`set_gimbal(roll_deg, pitch_deg, yaw_deg)`** control method.
- **Remove** the `ros2_subscriber` gimbal node from `camera_udp.usda` and `camera_ros.usda`
  (Ofer, USD). This also unblocks the config problem below: those node inputs are currently
  *connected* to the subscriber, and a connected USD attribute ignores authored values — with
  the subscriber gone, config and commands can both write them.
- `start_roll_deg` / `start_pitch_deg` / `start_yaw_deg` then actually reach the node at launch.
- `max_rate_deg_s` slew limiting — integrate toward the target instead of snapping. 2023 snapped
  unconditionally, so this is a real improvement over parity, not just parity.
- Document the angle convention in text (2023 relied on a reference image).

Acceptance: config start angles visible on the prim at launch; a commanded step visibly slews at
the configured rate rather than jumping; no gimbal ROS topic remains. Size: medium.
Owner: Kiro + Ofer (USD).

### M4 — Native RTSP streaming (replaces the RTP sidecar)

**Decided (Ofer, D21):** use Isaac Sim 6's **native RTSP** rather than our own RTP
implementation, wired into the **image-publisher action graph in both camera layers** so it is
**always on** — no flag, no sidecar process. Whoever wants the stream consumes it; whoever does
not ignores it, exactly like the image topic.

- Investigate how Isaac 6 exposes RTSP: which extension/node, what settings, what URL form.
- Add it to `camera_udp.usda` and `camera_ros.usda` beside the existing image publish, so both
  pose sources stream identically (Ofer, USD; Kiro specifies).
- Config surface for the stream URL/port, consistent with how topics are configured.
- Verify a real client (VLC/ffplay) plays the stream off-box.
- **Cleanup (D23), decided:** delete `sidecar/rtp.py` (~340 lines) and `tests/unit/sidecar/test_rtp.py`
  (16 tests), plus the `RtpVideoService` export in `sidecar/__init__.py` and its registration
  import in `sidecar/__main__.py`. **Keep** `sidecar/service.py` — the supervisor/registry
  framework (~310 lines, 15 tests) is genuinely reusable for future out-of-process services.
  Also retire `DEFAULT_RTP_VIDEO_PORT` / `DEFAULT_RTP_META_PORT` from `contracts/ports.py` if
  native RTSP does not need them.

Acceptance: a stream plays off-box from a default `isaac-core run` with no extra flags, and no
RTP code remains. Size: medium. Owner: Kiro (+ Ofer to view the stream and author the USD).

### M5 — Runtime control (commands over the control plane, D20)

The devkit already exposes most of these; they currently fail with an honest
"not implemented".

- **`capture_frame(path, width, height)`** — this is 2023's `--sat`, moved off ROS per D20.
  Ideally capture at a **requested resolution** independent of the viewport, which 2023 could
  not do. Makes `sim.control_plane.output_root` live (it is confined against traversal
  already).
- `reset()` — define the semantics first: timeline only, pose, or full stage reload?
- `features.enable/disable(...)` — compose/decompose a layer on a live stage.
- `config.patch(...)` — define the safely-mutable subset; most config is frozen for good
  reason.
- `set_pose` and `load_scene` — in the `Method` enum, not yet exposed.
- `set_gimbal` — see M3.

Acceptance: each devkit method works against a live sim. The static coverage guard already
prevents any of them regressing to unregistered. Size: medium-large. Owner: Kiro.

### M6 — Swarm / multi-vehicle

The architecture was built for this (per-vehicle ports, topic namespacing, `{instance}`
templating) but it has never been exercised with more than one vehicle.

- **Camera-key templating** — manifests hardcode `cameras.eo`; add a `{camera}` placeholder
  so camera keys and multiple cameras per vehicle work without editing `layer.toml`.
- **Unify the vehicle mount** — `config.mount` and the manifest `mount` are separate axes
  today; add a `{vehicle_mount}` placeholder rather than blindly overriding.
- Compose multiple camera layers simultaneously; per-vehicle `get_pose`; mixed pose sources
  (one drone on UDP, another on ROS).

Acceptance: two vehicles flying independently, correctly namespaced topics
(`/isaac_core/<vehicle>/...`), `get_pose` per vehicle. Size: large. Owner: Kiro.

### M7 — Production readiness

The polish that makes it safe for the team to depend on.

- **Video recording at the true frame rate.** The one place frame rate legitimately matters
  (D22). 2023's `video_capture` took a destination path *and an fps the user had to guess*,
  then wrote an mp4 at that constant rate — but Isaac runs at a variable ~30-50 fps, so parts
  of every recording played too fast and parts too slow. Fix it properly: derive the real
  timing from each message's `header.stamp` (we publish real advancing timestamps now, which
  2023 did not, so this is finally possible) and write either a variable-frame-rate video or
  one whose fps matches the measured average, with no fps argument from the user.
- **Cesium base-URL + multi-tileset** — config should carry only `http://<host>:<port>` and
  preserve each tileset's path, so several tilesets can be served from one host. Today the
  whole `cesium:url` is replaced.
- **Tile-streaming hiccup** — the render-side hitch during flight (the pose pipeline is proven
  smooth: 119 samples, 0 outliers). Investigate Cesium cache size / concurrent tile loads /
  pre-warming; optionally ease recovered frames in with the existing `slerp`.
- **Camera intrinsics as first-class config** — `focus_distance` and direct aperture control,
  so common tuning does not need a `prim_override`. (Note: **no** `publish_rate_hz` — rate
  manipulation is deliberately out of scope per D22.)
- **Resolve the remaining dead config keys** — `stage_units_in_meters`, `ros2.use_sim_time`,
  and the gimbal keys (M3). Target: the known-dead list is empty.
- **Fully silent boot** — the launcher/Warp banner still prints before Kit initialises.
- **`MinimalRendering`** — document as unsupported for Cesium terrain, or validate and warn.
- **Two georeferences** — unify `geo.enu_reference` with the scene's `/CesiumGeoreference`
  (today they can disagree and only a warning fires).
- **Monotonic frame id** — needs a C++ OmniGraph node; `header.stamp` covers most of it.
- **MAVLink pose source** — `pose_source = "mavlink"` is in the schema, unimplemented.
- **CLI tab-completion**, **README images**, and **Docker workflow** if the team wants
  reproducible deployments again (dropped in D10).

Explicitly **not** in scope: a Cesium-free scene. Ofer's call — the goal now is feature
development, not scene work. `earth.usda` and the layer/sensor USDs are maintained as normal.

Acceptance: known-dead config list empty; README fully true; recordings play at the correct
speed; a new team member can go from clone to flying without tribal knowledge. Size: large,
incremental. Owner: Kiro + Ofer.

### M8 — Dead-code and dead-config sweep (v2 exit gate)

The backstop for D23, not the method — the habit is to delete on the way past. This is the
final check before v2 is called done.

- Every config field is read by something (`test_no_dead_keys.py` known-dead list is **empty**).
- No module, class, function or ROS message definition without a consumer. Candidates already
  known: `sidecar/rtp.py` (M4), `Gimbal.msg` / `SATOutput.msg` (never vendored, M1),
  `DEFAULT_RTP_*` ports.
- No control-plane `Method` enum member without a registered handler *and* a caller (the
  coverage guard already checks registration; extend it to flag unused enum members).
- No `NotImplementedError` left that the README implies works.
- Consider a coverage run to surface never-executed code paths as a starting list.

Acceptance: the sweep finds nothing, and the guards that keep it that way are in the suite.
Size: small (if the habit held). Owner: Kiro.

## Version 3 — new capability (later)

Genuinely new ground, not parity. Not planned in detail yet; candidates raised so far:

- Waypoint missions — loiter/hold, takeoff/land/RTL, a mission file format and a
  control-plane command. The `vehicle` generators already exist.
- Velocity/acceleration-limited motion — `MotionLimits` is coded and tested but nothing
  enforces it at runtime (today the sim injects poses, it does not simulate dynamics).
- Target tracking / follow-me — pure vehicle logic, no new nodes needed.
- Physics-based flight instead of pose injection.
- Multi-sensor payloads (IR alongside EO) — the camera dict already supports it, M6 unblocks.

---

## Ofer's task list for Version 2

The things Kiro cannot do. Ordered by the milestone that needs them, so nothing blocks late.

### USD authoring (D15 — Kiro never writes `.usda`; Kiro specifies exactly what to build)

| # | Task | Milestone | Blocks |
|---|---|---|---|
| 1 | **Distance sensor layer** — `usd/layers/distance_sensor/`. Kiro will first evaluate Isaac 6's native physics raycast sensor and hand you a precise build sheet | M2 | Range publishing |
| 2 | **Bbox publisher layer** — `usd/layers/bbox_publisher/`, with a Cesium Globe Anchor per target object under `/World/bboxes` (this is where Globe Anchors genuinely belong) | M2 | Bbox publishing |
| 3 | **Remove the gimbal ROS subscriber** from `camera_udp.usda` and `camera_ros.usda` — the `ros2_subscriber` node wired to the gimbal offsets. Removing it also frees those inputs so config/commands can write them | M3 | Gimbal config + slew |
| 4 | **Add the native RTSP node** to the image-publisher graph in both camera layers, once Kiro has identified the right node and settings | M4 | Streaming |

### Eyes-on validation (needs a display and your judgement)

| # | Task | Milestone |
|---|---|---|
| 5 | Distance sensor: confirm the beam/range behaves sensibly against terrain | M2 |
| 6 | Bbox: confirm boxes track their objects on screen and drop out when occluded/off-frame | M2 |
| 7 | Gimbal: confirm a commanded angle slews smoothly at the configured rate rather than snapping | M3 |
| 8 | RTSP: play the stream off-box (VLC/ffplay) and confirm it is the camera view | M4 |
| 9 | Swarm: confirm two vehicles fly independently in one scene | M6 |

### Environment / decisions

| # | Task | Milestone |
|---|---|---|
| 10 | Cesium tileset server URL(s) for production use, and the base-URL form you want for multi-tileset | M7 |
| 11 | README images: `docs/images/sim_terrain.png`, `architecture.png`, `pose_sender.png` | M7 |
| 12 | Decide whether to revive the Docker workflow (dropped in D10) | M7 |

Already done, thank you: eyes-on non-headless flight validation and the full pose-sender GUI
walkthrough — both confirmed correct, and both closed out v1.

## Config keys that are declared but not applied

Found by auditing all 65 schema fields against every use in `src/`, `extensions/` and
`scripts/`. These parse and validate, so setting them looks like it works, but nothing reads
them. Two were reported by Ofer as "not working", which is how the audit started.

| Key | Status |
|---|---|
| `logging.isaac_logs` | Fixed — filters Isaac's Python loggers and suppresses Kit's stdout stream (~3400 log lines down to ~77) |
| `cesium.tileset_server_url` | Fixed — repoints tilesets at launch (was also shadowed by a prim_override, which now warns) |
| `assets.hdri` | Fixed — creates a `UsdLux.DomeLight` from the image path at runtime |
| `cesium.delete_cache_on_launch` | Fixed — deletes `~/.cache/ov/cesium-request-cache.sqlite*` before the scene opens |
| `vehicles.*.cameras.*.fov_deg` | Fixed — computes and binds `horizontalAperture` / `verticalAperture` |
| `vehicles.*.cameras.*.image_topic` | Fixed — the resolver now honours an explicit value instead of always deriving |
| `cesium.tilesets_root` | Fixed — default was `/tilesets`; the scene prim is at `/World/tilesets`, so `tileset_server_url` silently did nothing. Also corrected `BBOXES_ROOT`. |
| `assets.hdri` (second pass) | Fixed — now repoints the scene's existing `DomeLight` instead of creating a second one |
| `cesium.delete_cache_on_launch` (second pass) | Fixed — moved to before the app starts, so deleting no longer races Cesium's open handle (was causing intermittent `disk I/O error`) |
| `sim.scene` relative path | Fixed — a path like `./usd/scenes/x.usda` now resolves against the working directory |
| `ros2.domain_id` | Fixed — `None` inherits `$ROS_DOMAIN_ID`; an explicit value is exported before the bridge starts |
| `vehicles.*.cameras.*.publish_rate_hz` | Removed — was dead; re-add with a real implementation (see After version 1) |
| `vehicles.*.cameras.*.raw_topic` | Removed — dead, no raw-image node exists |
| `sim.stage_units_in_meters` | Dead |
| `vehicles.*.gimbal.start_roll_deg` / `start_pitch_deg` / `start_yaw_deg` | Dead. The USD wires the node's offset inputs to a ROS subscriber, so a config value cannot reach a connected attribute. Needs a design decision, not a binding. |
| `vehicles.*.gimbal.max_rate_deg_s` | Dead — no slew limiting is applied |
| `vehicles.*.primary_camera` | Dead |
| `ros2.use_sim_time` | Dead |

`test_no_dead_keys.py` guards against new dead keys and against leaving a fixed key on the
known-dead list, so this table cannot silently rot.

## Known issues

| Issue | Status | Detail |
|---|---|---|
| Intermittent startup segfault | Fixed | Frame warm-up (60 frames) after enabling extensions, before any stage operation. Measured 0 crashes in 27 runs vs 5 in 10 before. Tests lock in the ordering. |
| Stale `/tmp/carb.*` directories | Documented | Previous crashes leave debris that feeds further crashes. `rm -rf /tmp/carb.*` with no Isaac running helps. |
| Stale OGN generated databases | Mitigated | `link_extensions.sh` clears `~/.cache/ov/ogn_generated/*/isaac_core_ogn.*`. Deleting or renaming an OGN node still requires a manual clear before the new type registers. |
| `Could not import rclpy` warning | Harmless | Isaac's C++ bridge logs this on startup. The bridge publishes fine without Python rclpy. |
| Monotonic frame id unreachable | Deferred | `ROS2CameraHelper` bakes `frameId` once at init. `header.stamp` covers most use cases. |
| Cesium WAL file growth | Documented | Long sessions grow `~/.cache/ov/cesium-request-cache.sqlite-wal` to hundreds of GB. `delete_cache_on_launch = true` in config mitigates. |
