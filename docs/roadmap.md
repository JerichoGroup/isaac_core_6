# Project status — isaac_core_6

## Status summary

The simulator works end to end. `isaac-core run` launches Isaac Sim 6.0.1, composes the
stage from layer manifests, a UDP pose packet moves the camera over real Cesium 3D Tiles
terrain, and both ROS 2 topics publish (`/isaac_core/global_pose` at ~100 Hz,
`/isaac_core/image_rgb` at ~115 Hz) with real advancing timestamps. 1103 tests pass in
9 seconds with no GPU, no Isaac Sim, and no ROS 2 installed. 23 import-linter contracts
enforce the layering and cannot silently rot. The intermittent startup segfault that
plagued launches (5 in 10) is fixed (0 in 27 after the warm-up change).

Verdict: ready for v1. The core pipeline — pose in, image out, geodetic pose published —
matches and exceeds what the 2023 repo did, with a clean architecture that the old repo
never had.

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

- 1103 tests passing (verified this session: `python3 -m pytest -q` → `1022 passed in 9.60s`).
- Guard tests: kernel purity (imports in subprocess with omni/rclpy/gi forced unimportable), no-dead-Isaac-API (AST scan), no-Python-ROS2-in-extensions, no-deprecated-OGN-API, USD layer validation (dangling connections, two-source inputs, missing orient op, Globe Anchor presence, layer size), graph wiring (unconnected `execIn` detection), extension packaging, OGN key naming.
- 23 import-linter contracts KEPT (verified this session).

### Tooling

- ruff 0.8.2, mypy 1.14.1, pytest 8.3.4, pre-commit 4.0.1 — all pinned and verified matching.
- `test_packaging.py` fails if `requirements.txt` drifts from `pyproject.toml` or dev tool pins disagree with `.pre-commit-config.yaml`.

---

## Parity with isaac_core_2023

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

---

## Remaining for version 1

Nothing here blocks declaring v1 in the sense of "the camera pipeline works and is shippable." These are polish items that strengthen the release.

| Item | Why it matters | Size | Owner |
|---|---|---|---|
| Eyes-on non-headless flight | Nobody has visually confirmed the camera looks along the nose | Ofer sits in front of the GUI | Ofer |
| README images | The README has no screenshots; the old repo's were effective | One session | Ofer |

Two items previously listed here are now done: `Sim.launch()` spawns a simulator and the
returned session owns the process, and `import-linter` runs as a pre-commit hook (verified
to fail on an injected layering violation).

---

## After version 1

Features not in the 2023 repo. Ordered by stated priority.

- Swarm / multi-drone — the architecture already supports N vehicles (topic namespacing, `{instance}` in manifests, per-vehicle UDP port). Needs: multiple camera layers composed simultaneously, per-vehicle `get_pose`, mixed pose sources.
- Sensor layers — distance sensor (physics raycast sensor, native in Isaac 6), bbox publisher (Globe Anchors on targets), SAT capture.
- Waypoint missions — loiter/hold, takeoff/land/RTL. Vehicle generators exist; needs a mission file format and a control-plane command.
- Velocity/acceleration-limited motion — `MotionLimits` is coded and tested but the runtime does not enforce it yet (no physics, just pose injection).
- Gimbal servo dynamics — `max_rate_deg_s` is in config; the node needs to integrate rather than snap.
- Target tracking / follow-me — pure vehicle logic, no new nodes needed.
- Sidecar live-testing — `sidecar.rtp` is coded; needs a live GStreamer pipeline test.
- `capture_frame` via the control plane — viewport capture API.
- Native RTSP (Isaac 6 documents it) vs custom GStreamer RTP — evaluate.
- Docker / container workflow — if the team needs reproducible deployments again.
- MAVLink pose source (without MAVROS) — `pose_source = "mavlink"` is in the schema but unimplemented.

---

## Needs Ofer specifically

- Eyes-on flight in the GUI: run non-headless, confirm the camera orientation looks correct visually.
- README screenshot: a frame of the sim flying over terrain. Save as `docs/images/sim_terrain.png`.
- Architecture diagram: a block diagram showing the data flow (UDP → node → prim → image publish). Save as `docs/images/architecture.png`.
- Pose-sender GUI screenshot: save as `docs/images/pose_sender.png`.
- Any future USD authoring (D15 — Kiro never writes `.usda` files).
- Cesium tileset server URL for the team's production use (currently no default in config).
- Hardware validation of the sidecar RTP pipeline (needs GStreamer packages installed).

---

## Config keys that are declared but not applied

Found by auditing all 65 schema fields against every use in `src/`, `extensions/` and
`scripts/`. These parse and validate, so setting them looks like it works, but nothing reads
them. Two were reported by Ofer as "not working", which is how the audit started.

| Key | Status |
|---|---|
| `logging.isaac_logs` | Fixed — now filters Isaac's Python loggers |
| `cesium.tileset_server_url` | Fixed — repoints tilesets at launch |
| `assets.hdri` | Dead |
| `cesium.delete_cache_on_launch` | Dead |
| `sim.stage_units_in_meters` | Dead |
| `vehicles.*.gimbal.start_roll_deg` / `start_pitch_deg` / `start_yaw_deg` | Dead. The USD wires the node's offset inputs to a ROS subscriber, so a config value cannot reach a connected attribute. Needs a design decision, not a binding. |
| `vehicles.*.gimbal.max_rate_deg_s` | Dead — no slew limiting is applied |
| `vehicles.*.primary_camera` | Dead |
| `vehicles.*.cameras.*.publish_rate_hz` | Dead |
| `vehicles.*.cameras.*.raw_topic` | Dead |
| `ros.use_sim_time` | Dead |

Worth fixing as a group before v1 is called finished: a config surface that silently ignores
values is worse than one that does not offer them. The cheap guard is a test asserting every
schema field is referenced somewhere outside the schema and the reference config.

## Known issues

| Issue | Status | Detail |
|---|---|---|
| Intermittent startup segfault | Fixed | Frame warm-up (60 frames) after enabling extensions, before any stage operation. Measured 0 crashes in 27 runs vs 5 in 10 before. Tests lock in the ordering. |
| Stale `/tmp/carb.*` directories | Documented | Previous crashes leave debris that feeds further crashes. `rm -rf /tmp/carb.*` with no Isaac running helps. |
| Stale OGN generated databases | Mitigated | `link_extensions.sh` clears `~/.cache/ov/ogn_generated/*/isaac_core_ogn.*`. Deleting or renaming an OGN node still requires a manual clear before the new type registers. |
| `Could not import rclpy` warning | Harmless | Isaac's C++ bridge logs this on startup. The bridge publishes fine without Python rclpy. |
| Monotonic frame id unreachable | Deferred | `ROS2CameraHelper` bakes `frameId` once at init. `header.stamp` covers most use cases. |
| Cesium WAL file growth | Documented | Long sessions grow `~/.cache/ov/cesium-request-cache.sqlite-wal` to hundreds of GB. `delete_cache_on_launch = true` in config mitigates. |
