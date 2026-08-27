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
- Camera-key templating in manifests — the shipped `camera_udp` / `camera_ros` manifests hardcode the camera key `eo` in their bindings (`config = "vehicles.{instance}.cameras.eo.width"`), while the vehicle is templated as `{instance}`. So renaming a camera (e.g. `eo` -> `rgb`) or giving a vehicle a differently-named camera needs the two `layer.toml` files edited, not just config, or composition fails with `ConfigKeyError: ...cameras.eo... does not exist`. Add a `{camera}` placeholder to the binding resolver (parallel to `{instance}`) so camera keys stop being hardcoded. This also unblocks multiple differently-named cameras per vehicle cleanly.
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
- Cesium base-URL and multi-tileset — `cesium.tileset_server_url` currently replaces the whole `cesium:url`. The team will serve several tilesets from one host (`<ip>:<port>/city1/tileset.json`, `/city2/...`), so the config should carry only the base `http://<ip>:<port>` and each tileset's path should be preserved: read the existing `cesium:url` per prim and swap only the scheme+host+port. Also support per-tileset selection rather than one URL for all.
- Unify the vehicle mount — `config.vehicles.<id>.mount` sets the vehicle mount used for camera-prim resolution and `get_pose`, but layers actually mount at the manifest's own `mount` template (`/World/Environment/{instance}`). By default these agree; overriding `mount` makes them diverge and the camera prim resolves to a path no layer created. Manifest mount and vehicle mount are different axes (a sensor layer may mount elsewhere on purpose), so unifying them needs a `{vehicle_mount}` placeholder in the manifest resolver, not a blind override. Until then, treat `mount` as fixed.
- Two georeferences — changing `geo.enu_reference` moves the PoseSync ENU origin but not the scene's authored `/CesiumGeoreference` prim, because Cesium listens to USD notices and the tooling does not author USD (decisions D18/D19). The compositor already warns when the two disagree; a cleaner answer would derive one from the other or drive the Cesium georeference at runtime through its own API.
- Camera intrinsics as first-class config — support `focus_distance`, and expose `horizontalAperture` / `verticalAperture` directly, alongside `fov_deg` / `focal_length_mm`, so common camera tuning does not need a `prim_override`. Re-add `publish_rate_hz` here with a real implementation (throttling the camera helper / render product rate).
- Fully silent boot — with `isaac_logs=false` the Kit log stream is suppressed (~3400 lines to ~77), but the `isaacsim` launcher and Warp still print a boot banner and a GPU capability table to stdout via `print()` before Kit initialises, outside any logging control. Suppressing those cleanly means redirecting stdout around `SimulationApp()` construction without hiding genuine startup errors.
- MinimalRendering black screen — `renderer = "MinimalRendering"` produces a black viewport with no terrain (Ofer observed). Either document it as unsupported for this Cesium-terrain use case or validate/warn when it is selected.
- `output_root` and frame capture — `sim.control_plane.output_root` is only consumed by `capture_frame`, which raises `NotImplementedError`. Nothing writes there yet. Implement viewport capture (see the existing capture item) and this becomes live.
- Viewport config section — `sim.viewport.primary_camera` was removed (dead, and confusingly overlapping `sim.viewport_camera`). The good idea behind it survives: let the user pick the viewport camera by a logical key (`drone_0.eo`) rather than a raw prim path, and grow a `[sim.viewport]` section for multi-viewport / resolution / overlay settings. When built, it should supersede the raw `viewport_camera` prim path.
- Runtime control-plane methods — the devkit exposes `reset()`, `features.enable/disable(...)` and `config.patch(...)`. Their handlers are now registered but raise a clear "not implemented yet" error (matching `capture_frame`). Implement them: `reset` (timeline/pose/stage reset semantics TBD), runtime feature toggling (compose/decompose a layer on the live stage), and config patching (define the safely-mutable subset first). `set_pose` and `load_scene` are defined in the `Method` enum but not yet exposed or implemented.
- Recording serializers — `isaac_core.devkit.recording` factory functions (`video_recorder`, `pose_recorder`, `range_recorder`, `bbox_recorder`) have no unit tests for their message-field serialisation, so a ROS message field rename would go uncaught. Add round-trip tests with fake messages.
- Cesium tile-streaming hiccup — during a moving flight the viewport hitches briefly every
  few seconds. The pose pipeline is ruled out: sampling the live prim transform during an
  orbit showed a smooth stream (119 samples, 0 outliers >3x mean, 0 frozen). The stall is
  render-side, consistent with Cesium 3D Tiles loading/unloading as new terrain enters view.
  Investigate Cesium tuning (cache size / max simultaneous tile loads / pre-warming), and
  optionally offer client-side pose smoothing (the `slerp`/interpolation already exists) so a
  recovered frame eases in rather than snapping to the newest pose.
- Shell tab-completion for the CLI — `isaac-<TAB>` should complete to `isaac-core`, and `isaac-core r<TAB>` to `isaac-core run`, likewise for `doctor`/`config` and the `-inspect` / `-pose-sender` entry points. Ship completion scripts for bash and zsh (argcomplete or hand-written), and document how to source them in `scripts/setup.sh`. Quality-of-life, not functional.

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
