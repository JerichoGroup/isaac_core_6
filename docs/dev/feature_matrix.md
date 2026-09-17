# Feature matrix

Every capability this repo claims, how it is verified, and where the evidence lives. This is the
document the release gate is measured against, and it is **re-run in Phase 5** after the cleanup —
a matrix verified only before the cleanup certifies code that no longer exists.

Buckets:

- **A — automated.** A test asserts the behaviour with no Isaac Sim, GPU or ROS 2. Runs in CI.
- **B — live-scripted.** Needs a running simulator; verified by a script that asserts numbers.
- **C — eyes-on.** Needs a human to look at a picture.

Baseline at the time of writing: **1558 tests across 78 files**, all seven pre-commit gates green,
23 import-linter contracts kept.

Status values are `pass` (verified at the date shown), `todo` (bucket assigned, not yet run this
cycle), or `finding` (see the findings section).

---

## 1. Control plane

15 `Method` enum members, 15 registered handlers. Every handler is reachable from a shipped caller
and every enum member has a handler — enforced by `control/test_method_coverage.py`.

| Capability | Bucket | Evidence | Status |
|---|---|---|---|
| `ping` — readiness probe | A | `control/test_client.py`, `control/test_server.py` | pass |
| `get_state` — lifecycle + `ready` flag | A | `sim/test_runtime_handlers.py` | pass |
| `get_capabilities` — enabled/skipped layers | A | `sim/test_runtime_handlers.py` | pass |
| `get_config` / `set_config` — runtime patch | A | `sim/test_config_patch.py` | pass |
| `get_pose` — live prim transform | A + B | `sim/test_runtime_handlers.py`; live via inspector | pass |
| `set_pose` — inject a pose | A + B | `sim/test_set_pose.py` | pass |
| `set_gimbal` — slew-limited offsets | A + B | `sim/test_gimbal_control.py`, `geo/test_gimbal_axes.py` | pass |
| `capture_frame` at the camera's resolution | A + B | `tests/system/test_capture.py` | pass |
| `capture_frame` at a requested resolution | A + B | `tests/system/test_capture.py` asserts PNG pixels | pass |
| `pause` / `resume` — main-thread dispatch | A | `sim/test_main_thread_dispatch.py` | pass |
| `step(count)` — advance N frames while paused | A | `sim/test_runtime_handlers.py` | pass |
| `reset` — stop, restore, deferred replay | A | `sim/test_runtime_handlers.py` | pass |
| `get_runtime_values` — internal, for inspect | A | `control/test_method_coverage.py`, `debug/test_inspector.py` | pass |
| `read_prim_attribute` — internal, for inspect | A | `debug/test_inspector.py` | **finding F1** |

## 2. Devkit

14 public members on `SimSession`. `devkit/test_control_method_coverage.py` asserts the devkit and
the control plane agree.

| Capability | Bucket | Evidence | Status |
|---|---|---|---|
| `Sim.attach(host, port)` — connect to a running sim | A | `devkit/test_session.py` | pass |
| `Sim.launch(...)` — spawn, own, and stop the process | A + B | `devkit/test_session.py` | pass |
| Process-group kill on close (no orphans) | A | `devkit/test_session.py` | pass |
| `wait_until_ready` — waits on composition, not the port | A | `devkit/test_session.py` | pass |
| Context-manager cleanup on `BaseException` | A | `devkit/test_session.py` | pass |
| `UdpPoseTransport` + `pace` — send trajectories | A | `devkit/test_transport.py` | pass |
| Recording to disk | A | `devkit/test_recording.py`, `test_recording_serialisers.py` | pass |
| MAVLink host-side bridge | A | `devkit/test_mavlink.py` | pass |
| 60 s default call timeout | A | `devkit/test_session.py` | pass |

## 3. Pose input

| Capability | Bucket | Evidence | Status |
|---|---|---|---|
| 51-byte UDP packet, byte-compatible with 2023 | A | `protocol/test_pose_packet.py`, `contracts/test_packet.py` | pass |
| Freeze-on-bad-data (hold last good pose) | A | `protocol/test_pose_packet.py` | pass |
| Checksum + header validation | A | `contracts/test_packet.py` | pass |
| ROS 2 pose input via MAVROS topics | A + B | `sim/test_configurator.py` | pass |
| Per-vehicle UDP port allocation (`base + index`) | A | `sim/test_configurator.py` | pass |

## 4. Geodesy and conventions

| Capability | Bucket | Evidence | Status |
|---|---|---|---|
| LLA ↔ ECEF ↔ ENU | A | `geo/test_enu.py` | pass |
| NED→ENU with correct axis semantics | A | `geo/test_camera_axes.py` (asserts look direction) | pass |
| Gimbal composition onto a **non-level** airframe | A | `geo/test_gimbal_axes.py`, `geo/test_gimbal_on_attitude.py` | pass |
| Rotation frames (world/body), **two and three axes at once** | A | `geo/test_rotation_frames.py`, `test_rotation_pipeline.py`, `vehicle/test_multi_axis_rotation.py` | pass |
| Saturating rangefinder limits | A | `contracts/test_rangefinder.py` | pass |
| Georeference unification + mismatch guard | A | `sim/test_georeference.py` | pass |
| Unit-suffix naming discipline (`_deg`/`_r`/`_m`) | A | `contracts/test_naming.py` | pass |

## 5. Outputs

| Capability | Bucket | Evidence | Status |
|---|---|---|---|
| `/isaac_core/global_pose` (real quaternion) | A + B | `sim/test_shipped_manifests.py` | pass |
| `/isaac_core/image_rgb` | A + B | `sim/test_shipped_manifests.py` | pass |
| `/isaac_core/distance_sensor` | A + B | `sim/test_shipped_manifests.py` | pass |
| `/isaac_core/bbox` (`FrameBboxes`, parallel arrays) | A + B | `extensions/test_ogn_contracts.py` | pass |
| Native RTSP H.264 per camera | B | live `DESCRIBE` → 200 OK, SDP `H264/90000` | pass |
| Advancing `header.stamp` | A | `contracts/test_stamp.py` | pass |
| Topic namespacing only when >1 vehicle | A | `sim/test_configurator.py` | pass |

## 6. Layers and composition

Four shipped layers: `camera_udp`, `camera_ros`, `distance_sensor`, `bbox`.

| Capability | Bucket | Evidence | Status |
|---|---|---|---|
| Manifest-driven binding of config → prim attribute | A | `sim/test_manifest.py`, `test_configurator.py` | pass |
| Every `resolve = "..."` in every shipped manifest exists | A | `sim/test_manifest_resolvers.py` | pass |
| Binding prims exist in the USD | A | `test_usd_layers.py`, `test_usd_graph_wiring.py` | pass |
| Dependency-ordered planning (fixed point, D25) | A | `sim/test_planner.py` | pass |
| Third-party layers via `layer_search_paths` | A | `sim/test_discovery.py` | pass |
| Entry-point layers from another package | — | not implemented; see `docs/dev/roadmap.md` | **not built** |
| Multi-vehicle: per-vehicle mount, port, topics | A | `sim/test_configurator.py` | pass |
| Single-vehicle output unchanged by swarm support | A | `sim/test_configurator.py` | pass |
| Offscreen render products per camera | B | live: both image topics publish | pass |
| Semantics applied to bbox targets | A | `sim/test_composer_assets.py` | pass |

## 7. CLI and tools

Four console scripts.

| Capability | Bucket | Evidence | Status |
|---|---|---|---|
| `isaac-core run` | A + B | `cli/test_main.py` | pass |
| `isaac-core doctor` (probes for the install, reads no environment variable) | A | `cli/test_doctor.py`, `test_install_resolution.py`, `test_no_isaac_env_var.py` | pass |
| `isaac-core config dump` / `explain` | A | `cli/test_config_cmd.py` | pass |
| Shell completion, generated from the schema | A | `cli/test_completion.py` | pass |
| Shell completion, installed where a running shell finds it | A | `cli/test_completion_install.py` drives the installed script in a real bash | pass |
| `isaac-core-inspect` (live prim read) | A + B | `debug/test_inspector.py` | pass |
| `isaac-core-pose-sender` — tabs, per-tab pose source, readback | A + C | `debug/test_pose_sender_gui.py` | pass |
| `isaac-core-mavlink` | A | `devkit/test_mavlink.py` | pass |

## 8. Configuration

11 top-level sections: `sim`, `assets`, `geo`, `cesium`, `features`, `vehicles`, `layers`, `ros2`,
`sidecar`, `logging`, `prim_overrides`.

| Capability | Bucket | Evidence | Status |
|---|---|---|---|
| Layered precedence (defaults → file → env → `--set` → patch) | A | `config/test_loader.py`, `test_sources.py` | pass |
| Frozen models | A | `config/test_schema.py` | pass |
| No dead config keys | A | `config/test_no_dead_keys.py` | pass |
| `default.toml` documents every key | A | `config/test_default_toml.py` | pass |
| Renderer validation + `MinimalRendering` warning | A | `config/test_renderer.py` | pass |
| Cesium tile tunables | A | `sim/test_tile_tuning.py` | pass |
| Camera intrinsics | A | `sim/test_configurator.py` | pass |

## 9. Architecture guarantees

| Guarantee | Bucket | Evidence | Status |
|---|---|---|---|
| Kernel imports no Isaac/ROS/GStreamer | A | `test_kernel_purity.py` + 23 import-linter contracts | pass |
| No dead code, config keys, or unreachable handlers | A | `test_no_dead_code.py` | pass |
| No deprecated Isaac API use | A | `test_no_dead_isaac_api.py` | pass |
| No Python `rclpy` inside extensions | A | `extensions/test_no_python_ros2.py` | pass |
| Extensions package correctly | A | `extensions/test_extension_packaging.py` | pass |
| Distribution packaging | A | `test_packaging.py` | pass |
| Startup warm-up frames preserved | A | `sim/test_startup_warmup.py` | pass |

## 10. Eyes-on scenarios (bucket C)

Driven by `scripts/eyes_on_check.py`. Results are recorded per release, not remembered.

| # | Scenario | Last result |
|---|---|---|
| 1 | Single vehicle, UDP pose over terrain | pass |
| 2 | ROS 2 pose input | re-run needed |
| 3 | Distance sensor | re-run needed |
| 4 | Gimbal | pass |
| 5 | Gimbal axis check (`pitch -40` reads as pitch) | pass, worth a glance |
| 6 | Two vehicles (swarm) | pass (2026-09-08) |
| 7 | Bounding boxes | re-run needed |
| 8 | Frame capture | pass |

---

## Findings

Phase 0 exists to surface these. None is a crash; all are honesty or tidiness problems.

### F1 — two control methods bypass the `Method` enum

`get_runtime_values` and `read_prim_attribute` are registered handlers called by
`isaac-core-inspect` with raw strings. They are genuinely reachable, so
`control/test_method_coverage.py` passes correctly — but the `Method` enum is described as the
control-plane surface and these two are not in it.

`get_runtime_values` is at least declared in the guard's `_INTERNAL_HANDLERS_OK` allowlist with the
tool that reaches it. `read_prim_attribute` is not documented anywhere.

**Action (Phase 2):** either add both to the enum, or document both as internal in one place. I lean
toward adding them to the enum: they are shipped capabilities reachable over the wire, and a user who
reads the enum to learn what the sim can do currently gets an incomplete answer.

### F2 — four dead topic constants

In `contracts/topics.py`, with zero references anywhere in `src` or `tests`:

- `SAT` — for the SAT capture that was never implemented.
- `RAW_RGB` — no consumer.
- `MAVROS_LLA`, `MAVROS_ORIENTATION` — superseded by `_resolve_mavros_topic`, which builds these
  topic names dynamically per vehicle. The MAVROS input path itself **works**; only the constants
  are dead.

**Action (Phase 2):** delete all four, and extend the dead-symbol guard to cover module-level string
constants in `contracts/`, which it does not currently reach.

### F3 — the `test_no_dead_code` README guard is now vacuous

It checks that any capability raising `NotImplementedError` is framed in the README as not working.
With the deferred methods removed there are none, so the check passes trivially. Correct, but it now
guards nothing. Keep it — it costs nothing and becomes live again the moment someone defers a
capability.

### F4 — bucket B has no runner — **CLOSED**

`tests/system/` runs a real Isaac Sim under pytest (`--system`), and `eyes_on_check.py --verify`
remains for interactive use. Five of the eight
scenarios are automated: pose tracking, gimbal, capture, swarm and lifecycle. It found F6 on its first
run. The remaining three need ROS 2 subscribers or human eyes.

Originally: most B rows were marked pass from ad-hoc live runs recorded in the development log, not
from a repeatable script. That is exactly the weakness that let me report the swarm image topics as broken
when they were fine.

**Action (Phase 3):** `eyes_on_check.py --verify` that asserts numbers and exits non-zero, so bucket
B becomes reproducible rather than a matter of memory.

### F7 — the swarm system-test simulator is flaky to launch

Roughly one run in three cannot bring the two-vehicle simulator to a composed, ready stage, and those
tests **skip** rather than fail. Mitigated by running the heaviest module first, retrying a launch
twice, and bounding every wait so a bad run is slow rather than hung.

Not a product defect: standalone runs measure exactly 600.00 m of commanded separation, and eyes-on
scenario 6 passes. It is the harness contending with Isaac's own startup instability -- the same
roughly-one-in-three segfault documented in the README.

### F5 — one unreproduced gimbal report

One run in four appeared to apply offsets only at the end, with pitch never moving. Not reproduced
since the axis fix. If it recurs, capture whether `gimbal target set to roll=… pitch=… yaw=…` appears
in the log for each command.

---

### F8 — rows certified the implementation, not the thing a user touches — **AUDITED 2026-09-17**

Six bugs shipped past a green suite because each test verified the code as built rather than the
surface a user meets. Re-auditing every row against that pattern found:

- **`Entry-point layers from another package` claimed pass and is not implemented.** The cited test
  never mentions entry points and no `importlib.metadata` lookup exists. Row corrected to **not
  built**; the roadmap already listed it as a gap, so the matrix was the only thing claiming otherwise.
- **`Shell completion` cited the generator test only.** Generating a correct script and installing it
  where a shell will load it are different claims; the installed path now has its own row.
- **Gimbal composition was only ever tested on a level airframe** — the helper hardcodes
  `(0, 0, 0)`. Measured with a non-level airframe the composition is correct (pitched -30 plus gimbal
  -30 gives elevation -60; rolled 45 gives the bolted `sin30·cos45` tilt), so this was missing
  coverage rather than a bug. Pinned now.
- **`13 Method enum members` was wrong**; there are 15. A test now checks every count in this document
  against the code, because a hand-maintained number in a certifying document always rots.

Rows verified as genuinely strong, for contrast: the 51-byte packet is anchored to a hex literal
captured from the 2023 encoder rather than round-tripped through our own; `isaac-core-inspect` drives a
real `ControlClient` against a real `ControlServer` on an ephemeral port; freeze-on-bad-data covers
seven distinct malformed inputs; `slew_towards` limits each axis independently along the shortest
angular path, so the gimbal has none of the mid-path error that SLERP introduced elsewhere.

**The rule this produces.** A row may only cite evidence that exercises the same entry point a user
does. Testing a generator, a helper, or a default-shaped case is worth doing, but it does not certify
the capability — and a row that says "pass" while the roadmap says "gap" means the matrix is wrong.

## What this matrix does **not** cover

Stated so the gaps are deliberate rather than accidental:

- **Sustained-load behaviour.** Nothing here runs for hours. Cesium cache growth is documented in
  the README as a known operational issue.
- **Crash rate.** The ~1-in-3 startup segfault is Kit-internal and reproduces with our extensions
  disabled. It is worked around by warm-up frames and is not represented as a row.
- **Performance.** Tile-streaming frame dips are documented but not asserted; the one experiment
  that appeared to show a tuning win reversed when the test order was reversed.
- **Two-machine and fresh-install behaviour.** Deliberately deferred to the Phase 5 fresh-clone
  check on a different machine.
