# Phase 0.5 — review triage

Seven independent reviewers (architecture, tests, kernel correctness, Isaac surface, robustness,
fresh user, config) reviewed the repo with a mandate to be harsh. Every finding below is
dispositioned: **FIXED** (done, with the guard that stops it returning), **ACCEPTED** (queued to a
phase), **REJECTED** (with the reason), or **NEEDS OFER**.

I verified findings myself before accepting them. That mattered: the reviewers were largely right,
but several claims needed correcting, and one high-severity finding was a regression **I** introduced
last week.

State after this phase: **1571 tests** (up from 1558), all seven gates green, 23 contracts kept.

---

## FIXED in this phase

### 1. `capture_frame` at a requested resolution was silently broken — my regression

`runtime.py` looked up the OmniGraph node `isaac_get_viewport_render_product`. That node has **zero**
occurrences in the shipped camera layers: I replaced it with `isaac_create_render_product` during the
swarm viewport fix and never updated the runtime. `og.Controller.node()` raised, the `except
Exception` swallowed it, the lookup returned `None`, and capture fell back to viewport resolution —
reporting success. Headless capture would fail outright.

Verified by counting occurrences in both layers. The existing wiring test passed because the old name
only appeared inside an error-message string.

Fixed: the name is now the module constant `RENDER_PRODUCT_NODE_NAME`, and
`test_render_product_node_name_exists_in_every_camera_layer` asserts it against both shipped `.usda`
files. I confirmed the guard bites by reverting the constant to the broken name and watching it fail.

**This is the strongest argument for the whole phase.** It was invisible to 1558 passing tests, and I
had reported the manifest switch as verified.

### 2. A single UDP packet could crash the receive loop

`HoldLastGoodDecoder` only catches `PosePacketError`. A structurally valid packet — correct header,
correct checksum — carrying `lat = 200.0` or `NaN` reached `Lla.__post_init__`, which raises
`ValueError`. Not caught, so it propagated out of the receive loop. Remote-triggerable from any
sender on the wire.

Confirmed by building such packets and running the decoder: `CRASHED ValueError: lat_deg must be in
[-90.0, 90.0], got 200.0`. `alt = inf` decoded happily and propagated infinity into the stage — the
same class as the earlier `int(inf)` rangefinder crash.

Fixed: `decode` now validates all six fields for finiteness and lat/lon range, raising
`PacketPayloadError`, so the documented hold-last-good behaviour is true for **any** 51 bytes. All
five hostile variants now hold the last good pose and increment `failure_count`. Locked by 10
parametrized tests. The bounds are reused from `contracts/pose.py`, which now exports them, rather
than duplicated as literals.

### 3. `config/default.toml` silently changed behaviour if you used it

The file is documentation — it is never auto-loaded, so shipped defaults come from the schema. Three
values had drifted, and because the README tells users to start from this file, copying it changed
behaviour:

| Key | Schema | default.toml | Consequence |
|---|---|---|---|
| `cesium.tilesets_root` | `/World/tilesets` | `/tilesets` | `tileset_server_url` and every tile tunable silently no-op |
| `cesium.delete_cache_on_launch` | `false` | `true` | forces the worst-case tile hitching the README warns about |
| `ros2.domain_id` | `None` (inherit) | `13` | joins the wrong DDS domain; topics invisible to other nodes |

All three verified by loading both ways and diffing. Fixed, and
`test_default_toml_values_agree_with_schema_defaults` now compares every key, with an explicit
allowlist of two intentional differences. The old test checked only that the file was *valid*.

### 4. An entire documented config namespace did nothing

`config.layers` is read by **zero** code in `src`. Worse than dead — actively misleading:

- `[layers.distance_sensor] max_range_m = 180.0` contradicted the real key
  (`vehicles.<id>.distance_sensor.max_range_m`, default 5000), whose own docstring explains that a
  180 m ray never reaches the ground and the sensor then reports no detection forever while looking
  healthy. A user would edit the documented value and get nothing.
- `[layers.bbox_publisher]` used an id that is not the shipped layer's (`bbox`).
- `[layers.sat]` documented a layer that has never existed.

An existing test *required* these sections to be present, which is why they survived. Fixed: sections
removed, comment now points at the real per-vehicle keys, the namespace retained for user-authored
layers. That test now asserts shape rather than presence, and a new test rejects any `[layers.*]` id
that does not ship.

---

## ACCEPTED — queued, not yet done

### High priority (Phase 2, before verification)

| # | Finding | Why it matters |
|---|---|---|
| A1 | `set_gimbal` and `capture_frame` hardcode `next(iter(vehicles))` — 10 such sites | In a swarm you **cannot** aim `wing`'s gimbal or capture from its camera; the call succeeds and silently affects `lead`. The README sells multi-vehicle as first-class. Needs per-vehicle gimbal state, not just a parameter. |
| A2 | `vehicle.motion.move_to` accepts `limits` and ignores it | Its siblings honour limits. Confirmed: a 1 m/s limit produced the full-distance move. Either honour it or delete the parameter. |
| A3 | `PathTrajectory` interpolates longitude linearly | Crossing the antimeridian flies 358° the wrong way while duration comes from the correct short geodesic, so position and timing disagree. |
| A4 | `set_pose` does not range-check lat/lon | The config path validates; the live RPC path does not. `lat=9999` is accepted and packed. |
| A5 | Control server: unbounded per-connection buffer | A client streaming bytes with no newline grows memory without limit and OOMs the simulator, which is holding a GPU. |
| A6 | Control server dispatches inline on the selector thread | One slow `capture_frame` blocks every other client, including `ping`. Head-of-line stall. |
| A7 | Token compared with `!=` | Timing-observable on exactly the non-loopback path the token exists to protect. Use `hmac.compare_digest`. |
| A8 | `config.patch(**kwargs)` cannot express the wire contract | The server needs `params.key`/`params.value`; the only spelling that works is a coincidence of kwarg naming. Also, exactly one key is patchable, so the general-purpose facade oversells. |

### Test gaps (Phase 3) — the most valuable category

| # | Finding |
|---|---|
| T1 | **No OGN node `compute()` is ever executed.** All four runtime nodes are tested by parsing their `.ogn` JSON as data. The code that actually turns a pose into what the camera receives — zero-position guard, converter recreation, WXYZ→IJKR reorder, the full offset composition — is untested. |
| T2 | `test_gimbal_axes.py` **reimplements** the node's composition rather than calling it, and says so in its own docstring. Node and test can drift apart while both pass. |
| T3 | `test_orbit_yaw_is_tangent_to_path` asserts only `isfinite(yaw)` — it would pass with yaw hardcoded to 0. |
| T4 | One golden wire vector, at a pose symmetric enough that a lat/lon or roll/pitch transposition could survive. Needs 2–3 vectors with all-distinct fields. |
| T5 | `set_gimbal` target accumulation and slew completion are untested; the matrix credited `test_gimbal_control.py`, which only checks bindings and registration. |
| T6 | Nothing asserts the `.ogn` `enu_reference` default equals the config default. Drift misplaces the aircraft when the input is unconnected. |
| T7 | Bbox parallel-array index alignment is tested at the serialiser, not at the node that produces it — the exact invariant the design exists to protect. |

### Documentation (Phase 1 — these directly shape the rewrite)

| # | Finding |
|---|---|
| D1 | `docs/first_run.md` opens with "There is no `isaac-core run` yet" and walks through building the stage by hand. The README links it as *the* first-flight guide. A newcomer concludes the tool is half-built. |
| D2 | Test count appears as 1103 (README, twice), 1279 (roadmap), 1558 (matrix). Actual was 1558, now 1571. State it once or not at all. |
| D3 | `capture_frame` is documented as working **and** listed under "what does not work yet". |
| D4 | Install step 3 appears three ways: `-e .` (README), `-e ".[sim]"` (first_run + doctor's own hint). README says four steps; `setup.sh` prints five. |
| D5 | README claims `default.toml` is "the complete surface — every value appears there", but `rtsp_port`/`rtsp_mount_path` are real schema fields absent from it. |
| D6 | `docs/ros2_and_python.md` says "the whole `isaac_core_ogn.sensors` extension is gone". It exists, ships live nodes, and is enabled by default. |
| D7 | `usd_build_sheet.md` is linked as user documentation but is a task list between us, including the screenshot shot-list and bare decision refs (D14, D21, D25) never defined for a newcomer. |
| D8 | Nothing tells a user where to get Cesium tiles. Without a tile server there is no terrain, and the docs never say how to get one. |
| D9 | `Lla`'s docstring says altitude is above the WGS84 ellipsoid; the wire spec, `set_pose` and the MAVLink bridge all mean mean-sea-level. Tens of metres apart, no error. |
| D10 | Default config enables `distance_sensor`, which the first-flight walkthrough never mentions and "what does not work yet" calls unfinished. |

### Lower priority (Phase 2/4)

- Pole handling: `meters_to_latlon_offset` has no `1/cos(lat)` guard — at lat 90 it returns 1.47e13
  degrees rather than erroring. Orbits near the pole produce lat > 90 and then crash `Lla`.
- `normalize_angle` propagates NaN/inf silently while `rangefinder` and `stamp` guard finiteness —
  inconsistent hardening.
- `capabilities.probe()` checks 2023-era prim paths (`/Environment/main_camera`, `/semantics`) that
  no longer exist.
- Capture resizes the render product and restores it only on the success path; an error leaves the
  ROS/RTSP stream at capture resolution until relaunch.
- All four layers publish an empty `header.frame_id`, which breaks tf2/RViz consumers.
- `scripts/setup.sh` and `link_extensions.sh` hardcode `/home/ofer/isaacsim` while `install.py`
  correctly probes `Path.home()`. Ships one user's path to everyone.
- `setup.sh` ends with `isaac-core doctor || true`, so a broken environment still prints
  "Setup complete."
- `recording.save_video` does not check `writer.isOpened()`, so an unavailable codec yields an empty
  file and a success log.
- `sidecar` restart policy has no backoff, and abandons a service silently after 5 attempts.
- `SimSession` can be built with an unconnected client; only the factory methods connect.
- `session.state()` is inconsistent with `get_pose()`/`get_capabilities()` and the `GET_STATE` wire
  name.

---

## NEEDS OFER

1. **A1 scope (multi-vehicle gimbal and capture).** Two honest options: (a) add `vehicle`/`camera`
   parameters and key gimbal state per vehicle — correct, a day's work, touches 10 sites; or
   (b) reject the call when more than one vehicle is configured, so it fails loudly instead of
   silently retargeting — an hour. (b) is not a lie; it is a documented limitation. I lean (a),
   because the README already promises per-vehicle everything.

2. **Unwired `PoseSource` values.** `script`, `replay` and `mavlink` are accepted config with no
   implementation: the vehicle composes and never moves, with no error. Trim the enum to
   `{udp, ros}` for v2, or add a validator that rejects them? I lean trimming — this is the same
   "ships something that does not work" problem we just removed the deferred methods for.

3. **The `sidecar` package.** A full supervisor, registry and restart policy with **zero** registered
   services; `python3 -m isaac_core.sidecar` can only ever log "Unknown service kind". RTSP went
   native, so nothing uses it. Delete for v2, or keep as a documented extension seam with one
   working example? I lean delete — it is a subsystem's worth of surface for a capability that moved.

4. **Default features.** Should a default `isaac-core run` enable `distance_sensor` (as now), or just
   `camera_udp` to match the first-flight walkthrough? Affects what a new user sees first.

---

## REJECTED

- **"Path confinement in `capture_frame` may be exploitable."** Tested and it is sound: `.resolve()`
  follows symlinks before the `relative_to` check, so symlinks pointing outside, symlinked parents,
  absolute paths and `..` are all correctly rejected. The reviewer verified this too and said so. The
  only real issue is that the error message echoes absolute host paths — accepted under Phase 2.
- **"The import-linter layering is ceremony."** The reviewer examined it and withdrew this: the
  layering is what keeps 1571 tests runnable with no GPU, no Isaac and no ROS. Kept as is.
- **`normalize_angle` maps exactly `+pi` to `-pi`.** Correct and intentional; same angle, different
  representation. Documented, not changed.
- **`slerp` does not clamp `t`.** All callers pass `alpha` in `(0, 1]`. Noted as a precondition, not
  changed.
- **F5, the one-in-four gimbal hiccup.** Ofer's call: not reproduced since the axis fix, ignore
  unless it recurs.
