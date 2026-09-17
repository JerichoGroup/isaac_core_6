# Version 3 — plan

V2 shipped a sandbox that flies a camera and publishes what it sees. V3 hands the user better
controls, adds the sensing we do not have, and removes the last places where the repo assumes
something about the machine it runs on.

Sizes are honest: **S** under a day, **M** one to three days, **L** a week or more.

---

## Decisions taken, and what they cost

**Cameras keep no name.** `[vehicles.<id>.cameras.<name>]` becomes `[vehicles.<id>.camera]`. This does
**not** cost you a thermal camera alongside the visual one: an extra imaging sensor arrives as a
*layer*, which mounts under the same vehicle at `{mount}`, so it moves with the airframe and its gimbal
and brings its own camera prim, its own topic and its own RTSP node. What the unnamed built-in camera
keeps exclusively is being the one the pose drives and the one `capture_frame`, `image_rgb` and the
default RTSP stream refer to. I verified the layer path end to end in V2, including config reaching a
prim attribute. So: one name fewer, both cameras still possible. This supersedes the roadmap's *More
than one camera per vehicle* item, which existed only to distinguish cameras that could not coexist.

**`$ISAACSIM_PATH` goes entirely.** No reading it, no printing it, no mentioning it outside
`migrating_from_2023.md`. When the install is not found, `setup.sh` and `doctor` say so and tell the
user to pass `--isaac-path /path/to/isaacsim`. You were right to push back on my reasoning, too — you
had set that variable correctly early on, so "it worked despite staleness" was not evidence of
anything.

**Zoom level is linear in field of view**, not in focal length. Equal steps in focal length feel
wrong at the long end. `set_zoom(focal_mm=...)` stays available for the exact form.

**One sender command.** `isaac-core-pose-sender` keeps its name and gains a per-tab pose source, so a
single window can drive one vehicle over UDP and another over ROS 2. Two commands would have meant two
windows and two of every feature below.

**Live stage manipulation is gone** — removed from this plan and from the roadmap. Composing a layer
onto a running stage and swapping scenes were the two operations that historically aborted Kit with no
traceback, and neither blocks any workflow.

---

## What the investigation changed

Four requests are much smaller than they look.

| Request | Reality found in the code |
|---|---|
| Pose bot | **Every motion primitive already exists** as a pure function in `isaac_core.vehicle`: `move_to`, `move_forward_backward`, `move_right_left`, `move_up_down`, `turn_roll`, `turn_pitch`, `turn_yaw`, `turn_to_point`, `steer`. Only the stateful class is missing. The 2023 `turn_to_point` roll bug **is already fixed** here: ours computes yaw and pitch and preserves roll |
| Segmentation | The annotator exists and needs no new subsystem. Isaac's Python already has `cv2`, `PIL` and `numpy`, so the simulator can encode the mp4 itself |
| Autocompletion | `isaac-core completion bash\|zsh` already emits a working 42-line script generated from the live schema. Nothing installs it, which is why TAB falls back to filenames |
| Zoom | `focal_length_mm` is already a config key with both apertures beside it. The maths is `focal = aperture / (2·tan(hfov/2))` |
| Camera naming | `eo` is **only** a default dict key in `schema.py`. Not in any USD file, not a layer id, nothing outside the config reads it |

---

## Phase 1 — Controls the user drives (M)

### 1.1 `PoseBot` (S)

A stateful sender wrapping primitives that already exist. Commands, not pacing:

```python
with PoseBot(port=33333) as bot:                       # closes the socket on exit
    bot.move_to_point(32.22481, 35.25621, 900.0, duration_s=5.0)
    bot.move_forward_backward(100.0)
    bot.turn_to_point(32.22481, 35.25621, 1000.0)
    bot.steer(turn_radius_m=100.0, speed_ms=50.5, duration_s=0.5)
    bot.orbit(32.22481, 35.25621, radius_m=800.0, speed_mps=30.0, duration_s=60.0)
    bot.fly_path([p1, p2, p3], speed_mps=50.0, face_travel=True)
    print(bot.pose)                                    # where it thinks it is
```

Each call streams poses for `duration_s` and leaves the bot where it arrived, so calls compose. The
send loop uses an accumulating deadline rather than `sleep(dt)`, so a long session does not drift.
`MotionLimits` is accepted and **enforced**, closing the roadmap item where limits were coded and
tested while nothing applied them.

Named `PoseBot` rather than `UdpBot` because it takes a transport, so the same object drives a ROS 2
target.

**`hold()` is dropped.** You were right to question it: the simulator holds the last good pose by
design, so a "hold" is a `sleep` wearing a method's clothes.

### 1.2 Waypoint paths (S, part of 1.1)

`fly_path(waypoints, speed_mps, face_travel=True)` is exactly what you described: fly p1 → p2 → p3 at a
constant ground speed with the airframe pointing along travel, which means yaw and pitch come from the
leg direction and roll stays level. `PathTrajectory` already generates the positions; the new part is
the facing. Listed here rather than under swarm, because you are right that it has nothing to do with
swarms.

### 1.3 The GUI (M)

One window, tabs, and a `+` that adds one. **Every feature applies to every tab regardless of its pose
source** — the source is a per-tab toggle and nothing else differs.

**Core**
- **Per-tab target**: pose source (UDP or ROS 2), host, port or topic namespace, and rate.
- **Live readback** of what the simulator reports, beside what the tab is sending, read over the
  control plane. A frozen camera then tells you *which* half is wrong.
- **Connection state**: whether a simulator is up, and whether anything is listening.
- **Jog keys**: arrows for yaw and pitch, `PgUp`/`PgDn` for altitude, `Shift` for a coarse step.
- **Ramp instead of jump**: "go there over N seconds", driving `PoseBot` so it is a flight rather than
  a teleport.
- **Per-tab RTSP view**, so one window is the whole cockpit. Two ways to do it, and the choice is worth
  making explicitly: embed the decode with `opencv` plus `PIL` (already an optional devkit dependency,
  costs a decode thread per tab), or launch `ffplay` in its own window on a button. I would embed when
  `opencv` is importable and fall back to the button when it is not, so the feature never hard-depends
  on a codec being present.

**Worth having**
- **Named presets** saved to disk: the scene reference point, a few vantage points, last used.
- **Copy as snippet**: the current pose as TOML or as a `set_pose(...)` call.
- **Gimbal and zoom panels** over the control plane, so aiming needs no second terminal.
- **Hold/resume** on the send loop, with a visible packet counter.

Left out deliberately: a map widget (a real dependency for little gain) and recording controls (the
devkit does it better than a GUI would).

---

## Phase 2 — Segmentation (M)

Your requirement is the whole stage segmented by default, and a recorder that starts, stops and saves
an mp4 without being told a frame rate. That rules out the obvious route, so here is what the
investigation found.

**Why not the ROS 2 camera helper.** `ROS2CameraHelper` — the node our camera layer already uses —
offers `semantic_segmentation` and `instance_segmentation`. Both are Replicator "SD" annotators that
key off the semantic hierarchy: a prim with no `SemanticsAPI` is invisible to them, which is exactly
why `apply_target_semantics` exists for the bbox layer. Going that way would mean labelling prims and
would segment only what we labelled. The only "include unlabelled" switch in Replicator belongs to the
pointcloud annotator, not to segmentation. So yes — your reading was right, and it is the reason to
take a different route.

**What we use instead: `instance_id_segmentation`.** It is registered in this Isaac install, it keys
off prims rather than labels, and it takes a `colorize` option that yields an RGB image directly. Every
distinct prim in view gets its own colour with nothing to configure and nothing to label. It is not
exposed by the ROS 2 helper, so we read it in the simulator instead — which is fine, because the
deliverable is a file.

**Design**

1. A `segmentation` feature attaches the `instance_id_segmentation` annotator to the vehicle's render
   product, colorized.
2. Control plane gains `start_segmentation_recording()` and `stop_segmentation_recording(path)`. The
   simulator accumulates frames with their real timestamps while recording.
3. On stop it encodes an mp4 with `cv2` at the **measured** rate — reusing `measured_fps` and the
   `<name>.mp4.timestamps.txt` sidecar the video recorder already writes, so a variable frame rate is
   recorded rather than assumed. `cv2` is present in Isaac's Python, already checked.
4. The path is resolved under `sim.control_plane.output_root` and confined to it, exactly as
   `capture_frame` is.
5. `segmentation_recorder()` in the devkit wraps the two calls so a script reads like the video
   recorder does.

**Out of scope, deliberately:** semantic labels and class names, an `instance_segmentation` variant, and
a live ROS 2 topic. Each is a real thing someone might want later; none is needed for "record the
stage's segmentation to a file".

**The known limitation:** Cesium terrain tiles stream in as generated geometry, so whether they get
per-prim ids is unknown until measured. Expect terrain to be background. Anything authored in the scene
will segment.

**First hour of this phase is an experiment, not code:** attach the annotator to a running stage, dump
one frame, and count distinct colours over the labelled cubes and over terrain. That decides whether
the terrain caveat is a footnote or a headline before anything is built on top.

---

## Phase 3 — Zoom (S/M)

The camera gains `focal_length_min_mm` and `focal_length_max_mm`. A control-plane `set_zoom(level)`
takes 0–1, linear in field of view, and writes `focalLength` on the camera prim with the horizontal
aperture held fixed so the FOV follows:

```
focal_mm = horizontal_aperture_mm / (2 · tan(hfov_rad / 2))
```

`get_zoom()` reports level, focal length and the resulting horizontal FOV, because which number is
useful depends on the question. Rate limiting like the gimbal's, so a zoom can be a move rather than a
jump. `set_zoom(focal_mm=...)` for the exact form.

This phase also settles which of `fov_deg` and `focal_length_mm` is authoritative, since zoom makes
that existing overlap user-visible.

The reference node's measured HFoV curve is worth remembering: a real lens is not linear in either
quantity, so leave room for a per-camera calibration table later without designing for it now.

---

## Phase 4 — Stop assuming anything about the machine (S)

- **Remove `$ISAACSIM_PATH` from `IsaacInstall.locate` entirely.** Resolution becomes: explicit
  `--isaac-path` → config → probe known locations. Not found means a clear message naming the flag.
- **Never print an environment variable as an instruction.** Every message becomes the resolved
  absolute path, since `doctor` and `setup.sh` already know it. Sites: `README.md` install step 3,
  `docs/first_run.md`, `sim/__main__.py`'s docstring, and the `doctor` hint.
- **`scripts/crash_rate.sh`** currently defaults to `${ISAACSIM_PATH:-$HOME/isaacsim}`, so it can
  benchmark the wrong install silently. It uses the same resolution as everything else.
- The only surviving mention is historical, in `migrating_from_2023.md`. A guard test keeps it that way.

---

## Phase 5 — One camera, no name (S)

`cameras: dict[str, CameraConfig]` becomes `camera: CameraConfig`. Touches the schema, `{camera}`
substitution in the planner and manifests, the topic resolver, both config reference files, the README
and the docs. The loader rejects the old `[vehicles.x.cameras.y]` shape with a message naming the new
form rather than a generic validation error, and `migrating_from_2023.md` gains the note.

Do this **before** Phases 2 and 3 so segmentation and zoom are not plumbed through a camera dict that
is about to disappear.

---

## Phase 6 — Autocompletion, installed (S)

`setup.sh` installs it, not offers it. `isaac-core completion --install` writes the script for the
detected shell and says what it did; `--print` keeps the current behaviour for anyone scripting it.
`doctor` reports whether completion is installed, because a feature nobody knows about is not shipped.
Completes subcommands, flags and config keys after `--set`, all of which the generator already
produces.

---

## Phase 7 — Swarm parity and the last roadmap items (M)

- **Per-vehicle gimbal, capture and zoom.** `set_gimbal` and `capture_frame` refuse when more than one
  vehicle is configured. Give them a `vehicle` argument, key gimbal and zoom state per vehicle, and
  route them through the resolution `set_pose` and `get_pose` already use. This is the last place a
  swarm is a second-class citizen.
- **Layer registration by entry point.** Read the `isaac_core.layers` group during discovery and treat
  each resolved directory as another search path — the one remaining place where installing a layer is
  not just installing a package.
- **Target tracking / follow-me.** Pure `vehicle` logic on `PoseBot`: re-aim each tick at a moving
  point. No new nodes.

---

## Phase 8 — Production hardening (M, planned together later)

Not planned in detail on purpose. When we get here we do what we did for V2: a pass over the whole repo
with fresh eyes, then a written plan. The things already known to belong in it:

- Chase the swarm system-test failure to a root cause. The simulator's log shows it composed and was
  serving while `session.state()` did not report ready; the readiness helper now records the last state
  and error, so the next failure names its cause. Do not raise the retry budget — that turned a fast
  skip into a forty-minute hang in V2.
- Continuous integration. The unit suite needs no GPU and runs in 19 seconds.
- A packaged-wheel test: build, install into a clean environment, assert the scene and layers resolve
  from the installed location. V2 shipped a bug where nothing shipped at all.
- Version 1.0.0 and a changelog covering the config changes, so an existing config is migrated
  deliberately.

---

## Order

| Phase | Work | Size | Why here |
|---|---|---|---|
| 1 | `PoseBot`, waypoints, GUI | M | Highest daily value, no Isaac risk |
| 4 | Remove the env var | S | Cheap, and a correctness issue on any machine but yours |
| 6 | Autocompletion installed | S | Cheap, visible, already built |
| 5 | One camera, no name | S | Before 2 and 3, so per-camera plumbing is done once |
| 2 | Segmentation | M | Starts with a one-hour experiment |
| 3 | Zoom | S/M | Settles `fov_deg` vs `focal_length_mm` |
| 7 | Swarm parity, entry points, tracking | M | Removes the last special cases |
| 8 | Hardening and 1.0.0 | M | Planned together when we arrive |

Phases 1, 4 and 6 are independent. Everything else wants 5 done first.

---

## Nothing outstanding for you

All five questions from the first draft are answered and folded in. The one thing that could still
change the shape of a phase is the Phase 2 experiment: if `instance_id_segmentation` turns out not to
cover authored scene geometry the way the annotator's description implies, I will come back with what
it does cover before building the recorder on top.
