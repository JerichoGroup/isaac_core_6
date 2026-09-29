# Phase 8 Stage 3 — eyes-on scenarios

Sixteen scenarios covering every shipped capability. Each one launches a real simulator with a window,
drives it, and prints what to look at while it runs. Some also write a file to open afterwards.

Every scenario below has been smoke-tested end to end, so a crash is a finding rather than an expected
rough edge.

---

## Before you start

**Keep the GPU free.** A Cesium stage needs roughly 3 GiB, and two vehicles need more. A resident language
model or other GPU workload starves the renderer *silently* — the simulator composes and then stops
responding, with nothing in the output saying why. Check first:

```bash
nvidia-smi --query-gpu=memory.free --format=csv,noheader     # want > 4000 MiB
```

**Source ROS 2 for scenarios 3, 4, 11 and 14:**

```bash
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash   # only needed for scenario 4's bbox message
```

**Run everything from the repo root**, and note that Isaac takes 15–30 s to appear:

```bash
PYTHONPATH=src ./scripts/eyes_on_check.py list
PYTHONPATH=src ./scripts/eyes_on_check.py 9
PYTHONPATH=src ./scripts/eyes_on_check.py 9 --keep-open 600     # longer look
```

Ctrl-C leaves cleanly. Isaac ignores SIGTERM, so if you ever need to kill it from elsewhere use `kill -9`.

**Let each simulator exit before starting the next.** Isaac ignores SIGTERM, so a lingering one keeps
holding UDP 33333 and RTSP 8554. The next scenario then launches, composes, and reports a stage Z of 0.00
while the real reason sits buried in the log as `Address already in use`. If a scenario reports zeros, check
for a stray first:

```bash
pgrep -af 'isaac_core.sim' && echo "still running -- kill -9 it before continuing"
```

One caveat found while testing: `--verify all` launches five simulators back to back, and the two-vehicle
scenario can time out under that pressure even though it passes on its own. Verify scenario 6 separately if
you use that mode.

---

## The scenarios

| # | Scenario | What it proves | ROS 2 |
|---|---|---|---|
| 1 | Terrain and camera tracking | Cesium terrain, UDP pose, heading follows the track | |
| 2 | Gimbal start angles and slewing | gimbal config, rate-limited slew, axis directions | |
| 3 | Distance sensor | rangefinder readings against known altitude | yes |
| 4 | Bounding boxes with occlusion | bbox topic, occlusion, geodetic target positions | yes |
| 5 | Frame capture at several resolutions | capture independent of window size | |
| 6 | Two vehicles (swarm) | per-vehicle mounts, ports, topics, separation | |
| 7 | Lifecycle: pause, step, resume, reset | control plane stays responsive through all four | |
| 8 | RTSP video stream | H.264 stream on the expected mount | |
| 9 | Zoom, linear in field of view | zoom travel, rate limit, clamping, the FOV choice | |
| 10 | Segmentation recording | label-free per-prim segmentation to an mp4 | |
| 11 | ROS 2 pose source | the second pose source, driven as MAVROS | yes |
| 12 | PoseBot flight library | orbit, path, steer, track, follow | |
| 13 | MAVLink bridge | a real autopilot stream driving the sim | |
| 14 | Recording topics to disk | the topic recorders and their sidecar timing | yes |
| 15 | Pose sender GUI | flying by hand, readback, jog keys | |
| 16 | A layer authored outside the repo | discovery and composition without touching isaac_core |  |

---

## What to look for, and what broken looks like

The running scenario prints its own full watch list. This is the short version, plus the failure that
matters most in each case.

**1 — Terrain and camera tracking.** Textured ground, not grey or empty. On each straight leg the terrain
flows top-to-bottom. The nose re-points when the bearing changes rather than sliding sideways.
*Broken:* no ground at all (unreachable tileset), or the view is Kit's default perspective rather than the
drone camera. Brief stutters over new ground are tile streaming, not a fault.

**2 — Gimbal.** The camera already looks slightly down at startup (start angle −15°). Each move slews
smoothly at about 15°/s. `+pitch` raises the view, `+roll` drops the right side, `+yaw` turns right.
*Broken:* any move snapping instantly, or the aircraft moving rather than just the camera.

**3 — Distance sensor.** `ros2 topic echo /isaac_core/distance_sensor` in another terminal. The range
tracks the printed expectation at each altitude step. `min_range`/`max_range` read 0.2 and 5000.
*Broken:* range stuck, or the rated band reading 0.0 and a tiny negative number.

**4 — Bounding boxes.** Boxes appear around the labelled cubes, and disappear when occluded.
*Broken:* boxes on nothing, boxes that ignore occlusion, or an empty message array while cubes are plainly
in frame. If every message is empty, check the startup report — it now warns when bbox has no targets.

**5 — Frame capture.** Both PNGs are written; the 4K one really is 3840×2160 even though the window is
720p. The window returns to its original size afterwards.
*Broken:* a file at the window's size instead of the requested one, or a window left resized.

**6 — Two vehicles.** Two viewport windows, each named after its vehicle. The two aircraft sit at
different altitudes and do not track each other.
*Broken:* one window, or both vehicles moving together — that would mean the second vehicle's pose port is
being written by the first.

**7 — Lifecycle.** Pause stops motion but the terminal stays responsive. `step` advances visibly while
paused. Resume continues. Reset returns the gimbal to its configured start angles.
*Broken:* the control plane going silent during pause, which would mean the loop stopped rather than the
timeline.

**8 — RTSP.** `ffplay rtsp://127.0.0.1:8554/stream` shows the camera view.
*Broken:* connection refused (the server never bound), or a stream that opens but never paints.

**9 — Zoom.** Each step zooms smoothly at about 25°/s of field of view. **Level 0.5 should look about
halfway zoomed.** The printed focal length and field of view agree with the picture. Out-of-range values
clamp to 20 and 200 mm.
*Broken:* level 0.5 looking nearly fully zoomed in — that means the level is interpolating focal length
instead of field of view, which is the specific mistake this scenario exists to catch. Also broken: any
step snapping, or the aircraft moving.

**10 — Segmentation.** The *viewport* shows the normal camera; the segmentation goes to the file. Open
`~/isaac_core_out/eyes_on_segmentation.mp4` afterwards. Every distinct object should hold **one flat colour
that stays with it** as the viewpoint orbits.
*Broken:* colours flickering on a **stationary authored object**, or any solid black frame. Three things
that are **not** faults:
- Cesium terrain reading as a few large regions rather than per-building instances.
- The same patch of Cesium terrain **changing colour from frame to frame within one recording**. Streamed
  tiles are regenerated geometry with no stable per-prim identity, so the annotator has nothing durable to
  key on. Judge colour stability on authored objects only — the cubes are the thing to watch.
- Colours differing between two recordings, since instance ids are assigned per session.

**11 — ROS 2 pose source.** The aircraft moves with **no UDP sender running at all**. Each printed stage z
matches the commanded altitude minus 516.7 within a metre.
*Broken:* nothing moving, which would mean the ROS subscribers never matched. Note a positive yaw here
turns the opposite way to the UDP packet's yaw — MAVROS publishes ENU while the packet carries NED, and
that difference is correct.

**12 — PoseBot.** `orbit` is a smooth circle at constant radius with the nose tangential. `fly_path` flies
straight legs, turns to face each new one, and **ends exactly on the last waypoint**. `steer` is a
constant-radius turn, not a polygon. `track_point` holds position and only rotates. `follow_point` then
chases, holding roughly 250 m away and 150 m above.
*Broken:* any command jumping between poses instead of streaming, `track_point` drifting off station, or
`fly_path` overshooting its last waypoint.

**13 — MAVLink.** The aircraft follows the fake autopilot with no UDP sender and no ROS 2 involved. Each
printed stage z matches the commanded altitude minus 516.7.
*Broken:* nothing moving. If so the bridge is not receiving — it must be the listener (`udpin`) and must be
up before the autopilot starts talking.

**14 — Recording.** All three counts non-zero. The written mp4 plays and shows the flight. A
`.timestamps.txt` sits beside it, which is how real per-frame timing survives a constant-rate container.
The pickles are substantial, not a few bytes.
*Broken:* a zero count, or an mp4 that will not open.

**15 — Pose sender GUI.** One tab, already on the port this simulator listens on. The **`SIMULATOR SAYS`**
line shows the stage's own transform beside what you are sending. Arrow keys nudge yaw and pitch;
PageUp/PageDown nudge altitude. Copy call and Copy TOML put usable text on the clipboard. There is
deliberately no mission control here — scripted flight belongs in the devkit.
*Broken:* a tab on the wrong port, which shows as a sender reporting packets going out while the camera
never moves. The `SIMULATOR SAYS` line is what tells you which half is wrong.

**16 — A layer authored outside the repo.** The evidence is in the startup report, not the viewport:
`eyes_on_probe` appears as a **composed** feature layer. It lives in a temporary directory and was found
through `assets.layer_search_paths` alone, with nothing in `isaac_core` edited.
*Broken:* absent from the report entirely, which would mean discovery never reached the search path.

---

## What to report back

Per scenario, one line is enough:

```
9  PASS
6  FAIL - only one viewport window opened
10 PASS - but the ground read as one colour, not several
```

For anything that is not a clean pass, the useful details are:

- **What you saw**, against the specific expectation above.
- **The last 20 lines of terminal output**, which carry the printed expectations it was measured against.
- **For the file-writing scenarios (10 and 14)**, whether the file opened and what it looked like.
- **`nvidia-smi --query-gpu=memory.free --format=csv,noheader` at the time**, since a starved renderer
  looks like a broken feature and is the single most common false alarm.

If a scenario crashes rather than showing something wrong, the traceback is the finding — send it as is.

---

## Artefacts

| Scenario | File |
|---|---|
| 5 | `~/isaac_core_out/` PNGs at native and 4K |
| 10 | `~/isaac_core_out/eyes_on_segmentation.mp4` |
| 14 | `~/isaac_core_out/eyes_on_recording.mp4`, `eyes_on_poses.pkl`, `eyes_on_ranges.pkl` |

Scenario 10 and 14 write large files — roughly 20 MB and 44 MB in testing. Delete them when you are done.
