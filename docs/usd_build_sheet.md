# USD task list

The USD side is complete for the camera layers. `isaac-core run` launches Isaac Sim 6,
composes the stage, a UDP pose packet moves the camera, and both ROS topics publish with
real timestamps:

| Topic | Type | Rate | Verified |
|---|---|---|---|
| `/isaac_core/global_pose` | `geographic_msgs/msg/GeoPoseStamped` | ~100 Hz | lat/lon/alt match what was sent; stamp advances |
| `/isaac_core/image_rgb` | `sensor_msgs/msg/Image` | ~115 Hz | publishing |

Camera translate checked against the geodetic math: with the ENU reference at 516.7 m,
altitude 1500 gives 983.3 and altitude 1000 gives 483.3.

Timestamp checked with two consecutive `ros2 topic echo` samples:
`sec: 2, nanosec: 450000000` then `sec: 12, nanosec: 16666667` — non-zero, advancing, and
`nanosec` correctly inside `[0, 1e9)`.

**Nothing is outstanding for you in the camera layers.** The remaining item below is mine.

---

## 1. Mine to chase — intermittent segfault on startup

Roughly **one launch in three** dies with `exit code 139` immediately after the runtime
logs `simulation running`. py-spy shows the main thread inside Kit's `update_app()` with
the control-server thread idle in `select`, so it is not a Python-level race in our code.

Established so far:

- **Not caused by your wiring or the new node.** It reproduces with `isaac_core_ogn.math`
  and `isaac_core_ogn.position` both disabled, and predates the timestamp work.
- **Not the tile server.** `http://10.44.134.160:8088/nablus/tileset.json` returns HTTP 200.
- **Not the ROS 2 bridge**, which was yesterday's wrong suspect.
- **Aggravated by stale `/tmp/carb.*` directories.** Isaac leaves one behind per crashed
  process; 48 had accumulated. Clearing them improved the rate noticeably. If you see a
  run of failures, `rm -rf /tmp/carb.*` with no Isaac running is safe and worth trying.
- A run that opens **no stage at all** survives every time, so it is tied to stage opening.

Practical effect: relaunch and it usually works on the second try. Not a blocker for
development, but it needs a real fix before anyone relies on unattended runs.

---

## 2. Parked — sensor layers

Not needed for the core. When we pick this up I will write per-file instructions at the
same level of detail as the camera layers: `usd/layers/distance_sensor/`,
`usd/layers/bbox_publisher/`, `usd/layers/sat/`.

Already established for when we do:

- Isaac Sim 6 ships a native **physics raycast sensor**, which likely replaces the old
  custom raycast script node entirely.
- `bbox_publisher` is where Cesium **Globe Anchors genuinely belong**: each target object
  under `/bboxes` needs pinning to a fixed geographic spot, unlike the camera.

## 3. Parked — monotonic frame id

`ROS2CameraHelper` bakes `frameId` once at writer initialisation, so a counter cannot
reach the image topic. Downstream re-stamping is ruled out — it cannot detect upstream
drops. Likely answer is a custom **C++** OmniGraph node, immune to the Python ABI problem.
Now that `header.stamp` is real and advancing, it covers most of what a frame counter was
wanted for, so this is lower priority than it was.

---

## Settled, for reference

- **Body axes** in the `/Root/Xform` frame: **+X = nose, +Y = left wing, +Z = up**.
- **Roll and pitch are not flipped.** `ned_to_enu` swapping them is correct compensation
  for the body frame sitting 90° from the world axes at zero yaw.
- **Camera orientation** is `xformOp:orient = (0.5, 0.5, -0.5, -0.5)`, giving view along
  the nose and image-up up.
- **Prim targeting is by USD relationship**, never a path string, so a layer survives
  being referenced at any mount point and works for N swarm instances.
- **Layers mount under `/World/Environment/{instance}`**, matching the authored scenes.
  The 2023 repo rooted at `/Environment`, which put layers outside the scene graph.
- **ENU reference is derived from the scene's Cesium georeference** automatically.
- **No Cesium Globe Anchor on the camera Xform** — it fights the pose graph.
- **Feature layers are derived from `pose_source`**, so `[features] enabled` can stay
  empty for a single-vehicle config.
- **`ros2_context` and `isaac_read_simulation_time` correctly have no `execIn`.** Both are
  data-only providers, pulled rather than ticked. The wiring test knows this.
- **`seconds_to_ros_stamp` does need the tick**, because the timestamp must be recomputed
  every frame rather than cached.

## Three environment gotchas worth knowing

- Deleting or renaming an OGN node leaves its **generated database** behind in
  `~/.cache/ov/ogn_generated/`. Isaac still registers the stale type and can take the
  process down. `scripts/link_extensions.sh` now clears those caches. Clear them by hand
  after adding a node too, or the new type will not appear in the GUI.
- **ROS topics take several seconds to appear** after the sim reports ready: the publisher
  needs a couple of graph ticks plus DDS discovery. Checking too early looks exactly like
  a broken publisher, and cost me a long detour today.
- **`omni.graph.action.OnTick` does not fire** unless the timeline is playing or
  `inputs:onlyPlayback` is false. A graph that looks dead in a headless probe is often just
  not playing.
