# USD build sheet

What to create in the Isaac Sim 6 GUI, and why each piece exists. Kiro never authors
`.usda` files (decision D15) — authoring in the GUI guarantees valid, correct files
instead of hand-written multi-thousand-line text.

Work through this in stages. **Stage 1 alone gives a first end-to-end run**, which is
worth having before authoring anything else, because it validates the whole chain:
config → layer manifest → stage composition → OmniGraph → camera motion.

Read this alongside `KIRO.md` section 7 (agreed architecture) if you want the
reasoning behind the structure.

---

## Conventions that matter

**Prim paths are a contract.** Layer manifests bind config values to specific prim
paths, so a rename in the GUI breaks configuration silently. Names below are exact.
A contract test asserts every path a manifest declares actually exists in the USD, so
a mismatch fails CI rather than failing at 2am — but only once both sides exist.

**One concern per file.** The old repo had two 2.9 MB camera files that were
near-identical copies. Most of that bulk was an embedded viewport camera gizmo mesh
(`OmniverseKitViewportCameraMesh`). **Do not embed that mesh** — delete it if Isaac
adds one. Files here should be tens of kilobytes.

**Feature layers mount under `/Environment/<instance>`.** `<instance>` is the vehicle
id from config (default `drone_0`). Composition creates the mount prim and adds your
layer as a reference, exactly as the old repo did — that part of its design was good
and is kept.

**Save location:** `usd/scenes/` for base stages, `usd/layers/<layer_id>/` for feature
layers. Each layer directory also gets a `layer.toml` — I write those, you write the
USD, and they must agree.

---

## Stage 1 — minimum for a first run

Two files. Target: launch, see terrain, and have a UDP packet move the camera.

### 1.1 `usd/scenes/earth.usda` — the base stage

Create a new stage and build this hierarchy:

```
/World                       Xform   (default prim)
├── /Environment             Xform   ← mount point for all feature layers
├── /tilesets                Scope   ← Cesium tilesets live here
└── /bboxes                  Scope   ← objects eligible for bbox reporting
```

Then add Cesium, via the Cesium panel rather than by hand:

- **CesiumGeoreference** — set its anchor to the same point as `[geo].enu_reference`
  in config, currently `lat 32.22481, lon 35.25621, alt 516.7`. These two *must*
  agree or the terrain and the aircraft will disagree about where they are.
- One or more **Cesium Tileset** prims, parented under `/tilesets`.
  Leave each tileset's `cesium:url` pointing at whatever your tile server serves.
  Composition rewrites these at launch from `[cesium].tileset_server_url`, so the
  value you save is only a default — this is what removes the nine hardcoded
  `10.20.15.122` URLs the old `earth.usda` carried.

Notes:

- `/tilesets` and `/bboxes` may be **empty**. The capability probe detects their
  absence and skips dependent features with a printed reason instead of crashing,
  which is the fix for the old `Simulation` class blowing up on a stage without them.
- Add a dome light if you want ambient illumination. Keep any HDRI out of the package:
  large binaries live in `assets_heavy/` and are optional, with a default dome light
  fallback.
- **Do not** add a camera here. Cameras arrive with the camera layer, so the scene
  stays reusable across vehicle configurations.

### 1.2 `usd/layers/camera_udp/camera_udp.usda` — pose-driven camera

This is the layer that makes something move. Hierarchy:

```
/Root                                  Xform
├── /Root/Xform                        Xform   ← THE MOVED PRIM (writers target this)
│   └── /Root/Xform/main_camera_01      Camera
└── /Root/PoseSync                     OmniGraph
```

`/Root/Xform` is what the graph writes to; the camera hangs off it. Keeping the
camera a child of the written Xform means camera intrinsics and vehicle pose stay
independent — one is authored, the other is driven.

On `/Root/Xform`, make sure the transform stack has **exactly** these two ops, in
this order, and no others:

- `xformOp:translate` (double3)
- `xformOp:orient` (quatd — note **d**, not f)

Extra ops such as `xformOp:scale` or a `xformOp:rotateXYZ` will fight the writers.

#### The `PoseSync` graph

Add these nodes inside `/Root/PoseSync`. Node types in the `isaac_core_ogn.*`
namespace are ours; link them into Isaac first with `scripts/link_extensions.sh`,
then enable **Isaac Core Position** and **Isaac Core Math** in the extension manager
so they appear in the node search.

> **Changed 2026-08-24.** An earlier draft of this sheet included our own
> `Ros2Gimbal` and `Ros2GlobalPosePublisher` nodes. Those are gone: Humble's `rclpy`
> is a Python 3.10 C extension and Isaac Sim 6 runs Python 3.12, so no node of ours
> can import it. All ROS work is now done by Isaac's own C++ bridge nodes. See
> `docs/ros2_and_python.md`. **Stage 1 below needs no ROS at all** — start here and
> ROS comes in Stage 3.

| Prim name | Node type |
|---|---|
| `on_playback_tick` | `omni.graph.action.OnPlaybackTick` |
| `udp_to_global_position` | `isaac_core_ogn.position.UdpToGlobalPosition` |
| `global_to_local` | `isaac_core_ogn.math.GlobalPositionToLocalPosition` |
| `translate_writer` | `omni.graph.nodes.WritePrimAttribute` |
| `orient_writer` | `omni.graph.nodes.WritePrimAttribute` |

Wiring — execution first, every node ticked from the same source:

```
on_playback_tick.tick  ──►  udp_to_global_position.execIn
                       ──►  global_to_local.execIn
                       ──►  translate_writer.execIn
                       ──►  orient_writer.execIn
```

Then data:

```
udp_to_global_position.global_position     ──►  global_to_local.global_position
udp_to_global_position.global_orientation  ──►  global_to_local.global_orientation

global_to_local.local_position     ──►  translate_writer.value
global_to_local.local_orientation  ──►  orient_writer.value
```

Leave `global_to_local`'s `offset_roll` / `offset_pitch` / `offset_yaw` unconnected for
Stage 1 — they are the gimbal offsets, and static values are fine until Stage 3 wires a
live gimbal subscriber into them.

Writer targets:

- `translate_writer`: `prim` → `/Root/Xform`, `name` → `xformOp:translate`
- `orient_writer`: `prim` → `/Root/Xform`, `name` → `xformOp:orient`

Leave `ros2_context` unconnected for now if the gimbal and pose publisher resolve
their own context; connect its `context` output to their `context` input if they
expose one.

#### Values to leave alone

Don't bother tuning these in the GUI — composition overwrites them from config at
launch, which is the entire point of the binding table:

`udp_port`, `enu_reference`, the gimbal's `start_roll/pitch/yaw` and `rotation_frame`,
the pose publisher's `hz` and `topic_name`, the camera's `focalLength` and
`horizontalAperture`.

Set them to something sane so the file is usable standalone, then forget them.

#### Cesium globe anchor

Add a **Cesium Globe Anchor** to `/Root/Xform` via the Cesium panel. Without it the
camera moves in local ENU while the terrain is georeferenced, and the two drift apart.

---

## Stage 2 — image publishing

Add a second graph, `/Root/CameraImageExport`, to the same camera layer:

| Prim name | Node type |
|---|---|
| `on_tick` | `omni.graph.action.OnPlaybackTick` |
| `create_viewport` | Isaac Create Viewport |
| `set_viewport_resolution` | Isaac Set Viewport Resolution |
| `get_render_product` | Isaac Get Viewport Render Product |
| `set_camera` | Isaac Set Camera |
| `ros2_camera_helper` | `isaacsim.ros2.bridge.ROS2CameraHelper` (Isaac's own, C++) |

Chain them in that order, feed `set_camera` the path
`/Root/Xform/main_camera_01`, and feed `get_render_product`'s render product path into
`ros2_camera_helper.renderProductPath`.

Two things I could not verify without running it, so treat as the risk area:

1. Whether **multi-tick rendering** (new in 6.0) supersedes the hand-rolled publish
   rate gate. If it does, the `publishRateHZ` input becomes redundant and scheduling
   moves to the render product. Worth checking on first launch.

---

## Stage 3 — ROS pose source

`usd/layers/camera_ros/camera_ros.usda`: identical to `camera_udp` except
`udp_to_global_position` is replaced by TWO of Isaac's own
`isaacsim.ros2.bridge.ROS2Subscriber` nodes -- one for `sensor_msgs/msg/NavSatFix`
and one for `geometry_msgs/msg/PoseStamped` -- whose dynamic output attributes feed
`global_to_local`. Our old `Ros2ToGlobalPosition` node no longer exists.

The **easiest correct way** to make this: open `camera_udp.usda`, delete the UDP
node, add the two `ROS2Subscriber` nodes, set their message package/subfolder/name,
wire their fields into `global_to_local`, and Save As.

This is also where the gimbal arrives: add a third `ROS2Subscriber` for the `Gimbal`
message and wire its roll/pitch/yaw into `global_to_local.offset_*`. And a
`ROS2Publisher` for `geographic_msgs/msg/GeoPoseStamped` fed from
`global_to_local.global_position` / `global_orientation` replaces the old global pose
publisher.

Important asymmetry, and it is deliberate: the UDP path converts NED→ENU, the ROS
path does **not**, because MAVROS already publishes ENU. Getting that wrong silently
mirrors the aircraft. It is handled inside the nodes, so nothing for you to wire — but
worth knowing when a heading looks wrong.

---

## Bridging quaternions and Euler angles

Added 2026-08-25, after the first build. Isaac's ROS 2 bridge exposes a quaternion as
**four separate `double` attributes** (`pose:orientation:w/x/y/z`), while
`GlobalPositionToLocalPosition` speaks `global_orientation` as a **`vectord[3]` of
radians**. Nothing in the bridge or OmniGraph's stock nodes closes that gap, so there
are now two math nodes that do:

| Node type | Takes | Gives |
|---|---|---|
| `isaac_core_ogn.math.QuaternionToEuler` | `qw`, `qx`, `qy`, `qz` (double) | `euler_r` (vectord[3]) plus `roll_r`/`pitch_r`/`yaw_r` |
| `isaac_core_ogn.math.EulerToQuaternion` | `euler_r` (vectord[3]) | `qw`, `qx`, `qy`, `qz` plus a `quatd[4]` |

Both are thin adapters over `isaac_core.geo`, which is already unit tested, and both
take `execIn` from the same `on_playback_tick`.

### Fix 1 — ROS subscriber into the math node (`camera_ros` only)

Add a `QuaternionToEuler` named `quat_to_euler`:

```
ros2_subscriber.pose:orientation:w  ──►  quat_to_euler.qw
ros2_subscriber.pose:orientation:x  ──►  quat_to_euler.qx
ros2_subscriber.pose:orientation:y  ──►  quat_to_euler.qy
ros2_subscriber.pose:orientation:z  ──►  quat_to_euler.qz

quat_to_euler.euler_r  ──►  global_position_to_local_position.global_orientation
```

No frame conversion is needed or wanted: MAVROS already publishes ENU, which is what
the math node expects. `qw` defaults to 1.0, so before the first message arrives the
node reports identity rather than propagating NaN from an all-zero quaternion.

### Fix 2 — math node into the ROS publisher (**both** camera layers)

Add an `EulerToQuaternion` named `euler_to_quat`:

```
global_position_to_local_position.global_orientation  ──►  euler_to_quat.euler_r

euler_to_quat.qw  ──►  ros2_publisher.pose:orientation:w
euler_to_quat.qx  ──►  ros2_publisher.pose:orientation:x
euler_to_quat.qy  ──►  ros2_publisher.pose:orientation:y
euler_to_quat.qz  ──►  ros2_publisher.pose:orientation:z
```

⚠ **This is a deliberate behaviour change from the old repo, worth a decision.** The
old `global_pose` publisher stuffed roll, pitch and yaw straight into the quaternion's
`x`, `y`, `z` with `w` pinned to 1.0 — so `GeoPoseStamped.pose.orientation` did not
contain a quaternion at all, and its README documented the abuse. Wiring
`EulerToQuaternion` publishes a **real** quaternion, which is what the message type
means.

If anything downstream already parses `/isaac_core/global_pose` expecting the old
convention, it will need updating. To reproduce the old behaviour on purpose instead,
skip `EulerToQuaternion` and wire `quat_to_euler`-style scalars directly:
`global_orientation` → a `BreakVector3` → the publisher's `x`/`y`/`z`, leaving `w` at
1.0. Tell me which you want and I will make it the documented default.

### Fix 3 — delete two dangling connections in `camera_ros.usda`

`camera_ros.usda` still references `udp_to_global_position`, a node that only exists in
the UDP layer — a leftover from Save-As. Two connections to remove on
`global_position_to_local_position`:

- `inputs:global_orientation` → `udp_to_global_position.outputs:global_orientation`
  (this is *why* that input reads as unconnected; replace it with `quat_to_euler` per
  Fix 1)
- `inputs:global_position` → its connection list contains **both**
  `udp_to_global_position.outputs:global_position` *and* the correct
  `make_3_vector.outputs:tuple`. Remove the former; an input with two sources is
  ambiguous even when one of them is dangling.

Easiest in the GUI: select the node, find those two inputs, disconnect everything, then
reconnect only `make_3_vector.tuple` → `global_position` and `quat_to_euler.euler_r` →
`global_orientation`.

---

## Frame IDs: why the counter does not reach the topic

Investigated 2026-08-25 after the counter was seen incrementing in the GUI while the
published `header.frame_id` stayed constant.

**The wiring (`counter` → `to_string` → `ros2_camera_helper.frameId`) is correct.** The
behaviour comes from Isaac's node. `OgnROS2CameraHelper.compute` begins:

```python
if state.initialized:
    # Camera helper was already initialized, nothing to do
    return True
```

`frameId` is read once, baked into the Replicator writer's `init_params`, and never
looked at again. That explains both halves of the symptom: the value updates in the
node's output attribute (which is why the GUI shows it climbing) while the writer keeps
its original copy, and stopping the timeline calls `custom_reset()`, so replay re-bakes
whatever the counter happens to hold and freezes there.

`isaacsim.ros2.bridge.ROS2PublishImage` *does* take `frameId` as a per-tick input, but
it is not a hand-wirable alternative: Isaac drives it as a Replicator writer
(`rep.writers.get(f"{rv}ROS2PublishImage").attach([render_product])`), fed by the
synthetic-data pipeline rather than by graph connections. No `.ogn` in the install
outputs the `dataPtr` it needs.

### The deeper point

ROS 2's `std_msgs/Header` is only two fields:

```
builtin_interfaces/Time stamp
# Transform frame with which this data is associated.
string frame_id
```

There is **no `seq`**. ROS 1 had one; ROS 2 removed it deliberately. `frame_id` is a
*coordinate frame name* — `drone_0_eo_optical_frame` — not a counter. The old repo used
it as a frame counter, and needed a custom rclpy republisher to do so, which is the
real reason its image path ran `raw_rgb` → `image_rgb` through a second node. That
detail was under-documented when the pipeline was first catalogued.

So the camera helper baking `frameId` once is **correct behaviour for what the field
means**. Our usage was the problem, inherited from the old repo.

### Recommended

1. Set `frameId` to a static frame name and drop the `counter` → `to_string` chain from
   the `frameId` input. `/isaac_core/<vehicle>/<camera>_optical_frame` is conventional.
2. Correlate frames using **`header.stamp`**, which the writer sets per frame. This is
   what ROS 2 intends and it needs no extra machinery. Verify with:
   `ros2 topic echo /isaac_core/image_rgb --field header.stamp`

If a genuine monotonic counter is needed later — detecting dropped frames, say — the
options are a sidecar republisher on system Python 3.10 (the old design, but
out-of-process where rclpy works), or a separate counter topic published alongside. Both
are additions rather than changes; neither should block getting a first run working.

---

## Stage 4 — sensor layers

One directory each, mounted under `/Environment/<instance>`:

- `usd/layers/distance_sensor/` — an ActionGraph with a raycast range finder feeding
  an `isaacsim.ros2.bridge.ROS2Publisher` carrying `sensor_msgs/msg/Range`. Isaac Sim 6 ships a **physics raycast
  sensor**, which may replace the old custom raycast script node entirely — check
  before building this one, it could be much simpler than the 2023 version.
- `usd/layers/bbox_publisher/` — reads children of `/bboxes`. Each object you want
  reported needs a Cesium Globe Anchor and the right semantics, exactly as documented
  in the old repo's README.
- `usd/layers/sat/` — frame capture on demand.

I'll write the detailed node lists for these once Stage 1 is confirmed working, since
the 6.0 sensor APIs shifted and I would rather specify against something you have
actually launched.

---

## Before you start

```bash
scripts/setup.sh          # installs into both interpreters, links extensions
isaac-core doctor         # must show Isaac Sim 6.x found and extensions linked
```

`doctor` currently reports two outstanding items — `isaac_core` not yet installed into
Isaac's interpreter, and the extensions not yet linked. `setup.sh` fixes both. Do that
first, or the `isaac_core_ogn.*` nodes will not appear in the GUI node search at all.

## After Stage 1

Tell me and I will write `usd/layers/camera_udp/layer.toml` binding config to the prim
paths above, then finish the Isaac-coupled runtime — the `StageInspector`
implementation, stage composition and the step loop — against a stage that provably
exists. That ordering matters: I do not want to write a composer against imagined prim
paths.
