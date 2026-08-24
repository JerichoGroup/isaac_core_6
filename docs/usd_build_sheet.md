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
then enable **Isaac Core Position**, **Isaac Core Math** and **Isaac Core Sensors**
in the extension manager so they appear in the node search.

| Prim name | Node type |
|---|---|
| `on_playback_tick` | `omni.graph.action.OnPlaybackTick` |
| `ros2_context` | ROS 2 Context (from `isaacsim.ros2.bridge`) |
| `udp_to_global_position` | `isaac_core_ogn.position.UdpToGlobalPosition` |
| `gimbal` | `isaac_core_ogn.sensors.Ros2Gimbal` |
| `global_to_local` | `isaac_core_ogn.math.GlobalPositionToLocalPosition` |
| `global_pose_publisher` | `isaac_core_ogn.sensors.Ros2GlobalPosePublisher` |
| `translate_writer` | `omni.graph.nodes.WritePrimAttribute` |
| `orient_writer` | `omni.graph.nodes.WritePrimAttribute` |

Wiring — execution first, every node ticked from the same source:

```
on_playback_tick.tick  ──►  udp_to_global_position.execIn
                       ──►  gimbal.execIn
                       ──►  global_to_local.execIn
                       ──►  global_pose_publisher.execIn
                       ──►  translate_writer.execIn
                       ──►  orient_writer.execIn
```

Then data:

```
udp_to_global_position.global_position     ──►  global_to_local.global_position
udp_to_global_position.global_orientation  ──►  global_to_local.global_orientation

gimbal.roll   ──►  global_to_local.offset_roll
gimbal.pitch  ──►  global_to_local.offset_pitch
gimbal.yaw    ──►  global_to_local.offset_yaw

global_to_local.local_position     ──►  translate_writer.value
global_to_local.local_orientation  ──►  orient_writer.value

global_to_local.global_position     ──►  global_pose_publisher.global_position
global_to_local.global_orientation  ──►  global_pose_publisher.global_orientation
```

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
| `ros2_image_publisher` | `isaac_core_ogn.sensors.Ros2ImagePublisher` |

Chain them in that order, feed `set_camera` the path
`/Root/Xform/main_camera_01`, and feed `get_render_product`'s render product path into
`ros2_image_publisher.renderProductPath`.

Two things I could not verify without running it, so treat as the risk area:

1. The Isaac Sim 6 Replicator writer name our image publisher requests. It was
   confirmed by reading `OgnROS2CameraHelper.py` in your install, but never executed.
2. Whether **multi-tick rendering** (new in 6.0) supersedes the hand-rolled publish
   rate gate. If it does, the `publishRateHZ` input becomes redundant and scheduling
   moves to the render product. Worth checking on first launch.

---

## Stage 3 — ROS pose source

`usd/layers/camera_ros/camera_ros.usda`: identical to `camera_udp` except
`udp_to_global_position` is replaced by
`isaac_core_ogn.position.Ros2ToGlobalPosition`, which takes `lla_topic` and
`orientation_topic` instead of `udp_port`.

The **easiest correct way** to make this: open `camera_udp.usda`, delete the UDP
node, add the ROS node, rewire its two outputs, and Save As. Everything else is
unchanged.

Important asymmetry, and it is deliberate: the UDP path converts NED→ENU, the ROS
path does **not**, because MAVROS already publishes ENU. Getting that wrong silently
mirrors the aircraft. It is handled inside the nodes, so nothing for you to wire — but
worth knowing when a heading looks wrong.

---

## Stage 4 — sensor layers

One directory each, mounted under `/Environment/<instance>`:

- `usd/layers/distance_sensor/` — an ActionGraph with a raycast range finder feeding
  `isaac_core_ogn.sensors.Ros2RangePublisher`. Isaac Sim 6 ships a **physics raycast
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
