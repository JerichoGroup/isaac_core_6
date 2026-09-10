# USD task list

Tactical tasks only. Rationale lives in [docs/development-log.md](../docs/development-log.md).

---

## Decisions needed from you

Nothing below can be actioned until these are answered.

### 1. Delete the relationship-binding machinery?

Added then abandoned — writing a relationship onto a live OmniGraph node aborts Kit. Unused.

---

## Tasks

## Screenshots for the README

Done: `hero_terrain.png`, `gimbal_reference.png`, `bboxes.png`, `pose_sender.png`,
`ros_topics_rqt.png`. All five are referenced and resolve.

Dropped: the distance-sensor shot. A number next to a picture is not worth an image.

### Outstanding — `swarm_two_vehicles.png` (optional)

Blocked on the RTSP port collision below, and worth knowing before retrying: there is now only **one
viewport**, because each camera renders to an offscreen render product rather than its own viewport
window. So a single screenshot cannot show two camera views. The shot to take is the **Stage tree**
showing `/World/Environment/lead` and `/World/Environment/wing` alongside the viewport, or skip it --
the multi-vehicle table in the README carries the information without a picture.

## How to run

```bash
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash   # REQUIRED for the bbox layer
isaac-core run --config your.toml
```

```toml
[features]
enabled = ["camera_udp", "bbox"]
```

Build the message package once, if you have not:

```bash
cd ~/IsaacSim-ros_workspaces/humble_ws && colcon build --packages-select isaac_core_ros2_msgs
```

Check what loaded — expect `global_pose`, `image_rgb`, `bbox`:

```bash
ros2 topic list | grep isaac_core
```

---

## Do not author these

The compositor writes them at launch. Leave unauthored on `bbox_projector`:

| Input | Value at launch |
|---|---|
| `inputs:cameraPath` | `/World/Environment/drone_0/Xform/main_camera_01` |
| `inputs:targetsRootPath` | `/World/bboxes` |
| `inputs:topicName` | `/isaac_core/bbox` |

Boxes come from Isaac's annotators, measured off the render, so there are no camera values to
author at all. Semantic labels are applied automatically to every child of `/World/bboxes` at
launch — do not add them by hand.

---

## Conventions

- Author only inside `/Root`; it is the `defaultPrim`. A prim declared as a **sibling** of `/Root`
  is silently dropped when the layer is referenced.
- Body axes in `/Root/Xform`: **+X nose, +Y left wing, +Z up**.
- Camera orientation: `xformOp:orient = (0.5, 0.5, -0.5, -0.5)`.
- A sensor's ray fires along its own **−Z**.
- Layers mount at `/World/Environment/{instance}`.
- No Cesium Globe Anchor on the camera Xform; Globe Anchors are for bbox targets.
- `ros2_context` and `read_sim_time` take **no** `execIn`. Anything recomputing per frame does.

## Gotchas

- After adding or renaming an OGN node or its attributes:
  `rm -rf ~/.cache/ov/ogn_generated/*/isaac_core_ogn.*`
- ROS topics take several seconds to appear (graph ticks + DDS discovery).
- `omni.graph.action.OnTick` does not fire unless the timeline is playing.
- Clear crash debris between runs: `rm -rf /tmp/carb.*`
