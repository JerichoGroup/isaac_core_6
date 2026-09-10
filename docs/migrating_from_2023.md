# Migrating from the 2023 repo

For people who used the previous generation (`isaac_core_2023`). If you are new, you do not need this.

## What stayed the same

**The UDP pose packet is byte-identical.** Same 51 bytes, same field order, same XOR checksum, same
default port. An existing sender works against this repo with no change, and a test asserts encoding
against bytes captured from the 2023 encoder.

The angle conventions are unchanged: radians on the wire, NED, and the same NED→ENU mapping.

## What changed, and why

### One config file instead of command-line flags

2023 selected features with flags (`--camera-udp`, `--sat`, `--bbox`) and carried a `LAUNCH_CONFIG`
environment variable holding JSON. Everything is now in one TOML file with a documented schema,
resolved through a fixed precedence chain, and validated at load.

```bash
# 2023
./run_sim.sh --camera-udp --bbox

# now
isaac-core run --set features.enabled='["camera_udp","bbox"]'
```

`isaac-core config dump` shows the fully resolved result and `isaac-core config explain <key>` says
which source won.

### No Docker

2023 shipped base and simulation images and a Docker workflow. This repo runs against a local Isaac
Sim install, located and validated by `isaac-core doctor`. There is no container path.

### Features are layers, not code branches

Adding a sensor in 2023 meant editing the simulation app. Now a feature is a directory with a
manifest, discovered from `assets.layer_search_paths` or a `isaac_core.layers` entry point, and
composed by dependency order. Adding one requires no change to `isaac_core`.

### The geodesy left the OmniGraph nodes

In 2023 the LLA→ENU maths lived inside the nodes, so exercising it meant launching the simulator. It
now lives in `isaac_core.geo` and the nodes are thin adapters. That is why the test suite runs with no
GPU, no Isaac Sim and no ROS 2.

### ROS wrapper nodes are gone

2023 shipped custom nodes wrapping ROS publishers (`Ros2RangePublisher`, `Ros2ImagePublisher`,
`Ros2Gimbal`, and one more). Isaac Sim 6's generic bridge nodes replace all of them, so what we ship
now is only computation. Details in [ros2_and_python.md](ros2_and_python.md).

### RTSP is native, and RTP is gone

2023 pushed frames out over RTP through a sidecar process. Isaac Sim 6 has a native RTSP server, so
every camera streams H.264 with no separate process and no flag. If you consumed the RTP stream, move
to `rtsp://127.0.0.1:8554/stream`.

### `--sat` became `capture_frame`

Taking a picture was a ROS topic in 2023. It is now a control-plane command, because a capture is a
command rather than telemetry — and unlike 2023 it can capture at a resolution independent of the
window.

```python
session.capture_frame("shot.png", width=3840, height=2160)
```

### Multiple vehicles

2023 was single-vehicle. Declaring several now gives each its own camera, UDP port, topic namespace,
prim mount and RTSP stream. A single-vehicle configuration produces byte-identical output to before,
so nothing existing is affected.

`set_gimbal` and `capture_frame` are still single-vehicle and refuse when several are configured,
rather than silently acting on the first. Tracked in [roadmap.md](dev/roadmap.md).

### The tileset URL

2023 built the tileset URL as `f"{base}/{prim.GetName()}/tileset.json"`, which assumes the prim's name
matches its path segment. Where that is not true it 404s and you get no terrain and no error. The URL
is now configured directly:

```toml
[cesium]
tileset_server_url = "http://your-server:8088"
```

## Behaviour worth re-testing after you move

- **Gimbal axes.** 2023 had a roll/pitch swap for a level, north-heading aircraft. It is fixed here,
  which means if you compensated for it in your own code, remove the compensation. `+pitch` raises the
  look direction, `+roll` drops the right side, `+yaw` turns right.
- **Bounding box messages.** The message is `isaac_core_ros2_msgs/msg/FrameBboxes`, carrying **parallel
  arrays** rather than an array of per-object messages, because Isaac Sim 6 cannot publish a nested
  message array. Index `i` is the same object in every array.
- **Out-of-range distance readings** saturate at the rated limits rather than reporting infinity, so a
  consumer casting to `int` no longer crashes when the sensor sees nothing.
