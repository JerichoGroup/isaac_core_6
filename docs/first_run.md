# First run

The first flight, with something to check at every step so you find out *where* it went wrong rather
than just that it did.

The README's [Running the simulation](../README.md#running-the-simulation) section is the short
version. This is the same path with the verification steps filled in.

## 1. Check the environment before launching anything

```bash
isaac-core doctor
```

This is the fastest way to catch the two things that most often go wrong: an Isaac Sim install
somewhere the probe does not look, and the package not installed into Isaac's own interpreter. Each
failure line prints the exact command that fixes it.

**Expect:** every check `OK`, and a reported Isaac Sim version of 6.0.1 or newer.

If it reports the package missing from Isaac's Python:

```bash
/path/to/isaacsim/python.sh -m pip install -e ".[sim]"
```

## 2. Make sure you have terrain to look at

The scene streams Cesium 3D Tiles from a URL. Check yours is reachable **from this machine**:

```bash
curl -sI "$(isaac-core config dump | grep tileset_server_url | cut -d'"' -f2)/nablus/tileset.json" | head -1
```

**Expect:** `HTTP/1.1 200 OK`.

If you get nothing or a 404, fix it before launching — an unreachable tileset produces a scene that
runs perfectly and shows no ground, with no error anywhere. Set it with:

```bash
isaac-core run --set cesium.tileset_server_url http://your-server:8088
```

Without any tile server you can still fly and still get an image topic and RTSP stream; there will
just be no terrain under the camera.

## 3. Launch

```bash
isaac-core run
```

Startup takes 15–30 seconds. Watch for three lines in the log:

```
control plane listening on 127.0.0.1:8760
Feature layers:
  ✓ camera_udp [drone_0]
viewport looking through /World/Environment/drone_0/Xform/main_camera_01
simulation running
```

- **No `✓ camera_udp`** — the layer was not composed. A skipped layer is listed with the reason.
- **`control plane listening` but no `simulation running`** — composition failed. The exception is
  logged through our own logger before being re-raised, so the reason is in the output.
- **Roughly one launch in three segfaults** inside Kit's own `update_app()`. It is not your setup; it
  reproduces with all our extensions disabled. Relaunch, and `rm -rf /tmp/carb.*` if it is frequent.

The default run enables the UDP camera only, so at this point you have one camera, one pose input on
port 33333, and an RTSP stream.

## 4. Send a pose

The camera sits at the ENU origin until something tells it where to go. In a second terminal:

```bash
PYTHONPATH=src ./scripts/send_test_pose.py hold
```

That sends the scene's reference point at 1000 m, 30 times a second, until you stop it.

**Expect:** the viewport shows terrain from 1000 m up.

Only one process can own the UDP port. If an earlier sender or a pose-sender GUI is still running, it
will win and your new sender will appear to do nothing:

```bash
pgrep -af 'send_test_pose|pose_sender' || echo "no senders running"
```

## 5. Confirm the pose is really arriving

This is the step worth not skipping, because the picture can lie to you — the viewport may be showing
Kit's default camera rather than the vehicle's.

```bash
isaac-core-inspect --poll 0.5
```

```
    #          Translate (x, y, z)               Quaternion (w, x, y, z)
    1  T=(     0.000,      0.000,    483.300)  Q=(0.6830, -0.1830, 0.1830, 0.6830)
```

**Expect** `Translate z` to equal your altitude minus the scene's reference altitude:
1000 − 516.7 = 483.3. This reads the live prim transform through the control plane, so if it changes
as you fly, the whole chain works: packet decoded, geodesy computed, prim written.

If translate stays at zero, nothing is arriving on the UDP port. If it changes but the window does
not, the window is looking through a different camera — set `sim.viewport_camera`.

## 6. Fly

```bash
PYTHONPATH=src ./scripts/send_test_pose.py orbit --radius-m 800 --duration-s 60
PYTHONPATH=src ./scripts/send_test_pose.py path --speed-mps 50
```

Frame rate will dip the first time over fresh terrain, because tiles are being downloaded. Measured on
the reference machine: 7 fps on the first pass, 59.9 on the second over the same ground. Fly a route
once to warm the cache before anything that matters, and leave
`cesium.delete_cache_on_launch = false`.

## 7. Check the ROS topics, if you use ROS

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep isaac_core
ros2 topic hz /isaac_core/image_rgb
```

**Expect** `/isaac_core/global_pose` and `/isaac_core/image_rgb`, and roughly the render rate on the
image topic.

Give DDS several seconds after `simulation running` before concluding a topic is missing. If topics
never appear, confirm `ROS_DOMAIN_ID` matches your other nodes — it is inherited from the environment
unless you pin `ros2.domain_id`.

## 8. Try the video stream

```bash
ffplay rtsp://127.0.0.1:8554/stream
```

Always on, no flag. If you get `Address already in use` in the simulator log instead, a simulator from
an earlier session is still holding the port — Isaac ignores `SIGTERM`, so it survives an ordinary
kill:

```bash
pgrep -af 'isaac_core.sim' && kill -9 <pid>
```

## Where to go next

- Turn on more features: `--set features.enabled '["camera_udp","distance_sensor","bbox"]'`
- Drive it from your own script instead of the test sender: see
  [Scripting with the devkit](../README.md#scripting-with-the-devkit).
- Add a sensor of your own: [authoring_layers.md](authoring_layers.md).
