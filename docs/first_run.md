# First run

Getting the UDP camera flying over terrain. This proves the whole chain — extensions
load, the pose pipeline computes, the camera moves, Cesium streams — before any of the
Python runtime exists to automate it.

The UDP path needs **no ROS 2 at all**, which is why it comes first.

There is no `isaac-core run` yet, so composition is manual for now. That is deliberate:
I want to write the composer against a stage that has provably worked.

---

## 1. Finish setup

```bash
cd ~/clones/isaac_core_6
isaac-core doctor
```

Two items were still outstanding last time, and both matter:

```
[WARN] Cannot confirm isaac_core is installed in Isaac's interpreter
[FAIL] Extensions not linked
```

Fix both:

```bash
/home/ofer/isaacsim/python.sh -m pip install -e ".[sim]"
./scripts/link_extensions.sh
```

The first is not optional. Our nodes do `from isaac_core.geo import ...`, so without it
they will fail to import exactly the way the rclpy problem did. The second creates the
`extsUser` symlinks — and clears the stale `isaac_core_ogn.sensors` link, which no
longer exists.

Re-run `isaac-core doctor` and confirm both are `[PASS]`. `rclpy not importable` stays a
warning and is fine — nothing in this run needs it.

---

## 2. Set a default prim on both camera layers

**Do this before composing, or the reference will bring in nothing.**

Neither camera layer declares a `defaultPrim`. When you add a *reference* to a layer,
USD needs to know which prim to pull in, and with no default it pulls nothing —
silently. `earth.usda` has `defaultPrim = "World"`; the camera layers have none.

In the GUI, per layer: open the file, select `/Root` in the Stage tree, right-click →
**Set as Default Prim**, save.

Worth knowing what this buys: `Root`'s siblings in those files (`Cesium`,
`CesiumGeoreference`, `PhysicsScene`) are then *not* dragged in by the reference, which
is what you want — `earth.usda` supplies its own, and two georeferences on one stage
would fight.

---

## 3. Point Cesium at your tile server

`earth.usda` currently has `cesium:url = http://127.0.0.1:8088/nablus/tileset.json`.
Change it to your real server, or start a tile server on that port. Without terrain
you will still see the camera move, just against nothing — which is a perfectly valid
first test if the server is inconvenient.

---

## 4. Compose the stage

```bash
/home/ofer/isaacsim/isaac-sim.sh
```

Then:

1. **File → Open** `usd/scenes/earth.usda`
2. Select `/World/Environment`, **Create → Xform**, rename it `drone_0`
   (`drone_0` is the default vehicle id in config, so the layer manifest will mount here)
3. With `/World/Environment/drone_0` selected: **Add → Reference**, choose
   `usd/layers/camera_udp/camera_udp.usda`
4. Confirm the tree now shows `/World/Environment/drone_0/Xform/main_camera_01` and the
   two OmniGraphs beneath `drone_0`

If step 3 produces an empty prim, step 2 of this guide was skipped.

---

## 5. Look through the camera

In the viewport camera dropdown, pick `main_camera_01`. Or select the camera prim and
use the viewport's "look through selected" control.

---

## 6. Play, then fly

Press **Play** first. The graph is driven by `OnPlaybackTick`, so nothing computes while
the timeline is stopped — including the UDP socket bind. A sender started before Play
just sends into a closed port.

Then, in a terminal:

```bash
cd ~/clones/isaac_core_6
PYTHONPATH=src ./scripts/send_test_pose.py hold
```

The camera should jump to 32.22481, 35.25621 at 1000 m, pitched 30° down. Once that
works:

```bash
PYTHONPATH=src ./scripts/send_test_pose.py orbit --radius-m 800 --duration-s 60
PYTHONPATH=src ./scripts/send_test_pose.py path --speed-mps 50
```

`--help` lists every option. The script runs on system Python, needs no ROS, and drives
`isaac_core.vehicle` → `isaac_core.protocol` → `isaac_core.devkit.transport`, so a
moving camera also proves that whole slice of the kernel.

---

## Why the defaults should just work

Nothing needs configuring on the nodes for this run, because the `.ogn` defaults already
agree with the scene:

| Setting | Default | Matches |
|---|---|---|
| `udp_to_global_position.udp_port` | `33333` | the sender's default |
| `global_to_local.enu_reference` | `[32.22481, 35.25621, 516.7]` | `earth.usda`'s Cesium georeference origin |
| `global_to_local.rotation_frame` | `body` | gimbal convention (D14) |

Those three agreeing is what makes this a one-command test. If you change the scene's
georeference, change `enu_reference` to match or the camera and terrain will disagree
about where they are.

---

## If it does not work

**Nodes missing from the node search.** Extensions not linked, or not enabled. Check
`extsUser`, then Window → Extensions and enable *Isaac Core Math* and *Isaac Core
Position*. Watch the console during startup for import errors.

**`ModuleNotFoundError: isaac_core`.** Step 1's `pip install -e ".[sim]"` into Isaac's
interpreter was skipped.

**Nothing moves at all.** Check in order: is the timeline playing; is the sender running
without errors; is `udp_port` really 33333 on the node; is a firewall in the way. Prove
packets are arriving independently with
`sudo tcpdump -i lo -n udp port 33333` — one 51-byte packet per tick.

**Camera moves but terrain is absent.** Cesium URL or tile server. Independent of the
pose pipeline, so the run is still a success.

**Camera moves the wrong way.** Note the sign conventions: the wire is **NED**, so
`+pitch` raises the nose and `--pitch-deg -30` looks *down*. `UdpToGlobalPosition`
converts NED→ENU internally. If the aircraft appears mirrored, that conversion is the
place to look, and `isaac_core.geo.ned_to_enu` has the tests for it.

**Camera drifts away from the terrain as it moves.** The Cesium Globe Anchor on
`/Root/Xform`, or an `enu_reference` that disagrees with the scene georeference.

---

## After it works

Tell me and I will write:

1. `usd/layers/camera_udp/layer.toml` — binding config to the prim paths that now
   provably exist, so `udp_port`, `enu_reference` and the topic names come from config
   instead of being baked into the USD.
2. The Isaac-coupled runtime — the `StageInspector` implementation, stage composition and
   the step loop — which turns every manual step above into `isaac-core run`.
3. Stage 4 sensor layers, specified against a stage that has actually launched.
