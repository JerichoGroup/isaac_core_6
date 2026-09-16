# Authoring a feature layer

How to add a sensor, publisher or anything else to the simulation **without forking
`isaac_core`**. This is the extension seam; if you find yourself editing `isaac_core` to add a
feature, something here is missing and it is worth saying so.

## What a layer is

A directory with a manifest and, usually, a USD file:

```
my_layers/thermal_cam/
├── layer.toml          the manifest: what it needs, what it gives, how config reaches prims
└── thermal_cam.usda    the USD: prims and the OmniGraph that does the work
```

Point the simulator at it and enable it:

```toml
[assets]
layer_search_paths = ["/home/you/my_layers"]

[features]
enabled = ["camera_udp", "thermal_cam"]
```

That is the whole integration. Nothing in `isaac_core` changes.

## The manifest

```toml
id = "thermal_cam"
usd = "thermal_cam.usda"

# What must already exist on the stage. May be satisfied by the stage itself or by another
# layer's `provides`, so ordering is resolved for you.
requires = ["CAMERA"]

# What this layer contributes, for other layers to require.
provides = ["THERMAL"]

# Config -> prim attribute. This is wiring only: never put values here.
[[bindings]]
prim = "{mount}/ThermalExport/ros2_publisher"
attribute = "inputs:topicName"
resolve = "thermal_topic"

[[bindings]]
prim = "{mount}/ThermalExport/thermal_node"
attribute = "inputs:gain"
config = "layers.thermal_cam.gain"
```

Two ways to fill an attribute:

- `config = "dotted.key"` — read straight from the resolved config.
- `resolve = "name"` — computed by a named resolver in `sim/configurator.py`, for anything that
  depends on the vehicle (ports offset per vehicle, namespaced topics, mount paths).

`{mount}` expands to the vehicle's prim mount, e.g. `/World/Environment/drone_0`, which is what makes
a layer work unchanged in a swarm.

Your own settings live under `[layers.<your_id>]` in config.

**Every `resolve` name must exist.** A test scans every shipped manifest and fails if one names a
resolver that does not — added after a typo in a resolver name silently broke every launch.

## The USD side

USD is authored in the Isaac Sim GUI, not by hand. The pattern that works:

1. Open your layer file as its own stage.
2. Build the prims and the OmniGraph under a single root Xform.
3. Set that root as the **default prim**. Composition references the default prim; without one the
   layer contributes nothing and reports no error.
4. Keep the graph's nodes named exactly as your manifest's `prim` paths expect.

Things that will cost you an afternoon otherwise:

- **A declared-but-unconnected input silently uses its default.** Three separate bugs in this repo
  were exactly that. If a value must come from config, bind it in the manifest *and* check it arrives.
- **Renaming an `.ogn` attribute leaves the old one authored in the USD.** Inert, but it shows up as a
  duplicate field in the GUI, and if the *type* changed the node will not instantiate at all.
- **A new or renamed `.ogn` needs its cache cleared**: `rm -rf ~/.cache/ov/ogn_generated/*/isaac_core_ogn.*`.
  `scripts/link_extensions.sh` does this for you.
- **`OnTick` does not fire unless the timeline is playing.**


## A worked USD example

There is no substitute for authoring in the GUI, but here is the shape to build, and why each part is
where it is.

```
/Root                                       default prim -- composition references THIS
├── /Root/Xform                             prims that must MOVE with the vehicle
│   └── /Root/Xform/thermal_camera_01       your camera, sensor, or mesh
└── /Root/ThermalExport                     your OmniGraph, a SIBLING of Xform
    ├── on_playback_tick                    fires only while the timeline plays
    ├── thermal_node                        your compute node
    └── ros2_publisher                      one of Isaac's generic bridge nodes
```

Why the graph is *not* under `Xform`: `Xform` carries the vehicle's transform, and a graph has no
transform to inherit. Every layer this repo ships is built this way -- `camera_udp` has
`/Root/CameraImageExport` and `/Root/PoseSync` beside `/Root/Xform`, and `bbox` has
`/Root/BboxExport`. Putting a graph inside `Xform` is not fatal, but then your manifest paths must say
`{mount}/Xform/ThermalExport/...`, and you lose the symmetry with everything else.

Building it, step by step in the GUI:

1. **File > New**, then create an Xform named `Root` at the stage root.
2. Select `Root` and **set it as the default prim** (right-click > Set as Default Prim). Skipping this
   is the single most common reason a layer composes to nothing with no error.
3. Add a child Xform named `Xform` under `Root`, and put anything that must move with the aircraft
   inside it -- it inherits the vehicle's transform once mounted.
4. If the feature computes something, add an **Action Graph** directly under `/Root`, beside `Xform`,
   named to match what your manifest expects. Inside it, start with `on_playback_tick`, not `on_tick`.
5. Save as `<layer_id>.usda` next to your `layer.toml`.

At runtime that becomes, for a vehicle called `drone_0`:

```
/World/Environment/drone_0/Xform/thermal_camera_01
/World/Environment/drone_0/ThermalExport/thermal_node
```

The second is exactly what `{mount}/ThermalExport/thermal_node` resolves to in the manifest. The same
file serves every vehicle in a swarm, because only the mount differs and nothing inside the layer is
absolute.

### Checking the composition before you write any Python

```bash
isaac-core run --set features.enabled '["camera_udp","thermal_cam"]' \
               --set assets.layer_search_paths '["/home/you/my_layers"]'
```

The startup report lists what composed and what was skipped. Then confirm your values actually
arrived, rather than trusting the log:

```bash
isaac-core-inspect
```

If a binding silently did nothing, the attribute will still hold its `.ogn` default -- which is the
failure mode worth being paranoid about, because it looks like success.

## If your layer needs Python

Write an OmniGraph node. Copy `extensions/_template`, which is a working node with its `.ogn`,
extension manifest and test hooks in place.

Keep the node thin. Every node we ship is an adapter: it reads inputs, calls a pure function in
`isaac_core.geo`, `isaac_core.protocol` or `isaac_core.contracts`, and writes outputs. That is what
makes the logic testable without launching Isaac, and it is worth copying.

Do **not** import `rclpy` in a node. It cannot be imported inside Isaac's interpreter. Publish by
wiring your node's output into one of Isaac's generic ROS 2 bridge nodes — see
[ros2_and_python.md](ros2_and_python.md).

A layer that ships inside an installed package still hands over a directory: discovery scans paths, so
put the package's layer directory on `assets.layer_search_paths`. Use `importlib.resources` when you do
not know the install prefix:

```python
from importlib.resources import files

Sim.launch(overrides={"assets.layer_search_paths": [str(files("my_package") / "layers")]})
```

## Checking it worked

```bash
isaac-core run --set features.enabled '["camera_udp","thermal_cam"]'
```

The startup report lists every layer it composed, and every layer it skipped with the reason. A layer
that does not appear was not found; a layer listed as skipped had an unsatisfied `requires`.

Then confirm the values actually arrived, rather than trusting the log:

```bash
isaac-core-inspect
```

## Gotchas that are not your fault

- ROS topics take several seconds to appear after the control plane answers; that is DDS discovery.
- Roughly one launch in three segfaults inside Kit's own `update_app()`. It reproduces with all our
  extensions disabled. `rm -rf /tmp/carb.*` makes it less frequent.
- If two layers write the same prim attribute, the second wins and a warning is logged. Prefer
  distinct prims.
