# isaac_core_ogn.sensors

Physics-based sensing nodes for Isaac Core action graphs.

| Node | Purpose |
|---|---|
| `DistanceSensor` | Raycast from a prim and report the distance to the first hit, shaped for `sensor_msgs/Range` |

## Why a custom node

Isaac Sim 6 ships a `RaycastSensor`, but it is a Python runtime class rather than an
OmniGraph node or a USD schema, so it cannot be placed in an action graph. This node uses
the PhysX scene-query interface Isaac exposes for exactly this purpose, which keeps the
whole sensing pipeline inside the graph like every other feature layer.

All interpretation of the reading (out-of-band handling, `sensor_msgs/Range` conventions)
lives in `isaac_core.contracts.rangefinder`, unit-tested without Isaac Sim. This node only
performs the raycast and passes the result through.
