# ROS 2 and the Python version wall

**Our Kit extensions must never import `rclpy`.** This is not a style preference or a
configuration issue — it is a hard ABI incompatibility with no workaround.

## The constraint

| | Python | rclpy C extension |
|---|---|---|
| Isaac Sim 6.0.1 (bundled) | **3.12.13** | needs `cpython-312` |
| ROS 2 Humble (system) | **3.10.12** | ships `_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so` |

C extension ABIs are not compatible across Python minor versions. So `import rclpy`
inside Isaac Sim 6 fails:

```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/humble/lib/python3.10/site-packages/_rclpy_pybind11.cpython-312-x86_64-linux-gnu.so' isn't present
```

Note what rclpy is doing there: it is looking for a `cpython-312` build because the
*interpreter* is 3.12, and only a `cpython-310` build exists. OmniGraph then retries
the node file three times and abandons it, taking the extension's node definitions
with it.

The previous generation of this tooling used `rclpy` in its nodes freely, and that
worked only because Isaac Sim 2023.1.1 also bundled Python 3.10, matching Humble
exactly. Isaac Sim 6 moving to 3.12 breaks every one of those nodes.

## Why the "just source ROS first" reflex does not help

Sourcing `/opt/ros/humble/setup.bash` puts the 3.10 packages on `PYTHONPATH`, which
is precisely how Isaac's 3.12 interpreter finds `rclpy` and then fails on its C
extension. Sourcing makes the error *possible*, not fixable.

Options that genuinely would work, and why we rejected them:

- **Build ROS 2 from source against Python 3.12.** Large, fragile, and every team
  member would have to reproduce it.
- **Use a ROS 2 distro built for 3.12** (Jazzy on Ubuntu 24.04). A whole-OS upgrade
  and a distro migration, and it would still break the next time Isaac moves Python.
- **Vendor a 3.12 rclpy build.** Same fragility, now ours to maintain.

## What we do instead

Isaac Sim 6's own ROS 2 bridge is **C++**, with zero `rclpy` imports — verified by
grepping `isaacsim.ros2.bridge` in the install. C++ does not care about the Python
ABI, so the bridge works regardless of which Python Isaac ships.

So the division of labour is:

| Where | Python | Does what |
|---|---|---|
| Inside Isaac Sim | 3.12 | **Pure computation only.** Our nodes: geodesy, rotation composition, UDP decoding. No ROS. |
| Isaac's C++ bridge | n/a | **All ROS 2 publish/subscribe**, wired in the graph. |
| Host / devkit / sidecar | 3.10 | `rclpy` freely — recording, MAVROS interaction, scripting. |

This is a better architecture than the old repo's, and it arrives for free: it forces
our nodes to be the thin compute adapters they were always supposed to be, and it
deletes an entire class of the old repo's defects — rclpy lifetime management,
per-node executors, and background spin threads inside OmniGraph nodes — by removing
the code rather than fixing it.

It also vindicates the devkit/sidecar split. Anything that genuinely needs `rclpy`
runs out-of-process on the system Python, where Humble works perfectly.

## What was removed, and what replaces it

Five nodes were deleted. Every one has a native Isaac Sim 6 equivalent:

| Removed (ours) | Use instead (Isaac's, C++) |
|---|---|
| `isaac_core_ogn.position.Ros2ToGlobalPosition` | two × `isaacsim.ros2.bridge.ROS2Subscriber` — `sensor_msgs/msg/NavSatFix` and `geometry_msgs/msg/PoseStamped` |
| `isaac_core_ogn.sensors.Ros2GlobalPosePublisher` | `isaacsim.ros2.bridge.ROS2Publisher` — `geographic_msgs/msg/GeoPoseStamped` |
| `isaac_core_ogn.sensors.Ros2RangePublisher` | `isaacsim.ros2.bridge.ROS2Publisher` — `sensor_msgs/msg/Range` |
| `isaac_core_ogn.sensors.Ros2Gimbal` | `isaacsim.ros2.bridge.ROS2Subscriber` — our `Gimbal` message |
| `isaac_core_ogn.sensors.Ros2ImagePublisher` | `isaacsim.ros2.bridge.ROS2CameraHelper` or `ROS2PublishImage` |

Every ROS *wrapper* node is gone, because the generic bridge nodes replace them. The
`isaac_core_ogn.sensors` extension itself was later repurposed for sensors that compute something
the bridge cannot, so seven nodes ship today across three extensions:

| Extension | Node | What it computes |
|---|---|---|
| `isaac_core_ogn.math` | `GlobalPositionToLocalPosition` | LLA → local ENU + quaternion |
| `isaac_core_ogn.math` | `EulerToQuaternion` | roll/pitch/yaw → quaternion |
| `isaac_core_ogn.math` | `QuaternionToEuler` | quaternion → roll/pitch/yaw |
| `isaac_core_ogn.math` | `SecondsToRosStamp` | seconds → ROS `sec`/`nanosec` |
| `isaac_core_ogn.position` | `UdpToGlobalPosition` | UDP pose packets → LLA + orientation |
| `isaac_core_ogn.sensors` | `DistanceSensor` | render raycast → range in metres |
| `isaac_core_ogn.sensors` | `BboxProjector` | Isaac's bbox annotators → flat parallel arrays |

All seven are computation only. They delegate to `isaac_core.geo`, `isaac_core.protocol` and
`isaac_core.contracts`, and publish nothing themselves — a bridge node does that. There is also
`extensions/_template` to copy when adding your own.

## Using the generic bridge nodes

`ROS2Publisher` and `ROS2Subscriber` are message-agnostic. They take:

- `messagePackage` — e.g. `sensor_msgs`
- `messageSubfolder` — e.g. `msg`
- `messageName` — e.g. `NavSatFix`

and then expose that message's fields as dynamic node attributes you wire like any
other. Custom messages work the same way, provided the message package is built in a
ROS workspace the bridge can see — the same requirement the old repo had for its
`Gimbal` and `SATOutput` messages.

## The guard

`tests/unit/extensions/test_no_python_ros2.py` fails if any extension module imports
`rclpy`, `sensor_msgs`, `geometry_msgs`, `geographic_msgs`, `std_msgs`,
`builtin_interfaces`, `rosidl_runtime_py` or `cv_bridge`. It reads source as text, so
it runs with neither Isaac Sim nor ROS 2 present, and it names the offending file and
import.

That test exists because this failure is expensive to rediscover: it does not appear
until an extension is linked into a real Isaac Sim install and the GUI is launched.
