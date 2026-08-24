# isaac_core_ogn.sensors

OmniGraph nodes for publishing sensor data over ROS 2.

## Nodes

- **Ros2GlobalPosePublisher** — Publish global pose (LLA + orientation) to a
  GeoPoseStamped topic.
- **Ros2RangePublisher** — Publish distance sensor data as sensor_msgs/Range.
- **Ros2ImagePublisher** — Write raw RGB from a render product and republish with
  rate limiting and frame counters.
- **Ros2Gimbal** — Subscribe to a gimbal topic and output roll/pitch/yaw offsets
  in degrees, using start values until the first message arrives.
