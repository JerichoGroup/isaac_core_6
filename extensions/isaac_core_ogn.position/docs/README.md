# isaac_core_ogn.position

OmniGraph nodes that read global position data from external sources and output
LLA + ENU orientation for downstream processing.

## Nodes

- **UdpToGlobalPosition** — Listen on a UDP port for the 51-byte pose packet and output
  global position and orientation in ENU.
- **Ros2ToGlobalPosition** — Subscribe to ROS 2 NavSatFix and PoseStamped topics and output
  global position and orientation in ENU.
