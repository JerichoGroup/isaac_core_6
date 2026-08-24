# isaac_core_ogn.math

OmniGraph nodes for pure math operations: LLA-to-ENU coordinate conversion, rotation
composition with configurable frame (world/body), gimbal offset application.

## Nodes

- **GlobalPositionToLocalPosition** — Convert global LLA + ENU orientation to local ENU
  position and quaternion, applying gimbal offsets.
