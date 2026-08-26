# Documentation index

## [first_run.md](first_run.md)

Step-by-step guide for the first end-to-end flight using the UDP camera layer. Covers setup verification, stage composition in the GUI, and flying a trajectory with `send_test_pose.py`. Start here after installation.

## [usd_build_sheet.md](usd_build_sheet.md)

Task list for authoring and wiring USD in the Isaac Sim 6 GUI. Describes the remaining wiring gap (ROS publisher execution input) and records settled decisions about prim targeting, mount points, and body-axis conventions.

## [ros2_and_python.md](ros2_and_python.md)

Explains why `rclpy` cannot be imported inside Isaac Sim 6 (Python 3.12 vs Humble's 3.10 C extensions), what was removed, what replaces it (Isaac's C++ bridge nodes), and how the test guard prevents regression. Read this before adding any ROS-related code to an extension.
