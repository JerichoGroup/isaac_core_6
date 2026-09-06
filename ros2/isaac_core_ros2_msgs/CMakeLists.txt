cmake_minimum_required(VERSION 3.8)
project(isaac_core_ros2_msgs)

# Only FrameBboxes ships: it carries parallel arrays rather than a Bbox[], because Isaac Sim 6
# cannot publish an array of nested messages. A standalone Bbox message would have no
# publisher or consumer, so it is not shipped.
#
# This package generates message interfaces only -- no C++ targets -- so it needs no language
# standard settings or compile options.
#
# It deliberately does NOT reuse the name isaac_ros2_messages. That name belongs to NVIDIA's
# package, which also ships the .srv files Isaac Sim's own ROS bridge tooling depends on
# (IsaacPose, GetPrims, GetPrimAttribute(s), SetPrimAttribute). Shipping a second package under
# that name would collide, and if ours won the overlay it would remove those services.

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrameBboxes.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
