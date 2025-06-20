cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(virelex_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # cluster_msg
  "msg/PointCloudCluster.msg"
  # robot_info_msg
  "msg/RobotInfo.msg"
  # state_msg
  "msg/State.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
