cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(virelex)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Torch QUIET)
if ( NOT Torch_FOUND )
  include(FetchTorch)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(message_filters REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED)

find_package(virelex_msgs REQUIRED)

# Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${TORCH_INCLUDE_DIRS}
)

# action_planner
add_executable(action_planner_node src/action_planner_node.cpp)
ament_target_dependencies(action_planner_node
  rclcpp
  std_msgs
  virelex_msgs
)
target_link_libraries(action_planner_node
  ${TORCH_LIBRARIES}
)
install(TARGETS
  action_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# collision_detector
add_executable(collision_detector_node src/collision_detector_node.cpp)
ament_target_dependencies(collision_detector_node
  rclcpp
  std_msgs
  nav_msgs
)
install(TARGETS
  collision_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# lidar_processor
add_executable(lidar_processor_node src/lidar_processor_node.cpp)
ament_target_dependencies(lidar_processor_node
  rclcpp
  sensor_msgs
  virelex_msgs
)
install(TARGETS
  lidar_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# state_processor
add_executable(state_processor_node src/state_processor_node.cpp)
ament_target_dependencies(state_processor_node
  rclcpp
  sensor_msgs
  geometry_msgs
  message_filters
  std_msgs
  nav_msgs
  virelex_msgs
)
install(TARGETS
  state_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and pretrained weight files
install(DIRECTORY
  config
  launch
  trained
  models
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR "python/${PROJECT_NAME}"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
