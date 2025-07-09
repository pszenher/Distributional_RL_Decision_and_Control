cmake_minimum_required(VERSION 3.8)
project(gzilla)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package( gz-sim REQUIRED
  COMPONENTS all
)

find_package(gz_gui_vendor REQUIRED)
find_package(gz-gui REQUIRED)

find_package(gz_fuel_tools_vendor REQUIRED)
find_package(gz-fuel_tools REQUIRED)

find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)

find_package(sdformat_vendor REQUIRED)
find_package(sdformat REQUIRED)

# Build python bindings for gz-math
ament_vendor("gz_math${gz-math_VERSION_MAJOR}_python_vendor"
  SATISFIED "${gz-math${gz-math_VERSION_MAJOR}-python_FOUND}"
  VCS_URL "https://github.com/gazebosim/gz-math.git"
  VCS_VERSION "gz-math${gz-math_VERSION_MAJOR}_${gz-math_VERSION}"
  SOURCE_SUBDIR "src/python_pybind11"
  CMAKE_ARGS "-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON"
)

# Build python bindings for sdformat
ament_vendor("sdformat${sdformat_VERSION_MAJOR}_python_vendor"
  SATISFIED "${sdformat${sdformat_VERSION_MAJOR}-python_FOUND}"
  VCS_URL "https://github.com/gazebosim/sdformat.git"
  VCS_VERSION "sdformat${sdformat_VERSION_MAJOR}_${sdformat_VERSION}"
  SOURCE_SUBDIR "python"
  CMAKE_ARGS "-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON"
)

# PYTHONPATH hooks for vendored packages
ament_environment_hooks("hooks/python_path_vendor.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

# Install static resources
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install executable scripts
install(
  DIRECTORY   bin
  DESTINATION .
)

# GzServer composable node
# ================================
add_library(gzserver_component
  SHARED
  src/gzserver.cpp
)
rclcpp_components_register_nodes(gzserver_component
  "${PROJECT_NAME}::GzServer"
)
target_link_libraries(gzserver_component
  PUBLIC
  ${std_msgs_TARGETS}
  gz-sim::core
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
)

target_include_directories(gzserver_component
  PUBLIC
  ${std_msgs_TARGETS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(
  gzserver_component
  PLUGIN "${PROJECT_NAME}::GzServer"
  EXECUTABLE gzserver
)

ament_export_targets(export_gzserver_component)
install(
  TARGETS gzserver_component
  EXPORT  export_gzserver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS gzserver
  DESTINATION lib/${PROJECT_NAME}
)

# GzGui imported boilerplate
# ================================
add_library(gui_runner
  SHARED src/GuiRunner.cc
)
target_link_libraries(gui_runner
  gz-gui
  gz-fuel_tools
  gz-sim::core
  gz-sim::gui
)
target_include_directories(gui_runner
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_export_targets(gui_runner)
install(
  TARGETS gui_runner
  EXPORT  gui_runner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# GzGui composable node
# ================================
add_library(gzgui_component
  SHARED
  src/gzgui.cpp
)
rclcpp_components_register_nodes(gzgui_component
  "${PROJECT_NAME}::GzGui"
)
target_link_libraries(gzgui_component
  PUBLIC
  gui_runner
  ${std_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
  gz-sim::core
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
)

target_include_directories(gzgui_component
  PUBLIC
  ${std_msgs_TARGETS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(
  gzgui_component
  PLUGIN "${PROJECT_NAME}::GzGui"
  EXECUTABLE gzgui
)

ament_export_targets(export_gzgui_component)
install(
  TARGETS gzgui_component
  EXPORT  export_gzgui_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS gzgui
  DESTINATION lib/${PROJECT_NAME}
)


# ================================


ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR "python/${PROJECT_NAME}"
)

ament_package()
