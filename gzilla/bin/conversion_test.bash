#!/usr/bin/env bash

xacro "$(ros2 pkg prefix --share virelex)/urdf/wamv_gazebo.urdf.xacro" \
      "namespace:=wamv11" \
      "vrx_sensors_enabled:=true" \
    | /ws/src/virelex/bin/urdf2sdf

