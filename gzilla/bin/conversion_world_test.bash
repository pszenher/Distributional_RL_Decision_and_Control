#!/usr/bin/env bash

cat "$(ros2 pkg prefix --share virelex)/worlds/sydney_regatta_minimal.sdf" \
    | /ws/src/virelex/bin/sdfdump.py

