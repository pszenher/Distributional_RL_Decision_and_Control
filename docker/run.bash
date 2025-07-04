#!/usr/bin/env bash

set -o errexit \
    -o nounset \
    -o pipefail

script_dir="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
repo_dir="$( realpath "${script_dir}/.." )"

oci_tool="podman"

xorg_socket_dir="/tmp/.X11-unix"

# TODO: add checks for:
#    - nvidia cdi generation (i.e., `/etc/cdi/nvidia.yaml` is present and valid)
#      - for `--device` flag
#    - SELinux enablement
#      - for `--security-opt` flag
#    - X11 support
#      - for `-e DISPLAY` and `-v ${xorg_socket_dir}`

${oci_tool} \
    run --rm --interactive --tty  \
    \
    --security-opt "label=type:container_runtime_t" \
    \
    --device "nvidia.com/gpu=all" \
    -e "DISPLAY" \
    -v "${xorg_socket_dir}:/tmp/.X11-unix:ro" \
    -v "${repo_dir}:/ws/src:ro" \
    \
    "local/virelex:latest"

# NOTE: For inter-container and host ROS2 integration
    # --network="host" \
    # --ipc="host" \
    # --pid="host" \
    # --cap-add="CAP_NET_ADMIN" \

# NOTE: for rosbridge websocket connection
    # -p "9090:9090" \

# NOTE: for gzweb websocket connection
#   -p 9002:9002 \
#   <exec_depend>gz_launch_vendor</exec_depend>
