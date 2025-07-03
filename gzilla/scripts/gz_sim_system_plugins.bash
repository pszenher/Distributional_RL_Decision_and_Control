#!/usr/bin/env -S unshare --mount --map-root-user /usr/bin/env bash

# ephemeral tmpfs script
#   see: https://overhead.neocities.org/blog/tmp-tmpfs

set -o errexit \
    -o nounset \
    -o pipefail

mount -t tmpfs tmpfs /tmp

repo_dir="/tmp/gz-sim"

action="fetch and check upstream gz-sim system plugin list"

if ! type "git" &> /dev/null
then
    echo "[ERR]: ${0}: 'git' command not found, cannot ${action}" >&2
    exit 1
fi

if ! type "rg" &> /dev/null
then
    echo "[ERR]: ${0}: 'rg' command not found, cannot ${action}" >&2
    exit 1
fi

git clone \
    --single-branch \
    --depth=1 \
    "https://github.com/gazebosim/gz-sim.git" "${repo_dir}"

rg 'filename=' "${repo_dir}" \
    | sed -n 's/^.*filename="\([A-z_-]*\)".*$/\1/p' \
    | sort \
    | uniq \
    | grep 'gz-sim-'
