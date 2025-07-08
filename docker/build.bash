#!/usr/bin/env bash

set -o errexit \
    -o nounset \
    -o pipefail

script_dir="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
repo_dir="$( realpath "${script_dir}/.." )"

oci_tool="podman"

containerfile="${repo_dir}/docker/Dockerfile.base-minimal"

image_name="localhost/virelex"
image_tag="latest"

# Cache `package.xml` files for image bulild
${repo_dir}/docker/collect_package_xml.bash

# Perform OCI image build
${oci_tool} \
    build \
    -t "${image_name}:${image_tag}" \
    -f "${containerfile}" \
    ${repo_dir}
