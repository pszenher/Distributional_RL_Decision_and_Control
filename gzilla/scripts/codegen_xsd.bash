#!/usr/bin/env bash

set -o errexit \
    -o nounset \
    -o pipefail

script_dir="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
repo_dir="$( realpath "${script_dir}/.." )"

function generate_xsd_bindings {

    if ((${#} != 3))
    then
	echo "Expected 3 arguments, received ${#};  exiting"
	exit 1
    fi

    local source_file="${1}"
    local package_dir="${2}"
    local package_name="${3}"
    
    echo "Generating xsd python reader package '${package_name}' in '${package_dir#${repo_dir}/}'"
    (
	mkdir -p "${package_dir}"
	cd "${package_dir}"
	xsdata \
	    generate \
	    \
	    --package "${package_name}" \
	    \
	    --output "dataclasses" \
	    --structure-style "filenames" \
	    --docstring-style "reStructuredText" \
	    --include-header \
	    \
	    "${source_file}"
    )
}

package_dir="${repo_dir}/config"
package_name="codegen_xsd.sdf"

# FIXME: generalize
source_file="${HOME}/src/sdformat/xsd_output/1.12/root.xsd"

# generate_xsd_bindings \
#     "${source_file}" \
#     "${package_dir}" \
#     "${package_name}"

generate_xsd_bindings \
    "${repo_dir}/config/xsd/urdf.xsd" \
    "${package_dir}" \
    "codegen_xsd.urdf"

# -ss, --structure-style [filenames|namespaces|clusters|single-package|namespace-clusters]
#                                 Output structure style [default: filenames]

