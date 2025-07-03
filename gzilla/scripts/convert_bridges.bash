#!/usr/bin/env bash

set -o errexit \
    -o nounset \
    -o pipefail

script_dir="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
repo_dir="$( realpath "${script_dir}/.." )"
vrx_dir="${repo_dir}/vrx"

bridges_file="${vrx_dir}/vrx_gz/src/vrx_gz/bridges.py"
payload_bridges_file="${vrx_dir}/vrx_gz/src/vrx_gz/payload_bridges.py"

# echo '%YAML 1.2'
# echo

# echo '# let prefix(world_name, model_name, link_name) := /world/{world_name}/model/{model_name}/link/{link_name}/sensor'
# echo

function convert_bridge {
    local py_file="${1}"

    echo "# Converted from:  ${py_file#${repo_dir}/}"

    cat ${py_file} \
	| grep -v '^ *#' \
	| grep -v '^from' \
	| grep -v '^import' \
	| sed '/def payload_bridges/Q' \
	| grep -v 'return Bridge(' \
	| sed "s/^ *return f'\(.*\)'$/#  := \1/" \
	| sed "s/ *gz_topic=f*'\(.*\)',/- gz_topic_name: \"\1\"/"  \
	| sed "s/ *ros_topic=f*'\(.*\)',/  ros_topic_name: \"\1\"/" \
	| sed "s/ *gz_type=f*'\(.*\)',/  gz_type_name: \"\1\"/"     \
	| sed "s/ *ros_type=f*'\(.*\)',/  ros_type_name: \"\1\"/" \
	| sed "s/ *direction=BridgeDirection\.\(.*\))/  direction: \"\1\"/" \
	| sed "s/^def \(.*(.*)\):$/# \1/" \
	| sed "s/^ *\(.*_prefix\) = \(.*\)$/#   let \1 := \2/" \
	| grep -v '^--$' \
	| sed -ze 's/\n\n\n*/\n\n/g'

}

function jq_conv {
    local key="${1}"

    echo "${key}=\\\"\(.${key})\\\""
}

function convert_bridge_xml {
    local yaml_file="${1}"

    if ! type "yq" > /dev/null
    then
	echo "WARN: 'yq' command not found, skipping xml generation" 1>&2
	return 1
    fi

    topic_attrs_query="$(jq_conv "ros_topic_name")
       $(jq_conv "gz_topic_name")
       $(jq_conv "ros_type_name")
       $(jq_conv "gz_type_name")
       $(jq_conv "direction")"
    
    yq_command=".[] | \"<topic ${topic_attrs_query} />\n\""

    echo '<?xml version="1.0" encoding="UTF-8"?>'
    echo '<ros_gz_bridge bridge_name="$(var bridge_name)" config_file="$(var config_file)">'
    echo
    
    cat "${yaml_file}" | yq -r "${yq_command}"

    echo '</ros_gz_bridge>'
}


echo "ERROR ${0}: Not Implemented: TODO: need to update paths for this file before executing" >&2
exit 1

convert_bridge ${bridges_file} > "${repo_dir}/virelex/config/vrx_bridges.yaml"
echo "Wrote ${repo_dir}/virelex/config/vrx_bridges.yaml" 1>&2

convert_bridge ${payload_bridges_file} \
  > "${repo_dir}/virelex/config/vrx_payload_bridges.yaml"
echo "Wrote ${repo_dir}/virelex/config/vrx_payload_bridges.yaml" 1>&2

convert_bridge_xml "${repo_dir}/virelex/config/vrx_bridges.yaml" \
  > "${repo_dir}/virelex/config/vrx_bridges.xml"
echo "Wrote ${repo_dir}/virelex/config/vrx_bridges.xml" 1>&2

convert_bridge_xml "${repo_dir}/virelex/config/vrx_payload_bridges.yaml" \
  > "${repo_dir}/virelex/config/vrx_payload_bridges.xml"
echo "Wrote ${repo_dir}/virelex/config/vrx_payload_bridges.xml" 1>&2
