#!/usr/bin/env bash

set -o errexit \
    -o nounset \
    -o pipefail

script_dir="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
repo_dir="$( realpath "${script_dir}/.." )"

schema_dir="${repo_dir}/config/schema"
instance_dir="${repo_dir}/config"

if ! type "jv" &> /dev/null
then
    echo "[ERR]: ${0}: 'jv' command not found, cannot validate jsonschema" >&2
    exit 1
fi

# TODO: investigate sourcemeta's or pip `jsonschema` commands as alternative
#       to `jv`

jv "${schema_dir}/gz_sensor_map.schema.json" \
   "${instance_dir}/gz_sensor_mappings.yaml"

jv "${schema_dir}/gz_plugin_map.schema.json" \
   "${instance_dir}/gz_plugin_mappings.yaml" \
   "${instance_dir}/gz_vrx_plugin_mappings.yaml"
