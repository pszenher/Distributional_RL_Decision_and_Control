#!/usr/bin/env -S unshare --mount --map-root-user /usr/bin/env bash

set -o errexit \
    -o nounset \
    -o pipefail

script_dir="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
package_dir="$( realpath "${script_dir}/.." )"

config_dir="${package_dir}/config"
schema_dir="${config_dir}/schema"
codegen_dir="${package_dir}/python/gzilla/codegen"

codegen_pkg="datamodel-code-generator"
codegen_bin="datamodel-codegen"

if ! [[ -e "${schema_dir}" ]]
then
    echo "[ERRO] ${0}: Schema directory '${schema_dir}' not found" >&2
    exit 1
fi

if ! type "${codegen_bin}" &> /dev/null
then
    echo "[WARN]: ${0}: Code generator '${codegen_bin}' not found" >&2

    if type "pipx" &> /dev/null;
    then
	codegen_bin="pipx run ${codegen_pkg}"
	echo "[INFO]: ${0}: Found 'pipx', using '${codegen_bin}'" >&2
    elif type "uvx"  &> /dev/null;
    then
	codegen_bin="uvx --from ${codegen_pkg} ${codegen_bin}"
	echo "[INFO]: ${0}: Found 'uvx', using '${codegen_bin}'" >&2
    else
	echo "[INFO]: ${0}: No ephemeral Python runners found, creating ephemeral venv" >&2
	mount -t tmpfs tmpfs /tmp
	python -m "venv" "/tmp/venv"
	source "/tmp/venv/bin/activate"
	pip install "${codegen_pkg}" --require-virtualenv --disable-pip-version-check \
	    | sed 's/^/[INFO]: (pip): /'
    fi
fi

echo "[INFO]: ${0}: Generating with ${codegen_pkg} version $(${codegen_bin} --version)"
echo "[INFO]: ${0}: Executing: '$(which ${codegen_bin})'"

${codegen_bin} \
    --input-file-type "jsonschema" \
    --output-model-type "pydantic.BaseModel" \
    \
    --input "${schema_dir}" \
    --output "${codegen_dir}" \
    \
    --enable-version-header \
    \
    --use-title-as-name \
    --use-schema-description \
    \
    --capitalize-enum-members \
    --enable-faux-immutability \
    --collapse-root-models \
    --use-exact-imports \
    \
    --target-python-version "3.11"
