#!/usr/bin/env bash

set -o errexit \
    -o nounset \
    -o pipefail

script_dir="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
repo_dir="$( git rev-parse --show-toplevel )"

if [[ -t 1 ]]
then
    echo "ERROR: ${0}: this utility must be run with stdout piped to a file/program"
    exit 1
fi

echo "Tarring package.xml files on stdout" >&2
echo "  repo_dir:  ${repo_dir}"      >&2

package_xml_files="$(
    cd "${repo_dir}"
    find -type f -name "package.xml"
)"

echo "Found $(echo "${package_xml_files}" | wc -l)" \
     "package.xml files in repo:" >&2
for pxml_file in ${package_xml_files}
do
    echo "  ${pxml_file#./}" 1>&2
done

tar --directory="${repo_dir}" \
    --create \
    ${package_xml_files}
