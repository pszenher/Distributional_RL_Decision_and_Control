#!/usr/bin/env bash

set -o errexit \
    -o nounset \
    -o pipefail

shopt -s globstar

if ! [[ -e "/run/.containerenv" ]]
then
    echo "ERROR:  expected to be run from within a container"
    exit 1
fi
    
if ((${#} != 2))
then
    echo "ERROR:  expected 2 passed arguments"
    exit 1
fi

repo_dir="${1}"
pxml_dir="${2}"

echo "Collecting package.xml files" 1>&2
echo "  Using repo_dir:  ${repo_dir}" 1>&2
echo "  Using pxml_dir:  ${pxml_dir}" 1>&2
echo 1>&2

if [[ -d "${pxml_dir}" ]]
then
    echo "NOTE:  existing cache found at package.xml cache dir location, deleting:" 1>&2
    echo "  ${pxml_dir}" 1>&2
    echo 1>&2
    rm -r "${pxml_dir}"
elif [[ -e "${pxml_dir}" ]]
then
     echo "ERROR:  non-directory file exists at package.xml cache dir location:" \
	  "${pxml_dir}" 1>&2
     exit 1
fi

mkdir "${pxml_dir}"

echo "repo_dir = ${repo_dir}"
ls -la "${repo_dir}"

(cd "${repo_dir}" && \
     shopt -s globstar && \
     cp -v --parents **/package.xml "${pxml_dir}")

package_xml_files="$(find ${pxml_dir} -type f -name "package.xml")"

echo 1>&2
echo "Collected $(echo "${package_xml_files}" | wc -l)" \
     "package.xml files in package.xml cache dir:" 1>&2

for file in ${package_xml_files}
do
    echo "  ${file#${pxml_dir}/}" 1>&2
done
