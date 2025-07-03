#!/usr/bin/env -S unshare --mount --map-root-user /usr/bin/env bash

# ephemeral tmpfs script
#   see: https://overhead.neocities.org/blog/tmp-tmpfs

set -o errexit \
    -o nounset \
    -o pipefail

mount -t tmpfs tmpfs /tmp

repo_dir="/tmp/sdformat"

action="fetch and check upstream sdformat sensor types"

if ! type "git" &> /dev/null
then
    echo "[ERR]: ${0}: 'git' command not found, cannot ${action}" >&2
    exit 1
fi

git clone \
    --single-branch \
    --depth=1 \
    "https://github.com/gazebosim/sdformat.git" "${repo_dir}"

# NOTE: this is brittle as all hell; we're relying on the fact that
#       the only lines in this file which start with two spaces and a
#       double-quote happen to be exactly those in the
#       `kSensorTypeStrs` array definition.  Correct thing to do here
#       would be to use a C++ ast parser, it's not worth it just
#       yet...
#

sensor_cc_types=$( cat "${repo_dir}/src/Sensor.cc" \
		       | grep '^  "' \
		       | sed 's/  "\(.*\)",*$/\1/' \
		       | sort )

pysensor_cc_types=$( cat "${repo_dir}/python/src/sdf/pySensor.cc" \
			 | sed -n 's/^ *.value("\([A-Z_]*\)", sdf::SensorType::[A-Z_]*).*$/\1/p' \
			 | tr '[:upper:]' '[:lower:]' \
			 | sort )


echo "Sensor.cc type names: $(echo "${sensor_cc_types}" | wc -l)" >&2
echo "pySensor.cc type names: $(echo "${pysensor_cc_types}" | wc -l)" >&2

echo "Diff type names: Sensor.cc pySensor.cc " >&2
diff -y <(echo "${sensor_cc_types}") <(echo "${pysensor_cc_types}") >&2 || :

echo "Printing full type name list to stdout..." >&2
(echo "${sensor_cc_types}" && echo "${pysensor_cc_types}") \
    | sort \
    | uniq
