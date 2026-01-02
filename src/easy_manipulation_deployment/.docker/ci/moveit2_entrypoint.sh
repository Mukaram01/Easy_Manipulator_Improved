#!/bin/bash
set -e

# setup moveit2 environment
# shellcheck disable=SC1091
source /opt/moveit2/install/setup.bash
exec "$@"
