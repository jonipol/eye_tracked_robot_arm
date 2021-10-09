#!/bin/bash
set -e  # This was causing Docker to exit when auto-filling or running into error

. "/opt/ros/$ROS_DISTRO/setup.sh"
. "$ROBOT_WS/install/setup.sh"
. "$PACKAGE_WS/install/setup.sh"
. "$UR_WS/install/setup.sh"

exec "$@"
