#!/usr/bin/env bash
# Sources ROS 2 and the robot's packages, then runs the given command.
set -e
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source /opt/robot/setup.bash
exec "$@"
