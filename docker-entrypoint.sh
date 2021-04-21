#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/ubuntu/Desktop/scenario_simulator_ws/install/local_setup.bash"
exec "$@"