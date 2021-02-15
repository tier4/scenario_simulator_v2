#!/bin/sh -e

repository_root="$(git rev-parse --show-toplevel)"

cd "$repository_root"

mkdir -p "$repository_root/external"

vcs import external < dependency.repos

git clone -b ros2 https://github.com/tier4/pilot.auto.git

mv "$repository_root/pilot.auto/awapi/autoware_api_msgs"              "$repository_root/external"
mv "$repository_root/pilot.auto/common/msgs/autoware_control_msgs"    "$repository_root/external"
mv "$repository_root/pilot.auto/common/msgs/autoware_perception_msgs" "$repository_root/external"
mv "$repository_root/pilot.auto/common/msgs/autoware_planning_msgs"   "$repository_root/external"
mv "$repository_root/pilot.auto/common/msgs/autoware_system_msgs"     "$repository_root/external"
mv "$repository_root/pilot.auto/common/msgs/autoware_vehicle_msgs"    "$repository_root/external"

rm -rf "$repository_root/pilot.auto"
