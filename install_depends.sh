#!/bin/sh -e

repository_root="$(git rev-parse --show-toplevel)"

cd "$repository_root"

mkdir -p "$repository_root/external"

vcs import external < dependency.repos

cp -r "$repository_root/external/autoware/awapi/autoware_api_msgs"              "$repository_root/external"
cp -r "$repository_root/external/autoware/common/msgs/autoware_control_msgs"    "$repository_root/external"
cp -r "$repository_root/external/autoware/common/msgs/autoware_debug_msgs"      "$repository_root/external"
cp -r "$repository_root/external/autoware/common/msgs/autoware_perception_msgs" "$repository_root/external"
cp -r "$repository_root/external/autoware/common/msgs/autoware_planning_msgs"   "$repository_root/external"
cp -r "$repository_root/external/autoware/common/msgs/autoware_system_msgs"     "$repository_root/external"
cp -r "$repository_root/external/autoware/common/msgs/autoware_vehicle_msgs"    "$repository_root/external"

rm -rf "$repository_root/external/autoware"
