#!/bin/sh -e

repository_root="$(git rev-parse --show-toplevel)"

cd "$repository_root"

mkdir -p "$repository_root/external"

vcs import external < dependency_"$1".repos

rm -rf external/AutowareArchitectureProposal_msgs/autoware_hmi_msgs

rosdep install -iry --from-paths . --rosdistro $1