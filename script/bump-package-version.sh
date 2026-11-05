#!/bin/sh

set -eu

cd "$(git rev-parse --show-toplevel)"

case "${1:-minor}" in
  minor | patch)
    year=$(TZ=Asia/Tokyo date +%y)
    while [ "$(($(catkin_package_version | cut -d . -f 1) + 1))" -lt "$year" ]
    do
      catkin_package_version --bump major 1>&2
    done
    if [ "$(catkin_package_version | cut -d . -f 1)" -ne "$year" ]
    then
      part=major
    else
      part=${1:-minor}
    fi
    catkin_generate_changelog -y 1>&2
    catkin_tag_changelog --bump "$part" 1>&2
    catkin_package_version --bump "$part" 1>&2
    catkin_package_version
    ;;
  *)
    echo "usage: $0 [minor|patch]" >&2
    exit 1
    ;;
esac
