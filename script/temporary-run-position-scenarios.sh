#!/bin/sh -u

scenario_directory=$(cd "$(dirname "$0")/../test_runner/scenario_test_runner/scenario" && pwd)

unexpected=0

run()
{
  output_directory="${TMPDIR:-/tmp}/temporary-run-position-scenarios.$$"

  rm -rf "$output_directory"

  ros2 launch scenario_test_runner scenario_test_runner.launch.py \
    launch_rviz:=true \
    output_directory:="$output_directory" \
    record:=false \
    scenario:="$scenario_directory/$1" \
    > /dev/null 2>&1

  grep -q 'failures="0" errors="0"' "$output_directory/scenario_test_runner/result.junit.xml" 2> /dev/null

  status=$?

  rm -rf "$output_directory"

  return $status
}

expect_success()
{
  if run "$1"
  then
    echo "$1: success"
  else
    echo "$1: failure"
    unexpected=$((unexpected + 1))
  fi
}

expect_failure()
{
  if run "$1"
  then
    echo "$1: success (failure was expected)"
    unexpected=$((unexpected + 1))
  else
    echo "$1: failure (expected)"
  fi
}

expect_success AddEntityAction.WorldPosition.yaml
expect_success AddEntityAction.RelativeWorldPosition.yaml
expect_success AddEntityAction.RelativeObjectPosition.yaml
expect_success AddEntityAction.LanePosition.yaml
expect_success TeleportAction.WorldPosition.yaml
expect_failure TeleportAction.RelativeWorldPosition.yaml
expect_success TeleportAction.RelativeObjectPosition.yaml
expect_success TeleportAction.LanePosition.yaml

test "$unexpected" -eq 0
