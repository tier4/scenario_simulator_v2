#!/bin/sh

file_path="$1"

# Arguments other than file path are passed to the command's arguments
shift 1

if [ ! -f "$file_path" ]
then
  echo "No such file: $file_path"
  exit 1
fi

exit_status=0

while IFS= read -r line
do
  output_directory="/tmp/scenario_workflow/$(basename "$file_path" | sed 's/\.[^.]*$//')/$(basename "$line" | sed 's/\.[^.]*$//')"
  ros2 launch scenario_test_runner scenario_test_runner.launch.py scenario:="$line" output_directory:=$output_directory "$@"
  ros2 run scenario_test_runner result_checker.py $output_directory/scenario_test_runner/result.junit.xml
  command_exit_status=$?
  if [ $command_exit_status -ne 0 ]; then
    echo "Error: caught non-zero exit status（code: $command_exit_status)"
    exit_status=1
  fi
done < "$file_path"

ci_test_inputs_dir="$(ros2 pkg prefix random_test_runner)/share/random_test_runner/ci_test_inputs"
random_output_dir="/tmp/random_test_runner"
mkdir -p "$random_output_dir"

ros2 launch random_test_runner random_test.launch.py \
  output_dir:=$random_output_dir \
  input_dir:=$ci_test_inputs_dir \
  test_timeout:=180.0 \
  spawn_ego_as_npc:=true

ros2 run scenario_test_runner result_checker.py $random_output_dir/result.junit.xml
random_test_exit_status=$?

if [ $random_test_exit_status -ne 0 ]; then
  echo "Error: caught non-zero exit status（code: $random_test_exit_status)"
  exit_status=1
fi

exit $exit_status
