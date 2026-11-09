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

exit $exit_status
