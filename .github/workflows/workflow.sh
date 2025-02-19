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
workflow_directory="/tmp/scenario_workflow/$(basename "$file_path" | sed 's/\.[^.]*$//')"
rm -rf "$workflow_directory"
mkdir -p "$workflow_directory"

while IFS= read -r line
do
  ros2 launch scenario_test_runner scenario_test_runner.launch.py scenario:="$line" "$@"
  ros2 run scenario_test_runner result_checker.py /tmp/scenario_test_runner/result.junit.xml
  command_exit_status=$?
  if [ $command_exit_status -ne 0 ]; then
    echo "Error: caught non-zero exit status（code: $command_exit_status)"
    exit_status=1
  fi

  # save the result file before overwriting by next scenario
  mkdir -p "$(dirname "$dest_file")"
  cp /tmp/scenario_test_runner/result.junit.xml "$workflow_directory/$(basename "$line" | sed 's/\.[^.]*$//').junit.xml"
done < "$file_path"

exit $exit_status
