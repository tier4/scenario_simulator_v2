#!/bin/sh

FILE_PATH="$1"

# Arguments other than file path are passed to the command's arguments
shift 1

if [ ! -f "$FILE_PATH" ]; then
    echo "No such file: $FILE_PATH"
    exit 1
fi

exit_status=0

while IFS= read -r line
do
    ros2 launch scenario_test_runner scenario_test_runner.launch.py scenario:="$line" "$@"
    ros2 run scenario_test_runner result_checker.py /tmp/scenario_test_runner/result.junit.xml
    cmd_exit_status=$?
    if [ $cmd_exit_status -ne 0 ]; then
        echo "Error: caught non-zero exit code（code: $cmd_exit_status)"
        exit_status=1
    fi
done < "$FILE_PATH"

exit $exit_status
