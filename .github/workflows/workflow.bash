#!/bin/bash

if [ $# -lt 2 ]; then
    echo "usage: workflow.bash <command> <path of workflow csv file>"
    exit 1
fi

COMMAND="$1"
FILE_PATHS="$2"

if [ ! -f "$FILE_PATHS" ]; then
    echo "No such file: $FILE_PATHS"
    exit 1
fi

exit_status=0

while IFS= read -r line
do
    $COMMAND"$line"
    cmd_exit_status=$?
    if [ $cmd_exit_status -ne 0 ]; then
        echo "Error: caught non-zero exit code（code: $cmd_exit_status)"
        exit_status=1
    fi
done < "$FILE_PATHS"

# スクリプトの終了コードを設定
exit $exit_status
