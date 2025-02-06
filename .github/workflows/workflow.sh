#!/bin/sh

file_path="$1"

# Arguments other than file path are passed to the command's arguments
shift 1

if [ ! -f "$file_path" ]
then
  echo "No such file: $file_path"
  exit 1
fi

workflow_directory="/tmp/scenario_workflow/$(basename "$file_path" | sed 's/\.[^.]*$//')"
rm -rf "$workflow_directory"
mkdir -p "$workflow_directory"

while IFS= read -r line
do
  ros2 launch scenario_test_runner scenario_test_runner.launch.py scenario:="$line" "$@"

  # save the result file before overwriting by next scenario
  mkdir -p "$(dirname "$dest_file")"
  cp /tmp/scenario_test_runner/result.junit.xml "$workflow_directory/$(basename "$line" | sed 's/\.[^.]*$//').junit.xml"
done < "$file_path"

failure_report="$workflow_directory/failure_report.md"
rm -f "$failure_report"
touch "$failure_report"
failure_found=0

for file in "$workflow_directory"/*.junit.xml; do
  [ -e "$file" ] || continue
  if grep -q '<failure' "$file"; then
    failure_found=1
    {
      echo "## $(basename "$file")"
      echo "<details><summary>scenario failed</summary><div>\n\n\`\`\`xml"
      grep '<failure' "$file"
      echo "\`\`\`\n</div></details>\n"
    } >> "$failure_report"
  fi
done

if [ $failure_found -eq 1 ]; then
  exit 1
else:
  exit 0
fi
