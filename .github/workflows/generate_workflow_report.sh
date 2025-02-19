#!/bin/sh

if [ $# -ne 1 ]; then
    echo "Usage: $0 <workflow_directory>"
    exit 1
fi

workflow_directory="$1"
failure_report="$workflow_directory/failure_report.md"

rm -f "$failure_report"
touch "$failure_report"

for file in "$workflow_directory"/*.junit.xml; do
  [ -e "$file" ] || continue
  if grep -q '<failure' "$file"; then
    {
      echo "<details><summary>scenario failed: $(basename "$file")</summary><div>\n\n\`\`\`xml"
      grep '<failure' "$file"
      echo "\`\`\`\n</div></details>\n"
    } >> "$failure_report"
  fi
done
