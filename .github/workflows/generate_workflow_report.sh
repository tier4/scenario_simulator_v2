#!/bin/sh

failure_report="/tmp/scenario_workflow/failure_report.md"

rm -f "$failure_report"
touch "$failure_report"

find /tmp/scenario_workflow -name "result.junit.xml" | while read file; do
  [ -e "$file" ] || continue
  if grep -q '<failure' "$file"; then
    {
      echo "<details><summary>scenario failed: $(dirname "${file#/tmp/scenario_workflow/}" | cut -d'/' -f1)</summary><div>\n\n\`\`\`xml"
      grep '<failure' "$file"
      echo "\`\`\`\n</div></details>\n"
    } >> "$failure_report"
  fi
done
