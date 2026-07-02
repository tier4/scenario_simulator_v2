#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 TIER IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from alpine_js import ALPINE_JS
from pico_css import PICO_CSS

HTML_TEMPLATE = """\
<!DOCTYPE html>
<html lang="en" data-theme="light">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Diffusion Planner Model Comparison</title>
  <style>{css}</style>
</head>
<body x-data="reportData()">
  <main class="container">
    <h1>Diffusion Planner Model Comparison</h1>
    <p x-text="summary"></p>
  </main>
  <script>{js}</script>
  <script>
    function reportData() {{
      const data = {data_json};
      return {{
        ...data,
        get summary() {{
          return `${{data.models.length}} model(s), ${{data.scenarios.length}} scenario(s)`;
        }},
      }};
    }}
  </script>
</body>
</html>
"""


def generate_report(host_dir, comparison_results, output_path=None):
    """Generate a self-contained HTML report.

    Parameters
    ----------
    host_dir : Path
        Host model output directory.
    comparison_results : list of (int, str, Path)
        [(model_index, model_name, output_dir), ...]
    output_path : Path, optional
        Where to write the report. Defaults to host_dir / "report.html".
    """
    if output_path is None:
        output_path = host_dir / "report.html"

    # TODO: read rosbags and extract data for the report
    data = {
        "models": [
            {"index": i, "name": name}
            for i, name, _ in comparison_results
        ],
        "scenarios": [
            p.stem for p in sorted(host_dir.rglob("*.xosc"))
        ],
    }

    import json
    html = HTML_TEMPLATE.format(
        css=PICO_CSS,
        js=ALPINE_JS,
        data_json=json.dumps(data),
    )
    output_path.write_text(html)
    return output_path
