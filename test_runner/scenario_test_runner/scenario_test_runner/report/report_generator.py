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

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from .alpine_js import ALPINE_JS
from .chota_css import CHOTA_CSS

_TEMPLATE_PATH = (
    Path(get_package_share_directory("scenario_test_runner"))
    / "templates"
    / "report_template.html"
)


def _load_template():
    return _TEMPLATE_PATH.read_text()


def generate_report(output_dir):
    """Generate self-contained HTML comparison reports.

    Discovers models from ``output_dir/staging/`` (symlink to raw bags).
    Falls back to single-model discovery when staging is absent.
    Reports are written to ``output_dir/result_archive/``.
    """
    from .rosbag_reader import extract_map_data, extract_trajectories

    output_dir = Path(output_dir)
    staging = output_dir / "staging"

    if staging.is_dir():
        model_dirs = sorted(d for d in staging.iterdir() if d.is_dir())
        if not model_dirs:
            return []
        first_model = model_dirs[0]
        scenario_bags = sorted(
            d for xosc in first_model.rglob("*.xosc")
            if (d := xosc.parent / xosc.stem).is_dir()
        )
    else:
        model_dirs = None
        scenario_bags = sorted(
            d for xosc in output_dir.rglob("*.xosc")
            if (d := xosc.parent / xosc.stem).is_dir()
        )

    if not scenario_bags:
        return []

    map_data = extract_map_data(scenario_bags[0])

    result_archive = output_dir / "result_archive"
    result_archive.mkdir(exist_ok=True)

    report_paths = []
    for scenario_bag in scenario_bags:
        if model_dirs:
            rel = scenario_bag.relative_to(model_dirs[0])
            models = []
            for md in model_dirs:
                bag = md / rel
                if bag.is_dir():
                    models.append({
                        "name": md.name,
                        "trajectories": extract_trajectories(bag),
                    })
        else:
            models = [{
                "name": output_dir.name,
                "trajectories": extract_trajectories(scenario_bag),
            }]

        data = {"map": map_data, "models": models}
        html = (
            _load_template()
            .replace("{{CHOTA_CSS}}", CHOTA_CSS)
            .replace("{{ALPINE_JS}}", ALPINE_JS)
            .replace("{{DATA_JSON}}", json.dumps(data, separators=(",", ":")))
        )

        if len(scenario_bags) == 1:
            out = result_archive / "report.html"
        else:
            out = result_archive / f"report_{scenario_bag.name}.html"

        out.write_text(html)
        report_paths.append(out)

    return report_paths


if __name__ == "__main__":
    import sys

    output_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/scenario_test_runner")
    paths = generate_report(output_dir)
    if not paths:
        print(f"No bags found in {output_dir}")
        sys.exit(1)
    for p in paths:
        print(f"Report generated: {p}")
