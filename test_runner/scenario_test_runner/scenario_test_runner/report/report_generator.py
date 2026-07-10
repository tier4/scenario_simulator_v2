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

if __name__ == "__main__" and __package__ is None:
    import sys as _sys
    from pathlib import Path as _Path

    _pkg_parent = str(_Path(__file__).resolve().parent.parent.parent)
    if _pkg_parent not in _sys.path:
        _sys.path.insert(0, _pkg_parent)
    __package__ = "scenario_test_runner.report"

import base64
import gzip
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from .alpine_js import ALPINE_JS
from .d3_js import D3_JS
from .open_props import OPEN_PROPS_CSS

_TEMPLATE_PATH = (
    Path(get_package_share_directory("scenario_test_runner"))
    / "templates"
    / "report_template.html"
)


def _load_template():
    return _TEMPLATE_PATH.read_text()


_MODEL_HUES = [210, 30, 150, 330, 60, 270, 0, 120]


def _assign_color(index):
    hue = _MODEL_HUES[index % len(_MODEL_HUES)]
    return f"hsl({hue},80%,60%)"


def _compute_bbox(models):
    all_x, all_y = [], []
    for m in models:
        all_x.extend(v for v in m["x"] if v is not None)
        all_y.extend(v for v in m["y"] if v is not None)
    if not all_x:
        return [0, 0, 1, 1]
    return [min(all_x), min(all_y), max(all_x), max(all_y)]


MAP_CLIP_MARGIN = 50.0  # meters around trajectory bbox


def _clip_segment(p1, p2, x0, y0, x1, y1):
    """Liang-Barsky clip of segment p1-p2 against axis-aligned rect."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    t0, t1 = 0.0, 1.0
    for p, q in [(-dx, p1[0] - x0), (dx, x1 - p1[0]),
                 (-dy, p1[1] - y0), (dy, y1 - p1[1])]:
        if abs(p) < 1e-12:
            if q < 0:
                return None
        else:
            r = q / p
            if p < 0:
                t0 = max(t0, r)
            else:
                t1 = min(t1, r)
            if t0 > t1:
                return None
    return ([round(p1[0] + t0 * dx, 2), round(p1[1] + t0 * dy, 2)],
            [round(p1[0] + t1 * dx, 2), round(p1[1] + t1 * dy, 2)])


def _clip_polyline(pts, x0, y0, x1, y1):
    """Clip polyline against rect. Returns list of sub-polylines."""
    chains = []
    cur = []
    for k in range(len(pts) - 1):
        seg = _clip_segment(pts[k], pts[k + 1], x0, y0, x1, y1)
        if seg is None:
            if cur:
                chains.append(cur)
                cur = []
            continue
        cp1, cp2 = seg
        if not cur:
            cur = [cp1]
        elif cur[-1] != cp1:
            chains.append(cur)
            cur = [cp1]
        cur.append(cp2)
    if cur:
        chains.append(cur)
    return chains


def _clip_map_data(map_data, bbox):
    """Clip lanelets/markings to the trajectory bbox + margin."""
    x0 = bbox[0] - MAP_CLIP_MARGIN
    y0 = bbox[1] - MAP_CLIP_MARGIN
    x1 = bbox[2] + MAP_CLIP_MARGIN
    y1 = bbox[3] + MAP_CLIP_MARGIN

    lanelet_lines = []
    for ll in map_data["lanelets"]:
        lanelet_lines.extend(_clip_polyline(ll["left"], x0, y0, x1, y1))
        lanelet_lines.extend(_clip_polyline(ll["right"], x0, y0, x1, y1))

    markings = []
    for mk in map_data["road_markings"]:
        clipped = _clip_polyline(mk["points"], x0, y0, x1, y1)
        for line in clipped:
            markings.append({"points": line, "type": mk["type"]})

    return {"lanelet_lines": lanelet_lines, "road_markings": markings}


def generate_report(output_dir, report_output_directory=None):
    """Generate self-contained HTML comparison reports.

    Discovers models from ``output_dir/staging/`` (symlink to raw bags).
    Falls back to single-model discovery when staging is absent.
    Reports are written to ``report_output_directory`` if given,
    otherwise ``output_dir/comparison_report/``.
    """
    from .rosbag_reader import (
        extract_entities, extract_map_data, extract_predicted_objects,
        extract_trajectories,
        PERCEIVED_OBJECTS_TOPIC, RATE_HZ,
    )

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

    if report_output_directory:
        result_archive = Path(report_output_directory)
    else:
        result_archive = output_dir / "comparison_report"
    result_archive.mkdir(parents=True, exist_ok=True)

    def _b64gz(text):
        return base64.b64encode(gzip.compress(text.encode())).decode()

    vendor_template = (
        _load_template()
        .replace("{{OPEN_PROPS_GZ}}", _b64gz(OPEN_PROPS_CSS))
        .replace("{{D3_GZ}}", _b64gz(D3_JS))
        .replace("{{ALPINE_GZ}}", _b64gz(ALPINE_JS))
    )

    report_paths = []
    for scenario_bag in scenario_bags:
        if model_dirs:
            rel = scenario_bag.relative_to(model_dirs[0])
            models = []
            for i, md in enumerate(model_dirs):
                bag = md / rel
                if bag.is_dir():
                    traj_data = extract_trajectories(bag)
                    traj_data["name"] = md.name
                    traj_data["color"] = _assign_color(i)
                    entities, ego_bbox = extract_entities(bag)
                    traj_data["entities"] = entities
                    traj_data["ego_bbox"] = ego_bbox
                    traj_data["perceived_objects"] = extract_predicted_objects(
                        bag, PERCEIVED_OBJECTS_TOPIC)
                    models.append(traj_data)
        else:
            traj_data = extract_trajectories(scenario_bag)
            traj_data["name"] = output_dir.name
            traj_data["color"] = _assign_color(0)
            entities, ego_bbox = extract_entities(scenario_bag)
            traj_data["entities"] = entities
            traj_data["ego_bbox"] = ego_bbox
            traj_data["perceived_objects"] = extract_predicted_objects(
                scenario_bag, PERCEIVED_OBJECTS_TOPIC)
            models = [traj_data]

        n = max((len(m["x"]) for m in models), default=0)
        bbox = _compute_bbox(models)
        data = {
            "rate_hz": RATE_HZ,
            "n": n,
            "bbox": bbox,
            "map": _clip_map_data(map_data, bbox),
            "models": models,
        }
        html = vendor_template.replace(
            "{{DATA_B64GZ}}",
            base64.b64encode(
                gzip.compress(json.dumps(data, separators=(",", ":")).encode())
            ).decode(),
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
