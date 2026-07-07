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

# === Tech Stack & Design Notes (for AI assistants and future maintainers) ===
#
# This package generates a single self-contained HTML report comparing
# simulation trajectories across multiple models/branches.
#
# Frontend stack (all embedded inline — no CDN, no external requests):
#   - Alpine.js  : UI reactivity. Components use Alpine.data() with closure-
#                   scoped private functions; the returned object stays small
#                   (only init + event handlers). Vendor module: alpine_js.py
#   - D3.js      : Time-series plots (velocity, accel, heading_rate, steer).
#                   Vendor module: d3_js.py
#   - Canvas+SVG : Map/trajectory viewer. Canvas for raster drawing (lanes,
#                   trajectories), SVG overlay for labels/markers. Uses
#                   devicePixelRatio for HiDPI and ResizeObserver for layout-
#                   aware initial fit.
#   - OpenProps   : CSS design tokens (custom properties only, no utility
#                   classes). All layout/component CSS is self-managed in the
#                   template, referencing OpenProps variables for spacing,
#                   radius, font sizes. Vendor module: open_props.py
#
# Vendor modules (alpine_js.py, d3_js.py, open_props.py):
#   Each embeds the library source as a Python string constant. Pattern:
#   CDN_URL, VENDOR_BEGIN/END sentinels, `python -m <module>` to re-fetch.
#
# Key files:
#   report_generator.py          — entry point; reads bags, fills template
#   rosbag_reader.py             — extracts ego trajectory + map from rosbags
#   templates/report_template.html — the single-file HTML template
#
# CSS approach: OpenProps tokens + self-managed CSS (not a component framework).
# Dark/light theming via CSS custom properties (--surface-*, --text-*, --accent).
# Previous approaches (chota.css, pico.css) were abandoned due to conflicts
# with canvas/SVG and poor dark-mode form control support.
