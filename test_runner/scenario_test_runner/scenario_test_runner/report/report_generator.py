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

from alpine_js import ALPINE_JS
from pico_css import PICO_CSS

HTML_TEMPLATE = """\
<!DOCTYPE html>
<html lang="en" data-theme="dark">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Diffusion Planner Model Comparison</title>
  <style>%(pico_css)s</style>
  <style>
    canvas { display: block; cursor: grab; background: #1a1a2e; }
    canvas:active { cursor: grabbing; }
    .controls { display: flex; align-items: center; gap: .5rem; }
    .controls > * { margin-bottom: 0; }
    .controls button { width: auto; }
    .controls input[type=range],
    .controls input[type=range]::-webkit-slider-runnable-track,
    .controls input[type=range]::-webkit-slider-thumb { all: revert; }
    .controls input[type=range] { flex: 1; }
    .swatch { display: inline-block; width: .9em; height: .9em; border-radius: 3px; }
  </style>
</head>
<body>
  <main class="container" x-data="reportApp()" x-init="init()">
    <hgroup>
      <h1>Diffusion Planner Model Comparison</h1>
      <p x-text="data.models.length + ' model(s)'"></p>
    </hgroup>

    <canvas x-ref="canvas" height="600"
      @mousedown="onMouseDown($event)"
      @mousemove="onMouseMove($event)"
      @mouseup="onMouseUp()"
      @mouseleave="onMouseUp()"
      @wheel.prevent="onWheel($event)"></canvas>

    <div class="controls">
      <button @click="togglePlay()" x-text="playing ? '\\u23F8' : '\\u25B6'"></button>
      <input type="range" min="0" :max="maxTime || 1" step="0.001"
        :value="currentTime"
        @pointerdown="if (playing) togglePlay()"
        @input="currentTime = +$event.target.value; draw()">
      <small x-text="currentTime.toFixed(1) + 's / ' + maxTime.toFixed(1) + 's'"></small>
      <button class="secondary" @click="fitToView(); draw()">Fit</button>
    </div>

    <template x-for="(model, mi) in data.models" :key="mi">
      <label>
        <input type="checkbox" checked
          @change="visible[mi] = $el.checked; draw()">
        <span class="swatch"
          :style="'background:hsl(' + hues[mi %% hues.length] + ',80%%,50%%)'"></span>
        <span x-text="model.name"></span>
      </label>
    </template>
  </main>

  <script>const REPORT_DATA = %(data_json)s;</script>
  <script>
  function reportApp() {
    const HUES = [210, 0, 120, 45, 270];
    const cam = { cx: 0, cy: 0, scale: 1 };
    const drag = { active: false, sx: 0, sy: 0, cx0: 0, cy0: 0 };
    let animId = null, lastAnimTime = 0;
    let ctx = null;
    let canvas = null;

    function wx(px) { return (px - cam.cx) * cam.scale + canvas.width / 2; }
    function wy(py) { return -(py - cam.cy) * cam.scale + canvas.height / 2; }

    function polyline(pts) {
      if (pts.length < 2) return;
      ctx.beginPath();
      ctx.moveTo(wx(pts[0][0]), wy(pts[0][1]));
      for (let i = 1; i < pts.length; i++) ctx.lineTo(wx(pts[i][0]), wy(pts[i][1]));
      ctx.stroke();
    }

    function drawMap(map) {
      ctx.strokeStyle = '#445'; ctx.lineWidth = 1;
      for (const ll of map.lanelets) { polyline(ll.left); polyline(ll.right); }
      for (const rm of map.road_markings) {
        if (rm.type === 'stop_line') { ctx.strokeStyle = '#c44'; ctx.lineWidth = 2; }
        else { ctx.strokeStyle = '#556'; ctx.lineWidth = 1; }
        polyline(rm.points);
      }
    }

    function velColor(vel, hue) {
      const l = 30 + Math.min(Math.abs(vel), 2.0) * 20;
      return 'hsla(' + hue + ',80%%,' + Math.round(l) + '%%,0.55)';
    }

    function closestFrame(trajs, t) {
      if (!trajs.length) return null;
      let lo = 0, hi = trajs.length - 1;
      while (lo < hi) { const m = (lo + hi + 1) >> 1; trajs[m].t <= t ? lo = m : hi = m - 1; }
      return trajs[lo];
    }

    return {
      data: REPORT_DATA,
      playing: false,
      currentTime: 0,
      maxTime: 0,
      visible: {},
      hues: HUES,

      init() {
        canvas = this.$refs.canvas;
        ctx = canvas.getContext('2d');
        for (let i = 0; i < this.data.models.length; i++) {
          this.visible[i] = true;
          const t = this.data.models[i].trajectories;
          if (t.length) this.maxTime = Math.max(this.maxTime, t[t.length - 1].t);
        }
        this.resizeCanvas();
        this.fitToView();
        this.draw();
        window.addEventListener('resize', () => { this.resizeCanvas(); this.fitToView(); this.draw(); });
      },

      resizeCanvas() {
        const w = canvas.parentElement.clientWidth;
        canvas.width = w;
        canvas.height = Math.max(Math.round(w * 0.55), 400);
      },

      fitToView() {
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        for (const ll of this.data.map.lanelets) {
          for (const side of [ll.left, ll.right]) {
            for (const p of side) {
              if (p[0] < minX) minX = p[0]; if (p[0] > maxX) maxX = p[0];
              if (p[1] < minY) minY = p[1]; if (p[1] > maxY) maxY = p[1];
            }
          }
        }
        if (!isFinite(minX)) return;
        cam.cx = (minX + maxX) / 2;
        cam.cy = (minY + maxY) / 2;
        cam.scale = Math.min(canvas.width / ((maxX - minX) || 1),
                             canvas.height / ((maxY - minY) || 1)) * 0.9;
      },

      draw() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = '#1a1a2e';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        drawMap(this.data.map);
        ctx.lineWidth = 3; ctx.lineCap = 'round'; ctx.lineJoin = 'round';
        for (let mi = 0; mi < this.data.models.length; mi++) {
          if (!this.visible[mi]) continue;
          const frame = closestFrame(this.data.models[mi].trajectories, this.currentTime);
          if (!frame) continue;
          const hue = HUES[mi %% HUES.length];
          const pts = frame.points;
          for (let i = 0; i < pts.length - 1; i++) {
            ctx.strokeStyle = velColor(pts[i][2], hue);
            ctx.beginPath();
            ctx.moveTo(wx(pts[i][0]), wy(pts[i][1]));
            ctx.lineTo(wx(pts[i+1][0]), wy(pts[i+1][1]));
            ctx.stroke();
          }
        }
      },

      togglePlay() {
        this.playing = !this.playing;
        if (this.playing) { lastAnimTime = performance.now(); this.animate(); }
        else if (animId) { cancelAnimationFrame(animId); animId = null; }
      },

      animate() {
        if (!this.playing) return;
        const now = performance.now();
        this.currentTime += (now - lastAnimTime) / 1000;
        lastAnimTime = now;
        if (this.currentTime > this.maxTime) this.currentTime = 0;
        this.draw();
        animId = requestAnimationFrame(() => this.animate());
      },

      onMouseDown(e) {
        drag.active = true;
        drag.sx = e.clientX; drag.sy = e.clientY;
        drag.cx0 = cam.cx; drag.cy0 = cam.cy;
      },
      onMouseMove(e) {
        if (!drag.active) return;
        cam.cx = drag.cx0 - (e.clientX - drag.sx) / cam.scale;
        cam.cy = drag.cy0 + (e.clientY - drag.sy) / cam.scale;
        this.draw();
      },
      onMouseUp() { drag.active = false; },

      onWheel(e) {
        const f = e.deltaY > 0 ? 0.9 : 1.1;
        const r = canvas.getBoundingClientRect();
        const mx = e.clientX - r.left, my = e.clientY - r.top;
        const wxx = (mx - canvas.width / 2) / cam.scale + cam.cx;
        const wyy = -(my - canvas.height / 2) / cam.scale + cam.cy;
        cam.scale *= f;
        cam.cx = wxx - (mx - canvas.width / 2) / cam.scale;
        cam.cy = wyy + (my - canvas.height / 2) / cam.scale;
        this.draw();
      },
    };
  }
  </script>
  <script>%(alpine_js)s</script>
</body>
</html>
"""


def generate_report(output_dir):
    """Generate self-contained HTML comparison reports.

    Discovers models from ``output_dir/staging/`` (symlink to raw bags).
    Falls back to single-model discovery when staging is absent.
    Reports are written to ``output_dir/result_archive/``.
    """
    from rosbag_reader import extract_map_data, extract_trajectories

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
        html = HTML_TEMPLATE % {
            "pico_css": PICO_CSS,
            "alpine_js": ALPINE_JS,
            "data_json": json.dumps(data, separators=(",", ":")),
        }

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
