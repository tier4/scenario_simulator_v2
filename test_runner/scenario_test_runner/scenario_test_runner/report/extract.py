"""Extract embedded JSON data from scenario_test_runner HTML reports.

The report template stores its payload inside a tag:
    <script type="application/gzip+base64" id="report-data">…</script>

This module provides helpers to decode that payload and compute the same
Numerical Comparison metrics that the in-browser JS produces, so that
index pages can aggregate them across scenarios.
"""

from __future__ import annotations

import base64
import gzip
import json
import math
from html.parser import HTMLParser
from pathlib import Path
from statistics import median
from typing import Any


class _ScriptTagParser(HTMLParser):
    """Extract text content of a ``<script>`` tag by its ``id``."""

    def __init__(self, script_id: str) -> None:
        super().__init__()
        self._target_id = script_id
        self._inside = False
        self.data: str | None = None

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag == "script" and dict(attrs).get("id") == self._target_id:
            self._inside = True
            self.data = ""

    def handle_data(self, data: str) -> None:
        if self._inside:
            assert self.data is not None
            self.data += data

    def handle_endtag(self, tag: str) -> None:
        if self._inside and tag == "script":
            self._inside = False


def extract_json(html_path: Path) -> dict[str, Any]:
    """Decode the embedded report data from an HTML file and return parsed JSON."""
    parser = _ScriptTagParser("report-data")
    parser.feed(html_path.read_text(encoding="utf-8"))
    if parser.data is None:
        raise ValueError(f"No <script id='report-data'> found in {html_path}")
    raw = base64.b64decode(parser.data.strip())
    return json.loads(gzip.decompress(raw))


def extract_index_data(html_path: Path) -> dict[str, Any]:
    """Extract the embedded index-data JSON from an index.html file."""
    parser = _ScriptTagParser("index-data")
    parser.feed(html_path.read_text(encoding="utf-8"))
    if parser.data is None:
        raise ValueError(f"No <script id='index-data'> found in {html_path}")
    return json.loads(parser.data.strip())


# ---------------------------------------------------------------------------
# Numerical Comparison — mirrors the JS logic in report_template.html
# ---------------------------------------------------------------------------

_LATERAL_THRESH = 15.0
_BRAKE_THRESHOLD = -1.0
_TURN_LEFT, _TURN_RIGHT = 2, 3


def _interp_at_s(m: dict, s: float) -> dict | None:
    """Interpolate position and velocity of model *m* at arc-length *s*."""
    nv = m.get("n_valid", 0)
    if nv < 2:
        return None
    sc = min(max(s, 0), m["s_total"])
    sarr = m["s"]
    lo, hi = 0, nv
    while lo < hi:
        mid = (lo + hi) >> 1
        if sarr[mid] <= sc:
            lo = mid + 1
        else:
            hi = mid
    j = min(max(lo, 1), nv - 1)
    i = j - 1
    ds = sarr[j] - sarr[i]
    f = (sc - sarr[i]) / ds if ds > 1e-9 else 0.0
    return {
        "x": m["x"][i] + (m["x"][j] - m["x"][i]) * f,
        "y": m["y"][i] + (m["y"][j] - m["y"][i]) * f,
        "v": m["v"][i] + (m["v"][j] - m["v"][i]) * f,
    }


def _find_turn_clusters(m: dict) -> list[dict]:
    cmd = (m.get("ch") or {}).get("turn_cmd")
    if not cmd:
        return []
    clusters: list[dict] = []
    cur = None
    for k in range(m.get("n_valid", 0)):
        c = cmd[k]
        if c in (_TURN_LEFT, _TURN_RIGHT):
            if cur is None or cur["dir"] != c:
                if cur:
                    clusters.append(cur)
                cur = {"dir": c, "s0": m["s"][k], "s1": m["s"][k]}
            else:
                cur["s1"] = m["s"][k]
        else:
            if cur:
                clusters.append(cur)
                cur = None
    if cur:
        clusters.append(cur)
    return clusters


def _compare_turn_indicators(
    base_clusters: list[dict], cand_clusters: list[dict],
) -> dict:
    matched = 0
    missing = 0
    iou_sum = 0.0
    for bc in base_clusters:
        best_iou = 0.0
        for cc in cand_clusters:
            if cc["dir"] != bc["dir"]:
                continue
            i0 = max(bc["s0"], cc["s0"])
            i1 = min(bc["s1"], cc["s1"])
            if i1 <= i0:
                continue
            inter = i1 - i0
            union = (bc["s1"] - bc["s0"]) + (cc["s1"] - cc["s0"]) - inter
            iou = inter / union if union > 0 else 0.0
            if iou > best_iou:
                best_iou = iou
        if best_iou > 0:
            matched += 1
            iou_sum += best_iou
        else:
            missing += 1
    return {
        "clusters": len(base_clusters),
        "missing": missing,
        "mean_iou": iou_sum / matched if matched > 0 else float("nan"),
    }


def _count_strong_brakes(m: dict) -> int:
    accel = (m.get("ch") or {}).get("accel")
    if not accel:
        return 0
    count = 0
    braking = False
    for k in range(m.get("n_valid", 0)):
        if accel[k] is not None and accel[k] <= _BRAKE_THRESHOLD:
            if not braking:
                count += 1
                braking = True
        else:
            braking = False
    return count


def _candidate_raw(cand: dict, base: dict) -> dict:
    """Return raw comparison arrays for a candidate model vs the base."""
    s_max = min(base["s_total"], cand["s_total"])
    laterals: list[float] = []
    delta_vs: list[float] = []
    for k in range(base.get("n_valid", 0)):
        bs = base["s"][k]
        if bs > s_max:
            break
        cs = _interp_at_s(cand, bs)
        if cs is None:
            continue
        dx = cs["x"] - base["x"][k]
        dy = cs["y"] - base["y"][k]
        laterals.append(math.sqrt(dx * dx + dy * dy))
        delta_vs.append(cs["v"] - (base["v"][k] or 0))

    ti = _compare_turn_indicators(
        _find_turn_clusters(base), _find_turn_clusters(cand),
    )
    ti_matched = ti["clusters"] - ti["missing"]
    return {
        "model": cand.get("name", ""),
        "color": cand.get("color", ""),
        "is_reference": False,
        "laterals": laterals,
        "delta_vs": delta_vs,
        "ti_clusters": ti["clusters"],
        "ti_missing": ti["missing"],
        "ti_matched": ti_matched,
        "ti_iou_sum": ti["mean_iou"] * ti_matched if ti_matched > 0 and math.isfinite(ti["mean_iou"]) else 0.0,
        "strong_brakes": _count_strong_brakes(cand),
    }


def _reference_raw(base: dict) -> dict:
    """Return raw data for the reference model."""
    return {
        "model": base.get("name", ""),
        "color": base.get("color", ""),
        "is_reference": True,
        "laterals": [],
        "delta_vs": [],
        "ti_clusters": 0,
        "ti_missing": 0,
        "ti_matched": 0,
        "ti_iou_sum": 0.0,
        "strong_brakes": _count_strong_brakes(base),
    }


def _row_from_raw(raw: dict) -> dict:
    """Compute a finished numcomp row dict from raw arrays."""
    if raw["is_reference"]:
        return {
            "model": raw["model"],
            "color": raw["color"],
            "match_valid_pct": 100.0,
            "median_lateral_m": 0.0,
            "max_lateral_m": 0.0,
            "mean_dv": 0.0,
            "median_dv": 0.0,
            "avg_over": 0.0,
            "avg_under": 0.0,
            "peak_over": 0.0,
            "peak_under": 0.0,
            "ti_clusters": float("nan"),
            "ti_missing": float("nan"),
            "ti_mean_iou": float("nan"),
            "strong_brakes": raw["strong_brakes"],
        }
    laterals = raw["laterals"]
    delta_vs = raw["delta_vs"]
    n = len(delta_vs)
    valid_cnt = sum(1 for l in laterals if l <= _LATERAL_THRESH)
    over_sum = sum(dv for dv in delta_vs if dv > 0)
    under_sum = sum(dv for dv in delta_vs if dv < 0)
    ti_matched = raw["ti_matched"]
    return {
        "model": raw["model"],
        "color": raw["color"],
        "match_valid_pct": (valid_cnt / n) * 100 if n > 0 else 0,
        "median_lateral_m": median(laterals) if laterals else float("nan"),
        "max_lateral_m": max(laterals) if laterals else float("nan"),
        "mean_dv": sum(delta_vs) / n if n > 0 else float("nan"),
        "median_dv": median(delta_vs) if delta_vs else float("nan"),
        "avg_over": over_sum / n if n > 0 else 0,
        "avg_under": under_sum / n if n > 0 else 0,
        "peak_over": max(delta_vs) if delta_vs else float("nan"),
        "peak_under": min(delta_vs) if delta_vs else float("nan"),
        "ti_clusters": raw["ti_clusters"],
        "ti_missing": raw["ti_missing"],
        "ti_mean_iou": raw["ti_iou_sum"] / ti_matched if ti_matched > 0 else float("nan"),
        "strong_brakes": raw["strong_brakes"],
    }


def numerical_comparison_raw(
    data: dict[str, Any], base_idx: int = 0,
) -> list[dict]:
    """Return per-model raw comparison data for one scenario.

    Each dict contains ``laterals`` and ``delta_vs`` arrays plus turn-indicator
    and braking summaries, suitable for cross-scenario aggregation via
    :func:`aggregate_numcomp_raw`.
    """
    models = data.get("models", [])
    if not models:
        return []
    base = models[base_idx]
    rows = [_reference_raw(base)]
    for i, m in enumerate(models):
        if i == base_idx:
            continue
        rows.append(_candidate_raw(m, base))
    return rows


def numerical_comparison(data: dict[str, Any], base_idx: int = 0) -> list[dict]:
    """Compute the Numerical Comparison table rows for one scenario."""
    return [_row_from_raw(r) for r in numerical_comparison_raw(data, base_idx)]


def aggregate_numcomp_raw(raw_lists: list[list[dict]]) -> list[dict]:
    """Aggregate raw comparison data across scenarios and compute final rows.

    Concatenates laterals / delta_vs arrays, sums counts, then derives
    median / max / mean etc. from the combined distribution.
    """
    model_order: list[str] = []
    buckets: dict[str, dict] = {}

    for raw_list in raw_lists:
        for raw in raw_list:
            name = raw["model"]
            if name not in buckets:
                model_order.append(name)
                buckets[name] = {
                    "model": name,
                    "color": raw["color"],
                    "is_reference": raw["is_reference"],
                    "laterals": [],
                    "delta_vs": [],
                    "ti_clusters": 0,
                    "ti_missing": 0,
                    "ti_matched": 0,
                    "ti_iou_sum": 0.0,
                    "strong_brakes": 0,
                }
            b = buckets[name]
            b["laterals"].extend(raw["laterals"])
            b["delta_vs"].extend(raw["delta_vs"])
            b["ti_clusters"] += raw["ti_clusters"]
            b["ti_missing"] += raw["ti_missing"]
            b["ti_matched"] += raw["ti_matched"]
            b["ti_iou_sum"] += raw["ti_iou_sum"]
            b["strong_brakes"] += raw["strong_brakes"]

    return [_row_from_raw(buckets[name]) for name in model_order]


# Column definitions matching the JS GROUPS in report_template.html
NUMCOMP_GROUPS = [
    {"header": "Correspondence", "cols": [
        {"key": "match_valid_pct", "label": "Valid", "unit": "%",
         "tip": "Share of samples with lateral distance ≤ 15 m"},
        {"key": "median_lateral_m", "label": "Median lateral", "unit": "m",
         "tip": "Median lateral distance at arc-length matched points"},
        {"key": "max_lateral_m", "label": "Max lateral", "unit": "m",
         "tip": "Maximum lateral distance at arc-length matched points"},
    ]},
    {"header": "Speed difference", "cols": [
        {"key": "mean_dv", "label": "Mean Δv", "unit": "m/s",
         "tip": "Mean speed difference (model − baseline)"},
        {"key": "median_dv", "label": "Median Δv", "unit": "m/s",
         "tip": "Median speed difference (model − baseline)"},
        {"key": "avg_over", "label": "Avg overspeed", "unit": "m/s",
         "tip": "Mean of positive Δv (model faster than baseline)"},
        {"key": "avg_under", "label": "Avg underspeed", "unit": "m/s",
         "tip": "Mean of negative Δv (model slower than baseline)"},
        {"key": "peak_over", "label": "Peak Δv+", "unit": "m/s",
         "tip": "Largest positive Δv (max excess over baseline)"},
        {"key": "peak_under", "label": "Peak Δv−", "unit": "m/s",
         "tip": "Largest negative Δv (max deficit vs baseline)"},
    ]},
    {"header": "Turn indicator", "cols": [
        {"key": "ti_clusters", "label": "Clusters",
         "tip": "Turn-indicator clusters on the baseline path"},
        {"key": "ti_missing", "label": "Missing",
         "tip": "Baseline clusters with no matching model signal"},
        {"key": "ti_mean_iou", "label": "Mean IoU",
         "tip": "Arc-length IoU of matched turn-indicator clusters"},
    ]},
    {"header": "Operational", "cols": [
        {"key": "strong_brakes", "label": "Strong brakes",
         "tip": "Consecutive intervals with acceleration ≤ −1.0 m/s²"},
    ]},
]


def fmt_val(v: Any) -> str:
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return "—"
    if isinstance(v, (int, float)):
        if isinstance(v, int) or v == int(v):
            return str(int(v))
        if abs(v) < 10:
            return f"{v:.3f}"
        return f"{v:.1f}"
    return str(v)
