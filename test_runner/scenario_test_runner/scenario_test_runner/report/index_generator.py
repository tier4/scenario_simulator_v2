"""Generate hierarchical index HTML pages for downloaded scenario reports.

Produces two levels of index pages:
    - Catalog index (output_dir/index.html)              — lists suites
    - Suite index   (output_dir/<suite>/index.html)       — lists scenarios

Each page is rendered from a single ``index_template.html`` with embedded JSON
data and Alpine.js for client-side rendering.

Usage (regenerate from existing data):
    python -m scenario_test_runner.report.index_generator ./reports
"""

from __future__ import annotations

if __name__ == "__main__" and __package__ is None:
    import sys as _sys
    from pathlib import Path as _Path

    _pkg_parent = str(_Path(__file__).resolve().parent.parent.parent)
    if _pkg_parent not in _sys.path:
        _sys.path.insert(0, _pkg_parent)
    __package__ = "scenario_test_runner.report"

import json
import math
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory

from .alpine_js import ALPINE_JS
from .extract import (
    NUMCOMP_GROUPS,
    aggregate_numcomp_raw,
    extract_index_data,
    extract_json,
    numerical_comparison_raw,
)

_TEMPLATES_DIR = (
    Path(get_package_share_directory("scenario_test_runner")) / "templates"
)
_INDEX_TEMPLATE_PATH = _TEMPLATES_DIR / "index_template.html"


def _load_template() -> str:
    if not _INDEX_TEMPLATE_PATH.exists():
        raise FileNotFoundError(f"Template not found: {_INDEX_TEMPLATE_PATH}")
    return _INDEX_TEMPLATE_PATH.read_text(encoding="utf-8")


def _alpine_script_tag() -> str:
    return f"<script>{ALPINE_JS}</script>"


def _prepared_template() -> str:
    return _load_template().replace("{{ALPINE_SCRIPT}}", _alpine_script_tag())


def _render(template: str, data: dict[str, Any]) -> str:
    return template.replace(
        "{{INDEX_DATA}}", json.dumps(data, separators=(",", ":")),
    )


# ---------------------------------------------------------------------------
# Initial generation (called from download_reports.py)
# ---------------------------------------------------------------------------

def _count_statuses(items: list[dict[str, Any]], key: str = "status") -> tuple[int, int]:
    ok = sum(1 for i in items if i.get(key) == "succeeded")
    fail = sum(1 for i in items if i.get(key) == "failed")
    return ok, fail


def _numcomp_rows(scenarios: list[dict[str, Any]]) -> tuple[list[dict], int]:
    raw_lists = [sc["numcomp_raw"] for sc in scenarios if sc.get("numcomp_raw")]
    if not raw_lists:
        return [], 0
    rows = aggregate_numcomp_raw(raw_lists)
    return rows, len(raw_lists)


def _sanitize_for_json(v: Any) -> Any:
    if isinstance(v, float) and not math.isfinite(v):
        return None
    return v


def _clean_rows(rows: list[dict]) -> list[dict]:
    return [{k: _sanitize_for_json(v) for k, v in row.items()} for row in rows]


def _scan_scenario(
    scenario_dir: Path, status: str = "unknown",
) -> dict[str, Any] | None:
    report_path = scenario_dir / "report.html"
    if not report_path.exists():
        return None

    numcomp_raw: list[dict[str, Any]] = []
    try:
        data = extract_json(report_path)
        numcomp_raw = numerical_comparison_raw(data)
    except Exception:
        pass

    return {
        "name": scenario_dir.name,
        "status": status,
        "has_report": True,
        "numcomp_raw": numcomp_raw,
    }


def _derive_suite_status(scenarios: list[dict[str, Any]]) -> str:
    if any(s["status"] == "failed" for s in scenarios):
        return "failed"
    if all(s["status"] == "succeeded" for s in scenarios):
        return "succeeded"
    return "unknown"


def _scan_output_dir(
    output_dir: Path,
    statuses: dict[str, dict[str, str]],
) -> dict[str, dict[str, Any]]:
    suites: dict[str, dict[str, Any]] = {}
    for suite_name, sc_statuses in sorted(statuses.items()):
        scenarios = []
        suite_dir = output_dir / suite_name
        for scenario_name, status in sorted(sc_statuses.items()):
            sc_dir = suite_dir / scenario_name
            if sc_dir.is_dir():
                sc = _scan_scenario(sc_dir, status)
                if sc is not None:
                    scenarios.append(sc)
                else:
                    scenarios.append({
                        "name": scenario_name,
                        "status": status,
                        "has_report": False,
                        "numcomp_raw": [],
                    })
        if scenarios:
            suites[suite_name] = {
                "status": _derive_suite_status(scenarios),
                "scenarios": scenarios,
            }
    return suites


def _catalog_data(
    title: str, suites: dict[str, dict[str, Any]], job_id: str,
) -> dict[str, Any]:
    items = []
    for suite_name, suite in sorted(suites.items()):
        scenarios = suite["scenarios"]
        ok, fail = _count_statuses(scenarios)
        items.append({
            "name": suite_name,
            "status": suite.get("status", "unknown"),
            "total": len(scenarios),
            "succeeded": ok,
            "failed": fail,
        })
    all_scenarios = [
        sc for suite in suites.values() for sc in suite["scenarios"]
    ]
    numcomp, numcomp_count = _numcomp_rows(all_scenarios)
    return {
        "mode": "catalog",
        "title": title,
        "subtitle": job_id,
        "breadcrumbs": [],
        "items": items,
        "numcomp": _clean_rows(numcomp),
        "numcomp_count": numcomp_count,
    }


def _suite_data(
    catalog_name: str, suite_name: str, suite: dict[str, Any],
) -> dict[str, Any]:
    items = []
    for sc in suite["scenarios"]:
        items.append({
            "name": sc["name"],
            "status": sc.get("status", "unknown"),
            "has_report": sc.get("has_report", False),
        })
    numcomp, numcomp_count = _numcomp_rows(suite["scenarios"])
    return {
        "mode": "suite",
        "title": suite_name,
        "subtitle": "",
        "breadcrumbs": [{"label": catalog_name, "href": "../index.html"}],
        "items": items,
        "numcomp": _clean_rows(numcomp),
        "numcomp_count": numcomp_count,
    }


def generate_indices(
    output_dir: Path,
    *,
    job_id: str,
    statuses: dict[str, dict[str, str]],
) -> None:
    """Build index.html files from scratch (called after downloading reports)."""
    template = _prepared_template()

    suites = _scan_output_dir(output_dir, statuses)
    cat_name = output_dir.name

    cat_data = _catalog_data(cat_name, suites, job_id)
    (output_dir / "index.html").write_text(
        _render(template, cat_data), encoding="utf-8",
    )

    for suite_name, suite in suites.items():
        suite_dir = output_dir / suite_name
        suite_dir.mkdir(parents=True, exist_ok=True)
        s_data = _suite_data(cat_name, suite_name, suite)
        (suite_dir / "index.html").write_text(
            _render(template, s_data), encoding="utf-8",
        )


# ---------------------------------------------------------------------------
# Regeneration (template swap, data preserved)
# ---------------------------------------------------------------------------

def regenerate_indices(output_dir: Path) -> None:
    """Re-render all index.html files with the current template.

    Reads the embedded JSON from each existing index.html and writes it
    back into the latest template.  Data is untouched.
    """
    template = _prepared_template()

    for index_html in output_dir.rglob("index.html"):
        data = extract_index_data(index_html)
        index_html.write_text(
            _render(template, data), encoding="utf-8",
        )


if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        description="Re-render index pages with the current template.",
    )
    parser.add_argument("output_dir", type=Path, help="Reports directory")
    args = parser.parse_args()

    if not args.output_dir.is_dir():
        print(f"Not a directory: {args.output_dir}", file=sys.stderr)
        sys.exit(1)

    regenerate_indices(args.output_dir)
    print(f"Done. Open {args.output_dir / 'index.html'} to browse.")
