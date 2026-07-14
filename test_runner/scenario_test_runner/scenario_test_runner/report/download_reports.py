#!/usr/bin/env python3
"""Download scenario HTML reports from WebAuto and generate index pages.

IO-bound work (webauto CLI calls, ZIP downloads) is parallelised with
``ThreadPoolExecutor``.  Processing is split into batch phases so that
each phase contains uniform work:

  Phase 1 – Metadata collection  (IO: parallel CLI list calls)
  Phase 2 – Archive download     (IO: parallel CLI downloads)
  Phase 3 – Index generation     (CPU, delegated to index_generator)

Usage:
    python -m scenario_test_runner.report.download_reports \\
        --project-id x2_dev \\
        --evaluation-job-id <UUID> \\
        --output-dir ./reports
"""

from __future__ import annotations

if __name__ == "__main__" and __package__ is None:
    import sys as _sys
    from pathlib import Path as _Path

    _pkg_parent = str(_Path(__file__).resolve().parent.parent.parent)
    if _pkg_parent not in _sys.path:
        _sys.path.insert(0, _pkg_parent)
    __package__ = "scenario_test_runner.report"

import argparse
import dataclasses
import json
import subprocess
import tempfile
import threading
import zipfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any

from .index_generator import generate_indices

_CLI_TIMEOUT = 600


# -- Data classes --


@dataclasses.dataclass(frozen=True)
class CaseInfo:
    suite_name: str
    suite_id: str
    scenario_name: str
    status: str
    log_id: str | None


@dataclasses.dataclass(frozen=True)
class DownloadResult:
    case: CaseInfo
    report_path: Path | None
    error: str | None


# -- webauto CLI helpers (thread-safe: only local state + subprocess) --


def _webauto_json(args: list[str]) -> dict[str, Any]:
    """Run a webauto CLI command with --output json and return parsed dict."""
    cmd = ["webauto", *args, "--output", "json"]
    try:
        out = subprocess.run(
            cmd, capture_output=True, text=True, timeout=_CLI_TIMEOUT, check=False,
        )
    except subprocess.TimeoutExpired as e:
        raise RuntimeError(f"webauto timeout: {' '.join(cmd)}") from e
    if out.returncode != 0:
        raise RuntimeError(
            f"webauto failed (rc={out.returncode}): {' '.join(cmd)}\n{out.stderr.strip()}"
        )
    stdout = out.stdout
    start = min(
        (i for i in (stdout.find("{"), stdout.find("[")) if i >= 0), default=-1,
    )
    try:
        return json.loads(stdout[start:]) if start >= 0 else json.loads(stdout)
    except json.JSONDecodeError as e:
        raise RuntimeError(
            f"Failed to parse webauto JSON: {' '.join(cmd)}\n{stdout[:500]}"
        ) from e


def _webauto_list(
    resource: str, project_id: str, extra_args: list[str],
) -> list[dict[str, Any]]:
    """Paginate through a webauto list command and collect all items."""
    items: list[dict[str, Any]] = []
    page_token: str | None = None
    while True:
        args = [
            "ci", resource, "list",
            "--project-id", project_id,
            "--page-size", "100",
            *extra_args,
        ]
        if page_token:
            args.extend(["--page-token", page_token])
        result = _webauto_json(args)
        key = next(
            (k for k in result if isinstance(result[k], list)), None,
        )
        if key:
            items.extend(result[key])
        page_token = result.get("nextToken") or result.get("next_token")
        if not page_token:
            break
    return items


def _download_archive(
    project_id: str, log_id: str, target_dir: Path,
) -> Path:
    """Download a simulation archive ZIP and return its path."""
    cmd = [
        "webauto", "ci", "evaluation-log", "download",
        "--project-id", project_id,
        "--log-id", log_id,
        "--target-dir", str(target_dir),
        "--force",
    ]
    subprocess.run(cmd, capture_output=True, text=True, timeout=_CLI_TIMEOUT, check=True)
    zips = list(target_dir.glob("*.zip"))
    if not zips:
        raise FileNotFoundError(f"No ZIP downloaded to {target_dir}")
    return zips[0]


def _sanitize(name: str) -> str:
    """Sanitize a display name for use as a directory name."""
    return name.replace("/", "_").replace("\\", "_").replace("\0", "").strip()


def _find_report_in_zip(zf: zipfile.ZipFile) -> str | None:
    """Find the report HTML file inside a simulation archive ZIP."""
    for name in zf.namelist():
        if name.endswith("report.html") or name.endswith(".html"):
            return name
    return None


def _fetch_catalog_name(project_id: str, job_id: str) -> str:
    """Fetch the catalog display name for an evaluation job."""
    result = _webauto_json([
        "ci", "evaluation-job-report", "describe",
        "--project-id", project_id,
        "--evaluation-job-id", job_id,
    ])
    return result.get("catalog", {}).get("display_name", "")


# -- Phase 1: Metadata collection (IO-bound, parallel) --


def _list_specs_for_suite(
    project_id: str, job_id: str, suite_report: dict[str, Any],
) -> tuple[str, str, list[dict[str, Any]]]:
    """List all specs for one suite. Returns (suite_name, suite_id, specs)."""
    suite_id = suite_report["id"]
    suite_name = _sanitize(suite_report.get("suite", {}).get("display_name", suite_id))
    specs = _webauto_list(
        "evaluation-job-spec-report", project_id,
        ["--evaluation-job-id", job_id, "--evaluation-job-suite-report-id", suite_id],
    )
    return suite_name, suite_id, specs


def _list_cases_for_spec(
    project_id: str, job_id: str, spec: dict[str, Any],
    suite_name: str, suite_id: str,
) -> list[CaseInfo]:
    """List all cases for one spec. Returns list of CaseInfo."""
    raw_cases = _webauto_list(
        "evaluation-job-case-report", project_id,
        ["--evaluation-job-id", job_id, "--evaluation-job-spec-report-id", spec["id"]],
    )
    return [
        CaseInfo(
            suite_name=suite_name,
            suite_id=suite_id,
            scenario_name=_sanitize(
                c.get("scenario", {}).get("display_name", c["id"]),
            ),
            status=c.get("status", "unknown"),
            log_id=(c.get("logs") or {}).get("simulation_archive", {}).get("id"),
        )
        for c in raw_cases
    ]


def _collect_all_cases(
    project_id: str, job_id: str, executor: ThreadPoolExecutor,
) -> tuple[list[CaseInfo], str]:
    """Collect metadata for all cases. Three batches of parallel CLI calls:

    1. catalog name + suite listing
    2. spec listing for all suites
    3. case listing for all specs (flattened)
    """
    catalog_future = executor.submit(_fetch_catalog_name, project_id, job_id)

    print("Fetching suite list...", flush=True)
    suites = _webauto_list(
        "evaluation-job-suite-report", project_id,
        ["--evaluation-job-id", job_id],
    )
    print(f"  {len(suites)} suite(s) found", flush=True)

    spec_futures = {
        executor.submit(_list_specs_for_suite, project_id, job_id, sr): sr
        for sr in suites
    }
    spec_tasks: list[tuple[str, str, dict[str, Any]]] = []
    for future in as_completed(spec_futures):
        try:
            suite_name, suite_id, specs = future.result()
            for spec in specs:
                spec_tasks.append((suite_name, suite_id, spec))
            print(f"  suite '{suite_name}': {len(specs)} spec(s)", flush=True)
        except Exception as e:
            sr = spec_futures[future]
            print(f"  Failed to list specs for suite "
                  f"{sr.get('id', '?')}: {e}", flush=True)

    case_futures = {
        executor.submit(
            _list_cases_for_spec, project_id, job_id, spec,
            suite_name, suite_id,
        ): (suite_name, spec)
        for suite_name, suite_id, spec in spec_tasks
    }
    all_cases: list[CaseInfo] = []
    for future in as_completed(case_futures):
        try:
            all_cases.extend(future.result())
        except Exception as e:
            suite_name, spec = case_futures[future]
            print(f"  Failed to list cases for spec "
                  f"{spec.get('id', '?')} in '{suite_name}': {e}", flush=True)

    catalog_name = catalog_future.result()
    print(f"  {len(all_cases)} case(s) total, catalog='{catalog_name}'", flush=True)
    return all_cases, catalog_name


# -- Phase 2: Archive download (IO-bound, parallel) --


def _download_single_report(
    project_id: str, case: CaseInfo, output_dir: Path,
) -> DownloadResult:
    """Download and extract one report. Thread-safe."""
    scenario_dir = output_dir / case.suite_name / case.scenario_name
    scenario_dir.mkdir(parents=True, exist_ok=True)

    if not case.log_id:
        return DownloadResult(case=case, report_path=None, error=None)

    try:
        with tempfile.TemporaryDirectory() as tmp:
            zip_path = _download_archive(project_id, case.log_id, Path(tmp))
            with zipfile.ZipFile(zip_path) as zf:
                html_name = _find_report_in_zip(zf)
                if not html_name:
                    return DownloadResult(case=case, report_path=None,
                                         error="No report.html in archive")
                zf.extract(html_name, scenario_dir)
                extracted = scenario_dir / html_name
                dest = scenario_dir / "report.html"
                if extracted != dest:
                    extracted.rename(dest)
                    for p in scenario_dir.iterdir():
                        if p.is_dir() and not any(p.iterdir()):
                            p.rmdir()
                return DownloadResult(case=case, report_path=dest, error=None)
    except Exception as e:
        return DownloadResult(case=case, report_path=None, error=str(e))


def _download_all_reports(
    project_id: str, cases: list[CaseInfo], output_dir: Path,
    executor: ThreadPoolExecutor,
) -> list[DownloadResult]:
    """Download all archives in parallel."""
    downloadable = [c for c in cases if c.log_id]
    results: list[DownloadResult] = [
        DownloadResult(case=c, report_path=None, error=None)
        for c in cases if not c.log_id
    ]

    if not downloadable:
        return results

    total = len(downloadable)
    lock = threading.Lock()
    counter = [0]

    futures = {
        executor.submit(_download_single_report, project_id, c, output_dir): c
        for c in downloadable
    }

    print(f"\nDownloading {total} archive(s)...", flush=True)
    for future in as_completed(futures):
        result = future.result()
        results.append(result)
        with lock:
            counter[0] += 1
            n = counter[0]
        label = f"{result.case.suite_name}/{result.case.scenario_name}"
        if result.error:
            print(f"  [{n}/{total}] FAIL {label}: {result.error}", flush=True)
        elif result.report_path:
            print(f"  [{n}/{total}] "
                  f"{result.report_path.relative_to(output_dir)}", flush=True)
        else:
            print(f"  [{n}/{total}] {label}: no report in archive", flush=True)

    return results


# -- Orchestrator --


def download_job_reports(
    project_id: str, job_id: str, output_dir: Path,
    *, max_workers: int | None = None,
) -> tuple[dict[str, dict[str, str]], dict[str, str], str]:
    """Download all reports for a job.

    Returns ``(statuses, suite_ids, catalog_name)`` where *statuses* maps
    ``{suite_name: {scenario_name: status}}``, *suite_ids* maps
    ``{suite_name: suite_uuid}``, and *catalog_name* is the evaluation
    catalog's display name.
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        all_cases, catalog_name = _collect_all_cases(
            project_id, job_id, executor,
        )
        results = _download_all_reports(
            project_id, all_cases, output_dir, executor,
        )

    statuses: dict[str, dict[str, str]] = {}
    suite_ids: dict[str, str] = {}
    for r in results:
        statuses.setdefault(r.case.suite_name, {})[r.case.scenario_name] = r.case.status
        suite_ids[r.case.suite_name] = r.case.suite_id

    return statuses, suite_ids, catalog_name


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Download scenario reports from WebAuto and generate index pages.",
    )
    parser.add_argument("--project-id", required=True)
    parser.add_argument("--evaluation-job-id", required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("./reports"))
    parser.add_argument("--max-workers", type=int, default=None,
                        help="Max concurrent webauto CLI calls (default: Python's ThreadPoolExecutor default)")
    args = parser.parse_args()

    statuses, suite_ids, catalog_name = download_job_reports(
        args.project_id, args.evaluation_job_id, args.output_dir,
        max_workers=args.max_workers,
    )
    generate_indices(
        args.output_dir,
        job_id=args.evaluation_job_id,
        statuses=statuses,
        suite_ids=suite_ids,
        catalog_name=catalog_name,
    )
    print(f"\nDone. Open {args.output_dir / 'index.html'} to browse.", flush=True)


if __name__ == "__main__":
    main()
