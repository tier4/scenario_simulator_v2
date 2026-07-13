#!/usr/bin/env python3
"""Download scenario HTML reports from WebAuto and generate index pages.

Usage:
    python -m scenario_test_runner.report.download_reports \
        --project-id x2_dev \
        --evaluation-job-id <UUID> \
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
import json
import subprocess
import tempfile
import zipfile
from pathlib import Path
from typing import Any

from .index_generator import generate_indices

_CLI_TIMEOUT = 600


# -- webauto CLI helpers --


def _webauto_json(args: list[str]) -> dict[str, Any]:
    """Run a webauto CLI command with --output json and return parsed dict."""
    cmd = ["webauto", *args, "--output", "json"]
    print(f"  $ {' '.join(cmd)}", flush=True)
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
    print(f"  $ {' '.join(cmd)}", flush=True)
    subprocess.run(cmd, capture_output=True, text=True, timeout=_CLI_TIMEOUT, check=True)
    zips = list(target_dir.glob("*.zip"))
    if not zips:
        raise FileNotFoundError(f"No ZIP downloaded to {target_dir}")
    return zips[0]


def _sanitize(name: str) -> str:
    """Sanitize a display name for use as a directory name."""
    return name.replace("/", "_").replace("\\", "_").replace("\0", "").strip()


# -- Main download logic --


def _find_report_in_zip(zf: zipfile.ZipFile) -> str | None:
    """Find the report HTML file inside a simulation archive ZIP."""
    for name in zf.namelist():
        if name.endswith("report.html") or name.endswith(".html"):
            return name
    return None


def download_job_reports(
    project_id: str, job_id: str, output_dir: Path,
) -> dict[str, dict[str, str]]:
    """Download all reports for a job.

    Returns a status map: ``{suite/scenario: status}`` keyed by the relative
    path from *output_dir* (e.g. ``"SuiteA/scenario1"``).
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Fetching suite reports for job {job_id}...", flush=True)
    suites = _webauto_list(
        "evaluation-job-suite-report", project_id,
        ["--evaluation-job-id", job_id],
    )

    statuses: dict[str, dict[str, str]] = {}

    for suite_report in suites:
        suite_id = suite_report["id"]
        suite_name = _sanitize(suite_report.get("suite", {}).get("display_name", suite_id))

        print(f"\nFetching specs for suite '{suite_name}'...", flush=True)
        specs = _webauto_list(
            "evaluation-job-spec-report", project_id,
            ["--evaluation-job-id", job_id, "--evaluation-job-suite-report-id", suite_id],
        )
        cases: list[dict[str, Any]] = []
        for spec in specs:
            spec_cases = _webauto_list(
                "evaluation-job-case-report", project_id,
                ["--evaluation-job-id", job_id, "--evaluation-job-spec-report-id", spec["id"]],
            )
            cases.extend(spec_cases)

        for case in cases:
            scenario_name = _sanitize(
                case.get("scenario", {}).get("display_name", case["id"])
            )
            case_status = case.get("status", "unknown")
            archive_log = (case.get("logs") or {}).get("simulation_archive")

            scenario_dir = output_dir / suite_name / scenario_name
            scenario_dir.mkdir(parents=True, exist_ok=True)

            statuses.setdefault(suite_name, {})[scenario_name] = case_status

            if archive_log:
                log_id = archive_log["id"]
                try:
                    with tempfile.TemporaryDirectory() as tmp:
                        zip_path = _download_archive(project_id, log_id, Path(tmp))
                        with zipfile.ZipFile(zip_path) as zf:
                            html_name = _find_report_in_zip(zf)
                            if html_name:
                                zf.extract(html_name, scenario_dir)
                                extracted = scenario_dir / html_name
                                dest = scenario_dir / "report.html"
                                if extracted != dest:
                                    extracted.rename(dest)
                                    for p in scenario_dir.iterdir():
                                        if p.is_dir() and not any(p.iterdir()):
                                            p.rmdir()
                                print(f"  -> {dest.relative_to(output_dir)}", flush=True)
                            else:
                                print(f"  -> No report.html found in archive for {scenario_name}", flush=True)
                except Exception as e:
                    print(f"  -> Failed to download archive for {scenario_name}: {e}", flush=True)
            else:
                print(f"  -> No simulation_archive for {scenario_name}", flush=True)

    return statuses


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Download scenario reports from WebAuto and generate index pages.",
    )
    parser.add_argument("--project-id", required=True)
    parser.add_argument("--evaluation-job-id", required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("./reports"))
    args = parser.parse_args()

    statuses = download_job_reports(args.project_id, args.evaluation_job_id, args.output_dir)
    generate_indices(args.output_dir, job_id=args.evaluation_job_id, statuses=statuses)
    print(f"\nDone. Open {args.output_dir / 'index.html'} to browse.", flush=True)


if __name__ == "__main__":
    main()
