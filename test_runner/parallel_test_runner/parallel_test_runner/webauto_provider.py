# Web.Auto scenario provider.
#
# Fetches suite / vehicle-catalog definitions from the Web.Auto Vehicle
# Evaluations API and pulls the referenced scenarios (with map assets) via the
# webauto CLI. The result is a manifest (JSON) that the parallel runner
# consumes, so that "where scenarios come from" stays decoupled from "how
# they are executed".

import functools
import json
import re
import subprocess
import sys
import urllib.parse
import urllib.request
from pathlib import Path

API_BASE = "https://evaluation.ci.web.auto/v3"


@functools.lru_cache(maxsize=1)
def get_token() -> str:
    # Cached: every api_get would otherwise spawn the webauto CLI, which adds
    # up over the per-suite / per-page calls of a catalog pull.
    result = subprocess.run(
        ["webauto", "auth", "token", "get", "--quiet"],
        capture_output=True, text=True, check=True)
    token = result.stdout.strip()
    if not token:
        sys.exit("error: 'webauto auth token get' returned an empty token")
    return token


def api_get(path: str) -> dict:
    request = urllib.request.Request(
        f"{API_BASE}{path}", headers={"Authorization": f"Bearer {get_token()}"})
    with urllib.request.urlopen(request) as response:
        return json.load(response)


def fetch_suite(project_id: str, suite_id: str) -> dict:
    """GET /projects/{project_id}/suites/{suite_id} (no CLI command exists)."""
    return api_get(f"/projects/{project_id}/suites/{suite_id}")


def fetch_catalog(project_id: str, catalog_id: str) -> dict:
    """Catalog metadata via the webauto CLI (no REST endpoint on this host)."""
    result = subprocess.run(
        ["webauto", "ci", "vehicle-catalog", "describe",
         "--project-id", project_id,
         "--vehicle-catalog-id", catalog_id,
         "--output", "json"],
        capture_output=True, text=True, check=True)
    return json.loads(result.stdout)


def list_catalog_suites(project_id: str, catalog_id: str) -> list:
    """All suites attached to a catalog.

    There is no direct endpoint; suites declare their catalogs in
    `attachments`, so paginate over /suites and filter client-side.
    """
    suites, token = [], None
    while True:
        path = f"/projects/{project_id}/suites?size=100"
        if token:
            path += f"&next_token={urllib.parse.quote(token)}"
        data = api_get(path)
        for suite in data.get("suites", []):
            if any(a.get("catalog_id") == catalog_id
                   for a in suite.get("attachments", [])):
                suites.append(suite)
        token = data.get("next_token")
        if not token:
            return suites


def pull_scenario(project_id: str, scenario_id: str, work_dir: Path) -> Path:
    """webauto ci scenario pull; returns the local scenario file path."""
    result = subprocess.run(
        ["webauto", "ci", "scenario", "pull",
         "--project-id", project_id,
         "--scenario-id", scenario_id,
         "--work-dir", str(work_dir)],
        capture_output=True, text=True)
    if result.returncode != 0:
        sys.exit(f"error: scenario pull failed for {scenario_id}:\n{result.stderr}")
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) == 2 and fields[0] == "scenario_path":
            return Path(fields[1])
    sys.exit(f"error: could not find scenario_path in pull output for {scenario_id}")


def sanitize_label(name: str) -> str:
    return re.sub(r"[^\w.-]+", "_", name).strip("_") or "unnamed"


def _pull_specs(project_id, specs, work_dir: Path, label_prefix="", list_only=False) -> list:
    scenarios = []
    for spec in specs:
        name = sanitize_label(
            spec.get("scenario_display_name") or spec["scenario_id"][:8])
        # Avoid "Suite_Suite_..." when scenario names already carry the suite name.
        prefix = "" if name.startswith(label_prefix) else label_prefix
        label = sanitize_label(prefix + name)
        if list_only:
            print(f"  {label} ({spec['scenario_id']})", flush=True)
            path = None
        else:
            print(f"  pulling {label} ({spec['scenario_id']}) ...", flush=True)
            path = pull_scenario(project_id, spec["scenario_id"], work_dir)
        scenarios.append({
            "label": label,
            "scenario_id": spec["scenario_id"],
            "path": str(path) if path else None,
        })
    return scenarios


def _write_manifest(manifest: dict, work_dir: Path) -> dict:
    manifest_path = work_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
    print(f"manifest: {manifest_path}")
    return manifest


def pull_suite(project_id: str, suite_id: str, work_dir: Path, list_only=False) -> dict:
    """Pull every scenario of a suite and write <work_dir>/manifest.json."""
    suite = fetch_suite(project_id, suite_id)
    work_dir.mkdir(parents=True, exist_ok=True)

    specs = suite.get("specs", [])
    print(f"suite '{suite.get('display_name', '')}': {len(specs)} scenario(s)", flush=True)
    manifest = {
        "project_id": project_id,
        "suite_id": suite_id,
        "suite_name": suite.get("display_name", ""),
        "scenarios": _pull_specs(project_id, specs, work_dir, list_only=list_only),
    }
    return _write_manifest(manifest, work_dir)


def pull_catalog(project_id: str, catalog_id: str, work_dir: Path, list_only=False) -> dict:
    """Pull every scenario of every suite attached to a vehicle catalog."""
    catalog = fetch_catalog(project_id, catalog_id)
    suites = list_catalog_suites(project_id, catalog_id)
    work_dir.mkdir(parents=True, exist_ok=True)

    print(f"catalog '{catalog.get('display_name', '')}': {len(suites)} suite(s)", flush=True)
    manifest = {
        "project_id": project_id,
        "catalog_id": catalog_id,
        "catalog_name": catalog.get("display_name", ""),
        "suites": [],
        "scenarios": [],
    }
    for suite_summary in suites:
        # The list endpoint returns summaries; fetch the full suite for specs.
        suite = fetch_suite(project_id, suite_summary["id"])
        suite_name = suite.get("display_name", "") or suite["id"][:8]
        specs = suite.get("specs", [])
        print(f"suite '{suite_name}': {len(specs)} scenario(s)", flush=True)
        manifest["suites"].append({
            "suite_id": suite["id"],
            "suite_name": suite_name,
            "scenario_count": len(specs),
        })
        if not specs:
            continue
        manifest["scenarios"] += _pull_specs(
            project_id, specs, work_dir,
            label_prefix=sanitize_label(suite_name) + "_", list_only=list_only)
    return _write_manifest(manifest, work_dir)


def manifest_entries(manifest: dict) -> list:
    """Returns [(label, scenario_path), ...] from a manifest dict."""
    entries = []
    for s in manifest["scenarios"]:
        if not s.get("path"):
            sys.exit(f"error: manifest entry '{s['label']}' has no local path "
                     "(was the manifest generated with --list-only?)")
        entries.append((s["label"], Path(s["path"])))
    return entries


def load_manifest(path: Path) -> list:
    """Returns [(label, scenario_path), ...] from a manifest.json."""
    return manifest_entries(json.loads(Path(path).read_text()))
