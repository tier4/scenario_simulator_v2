# parallel_test_runner: runs Web.Auto scenarios locally, in parallel, using the
# fast diffusion_planner lockstep configuration.
#
# Modeled after scenario_test_runner: a single flag-style command that owns the
# test workflow, delegating individual scenario execution to scenario_test_runner.
#
#   # Web.Auto suite を取得して並列実行
#   ros2 run parallel_test_runner parallel_test_runner \
#     --project-id <project-id> --suite-id <uuid> \
#     --vehicle-model <vehicle>_perfect_tracker --jobs 6
#
#   # vehicle catalog（所属する全suite）を実行
#   ros2 run parallel_test_runner parallel_test_runner \
#     --project-id <project-id> --catalog-id <uuid> ...
#
#   # 取得だけ / 一覧だけ
#   ... --pull-only            # manifest.json を作って終了
#   ... --list-only            # ダウンロードせず一覧表示のみ
#
#   # 取得済み manifest やローカルファイルを実行
#   ros2 run parallel_test_runner parallel_test_runner --manifest <manifest.json> ...
#   ros2 run parallel_test_runner parallel_test_runner --scenario <file.yaml> ...
#
# Component layering:
#   webauto_provider  -- where scenarios come from (suite / catalog / pull)
#   expansion         -- yaml -> per-permutation xosc (pure Python)
#   parallel_runner   -- isolated N-parallel execution + junit merge (generic)
#   lockstep_profile  -- launch command for the fast lockstep configuration

import argparse
import shutil
import sys
import time
from pathlib import Path

from . import expansion, lockstep_profile, mps, parallel_runner, webauto_provider


def make_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="parallel_test_runner",
        description="Run Web.Auto scenarios locally in parallel "
                    "(diffusion_planner lockstep configuration)")

    inputs = parser.add_argument_group("scenario sources")
    inputs.add_argument("--project-id", help="Web.Auto project ID")
    inputs.add_argument("--suite-id", help="Web.Auto suite ID")
    inputs.add_argument("--catalog-id",
                        help="Web.Auto vehicle catalog ID (runs all attached suites)")
    inputs.add_argument("--manifest", type=Path,
                        help="manifest.json produced by a previous --pull-only run")
    inputs.add_argument("--scenario", action="append", default=[],
                        help="local scenario file (.yaml or .xosc); repeatable. "
                             "$(find-pkg-share pkg) substitution is supported")

    modes = parser.add_argument_group("modes")
    modes.add_argument("--pull-only", action="store_true",
                       help="download scenarios and write manifest.json, then exit")
    modes.add_argument("--list-only", action="store_true",
                       help="list remote scenarios without downloading, then exit")

    execution = parser.add_argument_group("execution")
    execution.add_argument("--jobs", type=int, default=4,
                           help="number of parallel workers")
    execution.add_argument("--output-directory", type=Path,
                           default=Path("/tmp/parallel_test_runner"))
    execution.add_argument("--work-dir", type=Path,
                           default=Path("/tmp/parallel_test_runner_assets"),
                           help="download destination for scenarios and maps")
    execution.add_argument("--base-domain-id", type=int, default=60,
                           help="worker slot i uses ROS_DOMAIN_ID = base + i")
    execution.add_argument("--mps", action="store_true",
                           help="run the workers under NVIDIA MPS so their GPU "
                                "inference kernels execute concurrently "
                                "(~x1.5 on the driving phase with 6 workers); "
                                "a daemon started here is stopped afterwards")
    lockstep_profile.add_profile_arguments(execution)
    return parser


def collect_remote(args):
    """Resolve --suite-id / --catalog-id into a manifest (pull or list)."""
    if not args.project_id:
        sys.exit("error: --project-id is required with --suite-id / --catalog-id")
    if args.catalog_id:
        return webauto_provider.pull_catalog(
            args.project_id, args.catalog_id, args.work_dir, list_only=args.list_only)
    return webauto_provider.pull_suite(
        args.project_id, args.suite_id, args.work_dir, list_only=args.list_only)


def collect_scenarios(args):
    """All inputs -> [(label, scenario_path), ...]."""
    labeled = []
    if args.suite_id or args.catalog_id:
        labeled += webauto_provider.manifest_entries(collect_remote(args))
    if args.manifest:
        labeled += webauto_provider.load_manifest(args.manifest)
    labeled += expansion.label_scenario_paths(args.scenario)
    return labeled


def run_parallel(args, labeled_scenarios) -> int:
    out_dir = args.output_directory.resolve()
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)

    print("=== expanding scenarios ===", flush=True)
    xoscs = expansion.expand_scenarios(labeled_scenarios, out_dir / "expanded")
    print(f"{len(xoscs)} permutation(s):")
    for label, x in xoscs:
        print(f"  {label}: {x}")

    params_yaml = lockstep_profile.write_interpreter_params(out_dir)
    pool = parallel_runner.WorkerPool(
        jobs=args.jobs,
        base_domain_id=args.base_domain_id,
        out_dir=out_dir,
        timeout_sec=args.global_timeout + args.startup_margin,
        command_builder=lockstep_profile.make_command_builder(args, params_yaml))

    print(f"=== running with {args.jobs} worker(s) ===", flush=True)
    start = time.monotonic()
    with mps.daemon(args.mps, out_dir / "nvidia-mps-log"):
        results = pool.run(xoscs)
    wall = time.monotonic() - start

    tests, failures, errors = parallel_runner.merge_junit(
        results, out_dir / "result.junit.xml")
    print("=== summary ===")
    for name, (verdict, _, elapsed) in sorted(results.items()):
        print(f"  {verdict:14s} {name} ({elapsed:.1f} s)")
    print(f"total: {tests} tests, {failures} failures, {errors} errors "
          f"in {wall:.1f} s wall ({len(xoscs)} scenarios / {args.jobs} workers)")
    print(f"merged junit: {out_dir / 'result.junit.xml'}")
    return 0 if failures == 0 and errors == 0 else 1


def main():
    args = make_argument_parser().parse_args()

    if not (args.suite_id or args.catalog_id or args.manifest or args.scenario):
        sys.exit("error: no scenario source given "
                 "(--suite-id / --catalog-id / --manifest / --scenario)")
    if args.list_only or args.pull_only:
        if not (args.suite_id or args.catalog_id):
            sys.exit("error: --list-only / --pull-only require "
                     "--suite-id or --catalog-id")
        collect_remote(args)
        sys.exit(0)

    sys.exit(run_parallel(args, collect_scenarios(args)))


if __name__ == "__main__":
    main()
