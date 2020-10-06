#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import subprocess
import sys
import argparse
import shutil
import os


def get_source_file(gcno_file, build_dir):
    # the gcno path should have the structure: <builddir>/CMakeFiles/<target dir>/<sourcepath>.gcno.
    # We want to extract the sourcepath part
    relpath = os.path.relpath(gcno_file, os.path.join(build_dir, "CMakeFiles"))
    return os.path.join(*(relpath.rstrip(".gcno").split(os.path.sep)[1:]))


def cleanup_orphaned_gcnos(build_dir, project_dir):
    # cmake does not delete gcno files after their source file is gone. this confuses lcov.
    for root, dirs, files in os.walk(os.path.join(build_dir, "CMakeFiles")):
        for file in files:
            if not file.endswith(".gcno"):
                continue
            gcno_file = os.path.join(root, file)
            src_file = os.path.join(project_dir, get_source_file(gcno_file, build_dir))
            if not os.path.exists(src_file):
                print("Removing orphaned {}".format(file))
                os.remove(gcno_file)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Cleans up results of old tests and coverage runs, initializes coverage counters.')
    parser.add_argument('project_name', help='Name of the project')
    parser.add_argument('build_dir', help='The path where cmake was configured')
    parser.add_argument('project_dir', help='Path to the project')
    parser.add_argument('test_results_dir', help='The directory where test results will be written')
    parser.add_argument('coverage_dir', nargs='?', default="", help='the directory for the coverage')
    args = parser.parse_args(argv)

    test_dir = os.path.join(args.test_results_dir)
    print("Removing {}".format(test_dir))
    if os.path.exists(test_dir):
        shutil.rmtree(test_dir)
    os.makedirs(test_dir)

    if not args.coverage_dir:
        return

    if os.path.exists(args.coverage_dir):
        print("Removing {}".format(args.coverage_dir))
        shutil.rmtree(args.coverage_dir)
    if os.path.exists(args.build_dir):
        cleanup_orphaned_gcnos(args.build_dir, args.project_dir)
    os.mkdir(args.coverage_dir)
    out_file = os.path.join(args.coverage_dir, "baseline.lcov")
    cmd = ["lcov", "-i", "-c", "-o", out_file, "--directory", args.build_dir, "-q"]
    print("Initializing counters")
    return subprocess.call(cmd)


if __name__ == '__main__':
    sys.exit(main())
