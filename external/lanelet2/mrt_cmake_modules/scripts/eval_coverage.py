#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
from __future__ import print_function
import subprocess
import sys
import argparse
import shutil
import os
import re


def sync_gcno_files(path, build_dir):
    # sync the gcno files into this folder
    copied_something = False
    for root, dirs, files in os.walk(path):
        for file in files:
            if file.endswith(".gcda"):
                file_gcno = file[:len(file) - 4] + "gcno"
                gcda_file = os.path.join(root, file_gcno)[len(path):]
                if not os.path.exists(gcda_file):
                    raise Exception("No gcno file found matching {} (expected {})".format(file, gcda_file))
                shutil.copy(gcda_file, os.path.join(root, file_gcno))
                copied_something = True
    return copied_something


def print_coverage(output, coverage_dir, to_stderr):
    regex = re.search(r"\((?P<lines>\d*) of (?P<total>\d*) lines\)", output)
    lines = int(regex.group("lines")) if regex else 0
    total = int(regex.group("total")) if regex else 0
    # search for python coverage
    for file in os.listdir(coverage_dir):
        if file.endswith("coverage.xml"):
            from xml.dom import minidom
            cov_xml = minidom.parse(os.path.join(coverage_dir, file))
            coverage = cov_xml.getElementsByTagName("coverage")[0]
            lines += int(coverage.attributes["lines-covered"].value)
            total += int(coverage.attributes["lines-valid"].value)
    percent = float(lines) / total if total else 1
    file = sys.stderr if to_stderr else sys.stdout
    print("\n\033[1mTotal test coverage: {:.2%}\033[0m ({} of {} lines)".format(percent, lines, total), file=file)


def is_empty(tracefile):
    # lcov only tells us a tracefile is empty when it is too late.
    # This function tries to do the check efficently before we pass all tracefiles to lcov.
    with open(tracefile) as f:
        for line in f:
            if line.startswith("DA") or line.startswith("FN"):
                return False
    return True


def build_coverage(args):
    if not args.coverage_dir:
        return 0
    lcov_baseline = os.path.join(args.coverage_dir, "baseline.lcov")
    # check with --summary that the tracefile has valid records
    if not os.path.exists(lcov_baseline) or is_empty(lcov_baseline):
        print("No C++ coverage files found.")
        return 0

    # evaluate individual coverage
    lcovs = [lcov_baseline]
    for folder in os.listdir(args.coverage_dir):
        path = os.path.join(args.coverage_dir, folder)
        lcov_file = path + ".lcov"
        has_coverage = sync_gcno_files(path, args.build_dir)
        if not has_coverage:
            continue
        cmd = ["lcov", "-d", path, "-c", "-o", lcov_file, "-t", folder.replace("-", "_"), "-q"]
        fail = subprocess.call(cmd)
        if not fail and not is_empty(lcov_file):
            lcovs.append(lcov_file)

    # build full coverage (including files outside project)
    lcov_full = os.path.join(args.coverage_dir, "full_coverage.lcov")
    add_args = []
    for lcov in lcovs:
        add_args += ("-a", lcov)
    cmd = ["lcov", "-o", lcov_full, "-q"] + add_args
    fail = subprocess.call(cmd)
    if fail:
        print("No C++ coverage was generated")
        print_coverage("", args.coverage_dir, args.coverage_stderr)
        return 0

    # strip coverage of files outside project
    lcov_project = os.path.join(args.coverage_dir, "project_coverage.lcov")
    cmd = ["lcov", "-o", lcov_project, "--extract", lcov_full, args.project_dir + "/*", "-q"]
    fail = subprocess.call(cmd)
    if fail or os.path.getsize(lcov_project) == 0:
        # this is most likely because the test did not create any results
        print("No C++ coverage was generated")
        print_coverage("", args.coverage_dir, args.coverage_stderr)
        return 0

    # build html coverage
    html_dir = os.path.join(args.coverage_dir, "coverage")
    cmd = ["genhtml", "-o", html_dir, lcov_project, "-s"]
    proc = subprocess.Popen(cmd, universal_newlines=True, stdout=subprocess.PIPE)
    out, err = proc.communicate()
    if proc.returncode:
        print("No C++ coverage was generated")
        print_coverage("", args.coverage_dir, args.coverage_stderr)
        return 0

    # print & show
    print_coverage(out, args.coverage_dir, args.coverage_stderr)
    index_html = os.path.join(html_dir, "index.html")
    file = sys.stderr if args.coverage_stderr else sys.stdout
    print("  Coverage report: file://{}".format(index_html), file=file)
    if args.show:
        subprocess.call(["firefox", index_html])
    return 0


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Cleans up results of old tests and coverage runs, initializes coverage counters.')
    parser.add_argument('project_dir', help='Path of the project')
    parser.add_argument('build_dir', help='The path where cmake was configured')
    parser.add_argument('test_results_dir', help='The directory where test results will be written')
    parser.add_argument('coverage_dir', nargs='?', default="", help='the directory for the coverage')
    parser.add_argument('--show', action='store_true', help='Display results after run')
    parser.add_argument('--coverage_stderr', action='store_true', help='Print coverage to stderr instead')
    parser.add_argument('--fail_on_test', action='store_true', help='Report failure if tests failed')
    args = parser.parse_args(argv)
    fail = build_coverage(args)
    if fail:
        return fail

    # for ros2 we currently don't handle processing the results
    if os.environ.get("ROS_VERSION", "1") == "2":
        return 0

    # print unittests
    with open(os.devnull, "w") as f:
        has_ccat = subprocess.call(["which", "pygmentize"], stdout=f, stderr=f) == 0
        ccat_cmd = " | pygmentize" if has_ccat else ""
    verbose = " --verbose" if args.coverage_stderr else ""
    to_err = " 1>&2" if args.coverage_stderr else ""
    cmd = '/bin/bash -c "set -o pipefail; catkin_test_results{} {}{}{}"'.format(
        verbose, args.test_results_dir, ccat_cmd, to_err)
    fail = subprocess.call(cmd, shell=True)
    sys.stderr.write("\n")
    if args.fail_on_test:
        return fail
    return 0


if __name__ == '__main__':
    sys.exit(main())
