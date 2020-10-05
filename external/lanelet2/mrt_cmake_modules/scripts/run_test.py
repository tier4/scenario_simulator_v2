#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
from __future__ import print_function
import argparse
import os
import sys
import subprocess
import sys
try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET


def get_missing_junit_result_filename(filename):
    return os.path.join(os.path.dirname(filename), 'MISSING-%s' % os.path.basename(filename))


def remove_junit_result(filename):
    # if result file exists remove it before test execution
    if os.path.exists(filename):
        os.remove(filename)
    # if placeholder (indicating previous failure) exists remove it before test execution
    missing_filename = get_missing_junit_result_filename(filename)
    if os.path.exists(missing_filename):
        os.remove(missing_filename)


def ensure_junit_result_exist(filename, errors):
    if os.path.exists(filename):
        # if result file exists ensure that it contains valid xml
        try:
            ET.parse(filename)
        except ParseError:
            from catkin.tidy_xml import tidy_xml
            tidy_xml(filename)
            try:
                ET.parse(filename)
            except ParseError as e:
                print("Invalid XML in result file '%s' (even after trying to tidy it): %s " % (filename, str(e)), file=sys.stderr)
        return True
    # if result file does not exist create placeholder which indicates failure
    missing_filename = get_missing_junit_result_filename(filename)
    print("Cannot find results, writing failure results to '%s'" % missing_filename, file=sys.stderr)
    # create folder if necessary
    if not os.path.exists(os.path.dirname(filename)):
        try:
            os.makedirs(os.path.dirname(filename))
        except OSError as e:
            # catch case where folder has been created in the mean time
            if e.errno != errno.EEXIST:
                raise
    cdata = "<![CDATA[Executable crashed with:\n{}]]>".format(errors) if errors else ""
    cdata = cdata.replace("\033", "&#27")
    with open(missing_filename, 'w') as f:
        data = {'test': os.path.basename(filename), 'test_file': filename, 'cdata': cdata}
        f.write(
            ('<?xml version="1.1" encoding="UTF-8"?>\n'
             '<testsuite tests="1" failures="0" time="1" errors="1" name="%(test)s">\n'
             '  <testcase name="%(test)s" status="run" time="1" classname="Results">\n'
             '    <failure message="Unable to find test results for %(test)s, test most probably crashed.\nExpected results in %(test_file)s" type="">'
             '%(cdata)s'
             '    </failure>\n'
             '  </testcase>\n'
             '</testsuite>\n') %
            data)
    return False


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Runs the test command passed as an argument and verifies that the expected result file has been generated.')
    parser.add_argument('results', help='The path to the xunit result file')
    parser.add_argument('command', nargs='+', help='The test command to execute')
    parser.add_argument('--working-dir', nargs='?', help='The working directory for the executed command')
    parser.add_argument('--coverage-dir', nargs='?', help='The directory where coverage data should be generated')
    parser.add_argument(
        '--return-code',
        action='store_true',
        help='Set the return code based on the success of the test command')
    parser.add_argument('--redirect-stderr', action='store_true', help='Redirect stderr to stdout')
    args = parser.parse_args(argv)

    remove_junit_result(args.results)

    try:
        os.mkdir(os.path.dirname(args.results))
    except OSError:
        pass

    work_dir_msg = ' with working directory "%s"' % args.working_dir if args.working_dir is not None else ''
    cmds_msg = ''.join(['\n  %s' % cmd for cmd in args.command])
    print('-- run_test.py: execute commands%s%s' % (work_dir_msg, cmds_msg))

    if args.coverage_dir:
        env = os.environ.copy()
        print("Coverage goes to {}".format(args.coverage_dir))
        env['GCOV_PREFIX'] = args.coverage_dir
        if not os.path.exists(args.coverage_dir):
            os.mkdir(args.coverage_dir)
    else:
        env = os.environ

    rc = 0
    errors = []
    for cmd in args.command:
        stream = sys.stderr if not args.redirect_stderr else sys.stdout
        proc = subprocess.Popen(cmd + " | tee", cwd=args.working_dir, shell=True, env=env, stderr=subprocess.PIPE)
        stdout, stderr = proc.communicate()
        if stderr:
            print(stderr, file=stream)
            errors.append(stderr.decode("utf-8"))
        if proc.returncode:
            rc = proc.returncode
            break

    print('-- run_test.py: verify result "%s"' % args.results)
    exists = ensure_junit_result_exist(args.results, u"\n".join(errors))
    if not exists:
        rc = 1

    if args.return_code:
        return rc
    return 0


if __name__ == '__main__':
    sys.exit(main())
