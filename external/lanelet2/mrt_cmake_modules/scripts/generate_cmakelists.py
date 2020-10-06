#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import argparse
import os
import sys
import subprocess

parser = argparse.ArgumentParser(description='Generates a CMakeLists.txt for a project.')
parser.add_argument('package_name', type=str, help='name of the package')
parser.add_argument('-r', '--ros', action='store_true', help='add ROS support (messages, ...)')
parser.add_argument('-e', '--exe', action='store_true', help='create an executable/node package')
args = parser.parse_args()

# figure out where the template file is
currdir = os.path.dirname(os.path.abspath(__file__))
devel_template = os.path.join(currdir, "../ci_templates/CMakeLists.txt")
install_template = os.path.join(currdir, "../../share/mrt_cmake_modules/CMakeLists.txt")
if os.path.exists(devel_template):
    template = devel_template
elif os.path.exists(install_template):
    template = install_template
else:
    print("Failed to find the CMakeLists template file. Was this script moved?")
    sys.exit(1)

pattern = "@"
if args.ros:
    pattern += ".x"
else:
    pattern += "x."
pattern += "|"

if not args.exe:
    pattern += "x.@"
else:
    pattern += ".x@"

with open("CMakeLists.txt", "w") as file:
    with open(template) as template_file:
        file.write(template_file.read())

subprocess.call("sed -i " +
                "-e 's/^" + pattern + " //g' " +
                "-e '/^@..|..@/d' " +
                r"-e 's/\${CMAKE_PACKAGE_NAME}/" + args.package_name + "/g' " +
                "CMakeLists.txt", shell=True)
