#!/usr/bin/python3

from glob import glob
import re
from dataclasses import dataclass
from typing import Set, Dict
from catkin_pkg.packages import find_packages
from catkin_pkg.package import Package
from subprocess import check_output
from pathlib import Path
from itertools import chain
from json import dumps
import requests
import os


# cspell: ignore srvs
@dataclass
class UsedPackage:
    msgs: Set[str]
    srvs: Set[str]


# u_int8 -> UInt8
def to_camel_case(snake_str: str):
    return "".join(x.capitalize() for x in snake_str.lower().split("_"))


def either_glob(str: str):
    return "".join([f"[{c.lower()}{c.upper()}]" if c.isalpha() else c for c in str])


def main():
    # Glob all *.cpp / *.hpp files
    files = glob("**/*.cpp", recursive=True) + glob("**/*.hpp", recursive=True)

    # Message include regex
    msg_include_regex = re.compile(r"#include\s*[<\"](.+_msgs)/msg/(.+)\.hpp[>\"]")
    srv_include_regex = re.compile(r"#include\s*[<\"](.+_srv)/srv/(.+)\.hpp[>\"]")

    # Check for `#include` statements
    used_packages: Dict[str, UsedPackage] = {}
    for file in files:
        with open(file, "r") as f:
            lines = f.readlines()
            for line in lines:
                msg_match = msg_include_regex.match(line)
                srv_match = srv_include_regex.match(line)
                if msg_match:
                    package = msg_match.group(1)
                    msg = to_camel_case(msg_match.group(2))

                    if package not in used_packages:
                        used_packages[package] = UsedPackage(set(), set())
                    used_packages[package].msgs.add(msg)

                if srv_match:
                    package = srv_match.group(1)
                    srv = to_camel_case(srv_match.group(2))

                    if package not in used_packages:
                        used_packages[package] = UsedPackage(set(), set())
                    used_packages[package].srvs.add(srv)

    # Expect external packages to be in the workspace...
    pkgs: Dict[str, Package] = find_packages(".")
    known_packages = [pkg.name for pkg in pkgs.values()]
    unknown_packages = set(used_packages.keys()) - set(known_packages)
    if len(unknown_packages) > 0:
        print(f"Unknown packages: {unknown_packages}")

    updated_patches = []

    for path, pkg in pkgs.items():
        if pkg.name not in used_packages:
            continue

        used_msgs = used_packages[pkg.name].msgs
        used_srvs = used_packages[pkg.name].srvs
        used_names = used_msgs | used_srvs

        base_path = Path(path)
        check_paths = chain(
            *(
                [base_path.rglob(f"**/{either_glob(msg)}.msg") for msg in used_names]
                + [base_path.rglob(f"**/{either_glob(msg)}.srv") for msg in used_names]
                + [base_path.rglob(f"**/{either_glob(msg)}.idl") for msg in used_names]
            )
        )
        existing_files = [path for path in check_paths if path.exists()]
        existing_files = [str(path.relative_to(base_path)) for path in existing_files]

        if len(existing_files) == 0:
            print(
                f"Package {pkg.name} has no messages or services {used_msgs} {used_srvs}"
            )
            continue

        # Call `git log` for 7 days history
        # cspell: ignore oneline
        log = (
            check_output(
                [
                    "git",
                    "log",
                    "--since",
                    "7.days",
                    "-p",
                    "--minimal",
                    "--pretty=oneline",
                    "--",
                    *existing_files,
                ],
                cwd=base_path,
            )
            .strip()
            .decode("utf-8")
        )

        if len(log) == 0:
            print(f"Package {pkg.name} has no recent changes")
            continue

        updated_patches.append(log)

    if len(updated_patches) == 0:
        print("No patches to notify")
        return

    patches = "\n".join(updated_patches)

    # Post to GitHub issues comment

    body = dumps(
        {
            "body": f"```diff\n{patches}\n```",
        }
    )
    requests.post(
        f"https://api.github.com/repos/{os.environ['GITHUB_REPOSITORY']}/issues/{os.environ['ISSUE_NUMBER']}/comments",
        headers={
            "Authorization": f"token {os.environ['GITHUB_TOKEN']}",
            "Content-Type": "application/json",
        },
        data=body,
    )


if __name__ == "__main__":
    main()
