# Generic parallel scenario execution engine.
#
# Knows about worker isolation on a single host (ROS_DOMAIN_ID per slot,
# per-worker output directories, process-group cleanup) and junit merging.
# It does NOT know how a scenario is launched: the caller supplies a
# command_builder, so other run profiles (e.g. a conventional non-lockstep
# configuration) can reuse this engine unchanged.

import os
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from queue import Queue

MAX_DOMAIN_ID = 101  # safe upper bound for localhost-only DDS


class _Worker(threading.Thread):
    def __init__(self, slot, pool):
        super().__init__(daemon=True)
        self.slot = slot
        self.pool = pool

    def run(self):
        pool = self.pool
        while True:
            item = pool.queue.get()
            if item is None:
                return
            index, (label, xosc) = item
            name = f"{index:03d}_{label}"
            workdir = pool.out_dir / "workers" / name
            workdir.mkdir(parents=True, exist_ok=True)

            env = os.environ.copy()
            env["ROS_DOMAIN_ID"] = str(pool.base_domain_id + self.slot)
            # ROS_LOCALHOST_ONLY makes rmw_cyclonedds inject the loopback
            # interface; if a CYCLONEDDS_URI config already selects an
            # interface (e.g. "lo"), the duplicate selection aborts every
            # node. Only force localhost when no explicit DDS config exists.
            if not env.get("CYCLONEDDS_URI"):
                env["ROS_LOCALHOST_ONLY"] = "1"

            cmd = pool.command_builder(xosc, workdir, self.slot)

            with pool.lock:
                print(f"[slot {self.slot}] start  {name} "
                      f"(domain={env['ROS_DOMAIN_ID']})", flush=True)

            start = time.monotonic()
            with open(workdir / "launch.log", "w") as log:
                proc = subprocess.Popen(
                    cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)
                try:
                    proc.wait(timeout=pool.timeout_sec)
                except subprocess.TimeoutExpired:
                    with pool.lock:
                        print(f"[slot {self.slot}] TIMEOUT {name}; killing process group",
                              flush=True)
                    pgid = os.getpgid(proc.pid)
                    os.killpg(pgid, signal.SIGINT)  # let ros2 launch tear down children
                    try:
                        proc.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        os.killpg(pgid, signal.SIGKILL)
                        proc.wait(timeout=10)
            elapsed = time.monotonic() - start

            junit = workdir / "scenario_test_runner" / "result.junit.xml"
            verdict = "MISSING_RESULT"
            if junit.exists():
                try:
                    root = ET.parse(junit).getroot()
                    failures = int(root.get("failures", "1"))
                    errors = int(root.get("errors", "1"))
                    verdict = "PASSED" if failures == 0 and errors == 0 else "FAILED"
                except ET.ParseError:
                    verdict = "BROKEN_RESULT"
            with pool.lock:
                print(f"[slot {self.slot}] {verdict:7s} {name} ({elapsed:.1f} s)", flush=True)
            pool.results[name] = (verdict, junit if junit.exists() else None, elapsed)


class WorkerPool:
    """Runs (label, xosc) items through `jobs` isolated workers."""

    def __init__(self, jobs, base_domain_id, out_dir: Path, timeout_sec, command_builder):
        if base_domain_id + jobs - 1 > MAX_DOMAIN_ID:
            raise ValueError(
                f"ROS_DOMAIN_ID must stay <= {MAX_DOMAIN_ID}; "
                "lower jobs or base_domain_id")
        self.jobs = jobs
        self.base_domain_id = base_domain_id
        self.out_dir = out_dir
        self.timeout_sec = timeout_sec
        self.command_builder = command_builder
        self.queue = Queue()
        self.results = {}
        self.lock = threading.Lock()

    def run(self, items) -> dict:
        workers = [_Worker(slot, self) for slot in range(self.jobs)]
        for w in workers:
            w.start()
        for item in enumerate(items):
            self.queue.put(item)
        for _ in workers:
            self.queue.put(None)
        for w in workers:
            w.join()
        return self.results


def merge_junit(results: dict, out_path: Path):
    merged = ET.Element("testsuites", name="parallel_scenario_runner")
    tests = failures = errors = 0
    for name, (verdict, junit, _) in sorted(results.items()):
        if junit is None:
            suite = ET.SubElement(
                merged, "testsuite", name=name, tests="1", failures="0", errors="1")
            case = ET.SubElement(suite, "testcase", name=name)
            ET.SubElement(case, "error", message="no result.junit.xml (worker crashed?)")
            tests += 1
            errors += 1
            continue
        for suite in ET.parse(junit).getroot().iter("testsuite"):
            suite.set("name", f"{name}.{suite.get('name')}")
            merged.append(suite)
            tests += int(suite.get("tests", "0"))
            failures += int(suite.get("failures", "0"))
            errors += int(suite.get("errors", "0"))
    merged.set("tests", str(tests))
    merged.set("failures", str(failures))
    merged.set("errors", str(errors))
    ET.ElementTree(merged).write(out_path, encoding="unicode", xml_declaration=True)
    return tests, failures, errors
