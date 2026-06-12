# NVIDIA MPS (Multi-Process Service) lifecycle for parallel GPU inference.
#
# With N workers each running its own TensorRT engine process, the default GPU
# scheduler time-slices between CUDA contexts. MPS lets kernels from different
# processes run concurrently instead: measured x1.47 on the driving phase
# (RTX 4090, 6 workers, Release_J6_Gen2 A/B, 2026-06-12). Clients need no
# configuration -- they discover the daemon through the default pipe directory
# (/tmp/nvidia-mps).

import contextlib
import os
import shutil
import subprocess
from pathlib import Path

_CONTROL = "nvidia-cuda-mps-control"


def _daemon_running() -> bool:
    try:
        return subprocess.run(
            [_CONTROL], input="get_default_active_thread_percentage\n",
            capture_output=True, text=True, timeout=5).returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


@contextlib.contextmanager
def daemon(enabled: bool, log_dir: Path):
    """Ensure the MPS daemon runs while the body executes.

    A daemon that was already running is reused and left untouched; one
    started here is stopped on exit so the host returns to its previous
    state either way.
    """
    if not enabled:
        yield
        return
    if shutil.which(_CONTROL) is None:
        raise RuntimeError(f"--mps requires {_CONTROL} (CUDA driver tools) on PATH")
    if _daemon_running():
        print("MPS: reusing already-running daemon", flush=True)
        yield
        return
    log_dir.mkdir(parents=True, exist_ok=True)
    # The daemon's default log directory /var/log/nvidia-mps is not writable
    # for regular users; without an override it refuses to start.
    subprocess.run(
        [_CONTROL, "-d"],
        env={**os.environ, "CUDA_MPS_LOG_DIRECTORY": str(log_dir)},
        capture_output=True)
    if not _daemon_running():
        raise RuntimeError("failed to start the MPS control daemon")
    print(f"MPS: daemon started (logs: {log_dir})", flush=True)
    try:
        yield
    finally:
        subprocess.run([_CONTROL], input="quit\n", text=True, capture_output=True)
        print("MPS: daemon stopped", flush=True)
