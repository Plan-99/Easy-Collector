#!/usr/bin/env python3
import argparse
import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict

SKIP_DIRS = {".git", "__pycache__", ".venv", ".mypy_cache", ".pytest_cache", "node_modules", "logs"}
SKIP_SUFFIXES = {".pyc", ".log", ".db", ".sqlite3"}


def parse_args():
    p = argparse.ArgumentParser(description="Lightweight file watcher that restarts a process on change.")
    p.add_argument("--cmd", required=True, help="Command to run (quoted).")
    p.add_argument("--workdir", default=".", help="Working directory for the process.")
    p.add_argument("--log-file", default=None, help="Log file to append child stdout/stderr.")
    p.add_argument("--watch", action="append", default=["backend"], help="Paths to watch (relative or absolute).")
    p.add_argument("--interval", type=float, default=1.0, help="Polling interval in seconds.")
    return p.parse_args()


def _should_skip(path: Path) -> bool:
    if path.name in SKIP_DIRS:
        return True
    if path.suffix in SKIP_SUFFIXES:
        return True
    return False


def snapshot(paths) -> Dict[Path, float]:
    snap: Dict[Path, float] = {}
    for base in paths:
        if not base.exists():
            continue
        for root, dirs, files in os.walk(base):
            root_path = Path(root)
            dirs[:] = [d for d in dirs if not _should_skip(root_path / d)]
            for fname in files:
                path = root_path / fname
                if _should_skip(path):
                    continue
                try:
                    snap[path] = path.stat().st_mtime
                except OSError:
                    continue
    return snap


def changed(old: Dict[Path, float], new: Dict[Path, float]) -> bool:
    if old.keys() != new.keys():
        return True
    for k, v in new.items():
        if old.get(k) != v:
            return True
    return False


def log(msg: str):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[RELOAD] {ts} {msg}", flush=True)


def spawn(cmd: str, workdir: Path, log_file: Path | None):
    args = shlex.split(cmd)
    stdout = subprocess.PIPE
    if log_file:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        stdout = open(log_file, "a", buffering=1)
    proc = subprocess.Popen(args, cwd=str(workdir), stdout=stdout, stderr=subprocess.STDOUT)
    return proc, stdout


def main():
    args = parse_args()
    workdir = Path(args.workdir).resolve()
    watch_paths = [Path(p).resolve() if not Path(p).is_absolute() else Path(p) for p in args.watch]
    log_file = Path(args.log_file).resolve() if args.log_file else None

    stop = False

    def _signal_handler(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    snap = snapshot(watch_paths)
    proc, stream = spawn(args.cmd, workdir, log_file)
    log(f"Started: {args.cmd} (pid={proc.pid})")

    try:
        while not stop:
            time.sleep(max(0.2, args.interval))
            new_snap = snapshot(watch_paths)
            restart_needed = changed(snap, new_snap)
            if proc.poll() is not None:
                log(f"Process exited with code {proc.returncode}, restarting...")
                restart_needed = True
            if restart_needed:
                snap = new_snap
                try:
                    proc.terminate()
                    proc.wait(timeout=5)
                except Exception:
                    proc.kill()
                if stream not in (sys.stdout, sys.stderr) and stream is not None:
                    try:
                        stream.close()
                    except Exception:
                        pass
                proc, stream = spawn(args.cmd, workdir, log_file)
                log(f"Restarted: {args.cmd} (pid={proc.pid})")
    finally:
        try:
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=3)
        except Exception:
            proc.kill()
        if stream not in (sys.stdout, sys.stderr) and stream is not None:
            try:
                stream.close()
            except Exception:
                pass
        log("Watcher stopped.")


if __name__ == "__main__":
    main()
