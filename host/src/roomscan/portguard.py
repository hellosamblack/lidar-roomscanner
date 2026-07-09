"""Tell a *busy* scanner port (exists but locked by another program) from a
*missing* one (no such port / no scanner), and -- on Windows -- offer to close
the roomscan program holding it so the user doesn't have to hunt for the window.

Only one process can hold the CDC serial port at a time, so launching a second
viewer/panel while one is already open fails with "Access is denied". The common
holder is another one of OUR processes (`python -m roomscan.viewer/panel`), so
we enumerate those and, with the user's consent, terminate them and let the
caller retry. Killing is done by the user's own program at an interactive "y" --
not silently.
"""
from __future__ import annotations

import subprocess
import sys


def classify_open_error(exc: BaseException) -> str:
    """Classify a serial-open failure: 'busy' (port exists but locked),
    'missing' (no such port / no scanner found), or 'unknown'."""
    winerror = getattr(exc, "winerror", None)
    text = str(exc).lower()
    if isinstance(exc, PermissionError) or winerror == 5 or "access is denied" in text:
        return "busy"
    if (isinstance(exc, FileNotFoundError) or winerror == 2
            or "cannot find" in text or "no scanner" in text or "not found" in text):
        return "missing"
    return "busy" if "permission" in text else "unknown"


_PS_LIST = (
    "Get-CimInstance Win32_Process | "
    "Where-Object { $_.Name -in 'python.exe','pythonw.exe' "
    "-and $_.CommandLine -match 'roomscan\\.(viewer|panel)' } | "
    "ForEach-Object { \"$($_.ProcessId)`t$($_.CommandLine)\" }"
)


def roomscan_processes(exclude_pid: int | None = None) -> list[tuple[int, str]]:
    """(Windows) [(pid, command_line)] of python processes running a roomscan
    viewer/panel, excluding `exclude_pid`. Empty on non-Windows or any failure --
    the feature degrades to the plain 'close the other window' hint."""
    if not sys.platform.startswith("win"):
        return []
    try:
        out = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", _PS_LIST],
            capture_output=True, text=True, timeout=10,
        ).stdout
    except Exception:
        return []
    procs: list[tuple[int, str]] = []
    for line in out.splitlines():
        line = line.strip()
        if "\t" not in line:
            continue
        pid_s, cmd = line.split("\t", 1)
        try:
            pid = int(pid_s)
        except ValueError:
            continue
        if exclude_pid is not None and pid == exclude_pid:
            continue
        procs.append((pid, cmd.strip()))
    return procs


def terminate(pids: list[int]) -> list[int]:
    """Best-effort force-terminate by PID (Windows taskkill). Returns the pids
    that taskkill reported success for."""
    killed: list[int] = []
    for pid in pids:
        try:
            r = subprocess.run(["taskkill", "/F", "/PID", str(pid)],
                               capture_output=True, text=True, timeout=10)
            if r.returncode == 0:
                killed.append(pid)
        except Exception:
            pass
    return killed


def offer_to_close_holders(*, exclude_pid: int | None = None, input_fn=input,
                           out=print, list_fn=roomscan_processes, kill_fn=terminate) -> bool:
    """List the roomscan programs likely holding the port and, on an interactive
    'y', terminate them. Returns True iff processes were closed (caller retries).
    `list_fn`/`kill_fn`/`input_fn`/`out` are injectable for testing."""
    holders = list_fn(exclude_pid=exclude_pid)
    if not holders:
        out("  Could not identify the program holding the port. Close any other "
            "roomscan viewer/panel\n  window (or program using the port) and retry.")
        return False
    out("  The port is held by another program. These roomscan processes are running:")
    for pid, cmd in holders:
        out(f"    PID {pid}  {cmd[:72]}")
    try:
        answer = input_fn("  Close them and retry? [y/N] ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        answer = "n"
    if answer not in ("y", "yes"):
        return False
    killed = kill_fn([pid for pid, _ in holders])
    out(f"  Closed {len(killed)} process(es).")
    return bool(killed)
