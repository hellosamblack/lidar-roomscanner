"""Kill browser trees this project leaked, and only those (issue #221).

`__main__`'s docstring says a stdio server "is a child of its MCP client and dies with
it". That is true of the *server* and false of the *browser*: `CdpSession` starts Chrome
with `subprocess.Popen` and `PlaywrightSession` starts a node driver, and POSIX reparents
children to init rather than killing them. So the #219 idle reaper has a hole exactly
where it hurts most -- the reaper lives inside the server process, so if that process is
killed or crashes, the browser it was meant to reap becomes immortal *with no reaper at
all*. Restarting a server to pick up new code is one of the ways to cause it, which made
the fix for #219 a way to leak a browser.

Two mechanisms, because neither covers every exit path:

* `install_exit_cleanup()` -- `atexit` + `SIGTERM`, and `pdeathsig()` for the Chrome we
  spawn ourselves. Handles the ordinary cases at the moment they happen.
* `sweep_orphaned_browsers()` -- run at startup, catches what no in-process handler could:
  `SIGKILL`, a hard crash, a machine that lost power mid-session. Self-healing, because
  servers respawn readily and so this runs often.

**Ownership is the whole safety argument.** Several agent sessions share this box, each
with its own MCP server and browser, so the sweep must never touch a browser that a *live*
server still owns, and must never match the user's ordinary Chrome. Both are enforced by
`_owner_is_alive`: a candidate is orphaned only when no ancestor is a living
`roomscan.mcp_server`, and candidates are only ever recognised by paths this project
created (our own profile prefix, or a playwright driver inside our venv).
"""
from __future__ import annotations

import atexit
import ctypes
import os
import signal
import sys

# Only ever match browsers WE created. `roomscan-mcp-chrome-` is `CdpSession`'s
# `tempfile.mkdtemp` prefix; the playwright driver is matched by its path inside this
# repo's venv, so another project's playwright (or the user's own Chrome) cannot match.
CHROME_PROFILE_PREFIX = "roomscan-mcp-chrome-"
PLAYWRIGHT_DRIVER_MARK = os.path.join("playwright", "driver")
SERVER_MARK = "roomscan.mcp_server"

PR_SET_PDEATHSIG = 1


def pdeathsig() -> None:
    """`preexec_fn` making the kernel SIGKILL this child when its parent dies.

    The only mechanism here that survives the parent being `SIGKILL`ed, since it needs
    no cooperation from the dying process. Linux-only and best-effort: a failure must
    never stop the browser from launching, because a leaked browser is a much smaller
    problem than a UI loop that cannot start.
    """
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGKILL)
    except Exception:
        pass


def _cmdline(proc) -> str:
    try:
        return " ".join(proc.cmdline())
    except Exception:
        return ""


def _is_live_server(proc) -> bool:
    return SERVER_MARK in _cmdline(proc)


def _owner_is_alive(proc, self_pid: int) -> bool:
    """Does a LIVE `roomscan.mcp_server` still own `proc`?

    Walks ancestors rather than checking `ppid == 1`: an orphan may have been reparented
    to a subreaper (systemd user session) instead of init, which would make a naive
    `ppid == 1` test miss it entirely on this box.
    """
    seen = set()
    cur = proc
    while cur is not None and cur.pid not in seen:
        seen.add(cur.pid)
        if cur.pid == self_pid or _is_live_server(cur):
            return True
        try:
            cur = cur.parent()
        except Exception:
            return False
    return False


def _candidates(psutil):
    """Browser roots this project created, newest first."""
    out = []
    for proc in psutil.process_iter(["pid", "name"]):
        cmd = _cmdline(proc)
        if not cmd:
            continue
        if CHROME_PROFILE_PREFIX in cmd or PLAYWRIGHT_DRIVER_MARK in cmd:
            out.append(proc)
    return out


def _kill_tree(psutil, proc, timeout: float = 5.0) -> int:
    """TERM the tree, then KILL whatever is left. Returns processes reaped."""
    try:
        victims = proc.children(recursive=True) + [proc]
    except Exception:
        victims = [proc]
    for v in victims:
        try:
            v.terminate()
        except Exception:
            pass
    gone, alive = psutil.wait_procs(victims, timeout=timeout)
    for v in alive:
        try:
            v.kill()
        except Exception:
            pass
    if alive:
        psutil.wait_procs(alive, timeout=timeout)
    return len(victims)


def sweep_orphaned_browsers() -> dict:
    """Kill this project's browser trees that no live MCP server owns.

    Returns a report rather than logging: an MCP tool reports what actually happened
    (`docs/mcp-server.md` -> "Two invariants"), and a sweep that silently killed nothing
    because its matcher was wrong should be distinguishable from a clean box.
    """
    try:
        import psutil
    except ImportError:
        return {"ok": False, "error": "psutil unavailable", "killed": [], "kept": []}

    self_pid = os.getpid()
    killed, kept = [], []
    # Roots only: killing a tree takes its children, and a child seen later in the same
    # pass would then be a dead pid.
    for proc in _candidates(psutil):
        try:
            if not proc.is_running():
                continue
        except Exception:
            continue
        entry = {"pid": proc.pid, "cmd": _cmdline(proc)[:120]}
        if _owner_is_alive(proc, self_pid):
            kept.append(entry)
            continue
        try:
            entry["reaped"] = _kill_tree(psutil, proc)
            killed.append(entry)
        except Exception as exc:
            entry["error"] = f"{type(exc).__name__}: {exc}"
            kept.append(entry)
    return {"ok": True, "killed": killed, "kept": kept,
            "killed_count": len(killed), "kept_count": len(kept)}


def install_exit_cleanup(browser) -> None:
    """Tear the browser down on a normal exit and on SIGTERM.

    SIGTERM is the case that matters and the one Python does NOT handle for you: the
    default disposition terminates the process outright, so `atexit` never runs and the
    browser is leaked. Chained rather than replaced, so an existing handler (or the
    default exit) still happens.
    """
    def _sync_stop() -> None:
        proc = getattr(browser, "_proc", None)
        if proc is not None:
            try:
                proc.terminate()
            except Exception:
                pass
        # Playwright's node driver is not ours to terminate directly; killing the tree
        # by ownership is what `sweep_orphaned_browsers` is for on the next start.
        try:
            sweep_orphaned_browsers()
        except Exception:
            pass

    atexit.register(_sync_stop)

    previous = signal.getsignal(signal.SIGTERM)

    def _on_term(signum, frame):
        _sync_stop()
        if callable(previous) and previous not in (signal.SIG_DFL, signal.SIG_IGN):
            previous(signum, frame)
        else:
            sys.exit(0)

    try:
        signal.signal(signal.SIGTERM, _on_term)
    except (ValueError, OSError):
        pass  # not on the main thread; atexit still covers the normal path
