"""Firmware build/flash and the host test suite.

These wrap plain subprocesses, so the value is not abstraction -- it is that the
host-specific knowledge lives in code instead of prose. Three facts that bite every
session are encoded here: Ninja comes from the venv, the packaged stlink 1.8.0
cannot identify an H5 (so a locally built `develop` is used), and pytest must run
with cwd=host or `tests`/`tools` imports break.
"""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

from .paths import FIRMWARE, HOST, REPO, VENV_PY, rel
from .server import mcp

# Packaged stlink 1.8.0 reports chipid 0x000 for the H5; H5 IDCODE support landed
# on `develop` afterwards, so the skill has us build it locally.
STLINK_BIN = Path.home() / "scratchpad" / "stlink-build" / "src" / "build" / "bin"
STLINK_LIB = Path.home() / "scratchpad" / "stlink-build" / "src" / "build" / "lib"
FW_BIN = FIRMWARE / "build" / "Debug" / "scanner_stream.bin"
EXPECTED_CHIPID = "0x484"


def _run(cmd: list[str], *, cwd: Path, env: dict | None = None,
         timeout: int = 600) -> dict:
    try:
        p = subprocess.run(cmd, cwd=str(cwd), env=env, timeout=timeout,
                           capture_output=True, text=True)
    except FileNotFoundError as exc:
        return {"ok": False, "error": f"not found: {exc.filename}", "cmd": " ".join(cmd)}
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": f"timed out after {timeout}s", "cmd": " ".join(cmd)}
    return {"ok": p.returncode == 0, "returncode": p.returncode,
            "stdout": p.stdout, "stderr": p.stderr, "cmd": " ".join(cmd)}


@mcp.tool()
def fw_build(preset: str = "Debug", configure: bool = False) -> dict:
    """Build the scanner-stream firmware; returns parsed errors and the .bin size.

    Ninja is taken from host/.venv/bin, which is the only place it exists on this
    host. `configure=True` re-runs `cmake --preset` first (needed after editing
    CMakeLists.txt or adding a source file).
    """
    env = dict(os.environ)
    env["PATH"] = f"{VENV_PY.parent}{os.pathsep}{env.get('PATH', '')}"

    steps = []
    if configure:
        steps.append(_run(["cmake", "--preset", preset], cwd=FIRMWARE, env=env))
        if not steps[-1]["ok"]:
            return {"ok": False, "stage": "configure", "steps": steps}

    build = _run(["cmake", "--build", f"build/{preset}"], cwd=FIRMWARE, env=env)
    steps.append(build)

    text = build.get("stdout", "") + build.get("stderr", "")
    errors = [ln for ln in text.splitlines() if re.search(r"\b(error|Error):", ln)]
    warnings = [ln for ln in text.splitlines() if re.search(r"\bwarning:", ln)]
    # arm-none-eabi-size runs post-build: "   text	   data	    bss	    dec	..."
    size = None
    for i, ln in enumerate(text.splitlines()):
        if ln.split()[:3] == ["text", "data", "bss"]:
            parts = text.splitlines()[i + 1].split()
            if len(parts) >= 4:
                size = {"text": int(parts[0]), "data": int(parts[1]),
                        "bss": int(parts[2]), "dec": int(parts[3])}
            break
    out = {"ok": build["ok"], "preset": preset, "errors": errors[:40],
           "error_count": len(errors), "warning_count": len(warnings), "size": size}
    if FW_BIN.is_file():
        out["bin"] = {"path": rel(FW_BIN), "bytes": FW_BIN.stat().st_size}
    if not build["ok"]:
        out["tail"] = text[-3000:]
    return out


@mcp.tool()
def fw_flash(bin_path: str = "", probe_only: bool = False) -> dict:
    """Flash the firmware over SWD (`--connect-under-reset --reset`), or just probe.

    Always probes first and reports the chipid: the packaged stlink returns 0x000
    for an H5, so a wrong chipid means the wrong st-flash is on PATH rather than a
    hardware fault. Expected chipid is 0x484.
    """
    if not STLINK_BIN.is_dir():
        return {"ok": False, "error": f"stlink build not found at {STLINK_BIN}; "
                                      "see the firmware-loop skill for how to build it"}
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = f"{STLINK_LIB}{os.pathsep}{env.get('LD_LIBRARY_PATH', '')}"

    probe = _run([str(STLINK_BIN / "st-info"), "--probe"], cwd=REPO, env=env, timeout=30)
    m = re.search(r"chipid:\s*(0x[0-9a-fA-F]+)", probe.get("stdout", ""))
    chipid = m.group(1) if m else None
    result = {"probe_ok": probe["ok"], "chipid": chipid,
              "chipid_expected": EXPECTED_CHIPID,
              "chipid_match": chipid == EXPECTED_CHIPID,
              "probe_output": probe.get("stdout", "")[-800:]}
    if probe_only:
        result["ok"] = probe["ok"]
        return result
    if not probe["ok"] or chipid != EXPECTED_CHIPID:
        result["ok"] = False
        result["error"] = ("board not identified — check the USB/SWD link and that the "
                           "locally built stlink is being used")
        return result

    target = Path(bin_path) if bin_path else FW_BIN
    if not target.is_absolute():
        target = REPO / target
    if not target.is_file():
        result["ok"] = False
        result["error"] = f"no firmware binary at {rel(target)} — run fw_build() first"
        return result

    flash = _run([str(STLINK_BIN / "st-flash"), "--connect-under-reset", "--reset",
                  "write", str(target), "0x08000000"], cwd=REPO, env=env, timeout=180)
    text = flash.get("stdout", "") + flash.get("stderr", "")
    result.update({
        "ok": flash["ok"],
        "flashed": rel(target),
        "bytes": target.stat().st_size,
        "verified": "Flash written and verified" in text or "jolly good" in text.lower(),
        "flash_output": text[-1500:],
    })
    return result


def _parse_pytest(r: dict) -> dict:
    text = r.get("stdout", "") + r.get("stderr", "")
    counts = {}
    for kind in ("passed", "failed", "error", "errors", "skipped", "xfailed", "xpassed"):
        m = re.search(rf"(\d+) {kind}\b", text)
        if m:
            counts[kind.rstrip("s") if kind == "errors" else kind] = int(m.group(1))
    failures = re.findall(r"^(?:FAILED|ERROR) (\S+)", text, re.MULTILINE)
    rc = r.get("returncode")
    return {
        "ok": rc == 0,
        "returncode": rc,
        "counts": counts,
        "failures": failures[:50],
        "failure_count": len(failures),
        "summary": (text.strip().splitlines() or [""])[-1],
        "tail": text[-3000:] if rc not in (0, None) else "",
    }


@mcp.tool()
def run_tests(k: str = "", path: str = "", maxfail: int = 0,
              changed: str = "", parallel: bool = False, markers: str = "") -> dict:
    """Run the host pytest suite and return counts plus the failing test ids.

    Runs with cwd=host, which is required: pyproject sets pythonpath=["src","."],
    so running from the repo root breaks `tests`/`tools` imports.

    `k` is a -k expression; `markers` is a -m expression (e.g. "not slow" to skip
    the heavyweight families, "not serial" for the xdist-safe subset); `path`
    narrows to a file or directory under host/.

    `changed` enables automatic impact triage (local/agent fast path only -- CI
    still runs the whole suite): pass "auto" (diff vs origin/main) or a git ref to
    run only the tests whose import closure a changed module reaches, plus the
    always-run cross-cutting guards. A foundational or unmappable change safely
    falls back to a full run; the result reports `triaged`, `selected_count` and
    `full_run` so you see what actually ran. Empty selection (no impacted tests)
    runs nothing and reports it. See `impacted_tests` for a dry run.

    `parallel` runs under pytest-xdist as two passes -- `-n auto -m "not serial"`
    then the serial remainder -- so timing-sensitive tests stay correct.
    """
    from roomscan.testtriage import impacted_tests as _impacted

    triage = None
    positional: list[str] = []
    if changed:
        base = "origin/main" if changed == "auto" else changed
        triage = _impacted(base=base)
        if not triage["full_run"]:
            if not triage["selected"]:
                return {"ok": True, "triaged": True, "full_run": False,
                        "selected_count": 0, "changed": triage["changed"],
                        "note": "no impacted tests for this diff", "counts": {},
                        "failures": [], "failure_count": 0, "summary": "0 selected"}
            positional = list(triage["selected"])
    if path:
        positional.append(path)

    def _cmd(extra: list[str]) -> list[str]:
        cmd = [str(VENV_PY), "-m", "pytest", "-q", "--no-header"]
        if k:
            cmd += ["-k", k]
        if maxfail:
            cmd += [f"--maxfail={maxfail}"]
        cmd += extra
        cmd += positional
        return cmd

    if parallel:
        p1 = _parse_pytest(_run(_cmd(["-n", "auto", "-m",
                                      _join_markers(markers, "not serial")]),
                                cwd=HOST, timeout=1800))
        p2 = _parse_pytest(_run(_cmd(["-p", "no:xdist", "-m",
                                      _join_markers(markers, "serial")]),
                                cwd=HOST, timeout=1800))
        out = _merge_pytest(p1, p2)
        out["passes"] = {"parallel": p1["counts"], "serial": p2["counts"]}
    else:
        extra = ["-m", markers] if markers else []
        out = _parse_pytest(_run(_cmd(extra), cwd=HOST, timeout=1800))

    if triage is not None:
        out["triaged"] = True
        out["full_run"] = triage["full_run"]
        out["selected_count"] = triage["selected_count"]
        out["total_tests"] = triage["total_tests"]
        out["changed"] = triage["changed"]
        if triage["full_run"]:
            out["full_run_reason"] = triage["full_run_reason"]
    return out


def _join_markers(user: str, extra: str) -> str:
    return f"({user}) and ({extra})" if user else extra


def _merge_pytest(a: dict, b: dict) -> dict:
    counts: dict[str, int] = {}
    for d in (a["counts"], b["counts"]):
        for key, val in d.items():
            counts[key] = counts.get(key, 0) + val
    failures = (a["failures"] + b["failures"])[:50]
    # Exit 5 ("no tests collected") is neutral in a two-pass split: a pass whose
    # marker filter matched nothing is not a failure. Only 0 or 5 count as OK.
    ok = all(p["returncode"] in (0, 5) for p in (a, b))
    return {
        "ok": ok,
        "counts": counts,
        "failures": failures,
        "failure_count": a["failure_count"] + b["failure_count"],
        "summary": f"{a['summary']} | {b['summary']}",
        "tail": (a["tail"] + b["tail"])[-3000:],
    }


@mcp.tool()
def impacted_tests(base: str = "origin/main") -> dict:
    """Dry-run test triage: which tests a git diff likely impacts, and why.

    Runs nothing. Maps the working tree's diff vs `base` (merge-base) to the tests
    whose import closure a changed module reaches, via a static AST import graph
    over the `test_<module>.py` convention -- deterministic and DB-free (works cold
    in a fleet worktree). Returns `selected` (tests/test_x.py paths), per-test
    `reasons`, `full_run` (a foundational or unmappable change forces the whole
    suite), `changed` and counts. Use `run_tests(changed="auto")` to actually run
    the selection. This is the local/agent fast path; CI still runs everything.
    """
    from roomscan.testtriage import impacted_tests as _impacted
    return _impacted(base=base)
