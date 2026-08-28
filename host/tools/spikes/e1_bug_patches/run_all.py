"""Type-check every bugNNN.py patch with Pyright and print the E1 tally.

Usage (from host/):  python tools/spikes/e1_bug_patches/run_all.py
"""
from __future__ import annotations

import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
HOST = HERE.parents[2]


def main() -> int:
    pyright = pathlib.Path(sys.executable).with_name("pyright")
    patches = sorted(HERE.glob("bug*.py"))
    proc = subprocess.run(
        [str(pyright), "--pythonpath", sys.executable, "--outputjson", *map(str, patches)],
        cwd=HOST, capture_output=True, text=True)
    report = json.loads(proc.stdout)
    by_file: dict[str, list[str]] = {p.name: [] for p in patches}
    for d in report["generalDiagnostics"]:
        if d["severity"] != "error":
            continue
        name = pathlib.Path(d["file"]).name
        by_file.setdefault(name, []).append(
            f"L{d['range']['start']['line'] + 1}: {d['message'].splitlines()[0]}")
    rejected = 0
    for name, errs in by_file.items():
        verdict = "REJECTED" if errs else "accepted"
        rejected += bool(errs)
        print(f"{name}: {verdict} ({len(errs)} error(s))")
        for e in errs:
            print(f"    {e}")
    print(f"\nTALLY: {rejected}/{len(by_file)} rejected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
