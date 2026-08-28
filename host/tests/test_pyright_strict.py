"""E1 gate (#223): the domain-typed modules must pass Pyright strict with zero
errors. `pyrightconfig.json`'s `strict` list is the single source of which
modules are held to that bar; this test reads it rather than repeating it."""
from __future__ import annotations

import json
import pathlib
import shutil
import subprocess
import sys

import pytest

HOST = pathlib.Path(__file__).resolve().parents[1]


def _pyright_cmd() -> list[str] | None:
    exe = pathlib.Path(sys.executable).with_name("pyright")
    if exe.exists():
        return [str(exe)]
    found = shutil.which("pyright")
    return [found] if found else None


def strict_modules() -> list[str]:
    cfg = json.loads((HOST / "pyrightconfig.json").read_text())
    return list(cfg["strict"])


def run_pyright(paths: list[str]) -> tuple[int, list[dict]]:
    cmd = _pyright_cmd()
    if cmd is None:
        pytest.skip("pyright not installed (pip install -e '.[dev]')")
    proc = subprocess.run(
        [*cmd, "--pythonpath", sys.executable, "--outputjson", *paths],
        cwd=HOST, capture_output=True, text=True, timeout=600)
    try:
        report = json.loads(proc.stdout)
    except json.JSONDecodeError:
        pytest.fail(f"pyright produced no JSON: rc={proc.returncode}\n{proc.stdout}\n{proc.stderr}")
    diags = [d for d in report["generalDiagnostics"] if d["severity"] == "error"]
    return len(diags), diags


def test_strict_modules_have_zero_pyright_errors():
    mods = strict_modules()
    assert mods, "pyrightconfig.json lists no strict modules"
    n, diags = run_pyright(mods)
    msg = "\n".join(f"{d['file']}:{d['range']['start']['line'] + 1}: {d['message']}" for d in diags)
    assert n == 0, f"{n} pyright error(s) in strict modules:\n{msg}"
