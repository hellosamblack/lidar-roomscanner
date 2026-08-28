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


NEGATIVE = HOST / "tests" / "pyright_negative" / "frames_negative.py"


def test_frame_tags_reject_mismatch():
    """#225: the frame-tagged `Cloud[F]` / `Rotation[To, From]` types must reject
    exactly the lines `frames_negative.py` marks `# EXPECT-ERROR` -- the BUG-026
    shape (a CV cloud returned as the render-world cloud) among them -- and
    accept every other line. Asserting the *set* of lines, not just a count,
    keeps a new false positive from cancelling out a lost true one."""
    # Trailing marker only, so the fixture's own docstring (which names the
    # marker) is not counted as an expected error.
    expected = {i for i, line in enumerate(NEGATIVE.read_text().splitlines(), start=1)
                if line.rstrip().endswith("# EXPECT-ERROR")}
    assert expected, "fixture has no EXPECT-ERROR lines"
    _n, diags = run_pyright([str(NEGATIVE)])
    # A rejected line may carry a secondary "type is unknown" diagnostic; the
    # claim is about WHICH lines fail, so compare the set of lines.
    got = sorted({d["range"]["start"]["line"] + 1 for d in diags})
    msgs = "\n".join(f"L{d['range']['start']['line'] + 1}: {d['message'].splitlines()[0]}"
                     for d in diags)
    assert sorted(expected) == got, f"expected errors on {sorted(expected)}, got {got}:\n{msgs}"
