"""Guards on the tracker-doc layout so the BUGS.md / bugs/ split can't silently rot.

BUGS.md is an index table; each bug's full entry lives in `bugs/BUG-NNN.md`. Nothing
in the codebase enforces that the many scattered `BUG-NNN` references (source comments,
docs, user-facing strings) actually resolve, so a renamed/dropped entry would break
silently. This test asserts:

1. Every `BUG-NNN` referenced anywhere in the repo resolves to a `bugs/BUG-NNN.md` file.
2. Every `bugs/BUG-NNN.md` file has a row in the BUGS.md index (and vice versa) -- no
   orphan entry files, no dangling index rows.
3. The hot/cold split is structurally intact (ROADMAP.md carries the work-item register;
   docs/roadmap-history.md exists).
"""
from __future__ import annotations

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BUGS_INDEX = REPO / "BUGS.md"
BUGS_DIR = REPO / "bugs"
ROADMAP = REPO / "ROADMAP.md"
ROADMAP_HISTORY = REPO / "docs" / "roadmap-history.md"

BUG_RE = re.compile(r"BUG-(\d+)")
SCAN_SUFFIXES = {".md", ".py", ".js", ".c", ".h", ".ts", ".css", ".html", ".toml"}
# Dirs whose contents are not our tracker's concern (read-only vendor code, build
# output, gitignored binaries, dependency trees, this test itself is scanned but only
# references ids that exist).
SKIP_DIR_PARTS = {
    ".git", "node_modules", "build", "dist", "__pycache__", ".venv", "venv",
    "captures", "results", ".mypy_cache", ".pytest_cache",
}
SKIP_PATH_FRAGMENTS = ("firmware/vendor/",)


def _iter_repo_files():
    for path in REPO.rglob("*"):
        if path.suffix.lower() not in SCAN_SUFFIXES or not path.is_file():
            continue
        rel = path.relative_to(REPO).as_posix()
        if any(part in SKIP_DIR_PARTS for part in path.relative_to(REPO).parts):
            continue
        if any(frag in rel for frag in SKIP_PATH_FRAGMENTS):
            continue
        yield path


def _canonical(num: str) -> str:
    return f"BUG-{int(num):03d}"


def entry_files() -> set[str]:
    return {p.stem for p in BUGS_DIR.glob("BUG-*.md")}


def index_rows() -> set[str]:
    ids = set()
    for line in BUGS_INDEX.read_text(encoding="utf-8").splitlines():
        m = re.match(r"\|\s*\[?(BUG-\d+)", line.strip())
        if m:
            ids.add(m.group(1))
    return ids


def referenced_ids() -> dict[str, list[str]]:
    """canonical BUG id -> list of repo-relative files that reference it."""
    refs: dict[str, list[str]] = {}
    for path in _iter_repo_files():
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for m in BUG_RE.finditer(text):
            refs.setdefault(_canonical(m.group(1)), []).append(
                path.relative_to(REPO).as_posix()
            )
    return refs


def test_every_referenced_bug_resolves_to_a_file():
    files = entry_files()
    unresolved = {
        bug: sorted(set(where))
        for bug, where in referenced_ids().items()
        if bug not in files
    }
    assert not unresolved, (
        "BUG-NNN referenced with no bugs/BUG-NNN.md entry:\n"
        + "\n".join(f"  {bug}: {where}" for bug, where in sorted(unresolved.items()))
    )


def test_entry_files_and_index_rows_are_one_to_one():
    files = entry_files()
    rows = index_rows()
    assert files, "no bugs/BUG-*.md entry files found"
    orphan_files = sorted(files - rows)
    dangling_rows = sorted(rows - files)
    assert not orphan_files, f"bugs/ entries with no BUGS.md index row: {orphan_files}"
    assert not dangling_rows, f"BUGS.md rows with no bugs/ entry file: {dangling_rows}"


def test_hot_cold_split_intact():
    assert ROADMAP_HISTORY.is_file(), "docs/roadmap-history.md is missing"
    roadmap = ROADMAP.read_text(encoding="utf-8")
    assert "## Work-item register" in roadmap, "ROADMAP.md lost its Work-item register"
