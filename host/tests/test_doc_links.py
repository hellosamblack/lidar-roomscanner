"""Guards that legacy tracker IDs (BUG-NNN, SLAM-N, DC-E, ...) still resolve after
the 2026-08-10 migration to GitHub Issues.

Nothing in the codebase enforces that the many scattered legacy-ID references
(source comments, docs, user-facing strings) actually resolve to something real,
so a typo or a dropped mapping entry would break silently. This test asserts,
entirely offline (no network / `gh` calls, so it runs in CI):

1. `docs/issue-migration-map.json` is well-formed.
2. Every legacy ID referenced anywhere in the repo has an entry in that map (or
   aliases to one that does) -- the same invariant the old file-existence check
   enforced, checked against the map instead of `bugs/BUG-NNN.md` existence.
3. The hot/cold split is structurally intact (`docs/roadmap-history.md` exists;
   `ROADMAP.md` points at GitHub Issues instead of carrying the register itself).
"""
from __future__ import annotations

import json
import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
MAP_JSON = REPO / "docs" / "issue-migration-map.json"
ROADMAP = REPO / "ROADMAP.md"
ROADMAP_HISTORY = REPO / "docs" / "roadmap-history.md"

LEGACY_ID_RE = re.compile(
    r"\b(?:BUG|SLAM|WEB|SENS|XPORT|FW|OFFLINE|TOOL)-\d+\b|\bDC-[A-Z]\d?\b"
)
SCAN_SUFFIXES = {".md", ".py", ".js", ".c", ".h", ".ts", ".css", ".html", ".toml"}
# Dirs whose contents are not our tracker's concern (read-only vendor code, build
# output, gitignored binaries, dependency trees, this test itself is scanned but only
# references ids that exist).
SKIP_DIR_PARTS = {
    ".git", "node_modules", "build", "dist", "__pycache__", ".venv", "venv",
    "captures", "results", ".mypy_cache", ".pytest_cache",
    # #195: .claude/worktrees/<branch>/ holds full in-flight worktree copies of this
    # same repo (issue-fleet workers), each with its own docs/**. Without this, the
    # scanner walks into them from the main checkout and a worktree's in-progress
    # doc edits (e.g. IDs it hasn't finished wiring into the migration map yet) can
    # fail this test on `main`, for a change that isn't even landed there.
    ".claude",
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


def load_migration_map() -> dict[str, dict]:
    assert MAP_JSON.is_file(), f"{MAP_JSON} is missing"
    data = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    entries = data.get("entries")
    assert isinstance(entries, list) and entries, f"{MAP_JSON} has no entries"
    return {e["legacy_id"]: e for e in entries}


def referenced_ids() -> dict[str, list[str]]:
    """legacy id -> list of repo-relative files that reference it."""
    refs: dict[str, list[str]] = {}
    for path in _iter_repo_files():
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for m in LEGACY_ID_RE.finditer(text):
            refs.setdefault(m.group(0), []).append(path.relative_to(REPO).as_posix())
    return refs


def test_migration_map_is_well_formed():
    entries = load_migration_map()
    bad = []
    for legacy_id, e in entries.items():
        has_issue = "issue_number" in e and "issue_url" in e
        has_alias = "alias_of" in e
        if not (has_issue or has_alias):
            bad.append(legacy_id)
    assert not bad, f"migration map entries with neither issue_number nor alias_of: {bad}"


def test_every_legacy_id_resolves_to_migration_map_entry():
    entries = load_migration_map()
    unresolved = {
        legacy_id: sorted(set(where))
        for legacy_id, where in referenced_ids().items()
        if legacy_id not in entries
    }
    assert not unresolved, (
        "legacy tracker ID referenced with no docs/issue-migration-map.json entry "
        "(check it resolves to a GitHub issue, or add/fix the map entry):\n"
        + "\n".join(f"  {lid}: {where}" for lid, where in sorted(unresolved.items()))
    )


def test_hot_cold_split_intact():
    assert ROADMAP_HISTORY.is_file(), "docs/roadmap-history.md is missing"
    roadmap = ROADMAP.read_text(encoding="utf-8")
    assert "## Work tracking" in roadmap, "ROADMAP.md lost its GitHub-Issues pointer section"
    assert "issue-migration-map" in roadmap, "ROADMAP.md no longer points at the migration map"
