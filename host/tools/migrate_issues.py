"""One-shot migration: BUGS.md/bugs/ + ROADMAP.md's Work-item register and
Data-collection queue -> GitHub Issues.

    host/.venv/bin/python host/tools/migrate_issues.py ensure-labels
    host/.venv/bin/python host/tools/migrate_issues.py migrate --source all --dry-run
    host/.venv/bin/python host/tools/migrate_issues.py migrate --source bugs --only BUG-001,BUG-094
    host/.venv/bin/python host/tools/migrate_issues.py render-map
    host/.venv/bin/python host/tools/migrate_issues.py strip-prefixes --dry-run

WHY A SCRIPT, NOT 137 MANUAL CALLS

Bugs, register items, and DC-queue rows each have their own source format
(a markdown table row, a `- **ID — title**` bullet, a table row with a
different column set) and their own legacy status vocabulary that doesn't
map 1:1 onto GitHub's open/closed. Doing this by hand risks losing exactly
the nuance (by-design vs anomaly vs vendor, "blocked", partial-gate) the
original files carry. The parsing functions below are pure (no `gh` calls,
no filesystem writes beyond the migration map) so they can be exercised with
`--dry-run` and inspected before anything public happens.

WHAT KEEPS OLD CITATIONS RESOLVABLE

Hundreds of files cite bare `BUG-042`, `SLAM-4`, `DC-E`, etc. in historical
prose (code comments, docstrings, CLAUDE.md). Rewriting all of that text is
its own high-risk project and out of scope here (see the plan). Instead:
every migration writes an entry to `docs/issue-migration-map.json`, which
`host/tests/test_doc_links.py` checks every repo reference against (offline, no
network in CI). `migrate` seeds new issues with a `<legacy-id>: ` title prefix;
`strip-prefixes` later removes those prefixes (the type is denoted by labels,
and the prefix only collides with GitHub's own `#NNN`) while preserving the
legacy ID in each body so GitHub's search still resolves it.

RESUMABILITY

`docs/issue-migration-map.json` is loaded at the start of every `migrate`
run; legacy IDs already present are skipped. Each successful create(+close)
is appended and flushed to disk immediately, so a failure partway through
the ~137 calls loses at most the one in-flight item.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BUGS_INDEX = REPO / "BUGS.md"
BUGS_DIR = REPO / "bugs"
ROADMAP = REPO / "ROADMAP.md"
MAP_JSON = REPO / "docs" / "issue-migration-map.json"
MAP_MD = REPO / "docs" / "issue-migration-map.md"
GH_REPO = "hellosamblack/lidar-roomscanner"
RAW_BASE = f"https://raw.githubusercontent.com/{GH_REPO}/main"

# ---------------------------------------------------------------------------
# Labels
# ---------------------------------------------------------------------------

TYPE_LABELS = {
    "bug": ("bug", "d73a4a", "Something isn't working"),  # already exists; kept for ensure-labels idempotency
    "work-item": ("work-item", "1d76db",
                  "Forward-looking work item migrated from ROADMAP.md's Work-item register"),
    "data-collection": ("data-collection", "1d76db",
                         "Owner-collected-capture item migrated from ROADMAP.md's Data-collection queue"),
}

AREA_LABELS = {
    "area/host-viewer": ("0e8a16", "host/roomscan viewer"),
    "area/host-panel": ("0e8a16", "host/roomscan desktop panel (deprecated legacy)"),
    "area/host-sensors": ("0e8a16", "host/roomscan IMU/mag/baro sensor fusion"),
    "area/host-slam": ("0e8a16", "host/roomscan SLAM pipeline"),
    "area/host-web": ("0e8a16", "host/roomscan-web UI"),
    "area/host-transport": ("0e8a16", "host/roomscan transport (sources, protocol decode)"),
    "area/host-tools": ("0e8a16", "host/tools scripts"),
    "area/host-offline": ("0e8a16", "host offline post-processing (COLMAP/3DGS/ARCore)"),
    "area/host-splat": ("0e8a16", "host/roomscan.splat Gaussian-splat pipeline"),
    "area/firmware": ("d93f0b", "firmware, general"),
    "area/firmware-eth": ("d93f0b", "firmware Ethernet/lwIP"),
    "area/firmware-scanner-stream": ("d93f0b", "firmware/scanner-stream fork"),
    "area/firmware-build": ("d93f0b", "firmware build/CMake"),
    "area/firmware-host": ("d93f0b", "firmware+host boundary (protocol, joint fixes)"),
    "area/transform-lib": ("c5def5", "vl53l9-transform-c pipeline"),
    "area/environment": ("c5def5", "operating-environment/procedure issue, not code"),
}

STATUS_LABELS = {
    "status/by-design": ("fbca04", "Reported as a bug, concluded intentional"),
    "status/anomaly": ("fbca04", "Observed but not reproducible/root-caused"),
    "status/vendor": ("fbca04", "Defect is upstream/vendor; we mitigate"),
    "status/mitigated": ("fbca04", "Root cause not fully removed; a mitigation shipped"),
    "status/investigated": ("fbca04", "Looked into, no code change planned"),
    "status/fix-unverified": ("fbca04", "Fix implemented but the path could not be exercised to confirm"),
    "status/blocked": ("fbca04", "Blocked on another issue or on data collection"),
    "status/partial": ("fbca04", "Gate/acceptance criterion partially met"),
}

ALL_LABELS: dict[str, tuple[str, str, str]] = {
    **{v[0]: v for v in TYPE_LABELS.values()},
    **{name: (name, color, desc) for name, (color, desc) in AREA_LABELS.items()},
    **{name: (name, color, desc) for name, (color, desc) in STATUS_LABELS.items()},
}

# BUGS.md's literal Area column values -> our label (verified against the live
# table: 17 distinct strings, `python3 -c` count in the migration plan).
AREA_MAP = {
    "environment": "area/environment",
    "firmware": "area/firmware",
    "firmware/build": "area/firmware-build",
    "firmware/eth": "area/firmware-eth",
    "firmware/host": "area/firmware-host",
    "firmware/scanner-stream": "area/firmware-scanner-stream",
    "host/native": "area/host-viewer",  # BUG-020: native transform loader, symptom was a blank viewer
    "host/panel": "area/host-panel",
    "host/sensors": "area/host-sensors",
    "host/slam": "area/host-slam",
    "host/sources": "area/host-transport",  # sources.py IS the transport/source-selection code
    "host/tools": "area/host-tools",
    "host/transport": "area/host-transport",
    "host/viewer": "area/host-viewer",
    "host/web": "area/host-web",
    "host/web UI": "area/host-web",
    "transform lib": "area/transform-lib",
}

# Work-item register subsection -> area label.
REGISTER_AREA = {
    "SLAM": "area/host-slam",
    "WEB": "area/host-web",
    "SENS": "area/host-sensors",
    "XPORT": "area/host-transport",
    "FW": "area/firmware",
    "OFFLINE": "area/host-offline",
    "TOOL": "area/host-tools",
}

# ---------------------------------------------------------------------------
# Status mapping
# ---------------------------------------------------------------------------

# (state, close_reason, extra_labels) -- close_reason is None when state == "open"
BUG_STATUS_TABLE: list[tuple[str, tuple[str, str | None, tuple[str, ...]]]] = [
    ("fixed", ("closed", "completed", ())),
    ("by-design", ("closed", "not planned", ("status/by-design",))),
    ("anomaly", ("closed", "not planned", ("status/anomaly",))),
    ("vendor", ("closed", "completed", ("status/vendor",))),
    ("mitigated", ("closed", "completed", ("status/mitigated",))),
    ("investigated", ("closed", "not planned", ("status/investigated",))),
    ("fix unverified", ("open", None, ("status/fix-unverified",))),
    ("open", ("open", None, ())),
]

# Deliberate corrections where a later dated addendum in the entry's own body
# contradicts the index cell (found by a reconciliation pass over all 98
# files before this script was written -- see the migration plan). Trust the
# body, not a stale index row.
BUG_OVERRIDES: dict[str, dict] = {
    # Top metadata says "anomaly" but the entry's last dated addendum
    # (2026-08-06) is explicit: "Still open, still not root-caused."
    "BUG-070": {"state": "open", "reason": None, "labels": ("status/anomaly",)},
    # BUGS.md's index row says "open" / "host/slam"; the entry's own header
    # says "fixed + verified 2026-08-07" / "Area: host/splat".
    "BUG-094": {"state": "closed", "reason": "completed", "labels": (), "area": "area/host-splat"},
}


def parse_status(raw: str) -> tuple[str, str | None, tuple[str, ...]]:
    """BUGS.md status cell -> (state, close_reason, extra_labels)."""
    base = raw.split("(")[0].strip().lower()
    for prefix, result in BUG_STATUS_TABLE:
        if base.startswith(prefix):
            return result
    raise ValueError(f"unrecognized bug status: {raw!r}")


def normalize_area(raw: str) -> str:
    try:
        return AREA_MAP[raw.strip()]
    except KeyError:
        raise ValueError(f"unrecognized area: {raw!r}") from None


# ---------------------------------------------------------------------------
# Parsing: BUGS.md + bugs/
# ---------------------------------------------------------------------------

BUG_ROW_RE = re.compile(
    r"^\|\s*\[(BUG-\d+)\]\(bugs/BUG-\d+\.md\)\s*\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|\s*(.+?)\s*\|\s*$"
)


@dataclass
class BugRow:
    legacy_id: str
    status_raw: str
    area_raw: str
    title: str


def parse_bugs_index() -> list[BugRow]:
    rows = []
    for line in BUGS_INDEX.read_text(encoding="utf-8").splitlines():
        m = BUG_ROW_RE.match(line)
        if m:
            rows.append(BugRow(m.group(1), m.group(2), m.group(3), m.group(4)))
    return rows


def read_bug_body(legacy_id: str) -> str:
    text = (BUGS_DIR / f"{legacy_id}.md").read_text(encoding="utf-8")
    if legacy_id == "BUG-094":
        text = text.replace(
            "[bug-094.png](@lidar-roomscanner/bugs/bug-094/bug-094.png)",
            f"[bug-094.png]({RAW_BASE}/docs/assets/bugs/bug-094.png)",
        )
    return text


# ---------------------------------------------------------------------------
# Parsing: ROADMAP.md Work-item register
# ---------------------------------------------------------------------------

REGISTER_SECTION_RE = re.compile(
    r"^## Work-item register\n(.*?)\n## Completed phases", re.S | re.M
)
SUBSECTION_RE = re.compile(r"^### .+? \(`([A-Z]+)-`\)\s*$", re.M)
ITEM_RE = re.compile(r"^- \*\*([A-Z]+-\d+) — (.+?)\*\*", re.M)


@dataclass
class RegisterItem:
    legacy_id: str
    title: str
    body: str
    prefix: str
    blocked: bool


def _is_blocked(text: str) -> bool:
    """"blocked" in the prose, but not negated ("this is not blocked on DC-I" --
    OFFLINE-4 -- must not read as blocked)."""
    return bool(re.search(r"blocked", text, re.I)) and not re.search(r"not\s+blocked", text, re.I)


def parse_register() -> list[RegisterItem]:
    roadmap = ROADMAP.read_text(encoding="utf-8")
    section_m = REGISTER_SECTION_RE.search(roadmap)
    if not section_m:
        raise ValueError("ROADMAP.md: could not locate the Work-item register section")
    body_text = section_m.group(1)

    subsections = list(SUBSECTION_RE.finditer(body_text))
    items: list[RegisterItem] = []
    for i, sub_m in enumerate(subsections):
        prefix = sub_m.group(1)
        if prefix == "DC":
            continue  # pointer-only subsection; real DC items come from the queue table
        start = sub_m.end()
        end = subsections[i + 1].start() if i + 1 < len(subsections) else len(body_text)
        chunk = body_text[start:end]

        bullets = list(ITEM_RE.finditer(chunk))
        for j, b_m in enumerate(bullets):
            legacy_id, title = b_m.group(1), b_m.group(2)
            b_start = b_m.end()
            b_end = bullets[j + 1].start() if j + 1 < len(bullets) else len(chunk)
            item_body = chunk[b_start:b_end].strip()
            items.append(RegisterItem(
                legacy_id=legacy_id,
                title=title,
                body=item_body,
                prefix=prefix,
                blocked=_is_blocked(item_body),
            ))
    return items


# ---------------------------------------------------------------------------
# Parsing: ROADMAP.md Data-collection queue
# ---------------------------------------------------------------------------

DC_TABLE_RE = re.compile(
    r"^\| \*\*(DC-[A-Z]\d?)\*\* \|(.+)\|\s*$", re.M
)


@dataclass
class DcRow:
    legacy_id: str
    capture: str
    unblocks: str
    protocol: str
    gate: str
    status_raw: str


def parse_dc_queue() -> list[DcRow]:
    roadmap = ROADMAP.read_text(encoding="utf-8")
    rows = []
    for line in roadmap.splitlines():
        m = re.match(r"^\|\s*\*\*(DC-[A-Z]\d?)\*\*\s*\|(.*)\|\s*$", line)
        if not m:
            continue
        legacy_id, rest = m.group(1), m.group(2)
        cells = [c.strip() for c in rest.split(" | ")]
        # careful: cells can contain literal "|" only inside inline code/links in this
        # table, which none of the DC rows do -- verified against the live table.
        if len(cells) < 4:
            continue
        capture, unblocks, protocol, gate, status_raw = (cells + [""] * 5)[:5]
        rows.append(DcRow(legacy_id, capture, unblocks, protocol, gate, status_raw))
    return rows


# DC-H's own status cell just reads "⬜ open", same as DC-G's -- but its Protocol
# column requires physical, non-headless hardware access ("Needs the board's
# USB_USER cable into a machine that can open the port"), which is a real
# blocker the emoji alone doesn't carry. Judgment call, not mechanically derived.
DC_STATUS_OVERRIDES = {"DC-H": ("status/blocked",)}


def parse_dc_status(legacy_id: str, raw: str) -> tuple[str, str | None, tuple[str, ...]]:
    if legacy_id in DC_STATUS_OVERRIDES:
        return "open", None, DC_STATUS_OVERRIDES[legacy_id]
    if raw.startswith("✅"):
        return "closed", "completed", ()
    if raw.startswith("❌"):
        return "open", None, ("status/blocked",)
    if raw.startswith("⚠️"):
        return "open", None, ("status/partial",)
    if "blocked on design" in raw.lower():
        return "open", None, ("status/blocked",)
    return "open", None, ()


# ---------------------------------------------------------------------------
# Title / body construction
# ---------------------------------------------------------------------------

def _plain(text: str) -> str:
    """Strip markdown emphasis/code markers -- GitHub issue titles render as
    plain text, so a literal `**crashes the process**` would show its
    asterisks rather than bolding anything."""
    return re.sub(r"\s+", " ", re.sub(r"[*`]", "", text)).strip()


def _short_title(text: str, limit: int = 140) -> str:
    """BUGS.md's Title column is a rich one-line summary, not a short title --
    up to ~800 chars for the most eventful bugs. Cut at the earliest natural
    break (em-dash, sentence end, colon) within `limit` chars so issue list
    views stay scannable; the full text is never lost, only shortened for
    display (build_body re-attaches it when it differs)."""
    plain = _plain(text)
    if len(plain) <= limit:
        return plain
    cut = min([i for i in (plain.find(sep) for sep in (" — ", ". ", ": ")) if 0 <= i <= limit]
              or [limit])
    return plain[:cut].rstrip(" .:—") + "…"


def build_title(legacy_id: str, title: str) -> str:
    return f"{legacy_id}: {_short_title(title)}"


def strip_title_prefix(legacy_id: str, title: str) -> str:
    """Remove the leading `<legacy_id>: ` that build_title() prepended.

    The type is carried by labels now, not the title. Idempotent: a title with
    no such prefix is returned unchanged.
    """
    prefix = f"{legacy_id}: "
    return title[len(prefix):] if title.startswith(prefix) else title


def ensure_legacy_id_in_body(legacy_id: str, body: str) -> str:
    """Keep the legacy ID findable in the body once it leaves the title.

    Prepends a `**Legacy ID:** <id>` line unless the ID is already present as a
    whole token (so re-running is a no-op, and bodies that already cite it in a
    header aren't doubled up). Whole-token match avoids treating `SLAM-1` as
    present just because the body mentions `SLAM-14`.
    """
    if re.search(rf"(?<![\w-]){re.escape(legacy_id)}(?![\w-])", body):
        return body
    return f"**Legacy ID:** {legacy_id}\n\n{body}"


def build_body(body: str, source: str, extra_note: str = "", full_title: str | None = None) -> str:
    header = ""
    if full_title is not None:
        plain_full = _plain(full_title)
        if _short_title(full_title) != plain_full:
            header = f"**Original title:** {plain_full}\n\n"
    footer = (
        f"\n\n---\n_Migrated from `{source}`.{(' ' + extra_note) if extra_note else ''} "
        f"Full legacy-ID -> issue mapping: "
        f"https://github.com/{GH_REPO}/blob/main/docs/issue-migration-map.md_"
    )
    return header + body.rstrip() + footer


# ---------------------------------------------------------------------------
# Migration map I/O
# ---------------------------------------------------------------------------

@dataclass
class MapEntry:
    legacy_id: str
    issue_number: int | None = None
    issue_url: str | None = None
    state: str | None = None
    source: str = ""
    alias_of: str | None = None


def load_map() -> dict[str, MapEntry]:
    if not MAP_JSON.exists():
        return {}
    data = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    return {e["legacy_id"]: MapEntry(**e) for e in data.get("entries", [])}


def save_map(entries: dict[str, MapEntry]) -> None:
    MAP_JSON.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "repo": GH_REPO,
        "entries": [
            {k: v for k, v in vars(e).items() if v is not None}
            for e in sorted(entries.values(), key=lambda e: e.legacy_id)
        ],
    }
    MAP_JSON.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def render_map() -> None:
    entries = load_map()
    lines = [
        "# Legacy tracker ID -> GitHub issue map",
        "",
        "Generated by `host/tools/migrate_issues.py render-map`. Do not hand-edit;",
        "re-run the generator if `docs/issue-migration-map.json` changes.",
        "",
        "| Legacy ID | Issue | State |",
        "|---|---|---|",
    ]
    for e in sorted(entries.values(), key=lambda e: e.legacy_id):
        if e.alias_of:
            target = entries.get(e.alias_of)
            issue_col = f"alias of {e.alias_of} -> #{target.issue_number}" if target else f"alias of {e.alias_of}"
            state_col = target.state if target else "?"
        else:
            issue_col = f"[#{e.issue_number}]({e.issue_url})"
            state_col = e.state
        lines.append(f"| {e.legacy_id} | {issue_col} | {state_col} |")
    MAP_MD.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# gh wrappers
# ---------------------------------------------------------------------------

def _run(cmd: list[str]) -> str:
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}\n--- stderr ---\n{result.stderr}")
    return result.stdout.strip()


def ensure_labels(dry_run: bool = False) -> None:
    for name, color, desc in ALL_LABELS.values():
        cmd = ["gh", "label", "create", name, "--color", color, "--description", desc,
               "--repo", GH_REPO, "--force"]
        if dry_run:
            print("DRY:", " ".join(cmd))
            continue
        _run(cmd)
        print("ok  ", name)


def gh_create_issue(title: str, body: str, labels: list[str]) -> tuple[int, str]:
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False) as f:
        f.write(body)
        body_path = f.name
    cmd = ["gh", "issue", "create", "--repo", GH_REPO, "--title", title, "--body-file", body_path]
    for label in labels:
        cmd += ["--label", label]
    url = _run(cmd)
    number = int(url.rstrip("/").rsplit("/", 1)[-1])
    return number, url


def gh_close_issue(number: int, reason: str) -> None:
    _run(["gh", "issue", "close", str(number), "--repo", GH_REPO, "--reason", reason])


def gh_list_issues() -> dict[int, dict]:
    """Every issue (open + closed) as {number: {'number','title','body'}}."""
    raw = _run(["gh", "issue", "list", "--repo", GH_REPO, "--state", "all",
                "--limit", "1000", "--json", "number,title,body"])
    return {i["number"]: i for i in json.loads(raw)}


def gh_edit_issue(number: int, title: str | None = None, body: str | None = None) -> None:
    cmd = ["gh", "issue", "edit", str(number), "--repo", GH_REPO]
    if title is not None:
        cmd += ["--title", title]
    body_path = None
    if body is not None:
        import tempfile
        with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False) as f:
            f.write(body)
            body_path = f.name
        cmd += ["--body-file", body_path]
    _run(cmd)


# ---------------------------------------------------------------------------
# Item assembly (source-agnostic dry-run/migrate record)
# ---------------------------------------------------------------------------

@dataclass
class PlannedIssue:
    legacy_id: str
    title: str
    body: str
    labels: list[str]
    state: str
    reason: str | None
    source: str
    alias_of: str | None = None


def plan_bugs(only: set[str] | None) -> list[PlannedIssue]:
    planned = []
    for row in parse_bugs_index():
        if only and row.legacy_id not in only:
            continue
        override = BUG_OVERRIDES.get(row.legacy_id, {})
        state, reason, extra = parse_status(row.status_raw)
        state = override.get("state", state)
        reason = override.get("reason", reason)
        extra = override.get("labels", extra)
        area = override.get("area", normalize_area(row.area_raw))
        body = build_body(
            read_bug_body(row.legacy_id),
            f"bugs/{row.legacy_id}.md",
            extra_note=f"Original index status: `{row.status_raw}`, area: `{row.area_raw}`.",
            full_title=row.title,
        )
        planned.append(PlannedIssue(
            legacy_id=row.legacy_id,
            title=build_title(row.legacy_id, row.title),
            body=body,
            labels=["bug", area, *extra],
            state=state,
            reason=reason,
            source=f"bugs/{row.legacy_id}.md",
        ))
    return planned


def plan_register(only: set[str] | None) -> list[PlannedIssue]:
    planned = []
    for item in parse_register():
        if only and item.legacy_id not in only:
            continue
        if item.legacy_id == "XPORT-2":
            planned.append(PlannedIssue(
                legacy_id="XPORT-2", title="", body="", labels=[], state="",
                reason=None, source="ROADMAP.md#work-item-register", alias_of="BUG-049",
            ))
            continue
        labels = ["work-item", REGISTER_AREA[item.prefix]]
        if item.blocked:
            labels.append("status/blocked")
        body = build_body(item.body, "ROADMAP.md#work-item-register", full_title=item.title)
        planned.append(PlannedIssue(
            legacy_id=item.legacy_id,
            title=build_title(item.legacy_id, item.title),
            body=body,
            labels=labels,
            state="open",
            reason=None,
            source="ROADMAP.md#work-item-register",
        ))
    return planned


def plan_dc(only: set[str] | None) -> list[PlannedIssue]:
    planned = []
    for row in parse_dc_queue():
        if only and row.legacy_id not in only:
            continue
        state, reason, extra = parse_dc_status(row.legacy_id, row.status_raw)
        title = row.capture.strip("* ").split("—")[0].strip() or row.legacy_id
        body_md = (
            f"**Capture:** {row.capture}\n\n**Unblocks:** {row.unblocks}\n\n"
            f"**Protocol:** {row.protocol}\n\n**Acceptance gate:** {row.gate}\n\n"
            f"**Status (original):** {row.status_raw}"
        )
        body = build_body(body_md, "ROADMAP.md#data-collection-queue")
        planned.append(PlannedIssue(
            legacy_id=row.legacy_id,
            title=build_title(row.legacy_id, title),
            body=body,
            labels=["data-collection", *extra],
            state=state,
            reason=reason,
            source="ROADMAP.md#data-collection-queue",
        ))
    return planned


SOURCE_PLANNERS = {"bugs": plan_bugs, "register": plan_register, "dc": plan_dc}


def plan_all(sources: list[str], only: set[str] | None) -> list[PlannedIssue]:
    items: list[PlannedIssue] = []
    for src in sources:
        items.extend(SOURCE_PLANNERS[src](only))
    return items


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def cmd_migrate(args: argparse.Namespace) -> None:
    sources = ["bugs", "register", "dc"] if args.source == "all" else [args.source]
    only = set(args.only.split(",")) if args.only else None
    planned = plan_all(sources, only)

    existing = load_map()
    for p in planned:
        if p.legacy_id in existing:
            print(f"skip {p.legacy_id} (already migrated -> "
                  f"{existing[p.legacy_id].issue_url or existing[p.legacy_id].alias_of})")
            continue

        if p.alias_of:
            print(f"{'DRY ' if args.dry_run else ''}{p.legacy_id} -> alias of {p.alias_of}")
            if not args.dry_run:
                existing[p.legacy_id] = MapEntry(legacy_id=p.legacy_id, alias_of=p.alias_of,
                                                  source=p.source)
                save_map(existing)
            continue

        if args.dry_run:
            print(f"DRY {p.legacy_id} state={p.state} reason={p.reason} labels={p.labels}")
            print(f"    title: {p.title}")
            print(f"    body ({len(p.body)} chars): {p.body[:200]!r}...")
            continue

        number, url = gh_create_issue(p.title, p.body, p.labels)
        if p.state == "closed":
            gh_close_issue(number, p.reason or "completed")
        existing[p.legacy_id] = MapEntry(
            legacy_id=p.legacy_id, issue_number=number, issue_url=url,
            state=p.state, source=p.source,
        )
        save_map(existing)
        print(f"{p.legacy_id} -> #{number} ({p.state}) {url}")
        time.sleep(args.sleep)


def cmd_strip_prefixes(args: argparse.Namespace) -> None:
    """Strip the legacy-ID title prefix from every migrated issue.

    The issue type is denoted by labels (`bug`/`work-item`/`data-collection`)
    and the area by `area/*`, so `BUG-098: `/`SLAM-14: `/`DC-K: ` prefixes only
    collide with GitHub's own `#NNN`. The legacy ID is preserved in the body so
    GitHub search still resolves it; `docs/issue-migration-map.json` remains the
    authoritative map. Idempotent.
    """
    only = set(args.only.split(",")) if args.only else None
    entries = load_map()
    issues = {} if args.dry_run and args.offline else gh_list_issues()

    n_title = n_body = n_skip = 0
    for lid, e in sorted(entries.items()):
        if only and lid not in only:
            continue
        if e.alias_of or e.issue_number is None:
            continue  # aliases (e.g. XPORT-2 -> BUG-049) have no issue of their own
        issue = issues.get(e.issue_number)
        if issue is None:
            print(f"WARN {lid}: issue #{e.issue_number} not found on GitHub")
            continue
        cur_title, cur_body = issue["title"], issue.get("body") or ""
        new_title = strip_title_prefix(lid, cur_title)
        new_body = ensure_legacy_id_in_body(lid, cur_body)
        title_change, body_change = new_title != cur_title, new_body != cur_body
        if not title_change and not body_change:
            n_skip += 1
            continue
        n_title += title_change
        n_body += body_change
        if args.dry_run:
            print(f"DRY {lid} #{e.issue_number}"
                  + (f"\n    title: {cur_title!r} -> {new_title!r}" if title_change else "")
                  + ("\n    body : + legacy-id marker" if body_change else ""))
            continue
        gh_edit_issue(e.issue_number,
                      title=new_title if title_change else None,
                      body=new_body if body_change else None)
        print(f"{lid} -> #{e.issue_number} (title={title_change} body={body_change})")
        time.sleep(args.sleep)

    print(f"\n{'DRY: ' if args.dry_run else ''}titles={n_title} bodies={n_body} "
          f"already-clean={n_skip}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ensure-labels").set_defaults(
        func=lambda a: ensure_labels(dry_run=a.dry_run))
    ap._subparsers._group_actions[0].choices["ensure-labels"].add_argument(
        "--dry-run", action="store_true")

    p_migrate = sub.add_parser("migrate")
    p_migrate.add_argument("--source", choices=["bugs", "register", "dc", "all"], default="all")
    p_migrate.add_argument("--only", help="comma-separated legacy IDs")
    p_migrate.add_argument("--dry-run", action="store_true")
    p_migrate.add_argument("--sleep", type=float, default=1.5)
    p_migrate.set_defaults(func=cmd_migrate)

    sub.add_parser("render-map").set_defaults(func=lambda a: render_map())

    p_strip = sub.add_parser(
        "strip-prefixes",
        help="remove the legacy-ID prefix from issue titles (type is on labels)")
    p_strip.add_argument("--only", help="comma-separated legacy IDs")
    p_strip.add_argument("--dry-run", action="store_true")
    p_strip.add_argument("--offline", action="store_true",
                         help="with --dry-run, skip the gh list call (no titles shown)")
    p_strip.add_argument("--sleep", type=float, default=0.7)
    p_strip.set_defaults(func=cmd_strip_prefixes)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    sys.exit(main())
