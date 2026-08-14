#!/usr/bin/env python3
"""Pick a conflict-free batch of GitHub Issues for a fleet of parallel workers.

Answers one question: given the open tracker, which N issues should run right now
in separate worktrees such that impact is highest, issues carrying prior work go
first, and two workers are unlikely to collide?

Three measurements shape the design, and each one killed a simpler version:

1. **The median extractable footprint is one file.** 51 of 64 open issues mention at
   least one tracked path, but usually exactly one. Intersecting raw footprints
   therefore declares almost every pair conflict-free -- it is a check with no power.
   So seeds are *expanded* through a co-edit graph built from `git log --name-only`
   before conflict is computed (`web.py` and `test_web.py` co-occur in 37 commits).

2. **The collisions that actually bite are not files.** The MCP server holds one
   Playwright browser across calls; port 8000, the rig, and the GPU are each a
   singleton. Two workers with perfectly disjoint file footprints still deadlock on
   those, so `resources_for()` models them separately with a per-wave cap.

3. **Prior work and extractable paths are the same signal.** The comments that carry
   an implementation plan are also the only reliable source of file paths, so scoring
   that rewards prior work *and* a filter that defers unknown footprints would starve
   the cold tail forever. Hence an explicit exploration slot rather than an emergent
   bias -- see `select_batch`.

Structure follows `migrate_issues.py`: pure functions first, one thin subprocess
shell at the bottom, `argparse` front end last. Everything above `fetch_issues` is
importable and testable without network or git.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from collections import Counter, defaultdict
from datetime import UTC, datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
GH_REPO = "hellosamblack/lidar-roomscanner"

# --------------------------------------------------------------------------------
# Policy constants. All of these are asserted against the real tree by the tests, so
# a rename upstream fails loudly here instead of silently degrading the plan.
# --------------------------------------------------------------------------------

#: Touched by nearly every issue via the `status-sync` checklist, so they carry no
#: scheduling information. Workers are forbidden to edit them (the orchestrator runs
#: status-sync once, over the union of all workers' reported doc deltas), which means
#: they cannot collide there and must not be allowed to veto an otherwise-fine pair.
SHARED_DOC_FILES = frozenset({
    "ROADMAP.md", "AGENTS.md", "CLAUDE.md", "BUGS.md", "README.md",
    "docs/issue-migration-map.md", "docs/issue-migration-map.json",
    "host/tests/test_doc_links.py",
})
#: `.claude/` is here because it holds the agent definitions and hooks -- including
#: `.claude/agents/fleet-worker.md`, a worker's OWN contract. Without this prefix a
#: worker claiming an issue about the worker contract could legally rewrite it mid-run,
#: and because agent definitions load at session start, the edit would take effect on
#: the next wave rather than visibly failing.
SHARED_DOC_PREFIXES = ("docs/", ".remember/", ".agents/skills/", ".claude/")

#: Files so central that any two issues reaching them must never run concurrently,
#: even when the overlap is only inferred. Cheaper to defer an issue a wave than to
#: hand-resolve a rebase conflict in the shared checkout.
HOT_FILES = frozenset({
    "host/src/roomscan/web.py",
    "host/src/roomscan/static/index.html",
    "host/src/roomscan/static/scene.js",
    "host/src/roomscan/static/slam.js",
    "host/src/roomscan/static/layout.js",
    "host/tests/test_web.py",
    "host/tests/test_static_ui.py",
})

#: Singleton runtime resources. `port` is absent deliberately: it is assigned rather
#: than capped, because each worker can have its own.
RESOURCE_CAPS = {"browser": 1, "device": 1, "gpu": 1}

#: Fallback footprint when an issue names no path at all. Deliberately coarse -- its
#: only job is to stop two issues in the same subsystem running blind side by side.
AREA_GLOBS = {
    "area/host-web": ("host/src/roomscan/web.py", "host/src/roomscan/static/"),
    "area/host-slam": ("host/src/roomscan/slam/",),
    "area/host-sensors": ("host/src/roomscan/imufusion.py", "host/src/roomscan/magcal.py",
                          "host/src/roomscan/magsweep.py"),
    "area/host-transport": ("host/src/roomscan/sources.py", "host/src/roomscan/protocol.py",
                            "host/src/roomscan/decoder.py"),
    "area/host-splat": ("host/src/roomscan/splat/",),
    "area/host-offline": ("host/src/roomscan/splat/",),
    "area/host-tools": ("host/tools/", "host/src/roomscan/mcp_server/"),
    "area/host-viewer": ("host/src/roomscan/viewer.py",),
    "area/host-panel": ("host/src/roomscan/panel.py",),
    "area/firmware": ("firmware/scanner-stream/",),
    "area/firmware-build": ("firmware/scanner-stream/CMakeLists.txt",),
    "area/firmware-scanner-stream": ("firmware/scanner-stream/",),
    "area/firmware-eth": ("firmware/scanner-stream/",),
    "area/firmware-host": ("firmware/scanner-stream/", "host/src/roomscan/protocol.py"),
    "area/transform-lib": ("host/transform/",),
}

#: Area labels that legitimately have no code footprint. Enforced by a test that ensures
#: every area/* label in the tracker is either in AREA_GLOBS or AREA_GLOBS_EXCLUDED.
#: An issue with no stated paths and an excluded area gets an empty footprint, which
#: conflicts with nothing and permits co-scheduling with any other issue. Such an issue
#: is bounded only by the exploration slot (one unknown-footprint issue per wave).
AREA_GLOBS_EXCLUDED = {
    "area/environment",  # Operating-environment and procedure work
}

#: The `operator-request` skill's umbrella label: an issue already parked on the owner
#: (waiting on a physical capture, a hardware change, or a decision) and sitting in
#: `operator_queue()`. Scoring it as if it were actionable re-derives a veto that has
#: already been decided and written down (#177: on 2026-08-12, 2 of a wave's 3 picks were
#: already held, out of 9 held repo-wide) -- so it is excluded outright, the same as
#: `status/blocked`, rather than merely deprioritised. `operator_queue()` is the source
#: of truth for what is held; this module never calls it, it only reads the label gh
#: already returned with the issue.
NEEDS_OPERATOR_LABEL = "needs/operator"
OPERATOR_SUBTYPES = ("capture", "network", "hardware", "eyes", "decision")

PRIORITY_SCORE = {"priority/now": 100.0, "priority/next": 40.0, "priority/later": 10.0}
UNPRIORITISED_SCORE = 25.0  # between next and later: unlabelled is not the same as parked

PRIOR_WORK_MULTIPLIER = 2.0
GATE_BONUS_PER_DEPENDENT = 15.0
BUG_BONUS = 5.0

#: `## Implementation plan` / `### Root cause` -- a written plan, by an agent that had
#: read the code. `Files in scope:` outranks it: that is a *declared* footprint.
PLAN_HEADING_RE = re.compile(r"^#{2,4}\s*(implementation plan|root cause|plan)\b",
                             re.IGNORECASE | re.MULTILINE)
FILES_IN_SCOPE_RE = re.compile(r"files in scope\s*:(.*?)(?:\n\s*\n|\Z)",
                               re.IGNORECASE | re.DOTALL)
SESSION_START_RE = re.compile(r"\*\*Session start\*\*", re.IGNORECASE)

#: Deliberately requires an extension. Bare words like `slam` or `web` match far too
#: much prose, and every real citation in this tracker carries one.
PATH_RE = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_./-]*\.(?:py|js|html|css|c|h|md|json|toml|sh)")

#: `#NNN` references used to infer the dependency spine. GitHub's native `blockedBy`
#: is empty on every issue in this repo, so prose is all there is -- and prose is
#: ambiguous, so these edges are advisory and land in `notes`, never applied silently.
ISSUE_REF_RE = re.compile(r"#(\d{1,4})\b")
BLOCKER_PHRASE_RE = re.compile(
    r"(?:blocked (?:by|on)|depends on|waits? on|gated (?:by|on)|requires)\D{0,40}#(\d{1,4})",
    re.IGNORECASE)

FOOTPRINT_CONFIDENCE = ("declared", "planned", "mentioned", "none")

# --------------------------------------------------------------------------------
# Triage digest
# --------------------------------------------------------------------------------
#
# `fetch_issues` already pulls every open issue with its full body and comment threads
# -- ~260 KB in one gh call -- and the planner used to throw all of that text away,
# after which the skill instructed the orchestrator to fetch it back one issue at a
# time. That second fetch is the single largest avoidable cost in a fleet run: it is
# unbounded prose, it is O(candidates), and it lands in the most expensive context in
# the run. These caps exist so the digest replaces that loop instead of relocating it.
#
# The caps are small on purpose. A token added at turn `t` is re-sent on every one of
# the remaining turns, so bloat introduced at Step 2 -- the earliest step -- is the most
# expensive bloat there is.
TRIAGE_CHARS_PER_ISSUE = 1200
TRIAGE_SLICE_CHARS = 400
TRIAGE_COMMENT_CHARS = 300
TRIAGE_BODY_CHARS = 300
#: A few beyond the wave, so a hand-veto can be replaced without re-planning.
TRIAGE_EXTRA_CANDIDATES = 5

#: Mirrors `operator_queue.REQUEST_HEADING`. Kept as a literal rather than imported so
#: the planner stays free of that module; `test_operator_heading_matches_operator_queue`
#: pins them together.
OPERATOR_REQUEST_RE = re.compile(r"^##\s*\N{HAMMER AND WRENCH}?\s*Operator Request",
                                 re.IGNORECASE | re.MULTILINE)
#: The worker report contract from `references/worker-brief.md`.
WORKER_REPORT_RE = re.compile(
    r"^\s*[-*]?\s*(files_changed|tests_run|doc_deltas|blocked_on)\b",
    re.IGNORECASE | re.MULTILINE)
#: Acceptance that needs a human's eyes or a browser. Evidence: 6/29 open issues.
VISUAL_ACCEPTANCE_RE = re.compile(
    r"\b(looks? right|in the viewport|visually|by eye|screenshot|render(?:s|ed)? correctly)\b",
    re.IGNORECASE)

SLUG_STRIP_RE = re.compile(r"[^a-z0-9]+")


# --------------------------------------------------------------------------------
# Label helpers
# --------------------------------------------------------------------------------

def label_names(issue: dict) -> set[str]:
    return {lb.get("name", "") for lb in issue.get("labels") or []}


def issue_areas(issue: dict) -> list[str]:
    return sorted(n for n in label_names(issue) if n.startswith("area/"))


def _has_open_native_blocker(issue: dict) -> bool:
    """True only when GitHub's native blocked-by relation actually names something.

    The trap this exists for: `gh issue list --json blockedBy` returns
    `{"nodes": [], "totalCount": 0}` on every issue in this repo, and that dict is
    **truthy**. `if issue.get("blockedBy")` would treat all 64 open issues as blocked
    and return an empty batch forever.
    """
    nodes = (issue.get("blockedBy") or {}).get("nodes") or []
    return any((n or {}).get("state", "OPEN").upper() == "OPEN" for n in nodes)


# --------------------------------------------------------------------------------
# Footprint extraction
# --------------------------------------------------------------------------------

def comment_text(issue: dict) -> str:
    return "\n\n".join(c.get("body") or "" for c in issue.get("comments") or [])


def extract_paths(text: str, tracked: set[str]) -> set[str]:
    """Repo paths named in `text`, filtered to files that actually exist.

    The tracked-set filter is what makes this usable: prose is full of things that
    look like paths (`scene.js` in a sentence, `foo.py` in a hypothetical), and the
    only cheap way to tell a citation from a coincidence is whether git knows it.
    Bare basenames are resolved against the tree when unambiguous.
    """
    if not text:
        return set()
    by_base: dict[str, list[str]] = defaultdict(list)
    for p in tracked:
        by_base[p.rsplit("/", 1)[-1]].append(p)

    found: set[str] = set()
    for raw in PATH_RE.findall(text):
        cand = raw.strip("./")
        if cand in tracked:
            found.add(cand)
            continue
        # A suffix of a tracked path, e.g. `roomscan/web.py` for host/src/roomscan/web.py.
        suffix_hits = [p for p in tracked if p.endswith("/" + cand)]
        if len(suffix_hits) == 1:
            found.add(suffix_hits[0])
            continue
        hits = by_base.get(cand.rsplit("/", 1)[-1], [])
        if len(hits) == 1 and "/" not in cand:
            found.add(hits[0])
    return found


def seed_footprint(issue: dict, tracked: set[str]) -> tuple[set[str], str]:
    """Paths an issue names, plus how much to trust them.

    Confidence ranks by who wrote the claim and how deliberately:

    * ``declared``  -- a `Files in scope:` line from a `session-start` comment. An
      agent that had read the code stated this as its scope. Strongest signal here.
    * ``planned``   -- inside an `## Implementation plan` / `### Root cause` comment.
    * ``mentioned`` -- a bare prose citation anywhere in the body or comments.
    * ``none``      -- no tracked path anywhere; the area prior is all we have.
    """
    body = issue.get("body") or ""
    comments = comment_text(issue)

    declared: set[str] = set()
    for c in issue.get("comments") or []:
        cb = c.get("body") or ""
        if not SESSION_START_RE.search(cb):
            continue
        for m in FILES_IN_SCOPE_RE.finditer(cb):
            declared |= extract_paths(m.group(1), tracked)
    if declared:
        return declared, "declared"

    planned: set[str] = set()
    for c in issue.get("comments") or []:
        cb = c.get("body") or ""
        if PLAN_HEADING_RE.search(cb):
            planned |= extract_paths(cb, tracked)
    if planned:
        return planned, "planned"

    mentioned = extract_paths(body, tracked) | extract_paths(comments, tracked)
    if mentioned:
        return mentioned, "mentioned"
    return set(), "none"


# --------------------------------------------------------------------------------
# Co-edit graph and footprint expansion
# --------------------------------------------------------------------------------

def parse_git_log(raw: str) -> list[dict]:
    """Parse `git log --name-only --pretty=format:'%x00%s'` into commit records."""
    commits = []
    for rec in raw.split("\x00"):
        rec = rec.strip("\n")
        if not rec:
            continue
        lines = rec.split("\n")
        files = [f.strip() for f in lines[1:] if f.strip()]
        if files:
            commits.append({"subject": lines[0], "files": files})
    return commits


def build_coedit_graph(commits: list[dict], max_files: int = 25) -> dict[str, Counter]:
    """path -> Counter of paths it is co-edited with.

    Commits above `max_files` are skipped: a tree-wide sweep links everything to
    everything and would make the graph a complete graph, which conflicts with
    everything and therefore schedules nothing.
    """
    graph: dict[str, Counter] = defaultdict(Counter)
    for c in commits:
        files = [f for f in c["files"] if not is_shared_doc(f)]
        if len(files) < 2 or len(files) > max_files:
            continue
        for a in files:
            for b in files:
                if a != b:
                    graph[a][b] += 1
    return graph


def build_area_prior(commits: list[dict], tracked: set[str]) -> dict[str, Counter]:
    """area label -> Counter of files, inferred from conventional-commit scopes.

    `fix(host-web): ...` votes for every file that commit touched. Used only when an
    issue names no path at all, and only alongside `AREA_GLOBS`.
    """
    prior: dict[str, Counter] = defaultdict(Counter)
    scope_re = re.compile(r"^[a-z]+\(([a-z0-9-]+)\)")
    for c in commits:
        m = scope_re.match(c["subject"])
        if not m:
            continue
        area = f"area/{m.group(1)}"
        for f in c["files"]:
            if f in tracked and not is_shared_doc(f):
                prior[area][f] += 1
    return prior


def is_shared_doc(path: str) -> bool:
    return path in SHARED_DOC_FILES or path.startswith(SHARED_DOC_PREFIXES)


def sibling_tests(path: str, tracked: set[str]) -> set[str]:
    """The test module that conventionally accompanies `path`.

    `host/src/roomscan/slam/mapper.py` -> `host/tests/test_slam_mapper.py`. Encoded
    because it is by far the most common real co-edit and the graph only sees it
    after the pair has been committed together at least `min_support` times.
    """
    if not path.endswith(".py") or "/tests/" in path:
        return set()
    stem = path.rsplit("/", 1)[-1][:-3]
    parent = path.rsplit("/", 2)[-2] if path.count("/") >= 1 else ""
    candidates = {f"host/tests/test_{stem}.py"}
    if parent and parent not in ("roomscan", "src", "host"):
        candidates.add(f"host/tests/test_{parent}_{stem}.py")
    return {c for c in candidates if c in tracked}


def expand_footprint(seed: set[str], coedit: dict[str, Counter],
                     area_prior: dict[str, Counter], areas: list[str],
                     tracked: set[str], *, min_support: int = 3,
                     max_expansion: int = 12) -> dict[str, str]:
    """Grow a seed footprint into the set of files the work will *plausibly* touch.

    Returns path -> origin in {seed, sibling, coedit, area}, so a caller can tell a
    stated fact from an inference and weigh a conflict accordingly.
    """
    out: dict[str, str] = {}
    for p in seed:
        if not is_shared_doc(p):
            out[p] = "seed"

    for p in list(out):
        for t in sibling_tests(p, tracked):
            out.setdefault(t, "sibling")

    scored: Counter = Counter()
    for p in [k for k, v in out.items() if v == "seed"]:
        for nbr, support in coedit.get(p, {}).items():
            if support >= min_support and nbr not in out and nbr in tracked:
                scored[nbr] = max(scored[nbr], support)
    for nbr, _ in scored.most_common(max_expansion):
        out[nbr] = "coedit"

    if not out:
        for area in areas:
            for glob in AREA_GLOBS.get(area, ()):
                if glob.endswith("/"):
                    out.update({p: "area" for p in tracked if p.startswith(glob)})
                elif glob in tracked:
                    out[glob] = "area"
            for f, _ in area_prior.get(area, Counter()).most_common(8):
                out.setdefault(f, "area")
    return {p: o for p, o in out.items() if not is_shared_doc(p)}


def conflicts(a: dict[str, str], b: dict[str, str]) -> str:
    """`hard`, `soft`, or `none` for a pair of expanded footprints.

    Hard means never co-schedule: the two issues both *stated* a file, or they meet on
    one of the hot files where even an inferred overlap is not worth the rebase.
    Soft means they only meet on inferred paths -- schedulable, but reported so the
    orchestrator can veto with context the planner does not have.
    """
    shared = set(a) & set(b)
    if not shared:
        return "none"
    for p in shared:
        if p in HOT_FILES:
            return "hard"
        if a[p] in ("seed", "sibling") and b[p] in ("seed", "sibling"):
            return "hard"
    return "soft"


# --------------------------------------------------------------------------------
# Resources, scoring, selection
# --------------------------------------------------------------------------------

def resources_for(issue: dict, footprint: dict[str, str]) -> set[str]:
    """Singleton runtime resources this issue's verification will need."""
    areas = set(issue_areas(issue))
    paths = set(footprint)
    text = ((issue.get("title") or "") + " " + (issue.get("body") or "")).lower()
    res: set[str] = set()

    web = "area/host-web" in areas or any(
        p.startswith("host/src/roomscan/static/") or p.endswith(".html") for p in paths)
    if web:
        # Verifying web work means driving the UI, and the MCP server holds exactly
        # one Playwright browser across calls: two of these interleave on one page.
        res |= {"browser", "port"}
    if areas & {"area/host-slam", "area/host-splat", "area/host-offline"}:
        res.add("gpu")
    if any(a.startswith("area/firmware") for a in areas) or "flash" in text:
        res.add("device")
    return res


def has_prior_work(issue: dict) -> tuple[bool, list[str]]:
    """Whether someone has already done real thinking on this issue, and the evidence.

    Deliberately reads only `issue["comments"]`, never `issue["body"]`: an issue with
    zero comments must never be credited "implementation plan comment" off a plan-shaped
    body, because prior work doubles the score and a phantom match is a phantom 2x.
    Every evidence string below says "comment" because that is the only source this
    function is allowed to cite; if a future change wants a plan written straight into
    the body to count too, it must add that as its own tier with its own wording, not
    fold it into these strings.

    **This was never actually broken** -- #177 originally alleged it was, and the tests
    below were written to pin the invariant. The report was wrong: `gh issue view N
    --comments -q ...` errors with "cannot use `--jq` without specifying `--json`", and
    the probe that "found" #158 to have no comments had swallowed that error with
    `2>/dev/null`. #158 has a real `## Implementation plan` comment and the credit was
    correct. Kept as a regression guard, not as a fix. If you are here hunting the bug,
    it is in the probe, not in this function -- use `--json comments`.
    """
    evidence = []
    for c in issue.get("comments") or []:
        cb = c.get("body") or ""
        if PLAN_HEADING_RE.search(cb):
            evidence.append("implementation plan comment")
        if SESSION_START_RE.search(cb):
            evidence.append("prior session-start comment")
        if FILES_IN_SCOPE_RE.search(cb):
            evidence.append("declared files in scope")
    seen, uniq = set(), []
    for e in evidence:
        if e not in seen:
            seen.add(e)
            uniq.append(e)
    return bool(uniq), uniq


def build_dep_graph(issues: list[dict]) -> tuple[dict[int, set[int]], list[str]]:
    """Prose-inferred `issue -> issues it blocks`, plus notes on what was inferred.

    GitHub's native `blockedBy`/`blocking` are empty on every issue in this repo, so
    the #60 -> #138 -> #110 spine exists only in sentences. Inference from prose is
    unreliable, so every edge is reported for human judgement rather than applied as
    a hard exclusion.
    """
    blocks: dict[int, set[int]] = defaultdict(set)
    notes: list[str] = []
    numbers = {i["number"] for i in issues}
    for issue in issues:
        text = (issue.get("body") or "") + "\n" + comment_text(issue)
        for m in BLOCKER_PHRASE_RE.finditer(text):
            blocker = int(m.group(1))
            if blocker in numbers and blocker != issue["number"]:
                blocks[blocker].add(issue["number"])
                notes.append(
                    f"#{issue['number']} reads as blocked by #{blocker} "
                    f"(inferred from prose, not a GitHub relation -- verify before relying on it)")
    return blocks, notes


def score_issue(issue: dict, blocks: dict[int, set[int]]) -> tuple[float, list[str]]:
    """Impact score and the human-readable reasons behind it."""
    labels = label_names(issue)
    why: list[str] = []

    prio = next((p for p in PRIORITY_SCORE if p in labels), None)
    score = PRIORITY_SCORE[prio] if prio else UNPRIORITISED_SCORE
    why.append(f"{prio or 'no priority label'} ({score:g})")

    prior, evidence = has_prior_work(issue)
    if prior:
        score *= PRIOR_WORK_MULTIPLIER
        why.append(f"prior work x{PRIOR_WORK_MULTIPLIER:g} ({', '.join(evidence)})")

    dependents = blocks.get(issue["number"], set())
    if dependents:
        score += GATE_BONUS_PER_DEPENDENT * len(dependents)
        why.append(f"gates {len(dependents)} issue(s): "
                   + ", ".join(f"#{n}" for n in sorted(dependents)))

    if "bug" in labels:
        score += BUG_BONUS
        why.append(f"bug (+{BUG_BONUS:g})")
    return score, why


def suggest_tier(issue: dict, footprint: dict[str, str], confidence: str) -> str:
    """Advisory model tier. The orchestrator overrides freely -- this is a hint.

    Kept deliberately short: model choice is judgement that shifts as models change,
    and a long rule set frozen in Python goes stale silently.
    """
    labels = label_names(issue)
    areas = issue_areas(issue)
    if any(a.startswith("area/firmware") for a in areas) or "area/transform-lib" in areas:
        return "orchestrator-only"  # needs hardware and an unrecoverable-error rig
    if "area/host-slam" in areas and "bug" in labels:
        return "sonnet"             # numerics; a wrong fix here reads as plausible
    if confidence == "declared" and len(footprint) <= 3:
        return "haiku"
    if confidence == "none":
        return "sonnet"             # needs discovery before it can implement
    return "sonnet"


def slugify(text: str, limit: int = 28) -> str:
    s = SLUG_STRIP_RE.sub("-", text.lower()).strip("-")
    while len(s) > limit:
        s = s.rsplit("-", 1)[0] if "-" in s else s[:limit]
    return s or "issue"


def worktree_name(issue: dict) -> str:
    """`issue-NNN-short-slug`, short enough to keep the repo path under 150 chars."""
    return f"issue-{issue['number']}-{slugify(issue.get('title') or '')}"


def _clip(text: str, limit: int) -> str:
    """Collapse whitespace and cut to `limit`, marking the cut."""
    flat = " ".join((text or "").split())
    return flat if len(flat) <= limit else flat[:limit - 1].rstrip() + "…"


def _comment_kind(body: str) -> str:
    """What KIND of comment this is, which is most of what triage needs from it.

    A `Session start` with no outcome after it means a session died mid-flight; an
    operator request means the issue is parked on the owner; a worker report means a
    branch already exists. Each changes the claim decision, and none of them requires
    reading the comment.
    """
    if OPERATOR_REQUEST_RE.search(body):
        return "operator_request"
    if SESSION_START_RE.search(body):
        return "session_start"
    if PLAN_HEADING_RE.search(body):
        return "implementation_plan"
    if WORKER_REPORT_RE.search(body):
        return "worker_report"
    return "other"


def triage_digest(issue: dict, footprint: set[str] | None = None,
                  resources: set[str] | None = None,
                  chars: int = TRIAGE_CHARS_PER_ISSUE) -> dict:
    """A bounded read of what an issue's prose says, so nobody has to re-fetch it.

    Pure and deterministic. Answers the three questions Step 2 actually asks -- is
    there a plan, what happened most recently, and can its acceptance be run here --
    within a hard character budget, and says how much it left out.

    `chars_elided` is not decoration. A truncated plan reads exactly like a complete
    one, and the orchestrator's whole job at this step is judging whether it has enough
    to claim on. It is also the trigger for delegating a deeper read.
    """
    comments = issue.get("comments") or []
    body = issue.get("body") or ""
    footprint = footprint or set()
    resources = resources or set()

    plan_excerpt = ""
    for c in reversed(comments):          # the LATEST plan, not the first
        cb = c.get("body") or ""
        m = PLAN_HEADING_RE.search(cb)
        if m:
            plan_excerpt = _clip(cb[m.start():], TRIAGE_SLICE_CHARS)
            break

    latest = comments[-1] if comments else None
    latest_comment = None
    if latest:
        latest_comment = {
            "author": (latest.get("author") or {}).get("login", ""),
            "createdAt": latest.get("createdAt", ""),
            "kind": _comment_kind(latest.get("body") or ""),
            "excerpt": _clip(latest.get("body") or "", TRIAGE_COMMENT_CHARS),
        }

    haystack = body + " " + " ".join((c.get("body") or "") for c in comments)
    if {"device", "rig", "gpu"} & resources:
        hint = "hardware"
    elif "browser" in resources or VISUAL_ACCEPTANCE_RE.search(haystack):
        hint = "visual"
    elif any(f.rsplit("/", 1)[-1].startswith("test_") for f in footprint):
        hint = "pytest"
    else:
        hint = "unstated"

    digest = {
        "comment_count": len(comments),
        "plan_excerpt": plan_excerpt,
        "latest_comment": latest_comment,
        "body_excerpt": _clip(body, TRIAGE_BODY_CHARS),
        "acceptance_hint": hint,
    }
    carried = sum(len(v) for v in (plan_excerpt, digest["body_excerpt"]))
    carried += len(latest_comment["excerpt"]) if latest_comment else 0
    digest["chars_elided"] = max(0, len(haystack) - carried)

    # Enforce the budget here rather than trusting the individual slices to add up.
    over = carried - chars
    if over > 0 and plan_excerpt:
        digest["plan_excerpt"] = _clip(plan_excerpt, max(0, len(plan_excerpt) - over))
    return digest


def excluded_reason(issue: dict) -> str | None:
    labels = label_names(issue)
    if NEEDS_OPERATOR_LABEL in labels:
        subtype = next((s for s in OPERATOR_SUBTYPES if f"needs/{s}" in labels), None)
        detail = f" ({subtype})" if subtype else ""
        return f"needs/operator{detail}: already parked on the owner in operator_queue()"
    if "status/in-progress" in labels:
        return "claimed by another session (status/in-progress)"
    if "status/blocked" in labels:
        return "status/blocked"
    if "data-collection" in labels:
        return "data-collection: needs the owner and the hardware physically present"
    if _has_open_native_blocker(issue):
        return "has an open GitHub blocked-by relation"
    return None


def select_batch(candidates: list[dict], *, max_agents: int,
                 resource_caps: dict[str, int] | None = None,
                 exploration_slots: int = 1) -> tuple[list[dict], list[dict], list[str]]:
    """Greedy highest-score-first selection under conflict and resource constraints.

    `exploration_slots` reserves room for issues with no known footprint. Without it
    they never run: prior work is what produces file paths, so the same issues that
    score low are also the ones whose footprint is unknown, and an unknown footprint
    is the most conflict-prone thing in the pool. That correlation quietly redefines
    "highest impact" as "highest impact among issues someone already planned".
    """
    caps = dict(resource_caps or RESOURCE_CAPS)
    chosen: list[dict] = []
    deferred: list[dict] = []
    notes: list[str] = []
    used: Counter = Counter()
    explored = 0
    next_port = 8001

    for cand in sorted(candidates, key=lambda c: (-c["score"], c["number"])):
        if len(chosen) >= max_agents:
            deferred.append({"number": cand["number"], "title": cand["title"],
                             "reason": "wave full", "conflicts_with": []})
            continue

        unknown = cand["footprint_confidence"] == "none"
        if unknown and explored >= exploration_slots:
            deferred.append({"number": cand["number"], "title": cand["title"],
                             "reason": "no known footprint; exploration slot already used",
                             "conflicts_with": []})
            continue

        hard, soft = [], []
        for picked in chosen:
            verdict = conflicts(cand["footprint"], picked["footprint"])
            if verdict == "hard":
                hard.append(picked["number"])
            elif verdict == "soft":
                soft.append(picked["number"])
        if hard:
            deferred.append({"number": cand["number"], "title": cand["title"],
                             "reason": "file conflict with a selected issue",
                             "conflicts_with": hard})
            continue

        over = [r for r in cand["resources"]
                if r in caps and used[r] + 1 > caps[r]]
        if over:
            deferred.append({"number": cand["number"], "title": cand["title"],
                             "reason": f"needs singleton resource(s) already taken: {', '.join(sorted(over))}",
                             "conflicts_with": []})
            continue

        for r in cand["resources"]:
            if r in caps:
                used[r] += 1
        if "port" in cand["resources"]:
            cand["port"] = next_port
            next_port += 1
        cand["soft_conflicts_with"] = sorted(soft)
        if soft:
            notes.append(
                f"#{cand['number']} shares inferred (not stated) files with "
                + ", ".join(f"#{n}" for n in sorted(soft))
                + " -- schedulable, but veto it if you know they really overlap")
        if unknown:
            explored += 1
            notes.append(
                f"#{cand['number']} took the exploration slot: no file paths anywhere in "
                f"its body or comments, so its footprint is an area-level guess")
        chosen.append(cand)

    return chosen, deferred, notes


def plan_fleet(issues: list[dict], tracked: set[str], commits: list[dict], *,
               max_agents: int = 3,
               include_priorities: tuple[str, ...] = ("now", "next"),
               exclude_areas: tuple[str, ...] = (),
               include_unknown_footprint: bool = True,
               triage: bool = True,
               triage_chars: int = TRIAGE_CHARS_PER_ISSUE,
               generated_at: str | None = None) -> dict:
    """Pure planner. Every input is data; no network, no git, no clock unless given.

    `triage` attaches a bounded prose digest to each selected issue and to a few
    near-misses. It defaults on because the alternative is the orchestrator re-fetching
    the same bodies one `gh issue view` at a time -- the planner already holds them.
    """
    coedit = build_coedit_graph(commits)
    area_prior = build_area_prior(commits, tracked)
    blocks, dep_notes = build_dep_graph(issues)

    wanted = {f"priority/{p}" for p in include_priorities}
    excluded: list[dict] = []
    candidates: list[dict] = []

    for issue in issues:
        labels = label_names(issue)
        areas = issue_areas(issue)

        reason = excluded_reason(issue)
        if reason:
            excluded.append({"number": issue["number"], "title": issue["title"], "reason": reason})
            continue
        if wanted and not (labels & wanted):
            excluded.append({"number": issue["number"], "title": issue["title"],
                             "reason": f"priority not in {sorted(include_priorities)}"})
            continue
        if exclude_areas and set(areas) & {f"area/{a}" for a in exclude_areas}:
            excluded.append({"number": issue["number"], "title": issue["title"],
                             "reason": "area excluded by caller"})
            continue

        seed, confidence = seed_footprint(issue, tracked)
        if confidence == "none" and not include_unknown_footprint:
            excluded.append({"number": issue["number"], "title": issue["title"],
                             "reason": "no known file footprint and unknowns are excluded"})
            continue

        footprint = expand_footprint(seed, coedit, area_prior, areas, tracked)
        score, why = score_issue(issue, blocks)
        prior, evidence = has_prior_work(issue)
        candidates.append({
            "number": issue["number"],
            "title": issue["title"],
            "score": round(score, 2),
            "why": why,
            "area": areas,
            "labels": sorted(labels),
            "footprint": footprint,
            "footprint_confidence": confidence,
            "seed_files": sorted(seed),
            "resources": sorted(resources_for(issue, footprint)),
            "suggested_model": suggest_tier(issue, footprint, confidence),
            "worktree_name": worktree_name(issue),
            "branch": f"issue-{issue['number']}",
            "soft_conflicts_with": [],
            "prior_work": evidence,
        })

    batch, deferred, sel_notes = select_batch(candidates, max_agents=max_agents)
    for item in batch:
        item["footprint_size"] = len(item["footprint"])

    # Triage digests go on the batch and on a few near-misses only. Attaching them to
    # `deferred`/`excluded` too would multiply the payload by ~20 for issues nobody is
    # about to claim -- which would recreate, inside the planner, the unbounded fetch
    # this is here to remove. `select_batch` rebuilds its dicts, so look the source
    # issue back up by number rather than carrying it through.
    if triage:
        by_number = {i["number"]: i for i in issues}
        for item in list(batch) + list(deferred[:TRIAGE_EXTRA_CANDIDATES]):
            source = by_number.get(item["number"])
            if source is not None:
                item["triage"] = triage_digest(source, set(item.get("footprint", ())),
                                               set(item.get("resources", ())),
                                               chars=triage_chars)

    return {
        "batch": batch,
        "deferred": deferred,
        "excluded": excluded,
        "resource_assignments": {
            "port": {i["number"]: i["port"] for i in batch if "port" in i},
        },
        "notes": sel_notes + dep_notes,
        "counts": {"considered": len(issues), "candidates": len(candidates),
                   "selected": len(batch), "deferred": len(deferred),
                   "excluded": len(excluded)},
        "generated_at": generated_at or datetime.now(UTC).isoformat(timespec="seconds"),
    }


# --------------------------------------------------------------------------------
# Impure edge: gh and git
# --------------------------------------------------------------------------------

def _run(cmd: list[str], cwd: Path | None = None) -> str:
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=cwd)
    if result.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}\n--- stderr ---\n{result.stderr}")
    return result.stdout


def fetch_issues(limit: int = 200) -> list[dict]:
    """Every open issue with its comments, in one call (~260 KB, ~1.4 s)."""
    out = _run(["gh", "issue", "list", "--repo", GH_REPO, "--state", "open",
                "--limit", str(limit), "--json",
                "number,title,labels,body,comments,updatedAt,createdAt,blockedBy,blocking"])
    return json.loads(out)


def fetch_tracked_files(repo: Path = REPO) -> set[str]:
    return {ln for ln in _run(["git", "ls-files"], cwd=repo).splitlines() if ln}


def fetch_commits(repo: Path = REPO, since: str = "6.months") -> list[dict]:
    raw = _run(["git", "log", f"--since={since}", "--name-only",
                "--pretty=format:%x00%s"], cwd=repo)
    return parse_git_log(raw)


def plan_fleet_live(max_agents: int = 3,
                    include_priorities: tuple[str, ...] = ("now", "next"),
                    exclude_areas: tuple[str, ...] = (),
                    include_unknown_footprint: bool = True,
                    triage: bool = True,
                    repo: Path = REPO) -> dict:
    """Collect from gh + git, then plan. The one function the MCP tool calls."""
    return plan_fleet(
        fetch_issues(), fetch_tracked_files(repo), fetch_commits(repo),
        max_agents=max_agents, include_priorities=include_priorities,
        exclude_areas=exclude_areas, include_unknown_footprint=include_unknown_footprint,
        triage=triage)


def _render(plan: dict) -> str:
    lines = [f"Fleet plan — {plan['generated_at']}", ""]
    c = plan["counts"]
    lines.append(f"{c['considered']} open · {c['candidates']} candidates · "
                 f"{c['selected']} selected · {c['deferred']} deferred · {c['excluded']} excluded")
    lines.append("")
    for i in plan["batch"]:
        port = f" port={i['port']}" if "port" in i else ""
        lines.append(f"  #{i['number']:<4} score={i['score']:<7g} {i['suggested_model']:<17}"
                     f" {i['footprint_confidence']:<9} files={i['footprint_size']:<3}"
                     f" res={','.join(i['resources']) or '-'}{port}")
        lines.append(f"         {i['title'][:88]}")
        lines.append(f"         why: {'; '.join(i['why'])}")
        lines.append(f"         worktree: .claude/worktrees/{i['worktree_name']}")
        if i["soft_conflicts_with"]:
            lines.append(f"         soft conflicts: {i['soft_conflicts_with']}")
    if plan["notes"]:
        lines += ["", "Notes (orchestrator judgement required):"]
        lines += [f"  - {n}" for n in plan["notes"]]
    if plan["deferred"]:
        lines += ["", "Deferred:"]
        lines += [f"  #{d['number']:<4} {d['reason']}" for d in plan["deferred"][:12]]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--max-agents", type=int, default=3)
    ap.add_argument("--priorities", default="now,next",
                    help="comma-separated priority tiers to consider (default: now,next)")
    ap.add_argument("--exclude-areas", default="",
                    help="comma-separated area suffixes to skip, e.g. 'firmware,host-slam'")
    ap.add_argument("--no-unknown", action="store_true",
                    help="skip issues whose file footprint cannot be determined")
    ap.add_argument("--json", action="store_true", help="emit the full plan as JSON")
    args = ap.parse_args(argv)

    plan = plan_fleet_live(
        max_agents=args.max_agents,
        include_priorities=tuple(p.strip() for p in args.priorities.split(",") if p.strip()),
        exclude_areas=tuple(a.strip() for a in args.exclude_areas.split(",") if a.strip()),
        include_unknown_footprint=not args.no_unknown)
    print(json.dumps(plan, indent=2) if args.json else _render(plan))
    return 0


if __name__ == "__main__":
    sys.exit(main())
