#!/usr/bin/env python3
"""Generate the owner-facing operator-action page from the held-issue runbooks.

`operator_queue()` returns a flat list of issues waiting on the owner. Read literally
that list asks for one trip to the rig per issue -- 14 of them today -- because every
`## 🔧 Operator Request` comment carries its own power-up preamble. Most of those
preambles are the same six steps, and two of the runbooks are pure aliases that say so
in prose ("Covered by the request on #145").

Prose cannot be batched, so this scrapes the runbooks and works out what the queue
actually costs: which issues ride free on another issue's action, which share a setup
that only has to happen once per sitting, and which must be scheduled last because they
take the rig out of action. The output is one self-contained HTML file -- a checklist
the owner can tick through while holding the hardware, with every runbook still present
verbatim so nothing is lost to summarising.

Three things are derived from the runbook text rather than hand-maintained, because a
hand-written batch table would drift the moment a runbook is revised:

* **aliases** -- a request titled "Covered by the request on #NNN" is a free rider.
* **shared steps** -- near-duplicate steps are matched across runbooks by token-set
  similarity (single-linkage, Jaccard >= `SIM_THRESHOLD`), so "Press the power button on
  the **battery pack** (the RavPower box...)" and the shorter "Press the power button on
  the **battery pack**." land in one cluster.
* **setup vs per-take** -- a shared cluster only counts as setup-done-once if it sits
  before that runbook's first *cycle* step (Record / Stop / name the file). Without this
  the "type exactly `i141-ambientffpan`" steps would cluster with "type exactly
  `i144-tumble`" -- textually near-identical, but each take needs its own.

Venue and hazard tags come from a keyword lexicon (`TAG_RULES`) over the request body.
That part is a lexicon rather than an inference, and it is the one place to edit when a
runbook introduces a genuinely new kind of constraint.

See `.agents/skills/operator-request/SKILL.md` and `host/tools/operator_queue.py`.
"""
from __future__ import annotations

import argparse
import html
import json
import re
import subprocess
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tools.operator_queue import (  # noqa: E402
    REPO,
    REQUEST_HEADING,
    UMBRELLA,
    parse_footer,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = REPO_ROOT / "host" / "src" / "roomscan" / "static" / "operator.html"

# Two steps are "the same step" above this token-set overlap. 0.6 merges the long and
# short forms of the power-up preamble without merging two different [Claude] steps.
SIM_THRESHOLD = 0.6

# Only these survive as section headers; a runbook contains other standalone bold lines
# ("**There is nothing extra for you to do here.**") that are emphasis, not structure.
SECTIONS = (
    "Why this matters",
    "What you need, and how long",
    "Steps",
    "While it is running, note down",
    "When you are done, say",
    "What I will do with it",
    "If something looks wrong",
)

# A step at or after the first of these begins a per-take cycle: it has to be repeated
# for every recording, so it can never be hoisted into a sitting's shared setup.
CYCLE_MARKERS = ("● record", "■ stop", "naming box", "type exactly", "done with #")

_STOPWORDS = frozenset(
    "a an and are as at be by for from in into is it its of on or that the then there to "
    "up with you your i me my so should will can not do does".split()
)

_STEP_RE = re.compile(r"^\s*(\d+)\.\s+\[(You|Claude)\]\s*(.*)$")
_SUBHEAD_RE = re.compile(r"^###\s+(.*)$")
_ALIAS_RE = re.compile(r"covered by the request on #(\d+)", re.I)
_DURATION_RE = re.compile(r"about\s+\*\*(\d+)\s*(minute|hour)s?\*\*", re.I)
_BLANK_FIELD_RE = re.compile(r"_{4,}")
# Strips the bullet marker only. `lstrip("-* ")` eats the leading `**` of a bullet that
# opens in bold ("- **No green light?** Unplug...") and leaves the closing pair to bind
# with the *next* `**`, inverting which half of the sentence is emphasised.
_BULLET_RE = re.compile(r"^[-*]\s+")

# tag -> (regex, human meaning, scope)
#
# `scope="venue"` matches only what the runbook says you need and what it asks you to
# *do*; `scope="body"` matches the whole request. The distinction is load-bearing: #60's
# "Why this matters" and troubleshooting prose mention a blank wall only to tell you to
# point *away* from one, and matching the whole body sent it to the wrong room. Negated
# matches are dropped by `_matches()` for the same reason.
TAG_RULES: dict[str, tuple[str, str, str]] = {
    "rig-power": (r"power button on the \*\*battery pack\*\*", "the scanner powered up", "body"),
    "rig-on": (r"scanner,? (?:powered up as usual|can stay exactly where it is)"
               r"|scanner is already on", "the scanner already on", "venue"),
    "no-hardware": (r"nothing physical", "nothing physical", "venue"),
    "browser": (r"localhost:8000", "the app open in a browser", "body"),
    "handheld": (r"hand-held|pick the scanner up|holding it at arm|hold the scanner",
                 "the scanner in your hands", "venue"),
    "metal-free": (r"from anything metal|away from metal|metal furniture|metal joists",
                   "a spot clear of metal", "venue"),
    "blank-wall": (r"blank,? (?:matte,? )?wall|\bblank wall\b", "a plain blank wall", "venue"),
    "still-surface": (r"table or shelf|sit undisturbed|sitting completely still",
                      "a surface it can sit still on", "venue"),
    "walk-loop": (r"walk a full loop|walk your route", "a room you can walk a loop in", "venue"),
    "phone": (r"rtab-map|pixel 10", "the Pixel with RTAB-Map", "venue"),
    "usb-cable": (r"usb_user", "the second USB cable", "venue"),
    "terminal": (r"terminal window", "a terminal you can leave open", "venue"),
    "mode-change": (r"ranging mode", "a ranging-mode change Claude makes and undoes", "body"),
    "viewer-down": (r"stop the live viewer", "the live viewer stopped", "body"),
    "reflash": (r"load the experimental software|experimental software onto the scanner",
                "different firmware loaded onto the board", "body"),
    "power-cycle": (r"power cycle|switch the scanner off and on",
                    "you within reach of the power button", "body"),
}

# Anything here means the sitting needs the rig present and powered, even when the
# runbook never spells out a power-up step (#142's recording is started by Claude).
RIG_TAGS = frozenset({"rig-power", "rig-on", "handheld", "metal-free", "blank-wall",
                      "walk-loop", "still-surface"})

# Most specific first: an issue's venue is the first of these it needs. Two issues with
# the same venue can share one setup, which is the whole point of the page.
VENUES: tuple[tuple[str, str], ...] = (
    ("blank-wall", "A plain, matte, blank wall"),
    ("walk-loop", "A room you can walk a full loop in"),
    ("metal-free", "Middle of a room, two arm-lengths clear of metal"),
    ("still-surface", "A table or shelf it can sit on undisturbed"),
    ("usb-cable", "At the computer, with the scanner's second USB cable"),
    ("terminal", "At the computer, in a terminal"),
    ("rig-on", "Wherever the scanner already is"),
    ("no-hardware", "Anywhere — nothing physical"),
)

# Venue ordering within a sitting: props-free work first, the fiddly staging last.
VENUE_ORDER = {tag: i for i, tag in enumerate(
    ("rig-on", "metal-free", "walk-loop", "still-surface", "blank-wall",
     "usb-cable", "terminal", "no-hardware"))}

# sitting key -> (heading, why these belong together, sort rank)
SITTINGS: dict[str, tuple[str, str, int]] = {
    "desk": (
        "At your desk — no hardware",
        "None of these need the scanner switched on. They are questions, a look at a "
        "screen, and one command. Do them any time, in any order.",
        0,
    ),
    "rig-live": (
        "One rig session — power up once",
        "Every runbook below opens with the same power-up, link check and browser check. "
        "Done as one sitting that happens once instead of once each.",
        1,
    ),
    "rig-exclusive": (
        "Last — these take the rig out of action",
        "Each of these stops the live viewer or loads different firmware, so nothing "
        "above can run while they do. Schedule them at the end of a session.",
        2,
    ),
}


# --------------------------------------------------------------------------- scraping


def _gh_json(args: list[str]) -> object:
    proc = subprocess.run(["gh", *args], capture_output=True, text=True, timeout=90)
    if proc.returncode != 0:
        raise RuntimeError(f"gh {' '.join(args)} failed: {proc.stderr.strip()}")
    return json.loads(proc.stdout or "null")


def fetch(repo: str = REPO) -> list[dict]:
    """Every open `needs/operator` issue with the body of its latest request comment."""
    issues = _gh_json([
        "issue", "list", "--repo", repo, "--state", "open",
        "--label", UMBRELLA, "--limit", "100",
        "--json", "number,title,labels,updatedAt,createdAt,url",
    ]) or []

    out = []
    for issue in issues:
        labels = [lb["name"] for lb in issue.get("labels", [])]
        record = {
            "issue": issue["number"],
            "title": issue["title"],
            "url": issue.get("url"),
            "labels": labels,
            "kind": next((s for s in ("capture", "network", "hardware", "eyes", "decision")
                          if f"needs/{s}" in labels), None),
            "priority": next((lb.split("/", 1)[1] for lb in labels if lb.startswith("priority/")), None),
            "status": [lb for lb in labels if lb.startswith("status/")],
            "body": None,
            "request_url": None,
        }
        try:
            data = _gh_json(["issue", "view", str(issue["number"]), "--repo", repo,
                             "--json", "comments"]) or {}
            requests = [c for c in data.get("comments", [])
                        if REQUEST_HEADING in (c.get("body") or "")]
        except (RuntimeError, json.JSONDecodeError, subprocess.SubprocessError, OSError) as exc:
            record["error"] = str(exc)
            requests = []
        if requests:
            record["body"] = requests[-1]["body"]
            record["request_url"] = requests[-1].get("url")
            record["request_count"] = len(requests)
        out.append(record)
    out.sort(key=lambda r: r["issue"])
    return out


# ---------------------------------------------------------------------------- parsing


def split_sections(body: str) -> dict[str, list[str]]:
    """Slice a request comment into its named sections, plus `_title` and `_intro`."""
    if REQUEST_HEADING in body:
        body = body[body.index(REQUEST_HEADING):]
    lines = body.splitlines()

    title = ""
    if lines and lines[0].startswith(REQUEST_HEADING):
        title = lines[0][len(REQUEST_HEADING):].lstrip(" —-").strip()
        lines = lines[1:]

    sections: dict[str, list[str]] = {"_title": [title], "_intro": []}
    current = "_intro"
    for line in lines:
        heading = line.strip()
        if heading.startswith("**") and heading.endswith("**"):
            name = heading[2:-2].strip()
            if name in SECTIONS:
                current = name
                sections.setdefault(current, [])
                continue
        sections.setdefault(current, []).append(line)
    return sections


def parse_steps(lines: list[str]) -> list[dict]:
    """`N. [You] ...` / `N. [Claude] ...` with their continuations and sub-bullets."""
    steps: list[dict] = []
    group = ""
    for line in lines:
        sub = _SUBHEAD_RE.match(line.strip())
        if sub:
            group = sub.group(1).lstrip(" —-").strip()
            continue
        m = _STEP_RE.match(line)
        if m:
            steps.append({
                "n": int(m.group(1)),
                "actor": m.group(2),
                "text": m.group(3).strip(),
                "notes": [],
                "group": group,
            })
            continue
        if not steps:
            continue
        stripped = line.strip()
        if not stripped:
            continue
        if _BULLET_RE.match(stripped):
            steps[-1]["notes"].append(_BULLET_RE.sub("", stripped).strip())
        elif line.startswith((" ", "\t")):
            # A wrapped continuation of the step or of its last sub-bullet.
            if steps[-1]["notes"]:
                steps[-1]["notes"][-1] += " " + stripped
            else:
                steps[-1]["text"] += " " + stripped
    return steps


def bullets(lines: list[str]) -> list[str]:
    """Top-level `-`/`*` items, with their wrapped continuation lines folded back in.

    The runbooks wrap at 96 columns, so a bullet is routinely two or three lines. Reading
    them line-by-line silently truncates: #183's third note prompt put its `______` on
    the continuation line and vanished from the page entirely.
    """
    out: list[str] = []
    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue
        if _BULLET_RE.match(stripped) and not line.startswith(("   ", "\t")):
            out.append(_BULLET_RE.sub("", stripped).strip())
        elif out and line.startswith((" ", "\t")):
            out[-1] += " " + stripped
    return out


def parse_prompts(lines: list[str]) -> list[str]:
    """The `... : ______` questions whose answers exist nowhere but the owner's memory."""
    out = []
    for b in bullets(lines):
        if not _BLANK_FIELD_RE.search(b):
            continue
        text = _BLANK_FIELD_RE.sub("", b).strip(" :—-").strip()
        out.append(text if text.endswith(("?", ".", ")")) else text + "?")
    return out


def parse_duration(lines: list[str]) -> int | None:
    m = _DURATION_RE.search("\n".join(lines))
    if not m:
        return None
    value = int(m.group(1))
    return value * 60 if m.group(2).lower() == "hour" else value


def parse_needs(lines: list[str]) -> list[str]:
    return bullets(lines)


_NEGATION = re.compile(r"\b(?:not|no|never|without|avoid)\b[^.]{0,24}$")


def _matches(pattern: str, text: str) -> bool:
    """True if `pattern` hits `text` somewhere it is not being negated.

    A runbook that says "point it at a bookshelf, **not a blank wall**" must not be
    tagged as needing a blank wall (#60 was, and got sent to the wrong room).
    """
    for m in re.finditer(pattern, text):
        if not _NEGATION.search(text[max(0, m.start() - 40):m.start()]):
            return True
    return False


def tags_for(body: str, venue_text: str) -> list[str]:
    scopes = {"body": body.lower(), "venue": venue_text.lower()}
    return [tag for tag, (pattern, _, scope) in TAG_RULES.items()
            if _matches(pattern, scopes[scope])]


def parse_runbook(record: dict) -> dict:
    """Everything the page needs from one request comment."""
    body = record.get("body") or ""
    sections = split_sections(body)
    steps = parse_steps(sections.get("Steps", []))
    needs = parse_needs(sections.get("What you need, and how long", []))
    # What the runbook says you need plus what it asks you to do -- deliberately not the
    # "Why this matters" or troubleshooting prose, which discuss what went wrong before.
    venue_text = "\n".join(needs + [s["text"] for s in steps]
                           + [n for s in steps for n in s["notes"]])
    parsed = {
        **record,
        "request_title": sections["_title"][0],
        "why": [ln for ln in sections.get("Why this matters", []) if ln.strip()],
        "needs": needs,
        "duration_min": parse_duration(sections.get("What you need, and how long", [])),
        "steps": steps,
        "prompts": parse_prompts(sections.get("While it is running, note down", [])),
        "outcome": [ln for ln in sections.get("What I will do with it", []) if ln.strip()],
        "troubles": bullets(sections.get("If something looks wrong", [])),
        "footer": parse_footer(body),
        "tags": tags_for(body, venue_text),
    }
    alias = _ALIAS_RE.search(parsed["request_title"])
    parsed["alias_of"] = int(alias.group(1)) if alias else None
    parsed["done_phrase"] = f"done with #{record['issue']}"
    return parsed


# ------------------------------------------------------------------------- clustering


def _tokens(text: str) -> frozenset[str]:
    text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", text)
    text = re.sub(r"<https?://[^>]*>", " ", text)
    text = re.sub(r"[*`_]", "", text.lower())
    words = re.findall(r"[a-z0-9]+", text)
    return frozenset(w for w in words if w not in _STOPWORDS)


def _jaccard(a: frozenset[str], b: frozenset[str]) -> float:
    if not a or not b:
        return 0.0
    return len(a & b) / len(a | b)


def _setup_end(steps: list[dict]) -> int:
    """Index of the first per-take cycle step; everything before it is setup."""
    for i, step in enumerate(steps):
        low = step["text"].lower()
        if any(marker in low for marker in CYCLE_MARKERS):
            return i
    return len(steps)


def cluster_steps(runbooks: list[dict]) -> list[dict]:
    """Single-linkage clusters of near-duplicate *setup* steps across runbooks.

    Only steps before their own runbook's first cycle step are eligible, so a shared
    cluster is genuinely "do this once per sitting" rather than "these two recordings
    are saved the same way".
    """
    items = []
    for rb in runbooks:
        limit = _setup_end(rb["steps"])
        for i, step in enumerate(rb["steps"][:limit]):
            items.append({"issue": rb["issue"], "index": i, "step": step,
                          "tokens": _tokens(step["text"])})

    clusters: list[dict] = []
    for item in items:
        for cluster in clusters:
            if any(_jaccard(item["tokens"], m["tokens"]) >= SIM_THRESHOLD
                   for m in cluster["members"]):
                cluster["members"].append(item)
                break
        else:
            clusters.append({"members": [item]})

    out = []
    for cid, cluster in enumerate(clusters):
        issues = sorted({m["issue"] for m in cluster["members"]})
        if len(issues) < 2:
            continue
        # The longest phrasing is the safest thing to show: it is the one that carries
        # every caveat the shorter variants dropped.
        canonical = max(cluster["members"], key=lambda m: len(m["step"]["text"]))
        out.append({
            "id": cid,
            "issues": issues,
            "actor": canonical["step"]["actor"],
            "text": canonical["step"]["text"],
            "notes": canonical["step"]["notes"],
            "order": min(m["index"] for m in cluster["members"]),
            "repeats_avoided": len(cluster["members"]) - 1,
        })
    out.sort(key=lambda c: (c["order"], -len(c["issues"])))
    return out


# ---------------------------------------------------------------------------- planning


def _sitting_for(rb: dict) -> str:
    tags = set(rb["tags"])
    if tags & {"viewer-down", "reflash"}:
        return "rig-exclusive"
    if tags & RIG_TAGS:
        return "rig-live"
    return "desk"


def _venue(rb: dict) -> tuple[str, str]:
    """(tag, label) for the place this runbook has to happen."""
    for tag, label in VENUES:
        if tag in rb["tags"]:
            return tag, label
    return "anywhere", "Anywhere"


def _order_hint(rb: dict) -> tuple[int, int]:
    """Props-free work first, then by venue; whatever leaves the rig worst off, last.

    A reflash outranks everything: #166 can leave the board needing a power cycle to
    come back, so nothing else in the sitting should be queued behind it.
    """
    venue, _ = _venue(rb)
    if "reflash" in rb["tags"]:
        rank = 3
    elif "mode-change" in rb["tags"]:
        rank = 2
    else:
        rank = 1
    return rank, VENUE_ORDER.get(venue, 99)


def plan(records: list[dict]) -> dict:
    """Turn the scraped runbooks into sittings, free riders and a savings summary."""
    parsed = [parse_runbook(r) for r in records]
    by_issue = {p["issue"]: p for p in parsed}

    missing = [p for p in parsed if not p["body"]]
    have = [p for p in parsed if p["body"]]

    aliases = [p for p in have if p["alias_of"]]
    primary = [p for p in have if not p["alias_of"]]
    for p in primary:
        p["riders"] = [a for a in aliases if a["alias_of"] == p["issue"]]

    clusters = cluster_steps(primary)
    shared_by_issue: dict[int, set[int]] = {}
    for cluster in clusters:
        for issue in cluster["issues"]:
            shared_by_issue.setdefault(issue, set()).add(cluster["id"])

    # Which of this runbook's own steps the sitting's shared block already covers.
    for p in primary:
        limit = _setup_end(p["steps"])
        covered = set()
        for cluster in clusters:
            if p["issue"] not in cluster["issues"]:
                continue
            tokens = _tokens(cluster["text"])
            for i, step in enumerate(p["steps"][:limit]):
                if _jaccard(tokens, _tokens(step["text"])) >= SIM_THRESHOLD:
                    covered.add(i)
        p["covered_indices"] = sorted(covered)
        p["delta_steps"] = [s for i, s in enumerate(p["steps"]) if i not in covered]

    groups: dict[str, list[dict]] = {}
    for p in primary:
        groups.setdefault(_sitting_for(p), []).append(p)

    sittings = []
    for key, members in groups.items():
        members.sort(key=lambda m: (_order_hint(m), m["issue"]))
        member_issues = {m["issue"] for m in members}
        # Clusters are global, so a cluster's `issues` and `order` can be dominated by a
        # runbook in a different sitting -- #109's "Open the app in your browser" is step
        # 1, which floated the whole browser step above the power-up in the rig block and
        # put a desk-only issue in its "covers" badge. Re-scope both to this sitting.
        block = []
        for cluster in clusters:
            mine = sorted(member_issues & set(cluster["issues"]))
            if len(mine) < 2:
                continue
            local_order = min(
                i for m in members if m["issue"] in mine
                for i, s in enumerate(m["steps"])
                if _jaccard(_tokens(cluster["text"]), _tokens(s["text"])) >= SIM_THRESHOLD
            )
            block.append({**cluster, "issues": mine, "order": local_order})
        block.sort(key=lambda c: (c["order"], -len(c["issues"])))
        heading, rationale, rank = SITTINGS[key]
        covered = sorted(member_issues | {r["issue"] for m in members for r in m["riders"]})

        # A leg is one venue. Two issues in the same leg are the headline saving the
        # step-clustering cannot see: #142 and #144 share no step text at all (Claude
        # starts one of the recordings) but both need you standing in the same
        # metal-free spot with the scanner in your hands.
        legs: dict[str, dict] = {}
        for m in members:
            tag, label = _venue(m)
            leg = legs.setdefault(tag, {"tag": tag, "label": label, "members": []})
            leg["members"].append(m)
        # Order legs the way their members are ordered, so a rig-state change (or a
        # reflash) still sorts last even when its venue would otherwise come first.
        ordered_legs = sorted(
            legs.values(),
            key=lambda lg: min(_order_hint(m) for m in lg["members"]))
        for leg in ordered_legs:
            leg["minutes"] = sum(m["duration_min"] or 0 for m in leg["members"])

        sittings.append({
            "key": key,
            "heading": heading,
            "rationale": rationale,
            "rank": rank,
            "members": members,
            "legs": ordered_legs,
            "shared": block,
            "issues": covered,
            "minutes": sum(m["duration_min"] or 0 for m in members),
            "repeats_avoided": sum(
                len(member_issues & set(c["issues"])) - 1 for c in block),
            "shared_venues": sum(len(lg["members"]) - 1 for lg in ordered_legs
                                 if len(lg["members"]) > 1),
        })
    sittings.sort(key=lambda s: s["rank"])

    separate_trips = len(have)
    separate_minutes = sum(
        (p["duration_min"] or by_issue.get(p["alias_of"], {}).get("duration_min") or 0)
        if p["alias_of"] else (p["duration_min"] or 0)
        for p in have
    )
    batched_minutes = sum(s["minutes"] for s in sittings)

    return {
        "generated": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "sittings": sittings,
        "riders": [{"issue": a["issue"], "title": a["title"], "host": a["alias_of"],
                    "why": a["request_title"], "url": a["url"]} for a in aliases],
        "missing": [{"issue": m["issue"], "title": m["title"], "url": m["url"],
                     "priority": m["priority"], "labels": m["labels"],
                     "error": m.get("error")} for m in missing],
        "summary": {
            "held": len(parsed),
            "with_runbook": len(have),
            "separate_trips": separate_trips,
            "sittings": len(sittings),
            "separate_minutes": separate_minutes,
            "batched_minutes": batched_minutes,
            "repeats_avoided": sum(s["repeats_avoided"] for s in sittings),
            "free_riders": len(aliases),
            "kinds": dict(Counter(p["kind"] or "?" for p in parsed)),
        },
    }


# --------------------------------------------------------------------------- rendering


def md(text: str) -> str:
    """The inline markdown the runbooks actually use, nothing more."""
    out = html.escape(text)
    out = re.sub(r"&lt;(https?://[^&\s]+)&gt;", r'<a href="\1">\1</a>', out)
    out = re.sub(r"\[([^\]]+)\]\((https?://[^)]+)\)", r'<a href="\2">\1</a>', out)
    out = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", out)
    out = re.sub(r"`([^`]+)`", r"<code>\1</code>", out)
    out = re.sub(r"(?<![*\w])\*([^*]+)\*(?!\w)", r"<em>\1</em>", out)
    out = _BLANK_FIELD_RE.sub("&nbsp;", out)
    return out


def _step_html(step: dict, key: str) -> str:
    actor = step["actor"]
    cls = "step claude" if actor == "Claude" else "step you"
    notes = "".join(f"<li>{md(n)}</li>" for n in step["notes"])
    notes_html = f"<ul class='hint'>{notes}</ul>" if notes else ""
    tick = ("<span class='tick claude-tick' title='Claude does this'>&#9881;</span>"
            if actor == "Claude"
            else f"<input type='checkbox' data-k='{html.escape(key)}'>")
    # "Write down the **Gaps** number right now: ______" is a step whose whole purpose is
    # the answer. #60 has four of them, and the runbook says plainly that without them
    # the takes have to be redone -- so give the blank somewhere to go.
    field = ""
    if _BLANK_FIELD_RE.search(step["text"]):
        field = (f"<textarea class='inline' rows='1' data-k='f-{html.escape(key)}' "
                 f"placeholder='write it here'></textarea>")
    return (f"<li class='{cls}'>{tick}"
            f"<div class='body'><span class='who'>{actor}</span>{md(step['text'])}"
            f"{field}{notes_html}</div></li>")


def _issue_card(rb: dict, shared_count: int) -> str:
    pills = "".join(
        f"<span class='pill'>{html.escape(TAG_RULES[t][1])}</span>"
        for t in rb["tags"] if t in TAG_RULES and t not in ("browser", "no-hardware")
    )
    riders = "".join(
        f"<a class='rider' href='{r['url']}'>&#43; also closes #{r['issue']}</a>"
        for r in rb["riders"]
    )
    dur = f"{rb['duration_min']} min" if rb["duration_min"] else "—"
    steps = "".join(_step_html(s, f"i{rb['issue']}-s{i}")
                    for i, s in enumerate(rb["delta_steps"]))
    covered = ""
    if shared_count:
        covered = (f"<p class='covered'>{shared_count} setup step"
                   f"{'s' if shared_count != 1 else ''} for this issue "
                   f"{'are' if shared_count != 1 else 'is'} already done by the shared "
                   f"block above — not repeated here.</p>")

    prompts = ""
    if rb["prompts"]:
        fields = "".join(
            f"<label class='prompt'><span>{md(p)}</span>"
            f"<textarea rows='1' data-k='n{rb['issue']}-{i}' "
            f"placeholder='write it down — this is not saved in the file'></textarea></label>"
            for i, p in enumerate(rb["prompts"])
        )
        prompts = (f"<div class='notes'><h4>Write these down — they are not in the "
                   f"recording</h4>{fields}</div>")

    gate = rb["footer"].get("gate", "")
    gate_html = f"<code>{html.escape(gate)}</code>" if gate else "—"
    outcome = " ".join(rb["outcome"])[:400]
    troubles = "".join(f"<li>{md(t)}</li>" for t in rb["troubles"])
    why = " ".join(rb["why"])

    return f"""
<article class="issue" id="i{rb['issue']}">
  <header>
    <a class="num" href="{rb['url']}">#{rb['issue']}</a>
    <h3>{md(rb['request_title'])}</h3>
    <span class="dur">{dur}</span>
  </header>
  <div class="meta">{pills}{riders}</div>
  <p class="why">{md(why[:420])}{'…' if len(why) > 420 else ''}</p>
  {covered}
  <ol class="steps">{steps}</ol>
  {prompts}
  <div class="done">Then say: <button class="say" data-say="{html.escape(rb['done_phrase'])}">{html.escape(rb['done_phrase'])}</button></div>
  <details class="more">
    <summary>What Claude checks, and what to do if it looks wrong</summary>
    <p><strong>Gate:</strong> {gate_html}</p>
    <p>{md(outcome)}</p>
    <ul class="trouble">{troubles}</ul>
  </details>
</article>"""


def render(report: dict, repo: str = REPO) -> str:
    s = report["summary"]
    saved = s["separate_minutes"] - s["batched_minutes"]

    sittings_html = []
    for sit in report["sittings"]:
        shared = "".join(
            _step_html(c, f"shared-{sit['key']}-{c['id']}") + ""
            for c in sit["shared"]
        )
        shared_block = ""
        if sit["shared"]:
            covers = ", ".join(f"#{i}" for i in sorted(
                {i for c in sit["shared"] for i in c["issues"]}))
            shared_block = f"""
  <div class="shared">
    <h3>Do this once <span class="badge">covers {covers}</span></h3>
    <p class="rationale">Each runbook below repeats these steps. They only have to happen
       once per sitting — {sit['repeats_avoided']} repeats avoided.</p>
    <ol class="steps">{shared}</ol>
  </div>"""

        legs_html = []
        for leg in sit["legs"]:
            issues = ", ".join(f"#{m['issue']}" for m in leg["members"])
            if len(leg["members"]) > 1:
                banner = (f"<div class='merge'><b>One setup, {len(leg['members'])} issues"
                          f"</b> — {issues} all need the same thing and differ only in "
                          f"what you do once you are standing there. Set up once, then "
                          f"work down them without moving.</div>")
            else:
                banner = ""
            cards = "".join(_issue_card(m, len(m["covered_indices"]))
                            for m in leg["members"])
            legs_html.append(f"""
  <div class="leg{' merged' if len(leg['members']) > 1 else ''}">
    <div class="leg-head">
      <h3>{html.escape(leg['label'])}</h3>
      <span class="leg-stats">{issues} · {leg['minutes']} min</span>
    </div>
    {banner}
    {cards}
  </div>""")

        sittings_html.append(f"""
<section class="sitting {sit['key']}">
  <div class="sit-head">
    <h2>{html.escape(sit['heading'])}</h2>
    <div class="sit-stats"><b>{len(sit['issues'])}</b> issues · <b>{sit['minutes']}</b> min</div>
  </div>
  <p class="rationale">{html.escape(sit['rationale'])}</p>
  {shared_block}
  {''.join(legs_html)}
</section>""")

    riders_html = ""
    if report["riders"]:
        rows = "".join(
            f"<li><a href='{r['url']}'>#{r['issue']}</a> "
            f"<span class='rtitle'>{html.escape(r['title'])}</span>"
            f"<span class='arrow'>rides on</span>"
            f"<a class='host' href='#i{r['host']}'>#{r['host']}</a></li>"
            for r in report["riders"]
        )
        riders_html = f"""
<section class="riders">
  <h2>Free — no extra work at all</h2>
  <p class="rationale">These issues ask for an action another issue already asks for.
     Doing the host action closes both. They are listed here so the queue count is
     honest, and they appear nowhere else on this page.</p>
  <ul>{rows}</ul>
</section>"""

    missing_html = ""
    if report["missing"]:
        rows = "".join(
            f"<li><a href='{m['url']}'>#{m['issue']}</a> {html.escape(m['title'])}"
            f"{' — ' + html.escape(m['error']) if m.get('error') else ''}</li>"
            for m in report["missing"]
        )
        missing_html = f"""
<section class="missing">
  <h2>Held on you, but with nothing to act on</h2>
  <p class="rationale">These carry <code>needs/operator</code> without a runbook. There is
     nothing here for you to do until one is written — they are shown so the hold is
     visible rather than silently dropped.</p>
  <ul>{rows}</ul>
</section>"""

    generated = report["generated"].replace("+00:00", "Z")
    return f"""<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>What the scanner needs from you</title>
<style>
:root {{
  --bg:#0b0c10; --panel:rgba(22,24,30,.6); --line:rgba(255,255,255,.08);
  --text:#e2e8f0; --muted:#94a3b8; --accent:#f59e0b; --accent2:#fbbf24;
  --ok:#10b981; --danger:#ef4444;
  --font:'Space Grotesk',-apple-system,BlinkMacSystemFont,sans-serif;
  --mono:'JetBrains Mono',ui-monospace,monospace;
}}
* {{ box-sizing:border-box; }}
body {{ margin:0; background:var(--bg); color:var(--text); font-family:var(--font);
  line-height:1.55; font-size:15px; }}
.wrap {{ max-width:1000px; margin:0 auto; padding:0 20px 120px; }}
header.top {{ position:sticky; top:0; z-index:20; background:rgba(11,12,16,.92);
  backdrop-filter:blur(12px); border-bottom:1px solid var(--line); padding:18px 0 14px;
  margin-bottom:28px; }}
header.top .wrap {{ padding-bottom:0; }}
h1 {{ font-size:26px; margin:0 0 4px; letter-spacing:-.01em; }}
.sub {{ color:var(--muted); font-size:13px; margin:0; }}
.stats {{ display:flex; gap:10px; flex-wrap:wrap; margin-top:14px; }}
.stat {{ background:var(--panel); border:1px solid var(--line); border-radius:10px;
  padding:8px 14px; min-width:96px; }}
.stat b {{ display:block; font-size:22px; font-family:var(--mono); color:var(--accent); }}
.stat span {{ font-size:11px; color:var(--muted); text-transform:uppercase;
  letter-spacing:.06em; }}
.stat.win b {{ color:var(--ok); }}
section {{ margin:0 0 34px; }}
h2 {{ font-size:19px; margin:0; letter-spacing:-.01em; }}
.sit-head {{ display:flex; align-items:baseline; justify-content:space-between; gap:16px;
  border-bottom:2px solid var(--accent); padding-bottom:8px; margin-bottom:10px; }}
.sit-stats {{ font-size:12px; color:var(--muted); font-family:var(--mono); }}
.sit-stats b {{ color:var(--text); }}
.rationale {{ color:var(--muted); font-size:13.5px; margin:0 0 12px; max-width:70ch; }}
.leg {{ margin:0 0 22px; }}
.leg-head {{ display:flex; align-items:baseline; gap:12px; margin-bottom:8px; }}
.leg-head h3 {{ font-size:14px; margin:0; color:var(--text); font-weight:600;
  letter-spacing:.01em; }}
.leg-head h3::before {{ content:'▸ '; color:var(--muted); }}
.leg-stats {{ font-family:var(--mono); font-size:11.5px; color:var(--muted); }}
.leg.merged {{ border-left:2px solid var(--ok); padding-left:14px;
  margin-left:-16px; }}
.merge {{ background:rgba(16,185,129,.10); border:1px solid rgba(16,185,129,.35);
  border-radius:10px; padding:9px 13px; font-size:13px; color:var(--text);
  margin-bottom:12px; max-width:74ch; }}
.merge b {{ color:var(--ok); }}
.shared {{ background:linear-gradient(180deg,rgba(245,158,11,.10),rgba(245,158,11,.03));
  border:1px solid rgba(245,158,11,.35); border-radius:14px; padding:16px 18px;
  margin-bottom:20px; }}
.shared h3 {{ margin:0 0 4px; font-size:16px; }}
.badge {{ font-size:11px; font-family:var(--mono); color:#0b0c10; background:var(--accent);
  padding:2px 8px; border-radius:999px; margin-left:8px; vertical-align:middle; }}
.issue {{ background:var(--panel); border:1px solid var(--line); border-radius:14px;
  padding:16px 18px; margin-bottom:14px; }}
.issue header {{ display:flex; align-items:baseline; gap:10px; }}
.issue h3 {{ font-size:16px; margin:0; flex:1; font-weight:600; }}
.num {{ font-family:var(--mono); color:var(--accent); text-decoration:none; font-size:14px; }}
.dur {{ font-family:var(--mono); font-size:12px; color:var(--muted); }}
.meta {{ display:flex; flex-wrap:wrap; gap:6px; margin:8px 0; }}
.pill {{ font-size:11px; color:var(--muted); border:1px solid var(--line);
  border-radius:999px; padding:2px 9px; background:rgba(255,255,255,.02); }}
.rider {{ font-size:11px; color:#0b0c10; background:var(--ok); border-radius:999px;
  padding:2px 9px; text-decoration:none; font-weight:600; }}
.why {{ color:var(--muted); font-size:13px; margin:6px 0 12px; max-width:72ch; }}
.covered {{ font-size:12px; color:var(--accent2); margin:0 0 10px;
  border-left:2px solid rgba(245,158,11,.5); padding-left:10px; }}
ol.steps {{ list-style:none; padding:0; margin:0; counter-reset:s; }}
li.step {{ display:flex; gap:10px; padding:7px 0; border-top:1px solid var(--line);
  align-items:flex-start; }}
li.step:first-child {{ border-top:none; }}
li.step .body {{ flex:1; }}
li.claude {{ opacity:.62; }}
.who {{ font-family:var(--mono); font-size:10px; text-transform:uppercase;
  letter-spacing:.08em; color:var(--muted); margin-right:8px; }}
li.you .who {{ color:var(--accent); }}
input[type=checkbox] {{ width:18px; height:18px; margin-top:3px; accent-color:var(--accent);
  flex:none; cursor:pointer; }}
.tick.claude-tick {{ width:18px; text-align:center; color:var(--muted); flex:none;
  margin-top:2px; }}
li.step.done .body {{ opacity:.4; text-decoration:line-through; }}
ul.hint {{ margin:6px 0 0; padding-left:16px; color:var(--muted); font-size:12.5px; }}
.notes {{ margin:14px 0 4px; padding:12px 14px; background:rgba(239,68,68,.07);
  border:1px solid rgba(239,68,68,.3); border-radius:10px; }}
.notes h4 {{ margin:0 0 8px; font-size:12px; text-transform:uppercase; letter-spacing:.06em;
  color:#fca5a5; }}
.prompt {{ display:block; margin-bottom:8px; font-size:13px; }}
.prompt span {{ display:block; color:var(--muted); margin-bottom:3px; }}
textarea {{ width:100%; background:rgba(0,0,0,.35); border:1px solid var(--line);
  border-radius:8px; color:var(--text); font-family:var(--font); font-size:13px;
  padding:7px 9px; resize:vertical; }}
textarea:focus {{ outline:none; border-color:var(--accent); }}
textarea.inline {{ margin-top:6px; border-color:rgba(245,158,11,.45);
  background:rgba(245,158,11,.06); }}
.done {{ margin-top:12px; font-size:13px; color:var(--muted); }}
button.say {{ font-family:var(--mono); font-size:13px; background:transparent;
  border:1px dashed var(--accent); color:var(--accent); border-radius:8px;
  padding:4px 10px; cursor:pointer; }}
button.say:hover {{ background:rgba(245,158,11,.12); }}
details.more {{ margin-top:12px; font-size:13px; }}
summary {{ cursor:pointer; color:var(--muted); font-size:12.5px; }}
details.more p, details.more ul {{ color:var(--muted); font-size:12.5px; }}
ul.trouble {{ padding-left:18px; }}
code {{ font-family:var(--mono); font-size:.9em; background:rgba(255,255,255,.06);
  padding:1px 5px; border-radius:4px; }}
.riders ul, .missing ul {{ list-style:none; padding:0; margin:0; }}
.riders li, .missing li {{ display:flex; gap:10px; align-items:center; flex-wrap:wrap;
  background:rgba(16,185,129,.08); border:1px solid rgba(16,185,129,.3);
  border-radius:10px; padding:9px 14px; margin-bottom:8px; font-size:13.5px; }}
.riders a, .missing a {{ color:var(--accent); font-family:var(--mono);
  text-decoration:none; }}
.rtitle {{ flex:1; color:var(--muted); }}
.arrow {{ font-size:11px; color:var(--muted); text-transform:uppercase;
  letter-spacing:.06em; }}
.missing li {{ background:rgba(239,68,68,.08); border-color:rgba(239,68,68,.3); }}
.riders h2, .missing h2 {{ border-bottom:2px solid var(--line); padding-bottom:8px;
  margin-bottom:10px; }}
.bar {{ position:fixed; left:0; right:0; bottom:0; z-index:30;
  background:rgba(11,12,16,.94); backdrop-filter:blur(12px);
  border-top:1px solid var(--line); padding:10px 20px; display:flex; gap:12px;
  align-items:center; justify-content:center; }}
.bar button {{ font-family:var(--font); font-size:13px; border-radius:8px; padding:7px 14px;
  cursor:pointer; border:1px solid var(--line); background:rgba(255,255,255,.04);
  color:var(--text); }}
.bar button.primary {{ background:var(--accent); color:#0b0c10; border-color:var(--accent);
  font-weight:600; }}
.bar .prog {{ font-family:var(--mono); font-size:12px; color:var(--muted); }}
.foot {{ color:var(--muted); font-size:12px; border-top:1px solid var(--line);
  padding-top:14px; }}
@media print {{
  body {{ background:#fff; color:#111; }}
  .bar, header.top {{ position:static; background:none; }}
  .issue, .shared {{ break-inside:avoid; border-color:#ccc; background:none; }}
  details.more[open] {{ display:block; }}
}}
</style>
</head><body>
<header class="top"><div class="wrap">
  <h1>What the scanner needs from you</h1>
  <p class="sub">{s['held']} issues are waiting on you. Batched, they are
     <strong>{s['sittings']} sittings</strong>, not {s['separate_trips']} trips.
     Generated {generated} from <code>{html.escape(repo)}</code>.</p>
  <div class="stats">
    <div class="stat"><b>{s['separate_trips']}</b><span>runbooks</span></div>
    <div class="stat win"><b>{s['sittings']}</b><span>sittings</span></div>
    <div class="stat win"><b>{s['free_riders']}</b><span>free riders</span></div>
    <div class="stat win"><b>{s['repeats_avoided']}</b><span>repeats avoided</span></div>
    <div class="stat"><b>{s['batched_minutes']}</b><span>minutes total</span></div>
    <div class="stat win"><b>{saved}</b><span>minutes saved</span></div>
  </div>
</div></header>
<div class="wrap">
{riders_html}
{''.join(sittings_html)}
{missing_html}
<p class="foot">Every runbook is reproduced above in full — nothing has been summarised
   away. Ticks and notes are saved in this browser only. "Repeats avoided" counts steps
   that appear in more than one runbook's setup and only have to be done once per
   sitting; "minutes saved" is those repeats plus the free riders, using each runbook's
   own stated duration. Regenerate with <code>python host/tools/operator_page.py</code>.</p>
</div>
<div class="bar">
  <span class="prog" id="prog"></span>
  <button class="primary" id="copy">Copy report for Claude</button>
  <button id="reset">Clear ticks &amp; notes</button>
</div>
<script>
const K='roomscan-operator-v1';
const load=()=>{{try{{return JSON.parse(localStorage.getItem(K))||{{}}}}catch(e){{return {{}}}}}};
let st=load();
const save=()=>localStorage.setItem(K,JSON.stringify(st));
function prog(){{
  const b=[...document.querySelectorAll('input[type=checkbox]')];
  document.getElementById('prog').textContent=
    b.filter(x=>x.checked).length+' / '+b.length+' steps done';
}}
document.querySelectorAll('input[type=checkbox]').forEach(b=>{{
  b.checked=!!st[b.dataset.k];
  b.closest('li').classList.toggle('done',b.checked);
  b.addEventListener('change',()=>{{
    st[b.dataset.k]=b.checked; save();
    b.closest('li').classList.toggle('done',b.checked); prog();
  }});
}});
document.querySelectorAll('textarea').forEach(t=>{{
  t.value=st[t.dataset.k]||'';
  t.style.height=t.scrollHeight+'px';
  t.addEventListener('input',()=>{{
    st[t.dataset.k]=t.value; save();
    t.style.height='auto'; t.style.height=t.scrollHeight+'px';
  }});
}});
document.querySelectorAll('button.say').forEach(b=>b.addEventListener('click',()=>{{
  navigator.clipboard.writeText(b.dataset.say);
  const o=b.textContent; b.textContent='copied'; setTimeout(()=>b.textContent=o,900);
}}));
document.getElementById('copy').addEventListener('click',()=>{{
  let out=['## Operator report'];
  const askedBy=t=>t.classList.contains('inline')
    ? t.closest('.body').innerText.split('\\n')[0].replace(/^YOU\\s*/,'').trim()
    : (t.previousElementSibling?t.previousElementSibling.textContent.trim():'');
  document.querySelectorAll('article.issue').forEach(a=>{{
    const boxes=[...a.querySelectorAll('input[type=checkbox]')];
    const notes=[...a.querySelectorAll('textarea')].filter(t=>t.value.trim());
    const anyDone=boxes.some(x=>x.checked);
    if(!anyDone&&!notes.length) return;
    out.push('');
    out.push('### '+a.id.replace('i','#')+' — '+
      boxes.filter(x=>x.checked).length+'/'+boxes.length+' steps ticked');
    notes.forEach(t=>out.push('- '+askedBy(t)+' **'+t.value.trim()+'**'));
  }});
  navigator.clipboard.writeText(out.join('\\n'));
  const b=document.getElementById('copy'); const o=b.textContent;
  b.textContent='copied'; setTimeout(()=>b.textContent=o,1200);
}});
document.getElementById('reset').addEventListener('click',()=>{{
  if(!confirm('Clear every tick and note on this page?')) return;
  st={{}}; save(); location.reload();
}});
prog();
</script>
</body></html>"""


# -------------------------------------------------------------------------------- API


def build_page(repo: str = REPO, out: str | Path | None = DEFAULT_OUT) -> dict:
    """Scrape the held issues, plan the sittings, and write the page.

    `out=None` skips the write and returns the HTML in the result.
    """
    try:
        records = fetch(repo)
    except (RuntimeError, json.JSONDecodeError, subprocess.SubprocessError, OSError) as exc:
        return {"ok": False, "error": str(exc)}

    report = plan(records)
    page = render(report, repo=repo)

    written = None
    if out is not None:
        written = Path(out)
        written.parent.mkdir(parents=True, exist_ok=True)
        written.write_text(page, encoding="utf-8")

    result = {
        "ok": True,
        "repo": repo,
        "generated": report["generated"],
        "path": str(written) if written else None,
        "bytes": len(page.encode("utf-8")),
        "summary": report["summary"],
        "sittings": [
            {"key": s["key"], "heading": s["heading"], "issues": s["issues"],
             "minutes": s["minutes"], "repeats_avoided": s["repeats_avoided"],
             "shared_venues": s["shared_venues"],
             "legs": [{"venue": lg["label"], "minutes": lg["minutes"],
                       "issues": [m["issue"] for m in lg["members"]]}
                      for lg in s["legs"]]}
            for s in report["sittings"]
        ],
        "riders": report["riders"],
        "missing": report["missing"],
    }
    if written is None:
        result["html"] = page
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", default=REPO)
    ap.add_argument("--out", default=str(DEFAULT_OUT),
                    help=f"output path, or '-' for stdout (default: {DEFAULT_OUT})")
    ap.add_argument("--json", action="store_true", help="print the plan instead of a summary")
    args = ap.parse_args()

    to_stdout = args.out == "-"
    result = build_page(repo=args.repo, out=None if to_stdout else args.out)
    if not result["ok"]:
        print(f"could not read the tracker: {result['error']}", file=sys.stderr)
        return 1
    if to_stdout:
        print(result.pop("html"))
        return 0
    if args.json:
        print(json.dumps(result, indent=2))
        return 0

    s = result["summary"]
    print(f"{s['held']} issues waiting on you -> {s['sittings']} sittings "
          f"({s['batched_minutes']} min, down from {s['separate_minutes']})")
    for sit in result["sittings"]:
        issues = ", ".join(f"#{i}" for i in sit["issues"])
        print(f"\n  {sit['heading']}  [{sit['minutes']} min]")
        print(f"    covers {issues}")
        for leg in sit["legs"]:
            tag = "  <- one setup" if len(leg["issues"]) > 1 else ""
            nums = ",".join(f"#{i}" for i in leg["issues"])
            print(f"      {leg['minutes']:>3} min  {leg['venue']:<46} {nums}{tag}")
        if sit["repeats_avoided"]:
            print(f"    {sit['repeats_avoided']} repeated setup steps avoided")
    for r in result["riders"]:
        print(f"\n  free: #{r['issue']} rides on #{r['host']}")
    for m in result["missing"]:
        print(f"\n  warning: #{m['issue']} is held but has no runbook")
    print(f"\nwrote {result['path']} ({result['bytes'] // 1024} KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
