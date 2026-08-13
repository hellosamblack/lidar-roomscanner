#!/usr/bin/env python3
"""What is waiting on the owner: open issues labelled `needs/operator`.

An issue held for a physical action carries two things -- the `needs/*` labels, and a
`## 🔧 Operator Request` comment holding the runbook plus a machine-readable footer.
This reads both back so "what do you need from me?" is one call instead of a chain of
`gh` invocations and hand-parsing.

The load-bearing check is `has_request`. The label is a *promise that the instructions
exist*; a held issue with no runbook comment is a dead end for the owner, who sees the
label and has nothing to act on. That case is reported rather than hidden, because it
is invisible from the issue list -- *unless* the issue is `priority/later`: a hold can
be labelled `needs/operator` before its runbook is written, deliberately, so it is
findable while parked behind the now/next tiers. That is not a dead end (nobody is
waiting on it yet), so it is reported as `parked`, not folded into `problems`. Anything
else missing a runbook -- `priority/now`/`priority/next`, or a comment read failure --
is still a problem: those ARE waited-on and a reader who skips `problems` because it is
full of noise is the exact failure this distinction exists to prevent.

See `.agents/skills/operator-request/SKILL.md`.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
from datetime import datetime, timezone

REPO = "hellosamblack/lidar-roomscanner"
UMBRELLA = "needs/operator"
SUBTYPES = ("capture", "network", "hardware", "eyes", "decision")

REQUEST_HEADING = "## 🔧 Operator Request"
RESULT_HEADING = "## ✅ Operator Result"
RESULT_HEADING_FAIL = "## ❌ Operator Result"

# A hold labelled with this priority is allowed to have no runbook yet -- it is
# deliberately parked behind the now/next tiers, not a broken hold. See #175.
DEFERRED_LABEL = "priority/later"

_FOOTER = re.compile(r"<!--\s*operator-request:\s*(.*?)\s*-->", re.DOTALL)


def parse_footer(body: str) -> dict:
    """Pull the `key=value` footer out of a request comment.

    Returns {} when absent -- an older or hand-written request is still a request, it
    just cannot be routed automatically.
    """
    m = _FOOTER.search(body or "")
    if not m:
        return {}
    out = {}
    for tok in m.group(1).split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def _gh_json(args: list[str]) -> object:
    proc = subprocess.run(["gh", *args], capture_output=True, text=True, timeout=60)
    if proc.returncode != 0:
        raise RuntimeError(f"gh {' '.join(args)} failed: {proc.stderr.strip()}")
    return json.loads(proc.stdout or "null")


def _age_days(stamp: str) -> float | None:
    try:
        then = datetime.fromisoformat(stamp.replace("Z", "+00:00"))
    except (ValueError, AttributeError):
        return None
    return round((datetime.now(timezone.utc) - then).total_seconds() / 86400.0, 1)


def _classify(labels: list[str]) -> str | None:
    for s in SUBTYPES:
        if f"needs/{s}" in labels:
            return s
    return None


def _is_deferred(labels: list[str]) -> bool:
    """True for a hold that is allowed to have no runbook yet.

    Only `priority/later` qualifies -- a hold with no priority label at all still
    reads as a broken/dead-end hold, because nothing marked it as deliberately
    parked. `priority/now` and `priority/next` never qualify: those ARE waited-on.
    """
    return DEFERRED_LABEL in labels


def collect(repo: str = REPO, include_comments: bool = True) -> dict:
    """Every open issue waiting on an owner action, in issue-number order.

    `include_comments=False` skips the per-issue comment fetch (one `gh` call each),
    giving a fast label-only listing with no runbook detail.
    """
    try:
        issues = _gh_json([
            "issue", "list", "--repo", repo, "--state", "open",
            "--label", UMBRELLA, "--limit", "100",
            "--json", "number,title,labels,updatedAt,createdAt",
        ])
    except (RuntimeError, json.JSONDecodeError, subprocess.SubprocessError, OSError) as exc:
        return {"ok": False, "error": str(exc), "pending": [], "count": 0}

    pending, problems = [], []
    for issue in issues or []:
        labels = [lb["name"] for lb in issue.get("labels", [])]
        entry = {
            "issue": issue["number"],
            "title": issue["title"],
            "kind": _classify(labels),
            "labels": labels,
            "status": [lb for lb in labels if lb.startswith("status/")],
            "age_days": _age_days(issue.get("createdAt", "")),
            "updated": issue.get("updatedAt"),
            "has_request": None,
            "request": None,
            "parked": False,
        }
        if entry["kind"] is None:
            problems.append(f"#{issue['number']} has {UMBRELLA} but no needs/<subtype>")

        if include_comments:
            read_failed = False
            try:
                data = _gh_json(["issue", "view", str(issue["number"]),
                                 "--repo", repo, "--json", "comments"])
                comments = (data or {}).get("comments", [])
            except (RuntimeError, json.JSONDecodeError, subprocess.SubprocessError, OSError):
                comments = []
                entry["request_error"] = "could not read comments"
                read_failed = True

            requests = [c for c in comments if REQUEST_HEADING in (c.get("body") or "")]
            results = [c for c in comments
                       if RESULT_HEADING in (c.get("body") or "")
                       or RESULT_HEADING_FAIL in (c.get("body") or "")]
            entry["has_request"] = bool(requests)
            entry["request_count"] = len(requests)
            entry["result_count"] = len(results)
            if requests:
                latest = requests[-1]
                body = latest.get("body") or ""
                entry["request"] = {
                    "posted": latest.get("createdAt"),
                    "url": latest.get("url"),
                    "title": _request_title(body),
                    "footer": parse_footer(body),
                }
                # Malformed is still a problem regardless of priority -- a footer
                # that fails to parse means a runbook was written, not deferred.
                if not entry["request"]["footer"]:
                    problems.append(f"#{issue['number']} request has no parseable footer")
            elif read_failed:
                # We could not even see whether a request exists, so a `priority/later`
                # label cannot excuse it -- report it, same as any other read failure.
                problems.append(
                    f"#{issue['number']} is labelled {UMBRELLA} but its comments could "
                    f"not be read -- cannot tell whether it has a runbook")
            elif _is_deferred(labels):
                entry["parked"] = True
            else:
                problems.append(
                    f"#{issue['number']} is labelled {UMBRELLA} but has no Operator Request "
                    f"comment -- the owner has nothing to act on")

        pending.append(entry)

    pending.sort(key=lambda e: e["issue"])
    return {"ok": True, "count": len(pending), "pending": pending, "problems": problems,
            "repo": repo, "detailed": include_comments}


def _request_title(body: str) -> str:
    for line in (body or "").splitlines():
        if line.startswith(REQUEST_HEADING):
            return line[len(REQUEST_HEADING):].lstrip(" —-").strip()
    return ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repo", default=REPO)
    ap.add_argument("--fast", action="store_true", help="labels only; skip runbook detail")
    ap.add_argument("--json", action="store_true", help="print the raw structure")
    args = ap.parse_args()

    report = collect(repo=args.repo, include_comments=not args.fast)
    if args.json:
        print(json.dumps(report, indent=2))
        return 0 if report["ok"] else 1

    if not report["ok"]:
        print(f"could not read the tracker: {report['error']}")
        return 1
    if not report["count"]:
        print("Nothing is waiting on you.")
        return 0

    print(f"{report['count']} issue(s) waiting on you:\n")
    for e in report["pending"]:
        kind = e["kind"] or "?"
        print(f"  #{e['issue']}  [{kind}]  {e['title']}")
        if e["request"]:
            print(f"      {e['request']['title']}  ({e['request']['url']})")
        elif e["parked"]:
            print("      -- parked (priority/later), no runbook yet")
        elif e["has_request"] is False:
            print("      !! no Operator Request comment -- nothing to act on")
    for p in report["problems"]:
        print(f"\n  warning: {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
