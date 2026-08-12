#!/usr/bin/env python3
"""What is waiting on the owner: open issues labelled `needs/operator`.

An issue held for a physical action carries two things -- the `needs/*` labels, and a
`## 🔧 Operator Request` comment holding the runbook plus a machine-readable footer.
This reads both back so "what do you need from me?" is one call instead of a chain of
`gh` invocations and hand-parsing.

The load-bearing check is `has_request`. The label is a *promise that the instructions
exist*; a held issue with no runbook comment is a dead end for the owner, who sees the
label and has nothing to act on. That case is reported rather than hidden, because it
is invisible from the issue list.

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
        }
        if entry["kind"] is None:
            problems.append(f"#{issue['number']} has {UMBRELLA} but no needs/<subtype>")

        if include_comments:
            try:
                data = _gh_json(["issue", "view", str(issue["number"]),
                                 "--repo", repo, "--json", "comments"])
                comments = (data or {}).get("comments", [])
            except (RuntimeError, json.JSONDecodeError, subprocess.SubprocessError, OSError):
                comments = []
                entry["request_error"] = "could not read comments"

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
                if not entry["request"]["footer"]:
                    problems.append(f"#{issue['number']} request has no parseable footer")
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
        elif e["has_request"] is False:
            print("      !! no Operator Request comment -- nothing to act on")
    for p in report["problems"]:
        print(f"\n  warning: {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
