#!/usr/bin/env python3
"""Lint GitHub issue labels against invariants.

Enforces mechanical rules on issue labels:
1. needs/operator ⇒ must have a needs/* subtype AND a status hold
2. every open issue must have exactly one priority/now|next|later
3. exactly one of bug/work-item/data-collection per issue
4. (advisory) area/* should be present

The pure function accepts an injectable issues list so tests run fully offline.
"""
from __future__ import annotations

import argparse
import json
import subprocess
from typing import Callable


REPO = "hellosamblack/lidar-roomscanner"

# Label categories and their requirements
PRIORITY_LABELS = ("priority/now", "priority/next", "priority/later")
CATEGORY_LABELS = ("bug", "work-item", "data-collection")
NEEDS_SUBTYPES = ("capture", "network", "hardware", "eyes", "decision")
STATUS_HOLDS = ("status/fix-unverified", "status/blocked")


def lint_issues(
    issues: list[dict],
) -> dict:
    """Check label invariants on a list of issues.

    Args:
        issues: List of dicts with keys: number (int), labels (list of str), state (str)

    Returns:
        A dict with:
        - violations: list of violation dicts, each with:
          - issue: issue number
          - rule: rule name ("operator_pairing", "priority_tier", "category", "area_advisory")
          - message: human-readable message
        - summary: dict with counts per rule
        - ok: True if no hard violations (rules 1-3); advisory violations don't fail ok
        - violation_count: count of hard violations only
        - advisory_count: count of advisory-only violations
    """
    hard_violations = []
    advisory_violations = []
    rule_counts = {
        "operator_pairing": 0,
        "priority_tier": 0,
        "category": 0,
        "area_advisory": 0,
    }

    for issue in issues:
        number = issue.get("number")
        labels = issue.get("labels", [])
        state = issue.get("state", "open")

        # Only check open issues
        if state != "open":
            continue

        # Rule 1: needs/operator requires a subtype AND a status hold
        if "needs/operator" in labels:
            has_subtype = any(f"needs/{st}" in labels for st in NEEDS_SUBTYPES)
            has_status_hold = any(sh in labels for sh in STATUS_HOLDS)

            if not has_subtype:
                hard_violations.append({
                    "issue": number,
                    "rule": "operator_pairing",
                    "message": f"#{number}: has needs/operator but no needs/<subtype> "
                    f"({', '.join(NEEDS_SUBTYPES)})"
                })
                rule_counts["operator_pairing"] += 1

            if not has_status_hold:
                hard_violations.append({
                    "issue": number,
                    "rule": "operator_pairing",
                    "message": f"#{number}: has needs/operator but no status hold "
                    f"({', '.join(STATUS_HOLDS)})"
                })
                rule_counts["operator_pairing"] += 1

        # Rule 2: exactly one priority tier
        priority_labels = [lb for lb in labels if lb in PRIORITY_LABELS]
        if len(priority_labels) != 1:
            if len(priority_labels) == 0:
                hard_violations.append({
                    "issue": number,
                    "rule": "priority_tier",
                    "message": f"#{number}: missing priority tier "
                    f"({', '.join(PRIORITY_LABELS)})"
                })
            else:
                hard_violations.append({
                    "issue": number,
                    "rule": "priority_tier",
                    "message": f"#{number}: has {len(priority_labels)} priority labels, "
                    f"need exactly 1: {', '.join(priority_labels)}"
                })
            rule_counts["priority_tier"] += 1

        # Rule 3: exactly one category
        category_labels = [lb for lb in labels if lb in CATEGORY_LABELS]
        if len(category_labels) != 1:
            if len(category_labels) == 0:
                hard_violations.append({
                    "issue": number,
                    "rule": "category",
                    "message": f"#{number}: missing category "
                    f"({', '.join(CATEGORY_LABELS)})"
                })
            else:
                hard_violations.append({
                    "issue": number,
                    "rule": "category",
                    "message": f"#{number}: has {len(category_labels)} category labels, "
                    f"need exactly 1: {', '.join(category_labels)}"
                })
            rule_counts["category"] += 1

        # Rule 4: (advisory) should have an area/* label
        area_labels = [lb for lb in labels if lb.startswith("area/")]
        if not area_labels:
            advisory_violations.append({
                "issue": number,
                "rule": "area_advisory",
                "message": f"#{number}: should have an area/* label (advisory)"
            })
            rule_counts["area_advisory"] += 1

    # Combine violations: hard first, then advisory
    all_violations = hard_violations + advisory_violations

    return {
        "violations": all_violations,
        "summary": rule_counts,
        "ok": len(hard_violations) == 0,
        "violation_count": len(hard_violations),
        "advisory_count": len(advisory_violations),
    }


def _gh_json(args: list[str]) -> object:
    """Run gh with JSON output, raise on error."""
    proc = subprocess.run(
        ["gh", *args],
        capture_output=True,
        text=True,
        timeout=60
    )
    if proc.returncode != 0:
        raise RuntimeError(f"gh {' '.join(args)} failed: {proc.stderr.strip()}")
    return json.loads(proc.stdout or "null")


def fetch_issues(repo: str = REPO) -> list[dict]:
    """Fetch all open issues with their labels.

    Args:
        repo: GitHub repository in format "owner/name"

    Returns:
        List of issue dicts with keys: number, labels, state
    """
    issues = _gh_json([
        "issue", "list", "--repo", repo, "--state", "open",
        "--limit", "100",
        "--json", "number,labels,state",
    ])
    return issues or []


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repo", default=REPO, help="GitHub repository (default: %(default)s)")
    ap.add_argument("--json", action="store_true", help="output raw JSON")
    args = ap.parse_args()

    try:
        issues = fetch_issues(repo=args.repo)
    except (RuntimeError, json.JSONDecodeError, subprocess.SubprocessError, OSError) as exc:
        print(f"Error fetching issues: {exc}")
        return 1

    result = lint_issues(issues)

    if args.json:
        print(json.dumps(result, indent=2))
        return 0 if result["ok"] else 1

    if result["ok"]:
        if result["advisory_count"] > 0:
            print(f"✓ No hard violations (but {result['advisory_count']} advisory)\n")
        else:
            print("✓ All issues pass label lint checks")
            return 0
    else:
        print(f"Found {result['violation_count']} hard violation(s):\n")

    # Group violations by rule
    by_rule = {}
    for v in result["violations"]:
        rule = v["rule"]
        if rule not in by_rule:
            by_rule[rule] = []
        by_rule[rule].append(v["message"])

    for rule in sorted(by_rule.keys()):
        is_advisory = rule == "area_advisory"
        label = f"  {rule} ({'advisory' if is_advisory else 'hard'}):"
        print(label)
        for msg in by_rule[rule]:
            print(f"    {msg}")
        print()

    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
