"""Tests for tracker_lint: mechanical GitHub issue label validation.

Covers all four invariant rules with both passing and violating cases:
1. needs/operator ⇒ subtype + status hold
2. exactly one priority/now|next|later per open issue
3. exactly one of bug/work-item/data-collection per issue
4. (advisory) area/* should be present
"""
from __future__ import annotations

import pytest

from tools.tracker_lint import lint_issues


class TestOperatorPairingRule:
    """Rule 1: needs/operator ⇒ needs/<subtype> AND status hold."""

    def test_pass_with_all_required_labels(self):
        """A needs/operator with subtype and status hold passes."""
        issues = [{
            "number": 100,
            "labels": [
                "needs/operator", "needs/capture",
                "status/fix-unverified",
                "priority/now", "bug"
            ],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]
        assert result["violation_count"] == 0

    def test_fail_no_subtype(self):
        """A needs/operator without a needs/<subtype> violates."""
        issues = [{
            "number": 100,
            "labels": [
                "needs/operator",
                "status/fix-unverified",
                "priority/now", "bug"
            ],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert not result["ok"]
        assert result["violation_count"] == 1
        violation = result["violations"][0]
        assert violation["issue"] == 100
        assert violation["rule"] == "operator_pairing"
        assert "needs/<subtype>" in violation["message"]

    def test_fail_no_status_hold(self):
        """A needs/operator without status/fix-unverified or status/blocked violates."""
        issues = [{
            "number": 100,
            "labels": [
                "needs/operator", "needs/capture",
                "priority/now", "bug"
            ],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert not result["ok"]
        assert result["violation_count"] == 1
        violation = result["violations"][0]
        assert violation["issue"] == 100
        assert violation["rule"] == "operator_pairing"
        assert "status hold" in violation["message"]

    def test_fail_missing_both_subtype_and_status(self):
        """A needs/operator missing both subtype and status is two violations."""
        issues = [{
            "number": 100,
            "labels": [
                "needs/operator",
                "priority/now", "bug"
            ],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert not result["ok"]
        # 2 operator violations (subtype + status) + 1 advisory (area)
        assert result["violation_count"] == 2
        assert result["advisory_count"] == 1
        # Check the hard violations specifically
        hard_violations = [v for v in result["violations"] if v["rule"] == "operator_pairing"]
        assert len(hard_violations) == 2
        assert all(v["issue"] == 100 for v in hard_violations)
        assert all(v["rule"] == "operator_pairing" for v in hard_violations)

    def test_pass_with_status_blocked_instead_of_fix_unverified(self):
        """Either status/fix-unverified or status/blocked satisfies the rule."""
        issues = [{
            "number": 100,
            "labels": [
                "needs/operator", "needs/hardware",
                "status/blocked",
                "priority/now", "bug"
            ],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]

    def test_pass_with_any_valid_subtype(self):
        """Any of the five subtypes is valid: capture/network/hardware/eyes/decision."""
        for subtype in ["capture", "network", "hardware", "eyes", "decision"]:
            issues = [{
                "number": 100,
                "labels": [
                    "needs/operator", f"needs/{subtype}",
                    "status/fix-unverified",
                    "priority/now", "bug"
                ],
                "state": "open"
            }]
            result = lint_issues(issues)
            assert result["ok"], f"needs/{subtype} should be valid"

    def test_ignore_needs_operator_on_closed_issues(self):
        """The rule only applies to open issues."""
        issues = [{
            "number": 100,
            "labels": ["needs/operator"],  # No subtype, no status hold
            "state": "closed"
        }]
        result = lint_issues(issues)
        assert result["ok"]  # Ignored because state is closed


class TestPriorityTierRule:
    """Rule 2: every open issue must have exactly one priority/now|next|later."""

    def test_pass_with_priority_now(self):
        """An issue with priority/now passes."""
        issues = [{
            "number": 100,
            "labels": ["priority/now", "bug"],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]

    def test_pass_with_priority_next(self):
        """An issue with priority/next passes."""
        issues = [{
            "number": 100,
            "labels": ["priority/next", "bug"],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]

    def test_pass_with_priority_later(self):
        """An issue with priority/later passes."""
        issues = [{
            "number": 100,
            "labels": ["priority/later", "bug"],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]

    def test_fail_no_priority(self):
        """An open issue without a priority tier violates."""
        issues = [{
            "number": 100,
            "labels": ["bug"],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert not result["ok"]
        assert result["violation_count"] == 1
        violation = result["violations"][0]
        assert violation["issue"] == 100
        assert violation["rule"] == "priority_tier"
        assert "missing priority tier" in violation["message"]

    def test_fail_multiple_priorities(self):
        """An issue with multiple priority tiers violates."""
        issues = [{
            "number": 100,
            "labels": ["priority/now", "priority/next", "bug"],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert not result["ok"]
        assert result["violation_count"] == 1
        violation = result["violations"][0]
        assert violation["issue"] == 100
        assert violation["rule"] == "priority_tier"
        assert "2 priority labels" in violation["message"]

    def test_ignore_priority_on_closed_issues(self):
        """Closed issues are not checked for priority."""
        issues = [{
            "number": 100,
            "labels": ["bug"],  # No priority
            "state": "closed"
        }]
        result = lint_issues(issues)
        assert result["ok"]


class TestCategoryRule:
    """Rule 3: exactly one of bug/work-item/data-collection per issue."""

    def test_pass_with_bug(self):
        """An issue with bug category passes."""
        issues = [{
            "number": 100,
            "labels": ["bug", "priority/now"],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]

    def test_pass_with_work_item(self):
        """An issue with work-item category passes."""
        issues = [{
            "number": 100,
            "labels": ["work-item", "priority/now"],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]

    def test_pass_with_data_collection(self):
        """An issue with data-collection category passes."""
        issues = [{
            "number": 100,
            "labels": ["data-collection", "priority/now"],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]

    def test_fail_no_category(self):
        """An issue without a category violates."""
        issues = [{
            "number": 100,
            "labels": ["priority/now"],  # No category
            "state": "open"
        }]
        result = lint_issues(issues)
        assert not result["ok"]
        assert result["violation_count"] == 1
        violation = result["violations"][0]
        assert violation["issue"] == 100
        assert violation["rule"] == "category"
        assert "missing category" in violation["message"]

    def test_fail_multiple_categories(self):
        """An issue with multiple categories violates."""
        issues = [{
            "number": 100,
            "labels": ["bug", "work-item", "priority/now"],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert not result["ok"]
        assert result["violation_count"] == 1
        violation = result["violations"][0]
        assert violation["issue"] == 100
        assert violation["rule"] == "category"
        assert "2 category labels" in violation["message"]

    def test_ignore_category_on_closed_issues(self):
        """Closed issues are not checked for category (only rules 1-2 are hard)."""
        issues = [{
            "number": 100,
            "labels": [],  # No category, no priority
            "state": "closed"
        }]
        result = lint_issues(issues)
        assert result["ok"]


class TestAreaAdvisoryRule:
    """Rule 4: (advisory) area/* should be present."""

    def test_pass_with_area(self):
        """An issue with an area/* label passes."""
        issues = [{
            "number": 100,
            "labels": ["area/firmware", "priority/now", "bug"],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]
        assert result["advisory_count"] == 0

    def test_advisory_no_area(self):
        """An issue without area/* is an advisory violation, not a hard failure."""
        issues = [{
            "number": 100,
            "labels": ["priority/now", "bug"],  # No area
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]  # Advisory violations don't fail ok
        assert result["violation_count"] == 0
        assert result["advisory_count"] == 1
        violations = [v for v in result["violations"] if v["rule"] == "area_advisory"]
        assert len(violations) == 1
        assert "advisory" in violations[0]["message"]

    def test_multiple_areas_are_ok(self):
        """Multiple area/* labels are allowed."""
        issues = [{
            "number": 100,
            "labels": ["area/firmware", "area/host-slam", "priority/now", "bug"],
            "state": "open"
        }]
        result = lint_issues(issues)
        violations = [v for v in result["violations"] if v["rule"] == "area_advisory"]
        assert not violations  # No advisory violations for this issue
        assert result["advisory_count"] == 0

    def test_ignore_area_on_closed_issues(self):
        """Closed issues are not checked for area (only rules 1-2 are hard)."""
        issues = [{
            "number": 100,
            "labels": ["priority/now", "bug"],  # No area
            "state": "closed"
        }]
        result = lint_issues(issues)
        assert result["ok"]
        assert result["advisory_count"] == 0


class TestMultipleIssues:
    """Test linting a batch of issues with mixed violations."""

    def test_multiple_issues_with_different_violations(self):
        """Linting multiple issues collects all violations."""
        issues = [
            {
                "number": 100,
                "labels": ["needs/operator", "priority/now", "bug"],  # No subtype, no status
                "state": "open"
            },
            {
                "number": 101,
                "labels": ["priority/now", "priority/next", "bug"],  # Multiple priorities
                "state": "open"
            },
            {
                "number": 102,
                "labels": ["priority/now", "bug"],  # OK except for area (advisory)
                "state": "open"
            },
        ]
        result = lint_issues(issues)
        assert not result["ok"]  # Hard violations exist
        # Issue 100: 2 operator (subtype and status) + advisory area
        # Issue 101: 1 priority + advisory area
        # Issue 102: advisory area only
        # Total: 3 hard violations, 3 advisory violations
        assert result["violation_count"] == 3  # Only hard violations
        assert result["advisory_count"] == 3  # Only advisory violations
        assert result["summary"]["operator_pairing"] == 2
        assert result["summary"]["priority_tier"] == 1
        assert result["summary"]["area_advisory"] == 3

    def test_audit_example_from_brief_case_1(self):
        """#95 and #60 -- both priority/now with needs/operator but no status hold."""
        issues = [{
            "number": 95,
            "labels": ["priority/now", "needs/operator", "needs/capture", "bug"],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert not result["ok"]
        violations = [v for v in result["violations"] if v["rule"] == "operator_pairing"]
        assert len(violations) == 1
        assert "status hold" in violations[0]["message"]

    def test_audit_example_from_brief_case_2(self):
        """#141 had no priority/* tier at all."""
        issues = [{
            "number": 141,
            "labels": ["bug"],  # No priority
            "state": "open"
        }]
        result = lint_issues(issues)
        violations = [v for v in result["violations"] if v["rule"] == "priority_tier"]
        assert len(violations) == 1
        assert "missing priority tier" in violations[0]["message"]

    def test_result_summary_structure(self):
        """The summary dict has correct structure and counts."""
        issues = [
            {
                "number": 100,
                "labels": ["needs/operator"],  # No subtype, no status, no priority, no category
                "state": "open"
            },
        ]
        result = lint_issues(issues)
        assert "summary" in result
        assert "operator_pairing" in result["summary"]
        assert "priority_tier" in result["summary"]
        assert "category" in result["summary"]
        assert "area_advisory" in result["summary"]
        # Issue 100: 2 operator violations (hard) + 1 priority (hard) + 1 category (hard) + 1 area (advisory)
        assert result["summary"]["operator_pairing"] == 2
        assert result["summary"]["priority_tier"] == 1
        assert result["summary"]["category"] == 1
        assert result["summary"]["area_advisory"] == 1
        assert result["violation_count"] == 4  # Only hard violations
        assert result["advisory_count"] == 1  # Only advisory violations

    def test_empty_issues_list(self):
        """An empty issues list passes with no violations."""
        result = lint_issues([])
        assert result["ok"]
        assert result["violation_count"] == 0
        assert result["summary"]["operator_pairing"] == 0
        assert result["summary"]["priority_tier"] == 0
        assert result["summary"]["category"] == 0
        assert result["summary"]["area_advisory"] == 0

    def test_violation_dict_structure(self):
        """Each violation has the required fields."""
        issues = [{
            "number": 100,
            "labels": ["bug"],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["violation_count"] == 1
        violation = result["violations"][0]
        assert "issue" in violation
        assert "rule" in violation
        assert "message" in violation
        assert isinstance(violation["issue"], int)
        assert isinstance(violation["rule"], str)
        assert isinstance(violation["message"], str)


class TestIntegration:
    """Integration tests combining multiple rules."""

    def test_completely_valid_issue(self):
        """A fully compliant issue passes all checks."""
        issues = [{
            "number": 100,
            "labels": [
                "priority/now",
                "bug",
                "area/firmware",
            ],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]
        assert result["violation_count"] == 0
        assert result["advisory_count"] == 0

    def test_valid_issue_without_area(self):
        """An issue with all hard requirements passes, even without area (advisory only)."""
        issues = [{
            "number": 100,
            "labels": [
                "priority/now",
                "bug",
            ],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]  # Passes because area is only advisory
        assert result["violation_count"] == 0
        assert result["advisory_count"] == 1

    def test_complex_valid_issue_with_all_optional_fields(self):
        """A valid issue with all optional labels still passes."""
        issues = [{
            "number": 100,
            "labels": [
                "priority/now",
                "bug",
                "area/firmware",
                "area/host-slam",
                "status/in-progress",  # Extra status label
                "frontend",  # Extra label
            ],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]
        assert result["violation_count"] == 0

    def test_operator_hold_with_all_details(self):
        """A properly-formed operator hold with everything required."""
        issues = [{
            "number": 200,
            "labels": [
                "priority/now",
                "needs/operator",
                "needs/capture",
                "status/fix-unverified",
                "work-item",
                "area/host-slam",
            ],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert result["ok"]
        assert result["violation_count"] == 0
        assert result["advisory_count"] == 0

    def test_issue_with_all_hard_violations(self):
        """An issue violating all hard rules demonstrates the full checker."""
        issues = [{
            "number": 999,
            "labels": [
                "needs/operator",  # No subtype, no status → operator_pairing
                # No priority → priority_tier
                # No category → category
                # No area → area_advisory
            ],
            "state": "open"
        }]
        result = lint_issues(issues)
        assert not result["ok"]
        # 2 operator violations (missing subtype and status) + 1 priority + 1 category = 4 hard
        # + 1 area advisory = 1 advisory
        assert result["violation_count"] == 4  # Hard violations only
        assert result["advisory_count"] == 1  # Advisory violations only
        assert result["summary"]["operator_pairing"] == 2
        assert result["summary"]["priority_tier"] == 1
        assert result["summary"]["category"] == 1
        assert result["summary"]["area_advisory"] == 1
