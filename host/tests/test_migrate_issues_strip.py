"""Pure-function guards for `migrate_issues strip-prefixes`.

These cover the title/body transforms only — no `gh` calls, no network — so the
idempotency and whole-token-boundary behaviour is pinned offline.
"""
import importlib.util
import sys
from pathlib import Path

_SPEC = importlib.util.spec_from_file_location(
    "migrate_issues",
    Path(__file__).resolve().parents[1] / "tools" / "migrate_issues.py",
)
mi = importlib.util.module_from_spec(_SPEC)
sys.modules["migrate_issues"] = mi  # dataclass resolves cls.__module__ here
_SPEC.loader.exec_module(mi)


def test_strip_removes_the_matching_prefix():
    assert mi.strip_title_prefix("SLAM-14", "SLAM-14: Evaluate BoW") == "Evaluate BoW"
    assert mi.strip_title_prefix("DC-K", "DC-K: Paired capture") == "Paired capture"


def test_strip_is_idempotent_and_leaves_foreign_prefixes():
    # already stripped -> unchanged
    assert mi.strip_title_prefix("SLAM-14", "Evaluate BoW") == "Evaluate BoW"
    # a different id's prefix is not this issue's prefix -> untouched
    assert mi.strip_title_prefix("SLAM-1", "SLAM-14: Evaluate") == "SLAM-14: Evaluate"


def test_body_marker_added_when_absent():
    out = mi.ensure_legacy_id_in_body("BUG-051", "The gimbal gate fired.")
    assert out == "**Legacy ID:** BUG-051\n\nThe gimbal gate fired."


def test_body_marker_skipped_when_id_present_as_token():
    body = "# BUG-098 — Preview mode\n\nDetails."
    assert mi.ensure_legacy_id_in_body("BUG-098", body) == body


def test_body_marker_is_idempotent():
    once = mi.ensure_legacy_id_in_body("SLAM-9", "Keyframe gate.")
    twice = mi.ensure_legacy_id_in_body("SLAM-9", once)
    assert once == twice


def test_body_marker_respects_token_boundary():
    # a body citing SLAM-14 must NOT count as containing SLAM-1
    body = "References SLAM-14 and SLAM-3 as prior art."
    out = mi.ensure_legacy_id_in_body("SLAM-1", body)
    assert out.startswith("**Legacy ID:** SLAM-1\n\n")
