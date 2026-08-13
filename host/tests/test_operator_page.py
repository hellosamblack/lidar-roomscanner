"""The operator page's derivations, pinned against the runbooks they were built from.

Every test here is a bug that shipped once during #184. The page is a *plan* the owner
acts on -- it sends them to a room with a piece of hardware -- so a derivation that is
quietly wrong costs a trip, which is the exact thing the page exists to save. The
fixtures below are trimmed but structurally faithful copies of the real runbooks
(#60, #141, #142, #144, #183); where a test pins a real issue's behaviour it says so.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.operator_page import (  # noqa: E402
    RIG_TAGS,
    SIM_THRESHOLD,
    _jaccard,
    _matches,
    _setup_end,
    _sitting_for,
    _tokens,
    _venue,
    bullets,
    cluster_steps,
    parse_prompts,
    parse_runbook,
    parse_steps,
    plan,
    render,
    split_sections,
)

REQUEST = "## 🔧 Operator Request"


def _runbook(issue: int, title: str, needs: str, steps: str, *,
             notes: str = "", kind: str = "capture", gate: str = "some.gate") -> dict:
    body = f"""{REQUEST} — {title}

**Why this matters**

Because it does.

**What you need, and how long**

{needs}

**Steps**

{steps}
{f'''
**While it is running, note down**

{notes}
''' if notes else ''}
**What I will do with it**

I will score it.

**If something looks wrong**

- **Something** → tell me.

<!-- operator-request: issue={issue} kind={kind} artifact=capture gate={gate} -->
"""
    return {"issue": issue, "title": title, "url": f"http://x/{issue}",
            "labels": [f"needs/{kind}", "needs/operator"], "kind": kind,
            "priority": "next", "status": [], "body": body, "request_url": "http://x"}


POWER_UP = """1. [You] Press the power button on the **battery pack** (the RavPower box the
   scanner is strapped to). Its battery lights should come on.
2. [You] Tell me "**powered up**" and wait — I will check whether it has appeared on the
   network.
3. [You] Open <http://localhost:8000/static/index.html> in your browser."""


# --------------------------------------------------------------------------- parsing


def test_bullets_folds_wrapped_continuations():
    """#183's third note prompt put its `______` on the continuation line.

    Reading bullets line-by-line dropped it entirely, so the page asked the owner for
    two of the three things they would have to re-run the whole job to recover.
    """
    lines = [
        "- Did it ever stop and wait for you to type something? ______",
        "- If it stopped early, the word after \"stopped on\" — one of `complete`,",
        "  `denied`, `cli_error`: ______",
        "- Roughly how long the whole thing took: ______",
    ]
    assert len(bullets(lines)) == 3
    assert len(parse_prompts(lines)) == 3
    assert "cli_error" in parse_prompts(lines)[1]


def test_bullet_marker_strip_preserves_leading_bold():
    """`lstrip("-* ")` ate the `**` opening a bold bullet.

    The orphaned closing `**` then bound to the *next* pair, so "**No green light?**
    Unplug the flat cable..." rendered with the emphasis on the wrong half of the
    sentence -- and this is the one sub-bullet that tells the owner how to recover a
    dead network link.
    """
    got = bullets(["- **No green light?** Unplug the flat cable, then click "
                   "**Bridge Mode** in the top bar."])
    assert got[0].startswith("**No green light?**")


def test_parse_steps_keeps_actor_and_sub_bullets():
    steps = parse_steps([
        "1. [You] Do the first thing.",
        "2. [Claude] I check something.",
        "3. [You] Do the third thing, which wraps",
        "   onto a second line.",
        "   - **A caveat** worth reading.",
    ])
    assert [s["actor"] for s in steps] == ["You", "Claude", "You"]
    assert steps[2]["text"].endswith("onto a second line.")
    assert steps[2]["notes"] == ["**A caveat** worth reading."]


def test_split_sections_ignores_bold_prose_that_is_not_a_heading():
    """#159 and #166 both have standalone bold sentences inside a section."""
    body = (f"{REQUEST} — T\n\n**Why this matters**\n\n"
            "**There is nothing extra for you to do here.**\n\nReally.\n\n"
            "**Steps**\n\n1. [You] Go.\n")
    sections = split_sections(body)
    assert sections["_title"] == ["T"]
    assert "There is nothing extra for you to do here." not in sections
    assert any("nothing extra" in ln for ln in sections["Why this matters"])


# ----------------------------------------------------------------------- tag scoping


def test_negated_venue_mention_is_not_a_requirement():
    """#60 says "a bookshelf or a doorway, **not a blank wall**".

    Matching the whole body sent a recording that has to sit still on a table into the
    blank-wall leg -- a wrong room, from a phrase that says the opposite.
    """
    assert _matches(r"blank wall", "stand at a blank wall")
    assert not _matches(r"blank wall", "point it at a bookshelf, not a blank wall")
    assert not _matches(r"blank wall", "a doorway rather than a **blank wall**"
                        .replace("rather than", "and never"))


def test_venue_tags_ignore_the_why_and_troubleshooting_prose():
    """#142's "something metal was near the scanner" is history, not a requirement.

    It happens to also genuinely need a metal-free spot, so this asserts the mechanism
    on a runbook that does not: the word appears only in the retrospective prose.
    """
    rb = parse_runbook(_runbook(
        900, "T",
        "- The scanner and its battery pack",
        "1. [You] Point it at the wall.",
    ))
    rb2 = parse_runbook({**_runbook(901, "T", "- The scanner", "1. [You] Go."),
                         "body": _runbook(901, "T", "- The scanner", "1. [You] Go.")["body"]
                         .replace("Because it does.",
                                  "Last time something metal was near the scanner.")})
    assert "metal-free" not in rb["tags"]
    assert "metal-free" not in rb2["tags"], "Why-section prose must not set a venue"


def test_runbook_with_no_power_up_step_still_needs_the_rig():
    """#142's recording is started by Claude, so it has no power-up step at all.

    Keying the sitting off the power-up step alone filed a hand-held magnetometer sweep
    under "no hardware" -- it would have been read as a desk job.
    """
    rb = parse_runbook(_runbook(
        902, "Redo the tilt sweep",
        "- The scanner, **hand-held**. **Not** on the tripod.\n"
        "- A spot at least **two arm-lengths from anything metal**.",
        "1. [Claude] I will start the recording for you.\n"
        "2. [You] Hold the scanner **level** for 15 seconds.",
    ))
    assert set(rb["tags"]) & RIG_TAGS
    assert _sitting_for(rb) == "rig-live"


def test_reflash_and_viewer_down_are_exclusive():
    rb = parse_runbook(_runbook(
        903, "Wedge experiment", "- The scanner, powered up as usual.",
        "1. [Claude] I will load the experimental software onto the scanner.",
        kind="hardware"))
    assert _sitting_for(rb) == "rig-exclusive"


# ------------------------------------------------------------------------ clustering


def test_long_and_short_power_up_phrasings_cluster():
    long_form = ("Press the power button on the **battery pack** (the RavPower box the "
                 "scanner is strapped to). Its battery lights should come on.")
    short_form = ("Press the power button on the **battery pack**. Its battery lights "
                  "should come on.")
    assert _jaccard(_tokens(long_form), _tokens(short_form)) >= SIM_THRESHOLD


def test_per_take_save_steps_never_cluster_as_shared_setup():
    """"type exactly `i141-ambientffpan`" and "type exactly `i144-tumble`" are textually
    near-identical, but each recording needs its own. Hoisting them into a sitting's
    shared block would tell the owner to name one file for two takes.
    """
    rbs = [
        parse_runbook(_runbook(
            141, "A", "- The scanner",
            POWER_UP + "\n4. [You] Click **● Record** once."
            "\n5. [You] In the naming box that pops up, type exactly `i141-pan` and "
            "click **Save**.")),
        parse_runbook(_runbook(
            144, "B", "- The scanner",
            POWER_UP + "\n4. [You] Click **● Record** once."
            "\n5. [You] In the naming box that pops up, type exactly `i144-tumble` and "
            "click **Save**.")),
    ]
    texts = [c["text"] for c in cluster_steps(rbs)]
    assert any("power button" in t for t in texts), "the shared preamble should cluster"
    assert not any("naming box" in t for t in texts)
    assert not any("Record" in t for t in texts)


def test_setup_end_stops_at_the_first_cycle_step():
    steps = parse_steps([
        "1. [You] Press the power button.",
        "2. [You] Click **● Record** once.",
        "3. [You] Press the power button.",
    ])
    assert _setup_end(steps) == 1


# --------------------------------------------------------------------------- planning


@pytest.fixture
def planned():
    records = [
        _runbook(60, "Two still recordings",
                 "- The scanner and its battery pack\n"
                 "- A **table or shelf** where the scanner can sit undisturbed\n"
                 "- About **15 minutes** in total",
                 POWER_UP + "\n4. [You] Click **● Record** once."
                 "\n5. [You] Write down the **Gaps** number right now: ______",
                 kind="network"),
        _runbook(141, "One slow sweep across a blank wall",
                 "- The scanner and its battery pack\n"
                 "- A **plain, matte, blank wall**\n"
                 "- About **8 minutes**",
                 POWER_UP + "\n4. [Claude] I set the Ambient ranging mode."
                 "\n5. [You] Click **● Record** once."),
        _runbook(142, "Redo the tilt sweep",
                 "- The scanner, **hand-held**. **Not** on the tripod.\n"
                 "- Two arm-lengths from anything metal.\n"
                 "- About **3 minutes**.",
                 "1. [Claude] I will start the recording for you.\n"
                 "2. [You] Hold it **level** for 15 seconds."),
        _runbook(144, "A 45-second tumble",
                 "- The scanner and its battery pack\n"
                 "- Enough clear space to turn around holding it at arm's length\n"
                 "- About **5 minutes**",
                 POWER_UP + "\n4. [You] Stay away from metal furniture."
                 "\n5. [You] Click **● Record** once."),
        _runbook(109, "Look at one thumbnail and tell me if it is good enough",
                 "- **Nothing physical.** The scanner does not need to be on.\n"
                 "- About **3 minutes**.",
                 "1. [You] Open the app in your browser.\n"
                 "2. [You] Click **Preview** and tell me what you see.", kind="eyes"),
        _runbook(16, "Covered by the request on #145 (same cable)",
                 "- Nothing beyond what #145 asks for.",
                 "1. [You] Follow the runbook on **#145**.", kind="hardware"),
        _runbook(145, "Plug the second USB cable in",
                 "- The scanner, powered up as usual.\n"
                 "- The **USB_USER** cable.\n- About **10 minutes**",
                 "1. [Claude] I will stop the live viewer.\n"
                 "2. [You] Plug it in.", kind="hardware"),
        {**_runbook(182, "No runbook here", "-", "1. [You] x"), "body": None},
    ]
    return plan(records)


def test_alias_becomes_a_free_rider_not_a_trip(planned):
    """#16's runbook says in prose that #145 covers it. Prose cannot be batched."""
    assert [r["issue"] for r in planned["riders"]] == [16]
    assert planned["riders"][0]["host"] == 145
    planned_issues = {m["issue"] for s in planned["sittings"] for m in s["members"]}
    assert 16 not in planned_issues, "a rider must not also be listed as its own trip"


def test_issues_needing_the_same_venue_share_one_leg(planned):
    """#142 and #144 share no step text at all -- Claude starts one of the recordings --
    so step clustering cannot see the saving. The venue can: both need the owner stood
    in the same metal-free spot with the scanner in their hands.
    """
    live = next(s for s in planned["sittings"] if s["key"] == "rig-live")
    metal = next(lg for lg in live["legs"] if lg["tag"] == "metal-free")
    assert sorted(m["issue"] for m in metal["members"]) == [142, 144]
    assert live["shared_venues"] == 1


def test_shared_block_is_scoped_to_its_own_sitting(planned):
    """Clusters are global. A desk-only runbook whose step 1 is "Open the app in your
    browser" once floated the browser step above the power-up in the rig block, and put
    itself in that block's "covers" badge -- an issue the owner is not doing there.
    """
    live = next(s for s in planned["sittings"] if s["key"] == "rig-live")
    members = {m["issue"] for m in live["members"]}
    for cluster in live["shared"]:
        assert set(cluster["issues"]) <= members
    orders = [c["order"] for c in live["shared"]]
    assert orders == sorted(orders)
    assert "power button" in live["shared"][0]["text"]


def test_a_rig_state_change_sorts_last_in_its_sitting(planned):
    """#141 has Claude switch the ranging mode and switch it back; nothing else in the
    sitting should be queued behind that."""
    live = next(s for s in planned["sittings"] if s["key"] == "rig-live")
    assert [m["issue"] for m in live["members"]][-1] == 141


def test_exclusive_work_is_its_own_sitting(planned):
    keys = [s["key"] for s in planned["sittings"]]
    assert keys == sorted(keys, key=lambda k: {"desk": 0, "rig-live": 1,
                                               "rig-exclusive": 2}[k])
    excl = next(s for s in planned["sittings"] if s["key"] == "rig-exclusive")
    assert [m["issue"] for m in excl["members"]] == [145]


def test_a_hold_with_no_runbook_is_surfaced_not_dropped(planned):
    """The label is a promise the instructions exist. An issue carrying it with nothing
    to act on is invisible from the issue list -- that is the whole reason to show it."""
    assert [m["issue"] for m in planned["missing"]] == [182]


def test_delta_steps_exclude_what_the_shared_block_already_covers(planned):
    live = next(s for s in planned["sittings"] if s["key"] == "rig-live")
    sixty = next(m for m in live["members"] if m["issue"] == 60)
    assert sixty["covered_indices"], "#60's preamble is in the shared block"
    delta = [s["text"] for s in sixty["delta_steps"]]
    assert not any("power button" in t for t in delta)
    assert any("Gaps" in t for t in delta)


def test_summary_counts_trips_honestly(planned):
    s = planned["summary"]
    assert s["held"] == 8
    assert s["with_runbook"] == 7
    assert s["free_riders"] == 1
    assert s["sittings"] == 3
    assert s["batched_minutes"] < s["separate_minutes"]


# -------------------------------------------------------------------------- rendering


def test_render_is_self_contained_and_escapes(planned):
    page = render(planned, repo="owner/repo")
    assert page.startswith("<!doctype html>")
    assert "<script src=" not in page and "<link rel=\"stylesheet\"" not in page
    assert page.count("<html") == 1 and page.rstrip().endswith("</html>")


def test_render_marks_every_you_step_tickable(planned):
    page = render(planned, repo="owner/repo")
    you_steps = sum(len([s for s in m["delta_steps"] if s["actor"] == "You"])
                    for sit in planned["sittings"] for m in sit["members"])
    assert page.count("type='checkbox'") >= you_steps


def test_render_gives_every_blank_field_somewhere_to_go(planned):
    """#60's runbook says plainly that without these numbers the takes have to be
    redone, and that they are not saved into the recording."""
    page = render(planned, repo="owner/repo")
    assert "textarea class='inline'" in page
