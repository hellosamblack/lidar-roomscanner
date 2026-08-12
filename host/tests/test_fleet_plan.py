"""Guards on `fleet_plan`'s issue scoring, footprint expansion and batch selection.

All offline: the fixtures under `fixtures/fleet/` are a captured slice of the live
tracker plus a real `git log`, so no `gh` call and no network. See that directory's
README for provenance and for what each fixture deliberately encodes.
"""
import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location("fleet_plan", _ROOT / "tools" / "fleet_plan.py")
fp = importlib.util.module_from_spec(_SPEC)
sys.modules["fleet_plan"] = fp
_SPEC.loader.exec_module(fp)

FIXTURES = Path(__file__).resolve().parent / "fixtures" / "fleet"


@pytest.fixture(scope="module")
def issues():
    return json.loads((FIXTURES / "issues_open.json").read_text())


@pytest.fixture(scope="module")
def tracked():
    return {ln for ln in (FIXTURES / "tracked_files.txt").read_text().splitlines() if ln}


@pytest.fixture(scope="module")
def commits():
    return fp.parse_git_log((FIXTURES / "git_log_name_only.txt").read_text())


@pytest.fixture(scope="module")
def footprints(issues, tracked, commits):
    """number -> (seed, confidence, expanded footprint), computed once."""
    coedit = fp.build_coedit_graph(commits)
    prior = fp.build_area_prior(commits, tracked)
    out = {}
    for issue in issues:
        seed, confidence = fp.seed_footprint(issue, tracked)
        expanded = fp.expand_footprint(seed, coedit, prior, fp.issue_areas(issue), tracked)
        out[issue["number"]] = (seed, confidence, expanded)
    return out


@pytest.fixture(scope="module")
def plan(issues, tracked, commits):
    return fp.plan_fleet(issues, tracked, commits, max_agents=3, generated_at="FIXED")


def _issue(issues, number):
    return next(i for i in issues if i["number"] == number)


# --------------------------------------------------------------------------------
# The truthy-empty-dict trap
# --------------------------------------------------------------------------------

def test_empty_blocked_by_relation_does_not_exclude(issues):
    """`{"nodes": [], "totalCount": 0}` is truthy, and every issue in this repo has one.

    A bare `if issue.get("blockedBy")` would mark all 64 open issues blocked and
    return an empty batch forever -- a total failure that looks like "nothing to do".
    """
    assert all("blockedBy" in i for i in issues), "fixture lost the blockedBy field"
    assert any(i["blockedBy"] == {"nodes": [], "totalCount": 0} for i in issues)
    assert all(i["blockedBy"] for i in issues), "the fixture's empty relations are truthy"
    assert not any(fp._has_open_native_blocker(i) for i in issues)


def test_open_native_blocker_is_honoured_when_one_exists():
    blocked = {"number": 1, "title": "t", "labels": [],
               "blockedBy": {"nodes": [{"number": 9, "state": "OPEN"}], "totalCount": 1}}
    closed = {"number": 2, "title": "t", "labels": [],
              "blockedBy": {"nodes": [{"number": 9, "state": "CLOSED"}], "totalCount": 1}}
    assert fp._has_open_native_blocker(blocked)
    assert not fp._has_open_native_blocker(closed)


# --------------------------------------------------------------------------------
# Footprint extraction and confidence
# --------------------------------------------------------------------------------

def test_extract_paths_keeps_only_real_files(tracked):
    text = "touches `host/src/roomscan/web.py` and imaginary/nope.py and scene.js"
    found = fp.extract_paths(text, tracked)
    assert "host/src/roomscan/web.py" in found
    assert "imaginary/nope.py" not in found
    assert "host/src/roomscan/static/scene.js" in found, "unambiguous basename should resolve"


def test_extract_paths_ignores_ambiguous_basenames(tracked):
    """`config.py` exists in more than one package, so a bare mention proves nothing."""
    multi = [p for p in tracked if p.endswith("/config.py")]
    assert len(multi) > 1, "fixture no longer has an ambiguous basename to test"
    assert not fp.extract_paths("something about config.py", tracked)


def test_declared_scope_outranks_a_planned_path(tracked):
    issue = {
        "number": 1, "title": "t", "body": "prose mentioning host/src/roomscan/decoder.py",
        "labels": [],
        "comments": [
            {"body": "## Implementation plan\nedit `host/src/roomscan/web.py`"},
            {"body": "**Session start** — now\n\nFiles in scope: `host/src/roomscan/protocol.py`\n\n"},
        ],
    }
    seed, confidence = fp.seed_footprint(issue, tracked)
    assert confidence == "declared"
    assert seed == {"host/src/roomscan/protocol.py"}


def test_planned_outranks_a_bare_mention(tracked):
    issue = {"number": 1, "title": "t", "body": "see host/src/roomscan/decoder.py", "labels": [],
             "comments": [{"body": "### Root cause\n`host/src/roomscan/web.py` is wrong"}]}
    seed, confidence = fp.seed_footprint(issue, tracked)
    assert confidence == "planned"
    assert seed == {"host/src/roomscan/web.py"}


def test_bare_mention_is_the_weakest_tier(tracked):
    issue = {"number": 1, "title": "t", "body": "see `host/src/roomscan/decoder.py`",
             "labels": [], "comments": []}
    assert fp.seed_footprint(issue, tracked)[1] == "mentioned"


def test_issue_with_no_paths_reports_none(tracked):
    issue = {"number": 1, "title": "t", "body": "the rig feels slow", "labels": [], "comments": []}
    assert fp.seed_footprint(issue, tracked) == (set(), "none")


def test_every_confidence_tier_is_present_in_the_corpus(footprints):
    """The fixture must keep exercising all four tiers, or these guards go hollow."""
    seen = {conf for _, conf, _ in footprints.values()}
    assert seen == set(fp.FOOTPRINT_CONFIDENCE), f"tiers missing from the fixture: {seen}"


# --------------------------------------------------------------------------------
# Expansion: the whole reason the planner is not a one-liner
# --------------------------------------------------------------------------------

def test_sibling_test_module_is_pulled_in(tracked):
    assert "host/tests/test_web.py" in fp.sibling_tests("host/src/roomscan/web.py", tracked)


def test_coedit_graph_finds_the_known_hot_pair(commits):
    graph = fp.build_coedit_graph(commits)
    assert graph["host/src/roomscan/web.py"]["host/tests/test_web.py"] >= 10


def test_tree_wide_sweeps_do_not_link_everything(commits):
    """A commit touching hundreds of files would make the graph complete, and a complete
    graph conflicts with everything and therefore schedules nothing."""
    graph = fp.build_coedit_graph(commits, max_files=25)
    widest = max(len(v) for v in graph.values())
    assert widest < len({f for c in commits for f in c["files"]}) * 0.5


def test_expansion_reaches_beyond_the_seed(footprints):
    seed, _, expanded = footprints[82]
    assert len(expanded) > len(seed)
    assert {origin for origin in expanded.values()} & {"coedit", "sibling"}


def test_seed_disjoint_host_web_issues_still_conflict(footprints, issues):
    """The regression test for the entire selection design.

    #107 and #121 are both `area/host-web` and their stated files are disjoint, so a
    seed-only intersection finds no conflict and schedules them side by side. They
    both reach `static/index.html` -- which **neither issue mentions anywhere**; it
    comes purely from the co-edit graph -- and that is a hot file, so they are a hard
    conflict. The median issue names exactly one file, so without this expansion
    conflict detection has almost no power.
    """
    seed_a, _, exp_a = footprints[107]
    seed_b, _, exp_b = footprints[121]
    shared_file = "host/src/roomscan/static/index.html"

    assert "area/host-web" in fp.issue_areas(_issue(issues, 107))
    assert "area/host-web" in fp.issue_areas(_issue(issues, 121))
    assert not (seed_a & seed_b), "fixture drifted: these issues now share a stated file"
    assert shared_file not in seed_a and shared_file not in seed_b, (
        "fixture drifted: the collision is no longer purely inferred")

    assert shared_file in set(exp_a) & set(exp_b)
    assert exp_a[shared_file] != "seed" and exp_b[shared_file] != "seed"
    assert fp.conflicts(exp_a, exp_b) == "hard"

    # And the counterfactual this whole mechanism exists to defeat.
    seed_only = ({p: "seed" for p in seed_a}, {p: "seed" for p in seed_b})
    assert fp.conflicts(*seed_only) == "none", (
        "a seed-only check would have co-scheduled these two")


def test_shared_docs_never_enter_a_footprint(footprints):
    """Every issue touches ROADMAP.md via status-sync, so it carries no scheduling
    information -- and workers are forbidden to edit it, so they cannot collide there."""
    for _, _, expanded in footprints.values():
        assert not [p for p in expanded if fp.is_shared_doc(p)]


def test_conflict_tiers_are_distinguished():
    stated = {"a.py": "seed"}
    inferred = {"a.py": "coedit"}
    assert fp.conflicts(stated, {"a.py": "sibling"}) == "hard"
    assert fp.conflicts(inferred, {"a.py": "coedit"}) == "soft"
    assert fp.conflicts(stated, {"b.py": "seed"}) == "none"


def test_hot_file_overlap_is_hard_even_when_only_inferred():
    hot = next(iter(fp.HOT_FILES))
    assert fp.conflicts({hot: "coedit"}, {hot: "area"}) == "hard"


# --------------------------------------------------------------------------------
# Scoring
# --------------------------------------------------------------------------------

def _stub(number=1, labels=(), comments=()):
    return {"number": number, "title": "t", "body": "",
            "labels": [{"name": n} for n in labels],
            "comments": [{"body": b} for b in comments]}


def test_priority_is_monotonic():
    now = fp.score_issue(_stub(labels=["priority/now"]), {})[0]
    nxt = fp.score_issue(_stub(labels=["priority/next"]), {})[0]
    later = fp.score_issue(_stub(labels=["priority/later"]), {})[0]
    assert now > nxt > later


def test_prior_work_doubles_the_score():
    plain = fp.score_issue(_stub(labels=["priority/next"]), {})[0]
    planned = fp.score_issue(
        _stub(labels=["priority/next"], comments=["## Implementation plan\ndo the thing"]), {})[0]
    assert planned == pytest.approx(plain * fp.PRIOR_WORK_MULTIPLIER)


def test_gating_other_issues_outranks_an_equal_peer():
    leaf = fp.score_issue(_stub(number=1, labels=["priority/next"]), {})[0]
    gate = fp.score_issue(_stub(number=2, labels=["priority/next"]), {2: {3, 4}})[0]
    assert gate > leaf


def test_score_explains_itself():
    _, why = fp.score_issue(
        _stub(labels=["priority/now", "bug"], comments=["## Implementation plan"]), {})
    joined = " ".join(why)
    assert "priority/now" in joined and "prior work" in joined and "bug" in joined


def test_dependency_edges_are_reported_not_silently_applied(issues, tracked, commits):
    """Prose inference is unreliable, so an inferred blocker must reach the operator."""
    graph, notes = fp.build_dep_graph([
        _stub(number=10), {**_stub(number=11), "body": "Blocked by #10 until that lands"}])
    assert graph[10] == {11}
    assert notes and "inferred from prose" in notes[0]


# --------------------------------------------------------------------------------
# Resources
# --------------------------------------------------------------------------------

def test_web_work_claims_the_browser_and_a_port():
    res = fp.resources_for(_stub(labels=["area/host-web"]), {})
    assert {"browser", "port"} <= res


def test_firmware_claims_the_device_and_slam_claims_the_gpu():
    assert "device" in fp.resources_for(_stub(labels=["area/firmware-scanner-stream"]), {})
    assert "gpu" in fp.resources_for(_stub(labels=["area/host-slam"]), {})


def test_only_one_browser_worker_per_wave():
    """No file footprint can express this: `ui_*` share one Playwright browser."""
    cands = [{"number": n, "title": "t", "score": 100 - n, "footprint": {f"f{n}.py": "seed"},
              "footprint_confidence": "planned", "resources": ["browser", "port"]}
             for n in (1, 2, 3)]
    batch, deferred, _ = fp.select_batch(cands, max_agents=3)
    assert len(batch) == 1
    assert all("browser" in d["reason"] for d in deferred)


def test_assigned_ports_are_unique_and_never_8000():
    cands = [{"number": n, "title": "t", "score": 100 - n, "footprint": {f"f{n}.py": "seed"},
              "footprint_confidence": "planned", "resources": ["port"]} for n in (1, 2, 3)]
    batch, _, _ = fp.select_batch(cands, max_agents=3, resource_caps={})
    ports = [i["port"] for i in batch]
    assert len(set(ports)) == len(ports)
    assert 8000 not in ports


# --------------------------------------------------------------------------------
# Selection
# --------------------------------------------------------------------------------

def test_batch_respects_max_agents(issues, tracked, commits):
    for n in (1, 2, 5):
        got = fp.plan_fleet(issues, tracked, commits, max_agents=n, generated_at="X")
        assert len(got["batch"]) <= n


def test_selected_issues_never_hard_conflict(plan):
    for a in plan["batch"]:
        for b in plan["batch"]:
            if a["number"] < b["number"]:
                assert fp.conflicts(a["footprint"], b["footprint"]) != "hard"


def test_soft_conflicts_are_selected_but_surfaced():
    cands = [
        {"number": 1, "title": "t", "score": 100, "footprint": {"a.py": "seed", "s.py": "coedit"},
         "footprint_confidence": "planned", "resources": []},
        {"number": 2, "title": "t", "score": 90, "footprint": {"b.py": "seed", "s.py": "coedit"},
         "footprint_confidence": "planned", "resources": []},
    ]
    batch, _, notes = fp.select_batch(cands, max_agents=2)
    assert len(batch) == 2, "a soft conflict must not block scheduling"
    assert batch[1]["soft_conflicts_with"] == [1]
    assert any("inferred" in n for n in notes), "a soft conflict must be reported"


def test_exactly_one_exploration_slot_and_it_goes_to_the_best(plan):
    unknown = [i for i in plan["batch"] if i["footprint_confidence"] == "none"]
    assert len(unknown) <= 1
    if unknown:
        deferred_unknown = [d for d in plan["deferred"] if "exploration slot" in d["reason"]]
        assert all(d["number"] != unknown[0]["number"] for d in deferred_unknown)
        assert any("exploration slot" in n for n in plan["notes"])


def test_unknown_footprints_are_not_starved(issues, tracked, commits):
    """Prior work is what produces file paths, so scoring that rewards prior work and a
    filter that defers unknown footprints are the same filter applied twice. Without a
    reserved slot the path-less tail never runs at all."""
    got = fp.plan_fleet(issues, tracked, commits, max_agents=3, generated_at="X")
    assert any(i["footprint_confidence"] == "none" for i in got["batch"])


def test_claimed_blocked_and_data_collection_are_excluded(plan, issues):
    excluded = {e["number"]: e["reason"] for e in plan["excluded"]}
    for issue in issues:
        labels = fp.label_names(issue)
        if "status/in-progress" in labels:
            assert "claimed" in excluded[issue["number"]]
        if "status/blocked" in labels:
            assert issue["number"] in excluded
        if "data-collection" in labels:
            assert issue["number"] in excluded


def test_no_selected_issue_carries_an_excluding_label(plan):
    for item in plan["batch"]:
        labels = set(item["labels"])
        assert not (labels & {"status/in-progress", "status/blocked", "data-collection"})


def test_selection_is_deterministic(issues, tracked, commits):
    a = fp.plan_fleet(issues, tracked, commits, max_agents=3, generated_at="X")
    b = fp.plan_fleet(issues, tracked, commits, max_agents=3, generated_at="X")
    assert [i["number"] for i in a["batch"]] == [i["number"] for i in b["batch"]]


def test_ties_break_by_issue_number():
    cands = [{"number": n, "title": "t", "score": 50, "footprint": {f"f{n}.py": "seed"},
              "footprint_confidence": "planned", "resources": []} for n in (9, 3, 7)]
    batch, _, _ = fp.select_batch(cands, max_agents=3)
    assert [i["number"] for i in batch] == [3, 7, 9]


def test_priority_filter_excludes_other_tiers(issues, tracked, commits):
    got = fp.plan_fleet(issues, tracked, commits, max_agents=10,
                        include_priorities=("now",), generated_at="X")
    assert all("priority/now" in i["labels"] for i in got["batch"])


# --------------------------------------------------------------------------------
# Worktree naming and the policy constants
# --------------------------------------------------------------------------------

def test_worktree_name_is_a_safe_slug(plan):
    for item in plan["batch"]:
        name = item["worktree_name"]
        assert re.fullmatch(r"issue-\d+-[a-z0-9-]+", name), name
        assert not name.endswith("-")
        assert len(f".claude/worktrees/{name}") <= 60


def test_worktree_path_stays_under_the_150_char_limit(issues):
    """`git worktree add` breaks past ~150 repo-relative chars (Windows 260 limit),
    and a worker still has to create files beneath this directory."""
    longest = max(issues, key=lambda i: len(i["title"]))
    path = f".claude/worktrees/{fp.worktree_name(longest)}/host/src/roomscan/static/index.html"
    assert len(path) <= 150, path


def test_area_globs_all_match_something_real(tracked):
    """A renamed module would otherwise silently turn an area prior into an empty set."""
    dead = [(area, glob) for area, globs in fp.AREA_GLOBS.items() for glob in globs
            if not any(p.startswith(glob) if glob.endswith("/") else p == glob
                       for p in tracked)]
    assert not dead, f"AREA_GLOBS entries matching no tracked file: {dead}"


def test_every_area_label_in_fixture_is_handled(issues):
    """Every `area/*` label on any issue in the fixture must either have an AREA_GLOBS
    entry or be explicitly excluded in AREA_GLOBS_EXCLUDED.

    This prevents a new area label from silently producing an empty footprint
    and co-scheduling everything.
    """
    # Collect all area labels in the fixture
    all_areas = set()
    for issue in issues:
        for label in issue.get("labels") or []:
            name = label.get("name", "")
            if name.startswith("area/"):
                all_areas.add(name)

    # Check each area is either in AREA_GLOBS or AREA_GLOBS_EXCLUDED
    unhandled = all_areas - set(fp.AREA_GLOBS.keys()) - fp.AREA_GLOBS_EXCLUDED
    assert not unhandled, (
        f"area/* labels in fixture are not handled: {sorted(unhandled)}. "
        f"Add them to AREA_GLOBS (with globs that match real files) or "
        f"AREA_GLOBS_EXCLUDED (if they have no code footprint).")


def test_hot_and_shared_constants_name_real_files(tracked):
    assert not [p for p in fp.HOT_FILES if p not in tracked]
    assert not [p for p in fp.SHARED_DOC_FILES
                if p not in tracked and not p.startswith(".")]


def test_firmware_work_is_not_handed_to_a_cheap_tier():
    """Firmware needs the rig, and every error path there spins forever rather than
    returning, so a worker cannot recover unattended."""
    tier = fp.suggest_tier(_stub(labels=["area/firmware-scanner-stream"]), {}, "planned")
    assert tier == "orchestrator-only"
