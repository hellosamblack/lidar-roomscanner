"""Guards on the rotation supervisor's decision logic — offline, no `claude` spawned.

The supervisor's whole job is deciding whether to spend another session, so its failure
mode is *spending*: a chain that keeps going when it should stop costs real quota with
nobody watching. Every stop condition therefore gets a test, and two of them encode
mechanics measured against the real CLI (2.1.228) rather than assumed:

* a denied tool leaves `is_error: false` and `subtype: success`, so `permission_denials`
  is the only signal that a link was gated (probe: `Write` to `/tmp/perm-probe.txt` under
  `--allowedTools Read` — denied, file never created, session "succeeded"),
* `session_chain` grows on every link whether or not the link achieved anything, so it
  cannot be part of the progress key.
"""
import importlib.util
import json
import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location("fleet_run", _ROOT / "tools" / "fleet_run.py")
fr = importlib.util.module_from_spec(_SPEC)
sys.modules["fleet_run"] = fr
_SPEC.loader.exec_module(fr)

LIMITS = fr.Limits(max_sessions=6, max_weighted=50_000_000)


def handoff_text(**over) -> str:
    state = {"run_id": "fleet-20260813-1600", "run_state": "rotated", "link": 2,
             "session_chain": "[aaa, bbb]", "waves_done": 2,
             "issues_landed": "[170, 171]", "issues_open": "[178]", "halt_reason": ""}
    state.update(over)
    body = "\n".join(f"{k}: {v}" for k, v in state.items())
    return f"# Handoff\n\nSome prose the model wrote.\n\n```run-state\n{body}\n```\n\nMore prose.\n"


def decide(**over):
    kw = {"link": 1, "state": fr.parse_run_state(handoff_text()), "denials": [],
          "exit_code": 0, "is_error": False, "prev_progress": (1, ("170",)),
          "spent_weighted": 0.0, "limits": LIMITS}
    kw.update(over)
    return fr.next_action(**kw)


# ----------------------------------------------------------------- the run-state block


def test_parse_run_state_reads_the_fenced_block():
    state = fr.parse_run_state(handoff_text())
    assert state["run_state"] == "rotated"
    assert state["waves_done"] == 2
    assert state["issues_landed"] == [170, 171]
    assert state["session_chain"] == ["aaa", "bbb"]
    assert state["halt_reason"] == ""


def test_parse_run_state_returns_none_when_absent():
    # A crashed link leaves prose and no block. `None` is the answer that stops the chain;
    # {} would be indistinguishable from a block that parsed to nothing.
    assert fr.parse_run_state("# Handoff\n\nI ran out of turns mid-wave.\n") is None


def test_parse_run_state_tolerates_model_written_slop():
    # This file is authored by a language model under a rotation deadline. A parser that
    # raised on a trailing comma or a stray blank would halt a healthy run.
    text = ("```run-state\n"
            "run_state: rotated\n"
            "\n"
            "waves_done: 3   # after the review round-trip\n"
            "issues_landed: [170, 171, ]\n"
            "session_chain: []\n"
            "nonsense line without a colon\n"
            "halt_reason: needs the rig for #178\n"
            "```\n")
    state = fr.parse_run_state(text)
    assert state["waves_done"] == 3
    assert state["issues_landed"] == [170, 171]
    assert state["session_chain"] == []
    assert "nonsense line without a colon" not in state
    # `#178` is a value, not a comment: every string in this block is about issues, so a
    # naive split on "#" drops the only fact in the halt reason.
    assert state["halt_reason"] == "needs the rig for #178"


def test_progress_key_ignores_session_chain():
    # Every link appends itself, so a chain that achieves nothing still grows this list.
    a = fr.parse_run_state(handoff_text(session_chain="[aaa]"))
    b = fr.parse_run_state(handoff_text(session_chain="[aaa, bbb, ccc]"))
    assert fr.progress_key(a) == fr.progress_key(b)


def test_progress_key_moves_on_waves_or_landed_issues():
    base = fr.parse_run_state(handoff_text())
    assert fr.progress_key(base) != fr.progress_key(fr.parse_run_state(handoff_text(waves_done=3)))
    assert fr.progress_key(base) != fr.progress_key(
        fr.parse_run_state(handoff_text(issues_landed="[170, 171, 178]")))
    assert fr.progress_key(None) == ()


# ---------------------------------------------------------------------- stop conditions


def test_spawns_when_a_link_rotated_with_progress():
    d = decide()
    assert d.action == "spawn" and d.code == "spawn"


def test_stops_on_complete():
    d = decide(state=fr.parse_run_state(handoff_text(run_state="complete")))
    assert (d.action, d.code) == ("stop", "complete")


def test_stops_on_halted_and_quotes_the_reason():
    state = fr.parse_run_state(handoff_text(run_state="halted",
                                            halt_reason="rig needed for #178"))
    d = decide(state=state)
    assert (d.action, d.code) == ("stop", "halted")
    assert "rig needed for #178" in d.reason


def test_stops_on_permission_denial_and_names_the_tool():
    # Shape copied from a real `result` event: the denial is recorded while `is_error`
    # stays False, which is why this case cannot be folded into the exit-status check.
    d = decide(denials=[{"tool_name": "Write", "tool_use_id": "toolu_1",
                         "tool_input": {"file_path": "/tmp/x"}}],
               is_error=False, exit_code=0)
    assert (d.action, d.code) == ("stop", "denied")
    assert "Write" in d.reason


def test_denial_outranks_a_completed_run_but_keeps_it():
    d = decide(state=fr.parse_run_state(handoff_text(run_state="complete")),
               denials=[{"tool_name": "Bash"}])
    assert d.code == "denied"
    assert "complete" in d.also  # nothing is discarded, so the log can say both


def test_stops_when_the_link_left_no_state_block():
    d = decide(state=None)
    assert (d.action, d.code) == ("stop", "no_state")


def test_stops_on_no_progress():
    state = fr.parse_run_state(handoff_text(waves_done=2, issues_landed="[170, 171]"))
    d = decide(state=state, prev_progress=fr.progress_key(state))
    assert (d.action, d.code) == ("stop", "no_progress")


def test_stops_at_max_sessions():
    d = decide(link=6, limits=fr.Limits(max_sessions=6, max_weighted=None))
    assert (d.action, d.code) == ("stop", "max_sessions")


def test_stops_at_the_weighted_ceiling():
    d = decide(spent_weighted=50_000_001)
    assert (d.action, d.code) == ("stop", "budget")


def test_no_ceiling_means_no_budget_stop():
    d = decide(spent_weighted=9e12, limits=fr.Limits(max_sessions=6, max_weighted=None))
    assert d.action == "spawn"


@pytest.mark.parametrize("kw", [{"exit_code": 1}, {"is_error": True}])
def test_stops_on_a_failed_cli_process(kw):
    assert decide(**kw).code == "cli_error"


def test_stops_on_an_unrecognised_run_state():
    d = decide(state=fr.parse_run_state(handoff_text(run_state="finished")))
    assert (d.action, d.code) == ("stop", "bad_state")


# ------------------------------------------------------------ default allowlist coverage


def test_default_allowlist_covers_python_from_repo_root_and_from_host():
    # fleet-20260814-1204: a worker's cwd is its own worktree, and once it `cd`s into
    # `host/` its relative invocation is `.venv/bin/python`, not `host/.venv/bin/python` --
    # a rule anchored on the latter alone silently excludes every worker running from there.
    assert "Bash(host/.venv/bin/python:*)" in fr.DEFAULT_ALLOWED_TOOLS
    assert "Bash(.venv/bin/python:*)" in fr.DEFAULT_ALLOWED_TOOLS


# ------------------------------------------------------------------- $HOME correction
#
# fleet-20260814-0157 found gh unauthenticated because the supervisor was launched as
# root ($HOME=/root) while gh was only ever authenticated for sam. Patching that with an
# exported GH_CONFIG_DIR uncovered a second, same-cause split at fleet-20260814-0322: the
# link's auto-memory landed under /root/.claude/... , unreadable by sam. Both are $HOME
# splits, so resolve_run_env fixes $HOME itself rather than chasing each downstream tool.


def test_resolve_run_env_corrects_home_to_the_repo_owner(monkeypatch):
    monkeypatch.setattr(fr.pwd, "getpwuid", lambda uid: type("pw", (), {"pw_dir": "/home/sam"}))
    env, warning = fr.resolve_run_env(fr.REPO, {"HOME": "/root", "PATH": "/usr/bin"})
    assert warning is None
    assert env["HOME"] == "/home/sam"
    assert env["PATH"] == "/usr/bin"  # untouched


def test_resolve_run_env_drops_stale_overrides_only_when_home_was_wrong():
    # XDG_CONFIG_HOME/GH_CONFIG_DIR were the manual workaround before this existed --
    # they must not linger and mask the fix, or shadow it, on a future debugging session.
    import pwd as real_pwd
    owner_home = real_pwd.getpwuid(fr.REPO.stat().st_uid).pw_dir
    env, warning = fr.resolve_run_env(fr.REPO, {
        "HOME": "/root", "XDG_CONFIG_HOME": "/root/.config", "GH_CONFIG_DIR": "/home/sam/.config/gh",
    })
    assert warning is None
    assert env["HOME"] == owner_home
    assert "XDG_CONFIG_HOME" not in env
    assert "GH_CONFIG_DIR" not in env


def test_resolve_run_env_leaves_env_alone_when_home_already_matches():
    import pwd as real_pwd
    owner_home = real_pwd.getpwuid(fr.REPO.stat().st_uid).pw_dir
    env, warning = fr.resolve_run_env(fr.REPO, {"HOME": owner_home, "XDG_CONFIG_HOME": "/kept"})
    assert warning is None
    assert env["XDG_CONFIG_HOME"] == "/kept"  # not stripped -- HOME was never wrong


def test_resolve_run_env_warns_rather_than_crashes_when_owner_lookup_fails(monkeypatch):
    def _raise(uid):
        raise KeyError(uid)
    monkeypatch.setattr(fr.pwd, "getpwuid", _raise)
    env, warning = fr.resolve_run_env(fr.REPO, {"HOME": "/root"})
    assert warning is not None
    assert env["HOME"] == "/root"  # unchanged -- the chain still runs, just as before


# ------------------------------------------------------------------- argv and prompting


def test_build_argv_pins_the_session_and_scopes_tools():
    argv = fr.build_argv("hi", session_id="sid-1", model=None,
                         allowed_tools=("Read", "Bash(git:*)"))
    assert argv[:3] == ["claude", "-p", "hi"]
    assert argv[argv.index("--session-id") + 1] == "sid-1"
    assert argv[argv.index("--allowedTools") + 1] == "Read,Bash(git:*)"
    assert argv[argv.index("--output-format") + 1] == "stream-json"
    assert "--model" not in argv  # inherit rather than pin a model nobody chose


def test_build_argv_passes_a_model_when_given():
    argv = fr.build_argv("hi", session_id="sid-1", model="claude-opus-5", allowed_tools=("Read",))
    assert argv[argv.index("--model") + 1] == "claude-opus-5"


def test_prompt_pins_session_id_against_the_turn_one_trap():
    # Without this the successor's first fleet_budget() call reads its PREDECESSOR's
    # records -- the newest on disk at that moment -- and rotates a session that has done
    # nothing.
    p = fr.successor_prompt(run_id="fleet-1", link=3, session_id="sid-3",
                            handoff=".fleet/fleet-1.md", seed=False)
    assert 'session_id="sid-3"' in p
    assert "link 2" in p  # it is told whose handoff it is reading
    assert ".fleet/fleet-1.md" in p


def test_prompt_states_every_run_state_and_the_two_silent_stops():
    p = fr.successor_prompt(run_id="fleet-1", link=1, session_id="sid",
                            handoff="h.md", seed=True)
    for token in ("run_state: rotated", "run_state: complete", "run_state: halted"):
        assert token in p
    flat = " ".join(p.lower().split())  # the prompt is hard-wrapped; assert on meaning
    assert "do not ask the owner to restart you" in flat
    # Both silent stops have to be in the prompt, or a link can trip one without knowing
    # it exists: an absent state block, and a link that advances nothing.
    assert "no-progress" in flat
    assert f"```{fr.RUN_STATE_FENCE}``` block, it stops" in flat


def test_seed_prompt_starts_a_run_rather_than_resuming_one():
    seeded = fr.successor_prompt(run_id="r", link=1, session_id="s", handoff="h.md", seed=True)
    resumed = fr.successor_prompt(run_id="r", link=2, session_id="s", handoff="h.md", seed=False)
    assert "owner's brief" in seeded and "handoff written by" not in seeded
    assert "handoff written by" in resumed


def test_task_override_replaces_the_skill_line_and_nothing_else():
    # The override exists so the chain can be smoke-tested without a link claiming issues.
    # It must not weaken the contract the rest of the prompt carries.
    p = fr.successor_prompt(run_id="r", link=2, session_id="s", handoff="h.md", seed=False,
                            task="SMOKE: rewrite the block and stop.")
    assert "issue-fleet" not in p
    assert "SMOKE: rewrite the block and stop." in p
    assert "run_state: rotated" in p and 'session_id="s"' in p


def test_render_event_summarises_tools_and_stays_quiet_otherwise():
    tool = {"type": "assistant", "message": {"content": [
        {"type": "tool_use", "name": "Bash", "input": {"command": "git status"}}]}}
    assert "Bash(git status)" in fr.render_event(tool)
    text = {"type": "assistant", "message": {"content": [
        {"type": "text", "text": "Wave 2 merged.\nsecond line"}]}}
    assert "Wave 2 merged." in fr.render_event(text)
    assert fr.render_event({"type": "system", "subtype": "thinking_tokens"}) is None
    assert fr.render_event({"type": "user", "message": {"content": []}}) is None


# ------------------------------------------------------------------------- the chain


def _fake_link(monkeypatch, script):
    """Drive run_chain with scripted links; each entry writes a handoff and returns a result."""
    calls = []

    def fake_run_link(prompt, *, session_id, link, model, allowed_tools, log_path,
                      timeout_s, echo=True):
        calls.append({"link": link, "session_id": session_id, "prompt": prompt})
        step = script[link - 1]
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text("", encoding="utf-8")
        step["handoff"].write_text(step["text"], encoding="utf-8")
        return fr.LinkResult(link=link, session_id=session_id,
                             denials=step.get("denials", []),
                             weighted_delta=step.get("weighted", 1_000_000.0))

    monkeypatch.setattr(fr, "run_link", fake_run_link)
    return calls


def test_chain_spawns_successors_until_a_link_reports_complete(tmp_path, monkeypatch):
    h = tmp_path / "fleet-x.md"
    h.write_text(handoff_text(waves_done=0, issues_landed="[]", link=0), encoding="utf-8")
    script = [
        {"handoff": h, "text": handoff_text(waves_done=1, issues_landed="[170]")},
        {"handoff": h, "text": handoff_text(waves_done=2, issues_landed="[170, 171]")},
        {"handoff": h, "text": handoff_text(run_state="complete", waves_done=3,
                                            issues_landed="[170, 171, 178]")},
    ]
    calls = _fake_link(monkeypatch, script)
    rec = fr.run_chain(run_id="fleet-x", handoff=h, limits=LIMITS, model=None,
                       allowed_tools=("Read",), timeout_s=10, echo=False)
    assert rec["sessions"] == 3 and rec["stopped_because"] == "complete"
    assert rec["weighted_total"] == pytest.approx(3_000_000.0)
    assert len({c["session_id"] for c in calls}) == 3  # a fresh session id per link
    assert "link 2" in calls[2]["prompt"]              # each link told whose handoff it reads


def test_chain_stops_at_the_second_link_when_nothing_advanced(tmp_path, monkeypatch):
    h = tmp_path / "fleet-y.md"
    h.write_text(handoff_text(waves_done=1, issues_landed="[170]"), encoding="utf-8")
    same = handoff_text(waves_done=1, issues_landed="[170]")
    _fake_link(monkeypatch, [{"handoff": h, "text": same}, {"handoff": h, "text": same}])
    rec = fr.run_chain(run_id="fleet-y", handoff=h, limits=LIMITS, model=None,
                       allowed_tools=("Read",), timeout_s=10, echo=False)
    assert rec["sessions"] == 1 and rec["stopped_because"] == "no_progress"


def test_chain_stops_on_the_weighted_ceiling_mid_run(tmp_path, monkeypatch):
    h = tmp_path / "fleet-z.md"
    h.write_text(handoff_text(waves_done=0, issues_landed="[]"), encoding="utf-8")
    _fake_link(monkeypatch, [
        {"handoff": h, "text": handoff_text(waves_done=1, issues_landed="[170]"),
         "weighted": 9_000_000.0},
        {"handoff": h, "text": handoff_text(waves_done=2, issues_landed="[171]")},
    ])
    rec = fr.run_chain(run_id="fleet-z", handoff=h, limits=fr.Limits(6, 8_000_000.0),
                       model=None, allowed_tools=("Read",), timeout_s=10, echo=False)
    assert rec["sessions"] == 1 and rec["stopped_because"] == "budget"


def test_chain_stops_on_a_denial_without_spawning_a_retry(tmp_path, monkeypatch):
    h = tmp_path / "fleet-d.md"
    h.write_text(handoff_text(waves_done=0, issues_landed="[]"), encoding="utf-8")
    _fake_link(monkeypatch, [
        {"handoff": h, "text": handoff_text(waves_done=1, issues_landed="[170]"),
         "denials": [{"tool_name": "Bash"}]},
    ])
    rec = fr.run_chain(run_id="fleet-d", handoff=h, limits=LIMITS, model=None,
                       allowed_tools=("Read",), timeout_s=10, echo=False)
    assert rec["sessions"] == 1 and rec["stopped_because"] == "denied"


def test_orch_edits_counts_only_this_link(tmp_path):
    # The hook's log is shared by every session on the box, so the count is filtered by
    # session id. Differencing it across a link would bill an unrelated session's edits to
    # whichever link happened to be running at the time.
    (tmp_path / fr.ORCH_EDIT_LOG).write_text(
        "2026-08-13T10:00:00Z\tsess-a\tEdit\thost/src/roomscan/web.py\n"
        "2026-08-13T10:01:00Z\tsess-b\tWrite\thost/tools/x.py\n"
        "2026-08-13T10:02:00Z\tsess-a\tEdit\tfirmware/y.c\n", encoding="utf-8")
    assert fr.orch_edits("sess-a", tmp_path) == 2
    assert fr.orch_edits("sess-b", tmp_path) == 1
    assert fr.orch_edits("sess-c", tmp_path) == 0


def test_orch_edits_is_zero_when_the_hook_never_ran(tmp_path):
    # An uninstalled hook and a perfectly-behaved orchestrator are indistinguishable here.
    # test_orch_edit_hook.py checks the wiring; this one only pins the safe default.
    assert fr.orch_edits("sess-a", tmp_path) == 0


def test_chain_record_is_json_serialisable(tmp_path, monkeypatch):
    h = tmp_path / "fleet-j.md"
    h.write_text(handoff_text(waves_done=0, issues_landed="[]"), encoding="utf-8")
    _fake_link(monkeypatch, [{"handoff": h, "text": handoff_text(run_state="complete")}])
    rec = fr.run_chain(run_id="fleet-j", handoff=h, limits=LIMITS, model=None,
                       allowed_tools=("Read",), timeout_s=10, echo=False)
    round_tripped = json.loads(json.dumps(rec))
    assert round_tripped["links"][0]["decision"]["code"] == "complete"


# ------------------------------------------------------------------------------- CLI


def test_cli_refuses_to_run_without_a_declared_ceiling(tmp_path, monkeypatch):
    # There is no quota API on this box, so any default baked in here would be an invented
    # denominator -- the exact failure fleet_budget.py's calibration note is about. Make
    # the owner name the number, or explicitly opt out of one.
    monkeypatch.setattr(fr, "FLEET_DIR", tmp_path)
    with pytest.raises(SystemExit) as exc:
        fr.main(["--dry-run", "--run-id", "fleet-cli"])
    assert exc.value.code == 2


def test_cli_dry_run_seeds_a_parseable_handoff_and_spawns_nothing(tmp_path, monkeypatch):
    monkeypatch.setattr(fr, "FLEET_DIR", tmp_path)
    monkeypatch.setattr(fr, "run_chain", lambda **kw: pytest.fail("dry run spawned a link"))
    assert fr.main(["--dry-run", "--json", "--run-id", "fleet-cli",
                    "--max-weighted", "1e7", "--brief", "burn down area/host-tools"]) == 0
    seeded = (tmp_path / "fleet-cli.md").read_text(encoding="utf-8")
    state = fr.parse_run_state(seeded)
    assert state["run_state"] == "rotated" and state["link"] == 0
    assert fr.progress_key(state) == (0, ())   # the baseline the first link must beat
    assert "burn down area/host-tools" in seeded


def test_cli_dry_run_reuses_an_existing_handoff_rather_than_reseeding(tmp_path, monkeypatch):
    monkeypatch.setattr(fr, "FLEET_DIR", tmp_path)
    (tmp_path / "fleet-cli.md").write_text(handoff_text(waves_done=4), encoding="utf-8")
    assert fr.main(["--dry-run", "--json", "--run-id", "fleet-cli", "--no-weighted-ceiling"]) == 0
    state = fr.parse_run_state((tmp_path / "fleet-cli.md").read_text(encoding="utf-8"))
    assert state["waves_done"] == 4  # a relaunch after a halt must not clobber the run
