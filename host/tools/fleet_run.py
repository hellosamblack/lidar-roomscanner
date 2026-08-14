#!/usr/bin/env python3
"""Chain orchestrator sessions across one `issue-fleet` run, so rotation needs no owner.

#182 made rotation *decidable*: `fleet_budget()` reads the orchestrator's own raw context
size and returns `rotate` at 300K. What it could not do is make rotation *free* -- the
skill's Step 1.5 ends with "write the handoff and stop", and someone then has to notice
the run stopped and start the next session by hand. A policy whose next step is a human
noticing is a policy that gets skipped at 2am, which is exactly when a 600K-token session
happens.

This is the missing half: a supervisor that owns the chain. It spawns each orchestrator as
a fresh `claude -p` process seeded with the run's handoff file, waits, reads the handoff's
`run-state` block back, and spawns the successor. The owner launches it once.

It deliberately lives OUTSIDE the session it manages:

* **Not an MCP tool.** An MCP tool runs inside the session being rotated and would die with
  it. This must outlive every link, so it is a CLI the owner runs from a terminal, and it
  is listed in `EXCLUDED` in `host/tests/test_mcp_registry.py` for that reason.
* **Not a `Stop` hook.** A hook could detach a successor as an interactive session ends,
  but then a session the owner walked away from keeps spending. Owner's call, 2026-08-13:
  only supervisor-started runs rotate automatically.

Four mechanics were measured against the real CLI (2.1.228) before this was written, and
three of them are not what the flag names suggest:

1. **`--allowedTools` is ADDITIVE, not restrictive.** `--allowedTools Read` does not stop
   Bash: a probe that asked for `git status --porcelain` under exactly that flag ran it,
   with an empty `permission_denials`, because read-only git is auto-approved upstream of
   the allowlist. The scope here therefore bounds what is *granted*; it is not a sandbox.
2. **A denied tool leaves `is_error: false`.** The Write probe (`/tmp/perm-probe.txt`,
   `--allowedTools Read`) was denied -- the file was never created -- and the session still
   exited `subtype: success`, `is_error: false`, `stop_reason: end_turn`, with the denial in
   `permission_denials[]` and an apology in `result`. **Gate on `permission_denials`, never
   on the exit status**; a supervisor that trusts the exit status chains straight past a
   session that silently did none of its work.
3. **`--session-id` is accepted with `-p`**, so the supervisor knows each link's session id
   before the link starts, and can tell the link to pin it. That closes the successor's
   turn-1 trap: at its first `fleet_budget()` call the newest top-level records on disk
   still belong to its *predecessor*, so an unpinned call reads a 400K context and rotates
   a session that has done nothing.
4. **The spawned link inherits `$HOME` from whoever's shell launched the supervisor, and
   both `claude` and `gh` key their config off it.** The first real run (fleet-20260814-0157)
   was launched as root and came up with `gh` unauthenticated; patching that with an
   exported `GH_CONFIG_DIR` only fixed `gh` and left a second split running --
   fleet-20260814-0322's auto-memory landed under `/root/.claude/...`, unreadable by the
   account that actually owns this checkout. `resolve_run_env()` now sets `$HOME` to the
   repo owner's home for every spawned link, which fixes both at once and needs no env var
   from whoever runs this.

The chain is bounded three ways, all required by the owner: a cumulative weighted-token
ceiling, `--max-sessions` (default 6), and a no-progress detector. The last one matters
most: without it, a link that fails the same way every time is a loop that spends until the
weekly limit stops it.
"""
from __future__ import annotations

import argparse
import json
import os
import pwd
import re
import subprocess
import sys
import threading
import uuid
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # `tools` package root
from tools import fleet_budget  # noqa: E402

REPO = Path(__file__).resolve().parents[2]

#: NOT `.claude/fleet/`, where #182 first put the handoff. Measured 2026-08-13 against CLI
#: 2.1.228: an unattended session cannot `Write` anywhere under `.claude/` even when the
#: allowlist grants `Write`, and neither `Write(.claude/fleet/**)` nor the absolute
#: `Write(//.../.claude/fleet/**)` form lifts it -- all three were denied, the file never
#: created. It is the path, not the tool: the same session wrote `host/probe-b.md` and
#: `.fleet/probe.md` with no denial, so a hidden directory is fine and `.claude` is not.
#: (`.claude/worktrees/**` IS writable, which is why the existing worker flow survives --
#: do not read that exception as `.claude/` being generally open.)
#:
#: The whole chain hangs off this: a handoff the successor cannot write is a chain that
#: halts on link 1, which is exactly how the first live smoke test failed.
FLEET_DIR = REPO / ".fleet"

#: The handoff is prose for a language model AND state for this parser. It stays prose --
#: the successor is a model, and a machine-only handoff would lose the reasoning that makes
#: the next session's first decision cheap -- with one fenced block the supervisor reads.
RUN_STATE_FENCE = "run-state"
_FENCE_RE = re.compile(rf"^```{RUN_STATE_FENCE}\s*$(.*?)^```\s*$", re.M | re.S)

#: A trailing comment is ` # note` — but NOT `#178`. Every value in this block that is
#: worth reading is about GitHub Issues, so a naive `split("#")` truncates
#: `halt_reason: rig needed for #178` to "rig needed for" and drops the only fact in it.
#: Caught by a test, not by review.
_COMMENT_RE = re.compile(r"\s#(?!\d)")

#: Written by the link into its handoff. `rotated` is the only one that continues a chain.
TERMINAL_STATES = ("complete", "halted")
RUN_STATES = ("rotated", *TERMINAL_STATES)

DEFAULT_MAX_SESSIONS = 6
DEFAULT_LINK_TIMEOUT_S = 5400  # 90 min; a wave plus a review round-trip, measured ~45 min

#: What an orchestrator legitimately needs. Anything outside it is denied and halts the
#: chain -- which is the point: a missing permission costs one stopped run, never an
#: ungated edit. Two notes before you narrow it:
#:
#: * `Bash(rm:*)` is the widest rule here. The skill removes the two worktree symlinks
#:   before `git worktree remove`, so dropping `rm` halts every run at teardown. Narrow it
#:   to your taste; do not remove it and expect the chain to finish.
#: * `Edit`/`Write` are unscoped. The "you coordinate, you do not author" rule is policy
#:   measured by the ledger's `orch_code_edits`, not something this allowlist enforces --
#:   the orchestrator's legitimate writes (comment bodies, the handoff, doc deltas, the
#:   ledger row) and its illegitimate ones use the same tool.
#: * `Bash(host/.venv/bin/python:*)` alone denied a real worker (#81, fleet-20260814-1204):
#:   a worker's cwd is its own worktree, and once it `cd`s into `host/` its relative
#:   invocation is `.venv/bin/python`, which doesn't match a rule anchored on `host/`.
#:   `Bash(.venv/bin/python:*)` covers that case too. Note this does not exhaust what can
#:   halt a link -- some denials that day (bare `python3`, and a lone `rm` even with
#:   `Bash(rm:*)` already granted) look like the auto-mode classifier acting independently
#:   of this allowlist (a known, intermittent behavior -- see the `gh` mutation case in
#:   memory `gh-mutating-calls-blocked-in-bash`), which no entry here can pre-empt.
DEFAULT_ALLOWED_TOOLS = (
    "Read", "Glob", "Grep", "Skill", "Task", "Agent", "TodoWrite",
    "Edit", "Write",
    "Bash(git:*)", "Bash(gh:*)", "Bash(ls:*)", "Bash(cat:*)", "Bash(mkdir:*)",
    "Bash(ln:*)", "Bash(rm:*)", "Bash(host/.venv/bin/python:*)", "Bash(.venv/bin/python:*)",
    "mcp__roomscan__fleet_plan", "mcp__roomscan__fleet_budget",
    "mcp__roomscan__run_tests", "mcp__roomscan__operator_queue", "mcp__roomscan__doctor",
)


# --------------------------------------------------------------------------- run state


def parse_run_state(text: str) -> dict | None:
    """Pull the ```run-state``` block out of a handoff file. `None` if it is absent.

    Absent is a real answer, not an error: a session that crashed, ran out of turns, or
    ended on a permission denial leaves prose and no block. The chain stops there rather
    than guessing -- see `next_action`'s `no_state`.

    Deliberately hand-parsed rather than YAML: the value space is scalars and flat lists,
    and a parser that raises on a trailing comma would turn a cosmetic slip in a file
    written by a *language model* into a halted run.
    """
    m = _FENCE_RE.search(text)
    if not m:
        return None
    state: dict = {}
    for line in m.group(1).splitlines():
        line = _COMMENT_RE.split(line, maxsplit=1)[0].strip()
        if not line or ":" not in line:
            continue
        key, _, raw = line.partition(":")
        state[key.strip()] = _coerce(raw.strip())
    return state


def _coerce(raw: str):
    if raw.startswith("[") and raw.endswith("]"):
        inner = raw[1:-1].strip()
        if not inner:
            return []
        return [_coerce(part.strip()) for part in inner.split(",") if part.strip()]
    if re.fullmatch(r"-?\d+", raw):
        return int(raw)
    return raw.strip("'\"")


def progress_key(state: dict | None) -> tuple:
    """What must advance for a successor to be worth spawning.

    `session_chain` is excluded on purpose: every link appends itself to it, so a chain
    that achieves nothing still grows it. Waves dispatched and issues landed are the two
    quantities a link cannot advance without doing the run's actual work.
    """
    if not state:
        return ()
    landed = state.get("issues_landed") or []
    if not isinstance(landed, list):
        landed = [landed]
    return (int(state.get("waves_done") or 0), tuple(sorted(str(i) for i in landed)))


@dataclass
class Decision:
    action: str          # "spawn" | "stop"
    code: str            # stable slug, logged and asserted in tests
    reason: str
    also: tuple = ()     # every other condition that fired, worst-first


@dataclass
class Limits:
    max_sessions: int = DEFAULT_MAX_SESSIONS
    max_weighted: float | None = None


def next_action(*, link: int, state: dict | None, denials: list, exit_code: int,
                is_error: bool, prev_progress: tuple, spent_weighted: float,
                limits: Limits) -> Decision:
    """Whether to spawn link `link + 1`, and why not. Pure -- no subprocess, no clock.

    Order is worst-first, and every condition that fired is kept in `also` so the log can
    say "completed, but also hit the token ceiling" instead of picking one and discarding
    the rest. `denied` outranks `complete` because a denial means the link worked around
    something, or failed to, and either way the owner needs to see the tool name.
    """
    codes: list[tuple[str, str]] = []

    if denials:
        names = sorted({d.get("tool_name", "?") for d in denials if isinstance(d, dict)})
        codes.append(("denied", f"permission denied for {', '.join(names) or '?'} — the "
                                f"link ran unattended without a tool it asked for. Grant it "
                                f"with --allow-add or narrow the task; the chain does not "
                                f"retry, because a denied session usually did partial work"))
    if exit_code != 0 or is_error:
        codes.append(("cli_error", f"the claude process exited {exit_code} "
                                   f"(is_error={is_error})"))

    run_state = (state or {}).get("run_state")
    if state is None:
        codes.append(("no_state", "the handoff has no ```run-state``` block, so a finished "
                                  "run cannot be told from a crashed one"))
    elif run_state == "complete":
        codes.append(("complete", "the link reported the run complete"))
    elif run_state == "halted":
        codes.append(("halted", "the link halted: "
                                f"{state.get('halt_reason') or 'no reason recorded'}"))
    elif run_state != "rotated":
        codes.append(("bad_state", f"run_state {run_state!r} is not one of {RUN_STATES}"))
    elif progress_key(state) == prev_progress:
        codes.append(("no_progress", f"nothing advanced: waves_done and issues_landed are "
                                     f"unchanged at {prev_progress}. A successor would "
                                     f"repeat the same link"))

    if link >= limits.max_sessions:
        codes.append(("max_sessions", f"{link} sessions is the --max-sessions ceiling"))
    if limits.max_weighted is not None and spent_weighted >= limits.max_weighted:
        codes.append(("budget", f"the run has spent {spent_weighted:,.0f} weighted tokens, "
                                f"at or over the --max-weighted ceiling of "
                                f"{limits.max_weighted:,.0f}"))

    if not codes:
        return Decision("spawn", "spawn", f"link {link} rotated with progress {progress_key(state)}")
    primary = codes[0]
    return Decision("stop", primary[0], primary[1], tuple(c[0] for c in codes[1:]))


# ------------------------------------------------------------------------------ prompts


#: The default work of a link. Overridable with `--task` for exactly one reason: it is how
#: the chain mechanics (spawn -> parse the block -> spawn the successor -> stop) can be
#: exercised end to end without a link claiming issues and dispatching real workers. A
#: smoke test that has to start a real fleet is a smoke test nobody runs twice.
DEFAULT_TASK = "Then invoke the `issue-fleet` skill and work the run from Step 0."


def successor_prompt(*, run_id: str, link: int, session_id: str, handoff: str,
                     seed: bool, task: str | None = None) -> str:
    """What each link is told. The contract, not the work — the work is in the skill."""
    task = task or DEFAULT_TASK
    if seed:
        opening = (f"You are the fleet orchestrator for run `{run_id}`, link 1 of an "
                   f"automated rotation chain. Read `{handoff}` — the owner's brief for "
                   f"this run. {task}")
    else:
        opening = (f"You are the fleet orchestrator for run `{run_id}`, link {link} of an "
                   f"automated rotation chain. Read `{handoff}` — the handoff written by "
                   f"link {link - 1}. It is the whole state of this run. {task}")
    return f"""{opening}

Your session id is `{session_id}`. **Pass `session_id="{session_id}"` to every
`fleet_budget()` call.** At your turn 1 the newest top-level records on disk still belong
to your predecessor; an unpinned call would read its context and rotate you before you have
done anything.

Rotation is automated by `host/tools/fleet_run.py`, which spawned you and is waiting on
your exit. When `fleet_budget()` returns `rotate` or `rotate_hard`, follow Step 1.5 as
written — finish the wave in flight, then rewrite `{handoff}` and **end your turn**. Do not
ask the owner to restart you; your successor starts as soon as you exit.

The handoff must contain a fenced ```{RUN_STATE_FENCE}``` block, which is the only part of
it this supervisor reads:

```{RUN_STATE_FENCE}
run_id: {run_id}
run_state: rotated
link: {link}
session_chain: [{session_id}]
waves_done: 0
issues_landed: []
issues_open: []
halt_reason:
```

- `session_chain` appends your session id to the list you inherited; the rest carry forward.
- `run_state: rotated` — work remains, the supervisor spawns link {link + 1}.
- `run_state: complete` — the run is done **and** you have run `session-end`. Chain stops.
- `run_state: halted` — you need the owner. Fill in `halt_reason`. Chain stops.

Two things stop the chain other than you asking it to. If you exit with **no**
```{RUN_STATE_FENCE}``` block, it stops, because a finished run and a crashed one look
identical from outside. If `waves_done` and `issues_landed` are unchanged from what you
inherited, it stops as no-progress — so if you deliberately spend a link on something that
advances neither, say so in `halt_reason` and set `run_state: halted` rather than leaving a
successor to repeat you.

You are running **unattended** under a scoped tool allowlist. A tool outside it is denied,
and a denial halts the chain. Do not engineer around a denial: write `run_state: halted`
with the tool name in `halt_reason`. `--allow-add` is how the owner grants it for the
retry — that decision is theirs, not yours.
"""


SEED_TEMPLATE = """# Fleet run {run_id} — owner brief

Written by `host/tools/fleet_run.py` at launch. Link 1 reads this instead of a handoff.

## Brief

{brief}

## Budget anchors

Pass these to `fleet_budget()` at Step 1 — they are the owner's declared figures, and
without them the verdict silently covers the 5h block only:

- `ceiling_pct`: {ceiling_pct}
- `observed_week_pct`: {week_pct}
- `observed_block_pct`: {block_pct}

## Supervisor bounds

- max sessions in this chain: {max_sessions}
- cumulative weighted-token ceiling: {max_weighted}
- no-progress detector: on (`waves_done` + `issues_landed`)

```{fence}
run_id: {run_id}
run_state: rotated
link: 0
session_chain: []
waves_done: 0
issues_landed: []
issues_open: []
halt_reason:
```
"""


# ------------------------------------------------------------------------- the driver


def resolve_run_env(repo: Path, base_env: dict[str, str]) -> tuple[dict[str, str], str | None]:
    """Env for the spawned link, with `$HOME` corrected to the repo's owner.

    `claude -p` and `gh` both resolve their config off `$HOME` (`~/.claude`, `~/.config/gh`).
    The supervisor is a CLI meant to be launched from whatever shell the owner happens to be
    in -- and on this box that has been root, whose `$HOME` is `/root`, not `/home/sam`. Two
    splits followed from the *same* mismatch: `gh` came up unauthenticated (fleet-20260814-0157),
    and once that was patched with `GH_CONFIG_DIR`, the link's auto-memory writes landed under
    `/root/.claude/projects/.../memory/` -- a directory `sam` cannot even read back
    (fleet-20260814-0322). Both are `$HOME`-rooted, so fixing `$HOME` once fixes both, and does
    it whether the owner launches this as root, sam, or anyone else: the repo checkout's owner
    is the identity whose config should apply, not whoever's shell happens to invoke the CLI.

    Returns `(env, warning)`. `warning` is set (not raised) when the repo owner can't be looked
    up via `pwd` -- e.g. a container with no matching `/etc/passwd` entry -- so a link still
    runs with the caller's own `$HOME` rather than crashing the whole chain over it; the
    supervisor prints the warning so the split doesn't silently reproduce.
    """
    env = dict(base_env)
    try:
        owner_home = pwd.getpwuid(repo.stat().st_uid).pw_dir
    except (KeyError, OSError) as exc:
        return env, f"could not resolve the repo owner's home ({exc}); links inherit $HOME={env.get('HOME')!r} as-is"
    if env.get("HOME") != owner_home:
        env["HOME"] = owner_home
        # XDG_CONFIG_HOME, if the caller's shell set one, would keep pointing at the wrong
        # place even with HOME corrected -- gh and claude both fall back to $HOME/.config
        # only when it is absent.
        env.pop("XDG_CONFIG_HOME", None)
        # GH_CONFIG_DIR was the manual workaround before this existed; drop it too so a
        # caller's exported override doesn't linger and mask the real fix in future debugging.
        env.pop("GH_CONFIG_DIR", None)
    return env, None


def build_argv(prompt: str, *, session_id: str, model: str | None,
               allowed_tools: tuple[str, ...]) -> list[str]:
    argv = ["claude", "-p", prompt,
            "--output-format", "stream-json", "--verbose",
            "--session-id", session_id,
            "--allowedTools", ",".join(allowed_tools),
            # fleet-20260814-1503: the owner's global ~/.claude/settings.json installs a
            # PreToolUse hook (rtk) that transparently rewrites every Bash command --
            # `git status` becomes `rtk git status` -- before the allowlist checks it. The
            # rewritten command no longer starts with `git`/`gh`/etc, so it matches nothing
            # in DEFAULT_ALLOWED_TOOLS and every Bash call in the link was denied, even ones
            # explicitly granted. That hook lives in the "user" setting source; "project"
            # and "local" carry what a link actually needs (orch-edit-count.sh,
            # session-end-guard.sh, the repo's own permission grants) and neither rewrites
            # Bash. Excluding "user" here, not disabling hooks wholesale (`--bare` also
            # drops auto-memory and CLAUDE.md discovery, both of which a link needs).
            "--setting-sources", "project,local"]
    if model:
        argv += ["--model", model]
    return argv


def render_event(event: dict) -> str | None:
    """One compact terminal line per interesting stream event, or None to stay quiet.

    An unattended link runs for tens of minutes; a supervisor that prints nothing until it
    exits is indistinguishable from one that has hung. The raw stream still goes to the log
    in full -- this is the watchable summary, not the record.
    """
    kind = event.get("type")
    if kind == "assistant":
        for block in event.get("message", {}).get("content", []) or []:
            if block.get("type") == "tool_use":
                name = block.get("name", "?")
                inp = block.get("input", {}) or {}
                detail = (inp.get("command") or inp.get("file_path")
                          or inp.get("pattern") or inp.get("description") or "")
                return f"    {name}({str(detail)[:90]})"
            if block.get("type") == "text" and block.get("text", "").strip():
                return f"  · {block['text'].strip().splitlines()[0][:110]}"
    elif kind == "system" and event.get("subtype") == "init":
        return f"    [session {event.get('session_id', '?')[:8]}]"
    return None


@dataclass
class LinkResult:
    link: int
    session_id: str
    exit_code: int = 0
    is_error: bool = False
    denials: list = field(default_factory=list)
    num_turns: int = 0
    cost_usd: float = 0.0
    weighted_delta: float = 0.0
    duration_s: float = 0.0
    timed_out: bool = False
    orch_code_edits: int = 0
    state: dict | None = None
    decision: Decision | None = None

    def as_dict(self) -> dict:
        d = {k: v for k, v in self.__dict__.items() if k != "decision"}
        if self.decision:
            d["decision"] = {"action": self.decision.action, "code": self.decision.code,
                             "reason": self.decision.reason, "also": list(self.decision.also)}
        return d


def _rel(path: Path) -> str:
    """Repo-relative when it can be — a link's `cwd` is the repo, and a relative path in
    the prompt is one the session can paste straight into `Read`. Absolute otherwise,
    rather than raising: a handoff kept outside the repo is unusual, not an error."""
    try:
        return path.relative_to(REPO).as_posix()
    except ValueError:
        return str(path)


#: Written by `.claude/hooks/orch-edit-count.sh`, one line per orchestrator edit onto code
#: outside a worker's worktree. Append-only and shared by every session on this machine,
#: which is why the count is filtered by session id rather than differenced.
ORCH_EDIT_LOG = "orch-edits.log"


def orch_edits(session_id: str, fleet_dir: Path | None = None) -> int:
    """How many code edits this link authored itself — the ledger's `orch_code_edits`.

    Filtered by session id, not differenced across the link: the log is shared with every
    other session on this box, so a difference would attribute an unrelated session's edits
    to whichever link happened to be running. Absent log means 0 -- the hook not being
    installed and the rule being obeyed look identical here, which is why
    `test_orch_edit_hook.py` checks the wiring separately.
    """
    log = (fleet_dir or FLEET_DIR) / ORCH_EDIT_LOG
    if not log.exists():
        return 0
    return sum(1 for line in log.read_text(encoding="utf-8").splitlines()
               if f"\t{session_id}\t" in line)


def weighted_7d(now: datetime | None = None) -> float:
    """Trailing-7d weighted tokens, read straight from the transcripts.

    Differenced across a link, this is the link's spend -- the same derivation the ledger
    uses at Step 9, and it counts subagents, which are most of a wave. The native reader
    rather than ccusage: no npx, no network, and no pinned-schema failure mode inside a
    loop that must keep running unattended.
    """
    records, _ = fleet_budget.read_transcripts()
    return fleet_budget.rolling_window(records, fleet_budget.WEEK_HOURS,
                                       now or datetime.now(UTC))


def run_link(prompt: str, *, session_id: str, link: int, model: str | None,
             allowed_tools: tuple[str, ...], log_path: Path, timeout_s: int,
             echo: bool = True) -> LinkResult:
    """Spawn one orchestrator session and drain its stream. Blocks until it exits."""
    res = LinkResult(link=link, session_id=session_id)
    argv = build_argv(prompt, session_id=session_id, model=model, allowed_tools=allowed_tools)
    started = datetime.now(UTC)
    before = weighted_7d(started)

    run_env, home_warning = resolve_run_env(REPO, dict(os.environ))
    if home_warning and echo:
        print(f"    ! {home_warning}", file=sys.stderr)

    proc = subprocess.Popen(argv, cwd=str(REPO), stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, bufsize=1,
                            env=run_env)

    def _kill():
        res.timed_out = True
        proc.kill()

    watchdog = threading.Timer(timeout_s, _kill)
    watchdog.start()
    try:
        with log_path.open("w", encoding="utf-8") as log:
            for line in proc.stdout:  # type: ignore[union-attr]
                log.write(line)
                line = line.strip()
                if not line.startswith("{"):
                    continue
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if event.get("type") == "result":
                    res.is_error = bool(event.get("is_error"))
                    res.denials = event.get("permission_denials") or []
                    res.num_turns = int(event.get("num_turns") or 0)
                    res.cost_usd = float(event.get("total_cost_usd") or 0.0)
                elif echo:
                    rendered = render_event(event)
                    if rendered:
                        print(rendered, flush=True)
        res.exit_code = proc.wait()
    finally:
        watchdog.cancel()

    finished = datetime.now(UTC)
    res.duration_s = round((finished - started).total_seconds(), 1)
    # A rolling window can drop records out of the back while a link runs, so the raw
    # difference is a lower bound on spend and can even go negative. Clamped, because a
    # negative "spend" would silently extend the run's ceiling.
    res.weighted_delta = max(0.0, weighted_7d(finished) - before)
    if res.timed_out:
        res.exit_code = res.exit_code or -1
    return res


def run_chain(*, run_id: str, handoff: Path, limits: Limits, model: str | None,
              allowed_tools: tuple[str, ...], timeout_s: int, echo: bool = True,
              seed: bool = False, task: str | None = None) -> dict:
    """Spawn links until something says stop. Returns the chain record."""
    links: list[LinkResult] = []
    spent = 0.0
    prev_progress = progress_key(parse_run_state(handoff.read_text(encoding="utf-8")))
    link = 1
    while True:
        session_id = str(uuid.uuid4())
        rel = _rel(handoff)
        prompt = successor_prompt(run_id=run_id, link=link, session_id=session_id,
                                  handoff=rel, seed=(seed and link == 1), task=task)
        if echo:
            print(f"\n── link {link}  session {session_id[:8]}  "
                  f"(spent {spent:,.0f} weighted) ──", flush=True)
        log_path = handoff.with_suffix(f".session-{link}.log")
        res = run_link(prompt, session_id=session_id, link=link, model=model,
                       allowed_tools=allowed_tools, log_path=log_path,
                       timeout_s=timeout_s, echo=echo)
        spent += res.weighted_delta
        text = handoff.read_text(encoding="utf-8") if handoff.exists() else ""
        res.state = parse_run_state(text)
        res.orch_code_edits = orch_edits(session_id, handoff.parent)
        res.decision = next_action(link=link, state=res.state, denials=res.denials,
                                   exit_code=res.exit_code, is_error=res.is_error,
                                   prev_progress=prev_progress, spent_weighted=spent,
                                   limits=limits)
        links.append(res)
        if echo:
            print(f"── link {link} ended after {res.duration_s:.0f}s, {res.num_turns} turns, "
                  f"{res.weighted_delta:,.0f} weighted → {res.decision.code}: "
                  f"{res.decision.reason}", flush=True)
        if res.decision.action == "stop":
            break
        prev_progress = progress_key(res.state)
        link += 1

    last = links[-1]
    return {
        "run_id": run_id,
        "handoff": str(handoff),
        "links": [link_res.as_dict() for link_res in links],
        "sessions": len(links),
        "weighted_total": round(spent, 1),
        "orch_code_edits": sum(l.orch_code_edits for l in links),
        "stopped_because": last.decision.code if last.decision else "?",
        "reason": last.decision.reason if last.decision else "",
        "also": list(last.decision.also) if last.decision else [],
        "final_state": last.state,
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--run-id", default=None,
                    help="default: fleet-YYYYmmdd-HHMM (also names the handoff file)")
    ap.add_argument("--brief", default=None,
                    help="what this run should work on; seeds link 1 when there is no "
                         "handoff yet (e.g. 'burn down priority/now in area/host-tools')")
    ap.add_argument("--ceiling-pct", type=float, default=80.0)
    ap.add_argument("--observed-week-pct", type=float, default=None)
    ap.add_argument("--observed-block-pct", type=float, default=None)
    ap.add_argument("--max-sessions", type=int, default=DEFAULT_MAX_SESSIONS)
    ap.add_argument("--max-weighted", type=float, default=None,
                    help="cumulative weighted-token ceiling for the WHOLE chain, "
                         "differenced from the transcripts across each link")
    ap.add_argument("--no-weighted-ceiling", action="store_true",
                    help="run without a token ceiling; --max-sessions and the no-progress "
                         "detector are then the only bounds")
    ap.add_argument("--model", default=None, help="model for every link (default: inherit)")
    ap.add_argument("--allow", action="append", default=None,
                    help="replace the default tool scope (repeatable)")
    ap.add_argument("--allow-add", action="append", default=[],
                    help="extend the default tool scope (repeatable)")
    ap.add_argument("--link-timeout", type=int, default=DEFAULT_LINK_TIMEOUT_S)
    ap.add_argument("--task", default=None,
                    help="override each link's task line (default: invoke the issue-fleet "
                         "skill). Exists so the chain mechanics can be smoke-tested "
                         "without claiming issues or dispatching workers")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan and link 1's prompt; spawn nothing")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    if args.max_weighted is None and not args.no_weighted_ceiling:
        ap.error("pass --max-weighted (a cumulative weighted-token ceiling for the chain) "
                 "or --no-weighted-ceiling to run without one. There is no defensible "
                 "default: this box has no quota API, so any number baked in here would be "
                 "invented — see fleet_budget.py's calibration note.")

    run_id = args.run_id or f"fleet-{datetime.now(UTC):%Y%m%d-%H%M}"
    FLEET_DIR.mkdir(parents=True, exist_ok=True)
    handoff = FLEET_DIR / f"{run_id}.md"
    seed = not handoff.exists()
    if seed:
        handoff.write_text(SEED_TEMPLATE.format(
            run_id=run_id, fence=RUN_STATE_FENCE,
            brief=args.brief or "(none given — link 1 should ask the owner before claiming)",
            ceiling_pct=args.ceiling_pct,
            week_pct=args.observed_week_pct if args.observed_week_pct is not None else "unset",
            block_pct=args.observed_block_pct if args.observed_block_pct is not None else "unset",
            max_sessions=args.max_sessions,
            max_weighted=(f"{args.max_weighted:,.0f}" if args.max_weighted else "none"),
        ), encoding="utf-8")

    allowed = tuple(args.allow) if args.allow else DEFAULT_ALLOWED_TOOLS
    allowed = allowed + tuple(args.allow_add)
    limits = Limits(max_sessions=args.max_sessions,
                    max_weighted=None if args.no_weighted_ceiling else args.max_weighted)

    if args.dry_run:
        plan = {
            "run_id": run_id, "handoff": str(handoff), "seeded": seed,
            "max_sessions": limits.max_sessions, "max_weighted": limits.max_weighted,
            "model": args.model or "(inherit)", "allowed_tools": list(allowed),
            "link_timeout_s": args.link_timeout,
            "prompt_link_1": successor_prompt(run_id=run_id, link=1,
                                              session_id="<uuid-per-link>",
                                              handoff=_rel(handoff), seed=seed,
                                              task=args.task),
        }
        print(json.dumps(plan, indent=2) if args.json else
              "\n".join(f"{k}: {v}" for k, v in plan.items() if k != "prompt_link_1")
              + f"\n\n--- link 1 prompt ---\n{plan['prompt_link_1']}")
        return 0

    record = run_chain(run_id=run_id, handoff=handoff, limits=limits, model=args.model,
                       allowed_tools=allowed, timeout_s=args.link_timeout,
                       echo=not args.json, seed=seed, task=args.task)
    chain_path = handoff.with_suffix(".chain.json")
    chain_path.write_text(json.dumps(record, indent=2), encoding="utf-8")

    if args.json:
        print(json.dumps(record, indent=2))
    else:
        print(f"\n{run_id}: {record['sessions']} session(s), "
              f"{record['weighted_total']:,.0f} weighted tokens, "
              f"stopped on {record['stopped_because']} — {record['reason']}")
        if record["also"]:
            print(f"  also fired: {', '.join(record['also'])}")
        print(f"  chain record: {chain_path}")
    # A chain that ended because the run finished is a success; every other stop is not.
    return 0 if record["stopped_because"] == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
