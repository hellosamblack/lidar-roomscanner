# Engineering Practices — roomscanner

Conventions for all work in this workspace. `AGENTS.md` (repo root; `CLAUDE.md` is a symlink to it)
points here; keep this doc short and binding.

## Repository rules

- `firmware/vendor/53L9A1/` is **read-only reference** (vendored ST package). Never edit it, even to fix known bugs (they're catalogued in
  `ROADMAP.md` → "Reference-firmware bugs"). Our firmware fork lives in `firmware/scanner-stream/` and references the
  package's Drivers/Middlewares/Utilities in place via CMake paths (`PKG_ROOT`).
- Layout: `firmware/` (STM32 apps), `host/` (PC Python package `roomscan`), `docs/` (specs, plans,
  captures), `references/` (imported research, read-only), `.agents/skills/` (project skills —
  `.claude/skills/` and `.codex/skills/` are symlinks to it).
- Commit style: conventional-commit-ish prefixes (`feat:`, `fix:`, `docs:`, `test:`, `chore:`), small and
  frequent. Never commit `build/` output or captured binary streams >1 MB (put large captures in
  `captures/` — gitignored — and check in only the small golden fixtures under `host/tests/fixtures/`).
- **Docs move with the code (status-sync rule).** Any commit that advances/closes a work item,
  clears a deferred item, changes a measured number, or invalidates a prediction updates `ROADMAP.md`
  (and `CLAUDE.md`/memory when status changes) **in the same commit** — follow the `status-sync` skill
  checklist. "Docs later" is how the 2026-07-10 drift happened. Before writing the commit that
  *closes* the session's governing issue (`Closes #NNN`), hand off to the `session-end` skill in the
  same turn: it records memory and applies self-improvements first, then lands that commit with them
  included. Committing first and wrapping up after strands every improvement in a trailing commit
  `status-sync` never sees (#169); a `Stop`-hook backstop
  (`.claude/hooks/session-end-guard.sh`) catches that degraded path. `status-sync` still runs on every
  landing, including mid-session ones that only `Refs #NNN`.
- **Tracker layout (hot vs cold).** `ROADMAP.md` is the *current-state* doc — standing decisions, the
  reference-firmware bug ledger, cross-cutting risks, and the plans/specs register. Completed-work
  narratives live in `docs/roadmap-history.md` (they keep their `Phase N` names). Keep `ROADMAP.md`
  small — when a piece is done, move its write-up to the archive and leave a one-line stub, don't
  accrete it.
- **Forward-looking work and defects are GitHub Issues** (moved off `BUGS.md`/`bugs/`/ROADMAP's
  Work-item register 2026-08-10; `docs/issue-migration-map.md` has the old-ID → issue mapping). File
  one: `gh issue create --repo hellosamblack/lidar-roomscanner --label bug|work-item|data-collection
  --label area/<area>` (area labels mirror the old `Area` vocabulary: `area/host-viewer`, `-panel`,
  `-sensors`, `-slam`, `-web`, `-transport`, `-tools`, `-offline`, `-splat`, `area/firmware`, `-eth`,
  `-scanner-stream`, `-build`, `-host`, `area/transform-lib`, `area/environment`). Close one:
  `gh issue close <n> --reason completed` (a code change shipped) or `"not planned"` (by-design,
  anomaly, investigated — nothing is going to change); add the matching `status/*` label
  (`status/by-design`, `-anomaly`, `-vendor`, `-mitigated`, `-investigated`, `-fix-unverified`,
  `-blocked`, `-partial`) when the close reason alone doesn't carry the nuance. `bug` is for defects,
  `work-item` for forward-looking work (was the `SLAM-`/`SENS-`/`XPORT-`/`FW-`/`OFFLINE-`/`TOOL-`
  register), `data-collection` for owner-collected-capture items (was the `DC-*` queue).
- **Branch discipline (owner workflow, 2026-07-16).** Land work by **committing straight to `main`
  and closing any feature/worktree branch without a PR** (the PR flow is retired — no `gh pr create`,
  no `gh pr merge`). A short-lived branch/worktree for isolation is fine; finish by getting the commit
  onto `main` and deleting the branch. Pushing to `origin` is a separate, owner-triggered step.
  Subagents don't commit; the controlling session does. **Fleet carve-out (owner, 2026-08-12,
  issue #170):** under the `issue-fleet` skill a worker *may* commit `Refs #NNN` on its own branch
  **inside its own worktree**, because the isolation is what the original rule was protecting
  against — the 2026-07-10 mis-commit was a subagent running `git commit` at the *main* checkout.
  The orchestrator still owns every boundary beyond that branch: it reviews, rebases in the
  worktree, `merge --ff-only`s onto `main`, and writes the single `Closes` commit. A worker still
  never touches `main`, shared docs, or `gh`.
- **Path lengths.** Repo-relative paths stay ≤150 characters (longer breaks `git worktree add` and
  fresh clones on default Windows git).

## Wire protocol

- Single source of truth: `docs/protocol.md`. The C header (`firmware/.../rs_protocol.h`) and Python
  module (`host/src/roomscan/protocol.py`) implement it; **golden test vectors** (exact bytes of a known
  frame, checked into `host/tests/fixtures/`) prove they agree.
- Little-endian everywhere. CRC32 = IEEE 802.3 (zlib) over header+payload, transmitted last.
- Any layout change **bumps `version`** and updates spec + C + Python + vectors in the same commit —
  follow the `protocol-change` skill checklist.
- Decoders never trust the link: resync on magic, bound `payload_len` before allocating, drop CRC
  failures silently but count them.

## Firmware (STM32H563)

- Build: `cmake --preset Debug && cmake --build build/Debug` from the app dir; flash + monitor via the
  `firmware-loop` skill. Validation is on-target — there is no simulator; every firmware change ends with
  a flash-and-observe step.
- **Build-time config is selected by CMake, never by hand-editing a `#define`.** `CONF_TRANSFORM_ONBOARD`
  is a real `option()` (default OFF → the shipped raw-only config); the `DebugOnboardTransform` preset
  turns it on into its own `build/DebugOnboardTransform` dir. Configure prints the chosen value, so a
  requested config can no longer silently fall back. **That alternate config currently wedges the rig on
  Ethernet — see #166 before flashing it.** If you add another such knob, follow the same shape: `option()`
  + `target_compile_definitions(... FOO=$<BOOL:${FOO}>)` + an `#ifndef` fallback in the source, so a
  non-CMake compile still builds and a `-D` on the command line is never a no-op.
- **Claude drives the on-target loop directly — this is an agentic project, not a hand-off.** Build, flash
  (`STM32_Programmer_CLI` over SWD), and monitor (native CDC via `capture.py` on VID/PID `CAFE:4001`;
  ST-Link VCOM for `printf`/probe output) are all Claude's to run. Toolchain + programmer paths and the
  probe/register-readback pattern are in the `firmware-loop` skill. Read registers on-target over SWD
  (`STM32_Programmer_CLI -c port=SWD mode=hotplug -r32 <addr> <n>`, addresses from `build/Debug/*.map`) to
  diagnose without guessing — e.g. `uwTick` for core-liveness, `g_lsm_ok`/`g_last_seq` for boot stage.
  Do **not** write "next bench steps (owner)" for anything firmware can do; take it to the hardware yourself.
- **The human does physical-only actions**, and only these: moving IKS4A1/53L9A1 jumpers & solder bridges,
  scope probing, and power-cycling (USB unplug/replug) to clear a warm-wedged I3C bus that survives MCU
  reset. Rapid flash/reset cycles during probing can wedge the independently-powered ToF/LSM on the shared
  I3C bus; if `capture.py` and `-hardRst` both fail to bring the CDC back but `uwTick` still advances, it's
  that warm-wedge — ask for a replug, don't keep resetting.
- **ST-Link power and clock dependency:** The target MCU's main system clock is configured to use the ST-Link's Master Clock Output (MCO) via `RCC_HSE_BYPASS_DIGITAL`. If the ST-Link USB cable is unplugged, the ST-Link chip is unpowered, the 8 MHz clock signal is lost, and the MCU halts in `Error_Handler()` inside `SystemClock_Config()`. Furthermore, an unpowered ST-Link pulls the target MCU's `NRST` line low (resetting it). Therefore, both USB USER and ST-Link cables must be connected (or ST-Link powered externally) for the board to run.
- Keep `USER CODE BEGIN/END` guards intact in CubeMX-generated files even in our fork.
- Error policy: no silent failures. Every `vl53l9_*`/`transform_*` call's return value is checked at the
  call site (watch reference bug #1 — a dropped assignment defeats the check). Streaming errors become
  event frames to the host, then re-init; only unrecoverable HAL faults may spin.
- No `malloc` after setup; all steady-state buffers allocated once. Size variables are `size_t`/`uint32_t`
  (reference `allocate_memory` uses `uint16_t` — don't inherit).
- TX must never stall acquisition: check busy, set the drop flag, move on. Measure TX time whenever the
  frame path changes.
- `-Ofast` is in effect: don't rely on NaN semantics in firmware code.
- Portable protocol/encoding code goes in standalone `.c/.h` with no HAL includes, so it can be compiled
  and unit-tested host-side if needed.

## Host (Python)

- Package `roomscan` under `host/` with `pyproject.toml`; Python ≥3.11. Deps: `numpy`, `pyserial`,
  `open3d`; dev: `pytest`, `ruff`.
- TDD for everything below the viewer: protocol, decoder, deprojection, sources are pure/mockable — write
  the failing test first. The Open3D render loop is validated manually (it's a window), but everything it
  consumes is tested.
- Decoder and deprojection operate on `bytes`/`numpy` — no I/O in those modules. I/O lives in
  `sources.py` so tests never need hardware.
- Every capture-format consumer must also accept file replay (`FileSource`) - hardware-free development
  and regression datasets come for free.
- **Logging**: The app writes automatic rotating logs to `logs/app.log` (Python tracebacks, UI actions) and 
  `logs/firmware.log` (ST-Link VCOM output). Always check these when diagnosing crashes or hangs.
- **Adding a parameter to a long function? Grep the body for that name first.** `web.py`'s builders are
  100+ lines and reuse obvious names, so a new parameter can be shadowed by an existing local and silently
  take the wrong value — `build_sensor_message` already had a local `display_quat` (the yaw-offset
  orientation view), which clobbered a new `display_quat` argument so it read the raw quat instead of the
  smoothed one (BUG-026 follow-up). It produced a *plausible* number, which is what made it dangerous; only
  a test asserting the `None` default caught it. Prefer a qualified name (`ir_display_quat`).
- **Never mutate interpreter-global state in a test — use a subprocess.** A test asserting that
  `roomscan.mcp_server` doesn't import `open3d` eagerly did `sys.modules.pop("open3d")`, which broke
  three unrelated `test_panel_modes` tests *only in a full-suite run* (they passed in isolation, which
  is the confusing part — the failures name the victim, not the culprit). The suite shares one
  interpreter, so `sys.modules`, `os.environ`, `logging` config and monkeypatched globals all leak
  forward. If an assertion is genuinely about import-time or process-level behaviour, run it in a
  `subprocess` and assert on its output. **A test that passes alone but fails in the suite is almost
  always a *different* test's leak** — bisect by running the failing file after suspect files, not by
  debugging the failing file.

## Web UI

- **Every interactive button/toggle/control must have a `title` attribute** (native browser tooltip)
  describing what it does. Use short, plain-language phrases — one sentence max, no jargon. When adding
  a new button to `index.html`, add a `title` in the same commit. For `<label class="toggle">` elements,
  put the `title` on the `<label>`, not the inner `<input>`.
  **This is now enforced** by `host/tests/test_static_ui.py` — it was review-only for months and had
  drifted past nine controls. That module holds the other whole-file markup invariants too: no
  duplicate `id=` (BUG-047 — one id named two buttons, so "Restart Server" also fired a playback
  transport restart), every `data-card-id` present in `CARD_ICONS`/`CARD_TITLES` (a card without an
  icon entry gets no squircle and is unreachable once collapsed), and `user-select: text` on the
  read-only sinks that `body { user-select: none }` would otherwise make uncopyable.
  *If a rule here could be checked by a grep, write the grep as a test in the same session.*
- **Read-only telemetry needs tooltips too.** The rule above says "interactive", so the HUD counters
  (Drops/Gaps/stream rows) had none — and those are exactly the numbers whose meaning is not
  guessable. Key any per-stream help map on `stream_id`, **not** `label`: `metrics.py` maps two
  different ids onto `"ToF"`, so a label-keyed map cannot tell a replay from a live capture.
- **Client-owned animation state must not live in a field the `state` handler assigns.** `state` is
  re-broadcast on *every* unrelated setting change, so a value stored in (say) the sign of
  `controls.autoRotateSpeed` is silently reset whenever someone clicks an unrelated control. Give it
  its own variable and re-assert it each frame; the server owns magnitude, the client owns
  direction/phase.
- **Changing a `ViewerConfig` default needs a migration.** `web._persist_ui` writes the whole
  `[viewer]` table on any single UI change, so an install that never touched a field still has the
  *then*-current default on disk, and `ui_from_config` reads it back in preference to the new code
  default — the change is invisible to every existing install. Migrate the exact old default only,
  and leave any other stored value alone as a real customization.

## Verification discipline

- Before claiming a milestone works: run the pytest suite, then the on-target check (flash, run the host
  tool, observe fps + zero CRC failures + zero seq gaps). State actual numbers, not "works".
- When debugging link problems, capture raw bytes first (`--record`), then debug offline against the
  file — don't iterate on live hardware.
- **Web UI work is verified visually, not just by tests.** On this headless host use the `ui_*` MCP
  tools — `rig_up(replay=…)`, then `ui_screenshot()` (returns the PNG inline), `ui_wait_for(…)` and
  `ui_eval(…)` to assert real state. `host/tools/web_ui_shot.py` remains the fallback for a client
  without MCP. Full recipe in `docs/web-ui-testing.md`; the tool surface in `docs/mcp-server.md`.
- **Prefer an MCP tool over parsing a CLI's prose** (`docs/mcp-server.md`): `capture_analyze` over
  reading `analyze_capture.py` output, `rig_status`/`orientation_probe` over scraping fps, `doctor`
  over eyeballing PASS/FAIL lines. New agent-facing capability ⇒ new MCP tool, per CLAUDE.md.
- **Don't assert on a quantity you haven't measured.** A sub-phase 6.G test checked that a CPU grid
  releases no CUDA cache, but asserted `point_cloud()` returns >100 points along the way — on that map
  (4 integrations at `weight_threshold=3.0`) it legitimately returns 0 where `mesh()` does not, so the
  test failed for a reason unrelated to what it covers. Assert the behaviour under test; for anything
  incidental, either measure the real value first or just call the function. This one landed **red on
  main** because the verification window was blocked (see the `concurrent-sessions-shared-checkout`
  memory — unverified work in a shared tree can be committed by another session before you run it).
- **A tool must report what happened, not what was asked for.** Three bugs found while verifying the
  MCP server were all this shape: a set that echoed a timer-driven `state` predating the change, a
  `go_live` that no-ops on a `--replay` server, and a record that silently refuses in replay. Each
  returned success. If a tool sends a request, check the resulting state actually changed.

- **Prove a regression test by reintroducing the defect — and check the injection landed.** A test
  written after the fix has never been seen to fail, so it is an assertion about nothing until you
  put the bug back. Two failure modes, both hit on 2026-07-31: an injection whose anchor text didn't
  match left the test passing, which is indistinguishable from a weak test; and an injection that
  *did* apply left the test still passing, correctly revealing the fix had **no** coverage at all
  (`DetailedSlamPreset.fingerprint`). So assert the edit applied (`assert s.count(old) == 1`)
  separately from the pytest result — only then does green-after-restore mean anything.
- **A shape test cannot see a units, origin, or clock error.** BUG-050 (`elapsed_s` computed as
  `time.time() - time.monotonic()`, reporting 1.78e9 s for a 90-second take) survived two existing
  tests because both asserted the key was present and was a float — and a wrong-by-a-billion float
  is one. When the value is a *quantity*, assert it against a **known** magnitude (advance the clock
  by 90 s, assert ~90), not against its type. Any subtraction across `time.time()` and
  `time.monotonic()` in the same file is a bug until proven otherwise.
- **Verify against the shipped function, not a retyped copy.** A JS check that re-types the
  algorithm into a scratch file tests the copy, and the copy drifts. Extract the real function out of
  the source at run time (`readFileSync` + a match + `new Function`) and drive it against a stub —
  that is what caught the oscillate orbit's `state`-echo interaction, which the retyped version could
  not see because the bug was in the interaction, not the arithmetic.
- **A stationary rig measures nothing.** An A/B whose code path is gated on motion (the mag-cal
  behind-camera recompute fires only past a 1.5° step) reports identical numbers in both arms on a
  parked device, and the "slower" arm can measure faster on noise alone. Drive the input the feature
  actually responds to — a synthetic tumble is fine — or say plainly that the measurement is absent.
- **Verify a backup before the step that needs it, and never at a guessable path.** Restoring a file
  from `/tmp/<name>.bak` overwrote uncommitted work with *another session's* leftover copy: the
  backup `cp` had silently failed (the Bash cwd had drifted, so the repo-relative path missed) while
  the restore `cp` succeeded against the pre-existing file. `set -e` doesn't cover a failing command
  in a compound line. Use `mktemp`, gate on `test -s`, use absolute paths in any script that mutates
  files, and prefer git (`git checkout HEAD -- <f>`, a scratch commit, `git diff > patch`) over an
  ad-hoc backup — in a shared checkout the obvious `/tmp` name is not yours. Commit valuable
  uncommitted work *before* experimenting on it.

- **Do not write down a mechanism you have not tested.** BUG-035 shipped with the explanation "the
  VoxelBlockGrid pre-allocates and does not grow", which was never checked and is false — a CUDA grid
  rehashes 40,000 → 80,000 at 99.2% load. The *effect* (560 lost frames at 40k vs 11 at 120k) and the
  *mitigation* were solidly measured; only the story about why was invented, and it reached BUGS.md,
  ROADMAP.md, CLAUDE.md, a warning string and a test before a stray "4896 blocks / capacity 2000"
  caught it. A measured effect plus an honest "mechanism unproven, best hypothesis is X" is worth more
  than a tidy causal story, because the tidy story is what the next session builds on. When the fix
  works regardless, say so — that is what licenses shipping without the full explanation.

### Verifying a rotation (or any sign)

A rotation that is correct in magnitude but inverted in direction passes most tests you would think to
write. The IR gravity roll shipped backwards **twice** this way (BUG-026 follow-ups) — the content
counter-rotated at 2× the board's rate, which is worse than applying no correction at all.

- **Never verify a sign at a multiple of 90°.** 180° is its own inverse (−180 ≡ +180), and a 90° turn
  swaps an image's width/height either way. Both of the original checks were of this form. Pick an angle
  like 30° or 45°, where a flip is unmissable.
- **Derive the expectation from an already-verified path, not from the formula.** Restating the
  implementation in the test only pins the typo, not the convention. Here the point cloud was already
  verified, and the IR image comes off the same 54×42 grid, so "where the aligned cloud puts the image's
  +u axis on screen" is an independent ground truth — and it caught the inversion instantly.
- **Watch for frames whose handedness flips the visual sense.** The specific trap:
  `T_WORLD_TO_CV @ R @ T_CV_TO_BODY` rotates points in the CV frame where **Y points down**, so a positive
  rotation there is *clockwise* on screen, while `np.rot90` is *counter-clockwise*. Same number, opposite
  turn. `docs/coordinate-frames.md` is the reference; screen-space sense is not written on the matrix.
- **Prefer an exact geometric check over a statistical proxy.** The first attempt here regressed
  structure-tensor edge orientation against the correction angle and came back inconclusive (21.4 vs 23.7)
  because a *panning* sweep has no stable dominant edge. Exact geometry settled it in one step; the
  statistics were only worth running afterwards, on a genuine boresight-roll capture, as independent
  confirmation.
- **Synthetic attitude captures cannot validate stabilisation.** `host/tools/roll_capture.py` rewrites
  only the stream-9 quaternion, so the image content does *not* co-rotate the way it would in reality. It
  exercises the mechanism (dimensions, transform plumbing, fit) but is blind to whether content is
  actually held still. That needs a real physical roll about the boresight — ask the owner to record one.

### Reporting a measured improvement

An X× claim is a technical assertion; treat a suspiciously good one as a bug in the measurement until
proven otherwise. The 2026-07-28 orientation-noise pass produced three wrong numbers before a right one
(BUG-027), all from the same class of mistake:

- **Know your metric's floor before you quote a ratio.** Measure the metric with the effect *disabled*
  (there, a replay with no IMU: floor 0.0004°). Without that you can't tell "we fixed it" from "we hit
  the measurement's own resolution".
- **Check for quantization in the readout path.** Reading noise off the `sensor` JSON `rot` gave a bogus
  42× because `build_sensor_message` rounds it to 5 decimals and censors small changes to exactly zero.
- **Before/after must be measured identically**, adjacent in time, with every intermediate filter in the
  same state — a host-side smoother left enabled will mask the firmware effect you're attributing.
- **Check that the data is still live.** A stalled sensor leaves the host holding a stale value, which
  reads as a spectacular noise reduction. Confirm stream rates + 0 drops/gaps after every reflash
  (`host/tools/orientation_probe.py health`).
- Order-independent statistics beat index-wise diffs on variable-length payloads: POINT_CLOUD carries
  only *valid* points, so index i is a different ray between frames.
- **Timing runs need an idle box, and a suspiciously bad tail is usually contention.** A 6.G replay
  reported p99 22.1 ms against a 14.8 ms baseline; the cause was a second heavy job this session had
  itself backgrounded, and the clean re-run came in at **11.9** — better than baseline. The tell is the
  shape: contention inflates p99/max while leaving p50 alone, whereas a real regression moves the median
  too. Check for other running jobs *before* the run, and prefer isolating the suspect cost directly (a
  microbenchmark put the accused code at 5.8 µs/call, 0.02 s across the whole scan) over re-running a
  20-minute end-to-end and hoping. Beware that `pgrep -f <pattern>` matches its own enclosing shell —
  it will answer "busy" when the box is idle.

## Self-improvement after milestones

- **After every milestone** (a phase completing, or any major merge to main), run a retrospective BEFORE
  starting the next phase — follow the `milestone-retro` skill. The question is always: *what would have
  made this push easier, done as a reusable artifact?*
- Convert findings into durable tooling, not notes: new/updated **skills** under `.agents/skills/` (with
  `references/` and `scripts/` subdirectories where they earn their keep), shared **scripts** under
  `host/tools/`, and corrections to existing docs. Match the structure of the skills already there —
  frontmatter `name`/`description` with concrete trigger phrases, then a short imperative body.
- Hard rules of thumb: any hardware ritual performed from prose by more than two subagents becomes a
  script; any environment fact discovered the hard way (tool paths, port quirks, timing windows) becomes
  a line in the relevant skill; any repeated review finding becomes a checklist item in the skill that
  governs that work.
- The retro's output is committed as part of closing the milestone — a milestone isn't done until the
  next one got easier.
