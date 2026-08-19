# AGENTS.md

Canonical agent-guidance file for this repository, read by Claude Code (claude.ai/code) and Codex.
The real file is root `AGENTS.md`; `CLAUDE.md` (repo root) and `.agents/AGENTS.md` are relative
symlinks to it, and `.claude/skills/` + `.codex/skills/` are symlinks to the canonical
`.agents/skills/` (shared-content canonicalization, 2026-08-11, issue #163). Edit this file; all
names resolve to it.
Phrasing below often says "Claude" because that is the primary driver, but the guidance applies to
any coding agent working here.

## What this is

`roomscanner/` is the **active development workspace** for a tethered handheld **3D room scanner**. The end goal: an STM32H563ZI board streams timestamped ToF (+ later IMU/env) frames to a PC that runs real-time SLAM (Open3D tensor ICP + TSDF), with an offline pass fusing 4K phone video into a ToF-seeded 3D Gaussian Splat.

New work — the PC-side visualizer, the binary frame protocol, and any new firmware — happens **here**. The existing STM32 firmware lives in a **reference package**, vendored in-repo at `firmware/vendor/53L9A1/`, that we treat as **read-only reference**, not something we edit in place.

## Repository layout

```
/home/sam/git/personal/lidar-roomscanner/   ← the repo root (active dev workspace)
├─ AGENTS.md               ← this file (CLAUDE.md + .agents/AGENTS.md are symlinks to it)
├─ ROADMAP.md              ← current-state doc: standing decisions, reference-firmware bug ledger, risks, plans register. Forward-looking work + defects are GitHub Issues (see "Work tracking" in ROADMAP.md) — labels bug/work-item/data-collection + area/* + status/* + priority/now|next|later
├─ docs/roadmap-history.md ← completed-phase narratives + measured outcomes (they keep their Phase-N names); ROADMAP.md links here
├─ BUGS.md                 ← stub pointing at GitHub Issues (`gh issue list --label bug`); old BUG-NNN → issue mapping in docs/issue-migration-map.md (human-readable, generated from docs/issue-migration-map.json — the machine-readable source of truth)
├─ .agents/skills/         ← canonical project skills (.claude/skills/ + .codex/skills/ symlink here): session-start (MANDATORY before writing code — anchor to a GitHub Issue), session-end (close-side bookend — runs *before* the `Closes #NNN` commit and lands it), firmware-loop (build/flash/monitor), protocol-change (wire-change checklist), status-sync (MANDATORY at ship time — docs move with the code), stack-electrical (jumpers/SBs/bus routing across the board stack), milestone-retro (post-milestone retrospective; see the self-improvement rule below), tof-scan-diagnosis (diagnose a wrong-looking scan against an imported ground-truth splat), issue-fleet (orchestrate several subagent workers across open issues, one worktree each, under a declared usage ceiling — `fleet_plan()`/`fleet_budget()` MCP tools; workers commit `Refs #NNN` on their own branch, the orchestrator lands everything; rotation is automated by `host/tools/fleet_run.py`, a CLI the owner launches once that chains successor sessions off `.fleet/<run-id>.md` — deliberately not an MCP tool, since it must outlive the session it rotates). Also present: three vendored third-party web-design skills (design-taste-frontend, high-end-visual-design, redesign-existing-projects; tracked in skills-lock.json) — generic, not project conventions; the web UI's own design language wins on conflict
├─ docs/                   ← index in `docs/README.md`. `engineering-practices.md` is BINDING; `superpowers/{plans,specs}/` holds implementation plans + design specs, lifecycle register in ROADMAP.md
├─ firmware/               ← our firmware forks + `vendor/` (53L9A1, stsw-img053, tinyusb, lwip) — every vendor package is READ-ONLY reference, never edited in place
├─ host/                   ← PC Python package `roomscan` (src/, tests/, tools/, transform/)
└─ references/
   ├─ roadmapResearch.md                     ← architecture design + critical review
   └─ 3D Mapping Architecture Evaluation.md
```

Follow `docs/engineering-practices.md` for all work here. Known bugs in the reference firmware (do not
inherit them into forks) are catalogued in `ROADMAP.md` → "Reference-firmware bugs". Note the vendored `53L9A1/`
package ships **no USB middleware** (`Middlewares/ST/` = media-object + vl53l9-transform-c only) — USB CDC
work vendors TinyUSB (see the Phase 1 plan). **Two vendor packages now carry `vl53l9-transform-c`** —
53L9A1 has 1.3.1, stsw-img053 has 1.5.0 — and `host/transform/CMakeLists.txt` picks between them with
`RS_TRANSFORM_PKG` (default: the newer). It also pins **`-DVL53L9_TRANSFORM_LIGHT=0`**, which is
load-bearing: the vendor header self-defines that macro to `1`, and under 1.5.0 LIGHT silently bypasses
TNR and the flying-pixel filter. Never assume a vendor build flag is off because you did not pass `-D`.
Re-run `host/tests/compare_transform_versions.py` on the next vendor drop; background in
`docs/transform-streams.md` → "Library upgrade 1.3.1 → 1.5.0".

**Issue-anchored sessions (owner, 2026-08-11):** every session that writes code must run the
`session-start` skill **before the first edit** — find or create the governing GitHub Issue, check
for in-progress conflicts in the same area, post a session-start comment, and emit the commit-prefix
template (`Refs #NNN`). The close-side bookend is the `session-end` skill: it records the session's
memory (to auto memory **and** as a comment on the issue), applies its self-improvements, then lands
the closing commit and posts the outcome comment (via the `status-sync` checklist it runs). It runs
**before** you write the `Closes #NNN` commit, not after — committing first strands every
improvement in a trailing commit `status-sync` never sees (#169). A `Stop`-hook backstop
(`.claude/hooks/session-end-guard.sh`) catches a closing commit that landed without it. The `gh issue`
subcommands the skills use (`create`, `comment`, `close`, `edit`, `list`, `view`, `reopen`, plus
`gh label list`) are unblocked in auto-mode (`.claude/settings.local.json`).

**Nothing closes on unverified work (owner, 2026-08-12, issue #173):** before any `gh issue close`
or `Closes #NNN`, run the `operator-request` skill's close-or-hold table — *what would prove this is
fixed, and did I actually run it, today, against real data?* If the acceptance needs a capture that
does not exist, a path this host cannot exercise (USB CDC is dead here), an observable not stored in
the capture file, or a human's eyes, the issue **stays open** with `needs/operator` + a subtype
(`capture`/`network`/`hardware`/`eyes`/`decision`), paired with `status/fix-unverified` or
`status/blocked`. The same skill writes the owner-facing runbook — plain language, strictly
alternating `[Claude]`/`[You]` steps composed from its step library, posted as an issue comment —
and processes the result when it comes back. `operator_queue()` answers "what do you need from me?"
in one call, and `operator_page()` answers the follow-up the owner actually cares about — *how few
trips is that?* — by scraping every runbook into a generated `/static/operator.html`: free riders
resolved, shared power-ups hoisted, same-venue issues merged into one setup. `status-sync`,
`session-end`, `session-start` and `issue-fleet` all route through it;
`host/tests/test_operator_skill.py` pins that wiring, because a gate nobody consults is worth
nothing (`status/fix-unverified` sat unused in the tracker for weeks). **A candid "not verified on
hardware" paragraph inside a closing comment is not a hold** — the retroactive audit (#174) reopened
#57, #168 and #171, each closed with an accurate note naming the exact unrun check, one of them
spelling out the command for a later session to run. Prose carries no label, does not reach
`operator_queue()`, and does not stop `Closes #NNN`; if you are writing that paragraph you have
already made the judgement, so change the keyword and the labels too.

**Self-improvement rule (owner, 2026-07-08):** after every milestone (phase completion / major merge),
run the `milestone-retro` skill BEFORE starting the next phase — convert the push's friction into
skills (with references/scripts), shared tools under `host/tools/`, and doc fixes. A milestone isn't
done until the next one got easier.

**Agentic firmware loop (owner, 2026-07-10):** this is an agentic project — **Claude reads/writes firmware
and drives the full build → flash → observe → diagnose loop itself**, it does not write up "bench steps"
for a human to run. The toolchain, `STM32_Programmer_CLI`, `capture.py` (native CDC), ST-Link VCOM, and
on-target SWD register reads (`-r32 <addr>`, addresses from the `.map`) are all Claude's to use — see the
`firmware-loop` skill and `docs/engineering-practices.md` → Firmware. The human is asked **only** for
physical actions Claude cannot perform: moving IKS4A1/53L9A1 jumpers & solder bridges, scope probing, and
power-cycling (USB replug) to clear a warm-wedged I3C bus. Diagnose in firmware first; escalate to the
human only for a genuinely physical cause, and name the exact physical action.

**New tools land in the MCP server (2026-07-29):** the agent-facing surface is `roomscan-mcp`
(`host/src/roomscan/mcp_server/`, registered in `.mcp.json`) — typed tools returning structured
data, documented in `docs/mcp-server.md`. **Any new agent-facing capability lands as an MCP tool,
not only as a script under `host/tools/`.** Write the logic as a pure function returning a dict,
register a thin wrapper (its **docstring is the description the agent sees**), and add a CLI front
end only if a human will run it directly — one implementation, two front ends. If a script is
deliberately *not* exposed, record it in `EXCLUDED` in `host/tests/test_mcp_registry.py` with the
reason; that test fails on any script which is neither exposed nor excluded. Two invariants: the
server never binds the device stream (`roomscan-web` owns it — recording goes through
`rig_record()`), and every tool reports what actually happened rather than what was requested.

**New dependencies are allowed (owner, 2026-07-31):** *"we are okay with installing new dependencies
if there is a material benefit."* **Do not self-impose a stdlib-only constraint, and never substitute
a proxy for the library whose performance is the actual question.** (Said after an investigation was
told to skip `zstd`/`lz4` if absent and approximate LZ4 with "zlib level 1" — that would have based a
firmware decision on a stand-in number.) Install the real thing and measure it. The dependency's cost
is a genuine input to the recommendation, so *report* it — footprint, build/runtime burden, whether it
is even shippable on the target — rather than treating it as a precondition that rules out the best
option before anything is measured. Weigh host and firmware separately: something unremarkable in
`host/` (Python, ample CPU/RAM) may be unshippable on the Cortex-M33, where RAM footprint beats ratio.
If a real implementation genuinely cannot be obtained, say so plainly instead of proxying.

The reference firmware in `firmware/vendor/53L9A1/` (bare-metal, STM32H563ZI + X-NUCLEO-53L9A1,
no unit tests, errors spin forever, never hand-edit generated CubeMX init) has its own scoped
`firmware/vendor/53L9A1/CLAUDE.md` — read it before touching anything under that path.

## Target architecture (where this is going)

Two decisions that override the older parts of `references/roadmapResearch.md`:
- **Transport: native USB CDC OR Ethernet UDP (Phase 5).** The device streams over either USB CDC or Ethernet (UDP unicast). If Ethernet is plugged in, the device acts as a DHCP client (or falls back to a self-assigned IP server) and streams via UDP. This removes the USB cable length limit and prepares the plumbing for Phase 6's hardware time-sync (PTP). USB CDC remains supported as an automatic fallback.
- **Sensors: X-NUCLEO-IKS4A1** — **integrated (Phase 4, 2026-07-10)**: the LSM6DSV16X shares I3C1 with the ToF as a native I3C target (HUB1-only jumpering, PartID-keyed multi-device ENTDAA, slow-PP workaround for the NXS0108 translator); SFLP orientation quaternion = stream 9, sensor-hub env (baro/mag/temp) = stream 10, both one sample per ToF frame; host panel shows gizmo/compass/sparklines and runs 9-axis mag yaw fusion (`docs/yaw-fusion.md`). Full stack streams at 27.85 fps, 0 CRC. Stacking recipe + bus-conflict resolution history in `docs/iks4a1-stacking.md`. On-rig mag calibration + `AXIS_CONVENTION` check completed 2026-07-10 (BUG-004: `mag_cal.json`, ~~`AXIS_CONVENTION = diag(1,-1,-1)`~~ — **BUG-059, 2026-08-02: that check used |B|, which all 48 signed permutations preserve, so it verified nothing; the field vector was ANTI-PARALLEL and every heading was 180° out (aimed north, read south) for three weeks. Now `MAG_FIELD_SIGN * MAG_MOUNT_ROTATION` = `-1 * diag(1,-1,-1)`, det −1 deliberately. Verify with the DIP — the world-frame field must point DOWN — never with |B|**). *(2026-07-28 orientation-noise pass, BUG-027: the SFLP quat was being decimated 480 Hz → 30 Hz by keeping one sample of ~16 — unfiltered, so the whole noise band aliased in. Firmware now averages the FIFO batch, enables the gyro LPF1 at 28.4 Hz, and sets LIS2MDL `CFG_REG_B = OFF_CANC|LPF`: **2.8× less orientation noise**, 0.0329 → 0.0118 deg/frame, streams 7/9/10 at 30.3 Hz / 0 drops / 0 gaps. The floor is now the SFLP FIFO's **fp16** encoding (~0.056°/step), not the sensor — see `docs/iks4a1-stacking.md` → "Orientation-noise pass".)* ~~Still open: SHT40 humidity unstreamed; beating the fp16 floor (batch raw XL/GY, fuse host-side).~~
*(2026-07-29: the fp16-floor item is **superseded** — raw XL/GY now ship as **stream 11** and a host
complementary filter `roomscan.imufusion` exists but is **gated off**; the floor also turned out to be
**dither**-limited, not step-limited, so a quieter board measures worse. More importantly the visible
noise was the **eCompass**, traced to a direction-dependent magnetometer calibration — ~~**BUG-030**,
the top open item~~ **BUG-030, closed 2026-07-30**: the owner re-fit hand-held off the tripod and it
validated on an independent room sweep (attitude-locked error 0.56%, tilt ramp 1.042×, `YawFusion`
`gated:anomaly` 58.6% → 0%, `active` 6.2% → 64.8%). Two traps learned there, now encoded in
`host/tools/mag_check.py` / the `capture_magcheck` MCP tool: **the live calibration is `./mag_cal.json`
at the repo root** (`mag_cal_path` is cwd-relative — a stale `host/` copy shadowed it for two weeks and
is now deleted), and **raw |B| spread is not calibration error on a moving capture** — indoor ambient
field varies ~±6% with position (BUG-034), so detrend before judging a fit. Full state + resume
instructions: `docs/superpowers/plans/2026-07-29-orientation-resume.md`.)* Still open: SHT40 humidity
unstreamed; **heading *direction*** remains unvalidated — |B| flatness cannot see DT0103's rotation
ambiguity, so it needs a braced fixed-heading tilt sweep (resume doc §4.6).

### Roadmap

Forward-looking work and open defects are **GitHub Issues** (2026-08-10) —
`gh issue list --repo hellosamblack/lidar-roomscanner --label bug|work-item|data-collection`;
completed-phase narratives and measured outcomes are in `docs/roadmap-history.md` (keeping their
`Phase N` names), and standing decisions, the reference-firmware bug ledger, risks and the plans
register are in `ROADMAP.md`. Those three are authoritative for current status — read them rather
than trusting the one-line arc below, which is only here to orient a fresh session:

- Phases 0–5 are **✅ done** — on-device transform (0) → binary frame protocol + live visualizer (1)
  → raw streaming with the transform moved host-side (2/2.5) → UI, runtime config, recording and the
  `roomscan-web` primary UI (3/3.5 + the 7-phase web program that retired `panel.py`) → X-NUCLEO-IKS4A1
  sensors, streams 9–13 (4) → Ethernet/UDP transport, untethered (5).
- **Phase 6 — real-time SLAM on PC: in progress.** Point-to-plane frame-to-model ICP against a TSDF
  raycast, SFLP rotation prior, bounded-authority baro Z. 6.G (GPU memory) done; 6.D (drift /
  loop closure) and 6.I (mesh-extraction ceiling) open. Open3D has no tensor G-ICP.
- **Phase 7 — offline 3DGS: partly shipped.** The `roomscan.splat` video→splat pipeline works today;
  ToF-seeded fusion and the ARCore capture app (OFFLINE-4) are the open half.

### Hard-won rules

Each cost a multi-day push, and unlike the narratives around them these sentences live **nowhere else**
in the repo. The evidence for every one is in `docs/roadmap-history.md`.

- **No scalar "yaw" off an attitude is the bearing of an axis** — ask for the bearing of the axis you
  actually mean. Four separate live-path defects were this same error (BUG-039, BUG-048, BUG-051,
  BUG-058), two of them in shipped code; the AST guard `test_no_new_yaw_twist_consumers` pins it.
- **Score ensembles, not single runs.** SLAM runs are chaotic: a deliberate 3 mm height nudge moves
  final height error by 146 mm and loop closure by 0.37 m.
- **Always cost a mitigation against the failure it prevents** (BUG-052: a workaround held the GIL for
  1.11 s to avoid a leak that cost 0.04 s to handle in place).
- **A native call's wall time is not its blocking cost** — numpy releases the GIL, Open3D C++ does not.
  Measure with `host/tools/slam_stall_profile.py` / MCP `slam_stall_profile`, and judge starvation by
  `tick_share`, never by summed tick-lateness (which reads ~0 even when blocking is total).
- **When every upstream stage measures healthy, stop bisecting upstream** and ask what is drawn in
  front of the geometry — project its bounding box to screen coordinates and intersect it with the DOM
  (BUG-064: two cards covered 97% of the map).
- **"No backpressure" is a property of the stack, not something a rate governor fixes** — a governor
  metered against an assumed drain rate does not bound a queue, and ordering matters on an ordered
  transport (BUG-061: pose JSON head-of-line blocked behind a mesh backlog).
- **Connection count is a performance variable on this server.** `/ws` work is per-client; stale
  headless/MCP sockets are real load (BUG-060).
- **Never assume a vendor default, in either direction, and it is not a C-only trap.** Vendor headers
  self-define via `#ifndef` — `VL53L9_TRANSFORM_LIGHT` defaults to `1` and silently bypasses TNR and
  the flying-pixel filter, so not passing `-D` selected the LIGHT build. The mirror image cost four
  days on the web side (#106): `gaussian-splats-3d` gates its **entire** per-scene opacity path behind
  a constructor option `enableOptionalEffects` that defaults to **`false`**, so the See-Through slider
  set `scene.opacity` on every move and the uniform was never even declared. An `#ifndef` default
  (surprise ON) and an opt-in default (surprise OFF) are the same mistake. Read the vendor's own
  default; never infer it from your call site looking correct. Not code-only either: the Pi bridge's
  silent full-network drops (#200) trace to `raspberrypi-sys-mods` shipping
  `/usr/lib/systemd/system.conf.d/40-rpi-enable-watchdog.conf` (`RuntimeWatchdogSec=1m`) — a 1-minute
  hardware watchdog that hard-resets the SoC with zero chance to log if PID 1 stalls that long.
  `/etc/systemd/system.conf`, the file a human checks, shows the setting **commented out**, reading
  as "off"; the drop-in overrides it silently. A config file's own silence proves nothing about a
  package-shipped default living somewhere else — check `/usr/lib/*.conf.d/` and `dpkg -S` before
  trusting it. Same trap a third time, same box (#204): NetworkManager's per-device-type
  `route-metric` default is 100 for ethernet vs. 600 for wifi, unstated anywhere in any config file
  — so the moment a USB debug NIC DHCPs onto the same subnet as `wlan0`, it silently outranks WiFi
  for the default route *and* the on-link subnet route (not just `0.0.0.0/0`), meaning even reply
  traffic for a socket bound to the WiFi address could leave over the debug cable. The stream
  looked healthy throughout, because inbound traffic is destination-IP-pinned by ARP; only the
  reply path was ever at risk. Fixed with an explicit `route-metric` + `never-default` profile
  rather than trusting NM to leave WiFi as primary.
- **Some failures abort the process instead of raising — you cannot handle those, only make
  them unreachable.** Open3D/Filament's `OffscreenRenderer` kills the interpreter outright
  (`utils::PreconditionPanic`, `terminate called`) on a *second* instance in one process, and
  on any call from a thread other than the one that created it; a live one collected at
  interpreter exit ends the run with `pure virtual method called`. No `try/except` reaches any
  of these. When a native library documents a precondition, the design — a singleton, an
  owning thread, an explicit teardown on that thread — *is* the error handling. Spike the
  failure modes before building on a native handle, not just the happy path (#194).
- **An invariant-preserving check verifies nothing.** |B| is preserved by all 48 signed axis
  permutations, so a magnetometer check "passed" for three weeks while every heading was 180° out
  (BUG-059). Ask which wrong answers a check can actually see before trusting it.
- **A check that reads back the value it just wrote is that same failure, cheaply.** `splat.js`
  exposed a `sceneOpacity` getter commented *"to verify See-Through end-to-end"* that returned the
  property `applySeeThrough()` had assigned one line earlier — it reported `1` and `0.08` faithfully
  while the renderer ignored both (#106). A read-back proves the setter ran, nothing more. An
  end-to-end claim needs an observable on the **far side** of the layer in question; for a rendering
  path that means **pixels**, framed so a null result cannot be explained away (the first attempt had
  the camera inside the splat, where alpha saturation could have masked a real change).
- **A percentage of a path is meaningless when the path is invented.** `icp_mode="translation"` gives
  rotation-prior error no rotational DoF to land in, so it emerges as translation — 18–20 m of
  fabricated path at 0 lost frames and 0.88 fitness (BUG-067). Validate against a null capture before
  trusting any drift figure.
- **Assert an interval, not a type.** A recording-clock bug survived review because the test asserted
  the field was a `float` (BUG-050, which reported 1.78e9 s for a 90 s take).
- **A new default is invisible without a migration** — `_persist_ui` writes every field on any change,
  so a stale persisted value shadows a new default forever.
- **Know what your null actually nulls.** #155's wrong-direction control was built to void
  smoothing-flattered wins, but it shared the exact-group association fix with the treatment — so
  when it "passed", it was attributing the win to the shared component, not voiding the result.
  A null voids only what it does not share; write down what each control arm retains vs inverts
  before reading its verdict, and shape the arms so every pairwise gate answers one named component.
- **A permission denial's error message does not tell you why — verify a permission-scope fix
  live before trusting it.** Two successive "fixes" to `fleet_run.py`'s Bash allowlist were each
  wrong, guessed from a denial's tool name rather than tested: a directory-only rule
  (`Bash(host/.venv/bin/:*)`) that matched nothing at all, and a denial blamed on "the auto-mode
  classifier acting independently" that was actually simpler outside-repo-path scoping. Both
  shipped with passing unit tests, because the tests checked the guess's own logic, not the real
  CLI. A cheap, real `--task` probe (`permcheck-20260814`+, `docs/fleet-ledger.md`) settled both
  in minutes; six real runs and real dollars had been spent guessing first (#183).

Guiding order (per project owner): mature the visualizer and UI/config on the ToF sensor alone **before** adding the IKS4A1 board. *(Satisfied — both are done; Phase 6 SLAM should likewise be validated against recorded captures before hardware-in-the-loop.)*
