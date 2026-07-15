# Engineering Practices — roomscanner

Conventions for all work in this workspace. CLAUDE.md points here; keep this doc short and binding.

## Repository rules

- `firmware/vendor/53L9A1/` is **read-only reference** (vendored ST package). Never edit it, even to fix known bugs (they're catalogued in
  `ROADMAP.md` → "Reference-firmware bugs"). Our firmware fork lives in `firmware/scanner-stream/` and references the
  package's Drivers/Middlewares/Utilities in place via CMake paths (`PKG_ROOT`).
- Layout: `firmware/` (STM32 apps), `host/` (PC Python package `roomscan`), `docs/` (specs, plans,
  captures), `references/` (imported research, read-only), `.claude/skills/` (project skills).
- Commit style: conventional-commit-ish prefixes (`feat:`, `fix:`, `docs:`, `test:`, `chore:`), small and
  frequent. Never commit `build/` output or captured binary streams >1 MB (put large captures in
  `captures/` — gitignored — and check in only the small golden fixtures under `host/tests/fixtures/`).
- **Docs move with the code (status-sync rule).** Any commit/PR that completes a phase, clears a
  deferred item, changes a measured number, or invalidates a prediction updates `ROADMAP.md` (and
  `CLAUDE.md`/memory when phase status changes) **in the same PR** — follow the `status-sync` skill
  checklist. "Docs later" is how the 2026-07-10 drift happened.
- **Branch discipline.** Work rides worktree branches → draft PRs. Never commit to local `main`, never
  merge locally, never merge your own PR — merging is the owner's decision. Subagents don't commit;
  the controlling session does.
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

## Verification discipline

- Before claiming a milestone works: run the pytest suite, then the on-target check (flash, run the host
  tool, observe fps + zero CRC failures + zero seq gaps). State actual numbers, not "works".
- When debugging link problems, capture raw bytes first (`--record`), then debug offline against the
  file — don't iterate on live hardware.

## Self-improvement after milestones

- **After every milestone** (a phase completing, or any major merge to main), run a retrospective BEFORE
  starting the next phase — follow the `milestone-retro` skill. The question is always: *what would have
  made this push easier, done as a reusable artifact?*
- Convert findings into durable tooling, not notes: new/updated **skills** under `.claude/skills/` (with
  `references/` and `scripts/` subdirectories where they earn their keep), shared **scripts** under
  `host/tools/`, and corrections to existing docs. Follow superpowers:writing-skills conventions.
- Hard rules of thumb: any hardware ritual performed from prose by more than two subagents becomes a
  script; any environment fact discovered the hard way (tool paths, port quirks, timing windows) becomes
  a line in the relevant skill; any repeated review finding becomes a checklist item in the skill that
  governs that work.
- The retro's output is committed as part of closing the milestone — a milestone isn't done until the
  next one got easier.
