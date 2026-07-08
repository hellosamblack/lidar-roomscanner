# Engineering Practices — roomscanner

Conventions for all work in this workspace. CLAUDE.md points here; keep this doc short and binding.

## Repository rules

- `../53L9A1/` is **read-only reference**. Never edit it, even to fix known bugs (they're catalogued in
  `ROADMAP.md` → "Reference-firmware bugs"). Our firmware fork lives in `firmware/` and references the
  package's Drivers/Middlewares/Utilities in place via CMake paths.
- Layout: `firmware/` (STM32 apps), `host/` (PC Python package `roomscan`), `docs/` (specs, plans,
  captures), `references/` (imported research, read-only), `.claude/skills/` (project skills).
- Commit style: conventional-commit-ish prefixes (`feat:`, `fix:`, `docs:`, `test:`, `chore:`), small and
  frequent. Never commit `build/` output or captured binary streams >1 MB (put large captures in
  `captures/` — gitignored — and check in only the small golden fixtures under `host/tests/fixtures/`).

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
- Every capture-format consumer must also accept file replay (`FileSource`) — hardware-free development
  and regression datasets come for free.

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
