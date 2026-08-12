# #155 session — timestamped pose buffer: decision log + handoff

**Live working notes for the 2026-08-12 session implementing #155.** If you are a later session
picking this up: read this whole file before touching anything, then read issue #155's
implementation-plan comment (2026-08-12) — the design authority — and the approved plan at
`~/.claude/plans/work-github-issue-155-starry-yao.md` (validation campaign + decision rule).

## The original problem (keep this in view — every choice ties back here)

Stream 9's SFLP quat is a FIFO-batch mean; its midpoint sits **after** the depth frame-ready
edge (+7.76 ms measured on the golden capture; **+5.13 ms** re-measured on DebugCapF per #143 —
the phase is NOT constant, which is the whole argument for interpolation over a constant).
Orientation leads depth ~0.3° at 38.5°/s. The naive fix (#126: constant gyro rollback) made the
tripod capture WORSE and bistable (0.211 ± 0.353 vs 0.121 ± 0.069) and ships default-off.

## The trap (check every metric against this)

BUG-067's phase sweep (`slam/mapper.py:272-281`): **any low-pass of the prior collapses tripod
instability (sd 0.489 → ~0.03)** regardless of phase direction. Interpolating between two 30 Hz
samples IS a low-pass. So "tripod got better" ≠ "phase corrected". Note: the issue body
attributes this trap to the orientation resume doc; the passage is not there — the mapper
comment is the recorded evidence. Countermeasure: the **reflected** null arm (query at
`mid + (mid − frame_ready)`, i.e. the wrong side). If reflected also beats baseline, the metric
is measuring smoothing and the result is VOID no matter how good the real arm looks.

Also constraining interpretation:
- #81/BUG-070: stationary-capture displacement is bistable under physically null heading
  relabels (0.104 → 2.44 m). Single runs are meaningless; ensembles, and read the sd.
- #95/BUG-084: officeFullScanAug6.bin has a reported fork ~frame 979, NOT reproduced by the
  2026-08-12 fleet session (baro ruled out). Both A/B arms hit the same capture, so it stays
  usable, but flag #95 next to its numbers.
- "A % of a path is meaningless when the path is invented" (BUG-067): with
  icp_mode=translation, rotation-prior error emerges as translation. That is precisely the
  channel our fix should reduce on moving captures.

## Pre-registered decision rule (written BEFORE any ensemble ran)

Close-worthy only if ALL of:
1. interp-on beats baseline consistently across ≥3 genuinely moving stream-13 captures
   (`slam_ensemble` n=10 decision / n=5 triage, `horizontal_closure_m` mean ± sd, same config
   otherwise; `runs_died`/`any_saturated` checked);
2. interp-on ≥ ties baseline on `imuTranslationError.bin` (the capture #126's fix regressed);
3. the reflected arm does NOT also beat baseline (else: smoothing, void);
4. legacy (no stream 13) captures byte-identical with flag on; stationary nulls fabricate
   nothing.
Anything mixed → hold open with `needs/operator` (+ subtype) via operator-request; the DC-F
pan set (#143) remains the named discriminating capture if SLAM metrics can't discriminate.

## Decision log (append-only; tie each entry to the problem statement)

- **D1 (scope):** offline/recorded paths only, per the #155 plan comment. Live reader dispatches
  depth before the group's trailing 9/13 arrive; remote drops the aux. Live adoption is a
  follow-up issue, not silent scope growth.
- **D2 (domain):** interpolate in LSM ticks using stream 13's exact pairing; no TIM2↔LSM fit.
  `quat_mid_ticks` is +ms AFTER frame-ready, so frame N's target is bracketed by groups N−1 and
  N — no extrapolation on healthy data.
- **D3 (pairing):** exact `(seq, t_us)` group join of 9+13; never seq alone (decoupled mode
  freezes seq), never latest-sync. Invalid/`quat_n==0` syncs are not timed samples.
- **D4 (fallback):** any missing bracket / missing 13 / malformed payload → keep the legacy
  carried-forward quat for that frame and its legacy offset behavior. Never extrapolate.
- **D5 (no double correction):** when interpolation replaced a frame's quat, that frame's
  `imu_aux` offset becomes None so `Mapper.step()`'s fixed rollback cannot also run.
- **D6 (validation-only reflected knob):** lives in the loader + slam_ensemble only, NOT in
  SlamConfig — it exists to kill our own result, not to ship.

## Status ledger (update as you go)

- [x] session-start run; #155 claimed (`status/in-progress`), comment posted 2026-08-12
- [x] sensor_time.py + tests (18 tests; slerp moved here, slam/frames re-exports)
- [x] protocol.py wrap-safe ImuSync.quat_offset_us (golden +7778.7 µs preserved; rollover tests)
- [x] cli.py collect-then-align loader (`_load_frames(quat_interp=)`, `_align_quats_to_frame_time`,
      `_ImuAuxList.interp_stats`, max_frames trailing-group drain) + 8 integration tests.
      **D7:** span guard `_INTERP_MAX_SPAN_US = 150 ms` (~4 lost frames) — bridging wider holes
      is smoothing, not phase correction. **D8:** per-frame fallback — a frame whose own group
      lacks a valid 13 keeps the legacy carried quat AND legacy fixed-rollback offset; coverage
      counters expose the mix. **D9 (compat):** `_load_frames_maybe_imu` passes `quat_interp`
      only when the lever is on, so legacy test doubles without the kwarg keep working.
- [x] mapper no-double-correct pinned (offset=None → no rollback; ZUPT unaffected) — no mapper
      code change needed, loader guarantees the None
- [x] slam_ensemble: `--quat-interp-mode reflected` (forces lever for arm parity), `quat_interp`
      stats in JSON + report, loud zero-coverage warning; CLI prints/reports coverage too
- [ ] full test suite + ruff green
- [ ] validation campaign (fill table below)
- [ ] docs (yaw-fusion / rtabmap-study / resume doc)
- [ ] operator-request close-or-hold on FULL evidence
- [ ] #126 fold-in decision
- [ ] session-end BEFORE closing commit

## Validation campaign — capture classification (capture_motion, 2026-08-12)

**Surprise:** several "moving" candidates are actually stationary. Measured, not assumed:
- **Moving A/B set:** `imuTranslationError.bin` (3 takes, mean 17°/s, p95 51 — NOT purely a
  tripod; #126's "tripod" was shorthand), `officeFullScanAug6.bin` (mean 21°/s, p95 48; #95
  caveat), `DebugCapF.bin` (#143 controlled pans 19–89°/s, long holds), `DebugCapC.bin`
  (mean 38°/s sustained — the exact rate BUG-031's 0.30° error is quoted at; highest dose),
  `NorthFacingRoll.bin` (short, 1 take, p95 70°/s).
- **Low-dose:** `web_20260801_225759.bin` (71 s hold + 1 take, mean 2.7°/s).
- **Stationary null (gate 4):** `web_20260803_121735.bin` (moving_frac 0.0, max 3°/s).
  `web_20260802_131231/200947.bin` also stationary — excluded, one null suffices.
- **Legacy no-op (gate 4): PASS** — tilt_sweep_20260729.bin (no stream 13), 3088 frames
  byte-identical with lever on; interp_stats {timed_samples: 0, applied: 0}.

Smoke (real data): tripod n=1 max-frames=300 → on: 300/300 aligned, 322 samples, h=0.046;
reflected: 299/300, h=0.048. Arms distinguishable, coverage ~100%.

Runner: `/tmp/155/run_campaign.sh` → `/tmp/155/<tag>_{base,on,refl}.json`, log
`/tmp/155/campaign.log`; addendum `/tmp/155/run_campaign2.sh` (DebugCapC) after it.
Tags: tripod, office, panset (DebugCapF), web0803 (stationary null), roll, web0801, capc.

## Validation results (fill in; mean ± sd horizontal_closure_m; n=10 CUDA:0; check died/saturated)

| capture | baseline | interp on | reflected | verdict/notes |
|---|---|---|---|---|
| imuTranslationError | | | | #126 fixed-offset made this 0.211±0.353 vs 0.121±0.069 |
| officeFullScanAug6 | | | | #95 fork caveat |
| DebugCapF (panset) | | | | #143; phase re-measured +5.13ms here |
| DebugCapC (38°/s) | | | | highest dose |
| NorthFacingRoll | | | | short |
| web_20260801_225759 | | | | low dose |
| web_20260803_121735 | | | | STATIONARY NULL — interp must change ~nothing |

## Side-findings to record before session end

- `capture_list`'s stream survey returns `streams: {}` on every capture newer than
  ~2026-08-03 16:00; `capture_analyze` decodes the same files cleanly. File as a bug
  (area/host-tools) — mirrored-constants class.
- `ImuFusion._tick_span_us`/`_sample_dts` ignore stream-12 `tick_us` (use the nominal constant)
  — latent ~3% clock error if #127 ever enables it. Note on #127.
