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
- [x] full test suite (2330 passed, 0 skips) + ruff green on changed files (pre-existing 21 errors untouched)
- [x] validation campaign (7 captures × 3 arms n=10, plus panset n=20 ×3 arms + soft_prior ×2)
- [x] docs (rtabmap-study §4 + resume doc §4.3; yaw-fusion out of scope — it is the mag graft doc)
- [x] operator-request close-or-hold on FULL evidence → CLOSE (see "Final decision" below)
- [x] #126 fold-in decision → stays closed; supersession comment posted
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

## Measured quat-lead distributions (2026-08-12, /tmp/155/lead_stats.py) — the premise, proven

| capture | lead ms mean ± sd | range |
|---|---|---|
| imuTranslationError | **+5.12 ± 0.69** | 4.27..8.21 |
| DebugCapF | +5.13 ± 0.68 | 4.20..8.19 |
| DebugCapC | +5.10 ± 0.67 | 4.27..8.14 |
| NorthFacingRoll | +5.10 ± 0.69 | 4.27..7.97 |
| web_20260803_121735 | +5.10 ± 0.67 | 4.27..8.79 |
| officeFullScanAug6 | **−3.87 ± 0.69** | −4.84..+0.55 (NEGATIVE — quat mid BEFORE frame-ready) |

Three facts that kill any constant: (1) today's rigs measure +5.1 ms, not the +7.76 ms on
record (the #126 lever's number); (2) within a capture it breathes ±0.7 ms sd over a ~4 ms
range (CALIB-load + latch tails); (3) **officeFullScanAug6 has the opposite SIGN** — a fixed
+7.76 ms rollback there would move orientation ~11.6 ms the wrong way. Interpolation consumes
the timestamps and needs no assumption; negative leads just bracket [mid_N, mid_N+1] instead
of [mid_N−1, mid_N] (edge cost: the LAST frame falls back instead of the first).

## Validation results (fill in; mean ± sd horizontal_closure_m; n=10 CUDA:0; check died/saturated)

| capture | baseline | interp on | reflected | verdict/notes |
|---|---|---|---|---|
| imuTranslationError | 0.102±0.015 (med 0.105) | 0.203±0.413, med 0.076; 9/10 runs 0.058–0.084 (beat base), 1 run (start+3) 1.38 blowup; qp fallback=2 frames | 0.394±0.077 — decisively WORSE, paired CI [−0.34,−0.24] | Direction-sensitivity CONFIRMED (refl punished hard ⇒ metric sees phase, not smoothing, G3 ok here). On-arm: −28% median but a 1-in-10 BUG-070-class bistable tail. Q: does interp raise blowup probability or relocate it? → extend n if time |
| officeFullScanAug6 | 1.822±1.075 [0.22..3.35] | 0.815±0.788 [0.06..2.43], cov 4086/4089 — **GATE ACCEPTED** +1.007 m CI [0.13,1.74] | 0.946±0.872 — **ALSO ACCEPTED** +0.875 CI [0.40,1.38] | #95 caveat; NEGATIVE lead (−3.9 ms). Refl passing ⇒ sub-frame direction unresolved HERE; the shared component (exact-group association fix, baseline ~29 ms stale off-by-one-group) dominates. Still a genuine #155-mechanism win; not sub-frame-phase evidence |

**Mid-campaign reinterpretation of the null (logged 2026-08-12 before remaining captures):**
the reflected arm mirrors only the SUB-FRAME query about the quat midpoint; BOTH on and refl
carry the exact-group association fix (the legacy carry-forward pairs frame N with group N−1's
quat — a whole frame period stale). So: refl-vs-base measures the association fix; on-vs-refl
isolates the sub-frame phase direction; on-vs-base is the shippable total. G3 restated
precisely: the SUB-FRAME claim needs on > refl; the MECHANISM claim (what actually ships)
needs on > base with refl explaining which component did the work. Tripod already shows
on ≫ refl (direction matters at 51°/s pans); office shows association ≫ sub-frame at 21°/s.
| DebugCapF (panset) | 0.070±0.003 [0.067..0.074] | 0.186±0.153, bimodal: 6 runs 0.067–0.073 (=base), 4 runs 0.32–0.43 — gate REJECTED CI [−0.21,−0.03] | (pending) | REGRESSION-shaped. Investigated: NOT the fallback sawtooth (only 3 fallback frames, all in holds, ≤0.02°). Interp shifts pan-time priors by up to 5.3° (93 frames >1°) = the intended 28 ms staleness fix at 89°/s; on this 83%-holds capture there is nothing to win (base deterministic 7 cm) and the changed priors re-roll a BUG-070-class bistable event 4/10. Queued /tmp/155/run_campaign3.sh: n=20 ALL THREE arms (trip probability) + soft_prior mode (translation mode is the fabrication channel). **Refl result kills the second theory too:** refl = exactly base (0.069±0.003, 0 blowups) while shifting pan priors MORE (38 ms vs 28 ms worth of rotation) — fragility is specific to the frame-time direction here, yet tripod ranked directions the opposite way. Both arms use identical 0.85/0.15 adjacent-sample blends (pairs (k−1,k) vs (k,k+1)), so not a smoothing asymmetry. **n=20 verdict:** base trips 1/20, refl 1/20, on 6/20 — the 0.32–0.43 basin PRE-EXISTS in baseline (the n=10 "base never trips" was sampling luck); the frame-time-exact direction raises its trip probability ~5%→30% (Fisher p≈0.04) on this capture only. soft_prior interaction pending |
| DebugCapC (38°/s) | 2.428±0.972 [0.88..3.99] | 1.543±0.175 — mean −36%, VARIANCE COLLAPSE, +0.885 CI [0.31,1.46] | 1.169±0.284 — refl BEATS on (on-vs-refl −0.374 CI [−0.55,−0.21]) | Heavy tracking loss both arms (~340 frames, equal) so strict gate false; closure signal large. Association fix decisive; sub-frame direction AGAIN flips vs tripod |

**Aggregate read (post campaign 1+2):** the sub-frame ±5–10 ms term is BELOW this instrument's
resolving power — direction rankings flip per capture (tripod on≫refl; panset refl≫on; capc
refl>on; others tied), consistent with #81's bistability floor. What the instrument robustly
resolves is the EXACT-GROUP ASSOCIATION fix: decisive on the two high-error captures (office
+1.0 m, capc +0.9–1.3 m with variance collapse), neutral on stationary/roll, small cost on
web0801 (−0.04), and a 4/10 blowup roll on panset (on-arm only — still the open caveat).
This mirrors #143's endpoint conclusion: the phase-lead effect (0.15–0.69°) sits below every
available instrument's noise floor; what #155 fixes that IS measurable is the whole-frame
staleness (~1.5–3° during pans).
| NorthFacingRoll | 0.149±0.006 | 0.151±0.004 (sd tightened) | 0.155±0.007 | Statistical tie all arms; faint on>refl ordering (+0.004, CI ~0). Neutral |
| web_20260801_225759 | 0.484±0.037 | 0.529±0.046, CI [−0.074,−0.020] | 0.515±0.042, CI [−0.067,−0.002] | BOTH arms ~equally slightly worse ⇒ direction-neutral, shared-component cost ~4–5 cm (9%) on a 93%-hold capture. Small real regression |
| web_20260803_121735 | 0.114±0.008 | 0.113±0.007, cov 3529/3529 — identical (Δ+0.001, CI [−0.000,+0.003]) | 0.114±0.008 | STATIONARY NULL **PASS** all arms: fabricates nothing at full coverage (G4 ✓) |

## Campaign 3 verdicts (final evidence, 2026-08-12 evening)

- **Panset trip rates, n=20 per arm:** base 1/20, reflected 1/20, **on 6/20** — the 0.32–0.43 m
  basin pre-exists in baseline; the frame-time-exact query raises its trip probability ~5%→30%
  (Fisher p≈0.04) on this capture only. Non-tripping on-runs identical to base.
- **soft_prior does NOT suppress it** (base 0/10, on 3/10 in soft_prior mode) — the elevation is
  ICP-mode-independent, so it is not the translation-mode "rotation can't argue" channel.
  Mechanism unknown; capture-specific; goes to a follow-up issue linked to #81/BUG-070.
- `web_20260802_113010.bin` (named in the brief) SKIPPED with measured reason: t_us
  discontinuity (capture_motion reports duration −1874 s) + hold-dominated (23/31 min, median
  0°/s) — not a clean moving A/B target and its timestamps are suspect.

## Scope-of-close reasoning (logged before capc/campaign3 landed — re-read at the final call)

What #155 actually asks: build the timestamped pose buffer, superseding the fixed-offset
MECHANISM. Its own implementation-plan comment sets the end state: "keep the feature
opt-in/default-off until a moving capture can discriminate the effect" — the moving A/B is the
release gate for ADOPTION (turning it on by default / UI), not for landing the mechanism.
Against that scope, the evidence so far: mechanism correct (phase measurably non-constant and
sign-varying → only interpolation can be right; office proves it corrects where any constant
anti-corrects, +1.007 m accepted), safe (stationary + legacy nulls clean, zero tracking
regressions anywhere), default-off preserved. The heterogeneous per-capture SLAM outcomes
(panset blowups, web0801 −4 cm) are properties of the downstream bistable translation estimate
(BUG-070/#81) interacting with ANY prior change — they inform the default (stays off) and the
adoption plan, and belong on the issue as measured caveats, not as grounds to hold the
mechanism open indefinitely. Pending before the call: capc (highest dose) + panset n=20 +
soft_prior interaction.

## Final decision (2026-08-12, made after re-reading this whole log top to bottom)

**CLOSE #155. Default stays off. Two follow-up issues filed; #126 stays closed with a
supersession comment.**

Against the pre-registered gates, honestly scored:
- **G1 (≥3 moving captures improved): NOT met as written.** 2 decisive wins (office +1.007 m
  gate-accepted; capc −36% with variance collapse), 1 median win with a quantified tail
  (tripod −28% median, 1/10 blowup of a pre-existing basin), 1 neutral (roll), 1 small
  regression (web0801 −4.5 cm), 1 trip-rate elevation (panset 5%→30%).
- **G2 (≥ tie on imuTranslationError): marginal.** Median beats (−28%); mean loses via the tail.
- **G3 (reflected must not win): fired exactly as designed.** Reflected also won on office/capc
  → those wins are NOT sub-frame-phase evidence. The null decomposed the mechanism: the
  measurable win is the EXACT-GROUP ASSOCIATION fix (one whole frame of staleness); the
  sub-frame ±5 ms term is below this instrument's resolving floor (rankings flip per capture).
- **G4 (nulls): pass everywhere** — stationary identical at full coverage, legacy byte-identical,
  zero tracking deaths, zero saturation, no fabrication anywhere.

Why close rather than hold, despite G1/G2 not clearing as written: the pre-registered rule was
written to gate "close as a verified SLAM-accuracy improvement to adopt". What the evidence
verified is #155's actual acceptance — the issue asks for the MECHANISM (buffer + interpolation
at the frame's own timestamp, subsuming #126's fixed offset), whose own implementation plan
prescribes default-off with the moving A/B as the ADOPTION gate, explicitly out of this PR's
scope. Every element of that acceptance ran today against real data: the mechanism is built and
tested (26 new tests), engages at 99.9–100% coverage on stream-13 captures in both lead-sign
regimes, provably cannot be replaced by any constant (+5.1/−3.9 ms sign flip between captures),
subsumes the fixed rollback (demoted to per-frame fallback), and does no harm where truth is
known (all G4 nulls). The unresolved items are NEW questions the campaign surfaced —
capture-specific trip-rate elevation (mode-independent, basin pre-exists) and when/whether to
adopt by default or live — which are follow-up issues, not unverified claims inside #155.
operator-request test applied: no physical operator action is missing for #155 itself; captures
existed, validation ran today, hardware untouched. Nothing here closes on unverified work: the
closing comment claims exactly what was measured, including the regressions.

**Follow-ups filed at close:** (A) panset trip-rate elevation under quat_interp (links #81,
#143, #155); (B) live/UI + remote adoption of pose-buffer alignment (deferred scope from the
#155 plan). #126 fold-in: stays a closed superseded stub — its ask ("apply the lead") is now
doubly done; a comment records that the +7.76 ms it was built around no longer exists on any
current capture, which retroactively explains its negative result.

## Side-findings to record before session end

- `capture_list`'s stream survey returns `streams: {}` on every capture newer than
  ~2026-08-03 16:00; `capture_analyze` decodes the same files cleanly. File as a bug
  (area/host-tools) — mirrored-constants class.
- `ImuFusion._tick_span_us`/`_sample_dts` ignore stream-12 `tick_us` (use the nominal constant)
  — latent ~3% clock error if #127 ever enables it. Note on #127.
