# ODR-triggered IMU sync vs. SFLP — costing note (2026-07-30)

BUG-031 offers a hardware fix for the ToF↔IMU timing skew: ST's **ODR-triggered mode**, which
phase-locks the LSM6DSV16X's data generation to an external reference on **INT2**. AN5763 §3.3 makes
it mutually exclusive with **SFLP**, so taking it means giving up stream 9 (the game-rotation
quaternion `Mapper` uses as its rotation prior) and fusing orientation on the host from stream 11.

**The gating precondition is therefore: does `roomscan.imufusion` actually beat SFLP?** This note
answers that from recorded captures, then costs the hardware change.

Sources trusted: AN5763 Rev 4 §3.3 + Table 15 (`pdftotext` of the PDF, not the derived `.md`),
UM3239 Rev 5 Table 4 (IKS4A1 Arduino connector map), X-NUCLEO-53L9A1 schematic, STM32H563
datasheet AF table, and eight captures in `captures/`.

---

## Verdict

**Keep SFLP. Do not take ODR-triggered mode.** Three independent reasons, in descending weight:

1. **`imufusion` does not currently beat SFLP — it has a broken heading loop.** Measured against
   SFLP on a stationary capture its heading sits **1.69° mean / 2.22° p95** off and stays there; a
   shorter `tau_yaw` does not move it (1.70° at τ = 0.3 s). Cause found: `_correct_yaw` measures
   error with `quat_yaw_deg`, which is **ZYX yaw about body Z**, and this device's body **X is Up** —
   on that capture the ZYX pitch is **86.2°, i.e. 3.8° from gimbal lock**. Substituting a world-Z
   heading error term drops the same number to **0.017° mean / 0.053° p95 (100×)**. So the filter is
   *incomplete*, not merely untuned, and the only shipped evidence that it beats fp16
   (`test_fused_is_quieter_than_fp16_quat_path`) is synthetic.
   > **Fixed 2026-07-30 (BUG-039).** `_correct_yaw` now uses `sensors.graft_yaw_error_deg`, the
   > world-Z swing-twist of the residual — `graft_yaw`'s exact inverse. The substitution this
   > section predicted was confirmed to the digit (stationary 1.689/2.218 → **0.017/0.053**) and
   > extended to a seven-capture ensemble, in `BUGS.md` → BUG-039. **This reason for the verdict is
   > spent, but the verdict is not: reasons 2 and 3 stand untouched, and the filter is still gated
   > off.** What the fix changes is §4.1 — the precondition is now *measurable* rather than
   > *unmeasured*. It is still unmet: §2.2's saturation and §5's absence of ground truth are exactly
   > as they were, so "does host fusion beat SFLP?" remains unanswered, not answered "yes".
2. **The improvement that *is* real turns out to be a phase correction, and it is obtainable without
   giving up SFLP.** Stream 9's quaternion is effectively timestamped at the **gyro batch midpoint**
   (measured: within 2.3 ms on four captures), i.e. **~15.4 ms before the batch end** — because the
   firmware averages the FIFO batch (`RS_LSM_SFLP_AVERAGE`, shipped for BUG-027's noise). At the room
   sweep's 38.5 °/s that is **0.59° of systematic staleness**, ~17× BUG-031's 890 µs residual. This
   is a firmware/host timestamping fix, not a reason to change the sensor's operating mode.
3. **ODR-triggered mode does not cleanly fit our frame rate anyway.** AN5763 Table 15 sets a
   **minimum T_ref period of 40 ms** for every ODRsel ≥ 200 Hz. Our ToF frame period is a very stable
   **33.00 ms** (p5 32.997 / p95 33.003 ms, TIM2; mean 33.018 ms — the p5/p95 band is the typical
   frame, the mean carries the occasional long one). A 1:1 frame-per-T_ref trigger is out of spec; it
   would have to be 1:2. And the ODRsel set available in ODR-triggered mode is
   12.5/25/50/100/200/400/800/1600/3200 Hz — **480 Hz, what we run today, is not in it.**

The INT2 pin *is* routed and free (§3.1), so availability does not settle it. The reasons above do.

---

## 1. Method, and the traps it had to clear

Each capture is replayed once through `StreamDecoder`; the **same byte stream** feeds both
estimators (stream 9 → SFLP; streams 11 + 12 → `ImuFusion`, seeded and yaw-anchored from stream 9 as
`SensorState` does). Consequences:

- **The dither confound is common-mode.** The fp16 floor is dither-limited, so a quieter board
  measures *worse* and two conditions recorded at different times can invert. Every comparison here
  is within one file, one board state, one moment — the confound cancels.
- **Quaternions are normalised before every angle** (float32 quats are only approximately unit;
  `arccos(|dot|)` on unnormalised input clips to 1.0 and reports zero for the smallest steps).
- **p95 is the headline, mean secondary, median never quoted.**

Three metrics, one of which is genuinely independent of SFLP:

| metric | definition | independent of SFLP? |
|---|---|---|
| **per-frame step** | angle between consecutive output quaternions | — (noise proxy only) |
| **gyro-consistency residual** | angle between `q[k]` and `q[k−1] ⊗ ∫ω dt` over the batch, ω = GY_NC − gbias, dt from the LSM TIMESTAMP words | partly (see below) |
| **tilt error vs raw accel** | angle between the estimator's predicted body-frame up and the **XL_NC** batch mean, gated to \|a\|−1 g < 0.01 g and \|ω\| < 2 °/s | **yes** — XL_NC is raw, not an SFLP product |

Gyro integration is good to ~0.02–0.03° over one 33 ms frame (ARW 0.0005°, residual bias 0.002°,
±1% sensitivity tolerance at 60 °/s ≈ 0.02°), so it is a valid short-baseline reference. **But
`imufusion` *is* gyro integration** — per frame it is ~94% propagation and ~6% correction — so its
residual on that metric is small by construction. Read the metric as *"how far SFLP's output sits
from the gyro"*, i.e. the size of the prize, not as a score for the filter.

---

## 2. Results

### 2.1 Stationary — `stationary_stream11_20260728_190311.bin`, first 3000 frames (100 s)

Rig parked, 0.13 °/s p95 residual rate, one attitude (body X at world −Z, ceiling-facing).

| p95 | SFLP (stream 9) | `imufusion` | ratio |
|---|---|---|---|
| per-frame rotation step | **0.0858°** | **0.0242°** | 3.5× quieter |
| tilt error vs raw accel | **0.233°** | **0.104°** | 2.2× better |
| gyro-consistency residual | 0.0861° | 0.0237° | — |
| heading error vs SFLP | — | **2.22° (defect, §2.4)** | — |

The tilt result decomposes cleanly: the **SFLP gravity vector** is itself 0.102° p95 from the raw
accel, and `imufusion` lands at 0.104° because gravity is its tilt reference. So what the 2.2× is
really measuring is *the fp16 quaternion is 2.2× further from truth than the 16-bit fixed-point
gravity vector the same chip also emits* — exactly the documented encoding penalty, now measured
against an independent reference rather than inferred.

> ⚠ **One attitude.** The fp16 floor is orientation-dependent (confirmed, 21% spread across a 90°
> rotation). This is a 1-point sample of a 3-D property and cannot be generalised. Also note my
> `step_sflp` mean of 0.029° is above the 0.018° in `docs/iks4a1-stacking.md`; both fall inside the
> documented dither swing (0.0118 → 0.0183), which is why only within-file ratios are quoted.

### 2.2 Under real motion — `roomSweepFull20260730.bin` (118 s, 3573 frames, 38.5 °/s rms)

| p95 | SFLP | `imufusion` | note |
|---|---|---|---|
| gyro-consistency residual | **0.523°** (mean 0.225°) | 0.111° (mean 0.049°) | SFLP is 10× further from the gyro under motion than at rest |
| tilt error vs raw accel | 1.278° | 1.277° | **metric saturated — see below** |
| divergence between the two | tilt 1.38° / heading 1.47°, mean 0.60° / 0.57° | | |

`coffeeRoomCircuitNoMnt.bin` (1987 frames, 32.9 °/s) and `coffeeRoomCircuitMnt.bin` agree: SFLP
residual p95 0.462° / 0.498°.

**The accel-referenced tilt metric stops discriminating during motion** and must not be quoted as a
result. On the room sweep SFLP, `imufusion` *and the SFLP gravity vector itself* all land within 2%
of each other (0.879 / 0.864 / 0.868° mean) — the reference is the limit, because linear
acceleration contaminates the accelerometer faster than the estimators differ. The honest statement
is: **under motion no capture in this repo can adjudicate between the two estimators**, because
there is no ground truth in any of them.

### 2.3 The phase finding — SFLP's quaternion is ~15 ms old

Two per-frame angular-rate series were built from the same capture and cross-correlated:
`ω_sflp[k] = 2·log(q[k−1]⁻¹q[k])/dt` against `ω_gyro[k]` = the bias-corrected GY_NC batch mean.

A backward difference is centred half a frame early by construction, so the quaternion's effective
time is `δ = (0.5 − L)·dt` where L is the measured shift:

| capture | rate rms | L (frames) | ⇒ δ vs gyro batch midpoint | corr |
|---|---|---|---|---|
| `roomSweepFull20260730` | 38.5 °/s | 0.5051 | **−0.16 ms** | 0.971 |
| `coffeeRoomCircuitNoMnt` | 32.9 °/s | 0.5619 | **−1.99 ms** | 0.965 |
| `coffeeRoomCircuitMnt` | 30.5 °/s | 0.5699 | **−2.24 ms** | 0.966 |
| `tilt_sweep_20260729` | 2.3 °/s | 0.5281 | −0.90 ms (low SNR) | 0.956 |

**Stream 9's quaternion carries the orientation at the FIFO batch midpoint**, to within 2.3 ms.
Measured batch span is 30.8 ms with a constant 2.083 ms inter-batch gap (i.e. the batch tiles the
frame interval and ends at the drain), so the midpoint is **~15.4 ms before the batch end**.

Corroboration from an independent route: `imufusion` propagates to the batch *end*, so it should
lead SFLP by 15.4 ms × 38.5 °/s = **0.59°**. Measured tilt divergence on that capture: **0.60° mean.**
Two unrelated measurements, same number.

**What this means for BUG-031.** *If* the ToF FRAME_READY stamp sits at the batch end — which is what
BUG-031's own 1072 µs-RMS measurement against "the last IMU sample of each batch" implies — then the
orientation attached to each depth frame is **~15 ms stale**, ~17× the 890 µs residual BUG-031 is
chasing. This is a consequence of `RS_LSM_SFLP_AVERAGE` (added 2026-07-28 to cut noise 2.8×): the
batch mean is the midpoint orientation, and nothing downstream knows that. **It is correctable in
firmware or on the host with no change to the sensor's operating mode** — average for noise, then
propagate the average forward to the frame stamp with the gyro words already in stream 11. That is
`imufusion` used as a *phase corrector*, which is a much smaller and safer change than replacing the
orientation source.

### 2.4 The defect: `imufusion`'s heading loop is degenerate in this body frame

`ImuFusion._correct_yaw` computes its error as `quat_yaw_deg(ref) − quat_yaw_deg(self._q)` — ZYX yaw,
i.e. the heading of body **Z**. The SFLP body frame has **X = Up**. Measured attitudes:

| capture | ZYX pitch | closest approach to gimbal lock |
|---|---|---|
| `stationary_stream11_20260728_190311` | 86.2° (p5 86.13 / p95 86.19) | **3.76°** |
| `roomSweepFull20260730` | 29.2° mean, p95 80.7° | **0.72°** |

At 86° pitch the ZYX decomposition is ill-conditioned: a small tilt perturbation reads as a large
apparent yaw. Result — heading error against SFLP, same capture, same filter otherwise:

| yaw error term | τ_yaw = 1.0 s | τ_yaw = 0.3 s |
|---|---|---|
| shipped `quat_yaw_deg` (stationary) | 1.689° mean / 2.217° p95 | 1.703° / 2.244° — **unmoved** |
| world-Z heading (drop-in replacement) | **0.017° / 0.053°** | 0.015° / 0.044° |
| shipped `quat_yaw_deg` (room sweep) | 0.581° mean | 0.392° |
| world-Z heading (room sweep) | 0.511° | **0.237°** |

The shipped loop's error is insensitive to loop gain — the signature of a wrong measurement, not a
mistuned one. This is a ~5-line fix, but it is unfixed today, and **it is why the answer to "does
host fusion beat SFLP" is currently *no*.**

> **FIXED 2026-07-30 — BUG-039.** The world-Z row above is now what ships:
> `sensors.graft_yaw_error_deg(target, quat)` = `2·atan2(rel_z, rel_w)` of `rel = target ⊗ quat*`,
> the swing-twist about world Z, which is the exact inverse of the `graft_yaw` the loop corrects
> with. Both rows of this table reproduced to the digit on an independent harness before the change
> (1.6892 / 2.2178 with the shipped term, matching the 1.689 / 2.217 recorded here), and the
> substituted term landed on 0.0171 / 0.0534 against the predicted 0.017 / 0.053.
>
> Two things this section could not see, added by the fix's own ensemble (seven captures, in
> `BUGS.md` → BUG-039):
> - **The two zero-pitch captures come out bit-identical.** The misreading scales as
>   `tilt × tan(pitch)` — 0.04° at level, ~7° at 86° — so this was a frame error at *this device's*
>   attitudes, not a general retune. Worth knowing before generalising the 100×: on the moving
>   captures, whose pitch swings through the whole range, the gain is only **1.1–1.8×**.
> - **`YawFusion` already defended against the same degeneracy** with a `gimbal_margin_deg = 15°`
>   gate — which would have gated the entire 86.2° stationary capture out. `imufusion` reused
>   `quat_yaw_deg` without inheriting that gate. The new term needs none.
>
> **The verdict of this note is unchanged.** Reason 1 above is spent; reasons 2 and 3 are untouched,
> the filter is still gated off, and §2.2 / §5 still stand — no capture in this repo can adjudicate
> the two estimators under motion, and none contains orientation ground truth. §4.1's remaining
> requirement is now the *re-measure with ground truth*, not the fix.

This matters more than it looks: `Mapper` re-injects the prior's **absolute** attitude every frame
(`predict_pose(quat, self._t_prev)`), and ICP's rotation correction is *not carried forward* —
`self._t_prev` keeps translation only. ICP does correct rotation against the raycast model within
each frame, so orientation error is not integrated blindly; but the prior's absolute accuracy is
re-tested every single frame and is what ICP has to start from.

---

## 3. What taking ODR-triggered mode would cost

### 3.1 Electrical — INT2 is routed and free (so this does not settle it)

| item | finding | source |
|---|---|---|
| LSM6DSV16X **INT2** → Arduino | **CN9 pin 5** (also morpho CN10.29) | UM3239 Rev 5, Table 4 |
| CN9.5 on the H563 stack | **PE14** | `references/kicad/roomscanner-stack/stack-pinmap.md` (CN9 = PB7·PB6·PG14·PE13·PE14·PE11·PE9·PG12) |
| PE14 used by X-NUCLEO-53L9A1? | no — the ToF board uses only PB8/PB9/PB5/PB1/PB6/PB7 | 53L9A1 schematic |
| PE14 used by firmware? | no — `.ioc` uses PE0, PE2–PE6 only | `firmware/scanner-stream/53L9A1_PostprocessSingle.ioc` |
| PE14 alternate function | **TIM1_CH4** (AF1) — can generate the reference in hardware | STM32H563 datasheet AF table |
| LSM6DSV16X **INT1** → CN9.6 → **PE11** = **TIM1_CH2** (AF1) | also free — input-capture capable | same |

Both INT lines land on channels of the **same timer**. That is a better fact for the *alternative*
in §4 than for ODR-triggered mode.

Two unresolved electrical caveats:
- INT2 crosses the IKS4A1's **U2 NXS0108** auto-direction translator. Today INT2 is an LSM *output*
  (1.8 V A-side → 3.3 V B-side). ODR-triggered mode reverses it: the MCU must drive **into** the
  1.8 V domain. NXS0108 has no DIR pin; it auto-senses, and this stack's history already names
  auto-direction translators as the leading suspect for I3C flakiness. A 400 Hz push-pull square
  with a ≥5 µs pulse should be easy for it, but this is **unverified**.
- The routing goes through **SB8/SB9**. Both are 0 Ω parts in the BOM and neither is in UM3239's
  not-mounted list (SB6, SB10, SB12, SB14, SB18–20, SB22), so INT2 is presumably connected as
  shipped — but the text dump does not resolve which of SB8/SB9 selects CN9.5. **Confirm on the
  schematic page or with a continuity check before committing.**

### 3.2 The trigger signal does not fit our frame rate

AN5763 §3.3 + Table 15:

- **ODRsel set is different from the normal ODR set**: 12.5 / 25 / 50 / 100 / 200 / 400 / 800 / 1600
  / 3200 Hz. **480 Hz — what we run — does not exist in this mode.** Nearest is 400 Hz (−17% of the
  samples per frame) or 800 Hz (2× the FIFO/stream-11 bandwidth).
- **T_ref must be an even multiple of the ODR period**, `ODR_TRIG_N_ODR` ∈ [4, 255] in steps of 2
  (8–510 samples), minimum ratio 16 samples at ODRsel 400 Hz.
- **Minimum T_ref period 40 ms** for every ODRsel ≥ 200 Hz. Our ToF frame period is **33.00 ms**
  (very stable: p5 32.997, p95 33.003 ms, from TIM2 stamps over 3573 frames). **1:1 is out of spec.**
  It would have to be T_ref = 2 frames = 66.04 ms, `N_ODR` = 26 → 393.7 Hz (inside the ±33% band
  around ODRsel 400). Workable, but it means the IMU's sample grid is only re-phased every *other*
  depth frame.
- Lock takes **4 T_ref periods** (~0.26 s at 1:2). A ToF frame drop perturbs T_ref; a burst of drops
  costs a re-lock, during which the IMU's rate is undefined. Today a ToF drop costs nothing on the
  IMU side.
- Forbidden ODR codes (`ODR_XL` 0001/0010/1100, `ODR_G` 0010/1100 = 1.875 / 7.5 / 7680 Hz) do **not**
  bite us. Qvar/EIS incompatibility does not bite us either. The SFLP incompatibility is the whole
  cost.

### 3.3 What disappears from stream 11 when SFLP is off — measured

`RS_LSM_SFLP_BATCH_AUX` batches the SFLP **gravity** and **gbias** vectors, and they are SFLP
embedded-function outputs (`lsm6dsv16x_fifo_sflp_batch_set`). Turning SFLP off removes **all three**
of `imufusion`'s inputs, not just the quaternion. `imufusion` as written cannot run in ODR-triggered
mode at all.

| lost input | what it does today | measured replacement cost |
|---|---|---|
| **stream 9 quat** | seed + the *only* yaw anchor | with `yaw_ref = None`, `_correct_yaw` is a no-op → yaw is **completely unanchored**. At the measured 0.18–0.20 °/s gyro bias that is ~23° of free-run over a 118 s sweep. The only replacement anchor is the magnetometer — whose *direction* accuracy is explicitly unvalidated (resume doc §4.6) and which the tripod ruins (BUG-034). |
| **SFLP gravity** | tilt reference; never trips the 5% accel gate (**0/3000 frames** on the room sweep) because SFLP filters linear acceleration out | raw XL_NC is outside the ±5% gate **23.1%** of frames on `roomSweepFull` and **15.0%** on `coffeeRoomCircuitNoMnt`. Tilt correction would be off for ~1 frame in 5, concentrated in exactly the fast-motion frames where gyro bias matters most. |
| **SFLP gbias** | live gyro-bias estimate, measured **0.177–0.196 °/s** | no host replacement exists — `KI_BIAS_HZ = 0.0` is reserved and unwritten. |

### 3.4 Host-side blast radius

- `SensorState.feed` drives **`YawFusion.update()` from the IMU_QUAT branch**. No stream 9 ⇒ the
  magnetometer fusion never runs. It must be rewired to the IMU_RAW branch.
- `Mapper` reads `SensorState.fused_quat()` and uses the **absolute** attitude every frame, with no
  ICP rotation feedback (§2.4). Any new yaw error is a mapping error.
- Web UI gizmo/compass/orientation modes, `web.view_rotation` (World/FPV/Mirror), the magcal modal's
  `MAGPOSE`, and the jitter diagnostics all read `fused_quat()` — they follow automatically once
  `imufusion` is wired, but every one of them inherits the heading defect of §2.4 until it is fixed.
- **Every capture in `captures/` that carries stream 11 (15 of them) also carries stream 9.**
  Post-change recordings would not, so
  the existing SLAM/orientation validation corpus becomes non-comparable on a like-for-like basis.
- Removing a stream is a wire change ⇒ the `protocol-change` checklist (spec, firmware C, host
  Python, golden vectors) applies.

### 3.5 A clock note found along the way

TIM2-vs-LSM rate ratio measured per capture: **30 725 ppm** on `stationary_stream11_20260728_190311`
(no stream 12 ⇒ nominal 21.7 µs tick; matches the documented 29 790 ppm) and **5437 ppm** on
`roomSweepFull20260730` (stream-12 corrected). The residual is larger than the 3345 ppm on record.
Since 2026-07-28 the H563 clocks PLL1 from **HSI, with no crystal on the board** (BUG-023/024/025) —
HSI is a ~±1% source, so a good part of that 5437 ppm is plausibly the *MCU's* time base, not the
LSM's. Relevant here because ODR-triggered mode would make that HSI the master clock for the IMU's
sample rate. Not disqualifying (a shared clock is the point, and absolute scale cancels for
alignment), but it should not be assumed to be an accuracy *gain*.

Detrended relative offset jitter between the two clocks: **1314 µs RMS** moving / **734 µs** stationary
— independently consistent with BUG-031's 1072 µs, and notably *worse under motion*.

---

## 4. What would have to be true to change the answer

In rough order of how much each would move it:

1. ~~**Fix `_correct_yaw` to use a world-Z heading error and re-measure.**~~ **Done 2026-07-30
   (BUG-039)** — stationary heading error 2.22° → 0.053° p95, confirmed on a seven-capture ensemble.
   The precondition is now *measurable*; it is still **unmet**, because the re-measure that would
   decide it needs item 2's ground truth. Nothing about the answer changed, only its status: it went
   from *unmeasurable* to *unmeasured*.
2. **A ground-truth capture.** Every "which is right" question here is unanswerable because no
   capture has an independent attitude reference. A braced fixed-heading tilt sweep (already required
   by resume §4.6) plus a controlled pan against a known angle would let the two estimators be
   *scored* rather than merely *differenced*.
3. **Confirmation of where FRAME_READY sits inside the IMU batch.** §2.3's 15 ms figure is the
   difference between the quaternion's effective time and the batch *end*; the step from there to
   "15 ms stale relative to the depth frame" leans on BUG-031's own measurement. **Stream 13
   `IMU_SYNC` (in flight, §5) measures this directly and keeps SFLP** — as would a TIM1_CH2 input
   capture of the LSM INT1 edge (pin already free, §3.1). If it showed FRAME_READY already near the
   batch midpoint, §2.3's prize evaporates and the case for host fusion weakens further.
4. **Allan-variance numbers** (resume §4.4, tool still unwritten; the 900 s capture is waiting).
   `imufusion`'s crossovers are currently datasheet guesses. If bias instability turns out large
   relative to what gbias removes, the tilt loop needs to become PI — which is also the loop that
   would have to survive without gbias in §3.3.
5. **Only after all of the above**: if measured, tuned, and ground-truthed host fusion beat SFLP by a
   margin that justified losing the gravity estimator, the 5%-gate exposure, and the magnetometer
   becoming the sole heading reference — then revisit ODR-triggered mode, and expect to run it at
   T_ref = 2 frames / ODRsel 400 Hz.

**The cheaper path that is not this trade-off:** wire `imufusion` in as a *phase corrector* over
SFLP (propagate the batch-mean quaternion forward to the frame stamp using the stream-11 gyro words),
and add a TIM1_CH2 input capture on INT1 to measure the cross-clock offset in hardware. That
addresses §2.3's 0.59° and BUG-031's 890 µs together, keeps the gravity estimator, keeps gbias, keeps
the yaw anchor, and requires no INT2, no NXS0108 direction reversal, no ODR change, and no protocol
change.

---

## 5. What could NOT be determined offline

- **Which estimator is more accurate under motion.** No capture contains ground truth, and the one
  independent metric (tilt vs raw accel) saturates on the accelerometer during motion — SFLP,
  `imufusion` and the SFLP gravity vector all land within 2% of each other (§2.2).
- **The absolute ToF↔IMU offset.** The captures carry no cross-clock reference; only the *jitter*
  (§3.5) and the *relative* quaternion phase (§2.3) are recoverable. Whether `imufusion`'s 15 ms
  phase lead is a correction or an overshoot depends on where FRAME_READY sits, which needs the rig.
  > **In flight (uncommitted at the time of writing):** a concurrent session is adding
  > **stream 13 `IMU_SYNC`** — the LSM `TIMESTAMP` register read at the FRAME_READY edge, one per ToF
  > frame, on the same counter as stream 11's TIMESTAMP words. That is exactly this missing
  > measurement, in hardware, **with SFLP left on**. Once a capture carries stream 13, §2.3's
  > "~15 ms stale" becomes directly checkable: compare `frame_ready_ticks` against the batch
  > midpoint. It also removes the last thing ODR-triggered mode was going to buy.
- **Whether the NXS0108 will pass an MCU-driven 400 Hz push-pull clock into the 1.8 V domain**
  (§3.1). Bench-only.
- **Whether SB8/SB9 as populated on this board actually connect LSM6DSV16X_INT2 to CN9.5.** The PDF
  text extraction does not resolve the SB matrix; needs the schematic page or a continuity check.
- **Heading *direction*.** Unvalidated repo-wide (resume §4.6) and untouched by this note. Nothing
  here asserts anything about absolute heading — the yaw numbers in §2.4 are *differences against
  SFLP*, whose yaw origin is itself arbitrary.
- **Orientation coverage of the stationary result.** §2.1 is one attitude; the fp16 floor is
  orientation-dependent. The motion captures span attitudes but cannot adjudicate (see above).

---

## Reproducing

No new tooling was added. The three measurements, replaying a capture through `StreamDecoder` and
feeding stream 9/11/12 to `ImuFusion` exactly as `SensorState.feed` does:

1. **Per-frame step / gyro residual** — for each IMU_RAW frame, integrate the batch's GY_NC words
   (minus the last gbias) at dts derived from the TIMESTAMP words into `Δq`; compare `q[k]` against
   `q[k−1] ⊗ Δq`. Normalise both before `2·acos(|dot|)`.
2. **Tilt vs raw accel** — gate on `abs(|XL_NC mean| − 1) < 0.01` and `max|GY_NC − gbias| < 2 °/s`;
   compare `R(q)ᵀ·[0,0,1]` against the normalised accel mean.
3. **Phase** — cross-correlate `2·log(q[k−1]⁻¹q[k])/dt` against the bias-corrected GY_NC batch mean
   over integer frame shifts with a parabolic sub-frame fit; the quaternion's effective time relative
   to the batch midpoint is `(0.5 − L)·dt` (the 0.5 removes the backward-difference artifact — omit
   it and you will report a spurious half-frame lag).
