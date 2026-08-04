"""Scanner-owned ranging-profile model: the single host-side owner of the
range/power/I3C-bus math for Room Mapping, Precision, High Frame-Rate, and Manual.

No device, web, or Open3D imports — pure data + arithmetic, safe to import
anywhere (host CLI, `roomscan-web`, `roomscan-mcp`) without pulling in a
transport or a renderer. `host/tools/profile_probe.py` reports what a capture
actually did; this module reports what a requested configuration is EXPECTED
to do, and is the truth-in-advertising layer the UI/MCP consequence readouts
sit on top of (Task 10/11).

RECONCILIATION (2026-08-03, Task 1 of docs/superpowers/plans/2026-07-31-high-
framerate-and-manual-ranging-modes.md)
-----------------------------------------------------------------------------
The source spec's first draft had a power equation that did not reproduce its
own preset table (`50 + 5*6*30 = 950`, not the stated 200 mW), a max-range
equation that did not reproduce its own preset table either (~7.9 m at 10 ms,
not the stated 8.8 m), and an unsupported 0.5 ms exposure step. This module
fixes the underlying model against DS14879 rev 6 (the VL53L9CX datasheet,
`references/datasheets/NUCLEO-VL53L9CX/datasheet.pdf`), not just the numbers
in the doc. See that spec's amended §3.2/§3.3/§7.1 for the prose version of
what follows.

**Power — SUPERSEDED 2026-08-03.** The DS14879-two-point-fit model described in
this paragraph was REPLACED by the decompiled-ProfileTuning.exe model below
("Power model, decompiled ProfileTuning.exe (2026-08-03)") once that model
validated 5/5 against owner-run tool readings within 0.01%. Kept here,
unedited, as the historical record of why the fit existed and what it traded
off — `POWER_COEFFICIENTS`, `_fit_line`, and `estimate_power_mw`'s old body are
gone from the code; this prose is not.

DS14879 Table 36 "Power consumption" gives real, indoor-only
(0 W/m^2 — the outdoor columns are deliberately unused, see below) power
figures for two (ranging_mode, power_mode) combinations, each measured at TWO
different exposures per Table 21 "Profile settings" — a real two-point fit,
not a guess:

  Precision, Regular, 54x42: (duty 0.12, 235 mW) @ 4 ms/30 fps indoor
                              (duty 0.30, 370 mW) @ 10 ms/30 fps outdoor-cloudy
    -> slope 750.0 mW per unit duty, intercept 145.0 mW
  Ambient,   Regular, 54x42: (duty 0.12, 225 mW) @ 4 ms/30 fps indoor
                              (duty 0.48, 560 mW) @ 16 ms/30 fps outdoor-sunny (gray 17%)
    -> slope 930.6 mW per unit duty, intercept 113.3 mW

`duty = exposure_ms * fps / 1000` is the fraction of the frame period the
laser spends integrating — a physically real quantity (unlike "exposure_ms *
fps" alone, which has units of ms/s and grows without bound).

CAVEAT carried forward, not hidden: the ambient-mode pair's two points also
change ambient light condition (indoor -> outdoor sunny) alongside exposure,
because that is how DS14879's own test matrix is built — it never varies
exposure at fixed ambient light. The fitted ambient slope therefore folds in
some genuine photon-current increase from brighter ambient light, not pure
laser-duty cost, so it is an upper bound on the duty term, not a clean
separation. No cleaner number exists in the public datasheet.

ULP/LOW power-mode intercepts are not directly measured at 54x42 in Table 36
(its only ULP row is 24x20, a different resolution). They are DERIVED:

  ULP, Ambient:   solved EXACTLY from Table 9 "Profile examples"' own Room
                  Mapping anchor (30 fps, 6 ms, ULP -> 200 mW, duty 0.18)
                  against the Ambient/Regular slope above:
                  200 = intercept + 930.6*0.18  =>  intercept = 32.5 mW
  ULP, Precision: no matching 54x42/ULP/Precision anchor exists at all.
                  Scaled from the Regular/Precision intercept (145.0 mW) by
                  the ULP/Regular intercept ratio measured on the Ambient
                  pair (32.5/113.3 ~= 0.287) — i.e. ASSUMING ultra-low-power's
                  idle-current saving is ranging-mode-independent (both
                  disable circuitry between frames, per DS14879 3.2 "Power
                  modes"). Documented assumption, not a table lookup:
                  ~41.6 mW.
  LOW (LP):       the only LP row in either table (Table 9's "AR glasses",
                  outdoor sunny / 100 klx) is confounded by ambient light the
                  same way as the Ambient/Regular pair AND is a different
                  power mode, so it cannot anchor an intercept by itself.
                  LP intercepts are the midpoint of the ULP and Regular
                  intercepts for each ranging mode — the WEAKEST-grounded
                  numbers in this model, flagged as such in `POWER_COEFFICIENTS`.

Validation against real anchors this fit was NOT built from: Precision/
Regular/DSS-off at 90 fps/4 ms (the shape of Table 9's 100 fps "Gaming"
example, NOT the High Frame-Rate preset, which was amended 2026-08-03 to
46 fps/DSS-on after a hardware ceiling measurement found 90 fps and 100 fps
requests are both delivered as period-multiples rather than 1:1 — see
"Measured hardware ceiling" below) predicts 415.0 mW at duty 0.36, against
Table 9's 420 mW at duty 0.40 (100 fps) — 1.2% off, despite running at a
different fps and with DSS off (the fitted precision slope came from DSS-ON
rows). That is the strongest external check this model has, and it passes.

**Power model, decompiled ProfileTuning.exe (2026-08-03) — REPLACES the fit
above.** ST ships `references/software/53L9A1/ProfileTuning.exe` (support-
gated, kept locally untracked), a GUI planning tool whose `MainWindow.update()`
computes AVDD/DVDD/IOVDD/VBAT_Rx/VBAT_Tx terms from a small set of exact
per-(ranging_mode, power_mode) coefficient tables — ST's OWN equations, not a
fit to sparse datasheet rows. Extracted with `pyinstxtractor-ng` +
`decompyle3` (PyInstaller/Python 3.7 payload) into a scratch directory, never
into `host/.venv`. Reproduced here exactly:

  duty_cycle_2 = exposure_ms / frame_period_ms                 (== this
                                                                  module's own
                                                                  `duty_cycle()`)
  duty_cycle   = max(exposure_ms*1.15 + 2.0, dss_duration_ms) / frame_period_ms
    where dss_duration_ms = 13.5 for 54x42/binning 2 with DSS on, else 0 — the
    tool's own per-resolution DSS "housekeeping" window, distinct from (and
    additional to) the I3C readout time above
  P_AVDD  = table[ranging_mode]*duty_cycle  + table[power_mode+2]*(1-duty_cycle)   table=(100,100,60,20,0,3,0)
  P_DVDD  = table[ranging_mode]*duty_cycle  + table[power_mode+2]*(1-duty_cycle)   table=(100,100,25,10,5,6,0)
  P_IOVDD = table[ranging_mode]*duty_cycle  + table[power_mode+2]*(1-duty_cycle)   table=(1,1,1,1,1,0,0)
  P_VBAT_Rx = table[ranging_mode]*duty_cycle_2*(ambient_lux+1000)*0.0002
              + table[power_mode+2]*(1-duty_cycle)                                table=(42,42,20,5,5,0,0)
  P_VBAT_Tx = table[ranging_mode]*duty_cycle_2 + table[power_mode+2]*(1-duty_cycle)   table=(550,660,15,0,0,6,0)
  P_total = P_AVDD + P_DVDD + P_IOVDD + P_VBAT_Rx + P_VBAT_Tx

`ranging_mode` indexes 0/1 = Precision/Ambient and `power_mode` indexes
0/1/2 = Regular/Low/UltraLow (offset +2 into the same tables) — both exactly
this module's own `RangingMode`/`PowerMode` wire values, so no remapping table
is needed; table indices 5/6 exist in the tool's own source but are dead code
(never read by `update()`), reproduced faithfully rather than trimmed.
`ambient_lux` is a genuine physical input the DS14879-fit model above had NO
way to represent at all (DS14879's own rows confound exposure with ambient
light — see the Max range section below for the same problem in a different
model); `AMBIENT_LUX_DEFAULT` (100.0) is the tool's own "Home, Theatres -
100 Lux" dropdown entry, a documented indoor reference point, not a
measurement of any specific room.

**Validation, 2026-08-03:** five owner-run ProfileTuning.exe readings (54x42,
I3C, DSS Enable, ambient = "Home, Theatres - 100 Lux" unless noted),
reproduced by the extracted equations to within 0.01% — an order of magnitude
inside the ~2% bar this replacement was gated on:

  Ambient,   ULP,     6 ms/30 fps: tool 208.4 mW, model 208.41 mW (+0.01%)
  Precision, ULP,    10 ms/30 fps: tool 255.7 mW, model 255.72 mW (+0.01%)
  Precision, Regular, 4 ms/37 fps: tool 243.7 mW, model 243.73 mW (+0.01%)
  Ambient,   Regular, 2 ms/37 fps: tool 210.5 mW, model 210.48 mW (-0.01%)
  Ambient,   Regular, 4 ms/37 fps: tool 260.0 mW, model 260.01 mW (+0.00%)

(`host/tests/test_profiles.py`'s "Power model (decompiled ProfileTuning.exe)"
section pins all five.) Because this model is exact-per-ST rather than a
fit, it no longer reproduces the two real DS14879 Table 36/Table 9 anchors
the old fit was built from as exactly as that fit necessarily did by
construction — e.g. Room Mapping now estimates 208.4 mW where DS14879 Table 9
states 200 mW "(5 klx)": ProfileTuning's own default ambient (100 lux) is not
DS14879's own footnoted test condition (5000 lux) for that row, and the two
numbers were never going to agree once ambient light became an explicit,
independently-set input instead of a hidden constant baked into a fitted
intercept. This is expected, not a regression — `power_mw` is still labelled
ESTIMATED everywhere it surfaces (no measured claim without a power meter),
and `estimate_power_mw`'s docstring/`ProfileEstimate.power_mw` field name are
unchanged, so `roomscan-web`/the UI consequence readouts pick up the new
numbers automatically with no call-site changes required.

**Max range.** The original continuous `sqrt(exposure)` formula is dropped
entirely: DS14879 Tables 23/24 vary exposure and ambient light TOGETHER in
every row (a longer exposure is how the vendor's own test compensates for
brighter ambient light), so "more exposure -> more range" is not something
the published data supports in isolation — the tables even show range
*decreasing* with more exposure at the less-reflective target, because the
brighter ambient light that motivated the longer exposure hurts SNR more than
the extra integration time helps. The host has no ambient-light sensor to
condition on either way. Modelling range as a function of exposure would
therefore be false precision.

Instead, max range is a lookup keyed on (ranging_mode, dss_enabled) — exactly
the categorical distinction DS14879's own profile examples make — using the
conservative (gray 17%) indoor figure:

  Ambient   + DSS on  (Room Mapping / Manual ambient, <=60 fps):  8.0 m
    (Table 9's Room Mapping anchor, ULP; Table 24's Regular-mode figure is
    8.5 m — the ULP anchor is used because ULP is the profile actually in
    force for Room Mapping)
  Precision + DSS on  (Precision / Manual precision, <=60 fps):   8.8 m
    (Table 9's own stated ranging ceiling "up to 8.8 m"; also Table 23 gray
    62% at both tested exposures)
  Precision + DSS off (>60 fps, forced):                          5.0 m
    (Table 9's Gaming anchor, exact match)

Minimum distance is Table 22 "Minimum ranging capabilities", exact and NOT
exposure-dependent: 450 mm ambient, 50 mm precision.

**I3C ToF bus airtime.** The spec's original percentages (28.5/57.0/85.5/95.0%
at 30/60/90/100 fps) had two problems, not one: the written formula
("9.49888 ms x FPS x 100%") had a units bug (fixed as `T_xfer_ms /
frame_period_ms * 100`, equivalently `T_xfer_ms * fps / 10`), AND the
9.49888 ms coefficient itself was wrong — it was derived from the raw
12.5 MHz I3C SDR clock, but that is not the achievable transfer rate; per-byte
protocol overhead (addressing, ACKs, CRC) brings the *effective* throughput
down to a documented 10 Mbps. **REFINEMENT (2026-08-03, decompiled
ProfileTuning.exe + AN6522 investigation):** two independent sources agree on
the 10 Mbps effective figure and disagree with the raw-clock number: AN6522
Table 5 "Readout duration depending on the readout interface" states, in the
vendor's own words, "54x42 -> I3C (10 Mbps): 11.8 ms"
(`references/datasheets/NUCLEO-VL53L9CX/an6522-guidelines-for-tuning-ranging-
profiles-with-vl53l9cx-stmicroelectronics.pdf`); and the decompiled
`ProfileTuning.exe` planning tool (`references/software/53L9A1/
ProfileTuning.exe`, support-gated, kept locally untracked) computes the same
quantity as `frame_size_bytes * 8 * 1000 / 10e6`. `I3C_XFER_MS` is now
`14,842 bytes * 8 / 10e6 * 1000 = 11.8736 ms` (was 9.49888 ms at the raw
12.5 MHz clock). Every bus-utilization consequence moves with it: 30 fps
28.5% -> ~35.6%, 46 fps (High Frame-Rate preset) 43.7% -> ~54.6%, 60 fps
57.0% -> ~71.2%; 90/100 fps requests now compute *above* 100% (106.9%/118.7%)
because the raw transfer alone no longer fits inside the requested period at
all — `i3c_bus_utilization_pct`'s existing `min(100.0, ...)` clamp reports
100% for both, which is the honest ceiling, not a claim those requests are
merely "near saturation". This is the sensor's own I3C link only ("ToF bus
airtime" per the plan's Task 10 label), not the Ethernet/USB transport link
Task 6 paces.

**Exposure granularity verdict.** `vl53l9_set_exposure(void*, vl53l9_context_t,
uint16_t exposure_ms)` (`firmware/vendor/53L9A1/Drivers/BSP/Components/vl53l9/
vl53l9.c:536`) takes an INTEGER-millisecond parameter and is the only public
entry point the vendor app (`vl53l9_utils.c`) uses. Internally it computes
`shots_base = (500000 * exposure_ms) / blank_sum` (integer arithmetic) and
writes per-step shot counts to `VL53L9_REGADDR_STREAM_NB_SHOT_STEP` via the
public `vl53l9_write()` register-I/O function — the same pattern the DSS
extension (non-negotiable finding #3) already uses to add a local, non-
vendor-edited setter. That means a SCANNER-OWNED low-level setter accepting a
fractional exposure (replicating this math in float instead of integer ms,
still through public register I/O) is theoretically POSSIBLE without editing
`firmware/vendor/`. It is NOT proven: proving it needs (a) new scanner-stream
firmware code (Task 4, not Task 1) and (b) an on-target timing measurement
confirming the resulting shot-count register value actually changes
integration time at sub-ms resolution rather than being masked by rounding or
a hardware minimum step (Task 5). Both are out of Task 1's host-only scope, so
the contract adopted HERE is a truthful 1 ms step (`EXPOSURE_STEP_MS`), and
the sub-ms question is recorded as **PENDING hardware verification (Task 4/5)**
in the amended spec, not guessed.

**Blanking margin.** DS14879 documents no minimum frame-period-minus-exposure
margin, and `vl53l9_set_frame_period()` (`vl53l9.c:397`) validates only the
absolute 10,000-1,000,000 us range, with no cross-check against exposure —
that check is entirely the application's responsibility. The true minimum
schedulable margin (how much internal FSM/housekeeping time the sensor needs
between finishing one frame's ranging and being ready to start the next) is
an empirical quantity, not derivable from the public register math, and is
**PENDING on-target measurement (Task 5)**. `BLANKING_MARGIN_US_PENDING_HW`
below is a clearly-labelled conservative placeholder — reject only the
mathematically impossible case (exposure that cannot fit inside the period at
all, plus a small safety pad) — not a measured value; do not read it as one.

**Measured hardware ceiling (2026-08-03) — the sensor's real per-frame floor.**
Task 5's on-target sweep (readback-exact `frame_period_us`, 54x42/binning 2,
Precision context, Regular power) closed the question the placeholder above
left open, and found something the datasheet does not document at all: the
sensor has an intrinsic per-frame floor, and a REQUESTED period shorter than
that floor is not rejected and not clamped — it is silently delivered as an
INTEGER MULTIPLE of the requested period. A 90 Hz request measured 44.85 fps
(a clean 2x), a 100 Hz request measured 33.2 fps (a clean 3x). The floor
brackets by exposure (upper bound of each bracket, the conservative/
under-promising side, since the true floor is somewhere inside the bracket):
1-2 ms exposure -> floor in (16.667, 20.0] ms; 4 ms exposure -> floor in
(20.833, 21.739] ms; 8 ms exposure -> floor in (22.222, 23.529] ms. It is
sub-linear in exposure (~0.5-0.7 ms of floor per ms of exposure) with a fixed
~16-17 ms component dominating. DSS on vs off made no measurable difference
to the floor (DSS is "free"; the existing <=60 Hz-on/>60 Hz-off rule is
unchanged by this finding). DS14879's own Table 9 "Gaming" anchor (54x42/
Precision/100 fps/4 ms/Regular) does NOT reproduce at its own stated config
(2.2x shortfall) — its Table 21 characterization matrix only ever exercises
30 fps, so nothing in the datasheet actually validates a >~46 fps request at
this resolution. The highest rate ever actually delivered in the
investigation was ~49.3 fps (from a 99 Hz request); the 90-100 Hz band also
showed a reconfig-instability anomaly (BUG-073), another reason not to park a
preset there.

Consequence: **the High Frame-Rate preset is amended from 90 fps to 46 fps**
(still Precision/Regular, still 4 ms exposure, DSS now ON because 46 <= the
60 Hz DSS ceiling) — the highest fps this exposure delivers 1:1 with no
quantization, measured 2026-08-03. See the amended spec's Sec 2.1/8 and the
plan's Task 11/12 for the full sweep data; `measured_floor_ms` /
`expected_delivered_fps` / `ceiling_fps_for_exposure` below are the model
this finding produced, and `PRESETS[ProfileId.HIGH_FRAMERATE]` is the
amended preset. Manual mode is NOT re-capped to 46/60/100 fps by this
finding — a request above a given exposure's 1x ceiling remains ACCEPTED by
the sensor (this was never a validation-rejectable condition; the sensor
does not reject it either) — but `validate_manual_params` now WARNS when a
request will be delivered as a period-multiple rather than 1:1, and
`ProfileEstimate.expected_delivered_fps` reports the honest expected rate
instead of echoing the request.

**Floor extrapolation above 8 ms exposure, REFINEMENT (2026-08-03).** The
original placeholder flat-held the 8 ms bracket's floor (23.529 ms) for the
whole 9-16 ms range and flagged it UNVERIFIED, with no principled basis for
why 23.529 ms specifically should keep holding. The same decompiled-
ProfileTuning.exe / AN6522 investigation that corrected the I3C coefficient
above supplies a principled line instead: `floor_ms(exposure_ms) =
FLOOR_FW_DEADTIME_MS (1.6 ms, the firmware dead-time term ProfileTuning's own
planning model adds on top of readout) + I3C_XFER_MS (11.8736 ms, AN6522/
ProfileTuning per above) + exposure_ms + FLOOR_EXTRAPOLATION_MARGIN_MS
(~3 ms, the largest residual this line has against the three MEASURED
brackets below) ~= 16.5 + exposure_ms`. This is DERIVED, not measured, and is
labelled as such everywhere it appears — Task 5's sweep never ran an exposure
above 8 ms. The three measured brackets (<=2 -> 20.0 ms, <=4 -> 21.739 ms,
<=8 -> 23.529 ms) remain authoritative BELOW 8 ms: the same investigation
showed ST's own planning-tool equation (a flat 26.9 ms regardless of
exposure, at 54x42/DSS-on — see the amended spec's Sec 3.2.3) diverges from our
measured hardware, which is consistently FASTER (20.0-23.5 ms), so real
measurements outrank the tool's formula in the range where both exist. Above
8 ms, no measurement exists to outrank the derived line, so it is what
`measured_floor_ms` now returns — an improvement over an unexplained flat
hold, but still not a substitute for Task 5 actually sweeping 9-16 ms.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

# --- Enums: wire-compatible with the firmware's own enums where one exists ---------


class ProfileId(IntEnum):
    """`SET_RANGING_PROFILE` (command 8) enum value. Order fixed by the plan's
    Task 2 §3: "preset IDs 0-2 apply immediately, while MANUAL (3) reapplies the
    last accepted manual candidate"."""
    ROOM_MAPPING = 0
    PRECISION = 1
    HIGH_FRAMERATE = 2
    MANUAL = 3


class RangingMode(IntEnum):
    """`SET_MANUAL_PARAMS` ranging_mode field. Values mirror `vl53l9_context_t`
    (`firmware/vendor/53L9A1/Drivers/BSP/Components/vl53l9/vl53l9.h:103`):
    `VL53L9_CONTEXT_SHORT = 0` (Precision), `VL53L9_CONTEXT_LONG = 1` (Ambient) —
    matching the global constraint "Ambient maps to VL53L9_CONTEXT_LONG; Precision
    maps to VL53L9_CONTEXT_SHORT"."""
    PRECISION = 0
    AMBIENT = 1


class PowerMode(IntEnum):
    """Matches `vl53l9_power_mode_t` (`vl53l9.h:85`) exactly."""
    REGULAR = 0
    LOW = 1
    ULTRA_LOW = 2


# --- JSON vocabulary ----------------------------------------------------------------
#
# The names the `/ws` `ranging` message, the web UI and the MCP tools all speak.
# They live HERE rather than in whichever module happened to need them first, so
# there is exactly one owner: a second, independently-maintained spelling of the
# same vocabulary drifts silently (BUG-076's lesson, applied before it can happen
# again). Note these are NOT the wire enum values -- those are the IntEnum members
# above, and the two are mapped BY NAME (`control._manual_params_to_wire`).

PROFILE_ID_TO_STR: dict[ProfileId, str] = {
    ProfileId.ROOM_MAPPING: "room_mapping",
    ProfileId.PRECISION: "precision",
    ProfileId.HIGH_FRAMERATE: "high_framerate",
    ProfileId.MANUAL: "manual",
}
STR_TO_PROFILE_ID: dict[str, ProfileId] = {v: k for k, v in PROFILE_ID_TO_STR.items()}

RANGING_MODE_TO_STR: dict[RangingMode, str] = {
    RangingMode.AMBIENT: "ambient",
    RangingMode.PRECISION: "precision",
}
STR_TO_RANGING_MODE: dict[str, RangingMode] = {v: k for k, v in RANGING_MODE_TO_STR.items()}

POWER_MODE_TO_STR: dict[PowerMode, str] = {
    PowerMode.ULTRA_LOW: "ulp",
    PowerMode.LOW: "lp",
    PowerMode.REGULAR: "regular",
}
STR_TO_POWER_MODE: dict[str, PowerMode] = {v: k for k, v in POWER_MODE_TO_STR.items()}


# --- FPS <-> frame period -----------------------------------------------------------

FPS_MIN = 1
FPS_MAX = 100
DSS_FPS_CEILING = 60  # DSS enabled at/below this rate, forced off above it
EXPOSURE_MS_MIN = 1
EXPOSURE_MS_MAX = 16  # product/UI ceiling per the spec's Manual panel
EXPOSURE_STEP_MS = 1  # truthful step; see module docstring "Exposure granularity verdict"
DRIVER_EXPOSURE_MS_MAX = 30  # vl53l9_set_exposure()'s own hardware ceiling (informational —
                             # the UI/spec deliberately caps lower; not a hardware rejection)

IMU_ENV_RATE_MAX_HZ = 480   # LSM6DSV16X XL/GY/SFLP ODR ceiling (rs_lsm.c)
IMU_ENV_HUB_CYCLE_HZ = 60   # sensor-hub (env, stream 10) cycle; above this stream 10 sub-samples

TRANSPORT_CDC_FPS_CEILING = DSS_FPS_CEILING  # 60; same boundary the spec's transport
                                              # guard and the DSS/context rule both use

# PENDING hardware measurement (Task 5) -- see module docstring "Blanking margin".
# Conservative placeholder: rejects only exposure that plainly cannot fit in the
# requested frame period, plus a small pad. NOT a measured minimum.
BLANKING_MARGIN_US_PENDING_HW = 500


def fps_to_period_us(fps: int) -> int:
    """Global constraint: `frame_period_us = round(1_000_000 / fps)`."""
    return round(1_000_000 / fps)


def period_us_to_fps(period_us: float) -> float:
    """Inverse of `fps_to_period_us`. Not exactly round-trip-safe at the rounding
    boundary (e.g. fps=61 -> 16393 us -> 61.0004... fps); callers that need the
    nominal integer fps back should round explicitly."""
    return 1_000_000.0 / period_us


def dss_enabled_for_fps(fps: int) -> bool:
    """Global constraint: "DSS is enabled at 60 Hz and below and forced off above
    60 Hz". Not a user-settable field — fully determined by fps."""
    return fps <= DSS_FPS_CEILING


# --- Measured hardware ceiling (2026-08-03) -- integer-multiple quantization below
# the sensor's own per-frame floor. See module docstring "Measured hardware ceiling"
# for the full investigation; these are its brackets, unchanged. -------------------

# (exposure_ms upper bound of the bracket, floor_ms -- the bracket's OWN upper bound,
# i.e. the conservative/under-promising side). Measured at 54x42/binning 2, Precision
# context, Regular power; DSS on vs off made no measurable difference. These three
# brackets are AUTHORITATIVE below 8 ms exposure -- real measurements, which the
# 2026-08-03 refinement investigation showed outrank ST's own planning-tool equation
# (a flat 26.9 ms) in this range. See module docstring "Floor extrapolation above 8 ms
# exposure" for what happens above 8 ms (UNMEASURED; a derived line, not this table).
_FLOOR_MS_BRACKETS: tuple[tuple[float, float], ...] = (
    (2.0, 20.0),
    (4.0, 21.739),
    (8.0, 23.529),
)

# Extrapolation above 8 ms exposure (UNMEASURED -- Task 5's sweep stopped at 8 ms).
# REFINEMENT (2026-08-03, decompiled ProfileTuning.exe + AN6522 investigation):
# floor_ms = FW dead time + I3C readout + exposure + margin, ~= 16.5 + exposure_ms.
# Replaces the earlier flat-hold-the-8ms-bracket placeholder. See module docstring.
FLOOR_FW_DEADTIME_MS = 1.6            # firmware dead time, decompiled ProfileTuning.exe
FLOOR_EXTRAPOLATION_MARGIN_MS = 3.0   # largest residual vs the three measured brackets


def measured_floor_ms(exposure_ms: float) -> float:
    """Per-frame floor, ms, for a given exposure. Below 8 ms exposure this is the
    measured (upper-bound-of-bracket, conservative) 2026-08-03 figure -- see module
    docstring. Above 8 ms (UNMEASURED) it is the DERIVED line `FLOOR_FW_DEADTIME_MS +
    I3C_XFER_MS + exposure_ms + FLOOR_EXTRAPOLATION_MARGIN_MS`, clearly not a
    measurement. Any requested frame period shorter than this floor is still ACCEPTED
    by the sensor but delivered as an integer multiple of the requested period, not 1:1.
    """
    for bracket_exposure_ms, floor_ms in _FLOOR_MS_BRACKETS:
        if exposure_ms <= bracket_exposure_ms:
            return floor_ms
    # Beyond 8 ms: derived, not measured -- see docstring above and module docstring
    # "Floor extrapolation above 8 ms exposure".
    return (FLOOR_FW_DEADTIME_MS + I3C_XFER_MS + exposure_ms
            + FLOOR_EXTRAPOLATION_MARGIN_MS)


def ceiling_fps_for_exposure(exposure_ms: float) -> float:
    """Highest fps delivered 1:1 (multiplier 1) at this exposure, i.e.
    `1000 / measured_floor_ms(exposure_ms)`. Requests above this are still
    accepted by the sensor but quantized — see `expected_delivered_fps`."""
    floor_ms = measured_floor_ms(exposure_ms)
    if floor_ms <= 0:
        return float("inf")
    return 1000.0 / floor_ms


def expected_delivered_fps(requested_fps: int, exposure_ms: float) -> float:
    """Honest expected delivered rate for a requested fps at a given exposure:
    `requested_fps / ceil(floor_ms / period_ms)` — the measured integer-multiple
    quantization (measured 2026-08-03). At/below the exposure's 1x ceiling this
    equals `requested_fps` exactly (multiplier 1); above it, this is what the
    sensor actually delivers, not what was asked for — callers must report THIS
    value, not echo the request, once a manual candidate exceeds its ceiling."""
    if requested_fps <= 0:
        return 0.0
    period_us = fps_to_period_us(requested_fps)
    floor_us = measured_floor_ms(exposure_ms) * 1000.0
    if period_us <= 0:
        return 0.0
    # Tiny epsilon guards the exact-boundary case (e.g. 46 fps @ 4 ms, where
    # floor_us and period_us are equal up to float representation) from spuriously
    # rounding up to a 2x multiplier.
    multiplier = max(1, math.ceil(floor_us / period_us - 1e-9))
    return requested_fps / multiplier


def duty_cycle(exposure_ms: float, fps: int) -> float:
    """Fraction of the frame period the laser spends integrating, clamped to
    [0, 1]. The physically meaningful quantity the power model is fit against —
    see module docstring."""
    if fps <= 0:
        return 0.0
    return min(1.0, max(0.0, exposure_ms * fps / 1000.0))


# --- I3C ToF bus airtime model ------------------------------------------------------
# REFINEMENT (2026-08-03): I3C_XFER_MS is the DOCUMENTED EFFECTIVE throughput, not the
# raw 12.5 MHz SDR clock -- AN6522 Table 5 states "54x42 -> I3C (10 Mbps): 11.8 ms"
# directly, and the decompiled ProfileTuning.exe planning tool computes the identical
# quantity as frame_size_bytes*8*1000/10e6. See module docstring "I3C ToF bus airtime"
# for the full derivation and citations.

RAW_3DMD_BYTES_BIN2 = 14842          # binning=2 (54x42), docs/protocol.md
I3C_EFFECTIVE_BPS = 10e6             # effective throughput incl. protocol overhead --
                                      # AN6522 Table 5 + decompiled ProfileTuning.exe,
                                      # NOT the raw 12.5 MHz SDR clock
I3C_XFER_MS = (RAW_3DMD_BYTES_BIN2 * 8) / I3C_EFFECTIVE_BPS * 1000.0  # 11.8736 ms


def i3c_bus_utilization_pct(fps: int) -> float:
    """ToF sensor's OWN I3C link airtime, not the Ethernet/USB transport link
    (Task 6 owns that budget). `min(100.0, ...)` guards fps values below what one
    raw transfer needs (would be a already-invalid config; the estimate must not
    return a value >100%)."""
    if fps <= 0:
        return 0.0
    period_ms = 1000.0 / fps
    return min(100.0, 100.0 * I3C_XFER_MS / period_ms)


# --- Power model (REPLACED 2026-08-03: decompiled ProfileTuning.exe) -----------------
# See module docstring "Power model, decompiled ProfileTuning.exe (2026-08-03)" for the
# full derivation, the supersession of the earlier DS14879-fit model (kept in the
# docstring as history, not in code), and the 5/5 validation against owner-run tool
# readings. Tables/formula reproduced exactly from the tool's own
# MainWindow.p_avdd/p_dvdd/p_iovdd/p_vbatrx/p_vbattx and MainWindow.update().

# index 0/1 = ranging_mode (Precision/Ambient); index 2/3/4 = power_mode+2
# (Regular/Low/UltraLow). Both match this module's own RangingMode/PowerMode wire
# values exactly -- no remapping table needed. Indices 5/6 are dead in the tool's own
# source (never read by update()); reproduced faithfully, not trimmed.
_P_VBATTX = (550.0, 660.0, 15.0, 0.0, 0.0, 6.0, 0.0)
_P_VBATRX = (42.0, 42.0, 20.0, 5.0, 5.0, 0.0, 0.0)
_P_AVDD = (100.0, 100.0, 60.0, 20.0, 0.0, 3.0, 0.0)
_P_DVDD = (100.0, 100.0, 25.0, 10.0, 5.0, 6.0, 0.0)
_P_IOVDD = (1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0)

# ProfileTuning's own per-resolution DSS "housekeeping" window, 54x42/binning 2 only
# (our only supported resolution). Distinct from, and additional to, I3C_XFER_MS above.
_DSS_HOUSEKEEPING_MS_BIN2 = 13.5

# ProfileTuning's own "Home, Theatres - 100 Lux" ambient-light dropdown entry -- a
# documented indoor reference, not a measurement of any specific room. The five
# 2026-08-03 validation anchors were all read at this setting (see module docstring).
AMBIENT_LUX_DEFAULT = 100.0


def _power_active_duty_cycle(exposure_ms: float, fps: int, dss_enabled: bool) -> float:
    """ProfileTuning's own `duty_cycle` (distinct from this module's `duty_cycle()`,
    which is the tool's `duty_cycle_2`): fraction of the frame period the analog
    front end stays active, driven by whichever is larger of an exposure-derived
    settle time or the DSS housekeeping window."""
    if fps <= 0:
        return 0.0
    frame_period_ms = 1000.0 / fps
    dss_ms = _DSS_HOUSEKEEPING_MS_BIN2 if dss_enabled else 0.0
    active_ms = max(exposure_ms * 1.15 + 2.0, dss_ms)
    return min(1.0, max(0.0, active_ms / frame_period_ms))


def estimate_power_mw(ranging_mode: RangingMode, power_mode: PowerMode,
                      exposure_ms: float, fps: int,
                      ambient_lux: float = AMBIENT_LUX_DEFAULT) -> float:
    """Estimated typical power draw, mW — decompiled ProfileTuning.exe's own
    AVDD+DVDD+IOVDD+VBAT_Rx+VBAT_Tx model (2026-08-03), validated 5/5 against
    owner-run tool readings within 0.01% (see module docstring). `ambient_lux`
    defaults to the tool's own "Home, Theatres - 100 Lux" indoor reference — a
    genuine physical input the earlier fitted model had no way to represent.
    Labelled ESTIMATED everywhere this surfaces in the UI/MCP layer — no measured
    claim without a power meter."""
    r = int(ranging_mode)      # 0 Precision / 1 Ambient -- matches the tool's index
    lp = int(power_mode) + 2   # 0/1/2 Regular/Low/UltraLow -> tool's table index 2/3/4
    dss = dss_enabled_for_fps(fps) if fps > 0 else False
    active_duty = _power_active_duty_cycle(exposure_ms, fps, dss)
    io_duty = duty_cycle(exposure_ms, fps)  # ProfileTuning's own duty_cycle_2

    avdd = _P_AVDD[r] * active_duty + _P_AVDD[lp] * (1 - active_duty)
    dvdd = _P_DVDD[r] * active_duty + _P_DVDD[lp] * (1 - active_duty)
    iovdd = _P_IOVDD[r] * active_duty + _P_IOVDD[lp] * (1 - active_duty)
    vbat_rx = (_P_VBATRX[r] * io_duty * (ambient_lux + 1000.0) * 0.0002
               + _P_VBATRX[lp] * (1 - active_duty))
    vbat_tx = _P_VBATTX[r] * io_duty + _P_VBATTX[lp] * (1 - active_duty)
    return round(avdd + dvdd + iovdd + vbat_rx + vbat_tx, 1)


# --- Max range / min distance model ---------------------------------------------------

# (ranging_mode, dss_enabled) -> max range, m. See module docstring "Max range".
# The (AMBIENT, False) entry is not a supported combination (validation rejects
# ambient above the DSS ceiling) -- present only so the lookup never KeyErrors on
# an already-invalid config; callers should check `errors` first.
MAX_RANGE_M: dict[tuple[RangingMode, bool], float] = {
    (RangingMode.AMBIENT, True): 8.0,
    (RangingMode.PRECISION, True): 8.8,
    (RangingMode.PRECISION, False): 5.0,
    (RangingMode.AMBIENT, False): 5.0,
}

# DS14879 Table 22 "Minimum ranging capabilities" -- exact, NOT exposure-dependent.
MIN_DISTANCE_MM: dict[RangingMode, float] = {
    RangingMode.AMBIENT: 450.0,
    RangingMode.PRECISION: 50.0,
}


def estimate_max_range_m(ranging_mode: RangingMode, dss_enabled: bool) -> float:
    return MAX_RANGE_M[(ranging_mode, dss_enabled)]


def estimate_min_distance_mm(ranging_mode: RangingMode) -> float:
    return MIN_DISTANCE_MM[ranging_mode]


# --- Transport warning -----------------------------------------------------------------


def transport_warning_message(transport: str, fps: int) -> str | None:
    """Non-blocking by design: "the specification asks for a warning rather than a
    hard ban" (global constraint). Ethernet/replay never warn; CDC warns above
    `TRANSPORT_CDC_FPS_CEILING`."""
    if transport.lower() == "cdc" and fps > TRANSPORT_CDC_FPS_CEILING:
        return (f"{fps} fps over USB CDC exceeds the ~{TRANSPORT_CDC_FPS_CEILING} fps "
                "this transport reliably sustains; frame loss is expected. Ethernet "
                "UDP is the only transport the 90 Hz acceptance gate accepts.")
    return None


# --- Validation --------------------------------------------------------------------


@dataclass(frozen=True)
class ValidationResult:
    ok: bool
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()


def validate_imu_env_rate(rate_hz: int | None) -> ValidationResult:
    """`None`/`0` = coupled to the ToF trigger (default, today's behavior) — always
    valid. An explicit rate must be an integer 1-480 Hz; requesting above the 60 Hz
    sensor-hub cycle is reported (stream 10/env sub-samples) rather than rejected,
    per the global constraint "Validation reports, rather than silently drops"."""
    if rate_hz in (None, 0):
        return ValidationResult(ok=True)
    errors: list[str] = []
    warnings: list[str] = []
    if not isinstance(rate_hz, int) or not (1 <= rate_hz <= IMU_ENV_RATE_MAX_HZ):
        errors.append(
            f"imu_env_rate_hz must be coupled (None/0) or an integer 1-"
            f"{IMU_ENV_RATE_MAX_HZ}, got {rate_hz!r}")
    elif rate_hz > IMU_ENV_HUB_CYCLE_HZ:
        warnings.append(
            f"imu_env_rate_hz={rate_hz} exceeds the {IMU_ENV_HUB_CYCLE_HZ} Hz sensor-hub "
            "cycle: stream 10 (env) will sub-sample from the requested rate; streams 9 "
            "(quat) and 11 (raw) can still hit it.")
    return ValidationResult(ok=not errors, errors=tuple(errors), warnings=tuple(warnings))


@dataclass(frozen=True)
class ManualParams:
    """Manual mode's user-settable fields. `imu_env_rate_hz` participates in the
    same validation/estimate model as the ranging fields (Task 7 cross-reference);
    `None`/`0` means coupled to the ToF trigger."""
    ranging_mode: RangingMode
    fps: int
    exposure_ms: int
    power_mode: PowerMode
    imu_env_rate_hz: int | None = None


def validate_manual_params(params: ManualParams) -> ValidationResult:
    """Full Manual-mode validation: range checks, the DSS/>60 Hz precision-only
    rule, the blanking-margin schedulability check, and IMU/env rate. Does NOT
    touch hardware -- this is the same check the firmware's own command validation
    should reproduce (Task 4), so a request this function accepts should never be
    rejected by the device for a reason expressible host-side."""
    errors: list[str] = []
    warnings: list[str] = []

    if not (FPS_MIN <= params.fps <= FPS_MAX):
        errors.append(f"fps must be {FPS_MIN}-{FPS_MAX}, got {params.fps}")

    if not (EXPOSURE_MS_MIN <= params.exposure_ms <= EXPOSURE_MS_MAX):
        errors.append(
            f"exposure_ms must be {EXPOSURE_MS_MIN}-{EXPOSURE_MS_MAX} (the driver's own "
            f"ceiling is {DRIVER_EXPOSURE_MS_MAX} ms; the product/UI contract caps lower), "
            f"got {params.exposure_ms}")

    if params.fps > DSS_FPS_CEILING and params.ranging_mode is not RangingMode.PRECISION:
        errors.append(
            f"fps={params.fps} > {DSS_FPS_CEILING} forces DSS off, which the global "
            "constraint permits only with Precision ranging mode")

    if FPS_MIN <= params.fps <= FPS_MAX and EXPOSURE_MS_MIN <= params.exposure_ms:
        period_us = fps_to_period_us(params.fps)
        exposure_us = params.exposure_ms * 1000
        if exposure_us + BLANKING_MARGIN_US_PENDING_HW > period_us:
            errors.append(
                f"exposure {params.exposure_ms} ms does not fit the {params.fps} fps frame "
                f"period ({period_us} us) with the {BLANKING_MARGIN_US_PENDING_HW} us "
                "blanking-margin placeholder (PENDING hardware measurement, Task 5)")

    if (FPS_MIN <= params.fps <= FPS_MAX
            and EXPOSURE_MS_MIN <= params.exposure_ms <= EXPOSURE_MS_MAX):
        ceiling_fps = ceiling_fps_for_exposure(params.exposure_ms)
        if params.fps > ceiling_fps:
            delivered = expected_delivered_fps(params.fps, params.exposure_ms)
            warnings.append(
                f"fps={params.fps} at exposure_ms={params.exposure_ms} exceeds the "
                f"measured ~{ceiling_fps:.1f} fps 1x delivery ceiling (measured "
                "2026-08-03, 54x42/binning 2, Precision context/Regular power — see "
                "module docstring 'Measured hardware ceiling'); the sensor ACCEPTS "
                f"this request but delivers period-multiples, expected ~{delivered:.1f} "
                f"fps, not {params.fps} fps.")

    rate_result = validate_imu_env_rate(params.imu_env_rate_hz)
    errors.extend(rate_result.errors)
    warnings.extend(rate_result.warnings)

    return ValidationResult(ok=not errors, errors=tuple(errors), warnings=tuple(warnings))


# --- Resolved configuration + presets --------------------------------------------------


@dataclass(frozen=True)
class ProfileConfig:
    """One fully-resolved ranging configuration — either a fixed preset or a
    resolved Manual candidate. `dss_enabled` is derived, never stored, so it can
    never disagree with `fps`."""
    profile_id: ProfileId
    ranging_mode: RangingMode
    fps: int
    exposure_ms: int
    power_mode: PowerMode
    imu_env_rate_hz: int | None = None

    @property
    def dss_enabled(self) -> bool:
        return dss_enabled_for_fps(self.fps)


# Table 9 "Profile examples" anchors, exactly (Room Mapping, and the Precision
# preset's own min-distance/power-mode choice) or, for High Frame-Rate, the
# MEASURED hardware ceiling (2026-08-03, see module docstring): the original
# design ran this preset at 90 fps by construction, matching Table 9's 100 fps
# "Gaming" example's shape, but an on-target sweep found that config does not
# reproduce -- 90 fps and 100 fps requests are both accepted and both silently
# delivered as period-multiples (44.85 fps and 33.2 fps respectively), not the
# requested rate. High Frame-Rate is amended to 46 fps -- the measured 1x
# delivery ceiling at this preset's own 4 ms exposure -- keeping Precision
# context and Regular power; DSS is now ON (46 <= the 60 Hz DSS ceiling), which
# also raises this preset's max range from 5.0 m to 8.8 m (see MAX_RANGE_M).
PRESETS: dict[ProfileId, ProfileConfig] = {
    ProfileId.ROOM_MAPPING: ProfileConfig(
        ProfileId.ROOM_MAPPING, RangingMode.AMBIENT, 30, 6, PowerMode.ULTRA_LOW),
    ProfileId.PRECISION: ProfileConfig(
        ProfileId.PRECISION, RangingMode.PRECISION, 30, 10, PowerMode.ULTRA_LOW),
    ProfileId.HIGH_FRAMERATE: ProfileConfig(
        ProfileId.HIGH_FRAMERATE, RangingMode.PRECISION, 46, 4, PowerMode.REGULAR),
}


def manual_profile_config(params: ManualParams) -> ProfileConfig:
    return ProfileConfig(ProfileId.MANUAL, params.ranging_mode, params.fps,
                        params.exposure_ms, params.power_mode, params.imu_env_rate_hz)


# --- Consequence estimate ---------------------------------------------------------------


@dataclass(frozen=True)
class ProfileEstimate:
    """Full consequence estimate for one resolved configuration. `ok` is the
    single field a caller needs to gate on; `errors`/`warnings` explain why."""
    profile_id: ProfileId
    ranging_mode: RangingMode
    fps: int
    exposure_ms: int
    power_mode: PowerMode
    dss_enabled: bool
    frame_period_us: int
    i3c_xfer_ms: float
    i3c_bus_utilization_pct: float
    i3c_airtime_left_pct: float
    power_mw: float
    max_range_m: float
    min_distance_mm: float
    transport_warning: str | None
    imu_env_rate_hz: int | None
    imu_env_coupled: bool
    expected_delivered_fps: float
    warnings: tuple[str, ...] = ()
    errors: tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        return not self.errors


def estimate_profile(config: ProfileConfig, transport: str = "ethernet",
                     ambient_lux: float = AMBIENT_LUX_DEFAULT) -> ProfileEstimate:
    """Compute the full consequence estimate for a resolved (preset or manual)
    profile. `transport` is `"ethernet"`, `"cdc"`, or `"replay"` — it only feeds
    the non-blocking CDC-above-60-fps warning; range/power never depend on it.
    `ambient_lux` reaches `estimate_power_mw` and nothing else (it is a real term
    in ST's own power equations — the Vbat Rx branch scales with ambient light —
    so a caller that offers the knob must actually pass it through, not accept
    and drop it).

    Applies the SAME validation manual candidates get (Task 1 non-negotiable
    finding: this module is "the single host-side owner of range/power/bus
    math", not one formula for presets and another for Manual) — a preset can
    only fail this if `PRESETS` itself is wrong.
    """
    params = ManualParams(config.ranging_mode, config.fps, config.exposure_ms,
                          config.power_mode, config.imu_env_rate_hz)
    validation = validate_manual_params(params)

    dss = dss_enabled_for_fps(config.fps) if FPS_MIN <= config.fps <= FPS_MAX else False
    period_us = fps_to_period_us(config.fps) if FPS_MIN <= config.fps <= FPS_MAX else 0
    util_pct = i3c_bus_utilization_pct(config.fps)
    power_mw = estimate_power_mw(config.ranging_mode, config.power_mode,
                                 config.exposure_ms, config.fps, ambient_lux)
    max_range = MAX_RANGE_M.get((config.ranging_mode, dss), float("nan"))
    min_dist = estimate_min_distance_mm(config.ranging_mode)
    tw = transport_warning_message(transport, config.fps)
    delivered_fps = (expected_delivered_fps(config.fps, config.exposure_ms)
                     if FPS_MIN <= config.fps <= FPS_MAX else 0.0)

    warnings = list(validation.warnings)
    if tw:
        warnings.append(tw)

    return ProfileEstimate(
        profile_id=config.profile_id, ranging_mode=config.ranging_mode, fps=config.fps,
        exposure_ms=config.exposure_ms, power_mode=config.power_mode, dss_enabled=dss,
        frame_period_us=period_us, i3c_xfer_ms=round(I3C_XFER_MS, 3),
        i3c_bus_utilization_pct=round(util_pct, 1),
        i3c_airtime_left_pct=round(100.0 - util_pct, 1),
        power_mw=power_mw, max_range_m=max_range, min_distance_mm=min_dist,
        transport_warning=tw, imu_env_rate_hz=config.imu_env_rate_hz,
        imu_env_coupled=config.imu_env_rate_hz in (None, 0),
        expected_delivered_fps=round(delivered_fps, 2),
        warnings=tuple(warnings), errors=validation.errors)


def estimate_preset(profile_id: ProfileId, transport: str = "ethernet",
                    imu_env_rate_hz: int | None = None,
                    ambient_lux: float = AMBIENT_LUX_DEFAULT) -> ProfileEstimate:
    if profile_id not in PRESETS:
        raise ValueError(f"{profile_id!r} is not a preset (use estimate_manual for MANUAL)")
    base = PRESETS[profile_id]
    config = ProfileConfig(base.profile_id, base.ranging_mode, base.fps, base.exposure_ms,
                          base.power_mode, imu_env_rate_hz)
    return estimate_profile(config, transport=transport, ambient_lux=ambient_lux)


def estimate_manual(params: ManualParams, transport: str = "ethernet",
                    ambient_lux: float = AMBIENT_LUX_DEFAULT) -> ProfileEstimate:
    return estimate_profile(manual_profile_config(params), transport=transport,
                            ambient_lux=ambient_lux)


def estimate_to_json(est: ProfileEstimate) -> dict:
    """The JSON shape of an estimate, as the `/ws` `ranging` message and the MCP
    `profile_estimate()`/`rig_profile()` tools both emit it -- one owner, so a
    browser and an agent can never be told different things about one config."""
    return {
        "profile": PROFILE_ID_TO_STR.get(ProfileId(est.profile_id)),
        "ranging_mode": RANGING_MODE_TO_STR.get(RangingMode(est.ranging_mode)),
        "fps": est.fps,
        "exposure_ms": est.exposure_ms,
        "power_mode": POWER_MODE_TO_STR.get(PowerMode(est.power_mode)),
        "dss_enabled": est.dss_enabled,
        "frame_period_us": est.frame_period_us,
        "i3c_xfer_ms": est.i3c_xfer_ms,
        "i3c_bus_utilization_pct": est.i3c_bus_utilization_pct,
        "i3c_airtime_left_pct": est.i3c_airtime_left_pct,
        "power_mw": est.power_mw,
        "max_range_m": est.max_range_m,
        "min_distance_mm": est.min_distance_mm,
        "transport_warning": est.transport_warning,
        "imu_env_rate_hz": est.imu_env_rate_hz,
        "imu_env_coupled": est.imu_env_coupled,
        # Honest expected delivery rate (2026-08-03 measured hardware ceiling,
        # `expected_delivered_fps`): equals `fps` exactly at/below the exposure's
        # measured 1x ceiling; above it, the sensor still ACCEPTS the request but
        # delivers period-multiples, and this is what it actually delivers --
        # callers must show THIS, never just echo the request.
        "expected_delivered_fps": est.expected_delivered_fps,
        "warnings": list(est.warnings),
        "errors": list(est.errors),
        "ok": est.ok,
    }
