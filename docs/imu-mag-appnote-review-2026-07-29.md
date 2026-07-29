# IMU + magnetometer application-note review (2026-07-29)

"Are there tricks we should be using but aren't?" — a goal-directed pass over the LSM6DSV16X and
LIS2MDL documentation catalogued in `references/datasheets/NUCLEO-IKS4A1/{LSM6DSV16XTR,LIS2MDL}/index.md`,
aimed at the two open bugs (**BUG-030** magnetometer calibration, **BUG-031** ToF↔IMU timing skew) and
at handheld dynamic accuracy.

Sources trusted: AN5763 Rev 4 (text dump `/tmp/an5763.txt`), AN5069 Rev 4 (`/tmp/an5069.txt`),
DT0155 Rev 2 (PDF), DT0103 Rev 1 (PDF). **Do not trust the auto-generated `.md` companions in this
tree** — a DT0131 summary invents PCB keep-out content absent from the real PDF, and
`LSM6DSV16XTR/ds.txt` has garbled table columns.

---

## (A) Should adopt

### A1. LIS2MDL `BDU` — done, but honestly scoped
`CFG_REG_C` (62h) bit 4. AN5069 §6.4: "strongly recommended" when reads are not synchronised to
`Zyxda` or DRDY. Our sensor hub polls on its own 60 Hz cadence against the mag's 100 Hz ODR, so we are
exactly that case. AN5069's own init flow is "COMP_TEMP_EN, **BDU**, Continuous mode, enable offset
cancellation, ODR = 100 Hz" — we had every item except BDU. **Fixed in `46b81b3`, not yet flashed.**

**But the tempting hypothesis is refuted.** Torn reads (MSB from one sample, LSB from the next, worth
up to 255 LSB = 38 µT) would inject outliers a least-squares ellipsoid fit cannot defend against —
a plausible BUG-030 contributor. Measured on the 902 s stationary capture (27 339 mag samples):
max consecutive per-axis jump **23 LSB**, and **zero** jumps above 64 LSB. The 11 spike-and-return
events are 17–22 LSB on z, ≈4σ given z's 4.75 LSB RMS — ordinary Gaussian tail. The 6-byte hub burst
completes well inside the 10 ms ODR interval. So: keep the fix, but it is latent-risk insurance, not
a cause. Do **not** spend an owner physical action re-tethering SWD for it.

### A2. Magnetometer self-test — rules the part in or out, with no owner action
`CFG_REG_C` bit 1 (`Self_test`). AN5069 §12: a current is forced through an internal coil producing a
known field; the output becomes the algebraic sum of the external field and the self-test field.
Procedure: average 50 samples with self-test off, enable, discard the first sample, average 50 more,
and check the per-axis delta against the datasheet limits.

**Why this matters now:** every BUG-030 hypothesis so far assumes the sensor is healthy and correctly
scaled. This proves it without moving the rig — which is exactly the kind of check that is cheap
before a calibration session and expensive to omit. It needs a firmware pass (sensor-hub write to
`CFG_REG_C`, collect, restore) plus one flash.

### A3. DT0103 — resolve the rotational ambiguity that ellipsoid fitting leaves behind
**This is the most important finding of the review.** DT0103 distinguishes three error classes:
hard-iron (offset), soft-iron (gains + cross-axis), and **installation error** — the magnetometer's
axes being misaligned with the accelerometer's by a small roll/pitch. On the IKS4A1 the LIS2MDL and
LSM6DSV16X are separate packages on one PCB, so a residual misalignment of a fraction of a degree to
a couple of degrees is expected. `AXIS_CONVENTION = diag(1,−1,−1)` handles 90° permutations and sign
flips; it does **not** handle a residual angle.

Two consequences we were not accounting for:

1. **Installation error cannot change |B|** — de-rotating preserves magnitude. So it is *not* a
   BUG-030 candidate, but it *is* an independent heading error, and heading is what the owner sees.
2. **A general ellipsoid fit is ambiguous up to a rotation.** Fitting determines the ellipsoid's
   *shape*, but factoring it into a correction matrix leaves an arbitrary rotation branch. A fit can
   therefore produce a perfectly constant |B| while systematically rotating the field vector — right
   magnitude, wrong heading. Pure 3D ellipsoid fitting **cannot** detect this; DT0103 notes MotionMC
   likewise "compensates for hard-iron and soft-iron effects but **not** for installation error".

DT0103's fix uses the accelerometer (gravity) to pin the magnetometer frame to the body frame, which
3D fitting cannot do alone. Useful detail for our tumble UX: "The algorithm is able to complete the
computation even if the data does not describe a full circle… However, a full rotation does maximize
the quality." Its assumption to respect: the soft-iron matrix must be **diagonal** (cross-axis gains
zero) — our `fit_ellipsoid` produces a general matrix, so the two models are not directly composable
and this needs thought, not a copy-paste.

---

## (B) Worth evaluating

### B1. Seed the SFLP gyro bias at boot (AN5763 §6.5.1)
We already batch the gbias vector (FIFO tag 0x16, 4.375 mdps/LSB), so we can *observe* ST's live bias
estimate. §6.5.1 gives a 13-step procedure to *write* a previously computed bias into
`SFLP_GAME_GBIASX_L..GBIASZ_H`, in half-precision rad/s divided by a k factor (**0.00125 at 480 Hz**,
Table 48).

**Why it appeals:** we measured a multi-minute settling transient after every power cycle, during
which orientation measurements are untrustworthy (a first post-power-cycle pair of runs disagreed
24.7%). Persisting the last-known-good gbias and seeding it at boot should shorten that.
**Costs:** the procedure *forces an SFLP reset*, temporarily disables embedded functions, requires
`EMB_FUNC_DEBUG`, writes accelerometer data into `SENSOR_HUB_1..9`, and wants the I²C master off —
i.e. it transiently disturbs the sensor hub we depend on for the magnetometer. Non-trivial; measure
the settling benefit first.

### B2. Route FIFO watermark / overrun to an interrupt pin
`FIFO_CTRL1` sets the watermark; `FIFO_WTM_IA` and `FIFO_OVR_IA` (FIFO_STATUS2) can be driven to INT1
or INT2 (`INT1_FIFO_TH` / `INT1_FIFO_OVR`). Today we count overruns in software after the fact
(`g_lsm_fifo_ovr`) and drain once per ToF frame with ~2.3× headroom — one missed drain overruns.
An interrupt would let firmware drain *early* instead of losing samples, which matters during ToF
standby or a recovery stall. **Do not set `STOP_ON_WTM`** — it caps the FIFO at the watermark and
would discard the newest data, the opposite of what we want.

### B3. Revisit the gyro LPF1 bandwidth for handheld use
LPF1 at 28.4 Hz was chosen to cut *stationary* noise, and it costs −50.7° of phase at 20 Hz. Now that
we batch raw gyro and intend to fuse host-side, a wider bandwidth may trade better: host fusion can
filter with knowledge of the motion, which a fixed hardware corner cannot. Note AN5763 says
ODR-triggered mode matches high-performance noise, and that LPF1 exists only in high-performance mode.
Needs a measurement during real motion, so it is blocked on an owner pan.

---

## (C) Considered and rejected / blocked

### C1. DT0155 ODR-triggered sync — blocked by SFLP incompatibility
This *looked* like the clean root-cause fix for BUG-031. The LSM6DSV16X can phase-lock its sampling to
an external reference on **INT2**, aligning frequency *and* phase, at no noise cost ("same as
high-performance mode", AN5763 §3.2). The arithmetic fits us almost perfectly: a reference derived from
ToF frame-ready at 15 Hz (every second frame, 66.7 ms — above the 40 ms minimum `T_ref`) with
`ODR_TRIG_N_ODR` = 32 samples yields exactly our 480 Hz. Both sensors would then share one MCU-derived
timebase, and both the ~890 µs skew and the residual 3345 ppm clock mismatch would vanish.

**But AN5763 §3.3: "ODR-triggered mode is not compatible with the pedometer, relative tilt, SFLP, DRDY
mask, or activity/inactivity functionality."** It would cost us the SFLP quaternion — our drift-free
attitude reference and what SLAM consumes. Also needs INT2 wired to an MCU timer output (physical
work; DT0155 Fig. 3 shows CN9 pin 5 / JP3 pin 1 on the IKS4A1), and must be enabled only in
power-down mode.

**Architectural fork worth recording:** if `roomscan.imufusion` ever matures enough to replace SFLP
entirely, ODR-triggered mode becomes available and BUG-031 dissolves in hardware. Until then it is
mutually exclusive with the attitude source we depend on.

### C2. LIS2MDL hardware offset registers (`OFFSET_X/Y/Z_REG`, 45h–4Ah)
Would move the hard-iron offset into the device. No benefit over our software offset: at ±50 gauss FS
and 1.5 mG/LSB there is no saturation or dynamic-range pressure from a ~65 µT offset, and splitting
the correction across hardware and software would complicate the fit and its validation.

### C3. DT0104 — magnetometer as a virtual gyroscope
Computes ω from dB/dt for spin rates beyond a real gyro's range. We have a real gyro well inside its
±500 dps range. No application here.

### C4. DT0105 — 1-point / 3-point tumble calibration
Static tumble against 1 g, aimed at accelerometer zero-g offset and scale. Strictly weaker than the
DT0059 ellipsoid fit we already run for the magnetometer; potentially of minor interest for
accelerometer calibration, which is not a limiting error for us.

---

## Bearing on BUG-030 specifically

Ruled **out** by this review: byte tearing (measured, A1), installation error (cannot change |B|, A3),
and axis convention (an orthogonal sign matrix cannot change magnitude).

Still live, and consistent with a max/min |B| ratio of **85.1 / 47.4 = 1.80**:
- a **soft-iron / diagonal-gain error** the current fit mis-estimates, which would follow the device
  anywhere; or
- a **world-fixed interferer** — most likely the tripod, since tilting on it *translates* the sensor
  through an arc past ferrous mass, changing the field at the sensor with position rather than
  orientation. This also explains the mean (~66 µT measured against 49.87 fitted) if the original
  calibration was tumbled by hand in free space.

Note a simple hard-iron residual is **arithmetically excluded**: reaching 85 µT from a ~50 µT field
needs |δ| ≈ 35 µT, which would drive the minimum to ~15 µT; the measured minimum is 47.4.

**Practical consequence:** calibrate **hand-held in open space, away from the tripod**. Calibrating
while mounted would bake a position-dependent error into the fit that then misbehaves in handheld use,
which is the actual use case. A 2-minute hand-held tilt check (level / 45° / vertical) discriminates:
flat |B| means the tripod was the culprit; a persistent ~1.8:1 swing means a soft-iron/model problem.
