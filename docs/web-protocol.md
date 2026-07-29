# The `roomscan-web` `/ws` protocol

The desktop panel talks Python-to-Python; the web app talks **browser ↔ FastAPI over a single
WebSocket** at `/ws`. That contract has grown one message type at a time across Web Phases 1–3, and
it lives entirely inside `host/src/roomscan/web.py` builder functions — there is **no enum registry**
the way the *binary wire* protocol has `docs/protocol.md` + `protocol.py`. This doc is that missing
index: every `/ws` frame, its shape, and where it is built/consumed, so the next phase (SLAM
trajectory + mesh) has one place to hook into.

This is the **app protocol** (host ↔ browser). It is unrelated to the **device wire protocol**
(MCU ↔ host, `docs/protocol.md`). The two only meet at the reader thread: device frames are decoded,
transformed, and *re-encoded* into these `/ws` messages.

**Governing design specs** (the `§N` refs in `web.py` docstrings point at Phase 1's):
`docs/superpowers/specs/2026-07-15-web-phase1-core-instrument-design.md` (binary tags + metrics/state/cmd/log/event),
`.../2026-07-16-web-phase2-sensors-design.md` (`sensor`),
`.../2026-07-16-web-phase3-recording-playback-design.md` (`session`/`captures` + inbound transport).

## Framing

One socket, two encodings, distinguished by WS frame opcode:

- **Binary frames** — high-rate render payloads. First 4 bytes are a little-endian `u32` **tag**;
  the frontend switches on it (`ws.js`). Tags: `TAG_POINT_CLOUD = 1`, `TAG_IR_IMAGE = 2`,
  `TAG_MESH = 3`, `TAG_SURFACE = 4`, `TAG_MAGPOSE = 5` (`web.py`). Add new binary tags here and keep
  them contiguous, and in lockstep across `web.py` / `ws.js` / this table (the `protocol-change` skill).
- **Text frames** — JSON objects, always with a `"type"` string field. Everything that isn't a
  per-frame render payload (metrics, sensors, UI state, control echoes, logs) is JSON.

All numeric binary fields are little-endian. JSON numbers go raw over the wire; the frontend formats
for display (units, precision).

## Outbound — server → browser

### Binary

| tag | name | layout | built by |
|-----|------|--------|----------|
| 1 | `POINT_CLOUD` | `u32 tag · f32[3N] positions · f32[3N] colors` (positions then colors, concatenated) | `pack_point_cloud` |
| 2 | `IR_IMAGE` | `u32 tag · u16 width · u16 height · u8[w*h*3] RGB` | `pack_ir_image` `web.py:212` |
| 3 | `MESH` | `9×u32 header (tag, mesh_seq, flags, then 6 counts) · per-submesh f32 pos·f32 col·u32 idx · floor f32 pos·u32 line-idx` | `pack_mesh` (web Phase 4) — a SLAM `MeshPacket`; flags bit0=decimated, bit1=walls_split; emitted on the mesh-throttle cadence only |
| 4 | `SURFACE` | `u32 tag · u16 w · u16 h · u32 n_tris · f32[3·W·H] positions · f32[3·W·H] colors · u8[W·H] valid · u32[3·T] tri_indices · u8[W·H] covered` | `pack_surface_cloud` — grid-ordered positions+colors + triangulated mesh indices; sent instead of POINT_CLOUD when `surface_enabled` is true |
| 5 | `MAGPOSE` | `u32 tag · u32 seq · f32[4] quat(w,x,y,z) · f32[3] field_dir_body · f32[3] gravity_body · f32 field_ut · f32 dev_pct · f32 dip_deg · i16 live_cell · i16 filled_cell · u16 flags · u16 pad` — **68 bytes**, ~2.0 kB/s at 30 Hz | `pack_magpose` / `build_magpose` (magcal 3D feedback, 2026-07-29) — sent **only to `state.magcal_clients`** on the 30 Hz broadcaster tick. `live_cell`/`filled_cell` are `-1` for none; `dev_pct`/`dip_deg` may be `NaN` (no calibration / no gravity). `flags`: bit0 collecting, bit1 stationary, bit2 mag_anomaly *(reserved, Phase 2)*, bit3 have_quat, bit4 provisional_binning, bit5 sample_rejected. See "Magnetometer sweep" below |

`POINT_CLOUD` (or `SURFACE` in surface mode) goes out every broadcast tick (so late joiners see data within ~36 ms);
`IR_IMAGE` rides a slower cadence (`web.py:855`, `:869`).

**Frame of reference — positions are gravity-aligned, not sensor-frame.** `POINT_CLOUD`/`SURFACE`
positions are pre-multiplied server-side by `display_rotation(fused_quat)` — the canonical
`T_WORLD_TO_CV @ R @ T_CV_TO_BODY` sandwich from `docs/coordinate-frames.md`, the same matrix shipped
as the `sensor` message's `rot` — so the scene reads upright however the board is held (desktop-panel
orbit parity). `IR_IMAGE` is rolled to match, snapped to the nearest 90° via `ir_gravity_rot`, so
**`width`/`height` swap on a 90°/270° roll** (54×42 ↔ 42×54): clients must size from the message and
follow its aspect, never assume landscape. With no orientation yet (ToF-only session, or before the
first stream-9 sample) both fall back to the raw sensor frame — identity rotation, zero turns.
The client applies **no** further rotation; doing so would double-count.

The display quat is **smoothed** first (`OrientationSmoother`). Raw fused orientation carries ~0.14°
mean / 0.25° p95 of zero-mean noise per update, which a 3 m lever arm turns into visible shimmer at the
cloud edges; a magnitude deadband can't suppress it without also dragging on a slow deliberate pan
(~0.7°/frame), so the gate is directional **coherence** over a trailing window — the same discriminator
as the SLAM stationarity hold, sharing `roomscan.motion.coherence`. Measured on the live rig
(2026-07-28): 1.72 mm → 0.34 mm mean edge motion at 3 m, with coherent motion still tracked 1:1.
Smoothing is **display-only** — SLAM reads `sensor_state.fused_quat()` directly, so it can never reach
the reconstruction, and the `sensor` message's `rot` stays **raw** so the gizmo remains an honest
sensor readout.

**Raw orientation numerics + jitter (owner ask, 2026-07-28).** The `sensor` message also carries
`orientation_raw` (the quaternion + Euler angles + heading at full precision) and `jitter` (rolling
frame-to-frame noise stats). Both are **pre-`OrientationSmoother`** — the same raw
`sensor_state.fused_quat()` SLAM sees, not the shimmer-suppressed display quat above — and computed
server-side from **full-precision** internal values, never from `rot`/`heading`: those are rounded
(5dp / 1dp) for the wire, which censors changes below ~0.0006°/0.05° — the same rounding-hides-the-signal
trap `docs/iks4a1-stacking.md` → "Orientation-noise pass" already hit once, here avoided by computing
jitter off the unrounded values before they're rounded for the wire. `orientation_raw.quat` is
`[w,x,y,z]` at 6dp as received (not re-normalized — "raw" means raw);
`roll_deg`/`pitch_deg`/`yaw_deg` are `sensors.quat_roll_deg`/`quat_pitch_deg`/`quat_yaw_deg` (ZYX
Tait-Bryan) at 4dp; `heading_deg` is `absolute_heading` (same math as `heading` above, just unrounded).
All four are `None` until the first stream-9 sample.

`jitter` is `{"window_s": 5.0, "<signal>": {"mean_deg", "p95_deg", "n"}, ...}` for
`roll`/`pitch`/`yaw`/`heading`/`orientation`, tracked by `web.OrientationJitter` over a trailing
`SENSOR_JITTER_WINDOW_S` (named constant, `web.py`) window of `sensor` messages (~75 samples at the
15 Hz `SENSOR_INTERVAL`). Each stat is the **absolute** frame-to-frame change (a magnitude, not a
signed drift): `roll`/`pitch`/`yaw`/`heading` diff with `sensors.wrap180` so a heading crossing
360°→0° doesn't register as a 360° jump; `orientation` is the quaternion dot-product angle
(`quat_angle_deg`) between consecutive **re-normalized** quats — skipping the re-normalization lets a
not-quite-unit float32 quat push `|dot|` past 1.0, get clamped to exactly 1.0, and silently report a
**zero** angle for the smallest real steps (this bit the 2026-07-28 measurement). **`p95_deg` is the
headline statistic** (measured CV 1.45%, the most stable of mean/median/p95 on this signal); `mean_deg`
is secondary; **median is deliberately not reported** — quantization ties pile up at near-zero steps
and make it the least stable (CV 4.41%). `n < 2` reports `mean_deg`/`p95_deg` as `None` (not enough
samples yet), not `0.0`.

**Orientation decomposition modes + labels (owner ask, 2026-07-28; world mode follow-up same day).**
Roll/pitch/yaw are not always valid names for the axes — they presume a fixed body "forward" a handheld
scanner doesn't have — and the ZYX Tait-Bryan decomposition above has a gimbal lock (pitch → ±90°) that
bites at exactly the attitudes a handheld device visits (measured live at ~86° pitch: roll/yaw both went
degenerate). `sensor.orientation_view` reports the **selected** decomposition of the same
`sensor_state.fused_quat()` — still presentation-only, still never touching `display_rotation`/
`fused_quat()`/the SLAM path. Modes (`web._VALID_ORIENTATION_MODES`, math in `sensors.py`'s "Alternate
orientation decompositions" section):

| mode | reports | singularity |
|------|---------|-------------|
| `zyx` (default) | the same numbers as `orientation_raw` (`quat_roll_deg`/`quat_pitch_deg`/`quat_yaw_deg`) | `pitch` → ±90° — body **X (Up)** axis → world vertical |
| `zxy` | alternate Tait-Bryan (`quat_roll_alt_deg`/`quat_pitch_alt_deg`/`quat_yaw_alt_deg`, sequence `R = Rz·Rx·Ry`) | `pitch` → ±90° — body **Y (Right)** axis → world vertical (a disjoint attitude from `zyx`'s lock — device rolled onto its side, not aimed steeply up/down) |
| `boresight` | azimuth/elevation/roll of the ToF optical axis (`boresight_view_deg`) — meaningful under any grip because it reports where the *sensor* points, not an arbitrary body "forward". The boresight is body **+Z**: `docs/coordinate-frames.md` "The four frames" lists SFLP body Z=Forward, and `T_CV_TO_BODY` maps the ToF (CV) frame's Z=Forward onto body Z unchanged, so body Z is the optical axis under both frames' own definitions | `elevation` → ±90° — pointing straight at the ceiling/floor |
| `world` | gravity+magnetometer TRIAD/eCompass reference (owner: "could we use gravity + the magnetometer..."): `pitch_deg`=tilt-of-the-boresight-from-horizontal, `yaw_deg`=heading-from-magnetic-north (reuses `absolute_heading` verbatim, not re-derived), `roll_deg`=twist about the boresight referenced to true vertical — the gravity-only half of TRIAD (`triad_roll_deg`), no mag/gyro involved. Drift-free and **grip-independent**, but a poor *dynamic* tracker: linear acceleration is indistinguishable from gravity tilt, so `pitch_deg`/`roll_deg` degrade badly while the device is moving — that dynamic weakness is exactly why the gyro + complementary filter drive the primary orientation instead. Not "better", just a different, absolute-referenced view | `pitch_deg` (tilt) → ±90° — boresight → vertical, where the TRIAD roll's perpendicular reference collapses |

`orientation_view` shape: `{mode, labels[3], roll_deg, pitch_deg, yaw_deg, singularity_margin_deg,
near_singularity, valid, reason}` plus, in `world` mode only, `gravity_source`("imu_raw"|"quat"),
`mag_norm_ut`, `mag_expected_ut`, `motion_stable`. `singularity_margin_deg` is `90 - |the mode's singular
component|`; `near_singularity` fires within `ORIENTATION_SINGULARITY_MARGIN_DEG` (15°, the same margin
`YawFusion.gimbal_margin_deg` already gates its own gimbal-lock freeze on — reused, not reinvented). In
`world` mode, `valid`/`reason` gate the **mag-dependent `yaw_deg` (heading) only** — `roll_deg`/`pitch_deg`
(gravity-only) are reported regardless, their own trustworthiness being `near_singularity`, not `valid`.
`valid` is false when either (a) the calibrated+axis-corrected mag magnitude deviates from the fitted
`field_ut` by more than `WORLD_MODE_MAG_ANOMALY_FRAC` (0.3, reusing `YawFusion.anomaly_frac`'s notion —
on the live rig this reads ~107-109 µT against a 49.87 µT fit, a ~2.15× anomaly, so World mode is
currently WRONG and the UI must show that, not a confident bad number) or (b) `imu_raw`'s accel batch
norm strays >`WORLD_MODE_ACCEL_TOL_G` (0.15 g) from 1 g (device accelerating). `gravity_source` is
`"imu_raw"` when the stream-11 SFLP gravity FIFO tag (0x17, fixed ±2g/0.061 mg-LSB, ~16× finer in tilt
than the fp16-encoded SFLP quaternion) was available, else `"quat"` (the `R.T @ [0,0,-1]` fallback
`ir_gravity_rot` already uses). `labels` are the user-renamable slot names (`orientation_labels` below) —
positional, same 3 slots regardless of mode.

`orientation_raw`/`jitter`'s `roll`/`pitch`/`yaw` fields are **unaffected** — always ZYX, exactly as
before (back-compat); it is `orientation_view` and `jitter`'s `roll`/`pitch`/`yaw` entries that switch
meaning with the mode. `jitter.orientation` (quat dot-product) and `jitter.heading` (magnetic) are
**convention-independent** and always mean the same thing — the trustworthy signals when comparing across
a mode switch. On a mode change, `OrientationJitter` purges only the `roll`/`pitch`/`yaw` window entries
(and resets their running previous-value state) — diffing a `zyx` roll against a `boresight` roll from the
sample before the switch would be nonsense.

**"Zero yaw here" (owner ask, 2026-07-29: "Yaw is at -43 degrees. Wish I could reset that").** The SFLP
quaternion is a **game rotation vector** — accel+gyro only, no magnetometer — so its yaw origin is
whatever attitude the sensor happened to be at power-on/init, and it free-runs with gyro drift; roll/pitch
are gravity-referenced and absolute, yaw is not. `UiState.yaw_offset_deg` lets the user pin a new zero:
`zero_yaw` (inbound, below) captures the **currently displayed mode's** `yaw_deg` and stores the
`sensors.graft_yaw` delta that would zero it (`web._YAW_GRAFT_SIGN` — `zyx`/`zxy` need the *negative* of
the captured yaw, `boresight`'s azimuth has the opposite handedness and needs the *positive*; derived
numerically against the real decompositions, not assumed). Every subsequent `sensor` message applies that
delta via `graft_yaw` to a **display-only copy** of the quat before it is decomposed — never to
`sensor_state.fused_quat()` itself — so `rot`/`heading`/`orientation_raw`/`jitter.orientation` and
everything SLAM consumes are computed from the untouched quat exactly as before (same guarantee as the
mode picker above). `graft_yaw` is used deliberately instead of adding/subtracting degrees from a Euler
field: it rotates about world +Z, which providably preserves roll/pitch/tilt, so the value fed into
`orientation_view` stays a valid rotation. **World mode is excluded**: its `yaw_deg` is the *absolute*
magnetic `heading_full` (`absolute_heading`), not a function of the quat's own yaw component, so grafting
it would be meaningless — the offset is force-zeroed in `orientation_view_out` whenever `mode == "world"`,
`zero_yaw` no-ops (logs a warning) while World is selected, and the Sensors card disables the "Zero Yaw
Here" button for the same reason (`sensors.js`'s `isWorld` guard). Because the offset is a **constant**
per session, it cancels exactly in any frame-to-frame diff — `jitter`'s roll/pitch/yaw magnitudes (p95/
mean) are unaffected by whether an offset is set, only the absolute reading shifts. The offset is echoed
in `state.yaw_offset_deg` (0.0 when none) and persisted to `roomscan.toml`'s `[viewer]` table exactly like
`orientation_mode`/`orientation_labels`.

**Magnetometer sweep + calibration quality (owner ask, 2026-07-29: "visualize ideal and current
magnetometer sweeping ... help the user understand what angles they're missing, and show calibration
quality").** The `magcal` message/action pair backs the Sensors card's **Calibrate Mag** modal
(`static/magcal.js`, math in `roomscan/magsweep.py`, fit reused verbatim from `magcal.fit_ellipsoid`).

*Why it exists.* A correct calibration reports a **constant |B| in every orientation**. The
calibration shipped 2026-07-15 did not — across a deliberate tilt sweep it read 50.5 µT at 0° tilt
(ceiling-facing) rising monotonically to 85.1 µT at 90° (horizontal), ~1.7×, producing heading errors up
to ~90° in exactly the wall-scanning attitude the scanner is used in. That is the signature of an
incomplete tumble, and nothing in the pipeline measured it. The modal therefore (a) shows missing
coverage **during** collection and (b) forces an explicit quality preview before a fit can overwrite the
saved file.

*Binning — a Fibonacci sphere lattice, not lat/lon.* Cells are the Voronoi cells of a
`SPHERE_CELLS = 92`-point Fibonacci (golden-spiral) lattice; a sample lands in the cell whose direction
it is closest to (one argmax of dot products — no wrap/branch logic). Lat/lon bins are rejected because
their areas differ by ~50× between pole and equator, so a lat/lon "coverage %" over-weights the poles
and would report *covered* for a tumble that missed most of the sphere — the exact misjudgement that
produced the bad calibration. Fibonacci cells are near-equal-area (nearest-neighbour spacing varies
< 1.2×, ~20°), so *fraction of cells occupied* **is** *fraction of the sphere covered*, and a K-cell gap
means the same thing wherever it sits. 92 cells ⇒ ~12° cell radius: fillable by a careful hand tumble,
coarse enough that a whole missing attitude family is many cells wide.

*What is binned.* The **calibrated, body-frame** field direction
(`AXIS_CONVENTION @ cal.apply(raw)`), **not** the raw direction. This is load-bearing: the rig's
hard-iron offset (~65 µT) exceeds the field (~50 µT), so raw sample vectors live in a cone around the
offset and their directions cover barely half the sphere however thoroughly you tumble — binned raw,
coverage could never reach 100%. Binned calibrated, a cell means "the device was held such that the
field entered it from this direction", which is both actionable and what the ellipsoid fit must span.
Precedence for the binning calibration is candidate → saved → `magsweep.provisional_calibration` (a
bounding-box hard-iron estimate used only for display on a first-ever tumble, never fitted, never saved);
the message reports which was used in `binning`.

*Quality metrics and their thresholds* (all computed server-side; the components are always shipped
alongside the headline, deliberately — a single opaque score is what let the bad calibration through):

| metric | definition | good | marginal | bad |
|---|---|---|---|---|
| **field spread** (headline) | `std(|B|)/mean(|B|)` over all samples after applying the calibration (`std_pct`) | < 2% | < 5% | ≥ 5% |
| **field bias** | `(mean(|B|) − field_ut)/field_ut` (`bias_pct`) | < 2% | < 5% | ≥ 5% |
| **coverage** | fraction of the 92 cells holding ≥ 1 sample | ≥ 85% | ≥ 60% | < 60% |
| **samples** | raw sample count | ≥ 300 | ≥ 100 | < 100 |

`field.verdict` is the worse of spread and bias (`spread_verdict`/`bias_verdict` are both shipped), and
the block's headline `verdict` is the worst of field / coverage / samples, with `limited_by` naming the
driving component and `reason` phrased as **what to do about it** — the three failures need opposite
responses (a spread/bias failure means the calibration is wrong; a coverage/sample failure means the
*measurement* is not yet conclusive, and "limited by coverage" alone reads as an indictment of the file
when it is really an indictment of the sweep). `field` also carries `min_ut`/`max_ut`/`ratio` (the ×1.7
the tilt sweep exposed) and `residual_rms_ut` (RMS deviation from the sphere of radius `field_ut`, which
combines both failure modes: `sqrt(std² + bias²)`). The 2% bar matches the existing convention in
`docs/yaw-fusion.md` ("< 0.02 is a clean fit").

Two traps the component split exists to close, both observed for real:
- **Spread alone is not sufficient.** Driving the modal on-rig on 2026-07-29, 255 stationary samples read
  |B| = 101.96 µT against the saved calibration's own `field_ut` of 49.87 µT — a ×2.04 bias — yet scored
  `std_pct = 0.22%`. A perfectly self-consistent calibration at completely the wrong magnitude must not
  score `good`, hence `bias_pct`.
- **Coverage must be read next to field, never folded into it.** A calibration fitted from a cap of the
  sphere is perfectly self-consistent *over that cap* and would score `field: good` on its own samples —
  which is exactly how the 2026-07-15 calibration passed unnoticed.

*Surfacing the existing defect.* `cell_dev_pct[i]` is the mean signed |B| error in cell *i* as a percent
of `field_ut`, computed under whichever calibration `view` selects. Point `view` at `current` and the
saved calibration's direction-dependent error becomes a picture: a hue that changes across the map **is**
a direction-dependent calibration error.

*Workflow and safety.* `start` → live coverage/quality → `stop` (fits a **candidate**, in memory only) →
preview → `save` or `discard`. `save` is the only action with a side effect: it writes to
`ViewerConfig.mag_cal_path` (default `mag_cal.json`, resolved once in `main()` so the web modal, the
loader and `panel.py:440` all agree on the file) in the same `{offset, matrix, field_ut}` format
`tools/mag_calibrate.py` writes, then hot-reloads via `web.install_mag_calibration` — which updates
`state.mag_cal`, the `YawFusion` filter's own captured `cal`, **and** calls `reset_fusion()` so the yaw
delta re-snaps instead of low-passing a stale one. No server restart is needed. `save` with no candidate
is refused, so the saved calibration can never be replaced by a fit nobody previewed; `discard` keeps the
samples so the user can keep tumbling into the same cloud.

*Isolation.* Everything except `save` is pure observation. Collecting/fitting/previewing/discarding —
**including the 30 Hz `MAGPOSE` stream** — leave `display_rotation`, the point-cloud bytes, `fused_quat()`
and the loaded calibration bit-identical; guarded by
`tests/test_magsweep.py::test_magcal_preview_does_not_touch_display_path`, which drives a full
open → pose-stream → close cycle. The live view keeps streaming while the modal is open (the modal is an
overlay, not a mode).

**3D feedback — the two-channel split (owner ask, 2026-07-29; design
`docs/superpowers/specs/2026-07-29-magcal-3d-feedback-design.md`, Phase 1).** The modal's hero is now a
WebGL "coverage shell" (`static/magcal3d.js`), with the 2D Lambert disc pair kept as the **fallback**
renderer. Two things follow for this protocol:

*The split.* Raising the `magcal` JSON to 30 Hz would be ~60 kB/s and 30 × `JSON.parse` per second of
UI-thread work for data (cell counts, verdicts, coverage) that changes at **human** speed — the wrong axis
to scale. Instead the channel is split by rate of change, exactly as the rest of the app splits it:
the **binary `MAGPOSE` tag 5** carries pose + field direction + gravity + live cell at 30 Hz (68 B), and
the **`magcal` JSON** stays the 5 Hz truth. `filled_cell` is what lets the JSON stay slow: the fast channel
carries the *delta* ("this sample just lit cell 47") so a cell goes solid the instant it fills, while the
slow channel reconciles the counts (`MagSweepSession.sync_occupied`). Binary rather than a small 30 Hz
JSON because orientation must not be measured off a rounded decimal — `sensor.rot` is 5 dp; f32 avoids
inventing a second rounding policy.

*Frames — what the client is allowed to compute.* Per `docs/coordinate-frames.md` and the "server-side
math stays server-side" invariant, **no sign or permutation matrix is ever written in JS**. The body-fixed
hero needs **no transform at all** (`cell_dirs`, `field_dir_body`, `gravity_body` are already SFLP-body
unit vectors — which is also why it renders correctly on a session with no stream 9). The world-fixed
Steering widget maps a body vector to the renderer's world as `T_WORLD_TO_CV · R · v_body` — note **not**
the `T_WORLD_TO_CV · R · T_CV_TO_BODY` sandwich, which maps *CV* points; ours are already body points, so
the `T_CV_TO_BODY` leg is absent. `T_WORLD_TO_CV` is shipped once as `t_world_to_cv[9]` (row-major) on
`open` and applied as a static matrix; the only per-frame client math is `quat → Quaternion` and a slerp.
The steering rotation likewise arrives as an explicit body `guidance_axis.axis` + `angle_deg`
(`axis = unit(t × d)`, `angle = acos(t·d)`, `magsweep.rotation_to`), so the client never re-derives it.

*Guidance has no dip or compass assumption.* The old text ("point the Top face toward magnetic north and
downward") assumed northern-hemisphere dip *and* that the user knows where north is. It is replaced by the
exact body-axis rotation above, plus a countdown of the cells left in the gap being steered at.

### JSON (`type` → shape)

| type | key fields | built by | notes |
|------|-----------|----------|-------|
| `metrics` | `render_fps`, `streams[]{stream_id,label,device_hz,host_hz,bytes_per_s,jitter_ms}`, `link_bytes_per_s`, `resources`(null), `drops`, `gaps` | `build_metrics_message` `web.py:221` | metrics cadence; `device_hz`/`jitter_ms` may be null |
| `sensor` | `have_quat`, `rot`[9 row-major], `heading`, `pressure_pa`, `temp_c`, `mag_ut`[3], `fusion`(human label), `fusion_key`(raw status), `has_mag_cal`, `pressure_hist[]`, `temp_hist[]`, `orientation_raw`{quat[4],roll_deg,pitch_deg,yaw_deg,heading_deg}, `jitter`{window_s,roll/pitch/yaw/heading/orientation→{mean_deg,p95_deg,n}}, `orientation_view`{mode,labels[3],roll_deg,pitch_deg,yaw_deg,singularity_margin_deg,near_singularity,valid,reason,yaw_offset_deg,...} | `build_sensor_message` `web.py:246` | **None (silent) on a ToF-only session**; `rot`/`heading` computed server-side so the frontend never re-derives sign/permutation matrices — see `docs/coordinate-frames.md`. `fusion` is a human-friendly label (`_FUSION_LABELS`); `fusion_key` is the raw `YawFusion.status` (`off`/`init`/`active`/`gated:no-cal`/`gated:gimbal`/`gated:motion`/`gated:anomaly`). `pressure_hist`/`temp_hist` are decimated (1 sample/2 s, ~10 min window) for sparklines. `orientation_raw`/`jitter` add full-precision numerics + noise stats alongside the existing (also raw, just rounded) `rot`/`heading` — see the frame-of-reference section above; `rot`/`heading` themselves are unchanged by this addition. `orientation_view` is the selected-mode decomposition + singularity/validity — see "Orientation decomposition modes" above; `yaw_offset_deg` echoes the applied "Zero yaw here" offset (0.0 in World mode, where it never applies) — see "Zero yaw here" below |
| `state` | `color_mode`, `ir_colormap`, `ir_freeze`, `view_colormap`(turbo\|gray), `point_size`, `point_size_auto`, `surface_enabled`, `surface_mode`(grid\|spatial), `surface_threshold_pct`, `mode`(realtime\|slam), `slam_trajectory`, `slam_walls`, `slam_follow`, `idle_enabled`, `idle_level`(soft\|hard), `orientation_mode`(zyx\|zxy\|boresight\|world), `orientation_labels`[3], `yaw_offset_deg` | `_state_message` | echoed after every `set_color`/`set_ir`/`set_view`/`set_mode`/`slam_opt`/`set_idle`/`set_orientation`/`zero_yaw`/`clear_yaw_offset` (one-way flow) **and** sent first on connect. Web Phase 5: the display prefs (all but `mode`) + the two sensor auto-idle prefs are seeded from + persisted to the shared `roomscan.toml` [viewer]` table (`web.ui_from_config`/`_persist_ui`), so a reconnect **and** a server restart bring a tab current; `mode` is not persisted (SLAM arms lazily → restart is always real-time). `orientation_mode`/`orientation_labels` (owner ask, 2026-07-28) follow the same persisted-pref pattern — `orientation_labels` round-trips to `roomscan.toml` as a single comma-joined string (`ViewerConfig.orientation_labels`), the flat-TOML writer being scalar-only. `yaw_offset_deg` (owner ask, 2026-07-29) is the raw "Zero yaw here" delta, also persisted (`ViewerConfig.yaw_offset_deg`, a plain float) — see "Zero yaw here" above |
| `session` | `mode`(live\|replay), `source_label`, `has_live`, `recording{active,path,elapsed_s,bytes}`, `playback{is_replay,capture_name,paused,speed_fps,loop,position,total_frames}` | `build_session_message` `web.py:400` | broadcast on change **and** on the metrics cadence (so timer/position tick) |
| `captures` | `items[]{name,bytes,mtime}` (newest first) | `build_captures_message` `web.py:354` | on connect, on `list_captures`, after a recording stops |
| `slam` | `pose`[16], `follow{eye,center,up}`, `traj_tail[][3]`, `traj_len`, `fitness`, `rmse`, `tracking_lost`, `slam_ms`, `frames_integrated`, `mesh_seq`, `mesh_verts` | `build_slam_message` (web Phase 4) | every processed frame in SLAM mode; follow eye/center/up computed server-side; traj downsampled to ≤256 |
| `saved` | `items[]{name,bytes,mtime}` (newest first) | `build_saved_message` (web Phase 4) | `results/*.ply`; on connect and after a Save completes |
| `magcal` | `collecting`, `sample_count`, `elapsed_s`, `cells`(92), `cell_counts`[92], `cell_dev_pct`[92] (null = empty cell), `view`(current\|candidate), `live_cell`, `live_dir`[3], `gaps[]{size,fraction,centroid[3],face}`, `guidance`, `guidance_axis`{axis[3],angle_deg,text,target[3],target_cell,region_size,from_face,to_face}, `live_fit`{samples,used,field_ut,std_pct,bias_pct,residual_rms_ut,spread_verdict,bias_verdict,verdict,error}, `motion`{stationary,spread_deg,window_s,n}, `has_current`, `has_candidate`, `binning`(candidate\|current\|provisional\|raw), `fit_error`, `current`/`candidate`→{samples,samples_verdict,field{mean_ut,std_ut,std_pct,min_ut,max_ut,ratio,residual_rms_ut,expected_ut,bias_pct,spread_verdict,bias_verdict,verdict},coverage{cells,occupied,empty,fraction,verdict},verdict,limited_by,reason}, `current_field_ut`, `candidate_field_ut`, `saved_path`; **on `open` only**: `cell_dirs`[92][3], `t_world_to_cv`[9] (row-major) | `magsweep.build_report` via `web._magcal_report` | **Per-tab, not broadcast**: sent at `MAGCAL_INTERVAL` (**5 Hz**) only to sockets in `state.magcal_clients` (i.e. tabs that sent `magcal/open`), plus immediately on `open` and after every state-changing action. A session with the modal closed everywhere costs nothing. `cell_dirs`/`t_world_to_cv` are deterministic constants and ride the `open` report ONLY (4490 B → 1982 B per tick, a 56% cut); the client caches them. The 30 Hz render payload is the binary `MAGPOSE` channel, not this one. See "Magnetometer sweep" below |
| `event` | `code`, `detail`, `msg` | `classify_bus_line` `web.py:142` | from a device EVENT bus line |
| `cmd` | `label`, `status`(ok\|busy\|timeout\|error), `detail` | `classify_bus_line` `web.py:151` | command-result echo; `status` via `_cmd_status` `web.py:156` |
| `log` | `line` | `classify_bus_line` `web.py:145,153` | catch-all bus line |

`event`/`cmd`/`log` are **all produced by one classifier**, `classify_bus_line` (`web.py:123`), which
reads raw reader/dispatcher bus lines and tags them. A free-text line that happens to contain ` -> `
is gated against `command_labels` (labels we actually dispatched) so it can't be mis-tagged as a `cmd`.

## Inbound — browser → server

All inbound is JSON with a `"type"`; routed by `_handle_inbound` (`web.py:942`). Unknown types warn
and are dropped. The `record`/`list_captures`/`load_capture`/`go_live`/`transport` handlers all require
a `SessionController` (`ctrl is not None`) — absent in a `--replay`-launched process with no live source.

| type | fields | effect | handler |
|------|--------|--------|---------|
| `cmd` | `name`, `param` | resolve → `CommandCode` (`resolve_command` `web.py:293`) and dispatch to the device; **in replay** publishes `"<label> -> not available in replay"` instead of a round-trip | `web.py:949` |
| `set_color` | `mode` | set point-cloud color plane (validated against `_VALID_COLOR_MODES`) → echo `state` | `web.py:1006` |
| `set_ir` | `colormap`, `freeze` | set IR colormap / freeze range (validated) → echo `state` | `web.py:1014` |
| `record` | `on` | start/stop recording to `captures/web_<ts>.bin` → echo `session` + fresh `captures` | `web.py:963` |
| `list_captures` | — | broadcast a fresh `captures` | `web.py:971` |
| `load_capture` | `name` | swap reader → replay (`sanitize_capture_name` → basename-only, `.bin`, must-exist; off-loop via `to_thread`) → echo `session` | `web.py:974` |
| `go_live` | — | swap reader → live proxy → echo `session` | `web.py:982` |
| `transport` | `action`(pause\|resume\|speed\|loop\|restart\|seek), `value` | playback control; `seek`/`restart` run off-loop via `to_thread` → echo `session` | `web.py:986` |
| `set_view` | `colormap?`(turbo\|gray), `point_size?`, `point_size_auto?`, `surface?`, `surface_mode?`(grid\|spatial), `surface_threshold?` | 3D viewport display: colormap, point size, surface interpolation toggle + settings; all optional, only provided fields update → echo `state`. `point_size_auto` scales each point by its range from the sensor (a client-side shader uniform, no wire change) so every zone subtends the same solid angle; with it on, `point_size` means the size at **1 m of range** rather than metres | `web.py` |
| `set_idle` | `enabled?`, `level?`(soft\|hard) | sensor auto-idle prefs (laser-wear reduction); persisted to `[viewer]` → echo `state`. Disabling cancels any armed idle timer. The idle itself is driven server-side off the viewer count (see below), not by an inbound message | this change |
| `set_mode` | `mode`(realtime\|slam) | switch top-bar mode; arms/disarms the `SlamRunner` off-loop (lazy worker build) → echo `state` | web Phase 4 |
| `slam_opt` | `trajectory?`, `walls?`(solid\|split), `follow?` | SLAM display toggles → echo `state` | web Phase 4 |
| `save` | — | write full-res `mapper.mesh()` + trajectory → `results/web_<ts>.ply`/`.tum` (off-loop); toast + `saved` echo. Disabled in real-time / empty map | web Phase 4 |
| `reset_fusion` | — | reset the `YawFusion` filter state (clears accumulated yaw correction); the heading snaps fresh on the next valid magnetometer sample. Publishes `"heading fusion reset"` on the bus | sensors card |
| `set_orientation` | `mode?`(zyx\|zxy\|boresight\|world), `labels?`[3] | select the orientation decomposition mode and/or rename the 3 axis-label slots (validated: unknown mode is rejected+logged, labels are trimmed/length-capped/defaulted per-slot via `_sanitize_axis_labels`); persisted to `[viewer]` → echo `state` | `web.py` |
| `zero_yaw` | — | capture the CURRENTLY DISPLAYED mode's `yaw_deg` at the current attitude and store the `graft_yaw` delta that zeroes it (`UiState.yaw_offset_deg`); persisted to `[viewer]` → echo `state`. **No-op (logged) in World mode** — its heading is absolute magnetic north, not offsettable — and when there is no orientation yet | `web.py` |
| `magcal` | `action`(open\|close\|start\|stop\|reset\|save\|discard\|view), `cal?`(current\|candidate) | magnetometer sweep/calibration modal (`web._handle_magcal`): `open`/`close` subscribe this socket to `magcal` reports; `start`/`stop` bracket a collection (`stop` also runs `magcal.fit_ellipsoid` → a **candidate**, never saved); `save` writes the candidate to `ViewerConfig.mag_cal_path` and hot-reloads it (`web.install_mag_calibration`); `discard` drops the candidate but KEEPS the samples; `reset` clears the samples; `view` picks which calibration colours the map. Unknown actions are rejected+logged. Every action echoes a fresh `magcal` | `web._handle_magcal` |
| `clear_yaw_offset` | — | reset `yaw_offset_deg` to 0.0 (back to raw SFLP yaw); persisted to `[viewer]` → echo `state` | `web.py` |

## Invariants (hold when adding a message)

- **One-way state flow.** The server is authoritative: inbound control mutates server state, then the
  server **echoes** the resulting `state`/`session`/`captures`. The frontend drives *all* active/disabled
  UI from that echo, never optimistically — so multiple tabs stay in sync. New control types must echo.
- **Untrusted inbound.** Every inbound handler validates before acting (`_VALID_COLOR_MODES`,
  `_VALID_IR_COLORMAPS`, `sanitize_capture_name`, `resolve_command` returning None on unknown). A new
  inbound type parsing a client-supplied string/path/enum does the same — reject, log, drop; never trust
  the field. (Retro checklist: adversarial/malformed input case for any new parser of untrusted bytes.)
- **Server-side math stays server-side.** `sensor.rot`/`heading` are computed in Python so the
  sign/permutation matrices live in exactly one place (`docs/coordinate-frames.md`). A SLAM
  trajectory/pose message should follow suit — send world-frame poses the browser renders verbatim,
  don't ship raw quats + matrices for JS to re-multiply.
- **Silent-when-empty.** `build_sensor_message` returns None (broadcaster sends nothing) when there's no
  sensor data. A SLAM message with no map yet should likewise stay silent rather than send an empty hull.
- **Off-loop for blocking work.** Anything that scans a file or joins a thread (`load_capture`, `seek`,
  `restart`) is dispatched via `asyncio.to_thread` so the single broadcaster/event-loop never stalls.
  A SLAM integrate/raycast that blocks belongs off-loop the same way.
- **Server-driven sensor auto-idle (no message).** To spare the ToF laser (VCSEL), the server watches its
  own viewer count: the last tab disconnecting arms a debounced (`sensor_idle_delay_s`) `SET_STANDBY`
  (`_viewer_left`), and a tab connecting wakes it (`_viewer_arrived`). It acts only on a live device we're
  streaming from (not a replay excursion — the command-ACK path there is the file, not the device), and a
  boot-time `startup-wake` guarantees a device a prior server left idled resumes streaming. This is *not* an
  inbound message — it's server policy over the WebSocket lifecycle. `set_idle` only tunes its enable/depth.
- **Persisted display state seeds the same `state` echo.** Web Phase 5: durable UI prefs live in
  `roomscan.toml` [viewer]`, seed `UiState` at boot (`web.ui_from_config`) and are written back on change
  (`web._persist_ui`, best-effort, reload-then-save). A new *durable* toggle adds a `ViewerConfig` field +
  the map on both sides; a purely *ephemeral* one stays out of the file. Persistence never adds a message —
  it rides the existing connect-time `state` echo, so the one-way-flow invariant is untouched.

## Frontend consumers

10 vanilla ES modules under `host/src/roomscan/static/`, wired through a hub in `app.js` (no build step,
no framework), plus `layout.js` — a deliberately *classic* (non-module) script owning the two-dock
column-wrapping layout and the diagnostics panel, so both survive a failure of the module graph
(see `docs/web-ui-testing.md` -> "The dock layout"). Binary tags are demuxed in `ws.js`; each JSON `type` is routed to its module
(`metrics.js`, `sensors.js`, `capture.js`, `controls.js`, `ir.js`, `slam.js`, `magcal.js`, …). `slam.js` (web Phase 4)
renders the SLAM mesh/trajectory into `scene.js`'s single Three.js context (via a handle `app.js` passes
it) and drives the follow camera — no second WebGL context. Saved maps download from a `/results/<name>`
static mount. Element ids for driving the UI headlessly are catalogued in `docs/web-ui-testing.md`.
