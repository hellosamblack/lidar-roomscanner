# Transform library streams/controls — boot dump capture

> **Phase 2 update (2026-07-08):** the transform now runs host-side (PC), not on the M33 — see
> ROADMAP.md's "Post-processing runs on the PC" decision bullet for why it was dropped, and
> `docs/h563-optimization-notes.md` for the retired on-device speed-up options. The
> stream/control inventory captured below is still the authoritative source for what the transform
> library exposes; only *where* it executes has changed.

Captured from the real board (NUCLEO-H563ZI, X-NUCLEO-53L9A1, VL53L9CX) running
`firmware/scanner-stream` (Task 6's unmodified fork of the reference app —
`CONF_USECASE = VL53L9_USECASE_AR_PRECISION`, `CONF_PRINT_FRAME = 1`).

**Capture method:** flashed `build/Debug/scanner_stream.bin` via
`STM32_Programmer_CLI -c port=SWD -w ... 0x08000000`, then opened the ST-Link
VCOM (COM14, 115200 8N1) with a Python/pyserial script *before* issuing
`STM32_Programmer_CLI -c port=SWD -hardRst` to force a real boot from reset,
then read for ~25 s. (An initial attempt using `-rst`, and a capture window
opened only after the flash command, missed the boot banner entirely because
the flash+reset round-trip outran the fixed capture window — the fix was to
open the serial port first, confirm it was listening, *then* trigger the
reset, and to use `-hardRst`.) Confirmed a genuine cold boot: `depth`
stream metadata's `frame_counter` restarted at 1 and climbed monotonically
(1, 2, 3, ... "@ 20 fps" transient settling to a steady "@ 3 fps") immediately
following the dump below — no gap, no repeated banner.

## Raw `streams_inspect` / `controls_inspect` output

```text
Streams:
	Name: raw
	Description: 
	Direction: 1
	Capabilities:
		Properties:
			format: 3DMD
			width: 100
			height: 149
		Properties:
			format: 3DMD
			width: 100
			height: 39
		Properties:
			format: 3DMD
			width: 14842
			height: 1
		Properties:
			format: 3DMD
			width: 3844
			height: 1
	Name: depth
	Description: 
	Direction: 2
	Capabilities:
		Properties:
			format: ZF32
			width: 54
			height: 42
		Properties:
			format: ZF32
			width: 24
			height: 20
		Properties:
			format: ZAPC
			width: 54
			height: 42
		Properties:
			format: ZAPC
			width: 24
			height: 20
		Properties:
			format: ZA16
			width: 54
			height: 42
		Properties:
			format: ZA16
			width: 24
			height: 20
	Name: ambient
	Description: 
	Direction: 2
	Capabilities:
		Properties:
			format: IF32
			width: 54
			height: 42
		Properties:
			format: IF32
			width: 24
			height: 20
	Name: amplitude
	Description: 
	Direction: 2
	Capabilities:
		Properties:
			format: AF32
			width: 54
			height: 42
		Properties:
			format: AF32
			width: 24
			height: 20
	Name: confidence
	Description: 
	Direction: 2
	Capabilities:
		Properties:
			format: CF32
			width: 54
			height: 42
		Properties:
			format: CF32
			width: 24
			height: 20
	Name: reflectance
	Description: 
	Direction: 2
	Capabilities:
		Properties:
			format: RF32
			width: 54
			height: 42
		Properties:
			format: RF32
			width: 24
			height: 20
	Name: status
	Description: 
	Direction: 2
	Capabilities:
		Properties:
			format: CU32
			width: 54
			height: 42
		Properties:
			format: CU32
			width: 24
			height: 20
Controls:
	Control:
		Name: bypass-r2p-algo
		Nick: 
		Description: 
		Quark: 0
		Value: false
		Type: 7
		Flags: 3
		Spec: min = unknown, max = unknown
	Control:
		Name: bypass-tnr-algo
		Nick: 
		Description: 
		Quark: 1
		Value: false
		Type: 7
		Flags: 3
		Spec: min = unknown, max = unknown
	Control:
		Name: bypass-r2p-filter
		Nick: 
		Description: 
		Quark: 2
		Value: false
		Type: 7
		Flags: 3
		Spec: min = unknown, max = unknown
	Control:
		Name: bypass-conf-filter
		Nick: 
		Description: 
		Quark: 3
		Value: false
		Type: 7
		Flags: 3
		Spec: min = unknown, max = unknown
	Control:
		Name: bypass-refl-filter
		Nick: 
		Description: 
		Quark: 4
		Value: false
		Type: 7
		Flags: 3
		Spec: min = unknown, max = unknown
	Control:
		Name: bypass-sharpener-filter
		Nick: 
		Description: 
		Quark: 5
		Value: false
		Type: 7
		Flags: 3
		Spec: min = unknown, max = unknown
	Control:
		Name: bypass-fp-filter
		Nick: 
		Description: 
		Quark: 6
		Value: false
		Type: 7
		Flags: 3
		Spec: min = unknown, max = unknown
	Control:
		Name: calib-buffer
		Nick: 
		Description: 
		Quark: 7
		Value: unknown
		Type: 9
		Flags: 3
		Spec: min = unknown, max = unknown
	Control:
		Name: cover-glass
		Nick: 
		Description: 
		Quark: 8
		Value: false
		Type: 7
		Flags: 3
		Spec: min = unknown, max = unknown
```

## ASCII depth-frame excerpt (post-boot, frame 5 of the capture)

```text
Processed frame n. 5 @ 3 fps
[0;0H@@@@@@@@@@@@@@@@@@@@@@@@@@ %%%%###############%%%%%%%%
@@@@@@@@@@@@@@@@@@@@@@@@@ %%%%%###############%%%%%%%%
@@@@@@@@@@@@@@@@@@@@@@@@  %%%%%  #############%%%%%%%%
@@@@@@@@@@@@@@@@@@@@@@@@ %%%%%%    ############%%%%%%%
@  @@@@@@@@@@@@@@@@@@@@@ %%%%%%#   ############%%%%%%%
@@%%%  @@@@@@@@@@@@@@@@@ %%% @    #############%%%%%%%
```

Observed frame rate: transient `20 fps` / `4 fps` on the first two processed
frames (pipeline warm-up), settling to a steady **3 fps** by frame 3 onward —
matches CLAUDE.md's estimate for a ~9 KB ASCII/ZF32 frame over 115200 VCOM
and confirms USB CDC (Phase 1) / Ethernet (Phase 4) are required for
real-time rates.

## Interpretation

**1. Streams beyond `depth`/ZF32 (names + formats).**
Eight streams total. One input (`raw`, direction `1`), seven outputs
(direction `2`):
- `raw` — input, format `3DMD`, four size variants split by capture
  interface (per the library source, `vl53l9_transform.c` ~375-376:
  `// csi` / `// i3c` comments): 100×149 and 100×39 are for **CSI-2**
  capture (an interface this board doesn't use); 14842×1 and 3844×1 are
  the **I3C** variants matching CLAUDE.md's binning-2 / binning-4 raw
  buffer sizes.
- `depth` — output, three formats each at two resolutions (54×42, 24×20):
  - `ZF32` — one float32 depth value per zone (what the app currently
    selects); 9 072 B/frame at 54×42.
  - `ZAPC` — **Android depth point cloud**: four float32 per zone,
    `[x, y, z, confidence]` (confidence normalized 0..1); 16 B/zone →
    36 288 B/frame at 54×42, 4× the ZF32 payload. This is an on-device
    point cloud — see Q2.
  - `ZA16` — Android depth16: 16 bits per zone (confidence in the 3 MSBs,
    depth in the remaining 13 bits) — a *smaller* wire format than ZF32
    if bandwidth ever dominates precision.
- `ambient` — `IF32` (32-bit float ambient/background light level).
- `amplitude` — `AF32` (32-bit float return-signal amplitude).
- `confidence` — `CF32` (32-bit float per-pixel confidence).
- `reflectance` — `RF32` (32-bit float reflectance/IR-intensity estimate).
- `status` — `CU32` (32-bit unsigned per-pixel status/error code).

This is the full Phase 2 stream menu: `reflectance` is the IR-intensity
channel the roadmap calls out for colorizing the cloud; `confidence` and
`status` are natural per-point quality gates; `ambient`/`amplitude` are
extra diagnostic channels. All seven output streams are available at the
same two resolutions (54×42 and 24×20), so Phase 2's protocol can multiplex
any of them without a resolution-negotiation problem.

**2. Is there an XYZ/point-cloud output stream?**
**Yes — not as a separate stream, but as a format of the `depth` stream.**
Negotiating the `depth` stream with format `ZAPC` (instead of `ZF32`)
yields an on-device point cloud: four float32 per zone,
`[x, y, z, confidence]` (confidence 0..1), 16 B/zone → 36 288 B/frame at
54×42 — 4× the ZF32 payload. Verified in the library source:
- `vl53l9-transform-c/README.md` (~43-46) defines ZAPC as "Android depth
  point cloud format, each point is represented by four floats:
  [x, y, z, confidence]".
- `vl53l9_transform.c` ~736-737: selecting ZAPC sets
  `is_pointcloud_requested = true`; ~1029-1030 copies the `_pointcloud`
  buffer (`resolution * 4 * sizeof(float)`) into the depth stream's output.
- `radial_to_perp.c` ~156-197 (`vl53l9_algo_pointcloud`): a true pinhole
  deprojection on-device — focal length derived from the lens EFL and SPAD
  pitch, per-pixel distortion correction, optional per-pixel parallax
  correction — i.e. it uses the sensor's factory-calibrated optics, not a
  linear-FoV approximation.

**Phase 1/2 implication:** Phase 1 stays on `ZF32` + PC-side `Deprojector`
as already decided in the plan's execution note. But Phase 2 has a real
choice: trade 4× payload bandwidth for on-device deprojection with
factory-calibrated intrinsics (plus a fused per-point confidence for free).
Either way, when Phase 2 arrives the PC-side `Deprojector`'s linear-FoV
model should be validated against ZAPC output for the same scene — ZAPC is
ground truth here, since it uses the real calibrated intrinsics.
*Phase 2.5 caveat:* ZAPC's per-point confidence was measured
**non-discriminating on real captures** (~1.0 on every zone, including
no-return ones) — a vendor bug, not a scene artifact: the library never
initializes its `conf_scaling` divisor, so the channel is structurally
constant. See `docs/deprojector-validation.md`'s confidence-channel finding
(root-cause paragraph included) before relying on it as a quality gate.

**3. Which controls exist — which are Phase-3 targets?**
Nine controls, all currently `false`/boolean except `calib-buffer`:
`bypass-r2p-algo`, `bypass-tnr-algo`, `bypass-r2p-filter`,
`bypass-conf-filter`, `bypass-refl-filter`, `bypass-sharpener-filter`,
`bypass-fp-filter` (seven algorithm-stage bypass toggles, `Type: 7` =
boolean, `Flags: 3`), `calib-buffer` (`Type: 9`, the mandatory
calibration-blob control already wired by `vl53l9_app.c`), and
`cover-glass` (boolean, presumably a cover-glass-compensation enable).
Phase 3 can target the seven `bypass-*` toggles and `cover-glass` directly
as runtime on/off controls (they're already transform-library `controls`,
so a host→device control message just needs to set `Value`). **Important
gap:** there is no `usecase` or `binning` control in this list — those are
not transform-pipeline controls at all. They're set before
`transform_initialize()` via `vl53l9_utils_set_profile()` against the
compile-time `CONF_USECASE`/`g_ranging_profiles[]` table (sensor-level
config), and changing them at runtime would require re-running sensor
init/profile-apply, not just poking a `controls` entry. Phase 3's
"set usecase/binning at runtime" goal is therefore a bigger lift than the
stream-toggle controls and needs its own host→device message that redrives
setup, not just a `controls_inspect`-style control write.

**4. Observed ZF32 value range (mm sanity check).**
Cannot be determined from this capture — and that's a real (not just
missing-data) finding. `print_frame()` (`firmware/scanner-stream/Src/
vl53l9_app.c:272-303`) computes `min`/`max` **per frame** from the ZF32
buffer, pads by 5%, then linearly remaps each pixel into one of 10 ASCII
shading characters (`"@%#*+=-:. "`). The characters therefore encode only
*relative contrast within a single frame* — the printed art carries no
absolute-value information, and neither `min` nor `max` (nor any other
depth value) is ever printed as a number anywhere in the boot dump. The 5%
padding constant (`(max - min) * 0.05f`) is consistent with values in the
hundreds-to-low-thousands range (typical for millimetre depth at these
usecase distances) rather than, say, single-digit metres, but this is
circumstantial, not a verified number. **To actually sanity-check the
millimetre assumption used by `Deprojector`, the firmware needs a one-line
addition** (e.g. `printf("min=%lu max=%lu\n", min, max)` right after the
existing min/max computation) and a re-capture — flagged here as follow-up
work rather than done silently, since Task 7's brief is to capture the
*unmodified* fork's boot dump.

---

## Library upgrade 1.3.1 → 1.5.0, and the `VL53L9_TRANSFORM_LIGHT` default (2026-08-10)

`firmware/vendor/stsw-img053/` (the STM32N6570-DK package, added 2026-08-10) ships
**vl53l9-transform-c 1.5.0**; the 53L9A1 reference we had been building against is **1.3.1**.
`host/transform/CMakeLists.txt` now selects between them with `RS_TRANSFORM_PKG`
(`STSW-IMG053` — the default — or `53L9A1`); the shim compiles unmodified against either,
since the public API and the media-object include layout are unchanged.

**The upgrade is output-neutral, but only because of an explicit `-DVL53L9_TRANSFORM_LIGHT=0`.**

`vl53l9_transform.h` self-defines `VL53L9_TRANSFORM_LIGHT (1)` when the caller leaves it
undefined, so **every build of this shim before 2026-08-10 was a LIGHT build without ever
saying so** — which is why the boot dump above shows blank `Description:` fields. Under 1.3.1
that was the whole effect: LIGHT blanked the control nick/description strings and made three
query methods return `MEDIA_ERROR_UNIMPLEMENTED`. Under 1.5.0 it *additionally* flips the
**defaults** of `bypass-tnr-algo` and `bypass-flying-pixel-filter` to `true`, silently
disabling the temporal denoiser and the flying-pixel filter.

That is not a knob we want. TNR-off measures 2.9× the depth temporal noise on a static scene
and *worse* SLAM closure (0.65 → 0.83 m on `coffeeRoomCircuitNoMnt`, n=5) — see the
`transform-algo-toggles-ab` A/B. Left at the default, 1.5.0 diverged from 1.3.1 on ~95% of
pixels: depth p99 10.5 mm (max 11.2 m on outliers the flying-pixel filter had been catching),
and confidence stuck flat at ~54 where 1.3.1 climbs to ~290 as TNR integrates. Frame 0 is
identical in both — TNR passes the first frame through — so the divergence starts at frame 1,
which is the signature to look for.

With `VL53L9_TRANSFORM_LIGHT=0` pinned, **1.5.0 is bit-identical to the 1.3.1 output we
shipped**: 1800 frames across `coffeeRoomCircuitNoMnt`, `ambientRegular8msFFpanLarge` and
`officeFullScanAug6` (both ranging modes), all four output planes, zero differing values.
`tests/test_equivalence.py` (PC-vs-MCU golden pairs) also still passes. Setting it explicitly
is a no-op for 1.3.1 — verified bit-identical — so both `RS_TRANSFORM_PKG` values stay
directly comparable.

Bisected by reverting individual 1.5.0 files into an out-of-tree copy: `ratenorm.c`,
`radial_to_perp.c` and `sharpener.c` each changed **nothing**, and only reverting
`vl53l9_transform.c` (which carries the LIGHT-gated control defaults) restored 1.3.1 exactly.
Compiler flags are not a factor — `-O2` and `-O3` builds of the same sources are bit-identical.

### What 1.5.0 brings that 1.3.1 lacks

- **Sharpener** rewritten (Python algo R_1.4.0 → R_1.6.2). Now also takes `reflectance` and
  `confidence` as inputs, and gains `recover_mode` (glare recovery around retro-reflective
  "aggressor" pixels, `reflectance > 300`), `th_peak_dominance_ratio` (a single hot pixel no
  longer sets its group's signal scale) and `exp_optim` (Taylor `exp`). All default off, and
  measured inert here — reverting the whole file changed no output value. `MAX_IMAGE_SIZE`
  grew 2268 → 9072 and `group_id[]` is a stack array, so this costs ~27 KB of stack if the
  library is ever built for the M33 again.
- **Binning 6/8/12/24 supported end-to-end**: new stream caps for 18×14, 12×10, 8×6 and 4×4
  with matching I3C raw sizes 1738/880/516/204 B. 1.3.1 accepted only binning 2 and 4.
- **`radial_to_perp`** binning distortion changed from `(b==2)?1.0:|√2·((b²/4)−(b/2))|` to
  `b²/4`. Both evaluate to 1.0 at binning 2 — no effect on our deprojection — but they diverge
  at binning ≥4 (b=4: 2.83 → 4.0), which matters only if the item above is ever acted on.
- **`dmax`** defaults moved (`ambient_clip` 2.0 → 7.0, new `amb_min_median` floor) but
  `_process_dmax` is still a stub in 1.5.0, so this remains inert.
