# Transform library streams/controls — boot dump capture

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
- `raw` — input, format `3DMD`, four size variants (100×149, 100×39,
  14842×1, 3844×1 — the last two match CLAUDE.md's binning-2/binning-4 raw
  buffer sizes; the 100×149/100×39 variants appear to be the same raw data
  reshaped as a 2‑D grid rather than a flat 1‑D buffer).
- `depth` — output, three formats each at two resolutions (54×42, 24×20):
  `ZF32` (32-bit float depth, what the app currently selects),
  `ZAPC` and `ZA16` (alternate packed/16-bit depth encodings — not decoded
  here, but available).
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
No. The output stream set is `depth, ambient, amplitude, confidence,
reflectance, status` — there is no `xyz`/`points`/`cloud` stream. The
transform library only emits 2‑D depth (+ auxiliary) maps; deprojection to
3‑D points is not done on-device. **Phase 2 implication:** the `Deprojector`
stays a PC-side responsibility exactly as planned in Phase 1 — the protocol
only ever needs to carry 2‑D per-pixel arrays (depth/ambient/amplitude/
confidence/reflectance/status), never point coordinates, keeping payload
sizes and CPU work on the M33 down.

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
