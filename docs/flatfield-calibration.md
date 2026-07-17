# Flat-field (fixed-pattern) calibration

The VL53L9 is a multi-zone SPAD imager, so every zone has a slightly different
response. On a uniform surface this shows up as a **stable, sensor-locked ripple
in the reflectance plane** — fixed-pattern noise (FPN). On a flat wall it measured
at **~18 % of signal**, rock-stable frame-to-frame (SNR ~9), aligned to the sensor
row/column axes (investigation 2026-07-16). It is *not* display resampling and
*not* scene texture. Uncorrected, it contaminates the IR view, reflectance-based
SLAM coloring, and any reflectance-derived enhancement (multi-frame super-res /
relief shading would bake in and amplify the grid).

Flat-field calibration measures that per-zone response once and divides it out.

## Model

A multiplicative **per-zone gain map** `gain[H,W]` (unit-mean). Correction is
`reflectance *= gain`, applied to the reflectance plane inside
`pipeline.TransformStage`, so **every** consumer — web, panel, viewer, SLAM —
sees corrected reflectance with no per-consumer wiring. Depth FPN (a per-zone
*range* offset) and confidence are separate calibrations; the map format is
per-plane-extensible but v1 ships reflectance only.

Code: `roomscan.flatfield` (`FlatField`, `build_flatfield`). Disabled by default
— with no map configured, `FlatField.load_configured()` returns `None` and the
pipeline is a no-op.

## Capturing a flat-field reference (REQUIRED: pan the sensor)

The one rule that makes the calibration valid: **slowly pan/sweep the sensor
across a uniform matte surface while recording.** Panning smears real surface
texture across zones so it averages toward the smooth illumination, leaving only
the sensor's fixed per-zone response. A **static** capture is invalid — it bakes
scene texture into the "correction" (a static recording of a plain wall still
produced a 44 % residual and a 0.25–2.2 gain spread: garbage, not a calibration).

A blank painted wall, foam board, or a grey card works. Keep it roughly
fronto-parallel, fill the frame, avoid specular hot-spots and the no-return
regime (get close enough that all zones return signal).

## Build and enable

```sh
cd host
# 1. record ~15-20 s while slowly panning across the uniform surface
python -m tools.capture --seconds 20 --out flatfield_pan.bin
# 2. build the correction map
python -m tools.build_flatfield flatfield_pan.bin --out flatfield.npz
# 3. enable it: add to roomscan.toml ([viewer] table)
#      flatfield_path = "flatfield.npz"
```

The builder prints the measured FPN residual and the gain spread. Sanity checks
for a *good* capture: residual in the low tens of percent at most, gain
comfortably inside `[0.5, 1.6]`, and **>= ~100 panned frames**. A high residual
or a gain spread near the `[0.33, 3.0]` clip bounds means the capture wasn't a
clean pan — recapture.

## Verifying it worked

Point the sensor at the flat wall again with correction enabled: the fixed grid
should collapse into near-uniform reflectance. The definitive live test is the
one that identified the FPN in the first place — **pan across the wall**: the
grid sits still (locked to zones) while the wall slides underneath, and with a
correct map it disappears.

## Files

- `host/src/roomscan/flatfield.py` — `FlatField` (apply/save/load/`load_configured`) + `build_flatfield`
- `host/tools/build_flatfield.py` — CLI: capture `.bin` → `.npz` map
- `host/src/roomscan/pipeline.py` — `TransformStage(..., flatfield=…)` applies it to reflectance
- `host/src/roomscan/config.py` — `[viewer] flatfield_path` persists the enabled map
- `host/tests/test_flatfield.py` — synthetic-FPN recovery, apply/save/load, config, DLL-gated pipeline integration
