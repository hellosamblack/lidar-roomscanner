# Viewer metrics HUD — design

**Date:** 2026-07-10
**Scope:** `host/` `roomscan-panel` only (the classic legacy-Open3D `roomscan-view`
cannot overlay text on its scene; it keeps its existing one-line stats).

## Goal

Add a live, on-scene overlay to the control panel showing, at a glance:

1. **Per-sensor data rates** — samples/second for every sensor stream, split into
   two numbers: the device-side production cadence (**→hub**) and the rate that
   actually reached the PC (**→host**).
2. **Rendered FPS** — the panel's actual scene redraw rate (distinct from sensor
   frame rate).
3. **System resource monitor** — CPU % per core, RAM used/total, GPU util + VRAM,
   and interface bandwidth.

## Rate semantics (owner-confirmed 2026-07-10)

Every wire frame carries an on-device microsecond timestamp `t_us` (see
`docs/protocol.md`, `FrameHeader`). For each stream we track two rates over a
sliding window:

- **→hub** (device production rate) = `1 / mean(Δt_us)` across consecutive frames
  of that stream. This is the cadence the MCU/sensor-hub produced samples at — it
  is what the hub *collected*, independent of the link.
- **→host** (arrival rate) = frames received / wall-clock window (`time.monotonic`
  stamped on receipt in the reader thread).

When the USB CDC link keeps up, the two match. When frames are dropped or
decimated in transit, **→host < →hub**, making link loss visible per sensor —
this is the "rate to the hub vs rate to the host" the owner asked for.

**Robustness:** if a stream's `t_us` is absent/zero/non-monotonic (e.g. firmware
doesn't stamp it for that stream), the device rate is reported as `—` and only
the host rate is shown. A stale stream (no frames for > ~1.5 s) decays to `0`.

## Sensors shown

| Label      | stream_id             | notes |
|------------|-----------------------|-------|
| ToF        | `RAW_3DMD` (7), or `DEPTH_ZF32` (0) on Phase-1 replays | the depth cloud source |
| IMU        | `IMU_QUAT` (9)        | LSM6DSV16X SFLP quaternion (~480 Hz) |
| Env        | `ENV` (10)            | baro+mag+temp are **bundled into one ENV frame**, so this is one sensor-hub cadence, not three separate rates. Per-sensor env rates are not separable in protocol v1. |
| CALIB      | `CALIB` (8)           | infrastructure, not a sensor — excluded from the sensor list (its bytes still count toward link bandwidth). |

## Interface bandwidth

The meaningful "interface" for this app is the **scanner link** itself. We own
every byte off the wire, so link throughput is measured exactly: each decoded
frame's full wire size (`HEADER_SIZE + payload_len + 4`) plus the delta of
`decoder.bytes_skipped`, summed over the window → **bytes/s on the CDC link**.
System NIC bytes/s (psutil `net_io_counters`) is shown secondarily as "NIC".

## GPU/VRAM (owner-confirmed: optional pynvml, graceful fallback)

Source order, first that works wins, all failures degrade to `GPU n/a`:
1. `pynvml` (optional dep, added to `[project.optional-dependencies].monitor`) —
   NVIDIA util % + used/total VRAM.
2. `nvidia-smi` subprocess (`--query-gpu=utilization.gpu,memory.used,memory.total`)
   — no dependency, still NVIDIA-only.
3. `n/a` — non-NVIDIA or no driver.

## Architecture

Two new pure/isolated units in a new module `host/src/roomscan/metrics.py`, plus
presentation wiring in `panel.py`.

### `RateMeter` (pure, fully unit-tested)

- One instance per tracked stream. Fed `(seq, t_us, arrival_s, nbytes)` for every
  DATA frame of that stream in the **reader thread**.
- Maintains a bounded deque of recent `(arrival_s, t_us, nbytes)` samples
  (window = `WINDOW_S`, default 2.0 s).
- `host_hz(now)` = count-1 over arrival span; `device_hz()` = `1/mean(Δt_us)`
  (None if <2 samples or non-monotonic t_us); `bytes_per_s(now)`; all pure reads,
  called from the UI thread. A single lock guards the deque (reader writes, UI
  reads) — mirrors `SensorState`'s threading contract.

### `MetricsRegistry`

- Owns a `RateMeter` per `stream_id` seen, plus the render-FPS counter and a
  handle to the `ResourceSampler`. `record(header, nbytes, now)` in the reader
  thread; `tick_render()` bumps the rendered-frame counter each scene redraw;
  `snapshot(now)` returns a plain dataclass the HUD formats. No Open3D imports —
  testable headless.

### `ResourceSampler` (background thread, ~1.5 Hz)

- `psutil` for `cpu_percent(percpu=True)`, `virtual_memory()`, `net_io_counters()`.
- GPU/VRAM via the fallback chain above, sampled on the same thread so a slow
  `nvidia-smi` never touches the render loop.
- Primed once on start (`cpu_percent(interval=None)` needs a baseline call).
- Exposes a latest-snapshot slot read locklessly (atomic reference swap) by the
  UI thread. Stops cleanly on panel close.

### Panel wiring (`panel.py`)

- Construct a `MetricsRegistry` (+ `ResourceSampler`) in `ControlPanel.__init__`;
  start the sampler in `start()`, stop it in `_on_close()`.
- In `_run_reader`, after a frame is decoded, feed the registry the header +
  wire size + arrival time. `_run_reader` gains an optional `metrics=` param
  (default `None`) so existing tests are unaffected; wire size comes from
  `HEADER_SIZE + header.payload_len + 4`.
- **The overlay**: a stack of `gui.Label` widgets (or one multiline label) added
  as **Window children**, positioned in `_on_layout` at the top-left of the
  scene rect (not inside the side panel). Refreshed on the existing ≤4 Hz
  `_on_tick` UI cadence via `_update_metrics()`. Rendered-FPS reuses the
  existing `self._fps` computation, now surfaced in the overlay.
- **Toggle**: `M` key and a "Metrics overlay" checkbox in the View group show/hide
  the overlay. Persisted via a new `metrics_overlay: bool = True` config field.
- Open3D `gui.Label` background styling is limited; the overlay relies on a bright
  monospace-ish text color over the (dark by default) scene. If contrast proves
  insufficient in the supervised run, a dim backdrop `ImageWidget` is the
  fallback — not built speculatively (YAGNI).

## What is NOT built

- No new wire protocol / firmware change (`docs/protocol.md` untouched — pure host
  presentation, like Phase 3.5).
- No overlay for the classic viewer (infeasible in legacy Open3D; out of scope).
- No historical graphs/plots of metrics — instantaneous readout only.
- No per-env-sensor rate split (bundled in ENV; would need a protocol change).

## Verification

- Unit tests (`tests/test_metrics.py`): `RateMeter` device/host/bytes math incl.
  the zero/non-monotonic-`t_us` fallback and stale decay; `MetricsRegistry`
  record/snapshot; `ResourceSampler` GPU fallback chain with the subprocess/pynvml
  mocked (no real GPU dependency in tests).
- `_run_reader` metrics feed covered by extending `test_panel.py`'s reader tests.
- Headless panel smoke (`run_one_tick`) exercises overlay construction + update
  without a display.
- **Honest limitation:** the overlay's *visual* placement/readability cannot be
  verified on this locked box (Open3D Filament offscreen fails — `EGL Headless not
  supported`). That check is owner-supervised, matching the standing Phase 3.5
  practice.
