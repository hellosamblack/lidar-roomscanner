# Phase 2: Raw Streaming + PC-Side Transform Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the `vl53l9-transform-c` post-processing pipeline from the STM32 to the PC: firmware streams raw `3DMD` frames + calibration; the PC runs the identical C library natively, unlocking full 54×42 resolution at ~30 fps over the existing USB CDC link and making every output stream (depth/IR/confidence/ambient/ZAPC) a host-side choice.

**Architecture:** Additive protocol change (two new stream IDs: RAW_3DMD, CALIB — still v1). A transitional **dual-stream firmware** emits raw+depth+calib together so we capture golden pairs for equivalence testing; the native transform (built as a DLL with MSVC from the read-only 53L9A1 sources + a thin shim replicating the firmware's transform setup) must reproduce the MCU depth output on those pairs before the firmware drops its transform. Then a raw-only firmware mode at a 30 Hz profile, and the host pipeline grows a transform stage between decoder and viewer.

**Tech Stack:** STM32 C (existing fork), CMake + MSVC Build Tools 18 (native DLL), Python ctypes wrapper, pytest, Open3D viewer (existing).

## Global Constraints

- `../53L9A1/` is read-only — the native build compiles its sources **in place** via `PKG_ROOT`, exactly like the firmware does; never copy or edit them.
- Protocol stays **v1** (additive stream IDs only, per the `protocol-change` skill). New registry entries: `RAW_3DMD = 7`, `CALIB = 8`. CRC last, little-endian, 32-byte header unchanged.
- **Full resolution is a hard requirement**: binning stays 2 (54×42); raw payload = 14,842 B; `VL53L9_CALIB_DATA_SIZE = 2332`.
- **Equivalence gate:** PC transform output vs MCU output on identical raw input: report exact-match percentage; gate = `np.allclose(atol=0.01)` (mm). Bit-exactness may be broken by float reassociation (`-Ofast` on ARM vs MSVC) — measure, don't assume, and document whichever result we get. TNR is stateful: all comparisons process frames **in capture order from frame 1 of a boot**.
- Commits: `git -c commit.gpgsign=false commit` (1Password signing flaky this setup) with trailers:
  `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_01YJym32WyVXthFmwK5SFDjY`
- ARM toolchain PATH prepend: `/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*/tools/bin` (resolve glob). Flash tool: `C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe`, `-c port=SWD -w firmware/scanner-stream/build/Debug/scanner_stream.bin 0x08000000 -rst`.
- MSVC: `C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools` — configure CMake with `-G "Visual Studio 18"` (or Ninja after `vcvars64`), x64.
- Hardware: ST-Link VCOM = COM14; CDC = VID:PID `CAFE:4001` (COM15). Host venv: `host/.venv/Scripts/python` (3.12). Tasks marked **[HW]** need the board.
- Suite baseline: 30 pytest passing before this plan; every task leaves it green.

---

### Task 1: Protocol additions — RAW_3DMD + CALIB streams

**Files:**
- Modify: `docs/protocol.md` (stream registry + two subsections + version history)
- Modify: `host/src/roomscan/protocol.py` (StreamId entries + constants)
- Modify: `firmware/scanner-stream/Src/rs_protocol.h` (defines)
- Test: `host/tests/test_protocol.py` (extend)

**Interfaces:**
- Produces: `StreamId.RAW_3DMD = 7`, `StreamId.CALIB = 8`, `RAW_3DMD_SIZE_BIN2 = 14842`, `CALIB_SIZE = 2332` (protocol.py); `RS_STREAM_RAW_3DMD (7u)`, `RS_STREAM_CALIB (8u)`, `RS_RAW_3DMD_SIZE_BIN2 (14842u)`, `RS_CALIB_SIZE (2332u)` (rs_protocol.h). Semantics consumed by Tasks 2/5/6.

- [ ] **Step 1: Spec first.** In `docs/protocol.md`'s Stream registry table change the two rows and append after the table:

Registry rows (replace "reserved" numbering note if it conflicts — 7 and 8 were unallocated):

```markdown
| 7 | RAW_3DMD | opaque vendor raw frame from the VL53L9CX (input to vl53l9-transform-c). At binning 2: `payload_len` = 14842. Header `width`/`height` carry the logical zone grid (54×42); `payload_len` is authoritative for size. `seq`/`t_us` as for DEPTH frames. | live (Phase 2) |
| 8 | CALIB | per-device calibration blob (`VL53L9_CALIB_DATA_SIZE` = 2332 B), required to run the transform host-side. `seq` = seq of the next RAW frame; `width`/`height` = zone grid. Sent at stream start and **retransmitted every 64 RAW frames** so late-attaching hosts acquire it (a host must buffer or discard RAW frames until a CALIB arrives). | live (Phase 2) |
```

Version history: `- **v1 rev 2026-07-08 (b)**: additive — RAW_3DMD (7) and CALIB (8) allocated for the PC-side-transform architecture. No layout change.`

- [ ] **Step 2: Failing tests.** Append to `host/tests/test_protocol.py`:

```python
def test_raw_and_calib_stream_ids():
    from roomscan.protocol import CALIB_SIZE, RAW_3DMD_SIZE_BIN2, StreamId
    assert StreamId.RAW_3DMD == 7
    assert StreamId.CALIB == 8
    assert RAW_3DMD_SIZE_BIN2 == 14842
    assert CALIB_SIZE == 2332
```

Run: `host/.venv/Scripts/python -m pytest tests/test_protocol.py -v` → FAIL (no attribute `RAW_3DMD`).

- [ ] **Step 3: Implement.** `protocol.py`: extend `StreamId` with `RAW_3DMD = 7` and `CALIB = 8`; add module constants `RAW_3DMD_SIZE_BIN2 = 14842` and `CALIB_SIZE = 2332` near `DEPTH_NO_RETURN_MM`. `rs_protocol.h`: add below `RS_STREAM_STATUS`:

```c
#define RS_STREAM_RAW_3DMD    (7u) /* opaque vendor raw frame (transform input) */
#define RS_STREAM_CALIB       (8u) /* per-device calibration blob */
#define RS_RAW_3DMD_SIZE_BIN2 (14842u)
#define RS_CALIB_SIZE         (2332u)
```

- [ ] **Step 4: Verify.** Full suite: 31 passing. Firmware compiles: `cmake --build build/Debug` from `firmware/scanner-stream` (defines only).

- [ ] **Step 5: Commit** — `feat(protocol): allocate RAW_3DMD + CALIB streams for PC-side transform (v1 additive)`

---

### Task 2: Dual-stream validation firmware + **[HW]** golden-pair capture

**Files:**
- Modify: `firmware/scanner-stream/Src/vl53l9_app.c`
- Create: `captures/golden_pairs.bin` (NOT committed — gitignored)
- Create: `host/tests/fixtures/golden_pairs_snippet.bin` (committed; covered by the existing `!host/tests/fixtures/*.bin` exception)

**Interfaces:**
- Consumes: `rs_send_depth_cdc`-era helpers (`rs_write_header`, `rs_crc32`, `rs_put_u32`, `rs_cdc_send`, `rs_time_us`, `rs_wait_event_usb`) — all exist in `vl53l9_app.c` today.
- Produces: firmware knob `CONF_STREAM_RAW (1)` — when set, each loop iteration sends **RAW(N-1) then DEPTH(N-1)** (same seq!), and a CALIB frame at startup + every 64 frames. Also generalizes the CDC sender to any stream: `rs_send_frame_cdc(uint8_t stream_id, uint32_t seq, uint8_t flags, const uint8_t *payload, uint32_t len, uint16_t w, uint16_t h)`. Golden capture consumed by Task 4.

- [ ] **Step 1: Generalize the sender.** In `vl53l9_app.c`, rename `rs_send_depth_cdc` → `rs_send_frame_cdc` adding a `uint8_t stream_id` first parameter; replace the hardcoded `RS_STREAM_DEPTH_ZF32` in its `rs_write_header` call with the parameter. Keep the `pending_dropped` static and connect-check logic exactly as is (one dropped-flag domain across all streams is correct: seq is shared). Update the existing call in the processing branch to `rs_send_frame_cdc(RS_STREAM_DEPTH_ZF32, ...)`.

- [ ] **Step 2: Add the raw + calib emission.** Below the `CONF_STREAM_BINARY` define add `#define CONF_STREAM_RAW (1) /**< also stream RAW_3DMD + periodic CALIB (dual-stream validation / PC-transform mode) */`. In `vl53l9_app()` after `vl53l9_get_calib_data(...)` succeeds, keep `calib_data` (it already exists, size `VL53L9_CALIB_DATA_SIZE`). In the acquisition loop's processing branch (where the DEPTH send happens, guarded `if (rs_have_prev)`), add BEFORE the depth send:

```c
#if CONF_STREAM_RAW
            {
                static uint32_t rs_calib_countdown = 0;
                if (rs_calib_countdown == 0) {
                    rs_send_frame_cdc(RS_STREAM_CALIB, rs_prev_counter, 0u, calib_data,
                                      VL53L9_CALIB_DATA_SIZE, out_width, out_height);
                    rs_calib_countdown = 64;
                }
                rs_calib_countdown--;
                /* raw buffer of the frame being processed = the PREVIOUS index (the pipeline
                 * input); send it with the same seq as the depth it produces */
                rs_send_frame_cdc(RS_STREAM_RAW_3DMD, rs_prev_counter, 0u,
                                  (const uint8_t *)in_raw_mem[(raw_mem_index + 1) % 2].data,
                                  raw_buffer_size, out_width, out_height);
            }
#endif
```

**Ordering constraint (verify while editing):** this block and the existing depth send must both run *after* `transform_process_stream` (depth valid) and *before* the DMA-completion wait overwrites nothing — the previous-index raw buffer is not written again until `raw_mem_index` toggles at loop end, so sending here is race-free. State this reasoning in a comment.

- [ ] **Step 3: Build + flash + capture from boot.** Build, flash with `-rst`, then immediately capture ≥60 s from the CDC port (VID `CAFE:4001`) into `captures/golden_pairs.bin` — the capture MUST include the very first frames after boot (TNR is stateful; Task 4's comparison starts at frame 1). Verify with a decode pass: expect interleaved CALIB (every 64) / RAW / DEPTH triples, matched seq on RAW/DEPTH pairs, crc 0, gaps 0. Bandwidth sanity: (14842+9072+2332/64+3×36) B/frame ≈ 24 KB/frame at ~6 fps ≈ 145 KB/s — well inside CDC. Report frames captured and fps.

- [ ] **Step 4: Cut the committed snippet.** Extract from the capture: the first CALIB frame + the first 3 complete RAW/DEPTH seq-matched pairs **starting at the earliest captured seq** (TNR is stateful — the pairs must be the stream's first processed frames), concatenated → `host/tests/fixtures/golden_pairs_snippet.bin` (~75 KB). Method: decode with `StreamDecoder`, select the frames, and re-emit each as `pack_frame(f.header, f.payload)` — re-packing is byte-identical to the wire bytes (that equivalence is exactly what the Task-1/Phase-1 golden fixtures pin), so the snippet remains a faithful wire sample; say so in the extraction script's comment.

- [ ] **Step 5: Commit** — `feat(firmware): dual-stream RAW+DEPTH+CALIB validation mode; golden-pair fixture`

---

### Task 3: Native transform DLL + Python wrapper

**Files:**
- Create: `host/transform/CMakeLists.txt`
- Create: `host/transform/rs_transform_shim.c`
- Create: `host/transform/rs_transform_shim.h`
- Create: `host/src/roomscan/native.py`
- Test: `host/tests/test_native.py` (build-presence-gated smoke test)

**Interfaces:**
- Consumes: 53L9A1 package sources in place (same file list as `firmware/scanner-stream/CMakeLists.txt`'s transform/media sections).
- Produces: `roomscan_transform.dll` exporting `rst_create(const uint8_t *calib, uint32_t calib_len, uint32_t in_width, uint32_t in_height) -> void*`, `rst_process(void *h, const uint8_t *raw, uint32_t raw_len, float *depth_out /*54*42*/) -> int`, `rst_destroy(void *h)`; Python `roomscan.native.Transform(calib: bytes)` with `.process(raw: bytes) -> np.ndarray (42, 54) float32` and `Transform.available() -> bool` (DLL findable?). Task 4/6 consume.

- [ ] **Step 1: The shim** (`rs_transform_shim.c`) — replicate `vl53l9_app.c`'s transform setup verbatim minus sensor code. Adapted from the firmware fork (same call order: initialize → set input `raw`/`3DMD` caps (width 14842, height 1 at binning 2) → set output `depth`/`ZF32` caps (54×42) → `calib-buffer` control → prepare; then per-call: point `in_raw_mems.items` at the caller's buffer, `transform_process_stream`, copy out). Struct-for-struct it is the code at `firmware/scanner-stream/Src/vl53l9_app.c:` capabilities section — copy that block, replacing `malloc`'d frame buffers with caller-provided pointers, wrapping state in:

```c
typedef struct {
    transform_t *p_transform;
    memory_t in_mem, out_mem;
    memories_t in_mems, out_mems;
    stream_buffers_t stream_buffers;
    stream_buffer_t bufs[2];
    uint8_t calib[2332];
} rst_ctx_t;
```

`rst_create`: heap-allocate ctx, copy calib (the transform holds a pointer to it — lifetime must outlive prepare/process), run the setup sequence; any non-zero return → free and return NULL. `rst_process`: set `in_mem.data/size` to the caller's raw buffer, `out_mem.data` to `depth_out`, run `transform_process_stream`, return its code. `rst_destroy`: call the teardown that the firmware never could (`transform_finalize`, `transform_release`, `vl53l9_transform_destroy` — the commented-out block in the reference; if any crashes, document and skip with a comment — reference bug #6 says this path is untested). Export with `__declspec(dllexport)`.

- [ ] **Step 2: CMake.** `host/transform/CMakeLists.txt`: `PKG_ROOT = ${CMAKE_CURRENT_SOURCE_DIR}/../../../53L9A1`; `add_library(roomscan_transform SHARED rs_transform_shim.c <the 14 transform/media .c files copied from the firmware CMakeLists source list, each prefixed ${PKG_ROOT}/>)`; include dirs = the same 4 middleware include paths. Float model: `/fp:precise` first (document); no `/fp:fast` until Task 4 measures. Configure+build:

```powershell
cmake -S host/transform -B host/transform/build -G "Visual Studio 18" -A x64
cmake --build host/transform/build --config Release
```

Expected: `host/transform/build/Release/roomscan_transform.dll`. If MSVC trips on GCC-isms in the vendor code (VLAs, typeof, etc.), STOP and report the exact errors (fallback decision — clang-cl or vendored mingw — is the controller's).

- [ ] **Step 3: ctypes wrapper** (`native.py`): search order for the DLL: env `ROOMSCAN_TRANSFORM_DLL`, then `host/transform/build/Release/`, then alongside the package. `Transform.__init__` raises `RuntimeError` with a build hint if unavailable; `available()` classmethod for test gating. `process` validates `len(raw) == RAW_3DMD_SIZE_BIN2`, allocates the output via numpy, checks the int return.

- [ ] **Step 4: Golden-pair loader + gated smoke test.** Create `host/tests/golden.py`:

```python
"""Load the hardware golden-pair fixture: one CALIB payload + seq-matched (raw, depth) pairs."""
from pathlib import Path

from roomscan.decoder import StreamDecoder
from roomscan.protocol import StreamId

FIXTURE = Path(__file__).parent / "fixtures" / "golden_pairs_snippet.bin"


def load_golden_pairs() -> tuple[bytes, list[tuple[bytes, bytes]]]:
    frames = StreamDecoder().feed(FIXTURE.read_bytes())
    calib = next(f.payload for f in frames if f.header.stream_id == StreamId.CALIB)
    raws = {f.header.seq: f.payload for f in frames if f.header.stream_id == StreamId.RAW_3DMD}
    depths = {f.header.seq: f.payload for f in frames if f.header.stream_id == StreamId.DEPTH_ZF32}
    pairs = [(raws[s], depths[s]) for s in sorted(raws.keys() & depths.keys())]
    return calib, pairs
```

Then `host/tests/test_native.py`:

```python
import numpy as np
import pytest

from roomscan.native import Transform
from tests.golden import load_golden_pairs

pytestmark = pytest.mark.skipif(not Transform.available(),
                                reason="native transform DLL not built")


def test_create_process_destroy_smoke():
    calib, pairs = load_golden_pairs()
    t = Transform(calib)
    depth = t.process(pairs[0][0])
    assert depth.shape == (42, 54) and depth.dtype == np.float32
    assert np.isfinite(depth).all()
```

(If `from tests.golden import ...` doesn't resolve under the project's pytest rootdir config, use a relative import or conftest path shim — match how the existing test suite is invoked from `host/`.)

- [ ] **Step 5: Verify + commit** — suite green (new tests skip if DLL absent, run if present — build it first so they run). Commit `feat(host): native vl53l9 transform DLL + ctypes wrapper` (include the built-DLL path in .gitignore — never commit binaries: add `host/transform/build/` to .gitignore).

---

### Task 4: Equivalence gate — PC transform vs MCU golden pairs

**Files:**
- Create: `host/tests/golden.py` (if not created in Task 3)
- Test: `host/tests/test_equivalence.py`

**Interfaces:**
- Consumes: `golden_pairs_snippet.bin` (Task 2), `Transform` (Task 3).
- Produces: the go/no-go evidence for removing the on-MCU transform. THE GATE OF THIS PLAN.

- [ ] **Step 1: The test:**

```python
import numpy as np
import pytest

from roomscan.native import Transform
from tests.golden import load_golden_pairs

pytestmark = pytest.mark.skipif(not Transform.available(),
                                reason="native transform DLL not built")


def test_pc_transform_matches_mcu_output():
    calib, pairs = load_golden_pairs()
    assert len(pairs) >= 3
    t = Transform(calib)
    exact = 0
    for i, (raw, depth_mcu) in enumerate(pairs):   # capture order — TNR is stateful
        depth_pc = t.process(raw)
        mcu = np.frombuffer(depth_mcu, dtype="<f4").reshape(42, 54)
        if np.array_equal(depth_pc, mcu):
            exact += 1
        assert np.allclose(depth_pc, mcu, atol=0.01, equal_nan=True), \
            f"frame {i}: max abs diff {np.nanmax(np.abs(depth_pc - mcu))} mm"
    print(f"\nexact-match frames: {exact}/{len(pairs)}")
```

- [ ] **Step 2: Run.** If it PASSES: record the exact-match count in the task report — that number goes in the ROADMAP later. If it FAILS on tolerance: capture per-frame max-diff stats, try `/fp:fast` on the DLL build (matches -Ofast reassociation better) and re-run; report both results and STOP for controller review (the tolerance decision is the project owner's if diffs exceed 0.01 mm).

- [ ] **Step 3: Offline full-capture check (not committed as a test):** one-off script run against the full `captures/golden_pairs.bin` (hundreds of frames) — same comparison; report aggregate stats in the task report. Catches drift that 3 frames can't.

- [ ] **Step 4: Commit** — `test(host): PC-vs-MCU transform equivalence gate on hardware golden pairs`

---

### Task 5: **[HW]** Raw-only firmware at 30 Hz profile

**Files:**
- Modify: `firmware/scanner-stream/Src/vl53l9_app.c`

**Interfaces:**
- Consumes: Task 2's `rs_send_frame_cdc` + knobs.
- Produces: `CONF_TRANSFORM_ONBOARD (0)` mode — no `transform_*` calls in the loop (skip transform init/prepare entirely when 0; keep calib read + CALIB emission), sends RAW only; local 30 Hz profile.

- [ ] **Step 1: Profile.** Read `g_ranging_profiles[]` in `../../../53L9A1/Utilities/vl53l9-common/vl53l9/vl53l9_utils.c` (READ-ONLY) to see AR_PRECISION's `frame_period_us`/`exposure_ms`/`power`/`sync` values. In `vl53l9_app.c`, when `CONF_TRANSFORM_ONBOARD == 0`, take a local copy of the profile and set `frame_period_us = 33333u;` before `vl53l9_utils_set_profile` (keep `binning = 2` — hard requirement; keep exposure unless the sensor rejects the combination — if `set_profile` errors, report the values and stop; per the VL53L9CX datasheet 30 Hz @54×42 is a characterized operating point).
- [ ] **Step 2: Gate the transform.** Wrap transform create/init/inspect/caps/prepare AND the per-loop `transform_process_stream` in `#if CONF_TRANSFORM_ONBOARD`; when 0, the loop body is: trigger → wait GPIO (pumped) → DMA kick → **send RAW(N-1) + periodic CALIB** → wait DMA (pumped) → ack → parse metadata → update `rs_prev_counter`. Depth send is compiled out. Default knobs for this build: `CONF_TRANSFORM_ONBOARD (0)`, `CONF_STREAM_RAW (1)`.
- [ ] **Step 3: Build/flash/measure.** 20 s CDC capture: expect RAW at the sensor rate. Acceptance: **≥25 fps** raw at full res, crc 0, gaps 0 (30 fps is the profile target; the Task 8-era handshake overhead (~5 ms settle + waits) may cost some — report the measured number honestly and where the time goes if <25).
- [ ] **Step 4: Commit** — `feat(firmware): raw-only streaming mode at 30 Hz profile (transform off-board)`

---

### Task 6: Host live pipeline — transform stage + stream-colorized viewer

**Files:**
- Create: `host/src/roomscan/pipeline.py`
- Modify: `host/src/roomscan/viewer.py`
- Test: `host/tests/test_pipeline.py`

**Interfaces:**
- Consumes: `Transform` (Task 3), decoder Frames, `Deprojector`, `turbo`.
- Produces: `class TransformStage` — feeds on Frames: buffers RAW until a CALIB seen (counts `raw_skipped_awaiting_calib`), then per RAW frame emits `(header, depth: np.ndarray(42,54) f32)`; passes through DEPTH_ZF32 frames unchanged (Phase 1 firmware compatibility: `roomscan-view` works against either firmware). Viewer flag `--color {depth,reflectance,confidence}` — reflectance/confidence deferred to a follow-up if the shim exposes only ZF32 out (see Step 3 honesty note).

- [ ] **Step 1: TDD `TransformStage`** (gated on DLL like Task 4): synthetic CALIB-then-RAW sequences from golden fixtures; asserts: RAW before CALIB counted+skipped; RAW after CALIB yields depth matching `Transform.process` directly; DEPTH frames pass through.
- [ ] **Step 2: Viewer integration.** In `_reader`, route DATA frames through a `TransformStage` when `Transform.available()` and RAW/CALIB frames appear; DEPTH frames keep the existing path. HUD gains `raw` frame counter. No behavior change for Phase 1 recordings (regression: replay `synthetic.bin` + Task 9 fixture still render).
- [ ] **Step 3: Multi-stream honesty note.** The shim (Task 3) negotiates ZF32-out only. Additional outputs (reflectance/confidence/ambient/ZAPC) each need extra output-stream capabilities in the shim + wrapper API — spec'd as the natural NEXT plan (small, host-only). Do not build them here (YAGNI); the `--color` flag lands with them. Note this in the task report so the roadmap stays honest.
- [ ] **Step 4: Commit** — `feat(host): live PC-transform pipeline stage; viewer handles RAW/CALIB streams`

---

### Task 7: **[HW]** End-to-end validation + docs

**Files:**
- Modify: `ROADMAP.md`, `docs/protocol.md` (if reality diverged), `docs/transform-streams.md` (cross-ref note)

- [ ] **Step 1: Live run.** Raw-only firmware (Task 5) + `roomscan-view` with the pipeline (Task 6): live point cloud at the measured raw rate, `crc 0`, `gaps 0`, HUD sane, 60 s soak + a stall/recover check (same procedure as Phase 1 Task 11). Record a 30 s capture; replay it and confirm identical rendering.
- [ ] **Step 2: Docs.** ROADMAP Phase 2 status block with measured numbers (raw fps, equivalence exact-match stats, e2e evidence); protocol.md version-history confirmation; one-line cross-ref in transform-streams.md ("transform now runs host-side; see h563-optimization-notes.md for the retired on-MCU option").
- [ ] **Step 3: Full suite** (expect ~36+ passing, DLL-gated tests included) + firmware builds clean. Commit `docs: Phase 2 complete — raw streaming + PC-side transform, measured results`.

---

## Execution notes

- Order: 1 → 2 → 3 → 4 (GATE) → 5 → 6 → 7. Task 3 can start in parallel with Task 2's hardware steps if desired, but nothing after Task 4 starts until the equivalence gate passes.
- If Task 4's gate fails outright (diffs ≫ 0.01 mm even with `/fp:fast`), STOP the plan — the fallback conversation (tolerance acceptance vs MCU-side comparison instrumentation) is the owner's.
- Keep the dual-stream firmware mode (Task 2) behind its knob permanently — it's the regeneration path for golden pairs after any future transform-library update.
