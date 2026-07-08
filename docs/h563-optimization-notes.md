# STM32H563ZI optimization notes — accelerating the vl53l9-transform-c pipeline

Scope: reduce the measured **37–40 ms/frame** transform time (2,268 zones, 54×42, float32
throughout) **without sacrificing output fidelity**, using only what the H563 silicon already
offers. Sources: STM32H562/H563 datasheet DS14258 Rev 6 (uploaded PDFs — device summary,
description, functional overview §3), our firmware at
`firmware/scanner-stream/` (roomscanner repo), and the transform library at
`53L9A1/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/` (**read-only
reference — nothing there was or should be edited**).

All cycle/time estimates below are rough order-of-magnitude, explicitly marked **(estimate)**.
None of this was measured on-target; treat every number as a hypothesis to validate with the
existing frame-time instrumentation before trusting it.

---

## 1. Chip capability inventory

| Capability | What the datasheet says | Applicability to this workload |
|---|---|---|
| **ART Accelerator — ICACHE** | 2-way set-associative (default) or direct-mapped, 0 wait-states on hit, hit-under-miss, dual master ports (fast port for internal flash/SRAM refill, slow port for external OctoSPI/FMC), critical-word-first refill, pLRU-t replacement (DS14258 §3.2.1, pp. 20–21) | **High — already in use.** All transform code executes from internal flash; ICACHE's fast master port is exactly the internal-flash path. Confirmed enabled in our firmware (see §2). No further action needed beyond confirming the working set fits reasonably in cache (8 KB icache per die, not stated in the pages read, so unverified). |
| **ART Accelerator — DCACHE** | Caches AHB **data** traffic to **external** memories only (OctoSPI/FMC); has its own master port distinct from ICACHE's (DS14258 §3.2.2, pp. 21–22) | **None.** This design has no OctoSPI/FMC/external RAM in the datapath — all buffers live in internal SRAM, which DCACHE does not front. Enabling it would be a no-op. Correctly left disabled in our firmware. |
| **SRAM banks & bus matrix** | SRAM1 256 KB, SRAM2 64 KB (ECC), SRAM3 320 KB (optional ECC, reserves 64 KB when on); all connect through the 32-bit multi-AHB bus matrix alongside CPU, GPDMA1/2, SDMMC1/2, Ethernet (DS14258 Fig. 1 p. 19; §3.5 pp. 23–24; §3.15 p. 36) | **Moderate, unexploited.** Our linker script (`STM32H563xx_FLASH.ld`) treats all 640 KB as one flat `RAM` region — no explicit SRAM1/2/3 placement. If the GPDMA-fed raw-frame double-buffer and the CPU-only working buffers (TNR candidates, sharpener group_infos, etc.) land in the *same* physical SRAM instance, DMA refill of frame N+1 and CPU compute over frame N can contend for that instance's single AHB port. Splitting them across instances is free of fidelity risk but the benefit is **speculative** until measured (§3c). |
| **CORDIC coprocessor** | 24-bit engine, Q1.31/Q1.15 fixed-point I/O, circular+hyperbolic, rotation+vectoring modes; functions: sin, cos, sinh, cosh, atan, atan2, atanh, modulus, **sqrt**, ln; single shared engine, DMA-capable (DS14258 §3.19, p. 39) | **Low.** The only per-pixel hot-loop transcendental it could reach is `sqrtf()` (used in `confidence.c` and `tnr.c`, ~2,268 calls/stage/frame). But the M33's FPU (FPv5-SP-D16) already executes `VSQRT.F32` in hardware — `sqrtf()` is *not* a slow software libm call here, it's a pipelined FPU instruction. CORDIC would add float↔Q1.31 conversion overhead and serializes all 2,268 zones through one shared engine, for no expected speedup over the instruction the FPU already has. The trig/log functions CORDIC uniquely offers (`sin`, `atan2`, `ln`) are used **once per frame**, not per-pixel (see `tnr.c` `iparams` setup), so accelerating them is immaterial to the 37–40 ms budget. **Verdict: not worth retrofitting.** |
| **FMAC (filter math accelerator)** | 16×16-bit multiplier, 24+2-bit accumulator, Q1.15 fixed-point I/O, 256×16-bit local memory, FIR/IIR (direct form 1), circular buffers, DMA read/write (DS14258 §3.20, p. 39) | **Low.** TNR's temporal blend (`depth = depth_in*alpha + depth_prev*(1-alpha)`, `tnr.c:461-474`) is structurally a 1-tap IIR — FMAC's home turf — but FMAC is a *single* filter with one active coefficient set, while TNR needs 2,268 independently-parameterized single-tap filters (`alpha` derives from a per-pixel, per-candidate `tnr_counters` value). Reprogramming FMAC's coefficient register 2,268× per frame will almost certainly cost more than the FPU's existing fused multiply-add already does per pixel. Its Q1.15 (±1) fixed-point range is also a poor match for millimeter depth values up to 12,000 mm without careful rescaling. **Verdict: not worth retrofitting.** |
| **GPDMA1 / GPDMA2** | Two independent dual-AHB-master controllers, 8 channels each, linked-list programmable, 4 of the 12 "linear" channels upgraded to 2D addressing per controller, peripheral/memory/memory-to-memory, autonomous in Sleep (DS14258 §3.16, pp. 36–38) | **Bandwidth/overlap only — not math.** GPDMA1 ch0–2 already handle I3C raw-frame ingest (`main.c` `MX_GPDMA1_Init`). GPDMA2 is completely idle. Neither engine does arithmetic; the best they offer here is offloading `memcpy`-shaped work (buffer resets, frame swaps) so the CPU isn't paying for it, and enabling genuine double-buffer pipelining between raw-frame DMA and CPU compute. This cannot reduce the ~37 ms of per-pixel float math itself. |
| **Cortex-M33 DSP/SIMD extension** | Armv8-M DSP instructions are **integer** SIMD only (packed 8/16-bit ops like `SADD16`, `SMLAD`); confirmed **no Helium/MVE** on M33 (DS14258 §3.1, p. 20 — "a set of DSP instructions"; Helium is an M55+/M85 feature, not listed anywhere for this part) | **None for the current float pipeline.** Every algo file (`confidence.c`, `tnr.c`, `sharpener.c`, `ratenorm.c`, etc.) is `float_t`/`f32` throughout. The integer SIMD instructions are unreachable without a full fixed-point rewrite — the same fidelity-risk category as the CORDIC/FMAC options above, just diffused across every stage instead of one. |
| **FPU** | Single-precision only, full Arm single-precision data-processing instruction set, hardware sqrt (DS14258 §2, p. 15: "single-precision arithmetic"); confirmed by our build flags `-mfpu=fpv5-sp-d16 -mfloat-abi=hard` (`cmake/gcc-arm-none-eabi.cmake:25`) | **Central constraint.** Any `double` arithmetic — from an un-suffixed literal (`0.5` instead of `0.5f`) or a double-returning libm call (`sqrt()` instead of `sqrtf()`) — gets software-emulated at a steep cost, since there's no hardware double path at all. See the code audit in §2 for whether this workload actually has that bug (short answer: no). |

---

## 2. Current-config audit (file:line evidence)

- **ICACHE: enabled, default 2-way set-associative.**
  `firmware/scanner-stream/Src/main.c:337` — `HAL_ICACHE_Enable()` in `MX_ICACHE_Init()`, no
  custom config, matching the HAL comment "Enable instruction cache (default 2-way set
  associative cache)" (`main.c:335`).
- **DCACHE: not initialized anywhere** in `main.c` — correct, since nothing in this design
  touches OctoSPI/FMC.
- **Voltage scaling: VOS0** (highest-performance scale, required for 250 MHz per DS14258
  §3.10.2 p. 32) — `__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0)`
  (`main.c:173`).
- **Flash wait states: `FLASH_LATENCY_5`**, programming delay 2 —
  `HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5)` (`main.c:213`);
  `__HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2)` (`main.c:220`). (The
  exact wait-state table lives in DS14258 §5.3.11, which was not part of the pages read for
  this review — WS5 is consistent with the PLL math below and with ST's usual VOS0/250 MHz
  guidance, but treat that specific cross-check as unverified against the table itself.)
- **SYSCLK = 250 MHz, confirmed by the PLL math**: `PLLM=1, PLLN=62, PLLFRACN=4096
  (→ effective N≈62.5), PLLP=2` (`main.c:189-196`) against an HSE bypass input ⇒
  VCO = HSE × 62.5, SYSCLK = VCO / 2. With the Nucleo's typical 8 MHz ST-Link MCO as HSE,
  that's 250 MHz exactly — matches the target clock.
- **Compiler flags**: `-Ofast` applied at the target level
  (`firmware/scanner-stream/CMakeLists.txt:116-118`), covering `vl53l9_app.c`, `rs_protocol.c`,
  and **all 12 algo/*.c files plus `vl53l9_transform.c`/`vl53l9_calib_utils.c`** since they're
  added directly to the `scanner_stream` target (not the separate `stm32cubemx` sublibrary).
  MCU flags `-mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb`
  (`cmake/gcc-arm-none-eabi.cmake:25`) confirm hard-float, single-precision-only FPU ABI.
  **No `-flto`/LTO anywhere** in `CMakeLists.txt` or the toolchain file — a free, zero-risk
  option left on the table (§3b).
- **Memory placement**: `STM32H563xx_FLASH.ld` places `.text`/`.rodata` in `FLASH` and
  `.data`/`.bss` in one flat 640 KB `RAM` region (`STM32H563xx_FLASH.ld:42-43`) — no
  SRAM1/SRAM2/SRAM3 split. The linker script does glob a `.RamFunc` section
  (`STM32H563xx_FLASH.ld:143-144`) but nothing in the current sources is attributed
  `__attribute__((section(".RamFunc")))`, so **no code actually executes from RAM** — the
  whole transform pipeline runs from flash through ICACHE.
- **GPDMA**: only `GPDMA1` channels 0–2 are configured (`main.c:228-253`,
  `MX_GPDMA1_Init`), servicing the I3C double-buffered raw-frame ingest documented in
  `CLAUDE.md`. `GPDMA2` is completely unconfigured/unused. No DMA use inside the transform
  library itself (confirmed — no `GPDMA`/`DMA` references in `vl53l9_app.c` outside the I3C RX
  wait).

### Double-promotion audit (the "free win" that usually hides here)

Grepped every `.c` file under `53L9A1/.../vl53l9-transform-c-lib/src/algo/` (12 files,
`confidence.c`, `depth16.c`, `distance_calibration.c`, `distance_check.c`, `dmax.c`,
`extract.c`, `flying_pixel.c`, `radial_to_perp.c`, `ratenorm.c`, `reflectance.c`,
`sharpener.c`, `tnr.c`) for two things:

1. Double-returning libm calls (`sqrt(`, `pow(`, `exp(`, `log(`, `sin(`, `cos(`, `atan(`,
   `fabs(` **without** an `f` suffix): **zero real hits.** The few regex matches were the
   custom `rsqrt()` helper name in `radial_to_perp.c:40` (a hand-written Quake-style fast
   inverse-sqrt using one bit-twiddle + two Newton-Raphson iterations, `radial_to_perp.c:40-49`
   — already a deliberate fixed/float hybrid optimization, not a bug) and MISRA-rule
   comment text (`// MISRAC2012-Dir-4.11_...`).
2. Un-suffixed float literals in arithmetic context: no genuine hits either — every match was
   either a MISRA-rule comment, a copyright/version string, or a `@param` doc comment
   (e.g. `ratenorm.c:91 "-0.75 is standard"` is documentation, not code).

**Conclusion: this codebase does not have the double-promotion bug.** Every math call I could
find is correctly f-suffixed (`sqrtf`, `powf`, `expf`, `fabsf`, `sinf`), and float literals are
consistently `f`-suffixed. This is unusually clean — plausibly because ST's MISRA-C tooling
(the `MISRAC2012-Dir-4.11` annotations visible throughout) already flags implicit
float→double promotion. **This is good news but means the single biggest fidelity-neutral win
people usually find here isn't available in this codebase.**

Call-site density that *is* real (per-pixel, 2,268 zones/frame):

| Function | Call sites (grep count) | Files |
|---|---|---|
| `sqrtf(` | 8 | `confidence.c` (×2, per-pixel), `tnr.c` (×3, per-pixel/candidate), `dmax.c` (×2), `radial_to_perp.c` (×1) |
| `powf(` | 8 | `tnr.c` (×1, once/frame), `sharpener.c` (×4, per-pixel), `reflectance.c` (×1, per-pixel), `ratenorm.c` (×2, per bicubic tap) |
| `expf(` | 1 | `sharpener.c` (per-pixel, gaussian branch only) |
| `fabsf(` | 13 | spread across `tnr.c`, `ratenorm.c`, `flying_pixel.c`, `radial_to_perp.c` — cheap (compiles to a sign-bit clear, not a real call) |

The `powf`/`expf` sites are the interesting ones: `ratenorm.c:611-617` (`cubic_kernel`, called
per bicubic interpolation tap) and `sharpener.c:261-267` (gaussian/distance scoring, called
per pixel, up to twice per pixel in `SHARPENER_MODE_DOUBLE_SHARP`) call `powf(x, 2.0f)` and
`powf(x, 3.0f)` — **constant integer exponents**. A generic `powf()` is an `exp(y·ln(x))`
routine costing on the order of 100+ cycles; `x*x` or `x*x*x` costs 1–2 FPU cycles. Compilers
do *not* reliably fold `powf(x, 2.0f)` into a multiply under `-Ofast`/`-ffast-math` on
arm-none-eabi's newlib-nano libm the way they might on a desktop libm — this is a real,
measurable, bit-exact-or-better opportunity, detailed below. (`sharpener.c:266-267`'s *outer*
`powf(sum, distance_power)` uses a runtime parameter, not a constant — that one has to stay a
real `pow()` call.)

---

## 3. Ranked recommendations

### Fidelity-neutral (do these first)

**(a) Replace constant-exponent `powf()` with direct multiplies.**
Sites: `ratenorm.c:611-617` (`cubic_kernel`, 2× `powf(_, 2.0f)`/`powf(_, 3.0f)` per call, called
per bicubic tap — up to 16 taps/pixel in the interpolation kernel) and `sharpener.c:261-267`
(2× `powf(_, 2.0f)` per pixel in the gaussian-distance branch, up to 2× per pixel in
double-sharp mode). Replacing `powf(d, 2.0f)` → `d*d` and `powf(fabsf(d), 3.0f)` →
`d2*fabsf(d)` (reusing `d2`) is **more** accurate than the generic pow path (no
exp/log round-trip), not less — this is bit-exact-or-better, not an approximation.
- **Estimated gain**: with up to ~10–30K such calls/frame across both stages
  (2,268 zones × up to ~4–16 calls depending on mode/kernel size) at ~100+ cycles each vs.
  ~2 cycles for the multiply, a conservative estimate is **0.3–2 ms/frame** recovered
  **(estimate — needs on-target measurement)**.
- **Effort**: trivial, ~10 lines total.
- **Blocker**: both files live under `53L9A1/Middlewares/...` — **read-only reference** per
  this project's own rules. This cannot be hand-edited in place. Options: (1) request/track
  this as an upstream fix with ST, or (2) compile a locally-patched copy of just these two
  files into `roomscanner/firmware/scanner-stream/` (shadowing the reference build via
  `target_sources` ordering) if the project is willing to fork that much of the library. Flag
  this decision to the project owner before doing anything.

**(b) Enable LTO for the `scanner_stream` target.**
Add `-flto` to `cmake/gcc-arm-none-eabi.cmake`'s `TARGET_FLAGS`/linker flags and
`CMakeLists.txt`'s target compile options. Zero fidelity risk beyond what `-Ofast` already
introduces (LTO doesn't add new floating-point reassociation on its own — `-Ofast`'s
`-ffast-math` already does). Lets the compiler inline/specialize across the 12 algo TUs and
`vl53l9_transform.c`'s per-stage dispatch instead of treating each as an opaque
compilation unit.
- **Estimated gain**: modest, likely low single-digit percent of the 37–40 ms, since each TU
  is already `-Ofast`-compiled individually **(estimate)**.
- **Effort**: low — flag-only change. Must re-check the post-build `arm-none-eabi-size`
  output against the 2 MB flash budget, since LTO can shift code size either direction.

**(c) SRAM bank placement to reduce bus-matrix contention.**
Pin the GPDMA1-fed raw-frame double-buffer to one SRAM instance (e.g. SRAM3) via a linker
section + `__attribute__((section(...)))` on the relevant static buffers in
`vl53l9_app.c` (which **is** project-owned, unlike the library), keeping CPU-only working
buffers (TNR candidate arrays, sharpener `group_infos`, etc. — these live inside the
library's context structs, so this only helps for buffers `vl53l9_app.c` itself owns) in a
different instance. This targets the AHB bus matrix (DS14258 §3.15) so DMA refill of frame
N+1 and CPU compute over frame N's buffers don't serialize on the same SRAM port.
- **Estimated gain**: unverified — could be near-zero if the bus matrix already arbitrates
  this cheaply, or meaningful if there's real contention. **Needs a before/after
  measurement**, not just applied blind.
- **Effort**: moderate (linker script edit + attribute annotations + retest).

### Fidelity-questionable (clearly separate bucket — only pursue if (a)–(c) aren't enough)

**(d) CORDIC for the `sqrtf()` call sites.** **Not recommended.** The M33's FPU already
executes `VSQRT.F32` in hardware; CORDIC would add float↔Q1.31 conversion overhead and
funnel all 2,268 zones through one shared engine for no expected win over the instruction
already in use. The trig/ln functions CORDIC actually adds value for are called once per
frame in this codebase, not per-pixel, so there's no hot loop for CORDIC to accelerate here.

**(e) FMAC for the TNR temporal IIR blend.** **Not recommended.** FMAC is a single
filter/single-coefficient-set engine; TNR needs 2,268 independently-parameterized 1-tap
filters (`alpha` varies per pixel per candidate via `tnr_counters`). Per-pixel FMAC
coefficient reprogramming almost certainly costs more than the FPU's existing
fused-multiply-add. FMAC's Q1.15 (±1) range is also a poor fit for 0–12,000 mm depth values
without careful, fidelity-risking rescaling.

Both of these would require porting real chunks of the pipeline to fixed-point to see any
benefit at all, which is squarely the fidelity-sacrificing territory the project explicitly
wants to avoid — not recommended given the "without sacrificing output fidelity" constraint.

### Non-transform wins (bandwidth/overlap — don't touch the CPU-bound math)

**(f) Offload the TNR reset-path `memcpy`s to GPDMA2.** `tnr.c:335-349` does 5×
9,072-byte buffer copies, but only on the very first frame after a context reset — not
steady-state. Fidelity-neutral, trivial, but the gain is confined to that one reset frame,
not the sustained 37–40 ms/frame this project is trying to shrink.

**(g) Use the idle GPDMA2 for genuine pipelining.** Since GPDMA1 already handles raw-frame
ingest and GPDMA2 is completely unused, it could prefetch the *next* raw frame or handle
output-side packing (`depth16`/protocol framing) in parallel with the CPU starting the next
frame's `extract` stage. This is real overlap, not a math speedup — moderate-to-high effort
(restructuring `vl53l9_app.c`'s acquisition loop), and it reduces wall-clock *between*
frames, not the 37–40 ms of CPU-bound math itself.

---

## Summary for the reader in a hurry

The two real, cheap, fidelity-neutral wins are **(a) kill the constant-exponent `powf()`
calls** (blocked on a read-only-reference decision) and **(b) turn on LTO** (no blocker,
just do it). SRAM bank placement **(c)** is plausible but unverified. CORDIC and FMAC are
both poor fits for this specific workload — the FPU's hardware sqrt already beats CORDIC's
sqrt for this use case, and FMAC's single-filter model doesn't match TNR's
2,268-independently-parameterized-taps structure. GPDMA2 is free capacity but only buys
memory-bandwidth/overlap wins, not compute wins — it cannot touch the actual 37–40 ms of
per-pixel float math. No double-promotion bug was found; the codebase is already
disciplined about `f`-suffixed literals and libm calls.
