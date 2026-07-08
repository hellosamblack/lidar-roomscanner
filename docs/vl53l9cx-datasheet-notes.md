# VL53L9CX Datasheet — Reference Notes

**Source:** VL53L9CX Datasheet, ST document DS14879, Rev 6, June 2026 ("Prerelease product(s)" on every page — the device/spec is not yet finalized). Uploaded 2026-07-08. Extracted with `pdftotext`/`pymupdf` (the PDF-render path in this environment lacked `pdftoppm`, so the doc was read as text rather than page images; tables were reconstructed from word coordinates where layout was ambiguous). The file is 51 printed pages; footer page numbers (`page N/51`) are used below and match the printed section numbering in the datasheet's own table of contents, so "page 15" etc. below means the same thing a human reader flipping to that page would see.

Every claim below is tagged with its page. Where I inferred something instead of quoting it directly, I've said so explicitly.

---

## 1. Output interface & data rates

**Two output paths exist, selected by the host at configuration time** (p.13, p.20):
- **MIPI CSI-2** — the *default* output interface.
- **Control interface (I3C or I²C)** — the host reads frame data as register bursts instead. This is what our firmware uses.

Per Table 2 (p.5), headline numbers:
- Communication (control) interface: **I²C 1 MHz / I3C 12.5 MHz**.
- Output interface: **MIPI CSI-2 1 Gbps / I3C 12.5 Mbps**.

**Direct, explicit statement of the tradeoff (p.9):** *"MIPI CSI-2 is not mandatory. It is possible to control and read the data using the communication interface (either I3C or I2C), but the maximum frame rate decreases depending on the sensor resolution and the interface."* This is the datasheet confirming, in its own words, that I3C-register-readout mode is slower than CSI-2 and that the slowdown scales with resolution — i.e., our 54×42 full-resolution choice is specifically the case where this penalty is largest.

**CSI-2 electrical (p.22, p.8 pinout table):**
- Single data lane, **200 Mbps min – 1 Gbps max**.
- Clock lane: pin table says 100–500 MHz (p.8); §5.2 says "1 clock lane @ 500 MHz" (p.22) — read the 500 MHz as the clock lane frequency at max data rate, 100 MHz as its floor.
- Recommended host config (p.24): frame width 100 "columns," data rate 1 Gbps, ISL data type 0x12, frame data type 0x2a (user-defined 0x30 at the transport-layer level, per p.23), virtual channel 0 for all data.
- Frame format uses standard CSI-2 short/long packets (frame start/end, line start/end, generic short packets) — nothing unusual (p.22–23).

**Frame content over either interface — 4 raw arrays per frame (p.20):** Range, Amplitude, Ambient, DSS (dynamic SPAD selection map). Table 12 (p.20), reconstructed by column:

| Resolution | Binning | Array size (bytes, all 4 arrays) | + 100-byte status line = Minimal frame (bytes) |
|---|---|---|---|
| 54×42 | 2 | 14742 | **14842** |
| 24×20 | 4 | 3744 | **3844** |
| 18×14 | 6 | 1638 | 1738 |
| 12×10 | 8 | 780 | 880 |
| 8×6 | 12 | 416 | 516 |
| 4×4 | 24 | 104 | 204 |

Range/Amplitude/Ambient are UINT16 (2 bytes/zone); DSS is 4 bits/zone, 2 zones packed per byte (footnote 3, p.20). **These numbers match our firmware's observed raw sizes exactly** (14842 at binning 2, 3844 at binning 4 — CLAUDE.md's numbers), which is a good cross-check that we're reading the same frame layout the datasheet describes and that the "raw" data the transform pipeline consumes is this same 4-array structure, not something upstream of it.

Data order: little-endian, column-by-column then row-by-row starting at (0,0) (p.21).

**Status line contents** (p.21) — worth knowing since it's already in every raw frame regardless of which output path is used: frame counter, sensor + laser-driver temperature, reference-array data (channel 1/2, step long/short, distance/amplitude), CSI-2 frame dimensions, static settings (mode/format/power/sync), dynamic settings (step number, context, **binning, DSS mode**), error code, FW error status, LDD error status, crop info, number of shots.

**I3C output-interface handshake (p.24, §5.3) — this is the single most important finding for our project:**

> "When using the I3C (or I2C) to output the data, the host uses the communication interface to read the array of data and acknowledge the frame. **The data is not updated until the host acknowledges the frame.** The final frame rate depends on the host capability to read and acknowledge the frame in the allotted time to achieve the expected frame rate."

In other words: over the register/control-interface output path, the sensor **will not produce a new frame until the host has read out and ack'd the current one.** The achievable frame rate on this path is *by design* gated by host readout speed, not purely by sensor integration time. This is presumably not true of CSI-2, which is a continuous push-style video interface with no per-frame host ack step.

---

## 2. Control interface & frame handshake

**I²C (p.15–16):** standard 7-bit-address, 8-bit-register-index-big-endian / 32-bit-register-little-endian protocol, up to **1 MHz (I²C fast mode+)**. Default address 0x52 (8-bit) / 0x29 (7-bit), reprogrammable via NVM (effective 1 ms after reset) or a standby-time register write. Supports autoincrement multi-byte read/write.

**MIPI I3C (p.17–19):**
- SDR-only client, release 1.0 compliant. Supports CCC commands (Table 10, p.17): ENTDAA, SETDASA, direct/broadcast activity control (ENEC/DISEC/ENTASx), SETXTIME/GETXTIME, RSTDAA, SETMWL/SETMRL, SETNEWDA, GETMWL/GETMRL/GETPID/GETBCR/GETDCR/GETSTATUS/GETMXDS.
- **DAA (dynamic address assignment) happens at ≤1 MHz; the host may raise the bus to 12.5 MHz only after DAA completes** (p.18). This matches our firmware's I3C init sequence (assign dynamic address, then presumably run faster).
- **Known limitations explicitly called out** (p.18):
  - Burst/multibyte access requires the register index to be word-aligned, *and* the host must enable the fast clock during the access (device uses the external/slow clock in STANDBY/BOOT to save power otherwise).
  - **The device does not support a repeated-start write-then-read on the same register.** The host must do a full START/WRITE/STOP, then a separate START/READ/STOP. The datasheet explicitly notes this pattern is how the driver issues an FW command (write command register 0x400) and then reads it back to confirm execution — i.e., **command issuance is inherently two full bus transactions, not one.**
  - If the device NACKs everything (communication error state), the host must issue a STOP to escape.
  - 0x00 static I²C address unsupported; an empty start-then-immediate-stop transaction is unsupported.

**Trigger modes (p.13) — three, selected at configuration time:**
1. **Autonomous** — the device generates its own trigger signal internally.
2. **External sync** — the `SYNC_IN` pin triggers new frames (pin description, p.8: "drive to logic 0 to trigger a new frame").
3. **Manual** — host sends a trigger command over the control interface (I²C/I3C) to start a new frame exposure. **This is the mode our firmware uses (`VL53L9_SYNC_MANUAL`).**

There is also a dedicated `INTR` pin, described only generically as "interrupt output, used to signal interruptions" (p.8) — the datasheet does not name it "FRAME_READY" or give a specific interrupt taxonomy; our firmware's `PLATFORM_GPIO_IT_EVT`/`PLATFORM_I3C_DMA_RX_EVT` distinction is presumably a driver-layer convention layered on top of this one physical pin, not something documented at the datasheet level.

**Boot/first-frame timing (p.13):** boot sequence is FW load → FW boot → HW init → sensor config → start streaming; "time to first range" depends on platform/bus speed/frame-rate/results config but no numeric latency is given — only the qualitative sequence diagram (Figure 9).

---

## 3. Ranging rate vs resolution (the 100 Hz question)

The "100 Hz" claim appears **twice at the headline level, unqualified**:
- Features list, p.1: "Up to 100 Hz frame rate capability."
- Overview description, p.2: "The VL53L9CX can stream processed data at maximum frame rate (100 Hz), which makes it the fastest, truly integrated 3D lidar camera module on the market."
- Table 2 technical specifications, p.5: "Sample rate: Up to 100 Hz."

**The only place 100 Hz is attached to specific conditions is Table 9, "Profile examples" (p.14), reconstructed from word positions:**

| Profile | Resolution | Ranging mode | Frame rate | Exposure | Power mode | Max range | Typical power |
|---|---|---|---|---|---|---|---|
| **Gaming** (high frame rate) | **54×42** (binning 2, full res) | Precision, **no DSS** | **100 fps** | 4 ms | Regular | 5 m | 420 mW @ 5 klx |
| Room mapping | 54×42 | Ambient | 30 fps | 6 ms | ULP | 8 m | 200 mW @ 5 klx |
| AR glasses | 54×42 | Ambient | 20 fps | 16 ms | LP | 3 m | 600 mW @ 100 klx |
| Autofocus | 24×20 | Precision | 15 fps | 5 ms | ULP | 8.8 m | 80 mW (indoor) |
| Wake on approach | 12×10 | Ambient | 1 fps | 2 ms | ULP | 8.5 m | 12.5 mW (dark) |
| Content/Volume calc | 54×42 | Precision | 1 frame/hour | 5 ms | Regular | 8.8 m | 2 mW (dark) |

**So: yes, per the datasheet, 100 fps at full 54×42 resolution is a named, specific example configuration** ("Gaming" profile) — not a lower-resolution-only number. Conditions attached: Precision ranging mode, **DSS disabled**, 4 ms exposure, Regular power mode, and a reduced max range spec of 5 m (vs 8.8 m for the standard precision profile).

**Important caveat — this number is NOT independently characterized elsewhere in the datasheet.** Section 8 ("Ranging performances," p.28–35) is the actual characterization chapter (accuracy, precision/noise, uniformity, power consumption tables), and its test-condition table, **Table 21 "Profile settings" (p.29), characterizes exactly four profiles — Precision 54×42, Ambient 54×42, Precision 24×20, Ambient 24×20 — and every one of them is run at 30 Hz**, not 100 Hz:

| Profile | Resolution | Frame rate | Exposure (dark / illuminated) | Power mode | DSS |
|---|---|---|---|---|---|
| Precision mode 54×42 | 54×42 | **30 Hz** | 4 ms / 10 ms | Regular | Enable |
| Ambient mode 54×42 | 54×42 | **30 Hz** | 4 ms / 16 ms | Regular | Enable |
| Precision mode 24×20 | 24×20 | **30 Hz** | 4 ms / 6 ms | Ultralow power | Enable |
| Ambient mode 24×20 | 24×20 | **30 Hz** | 4 ms / 8 ms | Regular | Enable |

All of Tables 22–36 (min/max distance, accuracy, precision/noise, uniformity, current consumption) are measured against these four 30 Hz profiles. **The 100 fps "Gaming" number therefore has no accompanying accuracy, noise, or uniformity data anywhere in this datasheet** — it's a marketing/use-case spec point, not a characterized measurement. Notably its own resolution/DSS/power combination (54×42, no DSS, Regular power) also doesn't correspond to any row actually measured in Section 8 (which only tests DSS-enabled configs at 54×42).

**Minimum ranging distance by mode** (Table 22, p.30, full frame): Ambient mode 450 mm; Precision mode 50 mm.

**Also relevant:** Table 21's frame-rate column is uniformly "30" across all four characterized profiles regardless of resolution — the datasheet doesn't show *rate falling as resolution rises* within its own characterization data because it only ever tests at one rate. The resolution-dependent-rate effect it does describe explicitly is the I3C/I²C penalty on p.9 quoted above, which is a bus-bandwidth statement, not a sensor-integration-time statement.

---

## 4. On-chip vs host processing split

What's computed **on-chip**, per scattered statements:
- Feature list (p.1): "On-chip processing streaming 2D IR image, depth and ambient maps"; "On-chip histogram processing and algorithmic compensation minimize or remove the impact of cover glass crosstalk and veiling glare"; "On-chip temperature monitor used for automatic temperature compensation" (p.10).
- **TNR (temporal noise reduction)** — defined in the acronym table (p.4) as "algorithm running in processing pipeline" and explicitly used when characterizing precision/noise (p.33: "Ranging precision and noise has been computed with TNR enabled"). This is a multi-frame temporal filter that appears to run **on the sensor SoC** before the range/amplitude/ambient/DSS arrays are ever exposed to the host — i.e., the "raw" arrays we read are not raw ADC histograms, they're already through on-chip histogram processing, crosstalk/veiling-glare compensation, temperature compensation, and (in default config) TNR.
- **DSS (dynamic SPAD selection)** is also on-chip firmware (p.14): keeps signal rate in range for accuracy; it's a per-frame dynamic setting reported in the status line (p.21) and can be disabled (as in the "Gaming" 100 fps profile, which explicitly runs "no DSS").

What's explicitly pushed to **host/postprocessing**:
- Feature list (p.1): "**Confidence level and reflectance maps generated in postprocessing**" — i.e., not on-chip. This lines up with our own architecture: the 4 raw arrays (range/amplitude/ambient/DSS) come off the sensor, and confidence + reflectance are derived downstream by `vl53l9-transform-c` on the M33, not by the sensor itself.
- §3.4 "Host algorithms" (p.14) is a single sentence: "STMicroelectronics recommends that the customer use the software processing pipeline provided" — pointing at the same transform library we already use, with no further detail on what it does internally (that detail lives in TN1596, "postprocessing technical note," referenced but not included in this datasheet — p.20).

**Net picture:** the split is roughly — sensor SoC does histogram accumulation, per-zone range/amplitude/ambient computation, DSS, crosstalk/glare compensation, temperature compensation, and (optionally) temporal noise reduction; host does depth-array formatting (ZF32), confidence, and reflectance. There is no raw photon-count/histogram output option described anywhere in this datasheet — "raw" in our own vocabulary (and the datasheet's "Frame content," p.20) already means "post on-chip-processing, pre-host-transform," not "pre-any-processing."

---

## 5. Facts relevant to our three levers

**Lever 1 — overlap sensor integration with MCU processing (i.e., stop using pure manual trigger-and-wait):**
- The existence of an **Autonomous trigger mode** (p.13, "the device generates its own trigger signal") is the most direct lead here. Our firmware uses Manual mode, where the host explicitly commands each frame's exposure start via a register write. Autonomous mode would let the sensor free-run its own trigger cadence independent of host commands.
- However, p.24's statement that "the data is not updated until the host acknowledges the frame" applies to the *I3C/I2C output path* regardless of trigger mode — so even in Autonomous mode, if we stay on I3C-register output, the sensor would (per this reading) hold a completed frame until we ack it before starting the next one. **This is an inference, not a directly stated fact** — the datasheet doesn't explicitly say whether Autonomous+I3C-output allows the sensor to integrate frame N+1 while frame N sits in an internal buffer awaiting host ack (true overlap), or whether it simply stalls at the same point Manual mode does. Worth testing empirically rather than assuming either way.
- The datasheet gives no exposure-vs-readout timing breakdown, so it cannot directly confirm or deny that our measured 26–29 ms "sensor trigger/integration/readout" figure is dominated by exposure time vs. I3C transfer time. What it does supply: exposure times for characterized profiles at 54×42 are 4 ms (dark) to 16 ms (bright, ambient mode) (Table 21, p.29) — so exposure alone plausibly accounts for a meaningful fraction of the 26–29 ms, with the rest being I3C register-read overhead (see Lever 3 below) plus whatever polling/ack latency our driver adds.

**Lever 2 — cheapen the transform via bypass-* toggles:**
- Not addressed by this datasheet at all — the transform library (`vl53l9-transform-c`) and its algo toggles live entirely in ST's separate postprocessing technical note (TN1596, referenced p.20 but not included here), not in DS14879.
- What this datasheet *does* establish is the boundary of what the transform library receives as input: the 4 raw arrays already reflect on-chip histogram processing, crosstalk/glare compensation, and (if enabled) on-chip TNR and DSS — meaning any host-side "bypass" toggle can only be skipping *host*-side steps (radial-to-perpendicular conversion, confidence/reflectance generation, ZF32 formatting), not re-doing sensor-side work. See Section 4 above.

**Lever 3 — stream raw sensor output to PC, run transform there:**
- Raw frame size at full 54×42/binning-2 is **14,842 bytes** (Table 12, p.20 — matches our firmware's number exactly).
- Over I3C at the max 12.5 Mbps control-interface clock, the theoretical floor for transferring one 14,842-byte frame is 14,842 × 8 / 12.5e6 ≈ **9.5 ms**, before I²C/I3C protocol overhead (per-byte ACK bit, register-index bytes for non-autoincrement transactions, and the mandatory two-transaction pattern for any command handshake, p.18). That's a *lower bound*, not a prediction — real I3C throughput will be somewhat below the raw bit rate.
- This is broadly consistent with (but doesn't fully explain) our measured 26–29 ms combined trigger+integration+readout: 4–16 ms exposure (Table 21) + ~9.5 ms+ I3C floor + protocol/ack overhead is in the right ballpark.
- Streaming raw over I3C to the PC and running the transform there would remove the **~37–40 ms on-MCU transform** step entirely, which is currently the single largest contributor to our 74 ms/frame budget — this is the most datasheet-supported argument for Lever 3 being worthwhile, independent of what I3C's exact overhead turns out to be.
- **CSI-2 would be dramatically faster for raw transfer** (14,842 bytes at up to 1 Gbps ≈ 0.12 ms, three orders of magnitude below the I3C floor) — but whether the STM32H563 has a MIPI CSI-2 receiver peripheral capable of driving this sensor is a question about the *MCU*, not something this datasheet can answer; it only describes the sensor side. Worth checking the H563 reference manual/datasheet separately before treating CSI-2 as a real option (this doc's CLAUDE.md context suggests the current transport plan is Ethernet/USB CDC over I3C-sensor-side, which implies CSI-2-to-MCU was likely already ruled out or not investigated — that determination isn't in scope for this note).

---

## 6. What the datasheet does NOT answer

- **No exposure-time vs. total-frame-time breakdown.** Table 21 gives exposure duration per profile but never states total time-per-frame (exposure + readout + settle) as a single number, so we cannot use the datasheet alone to decompose our measured 26–29 ms figure into its components.
- **No numeric handshake/ack timing.** Section 5.3 (p.24) states qualitatively that "the data is not updated until the host acknowledges the frame" and that frame rate "depends on the host capability to read and acknowledge... in the allotted time," but gives no minimum ack window, no timeout value, and no description of what happens if the host triggers a new frame before acknowledging the previous one (which is exactly the race our firmware has reportedly hit). Whether triggers are queued, dropped, or NACK'd in that case is undocumented here.
- **No confirmation of internal double-buffering (or lack thereof) on the sensor side.** Related to Lever 1 above — we cannot tell from this document whether Autonomous trigger mode + I3C output would let the sensor integrate a new frame while a previous one awaits host ack, or whether the "wait for ack" gate blocks new integration outright.
- **No characterization data for the 100 fps "Gaming" profile.** It's named with specific settings (Table 9, p.14) but none of the accuracy/precision/noise/power tables in Section 8 test it — all of Section 8's numbers are for 30 Hz profiles. So we don't know this sensor's actual noise/accuracy/thermal behavior at 100 fps full-resolution from this document; we only know ST offers it as a suggested operating point.
- **Nothing on INTR-vs-FRAME_READY pin semantics or edge ordering.** The datasheet names one generic INTR pin ("interrupt output, used to signal interruptions," p.8) with no enumeration of interrupt causes or timing relative to the I3C DMA-ready condition our firmware's `PLATFORM_GPIO_IT_EVT` / `PLATFORM_I3C_DMA_RX_EVT` distinction implies. That distinction must come from ST's driver/API documentation (not in this datasheet) or from empirical logic-analyzer work.
- **No I3C CSI-2 receiver requirements for the host MCU** — this is a sensor-only datasheet; it says what the sensor transmits/expects, not what an STM32H563 (or its CSI-2 peripheral availability) needs to receive it.
- **No thermal/duty-cycle limit tied to sustained 100 fps operation.** Laser safety (Section 10, p.40) is qualitative (Class 1 compliance) with no duty-cycle-vs-frame-rate table; thermal characteristics (Section 6, p.25) give only storage/ambient operating temperature ranges, not a power-dissipation-vs-frame-rate curve beyond the Table 36 power-consumption-by-profile numbers (all at 30 Hz per Table 21).
- **This is a "prerelease" datasheet, Rev 6, first public release (revision history, p.48).** Numbers may change in later revisions; nothing here should be treated as final silicon spec.

---

## Page-reference quick index

| Topic | Page(s) |
|---|---|
| Features/overview headline specs (100 Hz, interfaces) | 1, 2, 5 |
| Device operating modes, ranging modes, trigger modes | 12–13 |
| Profile examples table (incl. 100 fps "Gaming") | 14 |
| I²C control interface | 15–16 |
| I3C control interface, CCC commands, known limitations | 17–19 |
| Output interface, frame content, array-size table | 20–21 |
| MIPI CSI-2 interface details | 22–24 |
| I3C output-interface handshake ("data not updated until ack") | 24 |
| Electrical characteristics, current consumption | 26–27 |
| Ranging performance test conditions (Table 21, 30 Hz profiles) | 28–29 |
| Min/max distance, accuracy, precision/noise, uniformity, power tables | 30–35 |
| Laser safety | 40 |
| Revision history | 48 |
