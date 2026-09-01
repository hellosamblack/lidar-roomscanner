# C270 / Pi-bridge characterization — 2026-09-01 (issue #226, spec §5.1)

Measured with `pi-bridge/tools/cam_characterize.py` on the real bridge
(`roomscan-bridge`, 172.17.2.78) **while the scanner was streaming live** to the
owner's `roomscan-web` on the PC host. Raw JSON per arm is in
`cam_characterize_results_20260901/`; the numbers below are read out of it, not
retyped from memory.

The spec's own words: *"writing the daemon first and measuring later is
explicitly out of order."* This file is the gate that unblocked §5.2.

## Verdict

**Advertised mode: MJPG 1280×720 @ 30 fps** — it passed, at 29.73–29.80 fps
sustained over 90 s with zero invalid JPEGs, zero V4L2 buffer errors and
5 % of one Pi core, with no measurable effect on the scanner path.

**But only with `exposure_dynamic_framerate=0`.** That is the single most
important finding here and it is a control, not a mode: with the value the
device was actually carrying (`1`), the C270 delivers **14.7–14.9 fps** in this
room's light at *every* resolution, 720p and 480p alike. The uapi default is
`0`; something had set it to `1` on this unit, and the spec's 2026-08-28
read-only probe recorded that state without measuring its consequence. The
daemon must set this control explicitly and report the readback — a mode that
enumerates at 30 fps and a mode that delivers 30 fps are different claims.

## Environment as measured (not assumed)

| Item | Value |
|---|---|
| Board | Raspberry Pi 3 Model B, `armv7l`, 4 cores, 920 MB RAM |
| OS / Python | Raspbian GNU/Linux 13 (trixie), kernel `6.18.34+rpt-rpi-v7`, system `python3` **3.13.5** |
| Available tooling | `v4l2-ctl`, `tcpdump`, `nft`, `vcgencmd`, `iw` (in `/usr/sbin`, **not** on the login PATH). **No** `pip`, `ffmpeg`, `gst-launch-1.0`, or `python3-v4l2` |
| Camera | Logitech C270, USB `046d:0825`, serial `7936D010`, `uvcvideo`, card `C270 HD WEBCAM`, bus `usb-3f980000.usb-1.4` |
| Nodes | `/dev/video0` (Video Capture + Streaming) and `/dev/video1` (metadata; **capture=False**) |
| USB tree | one 480M root hub → SMSC9514 hub carrying **eth0 (the scanner's Ethernet NIC)**, the C270 (port 4), and the scanner's CDC-ACM debug interface (port 5, 12M) |
| wlan0 | 2.4 GHz, SSID `Surly_Office`, RSSI −40…−47 dBm, tx bitrate 72.2 Mbit/s throughout |
| `roomscan` account | already in the `video` group — the daemon needs no new group |

`roomscan` is in `video`, so nothing here needs root: every capture below ran
as the unprivileged bridge account. Only `nft` and `tcpdump` used the existing
`sudo -n` grant.

### Device selection

`select_device()` resolves `046d:0825` + Video Capture + Streaming + a
`video-index0` link and picked:

```
/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.4:1.0-video-index0
```

by-path is preferred over by-id **deliberately**: the spec asks for the stable
path of the physical port. `/dev/video1` is correctly rejected — it carries the
same USB id but reports `Video Capture = False`, so "first `/dev/video*` with
the right VID:PID" would have picked a metadata node roughly half the time.

### V4L2 ioctl layout — a real trap, resolved by probing

`struct v4l2_buffer` embeds a `struct timeval`, and the ioctl request number
encodes the struct's size. Raspbian 13 armhf is a **64-bit-`time_t`** port, so
the struct is 80 bytes (`VIDIOC_DQBUF`), not the 68 bytes a 32-bit-`time_t`
32-bit userspace would use (`VIDIOC_DQBUF_TIME32`). The kernel accepts both
numbers; only one matches what the process packs. `v4l2_capture.py` probes both
with a real `VIDIOC_QUERYBUF` and records the answer rather than deriving it
from `platform.architecture()`:

```
buffer_layout: "time64"
```

### Timestamps and controls

Every dequeued buffer on every arm carried `V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC`:

```
timestamp_domains: {"pi_monotonic": 2686}      (arm B, all frames)
```

so `timestamp_valid` is true and `X-Roomscan-Timestamp-Domain: pi_monotonic` is
honest on this unit. Dequeue time is recorded separately and never renamed
capture time.

Controls read back after `exposure_dynamic_framerate=0` (arm B):

```
auto_exposure               = 3 (Aperture Priority Mode)   [default 0]
exposure_dynamic_framerate  = 0                            [default 0]
exposure_time_absolute      = 333 (inactive; auto exposure owns it)
white_balance_automatic     = 1
white_balance_temperature   = 4040 (inactive)
power_line_frequency        = 2 (60 Hz)
gain = 24, brightness = 128, contrast = 32, saturation = 32, sharpness = 24
backlight_compensation      = 1
```

## Modes tried, and what each actually delivered

Every row was measured. Nothing is extrapolated. Rows marked *12 s probe* are
short-window mode probes, not full 90 s arms.

| Mode | `exposure_dynamic_framerate` | Achieved fps | Gap p95 / max (ms) | JPEG median / p95 / p99 / max (B) | Window |
|---|---|---|---|---|---|
| MJPG 1280×720@30 | **0** | **29.74** (steady 29.80) | 36.01 / 36.10 | 68 673 / 68 882 / 68 957 / 69 058 | 90 s (arm B) |
| MJPG 1280×720@30 | **0** | **29.73** (steady 29.79) | 36.01 / 67.96 | 53 841 / 68 894 / 69 001 / 69 122 | 90 s (arm C) |
| MJPG 1280×720@30 | 1 | 14.72 | 68.0 / 248 | 82 934 / 83 149 / 83 239 / 83 253 | 12 s probe |
| MJPG 640×480@30 | 0 | 29.44 | 36.0 / 180 | 18 023 / 18 108 / 18 141 / 18 153 | 12 s probe |
| MJPG 640×480@30 | 1 | 14.87 | 68.0 / 180 | 26 815 / 26 932 / 27 012 / 27 033 | 12 s probe |

The `max` of 180 ms in short probes is the stream-startup transient (first
frame after `STREAMON`); the arms discard the first second before computing
`_steady` statistics, which is why arm B's steady max is 36.10 ms.

YUYV was not benchmarked: the enumeration itself rules it out for this
product. YUYV 1280×720 tops out at **10 fps** and is uncompressed
(1.84 MB/frame ⇒ 18 MB/s), which does not fit the wlan0 uplink at any cadence.
MJPG advertises the full 30 fps ladder from 160×120 to 1280×720.

## The three arms (§5.1), 90 s each, scanner streaming throughout

Arm A = camera off. Arm B = preview on (one latest-wins MJPEG consumer over
wlan0 to a receiver on the PC host). Arm C = preview + a second 2 Hz snapshot
consumer on its own connection.

### Scanner path — no regression, in any arm

| Observable (Pi side) | A baseline | B preview | C preview+snapshot |
|---|---|---|---|
| eth0 rx bytes/s | 487 691 | 487 541 | 487 655 |
| eth0 rx packets/s | 451.9 | 451.8 | 451.9 |
| eth0 rx dropped/s, errors/s | 0.0, 0.0 | 0.0, 0.0 | 0.0, 0.0 |
| nft `roomscan-fwd-established` packets/s | 452.9 | 452.8 | 452.9 |
| nft `roomscan-fwd-established` bytes/s | 487 644 | 487 546 | 487 684 |
| pcap inter-packet gap p95 / **max** (ms) | 5.045 / **9.128** | 5.045 / **8.771** | 5.045 / **7.949** |
| pcap packets in window | 6 325 | 6 330 | 6 322 |

The max inter-packet gap is the observable a rate cannot see: a bus-contention
stall would show as a hole even while the average held. It did not move —
if anything it was *lowest* in the heaviest arm, which is noise, not a win.

### Scanner path — no regression, measured on the host too

A single read-only `host/tools/ws_probe.py` client watched `/ws` for 25 s inside
each arm's window (never sent a command, never recorded, closed after each arm):

| Host-side `/ws` arrival | A baseline | B preview | C preview+snapshot |
|---|---|---|---|
| cloud tag 1 | 30.00 Hz, 1 458 365 B/s | 30.00 Hz, 1 458 210 B/s | 30.00 Hz, 1 457 988 B/s |
| sensor tag 2 | 15.00 Hz, 102 180 B/s | 15.00 Hz, 102 180 B/s | 15.00 Hz, 102 180 B/s |
| `metrics` / `sensor` / `session` JSON | 4.00 / 15.00 / 4.04 Hz | 4.00 / 15.00 / 4.04 Hz | 4.00 / 15.00 / 4.04 Hz |
| ws / mesh errors | none | none | none |

Identical to four significant figures across the three arms.

### Pi cost

| | A baseline | B preview | C preview+snapshot |
|---|---|---|---|
| Instrument process CPU (% of one core) | 0.5 | **4.6** | **5.1** |
| System busy (all 4 cores) | 4.2 % | 8.9 % | 7.5 % |
| wlan0 tx bytes/s | 494 326 | **2 529 681** | **2 370 267** |
| wlan0 tx dropped/s | 0.0 | 0.0 | 0.0 |
| SoC temperature over the arm | 55.8–58.0 °C | 58.0–60.7 °C | 61.8–63.4 °C |
| `vcgencmd get_throttled` | `0x0` | `0x0` | `0x0` |
| wlan0 tx bitrate | 72.2 Mbit/s | 72.2 (one 65.0 sample) | 72.2 Mbit/s |

Camera adds ~2.0 MB/s to a wlan0 uplink already carrying the scanner's
0.49 MB/s, i.e. **~2.5 MB/s ≈ 20 Mbit/s on a 72.2 Mbit/s link**. Nothing
throttled; the highest temperature seen anywhere was 63.4 °C.

### Camera cadence and frame size (Pi side, 90 s)

| | B preview | C preview+snapshot |
|---|---|---|
| Frames captured | 2 686 | 2 685 |
| fps achieved / steady | 29.741 / 29.801 | 29.730 / 29.789 |
| Inter-frame gap median / p95 / p99 / steady max (ms) | 32.01 / 36.01 / 36.02 / 36.10 | 32.01 / 36.01 / 36.02 / 67.96 |
| JPEG bytes median / p95 / p99 / max | 68 673 / 68 882 / 68 957 / 69 058 | 53 841 / 68 894 / 69 001 / 69 122 |
| JPEG bytes/s | 2 030 770 | 1 694 854 |
| Invalid JPEG (SOI/EOI check) | 0 | 0 |
| V4L2 buffer errors | 0 | 0 |
| Driver sequence gaps | 1 in 2 686 | 2 in 2 685 |

(Arm C's lower *median* JPEG size is the auto-exposure tracking the room, not a
codec change: its p95/p99/max match arm B to within 0.3 %.)

### Far side: host-arrival age

Frames are stamped at V4L2 capture (Pi monotonic), at send, and again at
arrival on the PC host, and the Pi↔host clock offset is **measured** with an
NTP-style four-timestamp exchange rather than assumed from NTP. Offset
7.3–8.1 ms with a half-RTT uncertainty of **±1.2 ms**.

| Age (capture → host socket), ms | B preview | C preview | C snapshot |
|---|---|---|---|
| median | 55.16 | 52.21 | 72.05 |
| p95 | **89.61** | **90.38** | 137.15 |
| p99 | 395.87 | 273.31 | 377.09 |
| max | 3 158.95 | 462.46 | 505.22 |
| first-decile → last-decile median | 56.27 → 55.15 | 57.45 → 51.91 | 97.91 → 69.52 |

Two things this says and one it does not:

- **No monotonic growth.** The age at the end of a 90 s arm is *lower* than at
  the start in every case. A queue would have grown; latest-wins did what it is
  supposed to do.
- **The tail is wlan0, not a queue.** Arm B's 3.2 s maximum is a single arrival
  gap of 2 680 ms (one 2.4 GHz retransmission stall) after which the stream
  resumed at the newest frame. It is one frame out of 2 566. Arm C, on a
  quieter minute, peaked at 462 ms.
- **This is not the browser-pixel latency §5.1 requires.** It is the Pi half:
  capture → host socket. The host proxy, decode and compositor are still to
  come, and the ≤150 ms p95 / ≤250 ms hard gate applies to *pixels*. The 90 ms
  p95 measured here leaves ~60 ms of budget for the rest, which looks feasible
  and is not yet proven.

Of that 90 ms p95, the capture→send half is 36.96 ms p95 and the wlan0 transit
half is 49.56 ms p95. The capture→send figure is dominated by the ~33 ms frame
interval itself (a frame is not sendable until it exists), so there is little
to win on the Pi side.

### Latest-wins actually drops frames, and it is counted

| | B preview | C preview | C snapshot |
|---|---|---|---|
| sent | 2 566 | 2 656 | 178 |
| skipped by the latest slot | 117 (4.4 %) | 29 (1.1 %) | 2 506 (deliberate: 2 Hz) |
| arrived at the host | 2 566 @ 28.49 fps | 2 656 @ 29.49 fps | 178 @ 1.97 fps |
| bad JPEG at the host | 0 | 0 | 0 |

The snapshot consumer at 2 Hz skipped 2 506 frames and cost nothing measurable:
adding it moved system CPU from 8.9 % to 7.5 % (i.e. into the noise) and moved
the preview's own p95 age by 0.8 ms.

## What was NOT measured, and must not be claimed

1. **Browser-pixel latency.** §5.1's fiducial stimulus needs a host-timed
   black/white transition on a monitor *in the camera's view*. It is not:
   the camera currently sees a padded black equipment case in the near field,
   a cream painted panelled wall, and a wooden door frame, tilted ~10–15° off
   horizontal (one reference frame is at `/tmp/camchar/scene_20260901.jpg`,
   deliberately untracked). Someone has to aim the camera at a screen.
2. **The five-minute gate.** These arms are 90 s each. Nothing here trends,
   but a 90 s window cannot answer a five-minute question.
3. **Scanner fragment loss / reorder / incomplete counts, `/ws` backlog, and
   broadcaster `tick_share`.** `host/tools/ws_probe.py` has **no `metrics`
   handling at all** — it counts message *types*, and never reads the fields
   inside them. What it can prove (and did) is the far-side arrival rate and
   bytes of the scanner's cloud/sensor payloads, which is a real observable and
   was flat. The four named counters need a different probe.
4. **Unplug/replug and daemon restart.** Physical, and out of a worker's reach.
5. **Low light.** Everything above is one room, one time of day, with room
   lights on. `exposure_dynamic_framerate=0` buys 30 fps by refusing to extend
   exposure; in a genuinely dark room it will buy 30 fps of a darker image, not
   30 fps of the same image. The 720p JPEG shrank from 82.9 KB (dynfr=1) to
   68.7 KB (dynfr=0) here, which is that trade showing up in the compressor.

## Instrument defects found and fixed while measuring

Recorded because both would have produced confident wrong numbers:

- **A stale pcap read as a fresh one.** `scanner_pcap_gaps()` wrote to a fixed
  `/tmp` path. tcpdump drops privileges to the `tcpdump` user, so a second run
  could not replace the first run's file, and the failure was silent — arm A
  printed `seconds: 15` beside a 5-second run's packet count, byte for byte
  identical. Now: a pid+instant-unique path, tcpdump's exit status and stderr
  reported rather than swallowed, an explicit `file_is_fresh` mtime check that
  refuses to report a stale capture, and removal with the privilege that
  created the file.
- **The rate-limited snapshot sender busy-spun.** It skipped a frame without
  advancing its `last_seq`, so the latest slot handed it the same frame again
  immediately. The first run of arm C reported **98.3 % of one core** and
  **147 dropped camera frames at 28.12 fps** — all of it the instrument
  starving its own capture thread through the GIL. Fixed (consume the frame,
  then wait), the same arm reads 5.1 % and 29.73 fps. The 90 s arm C above is
  the re-run.
