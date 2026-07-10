# LSM6DSV16X Orientation + Environmental Panel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stream the LSM6DSV16X's SFLP orientation quaternion and its sensor-hub environmental data
(baro/mag/temp) to the PC, and show both **visually** in the roomscan panel — a live 3D orientation gizmo
plus compass + sparkline widgets.

**Architecture:** Two additive wire streams (`9 IMU_QUAT`, `10 ENV`) carry one quaternion + one env
sample per ToF frame. The host decodes them into a thread-safe `SensorState`, drives a scene coordinate-
frame gizmo from the quaternion, and renders a tilt-compensated compass + pressure/temperature sparklines.
Firmware configures the LSM's SFLP + I2C sensor-hub (SHUB), demuxes the FIFO, and emits the two streams.

**Tech Stack:** Python 3.12 + Open3D `visualization.gui` + numpy (host); STM32H5 HAL + native-I3C +
vendored `lsm6dsv16x_reg.c` (firmware); pytest; the `protocol-change` and `firmware-loop` project skills.

**Spec:** `docs/superpowers/specs/2026-07-09-lsm6dsv16x-orientation-env-panel-design.md`.

## Global Constraints

- **Two execution lanes.** Tasks 1–6 are host/protocol Python — TDD, agent-executable now against golden
  vectors, no hardware. Tasks 7–8 are firmware — the **owner drives these on-bench** (per the project's
  firmware bring-up division of labor); they are validated on-target (no unit-test framework — CLAUDE.md:
  "No unit tests — validation is on-target"), and are written as concrete guidance against the vendored
  `lsm6dsv16x_reg.c` whose exact signatures are read from the header in hand. Task 9 is joint hardware e2e.
- **Wire protocol is additive; go through the `protocol-change` skill** for Task 1 — spec (`docs/protocol.md`),
  firmware C (`rs_protocol.h`), host Python (`protocol.py`), and golden vectors stay in lockstep. Streams
  9/10 are new; hosts skip unknown `stream_id`s, so **no version bump**.
- **Stream payloads (little-endian, frozen here):**
  - `9 IMU_QUAT` = 16 B = `4×float32 [w, x, y, z]` (unit quaternion, LSM body frame).
  - `10 ENV` = 20 B = `float32 pressure (Pa)` + `3×float32 mag [x, y, z] (µT)` + `float32 temp (°C)`.
- **Cadence:** one `IMU_QUAT` and one `ENV` per ToF frame (paired). `SHUB_ODR ≈ 60 Hz`.
- **Address facts (already shipped):** ToF at `0x52`, LSM6DSV16X at `0x50` on shared I3C1. SHUB slaves:
  LPS22DF `0x5C` (baro), LIS2MDL `0x1E` (mag), STTS22H `0x38` (temp). SHT40 humidity is out of scope.
- **SFLP is 6-axis (game rotation vector) — yaw drift is accepted;** the mag is streamed for a future
  host-side correction but is NOT fused on-chip. Do not add on-chip mag fusion (it does not exist).
- **Error isolation (firmware):** any IMU/SHUB failure skips that frame's stream 9/10 emission and must
  never block, delay, or corrupt the ToF RAW/CALIB stream.
- **Host verification runs from `host/`:** `.venv/Scripts/python.exe -m pytest -q`. Keep the suite green.
- **Firmware build/flash per `firmware-loop` skill.** Commits are unsigned this session (`--no-gpg-sign`).

---

### Task 1: Protocol — add `IMU_QUAT` (9) and `ENV` (10) streams

**Files:**
- Modify: `docs/protocol.md` (stream registry + changelog)
- Modify: `host/src/roomscan/protocol.py` (StreamId + decoders)
- Modify: `firmware/scanner-stream/Src/rs_protocol.h` (stream IDs + sizes)
- Test: `host/tests/test_protocol_sensors.py` (new)

**Interfaces:**
- Produces: `StreamId.IMU_QUAT = 9`, `StreamId.ENV = 10`; `IMU_QUAT_SIZE = 16`, `ENV_SIZE = 20`;
  `decode_imu_quat(payload: bytes) -> tuple[float, float, float, float]` (w, x, y, z);
  `decode_env(payload: bytes) -> tuple[float, tuple[float, float, float], float]` (pressure_pa,
  (mx, my, mz) µT, temp_c). Task 2 consumes these.

- [ ] **Step 1: Write the failing tests**

Create `host/tests/test_protocol_sensors.py`:

```python
import struct

import pytest

from roomscan.protocol import (
    ENV_SIZE,
    IMU_QUAT_SIZE,
    ProtocolError,
    StreamId,
    decode_env,
    decode_imu_quat,
)


def test_stream_ids():
    assert StreamId.IMU_QUAT == 9
    assert StreamId.ENV == 10


def test_decode_imu_quat_roundtrip():
    payload = struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)  # identity [w, x, y, z]
    assert len(payload) == IMU_QUAT_SIZE
    w, x, y, z = decode_imu_quat(payload)
    assert (w, x, y, z) == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_decode_imu_quat_bad_length():
    with pytest.raises(ProtocolError):
        decode_imu_quat(b"\x00" * 12)


def test_decode_env_roundtrip():
    payload = struct.pack("<5f", 101325.0, 12.0, -34.0, 56.0, 21.5)
    assert len(payload) == ENV_SIZE
    pressure, mag, temp = decode_env(payload)
    assert pressure == pytest.approx(101325.0)
    assert mag == pytest.approx((12.0, -34.0, 56.0))
    assert temp == pytest.approx(21.5)


def test_decode_env_bad_length():
    with pytest.raises(ProtocolError):
        decode_env(b"\x00" * 16)
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_protocol_sensors.py -q`
Expected: FAIL (ImportError: cannot import name `IMU_QUAT_SIZE` / `decode_imu_quat`).

- [ ] **Step 3: Implement in `host/src/roomscan/protocol.py`**

Add to the `StreamId` enum after `CALIB = 8`:

```python
    IMU_QUAT = 9
    ENV = 10
```

Add near the other size constants (after `CALIB_SIZE = 2332`):

```python
IMU_QUAT_SIZE = 16  # 4x float32 [w, x, y, z], LSM body frame
ENV_SIZE = 20       # pressure f32 (Pa) + mag 3xf32 (µT) + temp f32 (°C)
```

Add decoders after `parse_ack`:

```python
def decode_imu_quat(payload: bytes) -> tuple[float, float, float, float]:
    """Decode a stream 9 IMU_QUAT payload -> (w, x, y, z) unit quaternion."""
    if len(payload) != IMU_QUAT_SIZE:
        raise ProtocolError(f"IMU_QUAT payload must be {IMU_QUAT_SIZE} bytes, got {len(payload)}")
    w, x, y, z = struct.unpack("<4f", payload)
    return w, x, y, z


def decode_env(payload: bytes) -> tuple[float, tuple[float, float, float], float]:
    """Decode a stream 10 ENV payload -> (pressure_pa, (mx, my, mz) µT, temp_c)."""
    if len(payload) != ENV_SIZE:
        raise ProtocolError(f"ENV payload must be {ENV_SIZE} bytes, got {len(payload)}")
    pressure, mx, my, mz, temp = struct.unpack("<5f", payload)
    return pressure, (mx, my, mz), temp
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_protocol_sensors.py -q`
Expected: PASS (5 passed).

- [ ] **Step 5: Update `firmware/scanner-stream/Src/rs_protocol.h`**

Add after `#define RS_STREAM_CALIB (8u) ...`:

```c
#define RS_STREAM_IMU_QUAT    (9u)  /* 4x float32 [w,x,y,z] quaternion, LSM body frame */
#define RS_STREAM_ENV         (10u) /* f32 pressure(Pa) + 3xf32 mag(uT) + f32 temp(C) */
#define RS_IMU_QUAT_SIZE      (16u)
#define RS_ENV_SIZE           (20u)
```

- [ ] **Step 6: Update `docs/protocol.md`**

Add two rows to the stream registry table after the CALIB (8) row:

```
| 9 | IMU_QUAT | LSM6DSV16X SFLP game-rotation-vector: 4×float32 `[w, x, y, z]` unit quaternion (16 B), LSM body frame. One per ToF frame. `t_us` = capture time. **6-axis fusion — yaw drifts, uncorrected on-chip.** | live (Phase 4) |
| 10 | ENV | LSM6DSV16X sensor-hub environmental sample: float32 pressure (Pa) + 3×float32 magnetic field `[x, y, z]` (µT) + float32 temperature (°C) = 20 B. One per ToF frame. `t_us` = capture time. | live (Phase 4) |
```

Add a changelog line under the version history:

```
- **v1 rev 2026-07-09**: additive — IMU_QUAT (9) and ENV (10) streams for LSM6DSV16X orientation +
  sensor-hub environmental data. No layout change; hosts skip unknown stream_ids, no version bump.
```

- [ ] **Step 7: Commit**

```bash
git add docs/protocol.md host/src/roomscan/protocol.py firmware/scanner-stream/Src/rs_protocol.h host/tests/test_protocol_sensors.py
git commit --no-gpg-sign -m "feat(protocol): add IMU_QUAT (9) + ENV (10) streams, host decoders"
```

---

### Task 2: Host `SensorState` + quaternion/heading math

**Files:**
- Create: `host/src/roomscan/sensors.py`
- Test: `host/tests/test_sensors.py` (new)

**Interfaces:**
- Consumes: `Frame`, `FrameType`, `StreamId`, `decode_imu_quat`, `decode_env` (Task 1).
- Produces:
  - `EnvSample` dataclass: `pressure_pa: float`, `mag_ut: tuple[float,float,float]`, `temp_c: float`, `t_us: int`.
  - `SensorState(history: int = 256)` with thread-safe methods: `feed(frame: Frame) -> None` (updates on
    streams 9/10, ignores others), `latest_quat() -> tuple[float,float,float,float] | None`,
    `latest_env() -> EnvSample | None`, `pressure_history() -> np.ndarray`, `temp_history() -> np.ndarray`.
  - `quat_to_matrix(w, x, y, z) -> np.ndarray` (3×3 rotation).
  - `tilt_compensated_heading(quat: tuple[float,float,float,float], mag_ut: tuple[float,float,float]) -> float`
    (degrees in [0, 360)). Tasks 3–5 consume these.

- [ ] **Step 1: Write the failing tests**

Create `host/tests/test_sensors.py`:

```python
import struct

import numpy as np
import pytest

from roomscan.protocol import (
    Frame,
    FrameHeader,
    FrameType,
    StreamId,
)
from roomscan.sensors import (
    SensorState,
    quat_to_matrix,
    tilt_compensated_heading,
)


def _frame(stream_id: int, payload: bytes) -> Frame:
    h = FrameHeader(FrameType.DATA, stream_id, 0, 1, 123, 0, 0, len(payload))
    return Frame(h, payload)


def test_quat_to_matrix_identity():
    m = quat_to_matrix(1.0, 0.0, 0.0, 0.0)
    assert np.allclose(m, np.eye(3), atol=1e-6)


def test_quat_to_matrix_90deg_about_z():
    # 90° about +Z: [w,x,y,z] = [cos45, 0, 0, sin45]
    s = np.sqrt(0.5)
    m = quat_to_matrix(s, 0.0, 0.0, s)
    # +X axis maps to +Y
    assert np.allclose(m @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-6)


def test_state_feeds_quat_and_env():
    st = SensorState()
    st.feed(_frame(StreamId.IMU_QUAT, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    st.feed(_frame(StreamId.ENV, struct.pack("<5f", 101325.0, 1.0, 2.0, 3.0, 20.0)))
    assert st.latest_quat() == pytest.approx((1.0, 0.0, 0.0, 0.0))
    env = st.latest_env()
    assert env.pressure_pa == pytest.approx(101325.0)
    assert env.mag_ut == pytest.approx((1.0, 2.0, 3.0))
    assert env.temp_c == pytest.approx(20.0)


def test_state_ignores_other_streams():
    st = SensorState()
    st.feed(_frame(StreamId.RAW_3DMD, b"\x00" * 8))
    assert st.latest_quat() is None
    assert st.latest_env() is None


def test_state_history_bounded():
    st = SensorState(history=4)
    for i in range(10):
        st.feed(_frame(StreamId.ENV, struct.pack("<5f", 1000.0 + i, 0, 0, 0, float(i))))
    p = st.pressure_history()
    assert len(p) == 4
    assert p[-1] == pytest.approx(1009.0)  # newest retained


def test_tilt_compensated_heading_level_north():
    # Level device (identity), mag pointing +X (north-ish) -> heading 0
    h = tilt_compensated_heading((1.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert h == pytest.approx(0.0, abs=1.0) or h == pytest.approx(360.0, abs=1.0)
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_sensors.py -q`
Expected: FAIL (ModuleNotFoundError: roomscan.sensors).

- [ ] **Step 3: Implement `host/src/roomscan/sensors.py`**

```python
"""LSM6DSV16X sensor state + orientation/heading math (streams 9/10). Thread-safe:
the reader thread calls feed(); the UI thread reads latest_*/history()."""
from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass

import numpy as np

from .protocol import Frame, FrameType, StreamId, decode_env, decode_imu_quat


@dataclass(frozen=True)
class EnvSample:
    pressure_pa: float
    mag_ut: tuple[float, float, float]
    temp_c: float
    t_us: int


class SensorState:
    def __init__(self, history: int = 256):
        self._lock = threading.Lock()
        self._quat: tuple[float, float, float, float] | None = None
        self._env: EnvSample | None = None
        self._pressure = deque(maxlen=history)
        self._temp = deque(maxlen=history)

    def feed(self, frame: Frame) -> None:
        if frame.header.frame_type != FrameType.DATA:
            return
        sid = frame.header.stream_id
        if sid == StreamId.IMU_QUAT:
            q = decode_imu_quat(frame.payload)
            with self._lock:
                self._quat = q
        elif sid == StreamId.ENV:
            pressure, mag, temp = decode_env(frame.payload)
            sample = EnvSample(pressure, mag, temp, frame.header.t_us)
            with self._lock:
                self._env = sample
                self._pressure.append(pressure)
                self._temp.append(temp)

    def latest_quat(self) -> tuple[float, float, float, float] | None:
        with self._lock:
            return self._quat

    def latest_env(self) -> EnvSample | None:
        with self._lock:
            return self._env

    def pressure_history(self) -> np.ndarray:
        with self._lock:
            return np.array(self._pressure, dtype=np.float64)

    def temp_history(self) -> np.ndarray:
        with self._lock:
            return np.array(self._temp, dtype=np.float64)


def quat_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Unit quaternion [w,x,y,z] -> 3x3 rotation matrix. Normalizes defensively."""
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tilt_compensated_heading(
    quat: tuple[float, float, float, float],
    mag_ut: tuple[float, float, float],
) -> float:
    """Heading in degrees [0,360): de-tilt the mag vector into the horizontal plane using
    the orientation, then atan2. Rotates the body-frame mag into world frame and reads the
    horizontal components, so the heading is correct when the device is not level."""
    r = quat_to_matrix(*quat)
    m_world = r @ np.array(mag_ut, dtype=np.float64)
    heading = np.degrees(np.arctan2(m_world[1], m_world[0]))
    return float(heading % 360.0)
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_sensors.py -q`
Expected: PASS (6 passed).

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/sensors.py host/tests/test_sensors.py
git commit --no-gpg-sign -m "feat(host): SensorState + quaternion/tilt-compensated-heading math"
```

---

### Task 3: Host sensor widgets (compass + sparkline rendering)

**Files:**
- Create: `host/src/roomscan/sensors_widgets.py`
- Test: `host/tests/test_sensors_widgets.py` (new)

**Interfaces:**
- Produces (pure numpy → `(H, W, 3)` uint8 images, mirroring `ir_image.reflectance_to_rgb`):
  - `render_compass(heading_deg: float, size: int = 180) -> np.ndarray`
  - `render_sparkline(values: np.ndarray, width: int = 220, height: int = 60, *, label: str = "", unit: str = "") -> np.ndarray`
    (empty/short `values` render a flat baseline, never raise). Tasks 5 consumes these.

- [ ] **Step 1: Write the failing tests**

Create `host/tests/test_sensors_widgets.py`:

```python
import numpy as np

from roomscan.sensors_widgets import render_compass, render_sparkline


def test_compass_shape_and_dtype():
    img = render_compass(0.0, size=120)
    assert img.shape == (120, 120, 3)
    assert img.dtype == np.uint8


def test_compass_needle_moves_with_heading():
    # The needle tip pixel region differs between N (0°) and E (90°).
    north = render_compass(0.0, size=120)
    east = render_compass(90.0, size=120)
    assert not np.array_equal(north, east)


def test_sparkline_shape():
    img = render_sparkline(np.linspace(1000.0, 1010.0, 50), width=200, height=50)
    assert img.shape == (50, 200, 3)
    assert img.dtype == np.uint8


def test_sparkline_empty_is_safe():
    img = render_sparkline(np.array([]), width=200, height=50)
    assert img.shape == (50, 200, 3)  # no exception, flat baseline


def test_sparkline_rising_trend_nonflat():
    flat = render_sparkline(np.full(50, 5.0), width=200, height=50)
    rising = render_sparkline(np.linspace(0.0, 10.0, 50), width=200, height=50)
    assert not np.array_equal(flat, rising)
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_sensors_widgets.py -q`
Expected: FAIL (ModuleNotFoundError: roomscan.sensors_widgets).

- [ ] **Step 3: Implement `host/src/roomscan/sensors_widgets.py`**

```python
"""Numpy-drawn panel widgets for the LSM6DSV16X sensors: a compass dial and a sparkline.
Pure functions producing (H, W, 3) uint8 RGB images, fed to Open3D gui.ImageWidget --
same role as ir_image.reflectance_to_rgb for the IR monitor."""
from __future__ import annotations

import numpy as np

_BG = (24, 24, 28)
_FG = (220, 220, 230)
_ACCENT = (240, 120, 90)


def _blank(h: int, w: int, color: tuple[int, int, int]) -> np.ndarray:
    img = np.empty((h, w, 3), dtype=np.uint8)
    img[:, :] = color
    return img


def _line(img: np.ndarray, x0: float, y0: float, x1: float, y1: float,
          color: tuple[int, int, int]) -> None:
    """Draw an anti-alias-free line (Bresenham-ish via sampling) into img in place."""
    n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
    xs = np.linspace(x0, x1, n).round().astype(int)
    ys = np.linspace(y0, y1, n).round().astype(int)
    h, w = img.shape[:2]
    ok = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
    img[ys[ok], xs[ok]] = color


def render_compass(heading_deg: float, size: int = 180) -> np.ndarray:
    img = _blank(size, size, _BG)
    cx = cy = size / 2.0
    r = size * 0.42
    # dial ring
    theta = np.linspace(0, 2 * np.pi, 180)
    xs = (cx + r * np.cos(theta)).round().astype(int)
    ys = (cy + r * np.sin(theta)).round().astype(int)
    img[np.clip(ys, 0, size - 1), np.clip(xs, 0, size - 1)] = _FG
    # needle: heading 0 = up (+screen -Y = north), clockwise
    a = np.radians(heading_deg)
    tipx = cx + r * 0.9 * np.sin(a)
    tipy = cy - r * 0.9 * np.cos(a)
    _line(img, cx, cy, tipx, tipy, _ACCENT)
    return img


def render_sparkline(values: np.ndarray, width: int = 220, height: int = 60, *,
                     label: str = "", unit: str = "") -> np.ndarray:
    img = _blank(height, width, _BG)
    v = np.asarray(values, dtype=np.float64)
    if v.size < 2:
        img[height // 2, :] = _FG  # flat baseline
        return img
    lo, hi = float(v.min()), float(v.max())
    span = hi - lo if hi > lo else 1.0
    xs = np.linspace(2, width - 3, v.size)
    ys = height - 3 - (v - lo) / span * (height - 6)
    for i in range(v.size - 1):
        _line(img, xs[i], ys[i], xs[i + 1], ys[i + 1], _ACCENT)
    return img
```

(The `label`/`unit` args are accepted for the panel's text label alongside the image; rendering text
into the numpy image is not needed — the panel places a `gui.Label` next to the widget.)

- [ ] **Step 4: Run to verify it passes**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_sensors_widgets.py -q`
Expected: PASS (5 passed).

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/sensors_widgets.py host/tests/test_sensors_widgets.py
git commit --no-gpg-sign -m "feat(host): compass + sparkline widget renderers for sensor data"
```

---

### Task 4: Host config fields + scene gizmo pose helper

**Files:**
- Modify: `host/src/roomscan/config.py`
- Create test additions: `host/tests/test_sensors.py` (append gizmo-pose tests)
- Modify: `host/src/roomscan/sensors.py` (add `gizmo_pose`)

**Interfaces:**
- Produces: `ViewerConfig` fields `imu_gizmo: bool = True`, `sensors_panel: bool = True`,
  `gizmo_scale: float = 0.15`; and `gizmo_pose(quat, scale, anchor) -> np.ndarray` (4×4 homogeneous pose:
  rotation from quaternion, uniform scale, translation to `anchor`). Task 5 consumes both.

- [ ] **Step 1: Write the failing tests (append to `host/tests/test_sensors.py`)**

```python
def test_gizmo_pose_identity():
    from roomscan.sensors import gizmo_pose
    m = gizmo_pose((1.0, 0.0, 0.0, 0.0), scale=0.2, anchor=(1.0, 2.0, 3.0))
    # scale on the diagonal of the rotation block
    assert m[0, 0] == pytest.approx(0.2)
    # translation column
    assert np.allclose(m[:3, 3], [1.0, 2.0, 3.0])
    assert m[3, 3] == pytest.approx(1.0)
```

And a config test — append to `host/tests/test_config.py` (create if absent) or a new
`host/tests/test_config_sensors.py`:

```python
from roomscan.config import ViewerConfig


def test_sensor_config_defaults():
    c = ViewerConfig()
    assert c.imu_gizmo is True
    assert c.sensors_panel is True
    assert c.gizmo_scale == 0.15
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_sensors.py::test_gizmo_pose_identity tests/test_config_sensors.py -q`
Expected: FAIL (ImportError `gizmo_pose`; AttributeError `imu_gizmo`).

- [ ] **Step 3: Implement**

Append to `host/src/roomscan/sensors.py`:

```python
def gizmo_pose(quat: tuple[float, float, float, float], scale: float,
               anchor: tuple[float, float, float]) -> np.ndarray:
    """4x4 pose for the orientation gizmo: rotation from quaternion, uniform scale, placed
    at anchor. Suitable for Open3D geometry.transform()."""
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(*quat) * scale
    m[:3, 3] = np.array(anchor, dtype=np.float64)
    return m
```

Add to `ViewerConfig` in `host/src/roomscan/config.py` (after the `surface_*` fields):

```python
    imu_gizmo: bool = True             # show the orientation gizmo in the scene
    sensors_panel: bool = True         # show the Sensors panel group
    gizmo_scale: float = 0.15          # gizmo axis length (metres)
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_sensors.py tests/test_config_sensors.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/sensors.py host/src/roomscan/config.py host/tests/test_sensors.py host/tests/test_config_sensors.py
git commit --no-gpg-sign -m "feat(host): sensor config fields + gizmo pose helper"
```

---

### Task 5: Host panel — wire SensorState, scene gizmo, and Sensors group

**Files:**
- Modify: `host/src/roomscan/panel.py`
- Test: `host/tests/test_panel_sensors.py` (new — headless, no GUI window)

**Interfaces:**
- Consumes: `SensorState`, `gizmo_pose`, `tilt_compensated_heading` (Task 2/4), `render_compass`,
  `render_sparkline` (Task 3), config fields (Task 4).
- Produces: panel behavior — `SensorState` fed on the reader thread; gizmo geometry updated from the
  latest quaternion each render tick; Sensors group images updated; graceful no-data.

This task integrates against Open3D `gui`, which cannot open a window headlessly in CI. Test the pure
seams (a `SensorState` owned by the panel, fed via the same site as the transform stage) and the
tick-compute functions; the on-screen gizmo/widgets are verified live in Task 9.

- [ ] **Step 1: Write the failing test**

Create `host/tests/test_panel_sensors.py`:

```python
import struct

import numpy as np

from roomscan.protocol import Frame, FrameHeader, FrameType, StreamId
from roomscan.sensors import SensorState, gizmo_pose, tilt_compensated_heading
from roomscan.sensors_widgets import render_compass, render_sparkline


def _env_frame(pressure, mag, temp):
    payload = struct.pack("<5f", pressure, *mag, temp)
    return Frame(FrameHeader(FrameType.DATA, StreamId.ENV, 0, 1, 0, 0, 0, len(payload)), payload)


def test_panel_sensor_tick_pipeline():
    # Simulate the reader->UI seam without a GUI: feed frames, compute what the tick would draw.
    st = SensorState()
    st.feed(Frame(FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, 1, 0, 0, 0, 16),
                  struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    st.feed(_env_frame(101325.0, (1.0, 0.0, 0.0), 21.0))

    quat = st.latest_quat()
    assert quat is not None
    pose = gizmo_pose(quat, 0.15, (0.0, 0.0, 0.0))
    assert pose.shape == (4, 4)

    env = st.latest_env()
    heading = tilt_compensated_heading(quat, env.mag_ut)
    compass = render_compass(heading)
    spark = render_sparkline(st.pressure_history())
    assert compass.ndim == 3 and spark.ndim == 3


def test_panel_graceful_no_data():
    st = SensorState()
    assert st.latest_quat() is None
    assert st.latest_env() is None
    # empty history renders without error
    assert render_sparkline(st.pressure_history()).shape[2] == 3
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_panel_sensors.py -q`
Expected: PASS on the seam functions already (Tasks 2–3 provide them). If it PASSES immediately, that is
correct — this test guards the integration contract; proceed to wire the panel so the contract is real.

- [ ] **Step 3: Wire `host/src/roomscan/panel.py`**

Imports (near the existing `from .ir_image import ...`):

```python
from .sensors import SensorState, gizmo_pose, tilt_compensated_heading
from .sensors_widgets import render_compass, render_sparkline
```

In `ControlPanel.__init__` (near where `self.stage` is stored), add:

```python
        self.sensor_state = SensorState()
        self.imu_gizmo = bool(getattr(args, "imu_gizmo", True))
        self.sensors_panel = bool(getattr(args, "sensors_panel", True))
        self.gizmo_scale = float(getattr(args, "gizmo_scale", 0.15) or 0.15)
        self._gizmo_added = False
```

In `_run_reader` (module function, at the `result = stage.feed(frame)` site ~line 205) feed the sensor
state on the reader thread, right before/after `stage.feed`:

```python
            state.feed(frame)   # streams 9/10 -> SensorState; ignores others
            result = stage.feed(frame)
```

Thread `state` through: add a `state` parameter to `_run_reader`'s signature and pass
`self.sensor_state` from `_reader_loop` (mirroring how `stage` is passed).

In `_build_panel`, after the IR Monitor group, add a Sensors group (guarded by `self.sensors_panel`):

```python
        if self.sensors_panel:
            sg = self._group("Sensors")
            self.compass_widget = gui.ImageWidget(_np_to_o3d(render_compass(0.0)))
            sg.add_child(gui.Label("Heading (tilt-compensated)"))
            sg.add_child(self.compass_widget)
            self.press_widget = gui.ImageWidget(_np_to_o3d(render_sparkline(np.zeros(2))))
            sg.add_child(gui.Label("Pressure (Pa)"))
            sg.add_child(self.press_widget)
            self.temp_widget = gui.ImageWidget(_np_to_o3d(render_sparkline(np.zeros(2))))
            sg.add_child(gui.Label("Temperature (°C)"))
            sg.add_child(self.temp_widget)
```

(Reuse the existing numpy→`o3d.geometry.Image` conversion the IR monitor uses; if it is inline, factor a
small `_np_to_o3d(rgb)` helper next to `reflectance_to_rgb`'s call site and use it for both.)

Add an `_update_sensors` method, called from the same render tick that calls `_update_ir`:

```python
    def _update_sensors(self):
        quat = self.sensor_state.latest_quat()
        if self.imu_gizmo and quat is not None:
            sc = self.scene_widget.scene
            if not self._gizmo_added:
                self._gizmo = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
                sc.add_geometry(_GIZMO_GEOM, self._gizmo, self.material)
                self._gizmo_added = True
            pose = gizmo_pose(quat, self.gizmo_scale, _GIZMO_ANCHOR)
            sc.scene.set_geometry_transform(_GIZMO_GEOM, pose) if hasattr(sc.scene, "set_geometry_transform") \
                else (sc.remove_geometry(_GIZMO_GEOM),
                      sc.add_geometry(_GIZMO_GEOM, self._gizmo.transform(pose), self.material))
        if not self.sensors_panel:
            return
        env = self.sensor_state.latest_env()
        if env is not None and quat is not None:
            heading = tilt_compensated_heading(quat, env.mag_ut)
            self.compass_widget.update_image(_np_to_o3d(render_compass(heading)))
        self.press_widget.update_image(_np_to_o3d(render_sparkline(self.sensor_state.pressure_history())))
        self.temp_widget.update_image(_np_to_o3d(render_sparkline(self.sensor_state.temp_history())))
```

Add module constants near the other geometry-name constants (`_GEOM`, `_MESH_GEOM`):

```python
_GIZMO_GEOM = "__imu_gizmo__"
_GIZMO_ANCHOR = np.array([0.0, 0.0, 0.0], dtype=np.float64)  # fixed scene anchor; calibrate later
```

Add a keybind to toggle the gizmo in the existing key handler (where `H` is handled), e.g. `G`:

```python
        if event.key == gui.KeyName.G and event.type == gui.KeyEvent.DOWN:
            self.imu_gizmo = not self.imu_gizmo
            if not self.imu_gizmo and self._gizmo_added:
                self.scene_widget.scene.remove_geometry(_GIZMO_GEOM)
                self._gizmo_added = False
            return gui.Widget.EventCallbackResult.HANDLED
```

Persist the three new fields in the config-save path (where `ir_colormap`/`point_size` are written):

```python
                imu_gizmo=self.imu_gizmo, sensors_panel=self.sensors_panel,
                gizmo_scale=self.gizmo_scale,
```

- [ ] **Step 4: Run tests + a headless import smoke**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_panel_sensors.py -q`
Expected: PASS.
Run: `cd host && .venv/Scripts/python.exe -c "import roomscan.panel"`
Expected: no ImportError (module imports cleanly with the new code).

- [ ] **Step 5: Run the full host suite**

Run: `cd host && .venv/Scripts/python.exe -m pytest -q`
Expected: PASS (all prior tests + the new ones green).

- [ ] **Step 6: Commit**

```bash
git add host/src/roomscan/panel.py host/tests/test_panel_sensors.py
git commit --no-gpg-sign -m "feat(host): wire SensorState + orientation gizmo + Sensors panel group"
```

---

### Task 6: Host end-to-end against a synthetic capture (host MVP milestone)

**Files:**
- Modify: `host/tests/make_fixtures.py` (add a synthetic sensor-capture builder)
- Create: `host/tests/fixtures/golden_sensors_snippet.bin`
- Test: `host/tests/test_sensors_e2e.py` (new)

**Interfaces:**
- Consumes: `pack_frame`, `FrameHeader`, `StreamId`, decoders (Task 1); `SensorState` (Task 2).
- Produces: a small binary with interleaved RAW/CALIB + IMU_QUAT + ENV frames, and a test that decodes it
  end-to-end through the decoder and confirms the sensor path populates.

- [ ] **Step 1: Add a synthetic builder to `host/tests/make_fixtures.py`**

```python
def build_sensors_snippet(path):
    """A tiny capture: CALIB, then N (RAW, IMU_QUAT, ENV) triples with a rotating quaternion."""
    import numpy as np
    from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame

    frames = []

    def data(stream_id, payload, seq, t_us):
        h = FrameHeader(FrameType.DATA, stream_id, 0, seq, t_us, 0, 0, len(payload))
        return pack_frame(h, payload)

    frames.append(data(StreamId.CALIB, b"\x00" * 2332, 1, 0))
    for i in range(8):
        ang = np.radians(i * 10.0)
        w, z = float(np.cos(ang / 2)), float(np.sin(ang / 2))
        raw = bytes(14842)
        frames.append(data(StreamId.RAW_3DMD, raw, i + 1, i * 35000))
        frames.append(data(StreamId.IMU_QUAT, __import__("struct").pack("<4f", w, 0.0, 0.0, z), i + 1, i * 35000))
        frames.append(data(StreamId.ENV, __import__("struct").pack("<5f", 101325.0 + i, 1.0, 0.0, 0.0, 21.0 + 0.1 * i), i + 1, i * 35000))
    with open(path, "wb") as f:
        f.write(b"".join(frames))
```

- [ ] **Step 2: Generate the fixture**

Run: `cd host && .venv/Scripts/python.exe -c "from tests.make_fixtures import build_sensors_snippet; build_sensors_snippet('tests/fixtures/golden_sensors_snippet.bin')"`
Expected: file created.

- [ ] **Step 3: Write the e2e test `host/tests/test_sensors_e2e.py`**

```python
from pathlib import Path

from roomscan.decoder import StreamDecoder   # existing frame decoder; .feed(bytes) -> list[Frame]
from roomscan.sensors import SensorState

FIX = Path(__file__).parent / "fixtures" / "golden_sensors_snippet.bin"


def test_sensor_state_populates_from_capture():
    data = FIX.read_bytes()
    frames = StreamDecoder().feed(data)   # same API host/tests/golden.py uses on golden_pairs_snippet
    st = SensorState()
    for frame in frames:
        st.feed(frame)
    assert st.latest_quat() is not None
    env = st.latest_env()
    assert env is not None
    assert 100000.0 < env.pressure_pa < 103000.0
    assert len(st.pressure_history()) == 8
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_sensors_e2e.py -q`
Expected: PASS.

- [ ] **Step 5: Run the full host suite**

Run: `cd host && .venv/Scripts/python.exe -m pytest -q`
Expected: PASS (all green). **This is the host MVP milestone** — orientation + env decode/visualize path
is complete and proven against synthetic data with no hardware.

- [ ] **Step 6: Commit**

```bash
git add host/tests/make_fixtures.py host/tests/fixtures/golden_sensors_snippet.bin host/tests/test_sensors_e2e.py
git commit --no-gpg-sign -m "test(host): end-to-end sensor decode against a synthetic capture"
```

---

### Task 7: Firmware — vendor LSM driver + LSM bring-up (SFLP + SHUB)  *(owner bench lane)*

**Files:**
- Create: `firmware/scanner-stream/Drivers/lsm6dsv16x/lsm6dsv16x_reg.c` + `.h` (vendored, unedited, from
  `references/software/x-nucleo-iks4a1/LSM6DSV16X/src/`)
- Create: `firmware/scanner-stream/Src/rs_lsm.c` + `firmware/scanner-stream/Src/rs_lsm.h`
- Modify: `firmware/scanner-stream/CMakeLists.txt` (add the two new `.c` files)

**Interfaces:**
- Produces (in `rs_lsm.h`):
  - `int rs_lsm_init(void);` — configure SFLP game-rotation-vector + SHUB (3 slaves) on the LSM at `0x50`.
    Returns 0 on success, negative on failure.
  - `int rs_lsm_read_latest(float quat_wxyz[4], float *pressure_pa, float mag_ut[3], float *temp_c);`
    — drain the FIFO, demux by tag, return the newest quaternion + env sample. Returns 0 if at least a
    quaternion was obtained this call, negative if nothing usable. Never blocks.

Firmware has no unit tests; verification is build + on-target observation. The code below is the concrete
integration approach against the vendored `lsm6dsv16x_reg.c` (a `stmdev_ctx_t` with read/write function
pointers) — wire those pointers to the existing native-I3C helpers to `0x50`. Confirm exact function
signatures against the vendored header as you implement.

- [ ] **Step 1: Vendor the driver**

Copy `references/software/x-nucleo-iks4a1/LSM6DSV16X/src/lsm6dsv16x_reg.c` and `.h` into
`firmware/scanner-stream/Drivers/lsm6dsv16x/`. Do not edit them (treat as vendored). Add both to the
build in `CMakeLists.txt` alongside the existing sources, and add the include dir.

- [ ] **Step 2: Implement `rs_lsm.c` — ctx + bring-up**

- Define a `stmdev_ctx_t` whose `write_reg`/`read_reg` call the file-local native-I3C private
  write/read to dynamic address `0x50` (the same transfer shape `iks4a1_i3c_probe`'s `i3c_priv_read`
  uses; add a matching private write). `handle` can be unused.
- `rs_lsm_init()`:
  1. `lsm6dsv16x_device_id_get()` → expect `0x70`; return negative on mismatch.
  2. `lsm6dsv16x_reset_set()` + poll done.
  3. Accel + gyro ODR (e.g. 120 Hz) and full-scale — SFLP needs both running.
  4. `lsm6dsv16x_sflp_game_rotation_set(ENABLE)` and set SFLP ODR (`lsm6dsv16x_sflp_data_rate_set`,
     120 Hz); enable FIFO batching of the game-rotation-vector.
  5. SHUB: `lsm6dsv16x_sh_slave_connected_set(3 slaves)`; for each of LPS22DF(`0x5C`)/LIS2MDL(`0x1E`)/
     STTS22H(`0x38`) fill an `lsm6dsv16x_sh_cfg_read_t` (7-bit addr, sub-address of the sensor's output
     register, byte count) via `lsm6dsv16x_sh_slv_cfg_read(idx, &cfg)`; set `lsm6dsv16x_sh_data_rate_set`
     (~60 Hz); one-time per-slave power-up writes via `lsm6dsv16x_sh_cfg_write()` + write-once as needed
     (LPS22DF CTRL_REG1 ODR, LIS2MDL CFG_REG_A continuous mode); `lsm6dsv16x_sh_master_set(ENABLE)`;
     enable FIFO batching of sensor-hub slaves.
  6. Set FIFO to continuous mode; return 0.

**Register-bank note:** switch `FUNC_CFG_ACCESS` only during config (the driver's `_sh_*` and `_sflp_*`
setters handle this internally); never toggle it mid-stream.

- [ ] **Step 3: Implement `rs_lsm_read_latest()` — FIFO demux**

- Loop `lsm6dsv16x_fifo_status_get()` while entries remain; `lsm6dsv16x_fifo_out_raw_get()` each word;
  switch on the tag:
  - `LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG` (0x13): 3 half-float components → reconstruct `w =
    sqrt(max(0, 1 - x² - y² - z²))`; store `quat_wxyz = {w, x, y, z}`.
  - Sensor-hub slave tags (0x0E baro, 0x0F mag, 0x10 temp — confirm mapping against the driver's tag enum
    and your slave ordering): convert each with its sensor's scale (LPS22DF: hPa = raw/4096 → ×100 = Pa;
    LIS2MDL: gauss = raw×1.5 mG/LSB → ×100 = µT; STTS22H: °C = raw×0.01).
- Keep only the newest of each; return 0 if a quaternion was seen. Bounded loop, never blocks.

- [ ] **Step 4: Build**

From `firmware/scanner-stream/` with the ARM toolchain on PATH (see `firmware-loop`):
`cmake --preset Debug && cmake --build build/Debug`
Expected: clean build, `.bin` produced.

- [ ] **Step 5: On-bench bring-up check (temporary probe)**

Behind a temporary `CONF_*` gate (like the existing probes), call `rs_lsm_init()` then loop
`rs_lsm_read_latest()` printing quat + env over VCOM (COM14 @ 921600). Flash and observe:
- WHO_AM_I path passes (init returns 0).
- Quaternion is unit-norm and tracks board rotation.
- Pressure ~101325 Pa, temp ~room, mag non-zero (µT).
Then remove/disable the temporary probe.

- [ ] **Step 6: Commit**

```bash
git add firmware/scanner-stream/Drivers/lsm6dsv16x firmware/scanner-stream/Src/rs_lsm.c firmware/scanner-stream/Src/rs_lsm.h firmware/scanner-stream/CMakeLists.txt
git commit --no-gpg-sign -m "feat(fw): LSM6DSV16X SFLP + sensor-hub driver (rs_lsm), vendored reg driver"
```

---

### Task 8: Firmware — emit streams 9/10 per ToF frame  *(owner bench lane)*

**Files:**
- Modify: `firmware/scanner-stream/Src/vl53l9_app.c`

**Interfaces:**
- Consumes: `rs_lsm_init`, `rs_lsm_read_latest` (Task 7); the existing frame-send helper
  (`rs_send_frame_cdc`) and `RS_STREAM_IMU_QUAT`/`RS_STREAM_ENV`/sizes (Task 1).

- [ ] **Step 1: Call `rs_lsm_init()` at boot**

After `rs_boot_bringup()` succeeds (LSM is at `0x50`), call `rs_lsm_init()`. On failure, emit
`rs_send_event(RS_EVT_SENSOR_INIT_FAIL, ...)` (or a new dedicated event code if preferred) and set a
`static bool g_lsm_ok = false;` — **do not** fail the ToF boot. The ToF stream must run IMU-less if the
LSM init fails.

- [ ] **Step 2: Emit streams 9/10 each ToF frame**

In the raw-only acquisition loop, after the ToF RAW frame is sent for the current iteration and only if
`g_lsm_ok`:

`rs_send_frame_cdc`'s exact signature is
`rs_send_frame_cdc(uint8_t stream_id, uint32_t seq, uint8_t flags, const uint8_t *payload, uint32_t len, uint16_t w, uint16_t h)`
(it auto-stamps `t_us` internally — there is no `t_us` argument). Use the same `seq` the RAW frame in this
iteration used (`rs_prev_counter`, per the `RS_STREAM_RAW_3DMD` send site), `flags = 0`, `w = h = 0`:

```c
        float quat[4], mag[3], pressure, temp;
        if (rs_lsm_read_latest(quat, &pressure, mag, &temp) == 0) {
            rs_send_frame_cdc(RS_STREAM_IMU_QUAT, rs_prev_counter, 0u,
                              (const uint8_t *)quat, RS_IMU_QUAT_SIZE, 0u, 0u);
            uint8_t env[RS_ENV_SIZE];
            memcpy(env + 0,  &pressure, 4);
            memcpy(env + 4,  mag,       12);
            memcpy(env + 16, &temp,     4);
            rs_send_frame_cdc(RS_STREAM_ENV, rs_prev_counter, 0u, env, RS_ENV_SIZE, 0u, 0u);
        }
```

Sent immediately after the RAW frame, so the auto-stamped `t_us` ≈ that frame's capture time (1 ms
resolution). A read failure simply skips this frame's IMU/ENV — the ToF stream is untouched.

- [ ] **Step 3: Build + flash**

`cmake --build build/Debug` then flash per `firmware-loop`. Expected: clean build, `.bin` flashed.

- [ ] **Step 4: On-bench verify streams + the cadence gate**

Run `host/.venv/Scripts/python.exe host/tools/capture.py --reset --seconds 15 --out captures/lsm_streams.bin`.
Expected (**the critical gate**):
- The report shows RAW_3DMD + CALIB **and** IMU_QUAT (9) + ENV (10) frames decoding, 0 CRC failures.
- ToF cadence unchanged: **~28 fps, 0 seq gaps** on RAW_3DMD with SHUB + SFLP active (directly tests the
  spec's one documentation-unanswerable risk — SHUB traffic vs ToF cadence). If fps drops or gaps appear,
  reduce `SHUB_ODR` / FIFO drain cost before proceeding.

- [ ] **Step 5: Commit**

```bash
git add firmware/scanner-stream/Src/vl53l9_app.c
git commit --no-gpg-sign -m "feat(fw): emit IMU_QUAT + ENV streams per ToF frame (LSM6DSV16X)"
```

---

### Task 9: End-to-end hardware verification + docs

**Files:**
- Modify: `docs/iks4a1-stacking.md` (Resolved section — add the live sensor result)
- Modify: `ROADMAP.md` (Phase 4 — mark orientation + env panel data delivered)

- [ ] **Step 1: Live panel run (both boards stacked)**

Flash the Task-8 build. Launch the panel: `host/.venv/Scripts/python.exe -m roomscan.panel` (or the
`roomscan-panel` entry point / `view-panel.bat`). Confirm:
- The orientation gizmo appears and rotates with the physical board.
- The Sensors group shows a moving tilt-compensated compass and live pressure/temperature sparklines.
- `G` toggles the gizmo; the cloud + all existing panel features still work.
- With the IKS4A1 unplugged (ToF only), no stream 9/10 arrives → gizmo hidden, sensors show "no data",
  everything else unaffected (graceful absence).

- [ ] **Step 2: Capture-based numeric confirmation**

`host/.venv/Scripts/python.exe host/tools/capture.py --reset --seconds 30 --out captures/lsm_e2e.bin` —
confirm RAW/CALIB/IMU_QUAT/ENV all present, 0 CRC, ToF ~28 fps, 0 gaps over the longer window.

- [ ] **Step 3: Update docs**

In `docs/iks4a1-stacking.md`'s "Resolved — HUB1 native-I3C" section, add a short paragraph: SFLP
orientation + SHUB env (baro/mag/temp) now stream (IMU_QUAT/ENV) and render in the panel; note the
measured ToF cadence held (fps/CRC/gaps) with SHUB active; SHT40 humidity still out (main-bus only).
In `ROADMAP.md` Phase 4, mark the orientation + environmental panel-data slice delivered, pointing to this
plan and the design spec.

- [ ] **Step 4: Commit**

```bash
git add docs/iks4a1-stacking.md ROADMAP.md
git commit --no-gpg-sign -m "docs: LSM6DSV16X orientation + env panel data delivered (Phase 4)"
```

---

## Self-Review Notes

- **Spec coverage:** orientation quaternion stream + scene gizmo (Tasks 1,2,4,5); SHUB baro/mag/temp
  stream + compass/sparkline widgets (Tasks 1,3,5); per-ToF-frame cadence (Tasks 1,8); SI units
  Pa/µT/°C (Tasks 1,3); tilt-compensated heading (Task 2); 6-axis/no-mag-fusion documented + mag streamed
  anyway (Tasks 1,7,9 + protocol.md note); firmware error isolation (Task 8); graceful absence (Task 5);
  SHUB-vs-ToF-cadence bench gate (Task 8/9); SHT40 + SLAM out of scope (Global Constraints). All covered.
- **Two-lane honesty:** Tasks 1–6 are complete-code TDD, executable now against golden vectors with no
  hardware. Tasks 7–8 are firmware guidance validated on-bench (no unit-test framework exists for it),
  written against the vendored `lsm6dsv16x_reg.c` whose exact signatures are confirmed in-hand — this is
  the accurate representation for hardware-in-the-loop firmware with an external driver, not a placeholder.
- **Type consistency:** `decode_imu_quat`/`decode_env` return types (Task 1) match `SensorState.feed`'s
  use (Task 2); `SensorState`/`gizmo_pose`/`tilt_compensated_heading`/`render_compass`/`render_sparkline`
  signatures are defined once and consumed with the same names in Tasks 5–6; wire payload layouts
  (`<4f`, `<5f`) are identical across host decoders (Task 1), the synthetic builder (Task 6), and the
  firmware `memcpy` packing (Task 8).
- **Integration seams pinned:** the host decoder is `StreamDecoder.feed(bytes) -> list[Frame]` (Task 6,
  matching `host/tests/golden.py`); `rs_send_frame_cdc(stream_id, seq, flags, payload, len, w, h)` with
  auto-stamped `t_us` (Task 8, verified against `vl53l9_app.c:353`). The one seam left to confirm at
  execution is the small numpy→`o3d.geometry.Image` helper the IR monitor already uses (Task 5 names it
  `_np_to_o3d` and says to factor it from the existing IR call site) — a rename-only concern, not logic.
