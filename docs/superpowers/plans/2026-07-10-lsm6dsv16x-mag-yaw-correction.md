# LSM6DSV16X Magnetometer Yaw-Drift Correction — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bound the SFLP quaternion's free-running yaw drift host-side by grafting a gated, long-time-constant, tilt-compensated magnetometer heading onto it (yaw-only; tilt stays SFLP), producing a drift-corrected orientation for the live gizmo and the future ICP rotation prior.

**Architecture:** All host-side Python in the `roomscan` package, zero firmware/protocol change (stream 9 SFLP quat + stream 10 mag already flow). A hard/soft-iron magnetometer calibration (`magcal.py`) feeds a stateful yaw complementary filter (`YawFusion` in `sensors.py`) exposed as `SensorState.fused_quat()`. The filter maintains a running scalar yaw-offset `delta`, driven toward the mag heading only when validity gates (magnetic-anomaly, fast-motion, gimbal-lock) pass, and applied as a world-frame Z rotation `Rz(delta) ⊗ q_sflp` — which provably changes heading only, preserving SFLP tilt.

**Tech Stack:** Python 3.11+, NumPy. No new third-party dependencies (the ellipsoid fit is pure NumPy `lstsq`/`eigh`). `pyserial` is used only by the calibration CLI and must stay a deferred import (tests never touch hardware).

## Global Constraints

- **Python ≥ 3.11** (project already uses `tomllib`). Type hints use `X | None`, `from __future__ import annotations` at module top (match existing files).
- **No new runtime dependencies.** NumPy only for the math; `pyserial` deferred-imported inside functions, never at module top (mirror `sources.py:31`).
- **Quaternion convention: `[w, x, y, z]`, unit, LSM body frame** — identical to existing `sensors.py`. Reuse `quat_to_matrix`; do not introduce a second convention.
- **Thread-safety:** all mutable `SensorState` access stays under the existing `self._lock`; reader thread calls `feed()`, UI thread reads `fused_quat()`/`fusion_status()`.
- **Backward compatibility:** the existing `tilt_compensated_heading(quat, mag_ut)` signature and behavior must not change (panel + tests depend on it).
- **Degraded modes never crash:** missing/invalid calibration or fusion disabled ⇒ `fused_quat()` returns the raw SFLP quat (today's behavior).
- **Test command:** `cd host && python -m pytest <path> -v`. Run from the `host/` directory (that is where the package + tests live).
- **Angles:** headings/yaw in **degrees**; wrap helpers keep values in `[-180, 180)` for errors and `[0, 360)` for absolute headings, matching the existing `heading % 360.0` convention.

---

### Task 1: Magnetometer calibration model + persistence (`magcal.py`)

**Files:**
- Create: `host/src/roomscan/magcal.py`
- Test: `host/tests/test_magcal.py`

**Interfaces:**
- Produces:
  - `MagCalibration` — frozen dataclass with fields `offset: tuple[float, float, float]` (hard-iron, µT), `matrix: tuple[tuple[float, ...], ...]` (3×3 soft-iron, row-major), `field_ut: float` (expected Earth-field magnitude after correction, µT).
  - `MagCalibration.apply(self, raw_ut: tuple[float, float, float] | np.ndarray) -> np.ndarray` — returns calibrated 3-vector `matrix @ (raw - offset)`.
  - `MagCalibration.save(self, path: str | Path) -> None` and `MagCalibration.load(path: str | Path) -> MagCalibration | None` (missing/unreadable/invalid ⇒ `None`, never raises).

- [ ] **Step 1: Write the failing test**

```python
# host/tests/test_magcal.py
import numpy as np
import pytest

from roomscan.magcal import MagCalibration


def test_apply_offset_and_matrix():
    cal = MagCalibration(offset=(1.0, 2.0, 3.0),
                         matrix=((2.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 0.5)),
                         field_ut=50.0)
    out = cal.apply((4.0, 4.0, 5.0))
    assert np.allclose(out, [2.0 * 3.0, 1.0 * 2.0, 0.5 * 2.0])  # matrix @ (raw - offset)


def test_save_load_roundtrip(tmp_path):
    cal = MagCalibration(offset=(0.1, -0.2, 0.3),
                         matrix=((1.0, 0.01, 0.0), (0.01, 1.0, 0.0), (0.0, 0.0, 1.0)),
                         field_ut=48.5)
    p = tmp_path / "mag_cal.json"
    cal.save(p)
    back = MagCalibration.load(p)
    assert back is not None
    assert back.offset == pytest.approx(cal.offset)
    assert np.allclose(back.matrix, cal.matrix)
    assert back.field_ut == pytest.approx(cal.field_ut)


def test_load_missing_returns_none(tmp_path):
    assert MagCalibration.load(tmp_path / "nope.json") is None


def test_load_corrupt_returns_none(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text("{not json", encoding="utf-8")
    assert MagCalibration.load(p) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd host && python -m pytest tests/test_magcal.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'roomscan.magcal'`

- [ ] **Step 3: Write minimal implementation**

```python
# host/src/roomscan/magcal.py
"""Hard/soft-iron magnetometer calibration for the LIS2MDL (stream 10 mag).

calibrated = matrix @ (raw - offset), where `offset` removes hard-iron bias and
`matrix` removes soft-iron scale/skew so calibrated samples lie on a sphere of
radius `field_ut`. Fit from a cloud of raw samples collected while rotating the
rig through all orientations (see `fit_ellipsoid`)."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class MagCalibration:
    offset: tuple[float, float, float]
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    field_ut: float

    def apply(self, raw_ut) -> np.ndarray:
        raw = np.asarray(raw_ut, dtype=np.float64)
        m = np.asarray(self.matrix, dtype=np.float64)
        b = np.asarray(self.offset, dtype=np.float64)
        return m @ (raw - b)

    def save(self, path) -> None:
        Path(path).write_text(json.dumps({
            "offset": list(self.offset),
            "matrix": [list(row) for row in self.matrix],
            "field_ut": self.field_ut,
        }), encoding="utf-8")

    @classmethod
    def load(cls, path) -> "MagCalibration | None":
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            offset = tuple(float(v) for v in data["offset"])
            matrix = tuple(tuple(float(v) for v in row) for row in data["matrix"])
            field_ut = float(data["field_ut"])
            if len(offset) != 3 or len(matrix) != 3 or any(len(r) != 3 for r in matrix):
                return None
            return cls(offset=offset, matrix=matrix, field_ut=field_ut)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            return None
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd host && python -m pytest tests/test_magcal.py -v`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/magcal.py host/tests/test_magcal.py
git commit -m "feat(host): MagCalibration model + JSON persistence"
```

---

### Task 2: Ellipsoid fit (`magcal.fit_ellipsoid`)

**Files:**
- Modify: `host/src/roomscan/magcal.py`
- Test: `host/tests/test_magcal.py`

**Interfaces:**
- Consumes: `MagCalibration` (Task 1).
- Produces: `fit_ellipsoid(samples: np.ndarray) -> MagCalibration` where `samples` is `(N, 3)`. Raises `ValueError` if `N < 20` or the fit is degenerate (non-positive-definite shape matrix). The returned calibration maps the input cloud onto a sphere of radius `field_ut` (geometric mean of the fitted semi-axes).

- [ ] **Step 1: Write the failing test**

```python
# add to host/tests/test_magcal.py
from roomscan.magcal import fit_ellipsoid


def _distorted_sphere(n=500, radius=45.0, offset=(5.0, -3.0, 2.0),
                      soft=((1.3, 0.1, 0.0), (0.1, 0.9, 0.05), (0.0, 0.05, 1.1)), seed=0):
    rng = np.random.default_rng(seed)
    dirs = rng.normal(size=(n, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    clean = dirs * radius                      # perfect sphere
    A = np.asarray(soft)
    raw = clean @ A.T + np.asarray(offset)     # apply soft-iron then hard-iron
    return raw, np.asarray(offset)


def test_fit_recovers_center_and_spherizes():
    raw, offset = _distorted_sphere()
    cal = fit_ellipsoid(raw)
    # center (hard-iron) recovered
    assert np.allclose(cal.offset, offset, atol=1.0)
    # calibrated samples have near-constant magnitude ~ field_ut
    norms = np.linalg.norm(np.array([cal.apply(r) for r in raw]), axis=1)
    assert np.std(norms) / np.mean(norms) < 0.02
    assert np.mean(norms) == pytest.approx(cal.field_ut, rel=0.05)


def test_fit_too_few_points_raises():
    with pytest.raises(ValueError):
        fit_ellipsoid(np.zeros((5, 3)))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd host && python -m pytest tests/test_magcal.py -k fit -v`
Expected: FAIL with `ImportError: cannot import name 'fit_ellipsoid'`

- [ ] **Step 3: Write minimal implementation**

```python
# add to host/src/roomscan/magcal.py

def fit_ellipsoid(samples: np.ndarray) -> MagCalibration:
    """Least-squares fit of a 3-D point cloud to an ellipsoid, returned as a
    MagCalibration that maps the cloud onto a sphere.

    Fits a*x^2 + b*y^2 + c*z^2 + 2f*yz + 2g*xz + 2h*xy + 2p*x + 2q*y + 2r*z = 1,
    recovers the center (hard-iron) and a symmetric shape matrix, then forms the
    soft-iron correction S = field * sqrtm(Q_n) so that S @ (raw - center) lies on
    a sphere of radius `field` = geometric mean of the ellipsoid semi-axes."""
    X = np.asarray(samples, dtype=np.float64)
    if X.ndim != 2 or X.shape[1] != 3 or X.shape[0] < 20:
        raise ValueError(f"need an (N>=20, 3) sample array, got {X.shape}")
    x, y, z = X[:, 0], X[:, 1], X[:, 2]
    D = np.column_stack([x * x, y * y, z * z, 2 * y * z, 2 * x * z, 2 * x * y,
                         2 * x, 2 * y, 2 * z])
    v, *_ = np.linalg.lstsq(D, np.ones(X.shape[0]), rcond=None)
    a, b, c, f, g, h, p, q, r = v
    Q = np.array([[a, h, g], [h, b, f], [g, f, c]])
    u = np.array([p, q, r])
    try:
        center = -np.linalg.solve(Q, u)
    except np.linalg.LinAlgError as exc:
        raise ValueError("degenerate ellipsoid fit (singular shape matrix)") from exc
    d = 1.0 + center @ Q @ center
    if d <= 0:
        raise ValueError("degenerate ellipsoid fit (non-positive scale)")
    Q_n = Q / d
    evals, evecs = np.linalg.eigh(Q_n)
    if np.any(evals <= 0):
        raise ValueError("degenerate ellipsoid fit (non-positive-definite)")
    semi_axes = 1.0 / np.sqrt(evals)
    field = float(np.prod(semi_axes) ** (1.0 / 3.0))
    sqrt_Qn = evecs @ np.diag(np.sqrt(evals)) @ evecs.T
    S = field * sqrt_Qn
    return MagCalibration(
        offset=tuple(float(v) for v in center),
        matrix=tuple(tuple(float(v) for v in row) for row in S),
        field_ut=field,
    )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd host && python -m pytest tests/test_magcal.py -v`
Expected: PASS (6 tests)

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/magcal.py host/tests/test_magcal.py
git commit -m "feat(host): ellipsoid fit for hard/soft-iron mag calibration"
```

---

### Task 3: Quaternion yaw helpers (`sensors.py`)

**Files:**
- Modify: `host/src/roomscan/sensors.py`
- Test: `host/tests/test_sensors.py`

**Interfaces:**
- Consumes: `quat_to_matrix` (existing).
- Produces (module-level functions in `sensors.py`):
  - `quat_mul(a, b) -> tuple[float, float, float, float]` — Hamilton product, `[w,x,y,z]`.
  - `quat_yaw_deg(quat) -> float` — ZYX yaw in degrees, `[-180, 180)`.
  - `quat_pitch_deg(quat) -> float` — pitch in degrees, `[-90, 90]` (for the gimbal gate).
  - `graft_yaw(quat, delta_deg) -> tuple[float, float, float, float]` — returns `Rz(delta) ⊗ quat` (world-frame Z rotation, pre-multiply), normalized. Changes heading only; tilt preserved.
  - `wrap180(deg) -> float` — wrap to `[-180, 180)`.

- [ ] **Step 1: Write the failing test**

```python
# add to host/tests/test_sensors.py
from roomscan.sensors import quat_mul, quat_yaw_deg, quat_pitch_deg, graft_yaw, wrap180


def test_wrap180():
    assert wrap180(190.0) == pytest.approx(-170.0)
    assert wrap180(-190.0) == pytest.approx(170.0)
    assert wrap180(30.0) == pytest.approx(30.0)


def test_quat_yaw_of_z_rotation():
    s = np.sqrt(0.5)  # 90 deg about +Z
    assert quat_yaw_deg((s, 0.0, 0.0, s)) == pytest.approx(90.0, abs=1e-4)


def test_graft_yaw_adds_heading_preserves_tilt():
    # 30 deg pitch about +Y, no yaw
    import math
    a = math.radians(30.0) / 2
    q = (math.cos(a), 0.0, math.sin(a), 0.0)
    grafted = graft_yaw(q, 40.0)
    # pitch unchanged (tilt preserved), yaw increased by ~40 deg
    assert quat_pitch_deg(grafted) == pytest.approx(quat_pitch_deg(q), abs=0.5)
    assert wrap180(quat_yaw_deg(grafted) - quat_yaw_deg(q)) == pytest.approx(40.0, abs=0.5)


def test_graft_yaw_zero_is_noop():
    q = (0.9238795, 0.0, 0.0, 0.3826834)  # 45 deg about Z
    g = graft_yaw(q, 0.0)
    assert np.allclose(g, q, atol=1e-6)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd host && python -m pytest tests/test_sensors.py -k "wrap180 or graft or quat_yaw or quat_pitch" -v`
Expected: FAIL with `ImportError: cannot import name 'quat_mul'`

- [ ] **Step 3: Write minimal implementation**

```python
# add to host/src/roomscan/sensors.py (after quat_to_matrix)
import math


def quat_mul(a, b) -> tuple[float, float, float, float]:
    """Hamilton product a ⊗ b for [w,x,y,z] quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def wrap180(deg: float) -> float:
    """Wrap an angle in degrees to [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0


def quat_yaw_deg(quat) -> float:
    """ZYX yaw (heading) of a [w,x,y,z] quaternion, in degrees, [-180, 180)."""
    w, x, y, z = quat
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def quat_pitch_deg(quat) -> float:
    """ZYX pitch of a [w,x,y,z] quaternion, in degrees, clamped to [-90, 90]."""
    w, x, y, z = quat
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.degrees(math.asin(s))


def graft_yaw(quat, delta_deg: float) -> tuple[float, float, float, float]:
    """Rotate `quat` about the WORLD +Z axis by `delta_deg` (pre-multiply). This
    changes only heading; roll/pitch (tilt) are preserved. Returns a unit quat."""
    a = math.radians(delta_deg) / 2.0
    qz = (math.cos(a), 0.0, 0.0, math.sin(a))
    w, x, y, z = quat_mul(qz, quat)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd host && python -m pytest tests/test_sensors.py -v`
Expected: PASS (all existing + 4 new)

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/sensors.py host/tests/test_sensors.py
git commit -m "feat(host): quaternion yaw/pitch + world-Z yaw-graft helpers"
```

---

### Task 4: `YawFusion` filter with validity gates (`sensors.py`)

**Files:**
- Modify: `host/src/roomscan/sensors.py`
- Test: `host/tests/test_yaw_fusion.py` (new)

**Interfaces:**
- Consumes: `quat_to_matrix`, `tilt_compensated_heading`, `quat_yaw_deg`, `quat_pitch_deg`, `graft_yaw`, `wrap180` (Task 3); `MagCalibration` (Tasks 1-2).
- Produces:
  - `AXIS_CONVENTION: np.ndarray` — module-level 3×3, default `np.eye(3)`, applied to the calibrated mag before the heading math. This is the on-target-resolved mag-mounting-vs-IMU sign/permutation (default identity; see the on-target procedure note below).
  - `class YawFusion` with:
    - `__init__(self, tau_s: float = 20.0, calibration: MagCalibration | None = None, anomaly_frac: float = 0.3, motion_rate_dps: float = 40.0, gimbal_margin_deg: float = 15.0)`
    - `update(self, quat, raw_mag, t_us: int) -> None`
    - `fused_quat(self) -> tuple[float, float, float, float] | None` — `graft_yaw(last_quat, delta)`, or `None` before the first `update`.
    - `status: str` — one of `"init"`, `"active"`, `"gated:anomaly"`, `"gated:motion"`, `"gated:gimbal"`, `"gated:no-cal"`.

Notes for the implementer:
- The filter keeps a running scalar `delta` (the yaw correction). Fused yaw = `quat_yaw_deg(quat) + delta`. On the first valid, gated-OK sample it **snaps**: `delta = wrap180(mag_heading - quat_yaw_deg(quat))`. Thereafter it low-passes: `gain = dt / (tau_s + dt)`, `err = wrap180(mag_heading - (quat_yaw_deg(quat) + delta))`, `delta += gain * err`.
- When a gate fails, `delta` is **held** (not reset) so fused yaw still tracks SFLP's yaw changes — the bridge between good mag samples.
- `dt` comes from consecutive `t_us` (microseconds → seconds). First call has no dt: store state, set `status="init"`, no correction.
- If `calibration is None`: `status="gated:no-cal"`, still track `last_quat` so `fused_quat()` returns the raw quat (delta stays 0).
- The mag heading reuses `tilt_compensated_heading(quat, cal_mag_after_axis_convention)`.

**On-target axis-convention procedure (documentation, executed once on hardware):** with the rig level and pointed at a known magnetic heading, compare `quat_yaw_deg(quat)` drift-corrected output against the known heading; if the fused heading rotates the wrong way or is mirrored, set `AXIS_CONVENTION` to the permutation/sign matrix that fixes it. Default `np.eye(3)` is correct if the LIS2MDL axes align with the LSM body frame.

- [ ] **Step 1: Write the failing test**

```python
# host/tests/test_yaw_fusion.py
import math

import numpy as np
import pytest

from roomscan.magcal import MagCalibration
from roomscan.sensors import YawFusion, quat_yaw_deg, quat_pitch_deg, wrap180

IDENT_CAL = MagCalibration(offset=(0.0, 0.0, 0.0),
                           matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                           field_ut=50.0)
LEVEL = (1.0, 0.0, 0.0, 0.0)


def _mag_for_heading(deg):
    # level device: world == body; mag in horizontal plane pointing so that
    # tilt_compensated_heading returns `deg` (atan2(my, mx)).
    r = math.radians(deg)
    return (50.0 * math.cos(r), 50.0 * math.sin(r), 0.0)


def test_converges_to_mag_heading_when_static():
    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL)
    mag = _mag_for_heading(30.0)
    t = 0
    for _ in range(200):          # ~2 s at 100 Hz
        t += 10_000               # 10 ms
        f.update(LEVEL, mag, t)
    fused = f.fused_quat()
    assert f.status == "active"
    assert wrap180(quat_yaw_deg(fused) - 30.0) == pytest.approx(0.0, abs=1.0)


def test_snaps_on_first_valid_sample():
    f = YawFusion(tau_s=100.0, calibration=IDENT_CAL)
    f.update(LEVEL, _mag_for_heading(80.0), 10_000)   # first: init, no dt
    f.update(LEVEL, _mag_for_heading(80.0), 20_000)   # second: snaps despite huge tau
    assert wrap180(quat_yaw_deg(f.fused_quat()) - 80.0) == pytest.approx(0.0, abs=1.0)


def test_gate_anomaly_holds_delta():
    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL, anomaly_frac=0.3)
    f.update(LEVEL, _mag_for_heading(0.0), 10_000)
    f.update(LEVEL, _mag_for_heading(0.0), 20_000)    # establish delta ~0
    strong = tuple(3.0 * c for c in _mag_for_heading(90.0))  # |mag| far from field
    f.update(LEVEL, strong, 30_000)
    assert f.status == "gated:anomaly"
    assert wrap180(quat_yaw_deg(f.fused_quat()) - 0.0) == pytest.approx(0.0, abs=1.0)


def test_gate_motion_holds_delta():
    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL, motion_rate_dps=40.0)
    f.update(LEVEL, _mag_for_heading(0.0), 0)
    f.update(LEVEL, _mag_for_heading(0.0), 1_000_000)   # settle at 0
    # now a big orientation jump over a tiny dt => high angular rate
    s = math.sqrt(0.5)
    fast = (s, 0.0, 0.0, s)   # 90 deg in 1 ms
    f.update(fast, _mag_for_heading(90.0), 1_001_000)
    assert f.status == "gated:motion"


def test_gate_gimbal_holds_delta():
    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL, gimbal_margin_deg=15.0)
    a = math.radians(85.0) / 2   # pitch 85 deg -> within 15 of 90
    steep = (math.cos(a), 0.0, math.sin(a), 0.0)
    f.update(steep, _mag_for_heading(0.0), 10_000)
    f.update(steep, _mag_for_heading(0.0), 20_000)
    assert f.status == "gated:gimbal"


def test_no_calibration_returns_raw():
    f = YawFusion(tau_s=1.0, calibration=None)
    f.update(LEVEL, (1.0, 0.0, 0.0), 10_000)
    assert f.status == "gated:no-cal"
    assert f.fused_quat() == pytest.approx(LEVEL)


def test_tilt_preserved():
    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL)
    a = math.radians(20.0) / 2
    tilted = (math.cos(a), math.sin(a), 0.0, 0.0)   # 20 deg roll
    t = 0
    for _ in range(100):
        t += 10_000
        f.update(tilted, _mag_for_heading(45.0), t)
    assert quat_pitch_deg(f.fused_quat()) == pytest.approx(quat_pitch_deg(tilted), abs=0.5)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd host && python -m pytest tests/test_yaw_fusion.py -v`
Expected: FAIL with `ImportError: cannot import name 'YawFusion'`

- [ ] **Step 3: Write minimal implementation**

```python
# add to host/src/roomscan/sensors.py
from .magcal import MagCalibration

AXIS_CONVENTION = np.eye(3)   # mag-mounting-vs-IMU sign/permutation; resolved on-target


class YawFusion:
    """Stateful yaw-only complementary filter: grafts a gated, low-passed
    tilt-compensated magnetometer heading onto the SFLP quaternion. Tilt is
    taken from SFLP unchanged; only heading is corrected."""

    def __init__(self, tau_s: float = 20.0, calibration: MagCalibration | None = None,
                 anomaly_frac: float = 0.3, motion_rate_dps: float = 40.0,
                 gimbal_margin_deg: float = 15.0):
        self.tau_s = float(tau_s)
        self.cal = calibration
        self.anomaly_frac = float(anomaly_frac)
        self.motion_rate_dps = float(motion_rate_dps)
        self.gimbal_margin_deg = float(gimbal_margin_deg)
        self._delta = 0.0
        self._have_delta = False
        self._last_quat: tuple[float, float, float, float] | None = None
        self._last_t: int | None = None
        self.status = "init"

    def update(self, quat, raw_mag, t_us: int) -> None:
        quat = tuple(float(v) for v in quat)
        prev_quat, prev_t = self._last_quat, self._last_t
        self._last_quat = quat
        if self.cal is None:
            self.status = "gated:no-cal"
            self._last_t = t_us
            return
        if prev_quat is None or prev_t is None:
            self.status = "init"
            self._last_t = t_us
            return
        dt = (t_us - prev_t) / 1e6
        if dt <= 0:
            dt = 1e-3
        # gate: gimbal lock
        if abs(quat_pitch_deg(quat)) > 90.0 - self.gimbal_margin_deg:
            self.status = "gated:gimbal"
            self._last_t = t_us
            return
        # gate: fast motion (SFLP quat angular rate as accel-free motion proxy)
        dot = sum(a * b for a, b in zip(prev_quat, quat))
        ang = 2.0 * math.acos(max(0.0, min(1.0, abs(dot))))   # rad between orientations
        if math.degrees(ang) / dt > self.motion_rate_dps:
            self.status = "gated:motion"
            self._last_t = t_us
            return
        # calibrate + axis-convention the mag, then anomaly gate on magnitude
        cal_mag = AXIS_CONVENTION @ self.cal.apply(raw_mag)
        mag_norm = float(np.linalg.norm(cal_mag))
        if abs(mag_norm - self.cal.field_ut) > self.anomaly_frac * self.cal.field_ut:
            self.status = "gated:anomaly"
            self._last_t = t_us
            return
        heading = tilt_compensated_heading(quat, tuple(cal_mag))
        yaw = quat_yaw_deg(quat)
        if not self._have_delta:
            self._delta = wrap180(heading - yaw)   # snap on first valid sample
            self._have_delta = True
        else:
            gain = dt / (self.tau_s + dt)
            self._delta += gain * wrap180(heading - (yaw + self._delta))
        self.status = "active"
        self._last_t = t_us

    def fused_quat(self):
        if self._last_quat is None:
            return None
        return graft_yaw(self._last_quat, self._delta)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd host && python -m pytest tests/test_yaw_fusion.py -v`
Expected: PASS (8 tests)

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/sensors.py host/tests/test_yaw_fusion.py
git commit -m "feat(host): YawFusion complementary filter with anomaly/motion/gimbal gates"
```

---

### Task 5: `SensorState` fusion integration (`sensors.py`)

**Files:**
- Modify: `host/src/roomscan/sensors.py`
- Test: `host/tests/test_sensors.py`

**Interfaces:**
- Consumes: `YawFusion` (Task 4).
- Produces (on `SensorState`):
  - `__init__(self, history: int = 256, fusion: YawFusion | None = None)` (new optional param; existing callers unaffected).
  - `fused_quat(self) -> tuple[float, float, float, float] | None` — fused quat if fusion is active and produced one, else the raw `latest_quat()`.
  - `fusion_status(self) -> str` — `fusion.status` or `"off"` when no fusion.
- Behavior: on an `IMU_QUAT` frame, if `fusion` is set and a mag sample has been seen, call `fusion.update(quat, latest_raw_mag, frame.header.t_us)` under the lock. On an `ENV` frame, cache the raw mag vector for the next quat update.

- [ ] **Step 1: Write the failing test**

```python
# add to host/tests/test_sensors.py
def test_fused_quat_falls_back_to_raw_without_fusion():
    st = SensorState()
    st.feed(_frame(StreamId.IMU_QUAT, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    assert st.fused_quat() == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert st.fusion_status() == "off"


def test_fused_quat_applies_yaw_correction():
    import math
    from roomscan.magcal import MagCalibration
    from roomscan.sensors import YawFusion, quat_yaw_deg, wrap180
    cal = MagCalibration(offset=(0.0, 0.0, 0.0),
                         matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                         field_ut=50.0)
    st = SensorState(fusion=YawFusion(tau_s=0.5, calibration=cal))
    mag = (50.0 * math.cos(math.radians(60.0)), 50.0 * math.sin(math.radians(60.0)), 0.0)
    for i in range(300):
        st.feed(_frame(StreamId.ENV, struct.pack("<5f", 101325.0, *mag, 20.0)))
        h = FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, 1, (i + 1) * 10_000, 0, 0, 16)
        st.feed(Frame(h, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    assert st.fusion_status() == "active"
    assert wrap180(quat_yaw_deg(st.fused_quat()) - 60.0) == pytest.approx(0.0, abs=1.5)
```

Note: `_frame` builds headers with `t_us=123` (constant); the second test builds its own `FrameHeader` with increasing `t_us` for the quat frames so the filter sees real `dt`.

- [ ] **Step 2: Run test to verify it fails**

Run: `cd host && python -m pytest tests/test_sensors.py -k fused -v`
Expected: FAIL with `AttributeError: 'SensorState' object has no attribute 'fused_quat'`

- [ ] **Step 3: Write minimal implementation**

```python
# modify SensorState in host/src/roomscan/sensors.py
    def __init__(self, history: int = 256, fusion: "YawFusion | None" = None):
        self._lock = threading.Lock()
        self._quat: tuple[float, float, float, float] | None = None
        self._env: EnvSample | None = None
        self._pressure = deque(maxlen=history)
        self._temp = deque(maxlen=history)
        self._fusion = fusion
        self._raw_mag: tuple[float, float, float] | None = None

    def feed(self, frame: Frame) -> None:
        if frame.header.frame_type != FrameType.DATA:
            return
        sid = frame.header.stream_id
        if sid == StreamId.IMU_QUAT:
            q = decode_imu_quat(frame.payload)
            with self._lock:
                self._quat = q
                if self._fusion is not None and self._raw_mag is not None:
                    self._fusion.update(q, self._raw_mag, frame.header.t_us)
        elif sid == StreamId.ENV:
            pressure, mag, temp = decode_env(frame.payload)
            sample = EnvSample(pressure, mag, temp, frame.header.t_us)
            with self._lock:
                self._env = sample
                self._raw_mag = mag
                self._pressure.append(pressure)
                self._temp.append(temp)

    def fused_quat(self):
        with self._lock:
            if self._fusion is not None:
                fused = self._fusion.fused_quat()
                if fused is not None:
                    return fused
            return self._quat

    def fusion_status(self) -> str:
        with self._lock:
            return self._fusion.status if self._fusion is not None else "off"
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd host && python -m pytest tests/test_sensors.py -v`
Expected: PASS (all existing + 2 new)

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/sensors.py host/tests/test_sensors.py
git commit -m "feat(host): wire YawFusion into SensorState (fused_quat/fusion_status)"
```

---

### Task 6: Config fields (`config.py`)

**Files:**
- Modify: `host/src/roomscan/config.py:38-57` (add fields to `ViewerConfig`)
- Test: `host/tests/test_config.py`

**Interfaces:**
- Produces: new `ViewerConfig` fields (load/save are already generic over `fields(cls)`, so no method changes):
  - `yaw_fusion: bool = True`
  - `yaw_fusion_tau: float = 20.0`
  - `mag_cal_path: str = "mag_cal.json"`
  - `yaw_anomaly_frac: float = 0.3`
  - `yaw_motion_rate_dps: float = 40.0`
  - `yaw_gimbal_margin_deg: float = 15.0`

- [ ] **Step 1: Write the failing test**

```python
# add to host/tests/test_config.py
def test_yaw_fusion_config_defaults():
    from roomscan.config import ViewerConfig
    c = ViewerConfig()
    assert c.yaw_fusion is True
    assert c.yaw_fusion_tau == 20.0
    assert c.mag_cal_path == "mag_cal.json"
    assert c.yaw_anomaly_frac == 0.3
    assert c.yaw_motion_rate_dps == 40.0
    assert c.yaw_gimbal_margin_deg == 15.0


def test_yaw_fusion_config_roundtrip(tmp_path):
    from roomscan.config import ViewerConfig
    p = tmp_path / "cfg.toml"
    ViewerConfig(yaw_fusion=False, yaw_fusion_tau=12.5, mag_cal_path="x.json").save(p)
    back = ViewerConfig.load(p)
    assert back.yaw_fusion is False
    assert back.yaw_fusion_tau == 12.5
    assert back.mag_cal_path == "x.json"
```

If `host/tests/test_config.py` does not exist, create it with the two functions above and the standard imports (`from roomscan.config import ViewerConfig`).

- [ ] **Step 2: Run test to verify it fails**

Run: `cd host && python -m pytest tests/test_config.py -k yaw_fusion -v`
Expected: FAIL with `AttributeError: 'ViewerConfig' object has no attribute 'yaw_fusion'`

- [ ] **Step 3: Write minimal implementation**

```python
# add to the ViewerConfig dataclass body in host/src/roomscan/config.py (after gizmo_scale)
    yaw_fusion: bool = True                 # graft mag heading onto SFLP yaw
    yaw_fusion_tau: float = 20.0            # complementary-filter time constant (s)
    mag_cal_path: str = "mag_cal.json"     # hard/soft-iron calibration JSON
    yaw_anomaly_frac: float = 0.3          # |mag| deviation from field to reject
    yaw_motion_rate_dps: float = 40.0      # quat angular rate above which to freeze
    yaw_gimbal_margin_deg: float = 15.0    # freeze within this of |pitch|=90
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd host && python -m pytest tests/test_config.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/config.py host/tests/test_config.py
git commit -m "feat(host): yaw-fusion config fields (toggle, tau, cal path, gate thresholds)"
```

---

### Task 7: Panel wiring — gizmo + compass use fused orientation (`panel.py`)

**Files:**
- Modify: `host/src/roomscan/panel.py` (init: build `YawFusion` from config, pass to `SensorState`; `_update_sensors`: use `fused_quat()` + corrected heading + status)
- Test: `host/tests/test_panel_sensors.py`

**Interfaces:**
- Consumes: `SensorState(fusion=...)`, `YawFusion`, `MagCalibration`, `fused_quat`, `fusion_status`, `tilt_compensated_heading` (existing), config fields (Task 6).

Implementation details:
- In the panel `__init__`, after reading the gizmo/sensors flags (near `panel.py:262-264`), build the fusion:

```python
        self.yaw_fusion = bool(getattr(args, "yaw_fusion", True))
        fusion = None
        if self.yaw_fusion:
            from .magcal import MagCalibration
            from .sensors import YawFusion
            cal = MagCalibration.load(getattr(args, "mag_cal_path", "mag_cal.json"))
            fusion = YawFusion(
                tau_s=float(getattr(args, "yaw_fusion_tau", 20.0) or 20.0),
                calibration=cal,
                anomaly_frac=float(getattr(args, "yaw_anomaly_frac", 0.3) or 0.3),
                motion_rate_dps=float(getattr(args, "yaw_motion_rate_dps", 40.0) or 40.0),
                gimbal_margin_deg=float(getattr(args, "yaw_gimbal_margin_deg", 15.0) or 15.0),
            )
        self.sensor_state = SensorState(fusion=fusion)
```

  (Replace the existing bare `self.sensor_state = SensorState()` at `panel.py:261`.)
- In `_update_sensors` (`panel.py:847-868`), replace `quat = self.sensor_state.latest_quat()` with `quat = self.sensor_state.fused_quat()`, and compute the compass heading from the fused quat + calibrated mag. Keep the existing raw-heading fallback when no calibration is loaded (so the compass still works uncalibrated). Concretely, the gizmo/compass now consume the corrected orientation; publish the fusion status to the log bus when it changes:

```python
        status = self.sensor_state.fusion_status()
        if status != getattr(self, "_last_fusion_status", None):
            self._last_fusion_status = status
            self.bus.publish(f"yaw-fusion -> {status}")
```

- [ ] **Step 1: Write the failing test**

```python
# add to host/tests/test_panel_sensors.py
def test_fused_quat_seam_uses_correction():
    import math
    from roomscan.magcal import MagCalibration
    from roomscan.sensors import SensorState, YawFusion, gizmo_pose, quat_yaw_deg, wrap180
    cal = MagCalibration(offset=(0.0, 0.0, 0.0),
                         matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                         field_ut=50.0)
    st = SensorState(fusion=YawFusion(tau_s=0.5, calibration=cal))
    mag = (50.0 * math.cos(math.radians(60.0)), 50.0 * math.sin(math.radians(60.0)), 0.0)
    for i in range(300):
        st.feed(_env_frame(101325.0, mag, 20.0))
        h = FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, 1, (i + 1) * 10_000, 0, 0, 16)
        st.feed(Frame(h, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    quat = st.fused_quat()                    # what the tick now draws
    assert wrap180(quat_yaw_deg(quat) - 60.0) == pytest.approx(0.0, abs=1.5)
    pose = gizmo_pose(quat, 0.15, (0.0, 0.0, 0.0))
    assert pose.shape == (4, 4)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd host && python -m pytest tests/test_panel_sensors.py -k fused -v`
Expected: FAIL (import of `YawFusion`/`fused_quat` seam not yet exercised — or, if Tasks 4-5 are already merged, this passes at the seam level; then proceed to wire the panel and re-run the full file).

Note: this test validates the seam (`SensorState.fused_quat()` → `gizmo_pose`) that the panel tick consumes; the panel edits themselves are GUI code verified by `/verify` on hardware, consistent with how `test_panel_sensors.py` already simulates the reader→UI seam without a GUI.

- [ ] **Step 3: Write minimal implementation**

Apply the `__init__` and `_update_sensors` edits described in the Interfaces block above.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd host && python -m pytest tests/test_panel_sensors.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/panel.py host/tests/test_panel_sensors.py
git commit -m "feat(host): panel gizmo + compass consume fused (yaw-corrected) orientation"
```

---

### Task 8: Magnetometer calibration CLI (`host/tools/mag_calibrate.py`)

**Files:**
- Create: `host/tools/mag_calibrate.py`
- Test: `host/tests/test_mag_calibrate.py`

**Interfaces:**
- Consumes: `fit_ellipsoid`, `MagCalibration` (Tasks 1-2); `SerialSource` + `StreamDecoder` (existing, `sources.py` / `decoder.py`); `decode_env`, `StreamId`, `FrameType` (existing `protocol.py`).
- Produces:
  - `collect_mag_from_frames(frames: Iterable[Frame]) -> np.ndarray` — pull mag vectors from ENV data frames into an `(N, 3)` array (pure, testable).
  - `calibrate(samples: np.ndarray, out_path: str | Path) -> MagCalibration` — fit, save, return.
  - `main(argv=None) -> int` — thin CLI: open `SerialSource`, pump for `--seconds` while the user rotates the rig, collect, fit, save to `--out` (default from `ViewerConfig().mag_cal_path`), print fit residual (std/mean of calibrated magnitudes). Deferred-imports `SerialSource`.

- [ ] **Step 1: Write the failing test**

```python
# host/tests/test_mag_calibrate.py
import struct

import numpy as np
import pytest

from roomscan.protocol import Frame, FrameHeader, FrameType, StreamId
from tools.mag_calibrate import collect_mag_from_frames, calibrate


def _env_frame(mag):
    payload = struct.pack("<5f", 101325.0, *mag, 20.0)
    return Frame(FrameHeader(FrameType.DATA, StreamId.ENV, 0, 1, 0, 0, 0, len(payload)), payload)


def test_collect_pulls_mag_vectors():
    frames = [_env_frame((1.0, 2.0, 3.0)), _env_frame((4.0, 5.0, 6.0))]
    out = collect_mag_from_frames(frames)
    assert out.shape == (2, 3)
    assert np.allclose(out[1], [4.0, 5.0, 6.0])


def test_calibrate_writes_json_and_spherizes(tmp_path):
    rng = np.random.default_rng(1)
    dirs = rng.normal(size=(400, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    raw = dirs * 45.0 @ np.array([[1.2, 0.0, 0.0], [0.0, 0.9, 0.0], [0.0, 0.0, 1.05]]).T + [4.0, -2.0, 1.0]
    out = tmp_path / "mag_cal.json"
    cal = calibrate(raw, out)
    assert out.exists()
    norms = np.linalg.norm(np.array([cal.apply(r) for r in raw]), axis=1)
    assert np.std(norms) / np.mean(norms) < 0.02
```

The test imports `from tools.mag_calibrate import ...`. Ensure the test can resolve `tools` — add `host/tools/__init__.py` (empty) and confirm `host/` is on the path via the existing pytest config (tests already import `roomscan` from `host/src`; if `tools` is not importable, add `pythonpath = ["src", "."]` under `[tool.pytest.ini_options]` in `host/pyproject.toml`, or place the test's `sys.path` insert — prefer the pyproject route).

- [ ] **Step 2: Run test to verify it fails**

Run: `cd host && python -m pytest tests/test_mag_calibrate.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tools.mag_calibrate'`

- [ ] **Step 3: Write minimal implementation**

```python
# host/tools/mag_calibrate.py
"""Interactive magnetometer calibration: collect ENV-stream mag samples while
rotating the rig through all orientations, fit hard/soft-iron correction, save.

Usage:  cd host && python -m tools.mag_calibrate --seconds 30 --out mag_cal.json
Rotate the rig slowly through as many orientations as possible during the window."""
from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Iterable

import numpy as np

from roomscan.decoder import StreamDecoder
from roomscan.magcal import MagCalibration, fit_ellipsoid
from roomscan.protocol import Frame, FrameType, StreamId, decode_env


def collect_mag_from_frames(frames: Iterable[Frame]) -> np.ndarray:
    out = []
    for fr in frames:
        if fr.header.frame_type == FrameType.DATA and fr.header.stream_id == StreamId.ENV:
            _, mag, _ = decode_env(fr.payload)
            out.append(mag)
    return np.asarray(out, dtype=np.float64).reshape(-1, 3)


def calibrate(samples: np.ndarray, out_path) -> MagCalibration:
    cal = fit_ellipsoid(np.asarray(samples, dtype=np.float64))
    cal.save(out_path)
    return cal


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Magnetometer hard/soft-iron calibration")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--out", default="mag_cal.json")
    ap.add_argument("--port", default=None)
    args = ap.parse_args(argv)

    from roomscan.sources import SerialSource  # deferred: no pyserial in tests
    src = SerialSource(port=args.port)
    dec = StreamDecoder()
    print(f"Rotate the rig through ALL orientations for {args.seconds:.0f} s...")
    samples: list[tuple[float, float, float]] = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        for fr in dec.feed(src.read()):
            if fr.header.frame_type == FrameType.DATA and fr.header.stream_id == StreamId.ENV:
                _, mag, _ = decode_env(fr.payload)
                samples.append(mag)
    src.close()
    arr = np.asarray(samples, dtype=np.float64).reshape(-1, 3)
    print(f"collected {arr.shape[0]} mag samples")
    cal = calibrate(arr, args.out)
    norms = np.linalg.norm(np.array([cal.apply(r) for r in arr]), axis=1)
    print(f"field_ut={cal.field_ut:.2f}  residual(std/mean)={np.std(norms)/np.mean(norms):.4f}")
    print(f"saved -> {Path(args.out).resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

Also create empty `host/tools/__init__.py` if `tools` is not already a package.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd host && python -m pytest tests/test_mag_calibrate.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add host/tools/mag_calibrate.py host/tests/test_mag_calibrate.py host/tools/__init__.py
git commit -m "feat(host): magnetometer calibration CLI (collect + ellipsoid-fit + save)"
```

---

### Task 9: Full-suite regression + docs

**Files:**
- Modify: `docs/protocol.md` (only if a stream note is warranted — none expected; no wire change) — **skip unless** a reviewer flags it.
- Modify: `host/README` or the panel help string if it enumerates config keys — add the yaw-fusion keys if such a list exists; otherwise skip.

- [ ] **Step 1: Run the full host suite**

Run: `cd host && python -m pytest -q`
Expected: PASS — all pre-existing tests (208 baseline) plus the new `test_magcal.py`, `test_yaw_fusion.py`, `test_mag_calibrate.py`, and the additions to `test_sensors.py` / `test_config.py` / `test_panel_sensors.py`.

- [ ] **Step 2: On-target verification (hardware, via `/verify` or the panel)**

Launch the panel with the LSM streaming; run `python -m tools.mag_calibrate` once and rotate the rig to produce `mag_cal.json`; restart the panel and confirm: (a) the gizmo yaw no longer drifts over a few minutes of stationary hold, (b) the compass reads a stable heading, (c) the log shows `yaw-fusion -> active` and transitions to `gated:*` under fast motion / near-vertical pitch. If the fused heading is mirrored or rotates the wrong way, set `AXIS_CONVENTION` per the Task 4 on-target procedure and re-verify.

- [ ] **Step 3: Commit any doc touch-ups**

```bash
git add -A
git commit -m "docs: note host-side yaw-fusion + mag calibration workflow"
```

---

## Self-Review

**Spec coverage:**
- magcal (hard/soft-iron, ellipsoid fit, persistence) → Tasks 1-2. ✓
- Axis-convention resolution → `AXIS_CONVENTION` + on-target procedure (Task 4) + verify step (Task 9). ✓
- `YawFusion` / `SensorState.fused_quat()` (gated yaw-only complementary blend, long τ, ±Q-safe via angle metric, tilt preserved) → Tasks 3-5. ✓
- Gates (anomaly / motion-proxy / gimbal) → Task 4. ✓
- Consumers/UI (gizmo uses fused, corrected compass, config toggle + thresholds, status line) → Tasks 6-7. ✓
- Calibration UX (CLI collector) → Task 8. ✓
- Tests (magcal recovery, convergence, tilt-unchanged, gates, degraded modes, sign) → Tasks 1-8. ✓
- Out-of-scope items (temp-bias, dead-reckoning, baro-Z, MLC/FSM/ASC, raw accel/gyro streaming) → not implemented, by design. ✓

**Placeholder scan:** No TBD/TODO; every code step carries complete code. The only deferred-to-hardware items (Task 9 on-target verify, `AXIS_CONVENTION` value) are inherently bench actions, documented with exact procedures — not code placeholders.

**Type consistency:** `MagCalibration.apply` returns `np.ndarray` (consumed by `np.linalg.norm` and `tilt_compensated_heading` via `tuple(cal_mag)`); `YawFusion.update(quat, raw_mag, t_us:int)` / `fused_quat()` names match across Tasks 4/5/7; `fusion_status()` returns the `YawFusion.status` string set in Task 4; config field names (`yaw_fusion`, `yaw_fusion_tau`, `mag_cal_path`, `yaw_anomaly_frac`, `yaw_motion_rate_dps`, `yaw_gimbal_margin_deg`) are identical in Tasks 6 and 7. ✓
