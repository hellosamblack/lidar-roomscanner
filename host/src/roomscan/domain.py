# pyright: strict
"""Domain newtypes for the quantities this codebase has historically confused.

Spec: docs/superpowers/specs/2026-08-27-rust-migration-feasibility.md,
"Compile-time correctness payoff" (experiment E1, issue #223). Every type here
is a `typing.NewType` (erased at runtime -- zero cost, zero behaviour change) or
a small frozen dataclass / enum, so the only thing they add is a *name* that
Pyright can refuse to conflate. Keep this module boring: no metaclasses, and
the ONE generic family (`Cloud[F]` / `Rotation[To, From]` at the bottom) exists
only because frame mismatch is not a scalar mistake -- an agent must still be
able to read each section in one screen.

What each family guards (the shipped bug behind it):

* Clock domains -- BUG-050 subtracted `time.time()` from a `time.monotonic()`
  stamp and reported a 90 s take as 1.78e9 s. `HostMono` and `WallTime` share
  no origin; `McuMicros` (frame header `t_us`) and `LsmTicks` (stream 13's
  uint32 LSM6DSV16X TIMESTAMP counter, ~21.7 us/LSB, wraps ~26 h) are two more.
* Lengths -- the ToF wire is millimetres, SLAM/Open3D is metres (BUG-055's
  cousin: a unitless shell scale drawn as metres).
* Angles -- `Bearing` (compass direction of a named axis, CW degrees) is NOT
  `EulerYaw` (ZYX Tait-Bryan yaw, gimbal-locks at this device's normal grip:
  BUG-039/048/051) and NOT `Twist` (swing-twist about world +Z, absorbs roll
  about a horizontal boresight: BUG-058). Only `sensors.boresight_bearing_deg`
  / `magnetic_north_bearing_deg` / `absolute_heading` mint a `Bearing`.
* `EnvSample` -- BUG-083: a zero-filled stream-10 sentinel (0.0 Pa) reached
  `baro_height_m` and fabricated 8.6 m of SLAM vertical. `from_wire` returns
  None for it, so a consumer must unwrap before it can read `pressure_pa`.
* `IcpMode` / `WireVersion` -- single-source enums so a missing case is a
  type error (`assert_never`) rather than a silent `None` (BUG-071's shape).
* Frames -- BUG-026: a body/CV-frame point cloud was drawn as if it were
  already in the render world (the display rotation was computed and then not
  applied). `Cloud[F]` tags an (N, 3) array with the frame it is expressed in
  and `Rotation[To, From]` tags a 3x3 matrix with both frames; `R @ cloud` and
  `R_a @ R_b` type-check the frames, so a `Cloud[CV]` cannot be returned where
  a `Cloud[CvWorld]` is declared and `T_WORLD_TO_CV @ T_CV_TO_BODY` cannot be
  written with the middle rotation missing. The frame names are the four in
  `docs/coordinate-frames.md`; the tags are also checked at runtime (cheap
  identity compare) so an untyped caller fails loudly instead of silently.

Python cannot forbid `Bearing - Bearing` (it is just `float - float`) -- the
arithmetic result is a plain float, which a `Bearing`-typed consumer then
rejects. Route angle differences through `bearing_delta` (BUG-082: a 359 -> 0
heading crossing read as 359 degrees of motion).
"""
from __future__ import annotations

import math
import time
from collections.abc import Sequence
from dataclasses import dataclass
from enum import IntEnum, StrEnum
from typing import Any, Generic, NewType, TypeAlias, TypeVar, overload

import numpy as np
from numpy.typing import NDArray

# --- Clock domains ------------------------------------------------------------

#: Device frame-header `t_us`: microseconds on the STM32's free-running clock.
McuMicros = NewType("McuMicros", int)
#: LSM6DSV16X TIMESTAMP register ticks (uint32, ~21.7 us/LSB, wraps ~26 h).
#: A float because stream 13's frame-ready instant is a fractional tick
#: (`lsm_ticks - latch_delay_us / tick_us`, see `protocol.ImuSync`).
LsmTicks = NewType("LsmTicks", float)
#: `time.monotonic()` seconds. Arbitrary origin; only differences mean anything.
HostMono = NewType("HostMono", float)
#: `time.time()` seconds since the Unix epoch. Never subtract from `HostMono`.
WallTime = NewType("WallTime", float)
#: `CLOCK_MONOTONIC` NANOSECONDS **on the Raspberry Pi bridge**, as carried by a
#: V4L2 buffer timestamp and relayed in `X-Roomscan-Capture-Mono-Ns` (#226).
#: A THIRD, independent monotonic origin: it shares neither epoch nor rate
#: guarantee with `HostMono` (a different SoC, a different boot) and it is not
#: `McuMicros` either. Subtracting it from anything but another `PiMonoNanos`
#: is meaningless until Stage B (#227) fits an explicit Pi->host affine
#: transform with an uncertainty bound; nothing in Stage A does that, which is
#: exactly why the type exists -- so "camera capture time" can never be quietly
#: differenced against a host arrival instant and read as a latency.
PiMonoNanos = NewType("PiMonoNanos", int)


def pi_mono_delta_s(a: PiMonoNanos, b: PiMonoNanos) -> float:
    """Seconds from `a` to `b`, BOTH on the Pi's monotonic clock.

    The only subtraction this domain permits. Used for the camera's
    capture-side cadence (`capture_fps`), which is a Pi-local rate and needs no
    cross-clock mapping to be true.
    """
    return (int(b) - int(a)) / 1e9


def now_mono() -> HostMono:
    return HostMono(time.monotonic())


def now_wall() -> WallTime:
    return WallTime(time.time())


def elapsed_s(now: HostMono, start: HostMono) -> float:
    """Seconds from `start` to `now`, both on the monotonic host clock."""
    return float(now) - float(start)


def lsm_delta(a: LsmTicks, b: LsmTicks) -> float:
    """Signed, wrap-safe tick delta a -> b (nearest modular representative)."""
    from .sensor_time import signed_tick_delta   # local: sensor_time imports us
    return signed_tick_delta(a, b)


# --- Lengths -----------------------------------------------------------------

Millimetres = NewType("Millimetres", float)
Metres = NewType("Metres", float)

#: The one place the ToF wire's mm and Open3D's m meet as a number.
MM_PER_M = 1000.0


def mm_to_m(x: Millimetres) -> Metres:
    return Metres(float(x) / MM_PER_M)


def m_to_mm(x: Metres) -> Millimetres:
    return Millimetres(float(x) * MM_PER_M)


# --- Angles (degrees) ----------------------------------------------------------

#: Compass bearing of a NAMED axis (boresight, magnetic north) in a world
#: frame, clockwise degrees. The only heading-like quantity a live path may use.
Bearing = NewType("Bearing", float)
#: ZYX Tait-Bryan yaw. Display only -- gimbal-locks at the upright grip.
EulerYaw = NewType("EulerYaw", float)
#: Swing-twist rotation about world +Z. A residual between two attitudes, or
#: the argument of `sensors.graft_yaw`; never a heading.
Twist = NewType("Twist", float)


def wrap_deg(deg: float) -> float:
    """Wrap degrees to (-180, 180]."""
    d = math.fmod(deg + 180.0, 360.0)
    if d <= 0.0:
        d += 360.0
    return d - 180.0


def bearing_delta(a: Bearing, b: Bearing) -> float:
    """Shortest signed turn from bearing `b` to bearing `a`, in (-180, 180].

    `a - b` on two bearings is a plain float that still carries the 360 wrap
    (BUG-082); this is the subtraction that knows bearings live on a circle."""
    return wrap_deg(float(a) - float(b))


# --- Validated sensor samples --------------------------------------------------

#: Physical bounds `protocol.is_valid_env` has always enforced (LPS22DF /
#: STTS22H datasheet ranges). Kept here so the sentinel check has one owner.
PRESSURE_PA_MIN = 30000.0
PRESSURE_PA_MAX = 120000.0
TEMP_C_MIN = -40.0
TEMP_C_MAX = 85.0
MAG_MIN_NORM2_UT = 1e-3


@dataclass(frozen=True)
class EnvSample:
    """One validated stream-10 sample. Construct via `from_wire` -- the
    constructor itself does no checking, so a hand-built instance is a claim."""
    pressure_pa: float
    mag_ut: tuple[float, float, float]
    temp_c: float

    @classmethod
    def from_values(cls, pressure_pa: float, mag_ut: tuple[float, float, float],
                    temp_c: float) -> EnvSample | None:
        """None for the firmware's zero-filled / out-of-range sentinel (BUG-083)."""
        if not (math.isfinite(pressure_pa) and PRESSURE_PA_MIN <= pressure_pa <= PRESSURE_PA_MAX):
            return None
        if not (math.isfinite(temp_c) and TEMP_C_MIN <= temp_c <= TEMP_C_MAX):
            return None
        mx, my, mz = (float(mag_ut[0]), float(mag_ut[1]), float(mag_ut[2]))
        if not (math.isfinite(mx) and math.isfinite(my) and math.isfinite(mz)):
            return None
        if (mx * mx + my * my + mz * mz) <= MAG_MIN_NORM2_UT:
            return None
        return cls(float(pressure_pa), (mx, my, mz), float(temp_c))

    @classmethod
    def from_wire(cls, payload: bytes) -> EnvSample | None:
        """Decode a stream-10 ENV payload; None for the zero-filled sentinel."""
        from .protocol import decode_env   # local: protocol imports us
        pressure, mag, temp = decode_env(payload)
        return cls.from_values(pressure, mag, temp)


# --- Single-source enums -------------------------------------------------------

class IcpMode(StrEnum):
    """`slam.odometry.register` modes (see that module's docstring)."""
    TRANSLATION = "translation"
    SIXDOF = "6dof"
    ADAPTIVE = "adaptive"
    SOFT_PRIOR = "soft_prior"


class WireVersion(IntEnum):
    """RSCN frame-header versions `protocol.FrameHeader.unpack` accepts."""
    V1 = 1
    V2 = 2


# --- Frames (frame-tagged clouds and rotations) --------------------------------
#
# The four frames of `docs/coordinate-frames.md`, as marker classes. They carry
# no data; they exist so `Cloud[CV]` and `Cloud[CvWorld]` are different types to
# Pyright and different objects at runtime. Never instantiate one.

class Frame:
    """Base marker for a coordinate frame. Subclass, never instantiate."""
    __slots__ = ()


class CV(Frame):
    """ToF camera frame: X = Right, Y = Down, Z = Forward (`Deprojector` output)."""
    __slots__ = ()


class Body(Frame):
    """SFLP body frame: X = Up, Y = Right, Z = Forward (board vertical, USB down)."""
    __slots__ = ()


class World(Frame):
    """SFLP world frame: X = North, Y = West, Z = Up. What stream 9 rotates into."""
    __slots__ = ()


class CvWorld(Frame):
    """Open3D CV world: X = East, Y = Down, Z = North. The renderer's / TSDF's world."""
    __slots__ = ()


class View(Frame):
    """The frame the live browser cloud is packed in for the selected view mode:
    `CvWorld` in World mode, a level boresight frame in FPV, mirrored in Mirror.
    Its axes vary with the mode, so it is its own tag rather than any of the four
    physical frames -- a consumer that wants gravity-true geometry must ask for
    `Cloud[CvWorld]`, not `Cloud[View]`."""
    __slots__ = ()


F = TypeVar("F", bound=Frame)
To = TypeVar("To", bound=Frame)
From = TypeVar("From", bound=Frame)
_X = TypeVar("_X", bound=Frame)

#: A [w, x, y, z] unit quaternion as this codebase passes it around.
Quat: TypeAlias = tuple[float, float, float, float]
#: What a quaternion-taking helper accepts: a tuple/list of 4 floats or a
#: 4-element float array. Helpers that need a real tuple call `sensors._unit_quat`.
QuatLike: TypeAlias = Sequence[float] | NDArray[np.floating[Any]]
#: A 3-vector by value; same accepted spellings as `QuatLike`.
Vec3Like: TypeAlias = Sequence[float] | NDArray[np.floating[Any]]
#: 3x3 rotation matrix, float64.
Mat3: TypeAlias = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class Cloud(Generic[F]):
    """(..., 3) points expressed in frame `F` -- a flat (N, 3) cloud or the
    surface path's (h, w, 3) grid. `Cloud(pts, CV)` infers `Cloud[CV]` -- always
    pass the frame class, never rely on inference from `pts` (an untagged
    construction would be `Cloud[Unknown]`, which Pyright lets through
    anywhere). Unwrap with `.xyz` at the numpy / Open3D boundary."""
    xyz: NDArray[np.floating[Any]]
    frame: type[F]

    def __post_init__(self) -> None:
        if self.xyz.ndim < 2 or self.xyz.shape[-1] != 3:
            raise ValueError(f"Cloud expects a (..., 3) array, got shape {self.xyz.shape}")

    def __len__(self) -> int:
        """Number of points (all leading axes flattened)."""
        return int(self.xyz.size // 3)


@dataclass(frozen=True, slots=True)
class Rotation(Generic[To, From]):
    """3x3 rotation taking `From`-frame coordinates to `To`-frame coordinates.
    Read `Rotation[CvWorld, CV]` as "CvWorld <- CV". `Rotation(m, CvWorld, CV)`
    infers the parameters from the two frame classes.

    Composition is checked both ways: `a @ b` requires `a.src is b.dst`
    (Pyright rejects the mismatch; at runtime it raises), and `r @ cloud` /
    `r.apply(cloud)` requires `cloud.frame is r.src`."""
    matrix: Mat3
    dst: type[To]
    src: type[From]

    def __post_init__(self) -> None:
        if self.matrix.shape != (3, 3):
            raise ValueError(f"Rotation expects a (3, 3) matrix, got shape {self.matrix.shape}")

    @overload
    def __matmul__(self, other: Rotation[From, _X]) -> Rotation[To, _X]: ...
    @overload
    def __matmul__(self, other: Cloud[From]) -> Cloud[To]: ...

    def __matmul__(self, other: Rotation[From, _X] | Cloud[From]) -> Rotation[To, _X] | Cloud[To]:
        if isinstance(other, Cloud):
            return self.apply(other)
        return self.then(other)

    def then(self, other: Rotation[From, _X]) -> Rotation[To, _X]:
        """`self @ other`: first `other` (X -> From), then `self` (From -> To)."""
        if other.dst is not self.src:
            raise ValueError(
                f"frame mismatch composing {self.dst.__name__}<-{self.src.__name__} "
                f"with {other.dst.__name__}<-{other.src.__name__}")
        return Rotation(self.matrix @ other.matrix, self.dst, other.src)

    def apply(self, cloud: Cloud[From]) -> Cloud[To]:
        """Rotate every point of a `From`-frame cloud into the `To` frame."""
        if cloud.frame is not self.src:
            raise ValueError(
                f"frame mismatch: rotation takes {self.src.__name__}, "
                f"cloud is in {cloud.frame.__name__}")
        return Cloud(cloud.xyz @ self.matrix.T, self.dst)

    def inverse(self) -> Rotation[From, To]:
        return Rotation(np.ascontiguousarray(self.matrix.T), self.src, self.dst)
