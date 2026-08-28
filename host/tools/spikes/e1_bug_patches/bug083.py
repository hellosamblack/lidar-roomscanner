"""BUG-083: a zero-filled ENV frame (0.0 Pa) reached baro_height_m, which
returned the formula ceiling (44330 m) and fabricated 8.6 m of SLAM vertical."""
from roomscan.domain import EnvSample
from roomscan.protocol import decode_env
from roomscan.slam.frames import baro_height_m


def height(payload: bytes, ref_pa: float) -> float:
    env = EnvSample.from_wire(payload)
    return baro_height_m(env.pressure_pa, ref_pa)   # env may be None: must unwrap first


def height_raw(payload: bytes, ref_pa: float) -> float:
    # ESCAPE HATCH: the untyped decoder still exists; Pyright accepts this line.
    pressure, _mag, _temp = decode_env(payload)
    return baro_height_m(pressure, ref_pa)
