"""ctypes wrapper around roomscan_transform.dll (host/transform/).

The DLL wraps ST's vl53l9-transform-c pipeline behind a tiny C ABI
(rst_create2/rst_process2/rst_destroy, see host/transform/rs_transform_shim.h)
so the same on-device depth pipeline can run PC-side against captured raw
frames. Requires the DLL to have been built (see host/transform/CMakeLists.txt);
Transform.available() reports whether that build is present so callers (and
tests) can gate on it instead of failing hard.

Multi-output: Transform(calib, outputs=(...)) selects any combination of
"depth" (ZF32), "reflectance" (RF32), "confidence" (CF32), "ambient" (IF32)
-- all four share one prepared transform instance and one
transform_process_stream() call per process() -- and "zapc" (ZAPC: an
on-device [x,y,z,confidence] point cloud), which the shim negotiates on a
second, independently-stateful instance because ZAPC is a mutually-exclusive
*format* of the same "depth" stream as ZF32, not a separate stream (see
rs_transform_shim.h's v2 header comment for the full cost/consistency
discussion). process() returns a dict keyed by output name; dtypes/shapes
per rs_transform_shim.h's format table.
"""
from __future__ import annotations

import ctypes
import os
from pathlib import Path

import numpy as np

from .protocol import RAW_3DMD_SIZE_BIN2, CALIB_SIZE

_DLL_NAME = "roomscan_transform.dll"

_RAW_IN_WIDTH = RAW_3DMD_SIZE_BIN2
_RAW_IN_HEIGHT = 1
_OUT_WIDTH = 54
_OUT_HEIGHT = 42
_OUT_SHAPE = (_OUT_HEIGHT, _OUT_WIDTH)
_OUT_COUNT = _OUT_WIDTH * _OUT_HEIGHT
_ZAPC_SHAPE = (_OUT_HEIGHT, _OUT_WIDTH, 4)  # [x, y, z, confidence] per zone

# Must match RST_OUT_* bits in host/transform/rs_transform_shim.h.
_OUTPUT_MASKS = {
    "depth": 1 << 0,
    "reflectance": 1 << 1,
    "confidence": 1 << 2,
    "ambient": 1 << 3,
    "zapc": 1 << 4,
}
# float32 planes (one value/zone); "zapc" is handled separately (4 values/zone).
_FLOAT_PLANE_OUTPUTS = ("depth", "reflectance", "confidence", "ambient")

_BUILD_HINT = (
    "native transform DLL not found. Build it with:\n"
    '  cmake -S host/transform -B host/transform/build -G "Visual Studio 18 2026" -A x64\n'
    "  cmake --build host/transform/build --config Release\n"
    "or point ROOMSCAN_TRANSFORM_DLL at an existing roomscan_transform.dll."
)


def _candidate_paths() -> list[Path]:
    """Search order: env override -> local build dir -> alongside this package."""
    candidates: list[Path] = []
    env_path = os.environ.get("ROOMSCAN_TRANSFORM_DLL")
    if env_path:
        candidates.append(Path(env_path))

    package_dir = Path(__file__).resolve().parent
    host_dir = package_dir.parent.parent  # roomscan/ -> src/ -> host/
    candidates.append(host_dir / "transform" / "build" / "Release" / _DLL_NAME)

    candidates.append(package_dir / _DLL_NAME)
    return candidates


def _find_dll() -> Path | None:
    for path in _candidate_paths():
        if path.is_file():
            return path
    return None


def _load_dll() -> ctypes.CDLL | None:
    path = _find_dll()
    if path is None:
        return None
    try:
        lib = ctypes.CDLL(str(path))
    except OSError:
        return None

    lib.rst_create.argtypes = [
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
    ]
    lib.rst_create.restype = ctypes.c_void_p

    lib.rst_process.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32, ctypes.POINTER(ctypes.c_float),
    ]
    lib.rst_process.restype = ctypes.c_int

    lib.rst_create2.argtypes = [
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
    ]
    lib.rst_create2.restype = ctypes.c_void_p

    lib.rst_process2.argtypes = [
        ctypes.c_void_p,                # h
        ctypes.POINTER(ctypes.c_uint8), # raw
        ctypes.c_uint32,                # raw_len
        ctypes.POINTER(ctypes.c_float), # depth_out
        ctypes.c_void_p,                # reflectance_out
        ctypes.c_void_p,                # confidence_out
        ctypes.c_void_p,                # ambient_out
        ctypes.POINTER(ctypes.c_float), # zapc_out
    ]
    lib.rst_process2.restype = ctypes.c_int

    lib.rst_destroy.argtypes = [ctypes.c_void_p]
    lib.rst_destroy.restype = None
    return lib


class Transform:
    """One prepared vl53l9 transform pipeline instance (binning-2 profile:
    14842-byte raw input, 54x42 output resolution). `outputs` selects any
    combination of "depth" (ZF32), "reflectance" (RF32), "confidence" (CF32),
    "ambient" (IF32) -- all four share one transform instance/process call --
    and "zapc" (ZAPC point cloud), negotiated on a second, independently
    stateful instance because it is a mutually-exclusive *format* of the same
    "depth" stream as ZF32, not a separate stream. See
    host/transform/rs_transform_shim.h for the full capability-model
    discussion and per-output dtypes/shapes."""

    def __init__(self, calib: bytes, outputs: tuple[str, ...] = ("depth",)):
        lib = _load_dll()
        if lib is None:
            raise RuntimeError(_BUILD_HINT)
        if len(calib) != CALIB_SIZE:
            raise ValueError(f"calib must be {CALIB_SIZE} bytes (VL53L9_CALIB_DATA_SIZE), got {len(calib)}")
        if not outputs:
            raise ValueError("outputs must be non-empty")
        unknown = sorted(set(outputs) - set(_OUTPUT_MASKS))
        if unknown:
            raise ValueError(f"unknown output(s) {unknown}; valid: {sorted(_OUTPUT_MASKS)}")

        self._lib = lib
        self._outputs = tuple(outputs)
        mask = 0
        for name in self._outputs:
            mask |= _OUTPUT_MASKS[name]

        calib_buf = (ctypes.c_uint8 * len(calib)).from_buffer_copy(calib)
        handle = lib.rst_create2(calib_buf, len(calib), _RAW_IN_WIDTH, _RAW_IN_HEIGHT, mask)
        if not handle:
            raise RuntimeError("rst_create2 failed (transform setup rejected the calib data, capabilities, or mask)")
        self._handle = handle

    @classmethod
    def available(cls) -> bool:
        """True if the native DLL can be found and loaded (does not require a live handle)."""
        return _load_dll() is not None

    def process(self, raw: bytes) -> dict[str, np.ndarray]:
        """Run one raw 3DMD frame through the transform. Returns a dict keyed by the
        `outputs` names passed to __init__: "depth"/"reflectance"/"confidence"/"ambient"
        are (42, 54) float32 arrays; "zapc" is a (42, 54, 4) float32 array of
        [x, y, z, confidence] per zone."""
        if self._handle is None:
            raise RuntimeError("Transform used after destroy()")
        # Assumes binning=2 profile (54x42 resolution); other binnings would need a parameterized size.
        if len(raw) != RAW_3DMD_SIZE_BIN2:
            raise ValueError(f"raw must be {RAW_3DMD_SIZE_BIN2} bytes, got {len(raw)}")

        raw_buf = (ctypes.c_uint8 * len(raw)).from_buffer_copy(raw)

        arrays: dict[str, np.ndarray] = {}
        float_ptrs: dict[str, object] = {}
        for name in _FLOAT_PLANE_OUTPUTS:
            if name in self._outputs:
                arr = np.empty(_OUT_SHAPE, dtype=np.float32)
                arrays[name] = arr
                float_ptrs[name] = arr.ctypes.data_as(ctypes.c_void_p)
            else:
                float_ptrs[name] = None

        if "zapc" in self._outputs:
            zapc = np.empty(_ZAPC_SHAPE, dtype=np.float32)
            arrays["zapc"] = zapc
            zapc_ptr = zapc.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        else:
            zapc_ptr = None

        depth_ptr = float_ptrs["depth"]
        depth_ptr = ctypes.cast(depth_ptr, ctypes.POINTER(ctypes.c_float)) if depth_ptr is not None else None

        ret = self._lib.rst_process2(
            self._handle, raw_buf, len(raw),
            depth_ptr, float_ptrs["reflectance"], float_ptrs["confidence"], float_ptrs["ambient"], zapc_ptr,
        )
        if ret != 0:
            raise RuntimeError(f"rst_process2 failed with code {ret}")
        return arrays

    def destroy(self) -> None:
        if self._handle is not None:
            self._lib.rst_destroy(self._handle)
            self._handle = None

    def __del__(self):
        try:
            self.destroy()
        except Exception:
            pass

    def __enter__(self) -> "Transform":
        return self

    def __exit__(self, *exc_info) -> None:
        self.destroy()
