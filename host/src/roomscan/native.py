"""ctypes wrapper around roomscan_transform.dll (host/transform/).

The DLL wraps ST's vl53l9-transform-c pipeline behind a tiny C ABI
(rst_create/rst_process/rst_destroy, see host/transform/rs_transform_shim.h)
so the same on-device depth pipeline can run PC-side against captured raw
frames. Requires the DLL to have been built (see host/transform/CMakeLists.txt);
Transform.available() reports whether that build is present so callers (and
tests) can gate on it instead of failing hard.
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

    lib.rst_destroy.argtypes = [ctypes.c_void_p]
    lib.rst_destroy.restype = None
    return lib


class Transform:
    """One prepared vl53l9 transform pipeline instance (binning-2 profile:
    14842-byte raw input, 54x42 ZF32 depth output)."""

    def __init__(self, calib: bytes):
        lib = _load_dll()
        if lib is None:
            raise RuntimeError(_BUILD_HINT)
        if len(calib) != CALIB_SIZE:
            raise ValueError(f"calib must be {CALIB_SIZE} bytes (VL53L9_CALIB_DATA_SIZE), got {len(calib)}")

        self._lib = lib
        calib_buf = (ctypes.c_uint8 * len(calib)).from_buffer_copy(calib)
        handle = lib.rst_create(calib_buf, len(calib), _RAW_IN_WIDTH, _RAW_IN_HEIGHT)
        if not handle:
            raise RuntimeError("rst_create failed (transform setup rejected the calib data or capabilities)")
        self._handle = handle

    @classmethod
    def available(cls) -> bool:
        """True if the native DLL can be found and loaded (does not require a live handle)."""
        return _load_dll() is not None

    def process(self, raw: bytes) -> np.ndarray:
        """Run one raw 3DMD frame through the transform. Returns a (42, 54) float32 depth array."""
        if self._handle is None:
            raise RuntimeError("Transform used after destroy()")
        # Assumes binning=2 profile (54x42 resolution); other binnings would need a parameterized size.
        if len(raw) != RAW_3DMD_SIZE_BIN2:
            raise ValueError(f"raw must be {RAW_3DMD_SIZE_BIN2} bytes, got {len(raw)}")

        raw_buf = (ctypes.c_uint8 * len(raw)).from_buffer_copy(raw)
        depth = np.empty(_OUT_SHAPE, dtype=np.float32)
        depth_ptr = depth.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        ret = self._lib.rst_process(self._handle, raw_buf, len(raw), depth_ptr)
        if ret != 0:
            raise RuntimeError(f"rst_process failed with code {ret}")
        return depth

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
