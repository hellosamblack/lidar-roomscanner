"""Device-memory probe for GPU verification (sub-phase 6.G).

Open3D exposes no "bytes currently allocated" API -- `o3d.core.cuda` offers only
`device_count` / `is_available` / `release_cache` / `synchronize` -- so measuring
the long-scan memory creep needs NVML. This is a ctypes binding rather than
pynvml so no new dependency lands in the venv (which has neither pynvml nor
torch), and so it works unchanged inside the GPU container.

Readings are DEVICE-WIDE, not per-process: sample a baseline before building any
Open3D state and subtract it, and don't trust the numbers with another CUDA
process on the same card.

Used by `host/tools/slam_gpu_memory.py` (the measurement rig) and
`tools/slam-container/cuda_smoke.py` (the memory-ceiling regression guard).
"""
from __future__ import annotations

import ctypes


class _NvmlMemory(ctypes.Structure):
    _fields_ = [("total", ctypes.c_ulonglong),
                ("free", ctypes.c_ulonglong),
                ("used", ctypes.c_ulonglong)]


class Nvml:
    """Minimal NVML binding: device-wide used/total bytes.

    `ok` is False when libnvidia-ml is missing or NVML init fails; every getter
    then returns 0 rather than raising, so a caller can still run (and report
    block counts) on a box without the driver library. Check `ok` before
    treating a 0 as a real measurement."""

    def __init__(self, index: int = 0):
        self.ok = False
        self._lib = None
        try:
            self._lib = ctypes.CDLL("libnvidia-ml.so.1")
        except OSError:
            return
        if self._lib.nvmlInit_v2() != 0:
            return
        self._handle = ctypes.c_void_p()
        if self._lib.nvmlDeviceGetHandleByIndex_v2(
                ctypes.c_uint(index), ctypes.byref(self._handle)) != 0:
            self._lib.nvmlShutdown()
            return
        self.ok = True

    def _mem(self) -> _NvmlMemory | None:
        if not self.ok:
            return None
        mem = _NvmlMemory()
        if self._lib.nvmlDeviceGetMemoryInfo(self._handle, ctypes.byref(mem)) != 0:
            return None
        return mem

    def used_bytes(self) -> int:
        mem = self._mem()
        return int(mem.used) if mem is not None else 0

    def total_bytes(self) -> int:
        mem = self._mem()
        return int(mem.total) if mem is not None else 0

    def close(self) -> None:
        if self.ok:
            self._lib.nvmlShutdown()
            self.ok = False

    def __enter__(self) -> "Nvml":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
