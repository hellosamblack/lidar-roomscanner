"""Unit tests for `roomscan.slam.gpumem` (ctypes NVML binding). No real GPU/
driver needed -- `ctypes.CDLL` is monkeypatched to either fail (simulating a
box with no `libnvidia-ml.so.1`, e.g. this headless dev host without a
passed-through GPU) or to a fake library object whose methods fill the ctypes
structs the same way the real driver call would."""
import ctypes

import roomscan.slam.gpumem as gpumem_mod
from roomscan.slam.gpumem import Nvml, device_utilization


class _FakeLib:
    """Stands in for the `ctypes.CDLL` handle: plain Python methods, called
    the same way `Nvml` calls the real driver entry points. A `byref(struct)`
    argument is recoverable via `ctypes.cast(..., POINTER(type))` -- this is
    how a pure-Python fake can still write into the caller's ctypes struct
    without a real C call underneath."""

    def __init__(self, gpu_pct=55, mem_pct=77):
        self._gpu_pct = gpu_pct
        self._mem_pct = mem_pct
        self.shutdown_calls = 0

    def nvmlInit_v2(self):
        return 0

    def nvmlDeviceGetHandleByIndex_v2(self, index, handle_ref):
        return 0

    def nvmlDeviceGetUtilizationRates(self, handle, util_ref):
        util = ctypes.cast(util_ref, ctypes.POINTER(gpumem_mod._NvmlUtilization)).contents
        util.gpu = self._gpu_pct
        util.memory = self._mem_pct
        return 0

    def nvmlShutdown(self):
        self.shutdown_calls += 1
        return 0


def test_device_utilization_is_none_when_nvml_library_is_absent(monkeypatch):
    # No libnvidia-ml.so.1 on the box (or driver not loaded) -- CDLL raises
    # OSError, same as the real failure mode on a GPU-less host.
    def raise_oserror(name):
        raise OSError("cannot load library")

    monkeypatch.setattr(gpumem_mod.ctypes, "CDLL", raise_oserror)
    assert device_utilization() is None


def test_device_utilization_is_none_when_nvml_init_fails(monkeypatch):
    class _InitFailsLib(_FakeLib):
        def nvmlInit_v2(self):
            return 1   # any non-zero NVML return code is failure

    monkeypatch.setattr(gpumem_mod.ctypes, "CDLL", lambda name: _InitFailsLib())
    assert device_utilization() is None


def test_device_utilization_parses_a_faked_struct_correctly(monkeypatch):
    fake = _FakeLib(gpu_pct=42, mem_pct=13)
    monkeypatch.setattr(gpumem_mod.ctypes, "CDLL", lambda name: fake)

    result = device_utilization()

    assert result == {"gpu_pct": 42, "mem_pct": 13}
    assert fake.shutdown_calls == 1     # Nvml is closed after the query, not leaked


def test_device_utilization_never_raises_on_a_broken_query(monkeypatch):
    class _BrokenUtilLib(_FakeLib):
        def nvmlDeviceGetUtilizationRates(self, handle, util_ref):
            raise AttributeError("symbol not present in this driver build")

    monkeypatch.setattr(gpumem_mod.ctypes, "CDLL", lambda name: _BrokenUtilLib())
    assert device_utilization() is None


def test_nvml_utilization_method_returns_none_when_not_ok(monkeypatch):
    # Direct unit test of Nvml.utilization() itself, not just the module
    # convenience wrapper -- `ok` gates the query the same way the other
    # getters (`_mem`, `name`) do.
    monkeypatch.setattr(gpumem_mod.ctypes, "CDLL",
                         lambda name: (_ for _ in ()).throw(OSError()))
    nvml = Nvml()
    assert nvml.ok is False
    assert nvml.utilization() is None
