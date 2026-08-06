"""Tests for the Filament srgbColor stderr filter (logfilter.py).

The pure predicate is checked directly; the fd-level plumbing is exercised
end-to-end by writing through the C runtime (fd 2) and asserting the benign
warning is dropped while everything else survives.
"""
import ctypes
import ctypes.util
import os
import sys
import threading
import time

import pytest


def _c_runtime_write():
    """Return `(lib, write_fn_name)` for the platform C runtime whose fd-level
    writes the Filament filter must intercept, or None if none is loadable.

    The srgbColor spam originates in Filament, which links UCRT on Windows -- so
    that is what the end-to-end test reproduces THERE. But the filter interposes
    on **fd 2**, which is OS-agnostic, so on POSIX the identical mechanism is
    exercised through the platform's own libc. Selecting the runtime by OS lets
    this test actually run everywhere instead of being a permanent skip on the
    (Linux) box this project develops on. The POSIX symbol is `write`; UCRT's is
    `_write`."""
    if sys.platform == "win32":
        try:
            return ctypes.CDLL("ucrtbase"), "_write"
        except OSError:
            return None
    libc_name = ctypes.util.find_library("c") or "libc.so.6"
    try:
        return ctypes.CDLL(libc_name), "write"
    except OSError:
        return None

from roomscan import logfilter


# ---- pure predicate --------------------------------------------------------

def test_should_drop_matches_both_warning_lines():
    assert logfilter._should_drop(
        b"in filament::UniformInterfaceBlock::getUniformOffset(...):120")
    assert logfilter._should_drop(
        b'reason: uniform named "srgbColor" not found')


def test_should_drop_keeps_real_output():
    for keep in (b"Traceback (most recent call last):",
                 b"[Open3D WARNING] something important",
                 b"reason: uniform named \"baseColor\" not found",
                 b"regular log line"):
        assert not logfilter._should_drop(keep)


# ---- end-to-end fd plumbing ------------------------------------------------

def _read_all(fd, stop_after, sink):
    buf = b""
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        try:
            chunk = os.read(fd, 65536)
        except OSError:
            break
        if chunk:
            buf += chunk
            if stop_after in buf:
                break
    sink.append(buf)


def test_pipe_drops_srgb_and_keeps_the_rest():
    """Interpose our filter on a private pipe pair (not the process's real fd 2,
    so pytest's own capture is untouched) and confirm srgbColor lines vanish
    while a sentinel survives."""
    # Downstream capture pipe: the filter re-emits kept lines into here.
    out_r, out_w = os.pipe()
    # Upstream pipe the "app" writes into; the filter pumps it to out_w.
    in_r, in_w = os.pipe()
    t = threading.Thread(target=logfilter._pump, args=(in_r, out_w), daemon=True)
    t.start()

    captured = []
    reader = threading.Thread(target=_read_all, args=(out_r, b"SENTINEL", captured))
    reader.start()

    payload = (b"in filament::UniformInterfaceBlock::getUniformOffset(...):120\n"
               b'reason: uniform named "srgbColor" not found\n'
               b"a normal line\n"
               b"SENTINEL\n")
    os.write(in_w, payload)
    os.close(in_w)          # EOF -> pump drains and exits
    reader.join(timeout=3.0)
    os.close(out_w)
    os.close(out_r)

    got = b"".join(captured)
    assert b"srgbColor" not in got
    assert b"getUniformOffset" not in got
    assert b"a normal line" in got
    assert b"SENTINEL" in got


def test_install_respects_opt_out(monkeypatch):
    monkeypatch.setenv("ROOMSCAN_KEEP_FILAMENT_LOGS", "1")
    assert logfilter.install_filament_stderr_filter() is False


def test_install_filters_real_c_runtime_stderr(monkeypatch):
    """Full-stack: install on the real fd 2, write the warning via the platform
    C runtime (UCRT on Windows, libc on POSIX -- the filter is fd-level, so
    either exercises it), and confirm it's dropped from what reaches the original
    console while a sentinel passes through."""
    monkeypatch.delenv("ROOMSCAN_KEEP_FILAMENT_LOGS", raising=False)
    runtime = _c_runtime_write()
    if runtime is None:
        pytest.skip("no C runtime loadable for a real fd-2 write")
    libc, write_name = runtime
    real = os.dup(2)                     # snapshot the true stderr up front
    sink_r, sink_w = os.pipe()
    os.dup2(sink_w, 2)                    # route the true stderr into our sink
    os.close(sink_w)
    try:
        installed = logfilter.install_filament_stderr_filter()
        assert installed is True
        msg = (b"in filament::UniformInterfaceBlock::getUniformOffset:120\n"
               b'reason: uniform named "srgbColor" not found\n'
               b"KEEPME-e2e\n")
        getattr(libc, write_name)(2, msg, len(msg))
        time.sleep(0.3)
        captured = []
        # Read whatever the filter re-emitted into the sink.
        os.set_blocking(sink_r, False)
        try:
            captured.append(os.read(sink_r, 65536))
        except (BlockingIOError, OSError):
            pass
    finally:
        os.dup2(real, 2)                 # always restore before leaving
        os.close(real)
        try:
            os.close(sink_r)
        except OSError:
            pass
    got = b"".join(captured)
    assert b"KEEPME-e2e" in got
    assert b"srgbColor" not in got
