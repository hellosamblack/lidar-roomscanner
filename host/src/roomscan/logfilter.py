"""Drop one known-benign, high-volume Filament warning from the process's
C-level stderr (fd 2) without hiding anything else.

Open3D 0.19 ships `defaultUnlitTransparency.filamat` WITHOUT the `srgbColor`
uniform that `defaultUnlit.filamat` declares, yet Filament's shared
`FilamentScene::UpdateDefaultUnlit` binds `srgbColor` unconditionally. Every
material bind of a translucent geometry -- our per-frame first-person IR
billboard (`_update_ir_overlay` re-adds it each frame) and the see-through
walls -- therefore makes Filament log, at the sensor frame rate:

    in ... filament::UniformInterfaceBlock::getUniformOffset(...):NNN
    reason: uniform named "srgbColor" not found

Rendering is unaffected -- it's pure console noise, but at ~28 fps it floods the
terminal. `contextlib.redirect_stderr` can't catch it: that only rebinds
Python's `sys.stderr`, whereas Filament writes through the C runtime's fd 2. So
we interpose an OS pipe on fd 2 and a reader thread that forwards every line
EXCEPT the two that make up this warning. Everything else on stderr -- Python
tracebacks, other library output -- passes through verbatim.

Verified on this platform: redirecting fd 2 captures UCRT-level stderr writes
(Filament links the same UCRT as CPython 3.12). Set ROOMSCAN_KEEP_FILAMENT_LOGS=1
to disable (leaves stderr untouched).
"""
from __future__ import annotations

import os
import threading

# Substrings unique to the two lines of the srgbColor warning. Both are so
# specific to this Filament diagnostic that no genuine error text collides, so
# dropping any line containing either can never hide a real problem.
_DROP_TOKENS = (b"srgbColor", b"getUniformOffset")


def _should_drop(line: bytes) -> bool:
    """True if `line` is part of the benign Filament srgbColor warning."""
    return any(tok in line for tok in _DROP_TOKENS)


def _pump(read_fd: int, out_fd: int) -> None:
    """Forward pipe->real stderr line by line, skipping the srgbColor warning.
    Wrapped so the thread can never die and leave the pipe to fill and block a
    stderr write (the warning is cosmetic; never let it hang the app)."""
    buf = b""
    while True:
        try:
            chunk = os.read(read_fd, 65536)
        except OSError:
            break
        if not chunk:
            break
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if not _should_drop(line):
                try:
                    os.write(out_fd, line + b"\n")
                except OSError:
                    pass
    if buf and not _should_drop(buf):          # trailing partial line at EOF
        try:
            os.write(out_fd, buf)
        except OSError:
            pass
    try:
        os.close(read_fd)
    except OSError:
        pass


def install_filament_stderr_filter():
    """Best-effort: interpose a filtering pipe on fd 2. Returns True if
    installed, False if skipped (opt-out env var, or fd 2 isn't a real OS fd --
    e.g. captured under pytest). Never raises."""
    if os.environ.get("ROOMSCAN_KEEP_FILAMENT_LOGS"):
        return False
    try:
        real_fd = os.dup(2)               # a private handle to the true stderr
    except (OSError, ValueError):
        return False                      # fd 2 not a real OS fd -> leave it be
    try:
        read_fd, write_fd = os.pipe()
        os.dup2(write_fd, 2)              # all fd-2 writers now feed the pipe
        os.close(write_fd)
    except OSError:
        try:
            os.close(real_fd)
        except OSError:
            pass
        return False
    threading.Thread(target=_pump, args=(read_fd, real_fd),
                     name="filament-stderr-filter", daemon=True).start()
    return True
