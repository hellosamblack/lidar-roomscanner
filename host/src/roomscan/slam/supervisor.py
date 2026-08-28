"""Spawn and supervise the SLAM compute service as a subprocess of the web app
(#224, productising E2 of #223).

`roomscan-web` with `[slam] backend = "remote"` needs a `SlamService` listening
on `remote_addr`. When that address is loopback and `remote_spawn` is on, this
owns it: `ensure()` starts `python -m roomscan.slam.service` if it is not
running (and waits until it accepts), `alive` says whether it still is, and
`stop()` ends it cleanly. The service holds the Open3D/CUDA state, so when the
mapper dies (CUDA OOM, BUG-053; an Open3D abort) the SERVICE process dies and
the web server does not -- `SlamRunner.poll` notices the dead socket, tears
its worker down, and the next frame's rebuild calls `ensure()` again, which is
the restart. There is deliberately no restart loop in here: a service that
dies on every connect would otherwise flap forever; letting the rebuild path
own it means each restart is one failed map, visible on the log bus.

The child runs the SAME source tree as the parent (`PYTHONPATH` is set to the
parent's `roomscan` package root): a worktree's server must not spawn main's
service, and vice versa. Its stdout/stderr are pumped line by line onto the
log bus as `[slam-service] ...`, so the operator sees its device/client/credit
lines where every other SLAM line lands.
"""
from __future__ import annotations

import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

_LISTEN_TIMEOUT_S = 90.0      # Open3D + CUDA import on a cold box
_LOOPBACK = ("127.0.0.1", "localhost", "::1", "")


def is_loopback(addr: str) -> bool:
    host, _, _port = addr.rpartition(":")
    return host.strip("[]") in _LOOPBACK


def _port_of(addr: str) -> int:
    return int(addr.rpartition(":")[2])


def _listening(port: int, timeout: float = 0.2) -> bool:
    try:
        socket.create_connection(("127.0.0.1", port), timeout=timeout).close()
        return True
    except OSError:
        return False


class SlamServiceSupervisor:
    def __init__(self, addr: str = "127.0.0.1:5555", device: str = "CUDA:0",
                 publish=None, python: str | None = None):
        self.addr = str(addr)
        self.device = str(device)
        self._publish = publish or (lambda line: None)
        self._python = python or sys.executable
        self._proc: subprocess.Popen | None = None
        self._pump: threading.Thread | None = None
        self._lock = threading.Lock()
        self.starts = 0
        self.last_exit_code: int | None = None

    # ---- state -------------------------------------------------------------------
    @property
    def pid(self) -> int | None:
        return self._proc.pid if self._proc is not None else None

    @property
    def alive(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    # ---- lifecycle -----------------------------------------------------------------
    def ensure(self, timeout: float = _LISTEN_TIMEOUT_S) -> bool:
        """Make a service listen on `addr`. Returns True when one is listening
        (ours, or -- for a non-loopback / already-bound address -- somebody
        else's). Spawns at most once per call; a child that exits before it
        listens is reported and False is returned."""
        port = _port_of(self.addr)
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return True
            if self._proc is not None:
                self._reap_locked()
            if not is_loopback(self.addr):
                return _listening(port)
            if _listening(port):
                # Something already owns the port (a hand-run service, or a
                # previous server's child that outlived it). Use it.
                self._publish(f"[slam-service] port {port} already in use; using it")
                return True
            self._spawn_locked(port)
            proc = self._proc
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                self.last_exit_code = proc.returncode
                self._publish(f"[slam-service] exited before listening (rc={proc.returncode})")
                return False
            if _listening(port):
                return True
            time.sleep(0.1)
        self._publish(f"[slam-service] did not listen on {port} within {timeout:.0f} s; killing")
        self.stop()
        return False

    def _spawn_locked(self, port: int) -> None:
        import roomscan
        env = dict(os.environ)
        src = str(Path(roomscan.__file__).resolve().parents[1])
        env["PYTHONPATH"] = src + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        env.setdefault("PYTHONUNBUFFERED", "1")
        cmd = [self._python, "-m", "roomscan.slam.service",
               "--host", "127.0.0.1", "--port", str(port), "--device", self.device]
        self._proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT, start_new_session=True)
        self.starts += 1
        self._publish(f"[slam-service] spawned pid {self._proc.pid} on 127.0.0.1:{port} "
                      f"device={self.device} (start #{self.starts})")
        self._pump = threading.Thread(target=self._pump_lines, args=(self._proc,),
                                      daemon=True, name="slam-service-log")
        self._pump.start()

    def _pump_lines(self, proc: subprocess.Popen) -> None:
        try:
            for raw in iter(proc.stdout.readline, b""):
                line = raw.decode("utf-8", "replace").rstrip()
                if line:
                    self._publish(line if line.startswith("[slam-service]")
                                  else f"[slam-service] {line}")
        except (OSError, ValueError):
            pass
        rc = proc.wait()
        self.last_exit_code = rc
        self._publish(f"[slam-service] pid {proc.pid} exited rc={rc}")

    def _reap_locked(self) -> None:
        proc, self._proc = self._proc, None
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5.0)
        if proc is not None:
            self.last_exit_code = proc.returncode

    def stop(self) -> None:
        with self._lock:
            self._reap_locked()
        pump = self._pump
        if pump is not None:
            pump.join(timeout=2.0)
            self._pump = None
