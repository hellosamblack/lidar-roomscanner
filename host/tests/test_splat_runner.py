"""SplatRunner state machine + the video-name sanitizer (web.py).

The runner spawns the splat CLI as a subprocess; here `_spawn` is replaced by a
fake process so the state machine is exercised with no GPU/COLMAP. `pgrep` is also
stubbed off, since the real detached build would otherwise make the guard refuse."""
import json
import threading
import types

from roomscan import web


class FakeProc:
    """Stand-in for subprocess.Popen. Writes the CLI's report/progress files itself
    (as the real child would), and stays 'alive' until finished."""

    def __init__(self, *, report_path=None, progress_path=None, ok=True, stats=None,
                 write_report=True, stderr_lines=(), auto_finish=True):
        self.returncode = 0 if ok else 1
        self.stderr = iter(list(stderr_lines))
        self._report_path, self._progress_path = report_path, progress_path
        self._ok, self._stats, self._write_report = ok, stats, write_report
        self._event = threading.Event()
        if auto_finish:
            self.finish()

    def finish(self):
        if self._progress_path is not None:
            self._progress_path.write_text(json.dumps({"phase": "train", "fraction": 0.5}))
        if self._write_report and self._report_path is not None:
            self._report_path.write_text(json.dumps(
                {"ok": self._ok, "stats": self._stats or {},
                 "reason": None if self._ok else "boom"}))
        self._event.set()

    def wait(self, timeout=None):
        self._event.wait(timeout)
        return self.returncode

    def poll(self):
        return self.returncode if self._event.is_set() else None

    def terminate(self):
        self._event.set()

    def kill(self):
        self._event.set()


def _runner():
    bus = types.SimpleNamespace(publish=lambda *a, **k: None)
    r = web.SplatRunner(bus=bus, results_dir="results", captures_dir="captures")
    r._external_build_running = lambda: False        # ignore the real detached build
    return r


def test_start_runs_to_done_with_stats():
    r = _runner()
    r._spawn = lambda cmd, env: FakeProc(
        report_path=r._report_path, progress_path=r._progress_path,
        ok=True, stats={"gaussians": 2_000_000, "peak_vram_gib": 2.6})
    res = r.start("captures/room.mp4", "Sam Office", {"iters": 500}, force=False)
    assert res["started"] and res["slug"] == "sam-office"
    r._thread.join(timeout=5)
    msg = r.poll()
    assert msg["done"] and msg["phase"] == "done"
    assert msg["stats"]["gaussians"] == 2_000_000
    assert r.poll() is None                          # completion emitted exactly once


def test_second_start_is_refused_while_running():
    r = _runner()
    # A proc that never finishes on its own -> stays 'alive' (poll() is None).
    proc = FakeProc(report_path=r._report_path, auto_finish=False)
    r._spawn = lambda cmd, env: proc
    assert r.start("captures/room.mp4", "One", {}, force=False)["started"]
    refused = r.start("captures/room.mp4", "Two", {}, force=False)
    assert refused["started"] is False and "running" in refused["reason"]
    r.close()                                        # releases the fake proc


def test_external_build_blocks_start():
    r = _runner()
    r._external_build_running = lambda: True         # the detached CLI holds the GPU
    res = r.start("captures/room.mp4", "Sam Office", {}, force=False)
    assert res["started"] is False and "GPU busy" in res["reason"]


def test_error_when_no_report():
    r = _runner()
    r._spawn = lambda cmd, env: FakeProc(report_path=r._report_path, ok=False,
                                         write_report=False)
    assert r.start("captures/room.mp4", "Sam Office", {}, force=False)["started"]
    r._thread.join(timeout=5)
    msg = r.poll()
    assert msg["done"] and msg["phase"] == "failed" and msg["error"]


def test_build_cmd_maps_params_to_flags():
    r = _runner()
    r._report_path = r._progress_path = None
    cmd = r._build_cmd("captures/room.mp4", "Sam Office",
                       {"depth_lambda": 0.1, "max_gaussians": 1500000}, force=True)
    assert "--force" in cmd
    assert "--depth-lambda" in cmd and "0.1" in cmd
    assert "--max-gaussians" in cmd and "1500000" in cmd
    assert cmd[cmd.index("--name") + 1] == "Sam Office"


def test_sanitize_video_name(tmp_path):
    (tmp_path / "room.mp4").write_bytes(b"x")
    (tmp_path / "clip.MOV").write_bytes(b"y")
    assert web.sanitize_video_name("room.mp4", tmp_path).name == "room.mp4"
    assert web.sanitize_video_name("clip.MOV", tmp_path).name == "clip.MOV"   # case-insensitive
    assert web.sanitize_video_name("../room.mp4", tmp_path) is None           # traversal
    assert web.sanitize_video_name("sub/room.mp4", tmp_path) is None          # separator
    assert web.sanitize_video_name("room.bin", tmp_path) is None              # wrong ext
    assert web.sanitize_video_name("missing.mp4", tmp_path) is None           # not present
    assert web.sanitize_video_name("", tmp_path) is None
