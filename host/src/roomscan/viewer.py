"""Live point-cloud viewer. Reader thread: source -> decoder -> latest-frame slot;
main thread: Open3D non-blocking render loop + 1 Hz stats line."""
from __future__ import annotations

import argparse
import queue
import sys
import threading
import time

import numpy as np

from .colors import turbo
from .decoder import StreamDecoder
from .deproject import Deprojector
from .protocol import FLAG_DROPPED, FrameType, ProtocolError, StreamId, parse_event
from .sources import FileSource, SerialSource, pump


class Stats:
    def __init__(self):
        self.frames = 0
        self.seq_gaps = 0
        self.dropped_flags = 0
        self._last_seq = None

    def update(self, header):
        self.frames += 1
        if header.flags & FLAG_DROPPED:
            self.dropped_flags += 1
        if self._last_seq is not None and header.seq > self._last_seq + 1:
            self.seq_gaps += header.seq - self._last_seq - 1
        self._last_seq = header.seq


def _reader(source, decoder, slot: queue.Queue, stats: Stats, record, fault: dict):
    try:
        for frame in pump(source, decoder, record_path=record):
            if frame.header.frame_type == FrameType.EVENT:
                try:
                    code, detail, msg = parse_event(frame.payload)
                    print(f"\n[device event] code={code} detail={detail} {msg}")
                except ProtocolError:
                    print(f"\n[device event] undecodable payload ({len(frame.payload)} B)")
                continue
            if frame.header.frame_type != FrameType.DATA or frame.header.stream_id != StreamId.DEPTH_ZF32:
                continue
            stats.update(frame.header)
            try:
                slot.get_nowait()          # latest-wins: drop stale frame
            except queue.Empty:
                pass
            slot.put(frame)
    except Exception as exc:  # surface, don't vanish: main loop reports and exits
        fault["error"] = exc


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="roomscan-view")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--replay")
    ap.add_argument("--record")
    ap.add_argument("--fov-h", type=float, default=60.0)
    ap.add_argument("--fov-v", type=float, default=45.0)
    args = ap.parse_args(argv)

    import open3d as o3d   # deferred: heavy import

    source = FileSource(args.replay) if args.replay else SerialSource(args.port, args.baud)
    decoder = StreamDecoder()
    stats = Stats()
    slot: queue.Queue = queue.Queue(maxsize=1)
    fault: dict = {}
    threading.Thread(target=_reader, args=(source, decoder, slot, stats, args.record, fault),
                     daemon=True).start()

    vis = o3d.visualization.Visualizer()
    vis.create_window("roomscan", width=1280, height=800)
    opt = vis.get_render_option()
    opt.point_size = 3.0
    opt.background_color = np.asarray([0.05, 0.05, 0.08])
    pcd = o3d.geometry.PointCloud()
    added = False
    deproj = None
    shown = 0
    t_stat = time.monotonic()
    f_stat = 0

    while vis.poll_events():
        try:
            frame = slot.get(timeout=0.02)
        except queue.Empty:
            frame = None
        if fault:
            print(f"\nreader stopped: {fault['error']!r}")
            break
        if frame is not None:
            h, w = frame.header.height, frame.header.width
            if deproj is None:
                deproj = Deprojector(w, h, args.fov_h, args.fov_v)
            depth = np.frombuffer(frame.payload, dtype="<f4").reshape(h, w)
            pts = deproj(depth)
            pcd.points = o3d.utility.Vector3dVector(pts)
            if len(pts):
                zn = (pts[:, 2] - pts[:, 2].min()) / max(float(np.ptp(pts[:, 2])), 1e-6)
                pcd.colors = o3d.utility.Vector3dVector(turbo(zn))
            else:
                pcd.colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))
            if not added:
                vis.add_geometry(pcd)
                added = True
                vis.reset_view_point(True)
            else:
                vis.update_geometry(pcd)
            shown += 1
        vis.update_renderer()
        now = time.monotonic()
        if now - t_stat >= 1.0:
            fps = (shown - f_stat) / (now - t_stat)
            print(f"\r{fps:5.1f} fps | frames {stats.frames} | seq gaps {stats.seq_gaps} "
                  f"| drops {stats.dropped_flags} | crc fail {decoder.crc_failures} "
                  f"| skipped {decoder.bytes_skipped} B ",
                  end="", flush=True)
            t_stat, f_stat = now, shown
    vis.destroy_window()
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
