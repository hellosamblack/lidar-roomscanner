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
from .pipeline import TransformStage
from .protocol import FLAG_DROPPED, FrameType, ProtocolError, parse_event
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


def _reader(source, decoder, slot: queue.Queue, stats: Stats, record, fault: dict,
            min_interval: float = 0.0, stage: TransformStage | None = None):
    if stage is None:
        stage = TransformStage()
    last = 0.0
    last_paced_seq = None
    try:
        for frame in pump(source, decoder, record_path=record):
            if frame.header.frame_type == FrameType.EVENT:
                try:
                    code, detail, msg = parse_event(frame.payload)
                    print(f"\n[device event] code={code} detail={detail} {msg}")
                except ProtocolError:
                    print(f"\n[device event] undecodable payload ({len(frame.payload)} B)")
                continue
            if frame.header.frame_type != FrameType.DATA:
                continue
            result = stage.feed(frame)          # RAW->transformed depth, DEPTH->passthrough,
            if result is None:                   # CALIB/unknown -> None (stays silent)
                continue
            header, depth = result
            stats.update(header)
            # Paced replay: don't drain a recording at decode speed. Pace per SENSOR
            # frame (seq change), not per stage result — dual-stream recordings yield
            # two results per sensor frame (RAW-transformed + DEPTH-passthrough, same
            # seq); pacing each would halve the effective frame rate.
            if min_interval > 0.0 and header.seq != last_paced_seq:
                wait = last + min_interval - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                last = time.monotonic()
                last_paced_seq = header.seq
            try:
                slot.get_nowait()          # latest-wins: drop stale frame
            except queue.Empty:
                pass
            slot.put((header, depth))
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
    ap.add_argument("--replay-fps", type=float, default=0.0,
                    help="pace file replay at N fps (0 = as fast as it decodes)")
    args = ap.parse_args(argv)

    import open3d as o3d   # deferred: heavy import

    source = FileSource(args.replay) if args.replay else SerialSource(args.port, args.baud)
    decoder = StreamDecoder()
    stats = Stats()
    stage = TransformStage()   # cheap to construct; only touches the DLL on first CALIB frame
    slot: queue.Queue = queue.Queue(maxsize=1)
    fault: dict = {}
    min_interval = 1.0 / args.replay_fps if (args.replay and args.replay_fps > 0) else 0.0
    threading.Thread(target=_reader,
                     args=(source, decoder, slot, stats, args.record, fault, min_interval, stage),
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
            item = slot.get(timeout=0.02)
        except queue.Empty:
            item = None
        if fault:
            print(f"\nreader stopped: {fault['error']!r}")
            break
        if item is not None:
            _hdr, depth = item
            h, w = depth.shape
            if deproj is None:
                deproj = Deprojector(w, h, args.fov_h, args.fov_v)
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
            line = (f"\r{fps:5.1f} fps | frames {stats.frames} | seq gaps {stats.seq_gaps} "
                    f"| drops {stats.dropped_flags} | crc fail {decoder.crc_failures} "
                    f"| skipped {decoder.bytes_skipped} B | raw {stage.raw_transformed}")
            if stage.raw_skipped_awaiting_calib:
                line += f" | raw-skip {stage.raw_skipped_awaiting_calib}"
            print(line + " ", end="", flush=True)
            t_stat, f_stat = now, shown
    vis.destroy_window()
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
