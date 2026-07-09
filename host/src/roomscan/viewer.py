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
from .config import ViewerConfig, apply_config_defaults
from .control import CommandClient
from .decoder import StreamDecoder
from .deproject import Deprojector
from .pipeline import TransformStage
from .protocol import CommandCode, FLAG_DROPPED, FrameType, ProtocolError, parse_event
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


class CommandKeyState:
    """Fire-and-forget command dispatch for the viewer's key bindings.

    Each key press hands its command off to a short-lived worker thread so
    the render loop never blocks on ``CommandClient.send()`` (up to its 2 s
    timeout) -- see ``roomscan.control``'s thread-contract note: send() must
    never run on the frame-feeding thread. A single in-flight-guard flag
    rejects a second press while one command is still pending (prints
    "busy", drops the new press) instead of queuing it.

    `client is None` means replay mode (no live device to command): every
    dispatch just prints that commands aren't available and returns.
    """

    def __init__(self, client: CommandClient | None):
        self.client = client
        self._lock = threading.Lock()
        self._busy = False

    def dispatch(self, cmd: int, param: int, label: str) -> None:
        if self.client is None:
            print(f"\n[cmd] {label} -> not available in replay")
            return
        with self._lock:
            if self._busy:
                print(f"\n[cmd] {label} -> busy, command already in flight")
                return
            self._busy = True

        def worker() -> None:
            try:
                result, applied = self.client.send(cmd, param)
                print(f"\n[cmd] {label} -> {result.name} applied={applied}")
            except TimeoutError as exc:
                print(f"\n[cmd] {label} -> TIMEOUT {exc}")
            except Exception as exc:  # e.g. SerialException on a dead port: report, don't traceback
                print(f"\n[cmd] {label} -> ERROR {exc!r}")
            finally:
                with self._lock:
                    self._busy = False

        threading.Thread(target=worker, daemon=True).start()


def _reader(source, decoder, slot: queue.Queue, stats: Stats, record, fault: dict,
            min_interval: float = 0.0, stage: TransformStage | None = None,
            client: CommandClient | None = None):
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
            if frame.header.frame_type == FrameType.ACK:
                # Command-channel replies: matched against a pending
                # CommandClient.send() by token, never rendered. Live mode
                # only -- client is None in replay, so ACKs (which can't
                # occur in a recording anyway) would just fall through below.
                if client is not None:
                    client.offer(frame)
                continue
            if frame.header.frame_type != FrameType.DATA:
                continue
            result = stage.feed(frame)          # RAW->transformed outputs dict, DEPTH->{"depth": ...},
            if result is None:                   # CALIB/unknown -> None (stays silent)
                continue
            header, outputs = result
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
            slot.put((header, outputs))
    except Exception as exc:  # surface, don't vanish: main loop reports and exits
        fault["error"] = exc


def _build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="roomscan-view")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--replay")
    ap.add_argument("--record")
    # fov-h/fov-v/replay-fps/color/port default to None (argparse's "not
    # passed" sentinel): apply_config_defaults() below fills anything still
    # None from the loaded roomscan.toml, which itself already carries the
    # built-in defaults for anything absent from the file. Net priority:
    # CLI flag > config file > built-in default.
    ap.add_argument("--fov-h", type=float, default=None)
    ap.add_argument("--fov-v", type=float, default=None)
    ap.add_argument("--replay-fps", type=float, default=None,
                    help="pace file replay at N fps (0 = as fast as it decodes)")
    ap.add_argument("--color", choices=("depth", "reflectance", "confidence"), default=None,
                    help="colorize the cloud by z-depth (default) or by an aux transform plane")
    ap.add_argument("--save-config", action="store_true",
                    help="persist the effective color/fov/replay-fps/port settings to roomscan.toml")
    return ap


def resolve_args(argv=None, config_path=None) -> argparse.Namespace:
    """Parse argv, then merge in roomscan.toml for any flag left at its None
    sentinel, optionally persisting the result via --save-config. Pure aside
    from the config file's own read/write -- no serial port, no open3d
    import -- the testable seam for config load/save/priority."""
    args = _build_arg_parser().parse_args(argv)
    cfg = ViewerConfig.load(config_path)
    apply_config_defaults(args, cfg)
    if args.save_config:
        effective = ViewerConfig(color=args.color, fov_h=args.fov_h, fov_v=args.fov_v,
                                  replay_fps=args.replay_fps, port=args.port)
        saved_path = effective.save(config_path)
        print(f"saved config to {saved_path}")
    return args


def main(argv=None) -> int:
    args = resolve_args(argv)

    import open3d as o3d   # deferred: heavy import

    source = FileSource(args.replay) if args.replay else SerialSource(args.port, args.baud)
    # Command channel rides the SAME open port: only meaningful for a live
    # SerialSource (replay has no device to command, so client stays None
    # and CommandKeyState prints "not available in replay" for every key).
    client = CommandClient(source.write) if isinstance(source, SerialSource) else None
    cmd_state = CommandKeyState(client)
    decoder = StreamDecoder()
    stats = Stats()
    # Always need "depth" for point positions; add the color channel if it's a different plane.
    stage_outputs = ("depth",) if args.color == "depth" else ("depth", args.color)
    stage = TransformStage(outputs=stage_outputs)   # cheap to construct; only touches the DLL on first CALIB frame
    slot: queue.Queue = queue.Queue(maxsize=1)
    fault: dict = {}
    min_interval = 1.0 / args.replay_fps if (args.replay and args.replay_fps > 0) else 0.0
    threading.Thread(target=_reader,
                     args=(source, decoder, slot, stats, args.record, fault, min_interval, stage, client),
                     daemon=True).start()

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window("roomscan", width=1280, height=800)

    def _key(cmd: int, param: int, label: str):
        def handler(_vis) -> bool:
            cmd_state.dispatch(cmd, param, label)
            return False   # result prints asynchronously; no redraw needed here
        return handler

    vis.register_key_callback(ord("P"), _key(CommandCode.PING, 0, "ping"))
    vis.register_key_callback(ord("C"), _key(CommandCode.SEND_CALIB, 0, "calib"))
    vis.register_key_callback(ord("R"), _key(CommandCode.REINIT, 0, "reinit"))
    vis.register_key_callback(ord("1"), _key(CommandCode.SET_USECASE, 0, "usecase 0"))
    vis.register_key_callback(ord("2"), _key(CommandCode.SET_USECASE, 1, "usecase 1"))

    def _print_keymap(_vis=None) -> bool:
        mode = "live" if args.replay is None else "replay (device keys inactive)"
        print(
            "\n=== roomscan viewer controls ===\n"
            f"  mode: {mode}\n"
            "  mouse: left-drag orbit | ctrl/middle-drag pan | wheel zoom\n"
            "  P ping device    C request calibration    R sensor reinit\n"
            "  1 usecase AR_RANGE (~32 fps)    2 usecase AR_PRECISION (~28 fps)\n"
            "  H this help    (stats print here once per second; close window to exit)\n"
            "================================"
        )
        return False

    vis.register_key_callback(ord("H"), _print_keymap)
    _print_keymap()  # discoverability: show the keymap banner at launch

    opt = vis.get_render_option()
    opt.point_size = 3.0
    opt.background_color = np.asarray([0.05, 0.05, 0.08])
    pcd = o3d.geometry.PointCloud()
    added = False
    deproj = None
    shown = 0
    fallback_warned = False
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
            _hdr, outputs = item
            depth = outputs["depth"]
            h, w = depth.shape
            if deproj is None:
                deproj = Deprojector(w, h, args.fov_h, args.fov_v)
            pts = deproj(depth)
            pcd.points = o3d.utility.Vector3dVector(pts)
            if len(pts):
                # Color source: z-depth (default) or an aux plane (reflectance/confidence),
                # normalized per-frame like the z case; same flat-plane divide guard either way.
                # The aux plane isn't deprojected -- it shares depth's (h, w) shape, so it's
                # filtered by the identical validity mask Deprojector used internally to keep
                # per-point alignment with `pts`.
                plane = None if args.color == "depth" else outputs.get(args.color)
                if plane is not None:
                    valid = np.isfinite(depth) & (depth > 0.0) & (depth < deproj.max_range_mm)
                    vals = plane[valid].astype(np.float64, copy=False)
                else:
                    if args.color != "depth" and not fallback_warned:
                        print(f"\n[viewer] no '{args.color}' plane in this stream — "
                              "coloring by depth instead", file=sys.stderr)
                        fallback_warned = True
                    vals = pts[:, 2]
                vn = (vals - vals.min()) / max(float(np.ptp(vals)), 1e-6)
                pcd.colors = o3d.utility.Vector3dVector(turbo(vn))
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
