"""Offline replay of a .bin capture file into Rerun .rrd observability recording (Issue #136).

Usage:
    host/.venv/bin/python host/tools/rerun_export.py captures/e2e_p2.bin
    host/.venv/bin/python host/tools/rerun_export.py <capture.bin> -o results/rerun/custom.rrd
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from roomscan.decoder import StreamDecoder
from roomscan.deproject import Deprojector
from roomscan.pipeline import TransformStage
from roomscan.protocol import FrameType, StreamId
from roomscan.rerun_sidecar import MockWriter, NativeRerunWriter, RerunSidecar, RerunSidecarConfig
from roomscan.sensors import SensorState
from roomscan.sources import FileSource, pump


def replay_capture_to_rerun(
    capture_path: Path | str,
    output_rrd: Path | str | None = None,
    rate_limit_hz: float = 0.0,
    use_mock: bool = False,
) -> dict:
    capture_path = Path(capture_path)
    if not capture_path.is_file():
        raise FileNotFoundError(f"Capture file not found: {capture_path}")

    stem = capture_path.stem
    out_dir = Path("results/rerun")
    out_dir.mkdir(parents=True, exist_ok=True)
    if output_rrd is None:
        output_rrd = out_dir / f"{stem}.rrd"
    else:
        output_rrd = Path(output_rrd)

    config = RerunSidecarConfig(
        enabled=True,
        output_dir=str(output_rrd.parent),
        recording_name=stem,
        rate_limit_hz=rate_limit_hz,
    )

    writer = None
    if use_mock:
        writer = MockWriter()
    else:
        try:
            writer = NativeRerunWriter(
                recording_id=stem,
                output_path=str(output_rrd),
            )
        except Exception as exc:
            print(f"[rerun_export] Notice: rerun-sdk not available ({exc}), using MockWriter", file=sys.stderr)
            writer = MockWriter()

    deprojector = Deprojector(54, 42)
    sidecar = RerunSidecar(config=config, writer=writer, deprojector=deprojector)
    sidecar.start()

    source = FileSource(capture_path)
    decoder = StreamDecoder()
    stage = TransformStage(outputs=("depth", "reflectance", "confidence", "ambient"))
    sensor_state = SensorState()

    frames_processed = 0
    t0 = time.time()

    try:
        for frame in pump(source, decoder):
            hdr = frame.header
            frames_processed += 1

            if hdr.frame_type == FrameType.DATA:
                sensor_state.feed(frame)

                if hdr.stream_id in (StreamId.RAW_3DMD, StreamId.DEPTH_ZF32, StreamId.CALIB):
                    res = stage.feed(frame)
                    if res is not None:
                        f_hdr, outputs = res
                        pts = None
                        if "depth" in outputs:
                            try:
                                pts = deprojector(outputs["depth"])
                            except Exception:
                                pass
                        sidecar.log_frame(f_hdr, outputs, points=pts)
                        quat = sensor_state.latest_quat()
                        fused_quat = sensor_state.fused_quat()
                        sidecar.log_pose(f_hdr.t_us, f_hdr.seq, quat=quat, fused_quat=fused_quat)

                elif hdr.stream_id in (StreamId.IMU_QUAT, StreamId.ENV, StreamId.IMU_RAW):
                    quat = sensor_state.latest_quat()
                    env = sensor_state.latest_env()
                    heading = sensor_state.heading_deg() if hasattr(sensor_state, "heading_deg") else None
                    sidecar.log_imu(hdr.t_us, hdr.seq, quat=quat, env=env, heading_deg=heading)

            elif hdr.frame_type == FrameType.EVENT:
                sidecar.log_event(hdr.t_us, hdr.seq, f"Event stream {hdr.stream_id}", code=hdr.flags)

    finally:
        source.close()
        sidecar.stop(reason="replay_eof")

    elapsed = time.time() - t0
    status = sidecar.status()
    status["frames_processed"] = frames_processed
    status["elapsed_s"] = elapsed
    status["capture_path"] = str(capture_path)
    return status


def main() -> int:
    parser = argparse.ArgumentParser(description="Export a roomscan .bin capture to a Rerun .rrd recording")
    parser.add_argument("capture", type=str, help="Path to .bin capture file")
    parser.add_argument("-o", "--output", type=str, default=None, help="Output .rrd file path")
    parser.add_argument("--rate-limit", type=float, default=0.0, help="Rate limit in Hz (0 for unmetered fast export)")
    parser.add_argument("--mock", action="store_true", help="Force MockWriter instead of NativeRerunWriter")
    args = parser.parse_args()

    try:
        status = replay_capture_to_rerun(
            capture_path=args.capture,
            output_rrd=args.output,
            rate_limit_hz=args.rate_limit,
            use_mock=args.mock,
        )
        print(f"Export completed in {status['elapsed_s']:.2f}s: {status['samples_logged']} samples logged, "
              f"{status['queue_drops']} drops, {status['fault_count']} faults.")
        return 0
    except Exception as exc:
        print(f"Error during export: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
