#!/usr/bin/env python3
"""Compare a roomscanner TUM path with an attached phone's RTAB-Map database.

The phone trajectory may cover only part of the scanner run.  Clock alignment is
automatic and uses angular-speed magnitude, so the fixed phone/scanner extrinsic
does not need to be calibrated first.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from roomscan.trajectory_compare import compare_rtabmap_to_roomscan


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rtabmap_db", help="phone RTAB-Map .db file")
    parser.add_argument("roomscan_tum", help="roomscanner SLAM .tum trajectory")
    parser.add_argument("--alignment-window-s", type=float, default=30.0,
                        help="leading position-fit window (default: 30 s)")
    parser.add_argument("--final-window-s", type=float, default=60.0,
                        help="trailing error-summary window (default: 60 s)")
    parser.add_argument("--clock-step-s", type=float, default=0.02,
                        help="angular-speed clock-search resolution (default: 0.02 s)")
    parser.add_argument("--json", metavar="PATH",
                        help="also write the structured report to this path")
    args = parser.parse_args(argv)

    report = compare_rtabmap_to_roomscan(
        args.rtabmap_db, args.roomscan_tum,
        alignment_window_s=args.alignment_window_s,
        final_window_s=args.final_window_s,
        clock_step_s=args.clock_step_s,
    )
    text = json.dumps(report, indent=2)
    if args.json:
        Path(args.json).write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if report.get("ok") else 2


if __name__ == "__main__":
    raise SystemExit(main())
