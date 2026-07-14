"""Supervised UI smoke for the panel redesign. Dev box only (Filament needs a
display). Cycles Real-Time/SLAM x first-person/orbit and asserts no crash.

  cd host && .venv\\Scripts\\python.exe tools\\panel_ui_smoke.py <capture.bin>
"""
import sys

from roomscan.panel import _resolve, run


def main():
    argv = ["--panel", "--replay", sys.argv[1]] if len(sys.argv) > 1 else ["--panel"]
    args = _resolve(argv)
    rc = run(args, smoke_ticks=60)     # opens, ticks, tears down cleanly
    print(f"[smoke] exit {rc}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
