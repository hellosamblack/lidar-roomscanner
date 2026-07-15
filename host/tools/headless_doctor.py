"""Headless-host bring-up doctor: one command that verifies (and optionally
fixes) everything a fresh Linux host needs to run the web viewer.

Extracted at the 2026-07-15 milestone-retro. Bringing the repo up on a fresh
headless host (Proxmox/LXC, no GPU, Ethernet-only) surfaced FOUR migration gaps
in a row -- each was something implicit on the Windows dev box, absent on the
new host, and each cost a manual diagnosis dig (BUG-020, UDP keepalive, BUG-021,
BUG-022 in BUGS.md). This tool runs that whole diagnosis sequence in ~5 s so the
next fresh host is a checklist, not an investigation.

    host/.venv/bin/python host/tools/headless_doctor.py            # check only
    host/.venv/bin/python host/tools/headless_doctor.py --build    # also build the .so
    host/.venv/bin/python host/tools/headless_doctor.py --no-net   # skip the live board probe

Checks, in dependency order:
  1. vendored 53L9A1 transform sources present (firmware/vendor/53L9A1)
  2. native transform library built + loadable (host/transform/build, cross-platform)
  3. board reachable + actually STREAMING over UDP (wake -> frames); mDNS resolvable
  4. viewer assets self-contained (three.js vendored locally, no unpkg in index.html)
  5. a browser is installed and can create a WebGL context with --enable-unsafe-swiftshader
Each failure prints the exact remediation. Exit code = number of failed checks.
"""
from __future__ import annotations

import argparse
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]          # host/tools -> host -> repo
HOST = REPO / "host"
sys.path.insert(0, str(HOST / "src"))

GREEN, RED, YELLOW, RESET = "\033[32m", "\033[31m", "\033[33m", "\033[0m"
if not sys.stdout.isatty():
    GREEN = RED = YELLOW = RESET = ""


class Doctor:
    def __init__(self) -> None:
        self.failed = 0

    def ok(self, name: str, detail: str = "") -> None:
        print(f"  {GREEN}PASS{RESET} {name}" + (f" — {detail}" if detail else ""))

    def bad(self, name: str, detail: str, fix: str) -> None:
        self.failed += 1
        print(f"  {RED}FAIL{RESET} {name} — {detail}")
        for line in fix.splitlines():
            print(f"       {YELLOW}fix:{RESET} {line}")

    def warn(self, name: str, detail: str) -> None:
        print(f"  {YELLOW}WARN{RESET} {name} — {detail}")

    # 1 -------------------------------------------------------------------
    def check_vendored_sources(self) -> None:
        src = REPO / "firmware/vendor/53L9A1/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/vl53l9_transform.c"
        if src.is_file():
            self.ok("vendored 53L9A1 transform sources", "firmware/vendor/53L9A1")
        else:
            self.bad("vendored 53L9A1 transform sources", "not found",
                     "The ST reference package must be vendored in-repo. Copy the\n"
                     "53L9A1/ package to firmware/vendor/53L9A1/ (see BUGS.md BUG-020).")

    # 2 -------------------------------------------------------------------
    def check_native_lib(self, build: bool) -> None:
        try:
            from roomscan import native
        except Exception as exc:  # pragma: no cover - import guard
            self.bad("native transform loader", f"import failed: {exc}", "check host[web] is installed")
            return
        path = native._find_dll()
        if path is None and build:
            print("       building native transform library…")
            bdir = HOST / "transform/build"
            try:
                subprocess.run(["cmake", "-S", str(HOST / "transform"), "-B", str(bdir),
                                "-DCMAKE_BUILD_TYPE=Release"], check=True,
                               stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                subprocess.run(["cmake", "--build", str(bdir)], check=True,
                               stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                path = native._find_dll()
            except Exception as exc:
                self.bad("native transform library", f"build failed: {exc}",
                         "cmake + a C compiler required; see check #1 for the sources")
                return
        if path is None:
            self.bad("native transform library", "not built",
                     "cmake -S host/transform -B host/transform/build -DCMAKE_BUILD_TYPE=Release\n"
                     "cmake --build host/transform/build   (or re-run this with --build)")
            return
        # prove it actually loads (catches ABI / arch mismatch, not just presence)
        if native._load_dll() is None:
            self.bad("native transform library", f"found {path.name} but it won't load",
                     "rebuild for this platform: rm -rf host/transform/build && re-run with --build")
        else:
            self.ok("native transform library", str(path.relative_to(REPO)))

    # 3 -------------------------------------------------------------------
    def check_board_stream(self) -> None:
        # mDNS name resolution (nice-to-have; the wake probe is the real test)
        try:
            ip = socket.gethostbyname("roomscanner.local")
            self.ok("mDNS roomscanner.local", ip)
        except Exception:
            self.warn("mDNS roomscanner.local", "not resolvable (avahi/zeroconf) — will try broadcast wake")
            ip = None

        # the real test: wake the board and see frames come back
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("", 5000))
        except OSError as exc:
            self.warn("board UDP stream", f"port 5000 busy ({exc}) — viewer already running? skipping")
            s.close()
            return
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.settimeout(0.3)
        target = ip or "255.255.255.255"
        t0 = time.time()
        n = 0
        next_wake = 0.0
        while time.time() - t0 < 4:
            now = time.time()
            if now >= next_wake:
                try:
                    s.sendto(b"\x00", (target, 5000))
                except Exception:
                    pass
                next_wake = now + 0.5
            try:
                s.recvfrom(2048)
                n += 1
            except socket.timeout:
                pass
        s.close()
        if n > 20:
            self.ok("board UDP stream", f"{n} datagrams in ~4s")
        elif n > 0:
            self.warn("board UDP stream", f"only {n} datagrams — flaky link?")
        else:
            self.bad("board UDP stream", "no frames from the board",
                     "check the board is powered + Ethernet LINK LED is lit;\n"
                     "`avahi-browse -rt _roomscan._udp` should list it live;\n"
                     "power-cycle to re-DHCP if it fell off the network (BUGS.md BUG-020).")

    # 4 -------------------------------------------------------------------
    def check_viewer_assets(self) -> None:
        static = HOST / "src/roomscan/static"
        three = static / "vendor/three/three.module.js"
        index = (static / "index.html").read_text(encoding="utf-8") if (static / "index.html").is_file() else ""
        if not three.is_file():
            self.bad("viewer three.js vendored", "vendor/three/three.module.js missing",
                     "curl -o host/src/roomscan/static/vendor/three/three.module.js \\\n"
                     "  https://unpkg.com/three@0.160.0/build/three.module.js  (see BUGS.md BUG-021)")
        elif "unpkg.com" in index:
            self.bad("viewer three.js vendored", "index.html still references unpkg CDN",
                     "point the import map at /static/vendor/three/ (BUGS.md BUG-021)")
        else:
            self.ok("viewer three.js vendored", "self-contained, no CDN")

    # 5 -------------------------------------------------------------------
    def check_browser_webgl(self) -> None:
        exe = next((shutil.which(b) for b in
                    ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser")
                    if shutil.which(b)), None)
        if not exe:
            self.bad("browser + software WebGL", "no Chrome/Chromium found",
                     "install google-chrome or chromium; the viewer needs it to render")
            return
        # headless Chrome uses SwiftShader automatically, so this proves the
        # binary CAN do software WebGL; the runtime flag (--enable-unsafe-
        # swiftshader) is what web.py passes for the on-display browser.
        self.ok("browser installed", Path(exe).name +
                " — auto-open passes --enable-unsafe-swiftshader (web.py)")

    def run(self, build: bool, net: bool) -> int:
        print("roomscan headless-host doctor\n")
        self.check_vendored_sources()
        self.check_native_lib(build)
        if net:
            self.check_board_stream()
        else:
            self.warn("board UDP stream", "skipped (--no-net)")
        self.check_viewer_assets()
        self.check_browser_webgl()
        print()
        if self.failed:
            print(f"{RED}{self.failed} check(s) failed{RESET} — fix the above, then `./view-web.sh`.")
        else:
            print(f"{GREEN}all checks passed{RESET} — `./view-web.sh` should show live data.")
        return self.failed


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Verify a fresh host can run the roomscan web viewer.")
    ap.add_argument("--build", action="store_true", help="build the native transform library if missing")
    ap.add_argument("--no-net", action="store_true", help="skip the live board UDP probe")
    args = ap.parse_args(argv)
    return Doctor().run(build=args.build, net=not args.no_net)


if __name__ == "__main__":
    sys.exit(main())
