"""Live verification of the #219 browser idle reaper, against a REAL headless Chrome.

    python tools/browser_reap_probe.py [--cdp] [--park S] [--stop S]

The unit tests drive a fake backend: they prove the state machine and nothing at all
about CPU. The observable that matters here is on the far side of the browser driver --
jiffies burned by the chrome process tree, and sockets it holds to roomscan-web. So
this loads the live UI, measures both, idles past the park TTL, measures again, then
proves the restored page is the SAME page (title + canvas size) rather than a blank one
that merely reports success.

Both findings in `docs/mcp-server.md` -> "The browser is warm, not immortal" were
invisible to the unit tests and came from this probe: the reaper cancelling its own
teardown midway, and `websockets` killing the CDP socket with a keepalive ping timeout
because Chrome cannot pong while SwiftShader saturates a dozen cores. Re-run it after a
Chrome, Playwright or `websockets` bump -- the numbers in that doc came from here.

Needs `roomscan-web` up on :8000. Takes ~2 min. Deliberately NOT an MCP tool: it must
own its browser, and the MCP server's browser singleton is the thing under test.
"""
import argparse
import asyncio
import os
import sys
import time

_ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
_ap.add_argument("--cdp", action="store_true",
                 help="exercise the raw-CDP fallback instead of Playwright")
_ap.add_argument("--park", default="12", help="park TTL, seconds")
_ap.add_argument("--stop", default="45", help="stop TTL, seconds")
_ap.add_argument("--port", type=int, default=9222, help="CDP debug port (--cdp only)")
_ap.add_argument("--url", default="http://localhost:8000/")
ARGS = _ap.parse_args()

os.environ["ROOMSCAN_MCP_BROWSER_PARK_S"] = ARGS.park
os.environ["ROOMSCAN_MCP_BROWSER_STOP_S"] = ARGS.stop
os.environ["ROOMSCAN_MCP_BROWSER_RESTORE_SETTLE_S"] = "8"

sys.path.insert(0, "src")
from roomscan.mcp_server import session as S  # noqa: E402

PAGE = ARGS.url


def descendants(root):
    kids = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/stat") as f:
                fields = f.read().rsplit(")", 1)[1].split()
            kids.setdefault(int(fields[1]), []).append(int(pid))
        except OSError:
            continue
    out, stack = [], [root]
    while stack:
        p = stack.pop()
        for c in kids.get(p, []):
            out.append(c)
            stack.append(c)
    return out


def jiffies(pids):
    total = 0
    for pid in pids:
        try:
            with open(f"/proc/{pid}/stat") as f:
                fields = f.read().rsplit(")", 1)[1].split()
            total += int(fields[11]) + int(fields[12])  # utime + stime
        except OSError:
            pass
    return total


HZ = os.sysconf("SC_CLK_TCK")


async def cpu_percent(pids, window=5.0):
    a = jiffies(pids)
    await asyncio.sleep(window)
    b = jiffies(pids)
    return round((b - a) / HZ / window * 100, 1)


async def main():
    b = S.CdpSession(port=ARGS.port) if ARGS.cdp else S.PlaywrightSession()
    print(f"backend={type(b).__name__} park={b.park_after_s}s stop={b.stop_after_s}s")

    await b.goto(PAGE, settle=10.0)
    chrome = descendants(os.getpid())
    print(f"loaded url={b.url}  chrome pids={len(chrome)}")

    title = await b.evaluate("document.title")
    canvas = await b.evaluate(
        "(()=>{const c=document.querySelector('canvas');"
        "return c ? c.width+'x'+c.height : 'no-canvas';})()")
    print(f"  title={title['value']!r} canvas={canvas['value']!r}")

    loaded = await cpu_percent(chrome, 6.0)
    print(f"\nCPU while LOADED  : {loaded}%")

    print(f"\nidling past the {b.park_after_s}s park TTL ...")
    while not b.parked:
        await asyncio.sleep(1.0)
        if b.idle_s > b.park_after_s + 40:
            print("!! never parked"); break
    print(f"parked={b.parked} after idle_s={b.idle_s:.0f} (url still {b.url}) park_error={b.last_park_error}")

    parked = await cpu_percent(chrome, 6.0)
    print(f"CPU while PARKED  : {parked}%")
    if loaded > 0:
        print(f"=> reduction: {round((1 - parked / loaded) * 100)}%")

    print("\nrestoring via evaluate() ...")
    t0 = time.monotonic()
    title2 = await b.evaluate("document.title")
    canvas2 = await b.evaluate(
        "(()=>{const c=document.querySelector('canvas');"
        "return c ? c.width+'x'+c.height : 'no-canvas';})()")
    print(f"  took {time.monotonic()-t0:.1f}s  title={title2['value']!r} "
          f"canvas={canvas2['value']!r}  restores={b.restore_count}")
    ok_restore = title2["value"] == title["value"] and canvas2["value"] == canvas["value"]
    print(f"  RESTORE MATCHES ORIGINAL PAGE: {ok_restore}")

    print(f"\nidling past the {b.stop_after_s}s stop TTL ...")
    deadline = time.monotonic() + b.stop_after_s + 60
    while b._alive() and time.monotonic() < deadline:
        await asyncio.sleep(2.0)
    alive_pids = [p for p in chrome if os.path.exists(f"/proc/{p}")]
    print(f"browser alive={b._alive()} stops={b.stop_count} "
          f"surviving chrome pids={len(alive_pids)}")

    await b.stop()
    print("\nfinal status:", b.status())


asyncio.run(main())
