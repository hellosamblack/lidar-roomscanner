"""SIGTERM must bound, with a client holding the camera stream open (#226).

The one test in this repo that runs a real `roomscan-web` process, a real
socket and a real signal. It exists because the cheap ones could not see the
defect *by construction*: `test_camera_routes.py` drives `_lifespan` directly,
so it has no uvicorn, no connection and no signal, and it stayed green through a
restart that hung for the full `TimeoutStopSec=90` and was SIGKILLed. AGENTS.md
names this shape exactly -- a fake is defined by what it omits, and async /
transport bugs live in the omissions.

Slow on purpose (~15-25 s per arm: a real interpreter start, a real replay
source, a real camera handshake). The fast lane skips it; CI does not.
"""
from __future__ import annotations

import subprocess  # noqa: F401 - marks this module `subprocess`/`slow` (conftest)
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.camera_shutdown_probe import DEFAULT_BUDGET_S, run_arm  # noqa: E402


def _arm(name: str, budget_s: float = DEFAULT_BUDGET_S) -> dict:
    result = run_arm(name, budget_s=budget_s)
    assert "error" not in result, (
        f"the probe could not set up arm {name!r}: {result.get('error')}. "
        f"A setup failure must never read as a pass -- it exercises nothing.")
    return result


def test_sigterm_exits_promptly_with_no_client_attached():
    """The control arm: proves the harness can observe a NORMAL exit.

    Without it, a probe that always reported a fast exit -- a server that died
    at startup, a SIGTERM sent to the wrong pid -- would make the regression
    test below pass for the wrong reason.
    """
    r = _arm("none")
    assert r["exited"] is True
    assert r["exit_s"] < DEFAULT_BUDGET_S, r
    assert r["via"] == "route", (
        "even with nothing attached the server needed the graceful timeout; "
        "something unrelated to the camera is holding a connection open")
    assert r["camera_state"] == "live", (
        "the arm never got the camera up, so it proved nothing about shutdown")


def test_sigterm_exits_while_a_client_holds_the_camera_stream_open():
    """The regression. Measured before the fix: never (>122 s live, >30 s here);
    after: ~0.2 s.

    Asserted as an INTERVAL against the budget, not as "the process is gone
    eventually" -- the defect was never that it failed to exit, it was that it
    took longer than systemd was willing to wait.
    """
    r = _arm("stream-open")
    assert r["camera_state"] == "live"
    assert r["client_bytes"] > 10_000, (
        f"the client read only {r['client_bytes']} B, so the response may not "
        f"have been streaming at SIGTERM -- the arm did not reproduce the setup")
    assert r["clients_reported"] == 1, (
        "the server did not report the attached viewer; the connection under "
        "test is not the one the server knows about")
    assert r["exited"] is True, (
        f"SIGTERM did not end the server within {r['exit_s']}s with a client "
        f"holding /api/camera/stream.mjpg -- this is the reported hang")
    assert r["exit_s"] < DEFAULT_BUDGET_S, r
    # WHICH mechanism ended it. A plain `exit_s < budget` is not enough: the
    # graceful timeout (3 s) is under the budget (5 s), so removing the
    # route-level gate entirely still "passed" -- the backstop was silently
    # standing in for the fix, and two mutants proved it. This is the assertion
    # that separates "the response let go" from "uvicorn cut the connection".
    assert r["via"] == "route", (
        f"the server exited in {r['exit_s']}s via {r['via']!r} -- that is "
        f"uvicorn's {r['graceful_timeout_s']}s graceful timeout cutting a "
        f"connection the route never released, not the shutdown gate working")
    assert r["exit_s"] < r["graceful_timeout_s"] * 0.5, r


@pytest.mark.slow
def test_sigterm_exits_even_when_the_stream_client_has_stopped_reading():
    """The case the route-level gate alone cannot fix.

    A client that opens the stream and reads nothing leaves the generator
    blocked inside its own `await send(...)`, past uvicorn's write-buffer
    high-water mark -- it is not waiting on the shutdown event and never
    reaches the check. Only `timeout_graceful_shutdown` cuts that, which is why
    the fix is both and not either.
    """
    r = _arm("stream-stalled")
    assert r["client_bytes"] == 0
    assert r["exited"] is True, f"a stalled reader still pins shutdown: {r}"
    assert r["exit_s"] < DEFAULT_BUDGET_S, r
    assert r["via"] == "graceful-timeout", (
        f"exited in {r['exit_s']}s via {r['via']!r} -- too fast to have gone "
        f"through the graceful timeout, so this arm is no longer exercising the "
        f"blocked-in-send path (has the frame size dropped below uvicorn's "
        f"write-buffer high-water mark?)")
