"""BUG-050: session_message subtracted a time.monotonic() start stamp from a
time.time() 'now' and reported a 90 s take as 1.78e9 s."""
from roomscan.domain import WallTime, elapsed_s
from roomscan.server.session import SessionController


def recording_elapsed(ctrl: SessionController, now: WallTime) -> float:
    return elapsed_s(now, ctrl._record_started)   # WallTime where a HostMono is required


def recording_elapsed_raw(ctrl: SessionController, now: WallTime) -> float:
    # ESCAPE HATCH: Python cannot forbid float - float. Pyright accepts this line.
    return now - ctrl._record_started
