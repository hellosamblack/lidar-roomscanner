"""Tests for busy-vs-missing port classification and the close-holders offer."""
from roomscan.portguard import classify_open_error, offer_to_close_holders


def _winerr(code):
    e = OSError("boom")
    e.winerror = code
    return e


def test_classify_busy():
    assert classify_open_error(PermissionError(13, "Access is denied.")) == "busy"
    assert classify_open_error(_winerr(5)) == "busy"
    assert classify_open_error(Exception("could not open port 'COM15': Access is denied")) == "busy"


def test_classify_missing():
    assert classify_open_error(FileNotFoundError(2, "cannot find")) == "missing"
    assert classify_open_error(_winerr(2)) == "missing"
    assert classify_open_error(Exception(
        "could not open port 'COM99': The system cannot find the file specified.")) == "missing"
    assert classify_open_error(RuntimeError("no scanner serial port found among []")) == "missing"


def test_classify_unknown():
    assert classify_open_error(Exception("something unrelated")) == "unknown"


def test_offer_closes_on_yes():
    killed = []
    ok = offer_to_close_holders(
        exclude_pid=1,
        input_fn=lambda _prompt: "y",
        out=lambda *_: None,
        list_fn=lambda exclude_pid=None: [(123, "python -m roomscan.viewer"),
                                          (456, "python -m roomscan.panel")],
        kill_fn=lambda pids: killed.extend(pids) or pids,
    )
    assert ok is True
    assert killed == [123, 456]


def test_offer_declines_on_no():
    killed = []
    ok = offer_to_close_holders(
        input_fn=lambda _prompt: "n",
        out=lambda *_: None,
        list_fn=lambda exclude_pid=None: [(123, "python -m roomscan.viewer")],
        kill_fn=lambda pids: killed.extend(pids) or pids,
    )
    assert ok is False
    assert killed == []          # nothing killed when the user declines


def test_offer_no_holders_found():
    ok = offer_to_close_holders(
        input_fn=lambda _prompt: "y",   # must not even be asked
        out=lambda *_: None,
        list_fn=lambda exclude_pid=None: [],
        kill_fn=lambda pids: pids,
    )
    assert ok is False


def test_offer_eof_treated_as_no():
    def _raise(_prompt):
        raise EOFError

    killed = []
    ok = offer_to_close_holders(
        input_fn=_raise,
        out=lambda *_: None,
        list_fn=lambda exclude_pid=None: [(123, "x")],
        kill_fn=lambda pids: killed.extend(pids) or pids,
    )
    assert ok is False
    assert killed == []
