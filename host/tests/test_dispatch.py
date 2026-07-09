"""Tests for CommandDispatcher (control.py) -- the client-agnostic,
callback-driven generalization of viewer.CommandKeyState. Phase 3.5 wires
this into both the viewer's key bindings and the GUI control panel so they
share one fire-and-forget dispatcher; CommandKeyState itself is untouched
here (see viewer.py) -- these tests only exercise the new class.
"""
import queue
import threading
import time

from roomscan.control import CommandDispatcher
from roomscan.protocol import ResultCode


class FakeClient:
    """Controllable stand-in for CommandClient.send() -- no serial port.

    By default returns immediately with `result`. Set `block=True` to make
    send() wait on `release` (fired by the test) before returning/raising,
    so "still in flight" is driven by an Event rather than a sleep.
    """

    def __init__(self, result=(ResultCode.OK, 1), raise_exc: Exception | None = None, block=False):
        self.calls = []
        self._result = result
        self._raise_exc = raise_exc
        self.release = threading.Event()
        self.started = threading.Event()
        self._block = block

    def send(self, cmd, param=0, timeout=2.0):
        self.calls.append((cmd, param))
        self.started.set()
        if self._block:
            self.release.wait(timeout=5.0)
        if self._raise_exc is not None:
            raise self._raise_exc
        return self._result


def _wait_for(predicate, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_dispatch_replay_mode_reports_not_available_no_thread_spawned():
    messages: queue.Queue = queue.Queue()
    dispatcher = CommandDispatcher(client=None, on_message=messages.put)
    before = threading.active_count()

    dispatcher.dispatch(1, 0, "ping")

    msg = messages.get(timeout=1.0)
    assert msg == "ping -> not available in replay"
    assert threading.active_count() == before  # no worker thread spawned
    assert dispatcher.busy is False


def test_dispatch_success_reports_result_and_clears_busy():
    messages: queue.Queue = queue.Queue()
    client = FakeClient(result=(ResultCode.OK, 3))
    dispatcher = CommandDispatcher(client, on_message=messages.put)

    dispatcher.dispatch(7, 2, "usecase 2")

    msg = messages.get(timeout=2.0)
    assert msg == "usecase 2 -> OK applied=3"
    assert client.calls == [(7, 2)]
    assert _wait_for(lambda: dispatcher.busy is False)


def test_dispatch_busy_guard_drops_second_call_and_first_still_completes():
    messages: queue.Queue = queue.Queue()
    client = FakeClient(result=(ResultCode.OK, 0), block=True)
    dispatcher = CommandDispatcher(client, on_message=messages.put)

    dispatcher.dispatch(1, 0, "ping")  # spawns worker, blocks inside send()
    assert dispatcher.busy is True     # set synchronously, before the worker even runs
    dispatcher.dispatch(1, 0, "ping")  # second call while the first is in flight

    busy_msg = messages.get(timeout=1.0)
    assert busy_msg == "ping -> busy, command already in flight"

    client.release.set()  # let the first send() return
    ok_msg = messages.get(timeout=2.0)
    assert ok_msg == "ping -> OK applied=0"

    assert messages.empty()
    assert client.calls == [(1, 0)]  # the dropped second dispatch never called send()
    assert _wait_for(lambda: dispatcher.busy is False)


def test_dispatch_timeout_error_reports_timeout_and_clears_busy():
    messages: queue.Queue = queue.Queue()
    client = FakeClient(raise_exc=TimeoutError("no ACK for cmd=1 token=9"))
    dispatcher = CommandDispatcher(client, on_message=messages.put)

    dispatcher.dispatch(1, 0, "ping")

    msg = messages.get(timeout=2.0)
    assert msg == "ping -> TIMEOUT no ACK for cmd=1 token=9"
    assert _wait_for(lambda: dispatcher.busy is False)

    # busy cleared -> a subsequent dispatch is accepted, not dropped as "busy"
    client2 = FakeClient(result=(ResultCode.OK, 1))
    dispatcher.client = client2
    dispatcher.dispatch(1, 0, "ping again")
    msg2 = messages.get(timeout=2.0)
    assert msg2 == "ping again -> OK applied=1"


def test_dispatch_generic_exception_reports_error_repr_and_clears_busy():
    messages: queue.Queue = queue.Queue()
    exc = RuntimeError("port is closed")
    client = FakeClient(raise_exc=exc)
    dispatcher = CommandDispatcher(client, on_message=messages.put)

    dispatcher.dispatch(1, 0, "ping")

    msg = messages.get(timeout=2.0)
    assert msg == f"ping -> ERROR {exc!r}"
    assert _wait_for(lambda: dispatcher.busy is False)
