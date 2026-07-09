"""Tests for roomscan.logbus.LogBus (TDD: written before the implementation)."""
from __future__ import annotations

import threading

import pytest

from roomscan.logbus import LogBus


def test_publish_then_drain_returns_messages_in_order():
    bus = LogBus()
    handle = bus.subscribe()

    bus.publish("first")
    bus.publish("second")
    bus.publish("third")

    assert bus.drain(handle) == ["first", "second", "third"]
    # a second drain with nothing new published returns empty
    assert bus.drain(handle) == []


def test_two_subscribers_each_receive_all_messages_independently():
    bus = LogBus()
    a = bus.subscribe()
    bus.publish("msg-1")
    b = bus.subscribe()  # subscribes after msg-1
    bus.publish("msg-2")

    # a saw both messages, b only saw the one published after it subscribed
    assert bus.drain(a) == ["msg-1", "msg-2"]
    assert bus.drain(b) == ["msg-2"]

    # cursors are independent: further drains are independently empty
    assert bus.drain(a) == []
    assert bus.drain(b) == []

    bus.publish("msg-3")
    assert bus.drain(a) == ["msg-3"]
    assert bus.drain(b) == ["msg-3"]


def test_unsubscribe_stops_further_delivery():
    bus = LogBus()
    handle = bus.subscribe()
    bus.publish("before-unsub")
    bus.unsubscribe(handle)
    bus.publish("after-unsub")

    # draining an unsubscribed handle yields nothing (no error either)
    assert bus.drain(handle) == []


def test_unsubscribe_does_not_affect_other_subscribers():
    bus = LogBus()
    a = bus.subscribe()
    b = bus.subscribe()
    bus.unsubscribe(a)
    bus.publish("hello")

    assert bus.drain(b) == ["hello"]
    assert bus.drain(a) == []


def test_backlog_cap_keeps_only_most_recent_messages():
    maxlen = 5
    bus = LogBus(maxlen=maxlen)
    handle = bus.subscribe()

    for i in range(maxlen + 3):  # publish more than the cap, never draining
        bus.publish(f"msg-{i}")

    # oldest messages (msg-0, msg-1, msg-2) were dropped; only the most
    # recent `maxlen` messages remain, in order.
    assert bus.drain(handle) == [f"msg-{i}" for i in range(3, maxlen + 3)]


def test_thread_safety_smoke_no_loss_no_exception():
    bus = LogBus()
    handle = bus.subscribe()

    n_threads = 4
    n_per_thread = 250
    total_published = n_threads * n_per_thread
    errors: list[BaseException] = []
    received: list[str] = []
    stop = threading.Event()

    def producer(tid: int) -> None:
        try:
            for i in range(n_per_thread):
                bus.publish(f"t{tid}-{i}")
        except BaseException as exc:  # pragma: no cover - failure path
            errors.append(exc)

    def drainer() -> None:
        try:
            while not stop.is_set():
                received.extend(bus.drain(handle))
        except BaseException as exc:  # pragma: no cover - failure path
            errors.append(exc)

    drain_thread = threading.Thread(target=drainer)
    drain_thread.start()

    producers = [threading.Thread(target=producer, args=(tid,)) for tid in range(n_threads)]
    for t in producers:
        t.start()
    for t in producers:
        t.join()

    stop.set()
    drain_thread.join()

    # final drain to catch anything published between the last loop
    # iteration and stop.set() being observed
    received.extend(bus.drain(handle))

    assert errors == []
    assert len(received) == total_published
    assert len(set(received)) == total_published  # no duplicates


def test_drain_unknown_handle_returns_empty_without_error():
    bus = LogBus()
    assert bus.drain(9999) == []


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
