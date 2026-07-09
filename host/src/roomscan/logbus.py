"""In-process thread-safe pub/sub log bus. Producers (device-reader thread,
command-worker threads) call publish(message) from any thread; consumers
(the GUI event-log panel, or tests) subscribe() once and drain() on their
own schedule -- typically the GUI main-loop tick -- to render new messages.
Classic console mode is unaffected: it keeps calling print() directly, this
bus is purely additive.

Design notes:
- Messages are stored verbatim (plain str). The bus does NOT stamp
  wall-clock time onto messages itself -- callers that want a timestamp
  prefix should format one into the message string before calling
  publish(). This keeps the bus's core append path free of time.time() /
  datetime.now() calls, so it stays trivially deterministic to test.
- Each subscriber gets its own bounded collections.deque acting as an
  independent read cursor: publish() appends the message to every live
  subscriber's deque, drain(handle) atomically pops and returns everything
  queued for that one subscriber. Subscribers never see each other's
  cursor position.
- Backlog cap: each subscriber's deque is constructed with maxlen (default
  DEFAULT_MAXLEN). Once a subscriber's backlog is full, appending a new
  message silently drops that subscriber's OLDEST undrained message
  (standard deque-with-maxlen behavior) to make room. publish() therefore
  never blocks and never raises because a subscriber is slow or has
  stopped draining -- it just loses old history for that subscriber.
- A single threading.Lock guards subscriber registration/removal and the
  publish fan-out, so publish() and drain() are safe to call concurrently
  from any number of threads.
"""
from __future__ import annotations

import threading
from collections import deque
from typing import Deque, Dict, List

DEFAULT_MAXLEN = 2000


class LogBus:
    """Thread-safe, dependency-free pub/sub bus for log/status messages."""

    def __init__(self, maxlen: int = DEFAULT_MAXLEN) -> None:
        self._maxlen = maxlen
        self._lock = threading.Lock()
        self._subscribers: Dict[int, Deque[str]] = {}
        self._next_handle = 0

    def publish(self, message: str) -> None:
        """Append `message` to every current subscriber's backlog. Safe to
        call from any thread. Never raises due to a full or absent
        subscriber -- a full backlog just drops that subscriber's oldest
        undrained message."""
        with self._lock:
            for backlog in self._subscribers.values():
                backlog.append(message)

    def subscribe(self) -> int:
        """Register a new subscriber and return an opaque handle. The
        subscriber only receives messages published *after* this call."""
        with self._lock:
            handle = self._next_handle
            self._next_handle += 1
            self._subscribers[handle] = deque(maxlen=self._maxlen)
            return handle

    def unsubscribe(self, handle: int) -> None:
        """Stop delivery to `handle` and free its backlog. No-op if the
        handle is unknown or already unsubscribed."""
        with self._lock:
            self._subscribers.pop(handle, None)

    def drain(self, handle: int) -> List[str]:
        """Return and clear all messages queued for `handle` since its last
        drain, in publish order. Returns an empty list if nothing is queued
        or if `handle` is unknown/unsubscribed."""
        with self._lock:
            backlog = self._subscribers.get(handle)
            if backlog is None:
                return []
            messages = list(backlog)
            backlog.clear()
            return messages
