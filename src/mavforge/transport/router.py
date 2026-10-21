"""Fan-out of received MAVLink messages to independent subscribers.

pymavlink's ``recv_match`` discards every message that does not match the caller's
filter, so two concurrent waiters silently steal each other's messages. MAVForge has
exactly one reader per link; it hands each message to every subscriber whose predicate
accepts it, each with its own queue.
"""

from __future__ import annotations

import queue
import threading
from collections.abc import Callable
from types import TracebackType
from typing import Self

from mavforge.domain.errors import ConnectionFailedError
from mavforge.transport.dialect import Message

Predicate = Callable[[Message], bool]

_CLOSED = object()


class Subscription:
    """A filtered, buffered view of the message stream. Use as a context manager."""

    def __init__(self, router: MessageRouter, predicate: Predicate, maxsize: int) -> None:
        self._router = router
        self._predicate = predicate
        self._queue: queue.Queue[object] = queue.Queue(maxsize=maxsize)
        self.dropped = 0
        """Messages discarded because this subscriber's buffer was full."""

    def deliver(self, msg: object) -> None:
        if msg is not _CLOSED and not self._predicate(msg):  # type: ignore[arg-type]
            return
        while True:
            try:
                self._queue.put_nowait(msg)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                    self.dropped += 1
                except queue.Empty:
                    pass

    def get(self, timeout: float) -> Message | None:
        """Next matching message, or ``None`` after ``timeout`` seconds.

        Raises ``ConnectionFailedError`` if the link closed or failed.
        """
        try:
            item = self._queue.get(timeout=max(timeout, 0.0))
        except queue.Empty:
            return None
        if item is _CLOSED:
            self._queue.put_nowait(_CLOSED)  # keep later calls failing too
            raise ConnectionFailedError(
                "MAVLink link closed", details={"reason": self._router.close_reason}
            )
        return item  # type: ignore[return-value]

    def close(self) -> None:
        self._router.unsubscribe(self)

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()


class MessageRouter:
    def __init__(self, *, default_maxsize: int = 10_000) -> None:
        self._lock = threading.Lock()
        self._subs: list[Subscription] = []
        self._default_maxsize = default_maxsize
        self._closed = False
        self.close_reason: str | None = None

    def subscribe(self, predicate: Predicate, *, maxsize: int | None = None) -> Subscription:
        sub = Subscription(self, predicate, maxsize or self._default_maxsize)
        with self._lock:
            if self._closed:
                sub.deliver(_CLOSED)
            else:
                self._subs.append(sub)
        return sub

    def unsubscribe(self, sub: Subscription) -> None:
        with self._lock:
            if sub in self._subs:
                self._subs.remove(sub)

    def dispatch(self, msg: Message) -> None:
        with self._lock:
            subs = list(self._subs)
        for sub in subs:
            sub.deliver(msg)

    def close(self, reason: str) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self.close_reason = reason
            subs, self._subs = self._subs, []
        for sub in subs:
            sub.deliver(_CLOSED)
