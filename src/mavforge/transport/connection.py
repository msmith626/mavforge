"""A thread-safe MAVLink connection with a single reader thread.

The connection owns the pymavlink transport. A background thread receives every
message and hands it to the ``MessageRouter``; callers subscribe *before* sending a
request so no reply can be missed. Sends are serialised with a lock.

MAVForge does not send its own HEARTBEAT. ArduPilot answers parameter and command
requests without one, and a tool that heartbeats could mask a GCS-failsafe condition
on a real vehicle if the operator's ground station disappears.
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import UTC, datetime
from types import TracebackType
from typing import Protocol, Self

from mavforge.domain.errors import ConnectionFailedError
from mavforge.domain.models import StatusText
from mavforge.transport.connstr import DEFAULT_BAUD, ConnectionSpec
from mavforge.transport.dialect import Message, enum_name, mavlink, mavutil
from mavforge.transport.router import MessageRouter, Predicate, Subscription

log = logging.getLogger(__name__)

DEFAULT_SOURCE_SYSTEM = 255
DEFAULT_SOURCE_COMPONENT = mavlink.MAV_COMP_ID_USER1
"""Component 25 (USER1): distinct from MAVProxy (0) and Mission Planner / QGC (190), so
replies addressed to MAVForge can be told apart when tools share a link."""


class Transport(Protocol):
    """The minimal byte-level link the connection needs. Implemented over pymavlink."""

    def recv(self, timeout: float) -> Message | None: ...
    def send(self, msg: Message) -> None: ...
    def close(self) -> None: ...
    @property
    def receive_errors(self) -> int: ...


def open_failure_hint(spec: ConnectionSpec, exc: Exception, *, platform: str = sys.platform) -> str:
    """A short remedy for common link-opening failures, or an empty string.

    ``platform`` defaults to the real ``sys.platform`` but is injectable so tests get
    deterministic, OS-independent results (CI runs Linux; a contributor's machine may not).
    """
    if not spec.is_serial:
        return ""
    text = str(exc)
    # Checked before the generic "could not open port" match below: pyserial's message
    # for a busy device also contains that phrase, so the specific case must win.
    if "Resource busy" in text or "Device busy" in text:
        return (
            ". The device is already open in another program (another mavforge "
            "process, Arduino IDE, 'screen', a GCS, or — on macOS — the Bluetooth "
            "pane in System Settings) — close it and try again"
        )
    if "Permission denied" in text:
        if platform.startswith("linux"):
            return (
                ". On Linux, add your user to the group that owns the device "
                "(usually 'dialout': sudo usermod -aG dialout $USER), then log in again"
            )
        return ""
    if "No such file" in text or "could not open port" in text:
        return ". Check the device path and that the flight controller is plugged in"
    return ""


def _raise_closed() -> None:
    raise ConnectionResetError("peer closed the connection")


class PymavlinkTransport:
    """``Transport`` over ``mavutil.mavlink_connection``. Not thread-safe by itself."""

    def __init__(self, spec: ConnectionSpec, *, source_system: int, source_component: int) -> None:
        try:
            self._conn = mavutil.mavlink_connection(
                spec.to_pymavlink(),
                baud=spec.baud or DEFAULT_BAUD,  # ignored for network links
                source_system=source_system,
                source_component=source_component,
                autoreconnect=False,
            )
        except Exception as exc:  # pymavlink raises a variety of OSError/SerialException types
            raise ConnectionFailedError(
                f"Could not open {spec}: {exc}{open_failure_hint(spec, exc)}",
                details={"connection": str(spec)},
            ) from exc
        self._errors = 0
        if hasattr(self._conn, "handle_eof"):
            # pymavlink's TCP class answers EOF/reset by printing to stdout and calling
            # reconnect(), which is a no-op without autoreconnect: recv() then returns
            # nothing forever and the loss goes unnoticed. Surface it as a link failure.
            self._conn.handle_eof = _raise_closed
            self._conn.handle_disconnect = _raise_closed

    def recv(self, timeout: float) -> Message | None:
        msg = self._conn.recv_match(blocking=True, timeout=timeout)
        if msg is None:
            return None
        if msg.get_type() == "BAD_DATA":
            self._errors += 1
            return None
        return msg

    def send(self, msg: Message) -> None:
        self._conn.mav.send(msg)

    def close(self) -> None:
        self._conn.close()

    @property
    def receive_errors(self) -> int:
        return self._errors


@dataclass(frozen=True)
class Seen:
    msg: Message
    monotonic: float


class MavlinkConnection:
    """Owns one link: reader thread, router, send lock, and recent-message caches."""

    def __init__(
        self,
        transport: Transport,
        description: str,
        *,
        source_system: int = DEFAULT_SOURCE_SYSTEM,
        source_component: int = DEFAULT_SOURCE_COMPONENT,
        statustext_history: int = 100,
    ) -> None:
        self._transport = transport
        self.description = description
        self.source_system = source_system
        self.source_component = source_component
        self.router = MessageRouter()
        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._latest: dict[tuple[str, int, int], Seen] = {}
        self._statustext: deque[tuple[int, StatusText]] = deque(maxlen=statustext_history)
        self._messages = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, name="mavforge-reader", daemon=True)
        self._thread.start()

    @classmethod
    def open(
        cls,
        spec: ConnectionSpec,
        *,
        source_system: int = DEFAULT_SOURCE_SYSTEM,
        source_component: int = DEFAULT_SOURCE_COMPONENT,
    ) -> MavlinkConnection:
        transport = PymavlinkTransport(
            spec, source_system=source_system, source_component=source_component
        )
        return cls(
            transport,
            str(spec),
            source_system=source_system,
            source_component=source_component,
        )

    # -- receiving -------------------------------------------------------------------

    def _reader(self) -> None:
        reason = "closed"
        try:
            while not self._stop.is_set():
                msg = self._transport.recv(0.1)
                if msg is not None:
                    self._record(msg)
                    self.router.dispatch(msg)
        except Exception as exc:
            reason = f"link error: {exc}"
            log.warning("MAVLink reader stopped: %s", exc)
        finally:
            self.router.close(reason)

    def _record(self, msg: Message) -> None:
        key = (msg.get_type(), msg.get_srcSystem(), msg.get_srcComponent())
        with self._state_lock:
            self._messages += 1
            self._latest[key] = Seen(msg, time.monotonic())
            if isinstance(msg, mavlink.MAVLink_statustext_message):
                text = StatusText(
                    severity=enum_name("MAV_SEVERITY", msg.severity),
                    text=str(msg.text).rstrip("\x00"),
                    received_at=datetime.now(UTC),
                )
                self._statustext.append((msg.get_srcSystem(), text))

    def subscribe(self, predicate: Predicate, *, maxsize: int | None = None) -> Subscription:
        return self.router.subscribe(predicate, maxsize=maxsize)

    def latest(self, msg_type: str, system_id: int, component_id: int) -> Seen | None:
        with self._state_lock:
            return self._latest.get((msg_type, system_id, component_id))

    def statustexts(self, system_id: int) -> list[StatusText]:
        with self._state_lock:
            return [t for sysid, t in self._statustext if sysid == system_id]

    @property
    def messages_received(self) -> int:
        with self._state_lock:
            return self._messages

    @property
    def receive_errors(self) -> int:
        return self._transport.receive_errors

    @property
    def is_open(self) -> bool:
        return self._thread.is_alive()

    # -- sending ---------------------------------------------------------------------

    def send(self, msg: Message) -> None:
        if not self.is_open:
            raise ConnectionFailedError(
                "MAVLink link closed", details={"reason": self.router.close_reason}
            )
        with self._send_lock:
            try:
                self._transport.send(msg)
            except OSError as exc:
                raise ConnectionFailedError(f"Send failed: {exc}") from exc

    # -- lifecycle -------------------------------------------------------------------

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._transport.close()
        self.router.close("closed")

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()
