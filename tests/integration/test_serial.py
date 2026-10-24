"""Serial links, exercised through a pseudo-terminal (no hardware needed).

Regression: pymavlink imports pyserial lazily without declaring it as a dependency, so
serial connections failed with "No module named 'serial'" while TCP-only tests passed.
"""

from __future__ import annotations

import os
import select
import sys
from collections.abc import Iterator

import pytest

from mavforge.domain.errors import ConnectionFailedError
from mavforge.transport.connection import MavlinkConnection, open_failure_hint
from mavforge.transport.connstr import parse_connection_string
from mavforge.transport.dialect import Message, mavlink

pytestmark = pytest.mark.skipif(sys.platform == "win32", reason="needs POSIX pseudo-terminals")


class _FdWriter:
    def __init__(self, fd: int) -> None:
        self._fd = fd

    def write(self, data: bytes) -> None:
        os.write(self._fd, data)


@pytest.fixture
def pty_pair() -> Iterator[tuple[int, str]]:
    """(master fd acting as the vehicle side, device path for MAVForge)."""
    master, slave = os.openpty()
    try:
        yield master, os.ttyname(slave)
    finally:
        os.close(master)
        os.close(slave)


def _read_messages(fd: int, codec: mavlink.MAVLink, timeout: float) -> list[Message]:
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return []
    return codec.parse_buffer(os.read(fd, 4096)) or []


def test_mavlink_round_trip_over_serial_device(pty_pair: tuple[int, str]) -> None:
    master, device = pty_pair
    vehicle = mavlink.MAVLink(_FdWriter(master), srcSystem=1, srcComponent=1)
    with MavlinkConnection.open(parse_connection_string(f"{device},57600")) as conn:
        with conn.subscribe(lambda m: m.get_type() == "HEARTBEAT") as sub:
            vehicle.heartbeat_send(
                mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 3
            )
            heartbeat = sub.get(2.0)
        assert heartbeat is not None
        assert heartbeat.get_srcSystem() == 1

        conn.send(mavlink.MAVLink_param_request_read_message(1, 1, b"FRAME_CLASS", -1))
        received = _read_messages(master, vehicle, 2.0)
    assert [m.get_type() for m in received] == ["PARAM_REQUEST_READ"]


def test_missing_serial_device_reports_a_hint() -> None:
    with pytest.raises(ConnectionFailedError) as info:
        MavlinkConnection.open(parse_connection_string("/dev/ttyMAVFORGE_DOES_NOT_EXIST"))
    assert "plugged in" in info.value.message


def test_permission_denied_hint_mentions_dialout_on_linux() -> None:
    spec = parse_connection_string("/dev/ttyACM0")
    exc = OSError("[Errno 13] could not open port /dev/ttyACM0: [Errno 13] Permission denied")
    assert "dialout" in open_failure_hint(spec, exc, platform="linux")
    assert (
        open_failure_hint(parse_connection_string("tcp:127.0.0.1:5760"), exc, platform="linux")
        == ""
    )


def test_permission_denied_hint_has_no_dialout_advice_outside_linux() -> None:
    spec = parse_connection_string("/dev/ttyACM0")
    exc = OSError("[Errno 13] could not open port /dev/ttyACM0: [Errno 13] Permission denied")
    assert open_failure_hint(spec, exc, platform="darwin") == ""


def test_busy_device_hint_beats_the_generic_could_not_open_message() -> None:
    spec = parse_connection_string("/dev/cu.usbmodem21201")
    exc = OSError(
        "[Errno 16] could not open port /dev/cu.usbmodem21201: "
        "[Errno 16] Resource busy: '/dev/cu.usbmodem21201'"
    )
    hint = open_failure_hint(spec, exc, platform="darwin")
    assert "already open in another program" in hint
    assert "plugged in" not in hint
