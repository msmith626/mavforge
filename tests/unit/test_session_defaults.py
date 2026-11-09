from typing import Any

import pytest

from mavforge.platforms.ardupilot.adapter import ArduPilotAdapter
from mavforge.services import session
from mavforge.services.session import (
    HEARTBEAT_TIMEOUT,
    NETWORK_HEARTBEAT_TIMEOUT,
    NETWORK_IDENTIFY_RETRIES,
    SERIAL_IDENTIFY_RETRIES,
    identify,
    link_defaults,
)
from mavforge.transport.connstr import parse_connection_string
from mavforge.transport.dialect import mavlink

AV = mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION


def test_serial_and_network_links_get_different_defaults() -> None:
    assert link_defaults(parse_connection_string("/dev/ttyACM0")) == (
        HEARTBEAT_TIMEOUT,
        SERIAL_IDENTIFY_RETRIES,
    )
    for text in ("udpin:0.0.0.0:14550", "tcp:127.0.0.1:5760"):
        assert link_defaults(parse_connection_string(text)) == (
            NETWORK_HEARTBEAT_TIMEOUT,
            NETWORK_IDENTIFY_RETRIES,
        )


class _Conn:
    def latest(self, *args: Any) -> None:
        return None


def _heartbeat() -> mavlink.MAVLink_heartbeat_message:
    hb = mavlink.MAVLink_heartbeat_message(
        mavlink.MAV_TYPE_FIXED_WING, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0, 3
    )
    hb._header = mavlink.MAVLink_header(0, 0, 0, 1, 1)
    return hb


def _run(
    monkeypatch: pytest.MonkeyPatch, retries: int, av_replies: list[bool]
) -> list[tuple[list[int], int]]:
    """Record request_messages calls; each call answers AUTOPILOT_VERSION per av_replies."""
    calls: list[tuple[list[int], int]] = []
    version = mavlink.MAVLink_autopilot_version_message(
        0, 0, 0, 0, 0, [0] * 8, [0] * 8, [0] * 8, 0, 0, 0x33001C00, [0] * 18
    )

    def fake(conn: Any, sysid: int, compid: int, ids: Any, **kw: Any) -> dict[int, Any]:
        ids = list(ids)
        calls.append((ids, kw["retries"]))
        answered = av_replies[len(calls) - 1]
        return {i: (version if i == AV and answered else None) for i in ids}

    monkeypatch.setattr(session, "request_messages", fake)
    identify(
        _Conn(),  # type: ignore[arg-type]
        _heartbeat(),
        ArduPilotAdapter(),
        set(),
        request_timeout=0.1,
        retries=retries,
    )
    return calls


def test_extra_retries_only_rerequest_autopilot_version(monkeypatch: pytest.MonkeyPatch) -> None:
    calls = _run(monkeypatch, NETWORK_IDENTIFY_RETRIES, [False, True])
    assert len(calls) == 2
    assert AV in calls[0][0]
    assert len(calls[0][0]) == 2
    assert calls[0][1] == 1
    assert calls[1] == ([AV], NETWORK_IDENTIFY_RETRIES - 2)


def test_no_extra_request_when_version_arrived(monkeypatch: pytest.MonkeyPatch) -> None:
    assert len(_run(monkeypatch, NETWORK_IDENTIFY_RETRIES, [True])) == 1


def test_serial_default_makes_one_request_call(monkeypatch: pytest.MonkeyPatch) -> None:
    assert len(_run(monkeypatch, SERIAL_IDENTIFY_RETRIES, [False])) == 1
