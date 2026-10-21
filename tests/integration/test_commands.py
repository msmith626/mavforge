"""Command-protocol client semantics."""

import pytest

from mavforge.domain.outcomes import OperationStatus
from mavforge.protocol.commands import send_command
from mavforge.testing.fake_vehicle import FakeFactory
from mavforge.transport.connection import MavlinkConnection
from mavforge.transport.connstr import parse_connection_string
from mavforge.transport.dialect import mavlink


def _open(fake_cs: str) -> MavlinkConnection:
    return MavlinkConnection.open(parse_connection_string(fake_cs))


def test_accepted_command(make_fake: FakeFactory) -> None:
    fake = make_fake()
    with _open(fake.connection_string) as conn:
        outcome = send_command(
            conn, 1, 1, mavlink.MAV_CMD_REQUEST_MESSAGE, [mavlink.MAVLINK_MSG_ID_SYS_STATUS]
        )
    assert outcome.status is OperationStatus.SUCCEEDED
    assert outcome.result == "ACCEPTED"


def test_unsupported_command(make_fake: FakeFactory) -> None:
    fake = make_fake()
    with _open(fake.connection_string) as conn:
        outcome = send_command(conn, 1, 1, mavlink.MAV_CMD_DO_SET_SERVO, [9, 1500])
    assert outcome.status is OperationStatus.UNSUPPORTED


def test_lost_ack_is_unknown_for_non_idempotent_commands(make_fake: FakeFactory) -> None:
    fake = make_fake(respond_to_requests=False)
    with _open(fake.connection_string) as conn:
        outcome = send_command(conn, 1, 1, mavlink.MAV_CMD_DO_SET_SERVO, [9, 1500], timeout=0.3)
    assert outcome.status is OperationStatus.UNKNOWN
    assert outcome.attempts == 1


def test_idempotent_commands_retry_with_confirmation(make_fake: FakeFactory) -> None:
    fake = make_fake(respond_to_requests=False)
    with _open(fake.connection_string) as conn:
        outcome = send_command(
            conn,
            1,
            1,
            mavlink.MAV_CMD_REQUEST_MESSAGE,
            [1],
            timeout=0.2,
            retries=2,
            idempotent=True,
        )
    assert outcome.status is OperationStatus.TIMED_OUT
    assert outcome.attempts == 3
    confirmations = [
        m.confirmation for m in fake.received if isinstance(m, mavlink.MAVLink_command_long_message)
    ]
    assert confirmations == [0, 1, 2]


def test_retries_require_idempotent_flag(make_fake: FakeFactory) -> None:
    fake = make_fake()
    with _open(fake.connection_string) as conn, pytest.raises(ValueError, match="idempotent"):
        send_command(conn, 1, 1, mavlink.MAV_CMD_DO_SET_SERVO, retries=1)
