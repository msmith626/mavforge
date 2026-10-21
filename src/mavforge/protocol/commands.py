"""MAVLink Command Protocol client (COMMAND_LONG / COMMAND_ACK).

See https://mavlink.io/en/services/command.html. Retransmission increments the
``confirmation`` field and is only allowed for commands the caller declares idempotent:
re-sending a non-idempotent command after a lost ACK could execute it twice.
"""

from __future__ import annotations

import time
from collections.abc import Sequence
from dataclasses import dataclass

from mavforge.domain.outcomes import OperationStatus
from mavforge.transport.connection import MavlinkConnection
from mavforge.transport.dialect import Message, enum_name, mavlink
from mavforge.transport.router import Predicate

_RESULT_STATUS: dict[int, OperationStatus] = {
    mavlink.MAV_RESULT_ACCEPTED: OperationStatus.SUCCEEDED,
    mavlink.MAV_RESULT_TEMPORARILY_REJECTED: OperationStatus.REJECTED,
    mavlink.MAV_RESULT_DENIED: OperationStatus.REJECTED,
    mavlink.MAV_RESULT_UNSUPPORTED: OperationStatus.UNSUPPORTED,
    mavlink.MAV_RESULT_FAILED: OperationStatus.FAILED,
    mavlink.MAV_RESULT_IN_PROGRESS: OperationStatus.IN_PROGRESS,
    mavlink.MAV_RESULT_COMMAND_LONG_ONLY: OperationStatus.UNSUPPORTED,
    mavlink.MAV_RESULT_COMMAND_INT_ONLY: OperationStatus.UNSUPPORTED,
    # Defined in upstream common.xml but absent from pymavlink 2.4.50's ardupilotmega build:
    6: OperationStatus.CANCELLED,  # MAV_RESULT_CANCELLED
    9: OperationStatus.UNSUPPORTED,  # MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME
    10: OperationStatus.REJECTED,  # MAV_RESULT_NOT_IN_CONTROL
}

_MISSING_RESULT_NAMES = {6: "CANCELLED", 9: "COMMAND_UNSUPPORTED_MAV_FRAME", 10: "NOT_IN_CONTROL"}


def result_name(result: int) -> str:
    return _MISSING_RESULT_NAMES.get(result) or enum_name("MAV_RESULT", result)


@dataclass(frozen=True)
class CommandOutcome:
    command: int
    status: OperationStatus
    """Command-level status: SUCCEEDED means the vehicle *accepted* the command, not that
    whatever it started has finished. Callers verify completion separately."""
    result: str | None
    """MAV_RESULT name from the final ACK, or None if no ACK arrived."""
    attempts: int
    progress: int | None = None


def _ack_matcher(conn: MavlinkConnection, target_system: int, command: int) -> Predicate:
    def match(msg: Message) -> bool:
        return (
            isinstance(msg, mavlink.MAVLink_command_ack_message)
            and msg.command == command
            and msg.get_srcSystem() == target_system
            and msg.target_system in (0, conn.source_system)
            and msg.target_component in (0, conn.source_component)
        )

    return match


def command_long(
    target_system: int,
    target_component: int,
    command: int,
    params: Sequence[float],
    confirmation: int = 0,
) -> Message:
    p = list(params) + [0.0] * (7 - len(params))
    if len(p) != 7:
        raise ValueError("COMMAND_LONG takes at most 7 params")
    return mavlink.MAVLink_command_long_message(
        target_system, target_component, command, confirmation, *p
    )


def send_command(
    conn: MavlinkConnection,
    target_system: int,
    target_component: int,
    command: int,
    params: Sequence[float] = (),
    *,
    timeout: float = 1.5,
    retries: int = 0,
    idempotent: bool = False,
    in_progress_timeout: float = 30.0,
) -> CommandOutcome:
    """Send COMMAND_LONG and wait for its final COMMAND_ACK.

    Without an ACK the outcome is TIMED_OUT for idempotent commands and UNKNOWN
    otherwise (the vehicle may have acted on it).
    """
    if retries and not idempotent:
        raise ValueError("Only idempotent commands may be retransmitted")
    with conn.subscribe(_ack_matcher(conn, target_system, command)) as acks:
        progress: int | None = None
        attempts = 0
        for attempt in range(retries + 1):
            attempts = attempt + 1
            conn.send(command_long(target_system, target_component, command, params, attempt))
            deadline = time.monotonic() + timeout
            while (remaining := deadline - time.monotonic()) > 0:
                ack = acks.get(remaining)
                if ack is None:
                    break
                assert isinstance(ack, mavlink.MAVLink_command_ack_message)
                if ack.result == mavlink.MAV_RESULT_IN_PROGRESS:
                    progress = ack.progress
                    deadline = time.monotonic() + in_progress_timeout
                    continue
                return CommandOutcome(
                    command=command,
                    status=_RESULT_STATUS.get(ack.result, OperationStatus.FAILED),
                    result=result_name(ack.result),
                    attempts=attempts,
                    progress=progress,
                )
            if progress is not None:
                break  # it was running; re-sending would restart it
    status = (
        OperationStatus.TIMED_OUT if idempotent and progress is None else OperationStatus.UNKNOWN
    )
    return CommandOutcome(
        command=command, status=status, result=None, attempts=attempts, progress=progress
    )
