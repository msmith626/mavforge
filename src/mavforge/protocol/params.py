"""MAVLink Parameter Protocol client (read side).

See https://mavlink.io/en/services/parameter.html. ArduPilot sends every value as a
float ("C cast" encoding) and ignores ``param_type`` on writes, so integers with a
magnitude above 2**24 can arrive rounded; such values are flagged
``precision_limited``. Parameter indices are not stable across reboots or firmware,
so they are only used to fill gaps within one download.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from mavforge.domain.errors import (
    OperationTimeoutError,
    ParamNotFoundError,
    ProtocolError,
)
from mavforge.domain.models import FLOAT_EXACT_INT_LIMIT, ParamType, ParamValue
from mavforge.transport.connection import MavlinkConnection
from mavforge.transport.dialect import Message, enum_name, mavlink

UNSOLICITED_INDEX = 65535
"""``param_index`` used by ArduPilot for PARAM_VALUE broadcasts outside a list download."""


def decode_param_value(msg: mavlink.MAVLink_param_value_message) -> ParamValue:
    name = str(msg.param_id).rstrip("\x00")
    try:
        ptype = ParamType(enum_name("MAV_PARAM_TYPE", msg.param_type))
    except ValueError:
        raise ProtocolError(
            f"Parameter {name!r} has unknown MAV_PARAM_TYPE {msg.param_type}"
        ) from None
    raw = float(msg.param_value)
    value: float | int = round(raw) if ptype.is_integer else raw
    index = msg.param_index if 0 <= msg.param_index < UNSOLICITED_INDEX else None
    return ParamValue(
        name=name,
        value=value,
        type=ptype,
        index=index,
        precision_limited=ptype.is_integer and abs(raw) > FLOAT_EXACT_INT_LIMIT,
    )


@dataclass
class ParamDownload:
    params: dict[int, ParamValue] = field(default_factory=dict[int, ParamValue])
    total: int | None = None
    missing: list[int] = field(default_factory=list[int])

    @property
    def complete(self) -> bool:
        return self.total is not None and not self.missing and len(self.params) == self.total


def _from_target(target_system: int, target_component: int, msg: Message) -> bool:
    return msg.get_srcSystem() == target_system and msg.get_srcComponent() == target_component


def fetch_all(
    conn: MavlinkConnection,
    target_system: int,
    target_component: int,
    *,
    gap_timeout: float = 1.0,
    total_timeout: float = 120.0,
    max_stalled_rounds: int = 3,
    batch_size: int = 20,
) -> ParamDownload:
    """Download the full parameter set with PARAM_REQUEST_LIST, then fill gaps by index.

    Returns whatever was received; ``missing`` lists indices still absent after
    ``max_stalled_rounds`` consecutive re-request rounds that made no progress.
    """
    result = ParamDownload()

    def match(msg: Message) -> bool:
        return isinstance(msg, mavlink.MAVLink_param_value_message) and _from_target(
            target_system, target_component, msg
        )

    with conn.subscribe(match) as sub:
        conn.send(mavlink.MAVLink_param_request_list_message(target_system, target_component))
        deadline = time.monotonic() + total_timeout
        stalled = 0
        progress_since_gap = False
        while (remaining := deadline - time.monotonic()) > 0:
            msg = sub.get(min(gap_timeout, remaining))
            if msg is not None:
                assert isinstance(msg, mavlink.MAVLink_param_value_message)
                if msg.param_index >= UNSOLICITED_INDEX:
                    continue
                result.total = msg.param_count
                if msg.param_index not in result.params:
                    result.params[msg.param_index] = decode_param_value(msg)
                    progress_since_gap = True
                if len(result.params) >= result.total:
                    break
                continue

            # Gap: nothing arrived for gap_timeout seconds.
            stalled = 0 if progress_since_gap else stalled + 1
            progress_since_gap = False
            if stalled > max_stalled_rounds:
                break
            if result.total is None:
                conn.send(
                    mavlink.MAVLink_param_request_list_message(target_system, target_component)
                )
                continue
            missing = [i for i in range(result.total) if i not in result.params]
            for index in missing[:batch_size]:
                conn.send(
                    mavlink.MAVLink_param_request_read_message(
                        target_system, target_component, b"", index
                    )
                )

    if result.total is not None:
        result.missing = [i for i in range(result.total) if i not in result.params]
    return result


def read_param(
    conn: MavlinkConnection,
    target_system: int,
    target_component: int,
    name: str,
    *,
    timeout: float = 1.0,
    retries: int = 2,
) -> ParamValue:
    """Read one parameter by name. Raises ``ParamNotFoundError`` or ``OperationTimeoutError``."""

    def match(msg: Message) -> bool:
        if not _from_target(target_system, target_component, msg):
            return False
        if isinstance(
            msg, mavlink.MAVLink_param_value_message | mavlink.MAVLink_param_error_message
        ):
            return str(msg.param_id).rstrip("\x00") == name
        return False

    with conn.subscribe(match) as sub:
        for _ in range(retries + 1):
            conn.send(
                mavlink.MAVLink_param_request_read_message(
                    target_system, target_component, name.encode(), -1
                )
            )
            msg = sub.get(timeout)
            if msg is None:
                continue
            if isinstance(msg, mavlink.MAVLink_param_value_message):
                return decode_param_value(msg)
            assert isinstance(msg, mavlink.MAVLink_param_error_message)
            if msg.error == mavlink.MAV_PARAM_ERROR_DOES_NOT_EXIST:
                raise ParamNotFoundError(f"Parameter {name} does not exist on the vehicle")
            raise ProtocolError(
                f"Vehicle reported {enum_name('MAV_PARAM_ERROR', msg.error)} reading {name}"
            )
    raise OperationTimeoutError(
        f"No response reading parameter {name}. It may not exist on this firmware "
        "(older ArduPilot versions do not report unknown names), or the link is lossy.",
        details={"param": name, "attempts": retries + 1},
    )
