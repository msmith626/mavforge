"""One-shot message requests via MAV_CMD_REQUEST_MESSAGE.

Requesting individual messages leaves the vehicle's stream configuration untouched
(unlike SET_MESSAGE_INTERVAL or SRx_* parameters), so it is safe for read-only use.
"""

from __future__ import annotations

import time
from collections.abc import Iterable

from mavforge.protocol.commands import command_long
from mavforge.transport.connection import MavlinkConnection
from mavforge.transport.dialect import Message, mavlink


def request_messages(
    conn: MavlinkConnection,
    target_system: int,
    target_component: int,
    message_ids: Iterable[int],
    *,
    timeout: float = 1.5,
    retries: int = 1,
) -> dict[int, Message | None]:
    """Request each message once (re-requesting missing ones) and collect the replies.

    COMMAND_ACKs for REQUEST_MESSAGE do not say which message they refer to, so this
    waits for the messages themselves. Missing entries map to ``None``.
    """
    wanted = {msg_id: mavlink.mavlink_map[msg_id] for msg_id in message_ids}
    got: dict[int, Message | None] = dict.fromkeys(wanted)

    def match(msg: Message) -> bool:
        return (
            msg.get_srcSystem() == target_system
            and msg.get_srcComponent() == target_component
            and msg.get_msgId() in wanted
        )

    with conn.subscribe(match) as sub:
        for attempt in range(retries + 1):
            missing = [i for i, m in got.items() if m is None]
            if not missing:
                break
            for msg_id in missing:
                conn.send(
                    command_long(
                        target_system,
                        target_component,
                        mavlink.MAV_CMD_REQUEST_MESSAGE,
                        [msg_id],
                        confirmation=attempt,
                    )
                )
            deadline = time.monotonic() + timeout
            while (
                any(m is None for m in got.values())
                and (remaining := deadline - time.monotonic()) > 0
            ):
                msg = sub.get(remaining)
                if msg is not None:
                    got[msg.get_msgId()] = msg
    return got
