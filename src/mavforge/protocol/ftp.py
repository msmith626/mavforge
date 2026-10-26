"""MAVLink FTP client (read side): open, read and close files over
FILE_TRANSFER_PROTOCOL. See https://mavlink.io/en/services/ftp.html.

Sequential reads only (one in-flight request at a time), not burst reads: simpler and
more reliable to get right than tracking and re-requesting gaps in a burst stream, at
the cost of one round trip per chunk instead of pipelining. Still far fewer round
trips than one PARAM_VALUE per parameter for a large parameter set.
"""

from __future__ import annotations

import contextlib
import struct
import time
from dataclasses import dataclass

from mavforge.domain.errors import OperationTimeoutError, ProtocolError
from mavforge.transport.connection import MavlinkConnection
from mavforge.transport.dialect import Message, mavlink

MAX_DATA = 239
"""Payload bytes per frame: 251-byte MAVLink field minus this module's 12-byte header."""

PAYLOAD_LEN = 251
_HEADER = struct.Struct("<HBBBBBxI")  # seq, session, opcode, size, req_opcode, burst, pad, offset

OP_TERMINATE_SESSION = 1
OP_OPEN_FILE_RO = 4
OP_READ_FILE = 5
OP_ACK = 128
OP_NAK = 129

NAK_FAIL = 1
NAK_INVALID_SESSION = 4
NAK_EOF = 6
NAK_UNKNOWN_COMMAND = 7
NAK_FILE_NOT_FOUND = 10

_ACK_TIMEOUT = 0.5
_RETRIES = 6


@dataclass(frozen=True)
class Frame:
    seq: int
    session: int
    opcode: int
    size: int
    req_opcode: int
    burst_complete: int
    offset: int
    data: bytes


def encode_frame(frame: Frame) -> list[int]:
    header = _HEADER.pack(
        frame.seq,
        frame.session,
        frame.opcode,
        frame.size,
        frame.req_opcode,
        frame.burst_complete,
        frame.offset,
    )
    payload = list(header + frame.data)
    if len(payload) > PAYLOAD_LEN:
        raise ProtocolError(f"FTP frame too large ({len(payload)} > {PAYLOAD_LEN} bytes)")
    return payload + [0] * (PAYLOAD_LEN - len(payload))


def decode_frame(payload: bytes) -> Frame:
    seq, session, opcode, size, req_opcode, burst_complete, offset = _HEADER.unpack_from(payload, 0)
    data = bytes(payload[_HEADER.size : _HEADER.size + size])
    return Frame(seq, session, opcode, size, req_opcode, burst_complete, offset, data)


def _from_target(target_system: int, target_component: int, msg: Message) -> bool:
    return msg.get_srcSystem() == target_system and msg.get_srcComponent() == target_component


def _request(
    conn: MavlinkConnection,
    target_system: int,
    target_component: int,
    *,
    seq: int,
    session: int,
    opcode: int,
    offset: int = 0,
    data: bytes = b"",
    size: int | None = None,
) -> Frame:
    """Send one request, retrying (same seq) on timeout, and return its ACK/NAK frame."""
    frame = Frame(seq, session, opcode, size if size is not None else len(data), 0, 0, offset, data)

    def match(msg: Message) -> bool:
        return isinstance(msg, mavlink.MAVLink_file_transfer_protocol_message) and _from_target(
            target_system, target_component, msg
        )

    with conn.subscribe(match) as sub:
        for _ in range(_RETRIES + 1):
            conn.send(
                mavlink.MAVLink_file_transfer_protocol_message(
                    0, target_system, target_component, encode_frame(frame)
                )
            )
            deadline = time.monotonic() + _ACK_TIMEOUT
            while (remaining := deadline - time.monotonic()) > 0:
                msg = sub.get(remaining)
                if msg is None:
                    break
                assert isinstance(msg, mavlink.MAVLink_file_transfer_protocol_message)
                reply = decode_frame(bytes(msg.payload))
                # ArduPilot's actual behavior (GCS_FTP.cpp: reply.seq_number =
                # request.seq_number + 1), not the "same number as the request" the
                # mavlink.io spec documents - confirmed against real ArduCopter SITL.
                if reply.seq != seq + 1:
                    continue  # a reply to an earlier request/retry; keep waiting
                if reply.opcode not in (OP_ACK, OP_NAK):
                    raise ProtocolError(f"FTP reply has an unexpected opcode {reply.opcode}")
                return reply
    raise OperationTimeoutError(
        f"No FTP response to opcode {opcode} after {_RETRIES + 1} attempts",
        details={"opcode": opcode, "offset": offset},
    )


def fetch_file(
    conn: MavlinkConnection, target_system: int, target_component: int, path: bytes
) -> bytes:
    """Download one file over MAVLink FTP: OpenFileRO, sequential ReadFile, TerminateSession.

    Raises ``ProtocolError``/``OperationTimeoutError`` on anything the server reports
    or fails to answer; callers decide whether to fall back to a slower method.
    """
    seq = 0

    def next_seq() -> int:
        nonlocal seq
        seq += 1
        return seq

    opened = _request(
        conn,
        target_system,
        target_component,
        seq=next_seq(),
        session=0,
        opcode=OP_OPEN_FILE_RO,
        data=path,
    )
    if opened.opcode != OP_ACK:
        raise ProtocolError(f"OpenFileRO({path!r}) failed: NAK error {_nak_error(opened)}")
    if len(opened.data) < 4:
        raise ProtocolError("OpenFileRO ACK did not include a file size")
    (file_size,) = struct.unpack_from("<I", opened.data, 0)
    session = opened.session

    chunks = bytearray()
    try:
        offset = 0
        while offset < file_size:
            want = min(MAX_DATA, file_size - offset)
            reply = _request(
                conn,
                target_system,
                target_component,
                seq=next_seq(),
                session=session,
                opcode=OP_READ_FILE,
                offset=offset,
                size=want,
            )
            if reply.opcode == OP_NAK:
                if reply.data and reply.data[0] == NAK_EOF:
                    break
                raise ProtocolError(
                    f"ReadFile at offset {offset} failed: NAK error {_nak_error(reply)}"
                )
            if not reply.data:
                raise ProtocolError(f"ReadFile at offset {offset} returned an empty ACK")
            chunks += reply.data
            offset += len(reply.data)
    finally:
        # Best-effort close; a failure here doesn't change the download result above.
        with contextlib.suppress(ProtocolError, OperationTimeoutError):
            _request(
                conn,
                target_system,
                target_component,
                seq=next_seq(),
                session=session,
                opcode=OP_TERMINATE_SESSION,
            )
    return bytes(chunks)


def _nak_error(frame: Frame) -> int | None:
    return frame.data[0] if frame.data else None
