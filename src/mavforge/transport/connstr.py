"""Validated MAVLink connection strings.

Only network and serial links are accepted. pymavlink's ``mavlink_connection`` also
opens log files and can *execute* programs given certain paths; MAVForge never passes
it an unvalidated string.

Accepted forms::

    tcp:HOST:PORT           connect to a TCP server (e.g. SITL on 5760)
    tcpin:HOST:PORT         listen for one TCP client
    udpin:HOST:PORT         listen for UDP (e.g. a MAVProxy --out udp:HOST:PORT)
    udpout:HOST:PORT        send UDP to HOST:PORT
    serial:DEVICE[:BAUD]    serial device, default 115200 baud
    /dev/ttyACM0[,BAUD]     shorthand for a serial device (MAVProxy style)
    COM3[,BAUD]             Windows serial shorthand
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from enum import StrEnum

from mavforge.domain.errors import InvalidInputError

DEFAULT_BAUD = 115200

_HOST_RE = re.compile(r"^[A-Za-z0-9.\-]+$|^\[[0-9A-Fa-f:]+\]$")
_DEVICE_RE = re.compile(r"^(/dev/[A-Za-z0-9._/\-]+|COM[0-9]+)$")


class LinkKind(StrEnum):
    TCP = "tcp"
    TCPIN = "tcpin"
    UDPIN = "udpin"
    UDPOUT = "udpout"
    SERIAL = "serial"


@dataclass(frozen=True)
class ConnectionSpec:
    kind: LinkKind
    host: str | None = None
    port: int | None = None
    device: str | None = None
    baud: int | None = None

    @property
    def is_serial(self) -> bool:
        return self.kind is LinkKind.SERIAL

    def to_pymavlink(self) -> str:
        """The device string to hand to ``mavutil.mavlink_connection``."""
        if self.kind is LinkKind.SERIAL:
            assert self.device is not None
            return self.device
        return f"{self.kind.value}:{self.host}:{self.port}"

    def __str__(self) -> str:
        if self.kind is LinkKind.SERIAL:
            return f"serial:{self.device}:{self.baud}"
        return f"{self.kind.value}:{self.host}:{self.port}"


def _parse_port(text: str, original: str) -> int:
    if not text.isdigit() or not 1 <= int(text) <= 65535:
        raise InvalidInputError(f"Invalid port in connection string {original!r}")
    return int(text)


def _parse_baud(text: str, original: str) -> int:
    if not text.isdigit() or int(text) <= 0:
        raise InvalidInputError(f"Invalid baud rate in connection string {original!r}")
    return int(text)


def _serial(device: str, baud_text: str | None, original: str) -> ConnectionSpec:
    if not _DEVICE_RE.match(device):
        raise InvalidInputError(f"Invalid serial device in connection string {original!r}")
    baud = _parse_baud(baud_text, original) if baud_text else DEFAULT_BAUD
    return ConnectionSpec(kind=LinkKind.SERIAL, device=device, baud=baud)


def parse_connection_string(text: str) -> ConnectionSpec:
    """Parse and validate a connection string, raising ``InvalidInputError`` if unsafe."""
    original = text
    text = text.strip()
    if not text:
        raise InvalidInputError("Connection string is empty")

    if text.startswith(("/dev/", "COM")):
        device, _, baud = text.partition(",")
        return _serial(device, baud or None, original)

    scheme, sep, rest = text.partition(":")
    if not sep:
        raise InvalidInputError(f"Unrecognised connection string {original!r}")

    if scheme == "serial":
        device, _, baud = rest.rpartition(":") if rest.count(":") else (rest, "", "")
        if not device:
            device, baud = rest, ""
        return _serial(device, baud or None, original)

    try:
        kind = LinkKind(scheme)
    except ValueError:
        hint = (
            " Use udpin: or udpout: to make the UDP direction explicit." if scheme == "udp" else ""
        )
        raise InvalidInputError(
            f"Unsupported connection type {scheme!r} in {original!r}.{hint}"
        ) from None

    host, sep, port = rest.rpartition(":")
    if not sep or not host:
        raise InvalidInputError(f"Expected {scheme}:HOST:PORT, got {original!r}")
    if not _HOST_RE.match(host):
        raise InvalidInputError(f"Invalid host in connection string {original!r}")
    return ConnectionSpec(kind=kind, host=host, port=_parse_port(port, original))
