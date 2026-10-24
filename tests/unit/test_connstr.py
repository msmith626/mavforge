import pytest

from mavforge.domain.errors import InvalidInputError
from mavforge.transport.connstr import DEFAULT_BAUD, LinkKind, parse_connection_string


@pytest.mark.parametrize(
    ("text", "kind", "pymavlink"),
    [
        ("tcp:127.0.0.1:5760", LinkKind.TCP, "tcp:127.0.0.1:5760"),
        ("tcpin:0.0.0.0:5760", LinkKind.TCPIN, "tcpin:0.0.0.0:5760"),
        ("udpin:127.0.0.1:14551", LinkKind.UDPIN, "udpin:127.0.0.1:14551"),
        ("udpout:sitl.local:14550", LinkKind.UDPOUT, "udpout:sitl.local:14550"),
        ("/dev/ttyACM0", LinkKind.SERIAL, "/dev/ttyACM0"),
        ("serial:/dev/ttyUSB0:57600", LinkKind.SERIAL, "/dev/ttyUSB0"),
        ("COM3,921600", LinkKind.SERIAL, "COM3"),
    ],
)
def test_accepts_network_and_serial_links(text: str, kind: LinkKind, pymavlink: str) -> None:
    spec = parse_connection_string(text)
    assert spec.kind is kind
    assert spec.to_pymavlink() == pymavlink


def test_accepts_device_paths_with_spaces() -> None:
    # macOS names a paired Bluetooth SPP device after the product, spaces included.
    spec = parse_connection_string("/dev/cu.SpeedyBee F405 WING-SPPDev")
    assert spec.kind is LinkKind.SERIAL
    assert spec.to_pymavlink() == "/dev/cu.SpeedyBee F405 WING-SPPDev"


def test_serial_baud_defaults_and_overrides() -> None:
    assert parse_connection_string("/dev/ttyACM0").baud == DEFAULT_BAUD
    assert parse_connection_string("/dev/ttyACM0,57600").baud == 57600
    assert parse_connection_string("serial:/dev/ttyACM0:921600").baud == 921600


@pytest.mark.parametrize(
    "text",
    [
        "",
        "flight.tlog",  # pymavlink would open a log file
        "/bin/sh",  # pymavlink would execute paths containing /bin/
        "./sitl.elf",
        "udp:127.0.0.1:14550",  # ambiguous direction
        "127.0.0.1:14550",  # pymavlink treats bare host:port as UDP
        "ws://example.com:8080",
        "tcp:127.0.0.1:99999",
        "tcp:127.0.0.1",
        "tcp:bad host:5760",
        "/dev/ttyACM0,fast",
        "serial:/etc/passwd",
    ],
)
def test_rejects_unsafe_or_malformed_strings(text: str) -> None:
    with pytest.raises(InvalidInputError):
        parse_connection_string(text)
