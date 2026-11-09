import time

import pytest

from mavforge.domain.errors import ConnectionFailedError, InvalidInputError, NoVehicleError
from mavforge.domain.models import ProbeOutcome, VehicleIdentity
from mavforge.services.discovery import PortInfo, discover, serial_targets
from mavforge.transport.connstr import ConnectionSpec

IDENTITY = VehicleIdentity(
    system_id=1,
    component_id=1,
    autopilot="ARDUPILOTMEGA",
    mav_type="FIXED_WING",
    vehicle_family="plane",
    platform="ardupilot",
    uid="00112233",
)

MAC_PORTS = [
    PortInfo("/dev/cu.debug-console"),
    PortInfo("/dev/cu.Bluetooth-Incoming-Port"),
    PortInfo("/dev/tty.usbmodem1101", vid=0x1209, pid=0x5741),
    PortInfo("/dev/cu.SpeedyBeeF405Wing-SPP"),
    PortInfo("/dev/cu.usbmodem1101", vid=0x1209, pid=0x5741, serial_number="ABC"),
]


def test_macos_skips_builtin_and_tty_twins_and_puts_usb_first() -> None:
    targets, skipped = serial_targets(MAC_PORTS, platform="darwin")
    assert [t.spec.device for t in targets] == [
        "/dev/cu.usbmodem1101",
        "/dev/cu.SpeedyBeeF405Wing-SPP",
    ]
    assert len(skipped) == 2
    assert all("built-in" in s for s in skipped)


def test_rejects_device_names_mavforge_will_not_open() -> None:
    targets, skipped = serial_targets([PortInfo("/bin/sh")], platform="linux")
    assert targets == []
    assert "not accepted" in skipped[0]


def test_operator_endpoints_are_validated() -> None:
    with pytest.raises(InvalidInputError):
        discover(also=["/etc/passwd"], serial=False, udp_port=None, prober=_never)


def _never(spec: ConnectionSpec, timeout: float) -> VehicleIdentity:
    raise AssertionError("should not probe")


def test_probe_outcomes_are_classified() -> None:
    def prober(spec: ConnectionSpec, timeout: float) -> VehicleIdentity:
        if spec.device == "/dev/cu.usbmodem1101":
            return IDENTITY
        if spec.device == "/dev/cu.SpeedyBeeF405Wing-SPP":
            raise NoVehicleError("No heartbeat")
        if spec.kind.value == "udpin":
            raise ConnectionFailedError("Could not open: [Errno 48] Address already in use")
        raise ConnectionFailedError("Could not open: something else")

    result = discover(
        also=["tcp:127.0.0.1:5760"], ports=MAC_PORTS, prober=prober, platform="darwin"
    )
    by_conn = {c.connection: c for c in result.candidates}
    usb = by_conn["serial:/dev/cu.usbmodem1101:115200"]
    assert usb.outcome is ProbeOutcome.VEHICLE
    assert usb.vehicle is not None
    assert usb.vehicle.uid == "00112233"
    assert usb.usb_vid == 0x1209
    assert usb.usb_serial == "ABC"
    assert by_conn["serial:/dev/cu.SpeedyBeeF405Wing-SPP:115200"].outcome is (
        ProbeOutcome.NO_HEARTBEAT
    )
    assert by_conn["udpin:0.0.0.0:14550"].outcome is ProbeOutcome.BUSY
    tcp = by_conn["tcp:127.0.0.1:5760"]
    assert tcp.source == "operator"
    assert tcp.outcome is ProbeOutcome.ERROR


def test_hung_open_is_reported_not_waited_on() -> None:
    def prober(spec: ConnectionSpec, timeout: float) -> VehicleIdentity:
        time.sleep(30)
        return IDENTITY

    t0 = time.monotonic()
    result = discover(
        ports=[PortInfo("/dev/ttyACM0")],
        udp_port=None,
        prober=prober,
        timeout=0.5,
        platform="linux",
    )
    assert time.monotonic() - t0 < 10
    assert result.candidates[0].outcome is ProbeOutcome.OPEN_TIMEOUT


def test_duplicate_operator_endpoint_is_probed_once() -> None:
    result = discover(also=["udpin:0.0.0.0:14550"], serial=False, prober=lambda s, t: IDENTITY)
    assert len(result.candidates) == 1
    assert result.candidates[0].source == "udp"
