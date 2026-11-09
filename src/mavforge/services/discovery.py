"""Finding vehicles: enumerate candidate links and probe each one for an autopilot.

Discovery never takes a connection string from an agent. Candidates come from the OS
(serial ports), a fixed UDP listen port, and endpoints the operator passes explicitly.
Every candidate is validated by ``parse_connection_string`` before it is opened.

Probing is passive until a vehicle is heard: MAVForge sends no HEARTBEAT, so opening a
port that turns out not to be a flight controller only reads from it. Once an autopilot
heartbeat arrives, the probe sends the same AUTOPILOT_VERSION request ``connect`` does,
to learn the board ``uid`` that later lets a reconnect recognise the same vehicle on a
different link.

Caveat: pymavlink opens serial ports non-exclusively and binds UDP with SO_REUSEADDR, so
probing a link another program already has open (a GCS, a running MAVForge MCP server)
can briefly steal its bytes or packets. Close other programs first; the MCP tool built on
this must skip the link the server itself holds.
"""

from __future__ import annotations

import sys
import threading
import time
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from typing import Literal

from mavforge.domain.errors import ConnectionFailedError, MAVForgeError, NoVehicleError
from mavforge.domain.models import DiscoveryResult, LinkCandidate, ProbeOutcome, VehicleIdentity
from mavforge.platforms.base import select_adapter
from mavforge.services.session import identify, wait_for_vehicle
from mavforge.transport.connection import MavlinkConnection
from mavforge.transport.connstr import ConnectionSpec, parse_connection_string

DEFAULT_PROBE_TIMEOUT = 4.0
DEFAULT_NETWORK_TIMEOUT = 10.0
"""WiFi bridges are lossy: on real hardware (2026-11-08) a SpeedyBee F405 WING over a
WiFi bridge averaged ~47 msg/s but went silent for up to 5.1 s, so a serial-length wait
regularly missed the vehicle or its AUTOPILOT_VERSION reply."""
NETWORK_IDENTIFY_RETRIES = 4
DEFAULT_UDP_PORT = 14550

# macOS ports that are always present and never a flight controller.
_MACOS_SKIP = ("/dev/cu.debug-console", "/dev/cu.Bluetooth-Incoming-Port")


@dataclass(frozen=True)
class PortInfo:
    """The subset of pyserial's ``ListPortInfo`` discovery uses (injectable in tests)."""

    device: str
    description: str | None = None
    manufacturer: str | None = None
    vid: int | None = None
    pid: int | None = None
    serial_number: str | None = None


@dataclass(frozen=True)
class Target:
    spec: ConnectionSpec
    source: Literal["serial", "udp", "operator"]
    port: PortInfo | None = None


Prober = Callable[[ConnectionSpec, float], VehicleIdentity]


def system_ports() -> list[PortInfo]:
    from serial.tools import list_ports

    return [
        PortInfo(
            device=p.device,
            description=p.description if p.description not in (None, "n/a") else None,
            manufacturer=p.manufacturer,
            vid=p.vid,
            pid=p.pid,
            serial_number=p.serial_number,
        )
        for p in list_ports.comports()
    ]


def serial_targets(
    ports: Iterable[PortInfo], *, platform: str = sys.platform
) -> tuple[list[Target], list[str]]:
    """Serial ports worth probing, and the ones skipped with a reason."""
    targets: list[Target] = []
    skipped: list[str] = []
    for port in ports:
        if platform == "darwin":
            if port.device.startswith("/dev/tty."):
                continue  # the /dev/cu.* twin is the one to open; not worth reporting
            if port.device in _MACOS_SKIP:
                skipped.append(f"{port.device}: built-in macOS port")
                continue
        try:
            spec = parse_connection_string(port.device)
        except MAVForgeError:
            skipped.append(f"{port.device}: device name not accepted by MAVForge")
            continue
        targets.append(Target(spec, "serial", port))
    # USB devices first: they are the likeliest flight controllers and the fastest to probe.
    targets.sort(key=lambda t: t.port is None or t.port.vid is None)
    return targets, skipped


def probe_link(spec: ConnectionSpec, timeout: float) -> VehicleIdentity:
    """Open ``spec``, wait for an autopilot heartbeat, identify it, and close. Network
    links get extra identity retries: a missing ``uid`` defeats reconnect matching."""
    conn = MavlinkConnection.open(spec)
    try:
        heartbeat, seen = wait_for_vehicle(conn, None, timeout)
        adapter = select_adapter(heartbeat)
        retries = 1 if spec.is_serial else NETWORK_IDENTIFY_RETRIES
        return identify(conn, heartbeat, adapter, seen, request_timeout=1.5, retries=retries)
    finally:
        conn.close()


def _classify(exc: BaseException) -> tuple[ProbeOutcome, str]:
    text = str(exc)
    if isinstance(exc, MAVForgeError) and (reason := exc.details.get("reason")):
        text = f"{text} ({reason})"
    if isinstance(exc, NoVehicleError):
        return ProbeOutcome.NO_HEARTBEAT, text
    busy = ("Resource busy", "Device busy", "Address already in use")
    if isinstance(exc, ConnectionFailedError) and any(b in text for b in busy):
        return ProbeOutcome.BUSY, text
    return ProbeOutcome.ERROR, text


def _candidate(
    target: Target,
    outcome: ProbeOutcome,
    seconds: float,
    *,
    detail: str | None = None,
    vehicle: VehicleIdentity | None = None,
) -> LinkCandidate:
    p = target.port
    return LinkCandidate(
        connection=str(target.spec),
        source=target.source,
        outcome=outcome,
        detail=detail,
        description=p.description if p else None,
        manufacturer=p.manufacturer if p else None,
        usb_vid=p.vid if p else None,
        usb_pid=p.pid if p else None,
        usb_serial=p.serial_number if p else None,
        vehicle=vehicle,
        probe_seconds=round(seconds, 2),
    )


def _probe_all(
    targets: Sequence[Target], timeouts: Sequence[float], prober: Prober
) -> list[LinkCandidate]:
    """Probe every target in parallel. Threads are daemons: an open() that hangs (macOS
    Bluetooth can) is reported as ``open_timeout`` rather than blocking exit."""
    results: list[LinkCandidate | None] = [None] * len(targets)
    started = time.monotonic()

    def run(i: int, target: Target) -> None:
        t0 = time.monotonic()
        try:
            identity = prober(target.spec, timeouts[i])
        except BaseException as exc:
            outcome, detail = _classify(exc)
            results[i] = _candidate(target, outcome, time.monotonic() - t0, detail=detail)
        else:
            results[i] = _candidate(
                target, ProbeOutcome.VEHICLE, time.monotonic() - t0, vehicle=identity
            )

    threads = [
        threading.Thread(target=run, args=(i, t), name=f"mavforge-probe-{i}", daemon=True)
        for i, t in enumerate(targets)
    ]
    for th in threads:
        th.start()
    # Heartbeat wait + identify (with retries on network links) + close, plus slack.
    slack = 1.5 * (NETWORK_IDENTIFY_RETRIES + 1) + 2.0
    deadline = started + max(timeouts, default=0.0) + slack
    for th in threads:
        th.join(max(0.0, deadline - time.monotonic()))
    return [
        r
        if r is not None
        else _candidate(
            targets[i],
            ProbeOutcome.OPEN_TIMEOUT,
            time.monotonic() - started,
            detail="Opening the link did not complete in time",
        )
        for i, r in enumerate(results)
    ]


_LOCAL_NETWORK_HINT = (
    "; if a WiFi bridge is up, check System Settings > Privacy & Security > "
    "Local Network allows Python (macOS silently drops its local traffic otherwise)"
)


def _hint_local_network(candidates: list[LinkCandidate], platform: str) -> list[LinkCandidate]:
    """On macOS, a UDP listen that hears nothing may be the Local Network privacy block."""
    if platform != "darwin":
        return candidates
    return [
        c.model_copy(update={"detail": f"{c.detail}{_LOCAL_NETWORK_HINT}"})
        if c.source == "udp" and c.outcome is ProbeOutcome.NO_HEARTBEAT
        else c
        for c in candidates
    ]


def discover(
    *,
    also: Sequence[str] = (),
    udp_port: int | None = DEFAULT_UDP_PORT,
    serial: bool = True,
    timeout: float = DEFAULT_PROBE_TIMEOUT,
    network_timeout: float = DEFAULT_NETWORK_TIMEOUT,
    ports: Iterable[PortInfo] | None = None,
    prober: Prober = probe_link,
    platform: str = sys.platform,
) -> DiscoveryResult:
    """Enumerate candidate links and probe each one. ``also`` holds operator-supplied
    connection strings (validated); ``ports``, ``prober`` and ``platform`` are for tests."""
    extra = [Target(parse_connection_string(c), "operator") for c in also]
    targets: list[Target] = []
    skipped: list[str] = []
    if serial:
        targets, skipped = serial_targets(
            system_ports() if ports is None else ports, platform=platform
        )
    if udp_port is not None:
        targets.append(Target(parse_connection_string(f"udpin:0.0.0.0:{udp_port}"), "udp"))
    seen = {str(t.spec) for t in targets}
    targets += [t for t in extra if str(t.spec) not in seen]
    return DiscoveryResult(
        probe_timeout=timeout,
        network_timeout=network_timeout,
        candidates=_hint_local_network(
            _probe_all(
                targets,
                [timeout if t.spec.is_serial else network_timeout for t in targets],
                prober,
            ),
            platform,
        ),
        skipped=skipped,
    )
