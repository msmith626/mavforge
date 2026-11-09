"""Connecting to a link, selecting the vehicle, and identifying it."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from mavforge.domain.errors import NoVehicleError
from mavforge.domain.models import FirmwareVersion, LinkStats, VehicleIdentity
from mavforge.platforms.base import PlatformAdapter, select_adapter
from mavforge.policy.audit import AuditLog
from mavforge.policy.permissions import Policy
from mavforge.protocol.messages import request_messages
from mavforge.transport.connection import (
    DEFAULT_SOURCE_COMPONENT,
    DEFAULT_SOURCE_SYSTEM,
    MavlinkConnection,
)
from mavforge.transport.connstr import ConnectionSpec, parse_connection_string
from mavforge.transport.dialect import Message, enum_name, mavlink

HEARTBEAT_TIMEOUT = 5.0
"""Default heartbeat wait on serial links."""
NETWORK_HEARTBEAT_TIMEOUT = 20.0
"""Default heartbeat wait on UDP/TCP links. WiFi bridges are lossy: on real hardware
(2026-11-08) a SpeedyBee F405 WING's WiFi averaged ~47 msg/s but went silent for up to
5.1 s, and a freshly opened UDP listener took 3.6-13.6 s to hear its first packet
(macOS, -32 dBm signal), so 5 s and even 10 s waits regularly reported no vehicle."""
SERIAL_IDENTIFY_RETRIES = 1
NETWORK_IDENTIFY_RETRIES = 4
"""AUTOPILOT_VERSION re-requests on network links. Without them a lossy link often
yields no board ``uid``, which weakens the same-vehicle check in ``apply_plan``."""


@dataclass(frozen=True)
class ConnectOptions:
    connection: str
    target_system: int | None = None
    source_system: int = DEFAULT_SOURCE_SYSTEM
    source_component: int = DEFAULT_SOURCE_COMPONENT
    heartbeat_timeout: float | None = None
    """None picks the link default: HEARTBEAT_TIMEOUT (serial) or NETWORK_HEARTBEAT_TIMEOUT."""
    request_timeout: float = 1.5
    identify_retries: int | None = None
    """None picks the link default: SERIAL_IDENTIFY_RETRIES or NETWORK_IDENTIFY_RETRIES."""


def link_defaults(spec: ConnectionSpec) -> tuple[float, int]:
    """(heartbeat timeout, identify retries) suited to this kind of link."""
    if spec.is_serial:
        return HEARTBEAT_TIMEOUT, SERIAL_IDENTIFY_RETRIES
    return NETWORK_HEARTBEAT_TIMEOUT, NETWORK_IDENTIFY_RETRIES


@dataclass
class Session:
    """A connected, identified vehicle plus the policy and audit log governing it."""

    connection: MavlinkConnection
    identity: VehicleIdentity
    adapter: PlatformAdapter
    policy: Policy
    audit: AuditLog
    interface: str
    options: ConnectOptions
    _closed: bool = field(default=False, repr=False)

    @property
    def target(self) -> tuple[int, int]:
        return self.identity.system_id, self.identity.component_id

    def link_stats(self) -> LinkStats:
        seen = self.connection.latest("HEARTBEAT", *self.target)
        age = round(time.monotonic() - seen.monotonic, 3) if seen else None
        return LinkStats(
            connection=self.connection.description,
            messages_received=self.connection.messages_received,
            receive_errors=self.connection.receive_errors,
            seconds_since_heartbeat=age,
        )

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self.connection.close()


def _is_autopilot_heartbeat(msg: Message) -> bool:
    return (
        isinstance(msg, mavlink.MAVLink_heartbeat_message)
        and msg.type != mavlink.MAV_TYPE_GCS
        and msg.autopilot != mavlink.MAV_AUTOPILOT_INVALID
    )


def wait_for_vehicle(
    conn: MavlinkConnection, target_system: int | None, timeout: float
) -> tuple[mavlink.MAVLink_heartbeat_message, set[int]]:
    """Wait for an autopilot heartbeat, preferring component 1 (MAV_COMP_ID_AUTOPILOT1).

    GCS heartbeats (e.g. from MAVProxy on a shared link) are ignored. Returns the chosen
    heartbeat and every autopilot system ID heard while waiting.
    """
    seen: set[int] = set()
    fallback: mavlink.MAVLink_heartbeat_message | None = None
    with conn.subscribe(_is_autopilot_heartbeat) as sub:
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            msg = sub.get(remaining)
            if msg is None:
                break
            assert isinstance(msg, mavlink.MAVLink_heartbeat_message)
            sysid = msg.get_srcSystem()
            seen.add(sysid)
            if target_system is not None and sysid != target_system:
                continue
            if msg.get_srcComponent() == mavlink.MAV_COMP_ID_AUTOPILOT1:
                return msg, seen
            fallback = fallback or msg
    if fallback is not None:
        return fallback, seen
    wanted = f"system {target_system}" if target_system is not None else "an autopilot"
    raise NoVehicleError(
        f"No heartbeat from {wanted} within {timeout:g}s on {conn.description}",
        details={"autopilot_systems_seen": sorted(seen)},
    )


def _decode_git_hash(raw: list[int]) -> str | None:
    data = bytes(raw).rstrip(b"\0")
    if not data:
        return None
    text = data.decode("ascii", errors="replace")
    return text if text.isprintable() and "�" not in text else data.hex()


def _capability_names(bits: int) -> list[str]:
    entries = mavlink.enums["MAV_PROTOCOL_CAPABILITY"]
    return sorted(
        enum_name("MAV_PROTOCOL_CAPABILITY", value)
        for value, entry in entries.items()
        if value and bits & value == value and not entry.name.endswith("_ENUM_END")
    )


def identify(
    conn: MavlinkConnection,
    heartbeat: mavlink.MAVLink_heartbeat_message,
    adapter: PlatformAdapter,
    systems_seen: set[int],
    *,
    request_timeout: float,
    retries: int = 1,
) -> VehicleIdentity:
    sysid, compid = heartbeat.get_srcSystem(), heartbeat.get_srcComponent()
    wanted = [mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION]
    if adapter.simulation_probe_message is not None:
        wanted.append(adapter.simulation_probe_message)
    # Every request is retried once. Only AUTOPILOT_VERSION gets the extra retries: the
    # simulation probe is never answered by real hardware, so retrying it would just add
    # seconds to every real-vehicle connect.
    got = request_messages(conn, sysid, compid, wanted, timeout=request_timeout, retries=1)
    av = mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION
    if got[av] is None and retries > 1:
        got[av] = request_messages(
            conn, sysid, compid, [av], timeout=request_timeout, retries=retries - 2
        )[av]

    version = got[mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION]
    fields: dict[str, object] = {}
    if isinstance(version, mavlink.MAVLink_autopilot_version_message):
        uid2 = bytes(version.uid2)
        fields = {
            "firmware": FirmwareVersion.from_packed(version.flight_sw_version),
            "firmware_git_hash": _decode_git_hash(list(version.flight_custom_version)),
            "board_version": version.board_version,
            "vendor_id": version.vendor_id,
            "product_id": version.product_id,
            "uid": uid2.hex() if any(uid2) else (f"{version.uid:016x}" if version.uid else None),
            "protocol_capabilities": _capability_names(version.capabilities),
        }

    simulated: bool | None = None
    if adapter.simulation_probe_message is not None:
        simulated = got[adapter.simulation_probe_message] is not None

    others = (systems_seen | _heard_autopilots(conn)) - {sysid}
    return VehicleIdentity(
        system_id=sysid,
        component_id=compid,
        autopilot=enum_name("MAV_AUTOPILOT", heartbeat.autopilot),
        mav_type=enum_name("MAV_TYPE", heartbeat.type),
        vehicle_family=adapter.vehicle_family(heartbeat.type),
        platform=adapter.name,
        simulated=simulated,
        other_systems_seen=sorted(others),
        **fields,  # type: ignore[arg-type]
    )


def _heard_autopilots(conn: MavlinkConnection) -> set[int]:
    return {
        sysid
        for sysid in range(1, 256)
        if (seen := conn.latest("HEARTBEAT", sysid, mavlink.MAV_COMP_ID_AUTOPILOT1)) is not None
        and _is_autopilot_heartbeat(seen.msg)
    }


def open_session(
    options: ConnectOptions,
    *,
    policy: Policy | None = None,
    audit: AuditLog | None = None,
    interface: str = "api",
) -> Session:
    """Open the link, select and identify the vehicle. The caller must ``close()`` it."""
    spec = parse_connection_string(options.connection)
    conn = MavlinkConnection.open(
        spec, source_system=options.source_system, source_component=options.source_component
    )
    try:
        default_timeout, default_retries = link_defaults(spec)
        heartbeat, seen = wait_for_vehicle(
            conn,
            options.target_system,
            default_timeout if options.heartbeat_timeout is None else options.heartbeat_timeout,
        )
        adapter = select_adapter(heartbeat)
        identity = identify(
            conn,
            heartbeat,
            adapter,
            seen,
            request_timeout=options.request_timeout,
            retries=default_retries
            if options.identify_retries is None
            else options.identify_retries,
        )
    except BaseException:
        conn.close()
        raise
    return Session(
        connection=conn,
        identity=identity,
        adapter=adapter,
        policy=policy or Policy.read_only(),
        audit=audit if audit is not None else AuditLog.default(),
        interface=interface,
        options=options,
    )
