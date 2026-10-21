"""Vehicle status snapshot."""

from __future__ import annotations

import time

from mavforge.domain.errors import ConnectionFailedError
from mavforge.domain.models import (
    BatteryStatus,
    GpsStatus,
    SensorHealth,
    StatusReport,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.protocol.messages import request_messages
from mavforge.services.catalog import STATUS, run_operation
from mavforge.services.session import Session
from mavforge.transport.dialect import Message, enum_name, mavlink

HEARTBEAT_FRESH_S = 3.0
STATUSTEXT_LIMIT = 20

_UINT16_UNKNOWN = 0xFFFF


def _sensor_bits() -> list[int]:
    return [
        value
        for value, entry in mavlink.enums["MAV_SYS_STATUS_SENSOR"].items()
        if value
        and value & (value - 1) == 0  # single bit
        and value != mavlink.MAV_SYS_STATUS_PREARM_CHECK
        and not entry.name.endswith(("_ENUM_END", "_EXTENSION_USED"))
    ]


def _sensors(sys_status: mavlink.MAVLink_sys_status_message) -> list[SensorHealth]:
    return [
        SensorHealth(
            sensor=enum_name("MAV_SYS_STATUS_SENSOR", bit),
            enabled=bool(sys_status.onboard_control_sensors_enabled & bit),
            healthy=bool(sys_status.onboard_control_sensors_health & bit),
        )
        for bit in _sensor_bits()
        if sys_status.onboard_control_sensors_present & bit
    ]


def _prearm(sys_status: mavlink.MAVLink_sys_status_message) -> bool | None:
    bit = mavlink.MAV_SYS_STATUS_PREARM_CHECK
    if not sys_status.onboard_control_sensors_present & bit:
        return None
    if not sys_status.onboard_control_sensors_enabled & bit:
        return None
    return bool(sys_status.onboard_control_sensors_health & bit)


def _battery(sys_status: mavlink.MAVLink_sys_status_message) -> BatteryStatus:
    mv, ca, pct = (
        sys_status.voltage_battery,
        sys_status.current_battery,
        sys_status.battery_remaining,
    )
    return BatteryStatus(
        voltage_v=None if mv == _UINT16_UNKNOWN else round(mv / 1000, 3),
        current_a=None if ca == -1 else round(ca / 100, 2),
        remaining_pct=None if pct == -1 else pct,
    )


def _gps(gps: mavlink.MAVLink_gps_raw_int_message) -> GpsStatus:
    return GpsStatus(
        fix_type=enum_name("GPS_FIX_TYPE", gps.fix_type),
        satellites_visible=None if gps.satellites_visible == 255 else gps.satellites_visible,
        hdop=None if gps.eph == _UINT16_UNKNOWN else round(gps.eph / 100, 2),
    )


def _fresh_heartbeat(session: Session) -> mavlink.MAVLink_heartbeat_message:
    conn, (sysid, compid) = session.connection, session.target
    seen = conn.latest("HEARTBEAT", sysid, compid)
    if seen and time.monotonic() - seen.monotonic < HEARTBEAT_FRESH_S:
        assert isinstance(seen.msg, mavlink.MAVLink_heartbeat_message)
        return seen.msg

    def match(msg: Message) -> bool:
        return (
            isinstance(msg, mavlink.MAVLink_heartbeat_message)
            and msg.get_srcSystem() == sysid
            and msg.get_srcComponent() == compid
        )

    with conn.subscribe(match) as sub:
        msg = sub.get(HEARTBEAT_FRESH_S)
    if msg is None:
        raise ConnectionFailedError(
            f"No heartbeat from system {sysid} for {HEARTBEAT_FRESH_S:g}s; link may be lost"
        )
    assert isinstance(msg, mavlink.MAVLink_heartbeat_message)
    return msg


def snapshot(session: Session) -> StatusReport:
    def run() -> StatusReport:
        heartbeat = _fresh_heartbeat(session)
        got = request_messages(
            session.connection,
            *session.target,
            [mavlink.MAVLINK_MSG_ID_SYS_STATUS, mavlink.MAVLINK_MSG_ID_GPS_RAW_INT],
            timeout=session.options.request_timeout,
        )
        sys_status = got[mavlink.MAVLINK_MSG_ID_SYS_STATUS]
        gps = got[mavlink.MAVLINK_MSG_ID_GPS_RAW_INT]
        unavailable = [
            name for name, msg in (("SYS_STATUS", sys_status), ("GPS_RAW_INT", gps)) if msg is None
        ]
        have_sys = isinstance(sys_status, mavlink.MAVLink_sys_status_message)
        return StatusReport(
            status=OperationStatus.PARTIAL if unavailable else OperationStatus.SUCCEEDED,
            vehicle=session.identity,
            armed=bool(heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
            mode=session.adapter.mode_name(heartbeat),
            system_status=enum_name("MAV_STATE", heartbeat.system_status),
            prearm_checks_passing=_prearm(sys_status) if have_sys else None,  # type: ignore[arg-type]
            sensors=_sensors(sys_status) if have_sys else [],  # type: ignore[arg-type]
            battery=_battery(sys_status) if have_sys else None,  # type: ignore[arg-type]
            gps=_gps(gps) if isinstance(gps, mavlink.MAVLink_gps_raw_int_message) else None,
            recent_statustext=session.connection.statustexts(session.identity.system_id)[
                -STATUSTEXT_LIMIT:
            ],
            unavailable=unavailable,
            link=session.link_stats(),
        )

    return run_operation(session, STATUS, run)
