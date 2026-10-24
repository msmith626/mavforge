"""A scripted ArduPilot-like MAVLink peer for tests and demos.

``FakeVehicle`` listens on a localhost TCP port and speaks real MAVLink 2 bytes, so the
code under test runs through the same pymavlink transport it uses against SITL or
hardware. It implements only the behaviour MAVForge relies on, modelled on ArduPilot
(see docs/research.md), plus fault-injection knobs. It is a test double, not a
simulator: passing tests against it says nothing about real firmware behaviour.
"""

from __future__ import annotations

import contextlib
import socket
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from types import TracebackType
from typing import Self

from mavforge.domain.models import ParamType
from mavforge.testing.fake_calibration import FakeCalibration, FakeCalibrationConfig
from mavforge.transport.dialect import Message, mavlink

ParamSpec = tuple[float, ParamType]


def default_copter_params() -> dict[str, ParamSpec]:
    return {
        "FORMAT_VERSION": (120, ParamType.INT16),
        "MAV_SYSID": (1, ParamType.INT16),  # SYSID_THISMAV before ArduPilot 4.7
        "FRAME_CLASS": (1, ParamType.INT8),
        "FRAME_TYPE": (1, ParamType.INT8),
        "ARMING_SKIPCHK": (0, ParamType.INT32),
        "BATT_MONITOR": (4, ParamType.INT8),
        "BATT_CAPACITY": (3300, ParamType.INT32),
        "BRD_SERIAL_NUM": (33554433, ParamType.INT32),
        "COMPASS_OFS_X": (5.0, ParamType.REAL32),
        "COMPASS_OFS_Y": (-12.5, ParamType.REAL32),
        "COMPASS_OFS_Z": (30.25, ParamType.REAL32),
        "COMPASS_USE": (1, ParamType.INT8),
        "INS_ACCOFFS_X": (0.001, ParamType.REAL32),
        "INS_ACCOFFS_Y": (-0.002, ParamType.REAL32),
        "INS_ACCOFFS_Z": (0.003, ParamType.REAL32),
        "INS_GYR_CAL": (1, ParamType.INT8),
        "LOG_BITMASK": (176126, ParamType.INT32),
        "RC1_MIN": (1100, ParamType.INT16),
        "RC1_MAX": (1900, ParamType.INT16),
        "RC1_TRIM": (1500, ParamType.INT16),
        "SERIAL0_BAUD": (115, ParamType.INT32),
        # Appended (indices above are relied on by tests): a calibrated, configured copter.
        "INS_ACC_ID": (2753028, ParamType.INT32),
        "INS_ACCSCAL_X": (1.001, ParamType.REAL32),
        "INS_ACCSCAL_Y": (0.998, ParamType.REAL32),
        "INS_ACCSCAL_Z": (1.002, ParamType.REAL32),
        "COMPASS_DEV_ID": (97539, ParamType.INT32),
        "COMPASS_DEV_ID2": (0, ParamType.INT32),
        "COMPASS_DEV_ID3": (0, ParamType.INT32),
        "COMPASS_LEARN": (0, ParamType.INT8),
        "RCMAP_ROLL": (1, ParamType.INT8),
        "RCMAP_PITCH": (2, ParamType.INT8),
        "RCMAP_THROTTLE": (3, ParamType.INT8),
        "RCMAP_YAW": (4, ParamType.INT8),
        "RC2_MIN": (1101, ParamType.INT16),
        "RC2_MAX": (1899, ParamType.INT16),
        "RC2_TRIM": (1502, ParamType.INT16),
        "RC3_MIN": (1098, ParamType.INT16),
        "RC3_MAX": (1902, ParamType.INT16),
        "RC3_TRIM": (1098, ParamType.INT16),
        "RC4_MIN": (1103, ParamType.INT16),
        "RC4_MAX": (1897, ParamType.INT16),
        "RC4_TRIM": (1499, ParamType.INT16),
        "FS_THR_ENABLE": (1, ParamType.INT8),
        "BATT_FS_LOW_ACT": (2, ParamType.INT8),
        "BATT_FS_CRT_ACT": (1, ParamType.INT8),
        "BATT_LOW_VOLT": (14.0, ParamType.REAL32),
        "BATT_CRT_VOLT": (13.2, ParamType.REAL32),
        "GPS1_TYPE": (1, ParamType.INT8),
        "LOG_BACKEND_TYPE": (1, ParamType.INT8),
        "INS_GYROFFS_X": (0.0011, ParamType.REAL32),
        "INS_GYROFFS_Y": (-0.0006, ParamType.REAL32),
        "INS_GYROFFS_Z": (0.0002, ParamType.REAL32),
        "AHRS_TRIM_X": (0.0, ParamType.REAL32),
        "AHRS_TRIM_Y": (0.0, ParamType.REAL32),
        "AHRS_ORIENTATION": (0, ParamType.INT8),
    }


_SENSORS_PRESENT = (
    mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
    | mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL
    | mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG
    | mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
    | mavlink.MAV_SYS_STATUS_SENSOR_GPS
    | mavlink.MAV_SYS_STATUS_SENSOR_BATTERY
    | mavlink.MAV_SYS_STATUS_PREARM_CHECK
)


@dataclass
class FakeVehicleConfig:
    system_id: int = 1
    component_id: int = 1
    mav_type: int = mavlink.MAV_TYPE_QUADROTOR
    autopilot: int = mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
    firmware: tuple[int, int, int, int] = (4, 7, 1, 255)
    git_hash: str = "abcdef12"
    simulated: bool = True
    armed: bool = False
    custom_mode: int = 0
    params: dict[str, ParamSpec] = field(default_factory=default_copter_params)
    heartbeat_interval: float = 0.2
    respond_to_requests: bool = True
    """False: heartbeat only; ignore every request (models a deaf or busy vehicle)."""
    silent: bool = False
    """True: send nothing at all (models a dead link)."""
    drop_param_indices_once: set[int] = field(default_factory=set[int])
    """Indices skipped during the first PARAM_REQUEST_LIST stream (models packet loss)."""
    withheld_param_indices: set[int] = field(default_factory=set[int])
    """Indices never sent, even when requested individually (models a broken link)."""
    statustext_on_connect: list[str] = field(default_factory=list[str])
    sensors_unhealthy: int = 0
    prearm_passing: bool = True
    param_write_errors: dict[str, int] = field(default_factory=dict[str, int])
    """PARAM_SET for these names is answered with PARAM_ERROR(code) and not applied."""
    param_constraints: dict[str, tuple[float, float]] = field(
        default_factory=dict[str, tuple[float, float]]
    )
    """Written values are clamped to these ranges, as ArduPilot does for some params."""
    ignore_param_set: set[str] = field(default_factory=set[str])
    """PARAM_SET for these names gets no reply and changes nothing."""
    no_echo_param_set: set[str] = field(default_factory=set[str])
    """PARAM_SET for these names is applied but not echoed."""
    drop_link_after_sets: int | None = None
    """Close the connection right after this many applied PARAM_SETs (link loss)."""
    calibration: FakeCalibrationConfig = field(default_factory=FakeCalibrationConfig)
    uptime_s: float = 600.0
    """Seconds since 'boot' reported in SYSTEM_TIME when the fake starts."""
    prearm_failures: list[str] = field(default_factory=list[str])
    """Reasons sent as 'PreArm: <reason>' STATUSTEXT when asked to run pre-arm checks."""
    prearm_checks_enabled: bool = True
    """False models ARMING_CHECK=0: the pre-arm bit is present but not enabled."""
    gps_fix_type: int = mavlink.GPS_FIX_TYPE_3D_FIX
    gps_eph: int = 80
    gps_satellites: int = 14
    unanswered_message_ids: set[int] = field(default_factory=set[int])
    """REQUEST_MESSAGE ids that are ACKed but never sent (models a missing stream)."""
    rc_inputs: list[int] = field(
        default_factory=lambda: [1500, 1500, 1100, 1500, 1100, 1100, 1500, 1500]
    )
    """PWM per RC input channel reported in RC_CHANNELS; empty models no receiver."""
    accel_mg: tuple[float, float, float] = (0.0, 0.0, -1000.0)
    """Accelerometer reading (milli-g, body frame as configured) reported in SCALED_IMU."""
    accel_jitter_mg: float = 0.0
    """Added to X with alternating sign on each SCALED_IMU (models a shaking vehicle)."""
    other_autopilot_sysids: list[int] = field(default_factory=list[int])
    gcs_heartbeat_sysids: list[int] = field(default_factory=list[int])


class _SocketWriter:
    def __init__(self, sock: socket.socket) -> None:
        self._sock = sock

    def write(self, data: bytes) -> None:
        self._sock.sendall(data)


class FakeVehicle:
    """Serve one or more sequential/concurrent TCP clients until ``stop()``."""

    def __init__(self, config: FakeVehicleConfig | None = None) -> None:
        self.config = config or FakeVehicleConfig()
        self._server = socket.create_server(("127.0.0.1", 0))
        self._server.settimeout(0.1)
        self.port: int = self._server.getsockname()[1]
        self._started = time.monotonic()
        self._imu_count = 0
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._received: list[Message] = []
        self._param_list_requests = 0
        self.calibration = FakeCalibration(self, self.config.calibration)
        self._sets_applied = 0
        self._clients: list[socket.socket] = []
        self._threads: list[threading.Thread] = []
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

    @property
    def connection_string(self) -> str:
        return f"tcp:127.0.0.1:{self.port}"

    @property
    def received(self) -> list[Message]:
        """Every message the fake has received from clients, in order."""
        with self._lock:
            return list(self._received)

    def received_types(self) -> set[str]:
        return {m.get_type() for m in self.received}

    # -- server plumbing -------------------------------------------------------------

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                sock, _ = self._server.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            sock.settimeout(0.02)
            with self._lock:
                self._clients.append(sock)
            t = threading.Thread(target=self._serve, args=(sock,), daemon=True)
            self._threads.append(t)
            t.start()

    def _codec(self, sock: socket.socket, sysid: int, compid: int) -> mavlink.MAVLink:
        codec = mavlink.MAVLink(_SocketWriter(sock), srcSystem=sysid, srcComponent=compid)
        codec.robust_parsing = True
        return codec

    def _serve(self, sock: socket.socket) -> None:
        cfg = self.config
        mav = self._codec(sock, cfg.system_id, cfg.component_id)
        others = [self._codec(sock, s, 1) for s in cfg.other_autopilot_sysids]
        gcses = [self._codec(sock, s, 190) for s in cfg.gcs_heartbeat_sysids]
        next_hb = 0.0
        try:
            if not cfg.silent:
                for text in cfg.statustext_on_connect:
                    self.send_statustext(mav, text)
            while not self._stop.is_set():
                now = time.monotonic()
                if now >= next_hb and not cfg.silent:
                    self._send_heartbeat(mav, others, gcses)
                    next_hb = now + cfg.heartbeat_interval
                if not cfg.silent:
                    self.calibration.tick(mav)
                try:
                    data = sock.recv(4096)
                except TimeoutError:
                    continue
                if not data:
                    return
                for msg in mav.parse_buffer(data) or []:
                    with self._lock:
                        self._received.append(msg)
                    if cfg.respond_to_requests and not cfg.silent:
                        self._handle(mav, msg)
        except OSError:
            return
        finally:
            sock.close()

    def stop(self) -> None:
        self._stop.set()
        self._server.close()
        with self._lock:
            clients = list(self._clients)
        for sock in clients:
            with contextlib.suppress(OSError):
                sock.shutdown(socket.SHUT_RDWR)
        self._accept_thread.join(timeout=2)
        for t in self._threads:
            t.join(timeout=2)

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.stop()

    # -- outgoing messages -----------------------------------------------------------

    def _send_heartbeat(
        self, mav: mavlink.MAVLink, others: list[mavlink.MAVLink], gcses: list[mavlink.MAVLink]
    ) -> None:
        cfg = self.config
        base_mode = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if cfg.armed:
            base_mode |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        state = mavlink.MAV_STATE_ACTIVE if cfg.armed else mavlink.MAV_STATE_STANDBY
        mav.heartbeat_send(cfg.mav_type, cfg.autopilot, base_mode, cfg.custom_mode, state)
        for other in others:
            other.heartbeat_send(cfg.mav_type, cfg.autopilot, base_mode, 0, state)
        for gcs in gcses:
            gcs.heartbeat_send(
                mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavlink.MAV_STATE_ACTIVE
            )

    def send_statustext(
        self, mav: mavlink.MAVLink, text: str, severity: int = mavlink.MAV_SEVERITY_INFO
    ) -> None:
        mav.statustext_send(severity, text.encode()[:50])

    def set_param(self, name: str, value: float, ptype: ParamType) -> None:
        """Change or add a parameter, as if configured by another tool. Thread-safe."""
        with self._lock:
            self.config.params[name] = (value, ptype)

    def set_rc(self, *values: int) -> None:
        """Set the RC input PWM of channels 1.. (models the person moving the sticks)."""
        self.config.rc_inputs = list(values) + self.config.rc_inputs[len(values) :]

    def set_accel(self, x: float, y: float, z: float) -> None:
        """Set the accelerometer reading (models the person tilting the vehicle)."""
        self.config.accel_mg = (x, y, z)

    def remove_param(self, name: str) -> None:
        """Remove a parameter, as if a firmware change dropped it. Thread-safe."""
        with self._lock:
            del self.config.params[name]

    def _param_snapshot(self) -> list[tuple[str, ParamSpec]]:
        with self._lock:
            return list(self.config.params.items())

    def _param_value(
        self, mav: mavlink.MAVLink, name: str, spec: ParamSpec, index: int, count: int
    ) -> None:
        value, ptype = spec
        mav.param_value_send(
            name.encode(),
            float(value),  # ArduPilot "C cast" encoding: every type travels as a float
            getattr(mavlink, f"MAV_PARAM_TYPE_{ptype.value}"),
            count,
            index,
        )

    def ack(self, mav: mavlink.MAVLink, req: Message, command: int, result: int) -> None:
        mav.command_ack_send(command, result, 0, 0, req.get_srcSystem(), req.get_srcComponent())

    def _send_requested(self, mav: mavlink.MAVLink, msg_id: int) -> bool:
        cfg = self.config
        if msg_id == mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION:
            major, minor, patch, kind = cfg.firmware
            mav.autopilot_version_send(
                mavlink.MAV_PROTOCOL_CAPABILITY_MAVLINK2
                | mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT
                | mavlink.MAV_PROTOCOL_CAPABILITY_FTP
                | mavlink.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION,
                (major << 24) | (minor << 16) | (patch << 8) | kind,
                0,
                0,
                0x00140000,
                list(cfg.git_hash.encode()[:8].ljust(8, b"\0")),
                [0] * 8,
                [0] * 8,
                0x1209,
                0x5741,
                0x0123456789ABCDEF,
            )
        elif msg_id == mavlink.MAVLINK_MSG_ID_SYS_STATUS:
            enabled = _SENSORS_PRESENT
            if not cfg.prearm_checks_enabled:
                enabled &= ~mavlink.MAV_SYS_STATUS_PREARM_CHECK
            health = _SENSORS_PRESENT & ~cfg.sensors_unhealthy
            if not cfg.prearm_passing:
                health &= ~mavlink.MAV_SYS_STATUS_PREARM_CHECK
            mav.sys_status_send(
                _SENSORS_PRESENT, enabled, health, 150, 12_600, 150, 87, 0, 0, 0, 0, 0, 0
            )
        elif msg_id == mavlink.MAVLINK_MSG_ID_GPS_RAW_INT:
            mav.gps_raw_int_send(
                0,
                cfg.gps_fix_type,
                -353632620,
                1491652370,
                584000,
                cfg.gps_eph,
                120,
                0,
                0,
                cfg.gps_satellites,
            )
        elif msg_id == mavlink.MAVLINK_MSG_ID_SYSTEM_TIME:
            boot_ms = int((cfg.uptime_s + time.monotonic() - self._started) * 1000)
            mav.system_time_send(0, boot_ms & 0xFFFFFFFF)
        elif msg_id == mavlink.MAVLINK_MSG_ID_RC_CHANNELS:
            values = list(cfg.rc_inputs[:18])
            padded = values + [65535] * (18 - len(values))
            boot_ms = int((cfg.uptime_s + time.monotonic() - self._started) * 1000)
            chans = {f"chan{i + 1}_raw": v for i, v in enumerate(padded)}
            mav.send(
                mavlink.MAVLink_rc_channels_message(
                    time_boot_ms=boot_ms & 0xFFFFFFFF, chancount=len(values), rssi=255, **chans
                )
            )
        elif msg_id == mavlink.MAVLINK_MSG_ID_SCALED_IMU:
            self._imu_count += 1
            jitter = cfg.accel_jitter_mg * (1 if self._imu_count % 2 else -1)
            x, y, z = cfg.accel_mg
            boot_ms = int((cfg.uptime_s + time.monotonic() - self._started) * 1000)
            mav.scaled_imu_send(
                boot_ms & 0xFFFFFFFF, round(x + jitter), round(y), round(z), 0, 0, 0, 0, 0, 0
            )
        elif msg_id == mavlink.MAVLINK_MSG_ID_SIMSTATE and cfg.simulated:
            mav.simstate_send(0, 0, 0, 0, 0, 0, 0, 0, 0, -353632620, 1491652370)
        else:
            return False
        return True

    # -- request handling ------------------------------------------------------------

    def _addressed_to_me(self, msg: Message) -> bool:
        target = getattr(msg, "target_system", 0)
        return target in (0, self.config.system_id)

    def _handle(self, mav: mavlink.MAVLink, msg: Message) -> None:
        if not self._addressed_to_me(msg):
            return
        if isinstance(msg, mavlink.MAVLink_command_long_message):
            self._handle_command(mav, msg)
        elif isinstance(msg, mavlink.MAVLink_param_request_list_message):
            self._handle_param_list(mav)
        elif isinstance(msg, mavlink.MAVLink_param_request_read_message):
            self._handle_param_read(mav, msg)
        elif isinstance(msg, mavlink.MAVLink_param_set_message):
            self._handle_param_set(mav, msg)

    def _handle_command(
        self, mav: mavlink.MAVLink, msg: mavlink.MAVLink_command_long_message
    ) -> None:
        cmd = msg.command
        if self.calibration.handle_command(mav, msg):
            return
        if cmd == mavlink.MAV_CMD_REQUEST_MESSAGE:
            msg_id = int(msg.param1)
            if msg_id in (
                mavlink.MAVLINK_MSG_ID_MAG_CAL_PROGRESS,
                mavlink.MAVLINK_MSG_ID_MAG_CAL_REPORT,
            ):
                self.ack(mav, msg, cmd, mavlink.MAV_RESULT_ACCEPTED)
                self.calibration.handle_request(mav, msg_id)
                return
            if msg_id in self.config.unanswered_message_ids:
                self.ack(mav, msg, cmd, mavlink.MAV_RESULT_ACCEPTED)
                return
            # Like ArduPilot: ACK the request, then send the message itself
            known = msg_id in (
                mavlink.MAVLINK_MSG_ID_SYSTEM_TIME,
                mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                mavlink.MAVLINK_MSG_ID_SYS_STATUS,
                mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
                mavlink.MAVLINK_MSG_ID_SCALED_IMU,
            ) or (msg_id == mavlink.MAVLINK_MSG_ID_SIMSTATE and self.config.simulated)
            self.ack(
                mav, msg, cmd, mavlink.MAV_RESULT_ACCEPTED if known else mavlink.MAV_RESULT_FAILED
            )
            if known:
                self._send_requested(mav, msg_id)
        elif cmd == mavlink.MAV_CMD_RUN_PREARM_CHECKS:
            # Like ArduPilot (GCS_Common.cpp): refused while armed, else report failures
            if self.config.armed:
                self.ack(mav, msg, cmd, mavlink.MAV_RESULT_TEMPORARILY_REJECTED)
                return
            self.ack(mav, msg, cmd, mavlink.MAV_RESULT_ACCEPTED)
            for reason in self.config.prearm_failures:
                self.send_statustext(mav, f"PreArm: {reason}", mavlink.MAV_SEVERITY_CRITICAL)
        elif cmd == mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            self.ack(mav, msg, cmd, mavlink.MAV_RESULT_ACCEPTED)
            self._send_requested(mav, mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
        else:
            self.ack(mav, msg, cmd, mavlink.MAV_RESULT_UNSUPPORTED)

    def _handle_param_list(self, mav: mavlink.MAVLink) -> None:
        with self._lock:
            first = self._param_list_requests == 0
            self._param_list_requests += 1
        params = self._param_snapshot()
        for index, (name, spec) in enumerate(params):
            if first and index in self.config.drop_param_indices_once:
                continue
            if index in self.config.withheld_param_indices:
                continue
            self._param_value(mav, name, spec, index, len(params))

    def _handle_param_set(
        self, mav: mavlink.MAVLink, msg: mavlink.MAVLink_param_set_message
    ) -> None:
        cfg = self.config
        name = str(msg.param_id).rstrip("\x00")
        if name in cfg.ignore_param_set:
            return
        if name in cfg.param_write_errors or name not in dict(self._param_snapshot()):
            code = cfg.param_write_errors.get(name, mavlink.MAV_PARAM_ERROR_DOES_NOT_EXIST)
            mav.param_error_send(
                msg.get_srcSystem(), msg.get_srcComponent(), name.encode(), -1, code
            )
            return
        value = float(msg.param_value)
        if name in cfg.param_constraints:
            lo, hi = cfg.param_constraints[name]
            value = min(max(value, lo), hi)
        with self._lock:
            _, ptype = self.config.params[name]  # ArduPilot keeps the stored type
            self.config.params[name] = (value, ptype)
            self._sets_applied += 1
            drop = (
                cfg.drop_link_after_sets is not None
                and self._sets_applied >= cfg.drop_link_after_sets
            )
        if name not in cfg.no_echo_param_set:
            # Like AP_Param::save -> GCS_SEND_PARAM: broadcast with index 65535
            self._param_value(mav, name, (value, ptype), 65535, len(self._param_snapshot()))
        if drop:
            raise ConnectionAbortedError("fake vehicle dropped the link")

    def _handle_param_read(
        self, mav: mavlink.MAVLink, msg: mavlink.MAVLink_param_request_read_message
    ) -> None:
        params = self._param_snapshot()
        names = [name for name, _ in params]
        param_id = str(msg.param_id).rstrip("\x00")
        if msg.param_index >= 0:
            if msg.param_index in self.config.withheld_param_indices:
                return
            if msg.param_index < len(params):
                name, spec = params[msg.param_index]
                self._param_value(mav, name, spec, msg.param_index, len(params))
            return
        if param_id in names:
            index = names.index(param_id)
            self._param_value(mav, param_id, params[index][1], index, len(params))
        else:
            mav.param_error_send(
                msg.get_srcSystem(),
                msg.get_srcComponent(),
                param_id.encode(),
                -1,
                mavlink.MAV_PARAM_ERROR_DOES_NOT_EXIST,
            )


FakeFactory = Callable[..., FakeVehicle]
"""Signature of the ``make_fake`` pytest fixture: keyword args become ``FakeVehicleConfig``."""
