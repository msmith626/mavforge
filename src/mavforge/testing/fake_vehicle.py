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
    statustext_on_connect: list[str] = field(default_factory=list[str])
    sensors_unhealthy: int = 0
    prearm_passing: bool = True
    unanswered_message_ids: set[int] = field(default_factory=set[int])
    """REQUEST_MESSAGE ids that are ACKed but never sent (models a missing stream)."""
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
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._received: list[Message] = []
        self._param_list_requests = 0
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
                    self._send_statustext(mav, text)
            while not self._stop.is_set():
                now = time.monotonic()
                if now >= next_hb and not cfg.silent:
                    self._send_heartbeat(mav, others, gcses)
                    next_hb = now + cfg.heartbeat_interval
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

    def _send_statustext(
        self, mav: mavlink.MAVLink, text: str, severity: int = mavlink.MAV_SEVERITY_INFO
    ) -> None:
        mav.statustext_send(severity, text.encode()[:50])

    def _param_value(self, mav: mavlink.MAVLink, name: str, index: int) -> None:
        value, ptype = self.config.params[name]
        mav.param_value_send(
            name.encode(),
            float(value),  # ArduPilot "C cast" encoding: every type travels as a float
            getattr(mavlink, f"MAV_PARAM_TYPE_{ptype.value}"),
            len(self.config.params),
            index,
        )

    def _ack(self, mav: mavlink.MAVLink, req: Message, command: int, result: int) -> None:
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
            health = _SENSORS_PRESENT & ~cfg.sensors_unhealthy
            if not cfg.prearm_passing:
                health &= ~mavlink.MAV_SYS_STATUS_PREARM_CHECK
            mav.sys_status_send(
                _SENSORS_PRESENT, enabled, health, 150, 12_600, 150, 87, 0, 0, 0, 0, 0, 0
            )
        elif msg_id == mavlink.MAVLINK_MSG_ID_GPS_RAW_INT:
            mav.gps_raw_int_send(
                0, mavlink.GPS_FIX_TYPE_3D_FIX, -353632620, 1491652370, 584000, 80, 120, 0, 0, 14
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

    def _handle_command(
        self, mav: mavlink.MAVLink, msg: mavlink.MAVLink_command_long_message
    ) -> None:
        cmd = msg.command
        if cmd == mavlink.MAV_CMD_REQUEST_MESSAGE:
            msg_id = int(msg.param1)
            if msg_id in self.config.unanswered_message_ids:
                self._ack(mav, msg, cmd, mavlink.MAV_RESULT_ACCEPTED)
                return
            # Like ArduPilot: ACK the request, then send the message itself
            known = msg_id in (
                mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                mavlink.MAVLINK_MSG_ID_SYS_STATUS,
                mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            ) or (msg_id == mavlink.MAVLINK_MSG_ID_SIMSTATE and self.config.simulated)
            self._ack(
                mav, msg, cmd, mavlink.MAV_RESULT_ACCEPTED if known else mavlink.MAV_RESULT_FAILED
            )
            if known:
                self._send_requested(mav, msg_id)
        elif cmd == mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            self._ack(mav, msg, cmd, mavlink.MAV_RESULT_ACCEPTED)
            self._send_requested(mav, mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
        else:
            self._ack(mav, msg, cmd, mavlink.MAV_RESULT_UNSUPPORTED)

    def _handle_param_list(self, mav: mavlink.MAVLink) -> None:
        with self._lock:
            first = self._param_list_requests == 0
            self._param_list_requests += 1
        for index, name in enumerate(self.config.params):
            if first and index in self.config.drop_param_indices_once:
                continue
            self._param_value(mav, name, index)

    def _handle_param_read(
        self, mav: mavlink.MAVLink, msg: mavlink.MAVLink_param_request_read_message
    ) -> None:
        names = list(self.config.params)
        param_id = str(msg.param_id).rstrip("\x00")
        if msg.param_index >= 0:
            if msg.param_index < len(names):
                self._param_value(mav, names[msg.param_index], msg.param_index)
            return
        if param_id in self.config.params:
            self._param_value(mav, param_id, names.index(param_id))
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
