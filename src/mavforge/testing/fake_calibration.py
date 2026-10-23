"""Calibration behaviour for ``FakeVehicle``, modelled on ArduPilot (see docs/research.md).

Like the rest of the fake, this is a protocol double for tests: it produces the messages
MAVForge relies on, in ArduPilot's order, with fault knobs. It does not model physics.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from mavforge.domain.models import ParamType
from mavforge.transport.dialect import mavlink

if TYPE_CHECKING:
    from mavforge.testing.fake_vehicle import FakeVehicle

POS_SUCCESS = 16777215
POS_FAILED = 16777216


@dataclass
class FakeCalibrationConfig:
    gyro_result: int = mavlink.MAV_RESULT_ACCEPTED
    level_result: int = mavlink.MAV_RESULT_ACCEPTED
    command_delay_s: float = 0.2
    """Time the synchronous gyro/level calibration takes before the ACK."""
    mag_duration_s: float = 1.0
    mag_statuses: list[int] = field(default_factory=lambda: [4])
    """Final MAG_CAL_STATUS per compass (4 = SUCCESS). The length is the compass count."""
    mag_save_on_accept: bool = True
    accel_fail: bool = False
    accel_stop_after_pose: int | None = None
    """Stop sending position requests after this pose (models a vehicle gone quiet)."""
    accel_sample_s: float = 0.1


class FakeCalibration:
    def __init__(self, vehicle: FakeVehicle, config: FakeCalibrationConfig) -> None:
        self.vehicle = vehicle
        self.config = config
        self.mag_started: float | None = None
        self.mag_accepted = False
        self._next_progress = 0
        self._next_report = 0
        self.accel_step = 0
        self.accel_waiting = False
        self.accel_sample_until = 0.0
        self.accel_result: int | None = None
        self.accel_result_sent = 0
        self._last_pos_request = 0.0

    # -- commands -------------------------------------------------------------------

    def handle_command(
        self, mav: mavlink.MAVLink, msg: mavlink.MAVLink_command_long_message
    ) -> bool:
        cfg = self.config
        cmd = msg.command
        if cmd == mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
            if self.vehicle.config.armed:
                self.vehicle.send_statustext(mav, "Disarm to allow calibration")
                self.ack(mav, msg, mavlink.MAV_RESULT_FAILED)
            elif msg.param1 == 1:
                time.sleep(cfg.command_delay_s)
                if cfg.gyro_result == mavlink.MAV_RESULT_ACCEPTED:
                    for axis, value in zip("XYZ", (0.0012, -0.0007, 0.0003), strict=True):
                        self.vehicle.set_param(f"INS_GYROFFS_{axis}", value, ParamType.REAL32)
                self.ack(mav, msg, cfg.gyro_result)
            elif int(msg.param5) == 2:
                time.sleep(cfg.command_delay_s)
                if cfg.level_result == mavlink.MAV_RESULT_ACCEPTED:
                    self.vehicle.set_param("AHRS_TRIM_X", 0.012, ParamType.REAL32)
                    self.vehicle.set_param("AHRS_TRIM_Y", -0.008, ParamType.REAL32)
                self.ack(mav, msg, cfg.level_result)
            elif int(msg.param5) == 1:
                time.sleep(cfg.command_delay_s)  # gyro calibration runs first
                self.ack(mav, msg, mavlink.MAV_RESULT_ACCEPTED)
                self.accel_step, self.accel_waiting, self.accel_result = 1, True, None
                self._last_pos_request = 0.0
            else:
                self.ack(mav, msg, mavlink.MAV_RESULT_UNSUPPORTED)
            return True
        if cmd == mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS:
            if self.accel_waiting and int(msg.param1) == self.accel_step:
                self.accel_waiting = False
                self.accel_sample_until = time.monotonic() + cfg.accel_sample_s
                self.ack(mav, msg, mavlink.MAV_RESULT_ACCEPTED)
            else:
                self.ack(mav, msg, mavlink.MAV_RESULT_FAILED)
            return True
        if cmd == mavlink.MAV_CMD_DO_START_MAG_CAL:
            self.mag_started, self.mag_accepted = time.monotonic(), False
            self.ack(mav, msg, mavlink.MAV_RESULT_ACCEPTED)
            return True
        if cmd == mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL:
            ok = self._mag_done() and all(s == 4 for s in cfg.mag_statuses)
            if ok and cfg.mag_save_on_accept:
                for i in range(len(cfg.mag_statuses)):
                    suffix = "" if i == 0 else str(i + 1)
                    for axis, value in zip("XYZ", self._mag_offsets(i), strict=True):
                        self.vehicle.set_param(
                            f"COMPASS_OFS{suffix}_{axis}", value, ParamType.REAL32
                        )
                self.mag_accepted = True
            self.ack(mav, msg, mavlink.MAV_RESULT_ACCEPTED if ok else mavlink.MAV_RESULT_FAILED)
            return True
        if cmd == mavlink.MAV_CMD_DO_CANCEL_MAG_CAL:
            self.mag_started = None
            self.ack(mav, msg, mavlink.MAV_RESULT_ACCEPTED)
            return True
        return False

    def handle_request(self, mav: mavlink.MAVLink, msg_id: int) -> bool:
        """REQUEST_MESSAGE for MAG_CAL_PROGRESS / MAG_CAL_REPORT. True if handled."""
        n = len(self.config.mag_statuses)
        if msg_id == mavlink.MAVLINK_MSG_ID_MAG_CAL_PROGRESS:
            if self.mag_started is not None and not self._mag_done():
                i = self._next_progress % n
                self._next_progress += 1
                pct = int(100 * (time.monotonic() - self.mag_started) / self.config.mag_duration_s)
                status = 2 if pct < 50 else 3
                mav.mag_cal_progress_send(
                    i, (1 << n) - 1, status, 1, min(pct, 99), [0] * 10, 0, 0, 0
                )
            return True
        if msg_id == mavlink.MAVLINK_MSG_ID_MAG_CAL_REPORT:
            if self.mag_started is not None and self._mag_done():
                # Like Compass::send_mag_cal_report: one request sends a report for every
                # finished compass, starting after the last one sent.
                for _ in range(n):
                    i = (self._next_report + 1) % n
                    self._next_report = i
                    x, y, z = self._mag_offsets(i)
                    mav.mag_cal_report_send(
                        i,
                        (1 << n) - 1,
                        self.config.mag_statuses[i],
                        int(self.mag_accepted),
                        4.5,
                        x,
                        y,
                        z,
                        1,
                        1,
                        1,
                        0,
                        0,
                        0,
                    )
            return True
        return False

    # -- periodic -------------------------------------------------------------------

    def tick(self, mav: mavlink.MAVLink) -> None:
        now = time.monotonic()
        if self.accel_step == 0:
            return
        if self.accel_result is not None:
            if self.accel_result_sent < 3 and now - self._last_pos_request > 0.2:
                self._send_position(mav, self.accel_result)
                self._last_pos_request = now
                self.accel_result_sent += 1
            return
        if not self.accel_waiting and now >= self.accel_sample_until:
            if self.accel_step == 6:
                self._finish_accel(mav)
                return
            if self.config.accel_stop_after_pose == self.accel_step:
                self.accel_step = 0  # goes quiet
                return
            self.accel_step += 1
            self.accel_waiting = True
            self._last_pos_request = 0.0
        if self.accel_waiting and now - self._last_pos_request > 0.2:
            self._send_position(mav, self.accel_step)
            self._last_pos_request = now

    # -- internals ------------------------------------------------------------------

    def _finish_accel(self, mav: mavlink.MAVLink) -> None:
        if self.config.accel_fail:
            self.accel_result = POS_FAILED
            self.vehicle.send_statustext(mav, "Calibration FAILED", mavlink.MAV_SEVERITY_CRITICAL)
        else:
            for axis, value in zip("XYZ", (0.051, -0.032, 0.115), strict=True):
                self.vehicle.set_param(f"INS_ACCOFFS_{axis}", value, ParamType.REAL32)
            self.accel_result = POS_SUCCESS
            self.vehicle.send_statustext(
                mav, "Calibration successful", mavlink.MAV_SEVERITY_CRITICAL
            )
        self.accel_result_sent = 0
        self._last_pos_request = 0.0

    def _send_position(self, mav: mavlink.MAVLink, value: int) -> None:
        mav.command_long_send(
            0, 0, mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS, 0, value, 0, 0, 0, 0, 0, 0
        )

    def _mag_done(self) -> bool:
        return (
            self.mag_started is not None
            and time.monotonic() - self.mag_started >= self.config.mag_duration_s
        )

    def _mag_offsets(self, compass: int) -> tuple[float, float, float]:
        return (21.5 + compass, -40.25 - compass, 118.0 + compass)

    def ack(
        self, mav: mavlink.MAVLink, msg: mavlink.MAVLink_command_long_message, result: int
    ) -> None:
        self.vehicle.ack(mav, msg, msg.command, result)
