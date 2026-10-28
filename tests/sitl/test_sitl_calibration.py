"""Calibration workflows against real ArduCopter SITL firmware.

Gyro and level run on the standard SITL. Compass and accelerometer use the SITL
'calibration' model; this test plays the person at the vehicle by moving the simulated
vehicle with SERVO5-8 (as ArduPilot's Tools/mavproxy_modules/sitl_calibration.py does).
MAVForge itself never sends servo commands; only this test harness does.
"""

from __future__ import annotations

import math
import time

import pytest

import mavforge
from mavforge.domain.models import CalibrationState, CalibrationStatus
from mavforge.policy.permissions import Policy
from mavforge.protocol.commands import send_command
from mavforge.transport.dialect import mavlink

pytestmark = [pytest.mark.sitl, pytest.mark.timeout(400)]

SIM_ONLY = Policy.calibrate(allow_real_vehicle=False)

# Attitudes for each pose, from ArduPilot's sitl_calibration.py (roll, pitch, yaw)
POSE_ATTITUDE = {
    "LEVEL": (0.0, 0.0, 0.0),
    "LEFT": (-math.pi / 2, 0.0, 0.0),
    "RIGHT": (math.pi / 2, 0.0, 0.0),
    "NOSEDOWN": (0.0, -math.pi / 2, 0.0),
    "NOSEUP": (0.0, math.pi / 2, 0.0),
    "BACK": (math.pi, 0.0, 0.0),
}


def servo(vehicle: mavforge.Vehicle, channel: int, pwm: int) -> None:
    """Test harness only: move the simulated vehicle (SITL 'calibration' model)."""
    send_command(
        vehicle.session.connection,
        *vehicle.session.target,
        mavlink.MAV_CMD_DO_SET_SERVO,
        [channel, pwm],
        timeout=3,
    )


def set_attitude(vehicle: mavforge.Vehicle, roll: float, pitch: float, yaw: float) -> None:
    scale = 500.0 / math.pi
    servo(vehicle, 5, 1150)  # attitude mode
    for channel, angle in zip((6, 7, 8), (roll, pitch, yaw), strict=True):
        servo(vehicle, channel, 1500 + int(angle * scale))


def run(
    vehicle: mavforge.Vehicle, kind: str, on_wait: object = None, timeout: float = 300
) -> CalibrationStatus:
    request = vehicle.calibration.request(kind)
    mavforge.approve_plan(request, approved_by="sitl-test")
    st = vehicle.calibration.start(request)
    deadline = time.monotonic() + timeout
    while not st.state.finished and time.monotonic() < deadline:
        if st.awaiting_confirmation and callable(on_wait):
            on_wait(st)
            st = vehicle.calibration.confirm(st.operation_id, st.awaiting_confirmation)
            continue
        time.sleep(0.25)
        st = vehicle.calibration.status(st.operation_id)
    return st


def test_gyro_and_level(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=60, policy=SIM_ONLY) as vehicle:
        gyro = run(vehicle, "gyro")
        time.sleep(6)  # ArduPilot rejects a level calibration within 5 s of another
        level = run(vehicle, "level")
    assert gyro.state is CalibrationState.SUCCEEDED, gyro.result
    assert level.state is CalibrationState.SUCCEEDED, level.result
    assert level.result is not None
    assert set(level.result.params_after) == {"AHRS_TRIM_X", "AHRS_TRIM_Y"}


def test_baro(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=60, policy=SIM_ONLY) as vehicle:
        baro = run(vehicle, "baro")
    assert baro.state is CalibrationState.SUCCEEDED, baro.result
    assert baro.result is not None
    assert "BARO1_GND_PRESS" in baro.result.params_after


def test_compass_calibration_with_simulated_rotation(sitl_calibration_connection: str) -> None:
    with mavforge.connect(sitl_calibration_connection, timeout=60, policy=SIM_ONLY) as vehicle:
        time.sleep(5)
        request = vehicle.calibration.request("compass")
        mavforge.approve_plan(request, approved_by="sitl-test")
        st = vehicle.calibration.start(request)
        servo(vehicle, 5, 1250)  # the "person" rotates the vehicle (autonomous magcal motion)
        try:
            st = vehicle.calibration.wait(st.operation_id, 240)
        finally:
            servo(vehicle, 5, 1000)
    assert st.state is CalibrationState.SUCCEEDED, st.result
    assert st.result is not None
    assert st.result.reboot_required
    assert all(c.status == "SUCCESS" for c in st.compasses)
    assert any("match the reported offsets" in e for e in st.result.evidence)


def test_accel_calibration_six_poses(sitl_calibration_connection: str) -> None:
    def place(status: CalibrationStatus) -> None:
        assert status.awaiting_confirmation is not None
        set_attitude(vehicle, *POSE_ATTITUDE[status.awaiting_confirmation])
        time.sleep(4)  # let the simulated vehicle settle before confirming

    with mavforge.connect(sitl_calibration_connection, timeout=60, policy=SIM_ONLY) as vehicle:
        try:
            st = run(vehicle, "accel", on_wait=place)
        finally:
            servo(vehicle, 5, 1000)
    assert st.state is CalibrationState.SUCCEEDED, st.result
    assert st.result is not None
    assert "Vehicle reported ACCELCAL_VEHICLE_POS_SUCCESS" in st.result.evidence
