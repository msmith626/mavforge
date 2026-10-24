"""Board orientation check against ArduCopter SITL (the 'calibration' model).

The test plays the person tilting the vehicle by posing the simulated vehicle with
SERVO5-8, and simulates a rotated board with SIM_IMU_ORIENT: SITL rotates its simulated
IMU samples by that rotation before ArduPilot applies AHRS_ORIENTATION
(libraries/AP_InertialSensor/AP_InertialSensor_SITL.cpp), so the correct AHRS_ORIENTATION
is its inverse. MAVForge itself never sends servo commands; only this test harness does.
"""

from __future__ import annotations

import time

import pytest

import mavforge
from mavforge.domain.models import CalibrationState, CalibrationStatus
from mavforge.policy.permissions import Policy

from .test_sitl_calibration import servo, set_attitude

pytestmark = [pytest.mark.sitl, pytest.mark.timeout(300)]

SIM_ONLY = Policy.configure(allow_real_vehicle=False)
TILT = 0.6  # radians, about 34 degrees
POSES = {"level": (0.0, 0.0, 0.0), "nose_up": (0.0, TILT, 0.0), "right_down": (TILT, 0.0, 0.0)}


def set_param(vehicle: mavforge.Vehicle, name: str, value: int) -> None:
    plan = vehicle.params.plan({name: value}, offline=True)
    mavforge.approve_plan(plan, approved_by="sitl-test")
    assert vehicle.params.apply(plan).status.value == "succeeded"


def check(vehicle: mavforge.Vehicle) -> CalibrationStatus:
    st = vehicle.calibration.start_orientation(offline=True)
    deadline = time.monotonic() + 90
    while not st.state.finished and time.monotonic() < deadline:
        if (step := st.awaiting_confirmation) is not None:
            set_attitude(vehicle, *POSES[step])
            time.sleep(3)  # let the simulated vehicle settle
            st = vehicle.calibration.confirm(st.operation_id, step)
            continue
        time.sleep(0.1)
        st = vehicle.calibration.status(st.operation_id)
    return vehicle.calibration.wait(st.operation_id, 10)


def test_orientation_check_finds_a_simulated_rotated_board(
    sitl_calibration_connection: str,
) -> None:
    with mavforge.connect(sitl_calibration_connection, timeout=60, policy=SIM_ONLY) as vehicle:
        time.sleep(5)
        try:
            square = check(vehicle)
            set_param(vehicle, "SIM_IMU_ORIENT", 2)  # IMU samples rotated by Yaw90
            rotated = check(vehicle)
            assert rotated.result is not None
            assert rotated.result.plan is not None
            mavforge.approve_plan(rotated.result.plan, approved_by="sitl-test")
            assert vehicle.params.apply(rotated.result.plan).status.value == "succeeded"
            fixed = check(vehicle)  # ArduPilot applies AHRS_ORIENTATION without a reboot
        finally:
            set_param(vehicle, "SIM_IMU_ORIENT", 0)
            set_param(vehicle, "AHRS_ORIENTATION", 0)
            servo(vehicle, 5, 1000)
    assert square.state is CalibrationState.SUCCEEDED, square.result
    assert square.result is not None
    assert "matches how the board is mounted" in square.result.summary

    assert rotated.state is CalibrationState.SUCCEEDED, rotated.result
    assert rotated.result is not None
    assert rotated.result.plan is not None
    changes = {c.name: c.requested for c in rotated.result.plan.changes if not c.no_change}
    assert changes == {"AHRS_ORIENTATION": 6}  # Yaw270 undoes the simulated Yaw90

    # closed loop: with the proposed value applied, the firmware's readings are right
    assert fixed.state is CalibrationState.SUCCEEDED, fixed.result
    assert fixed.result is not None
    assert "AHRS_ORIENTATION 6 (Yaw270) matches" in fixed.result.summary
