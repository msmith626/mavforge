"""Calibration workflows against the fake vehicle: success, failure, cancel, gates."""

import json
import time
from pathlib import Path

import pytest

import mavforge
from mavforge.domain.errors import InvalidInputError, NotPermittedError, UnsupportedError
from mavforge.domain.models import CalibrationKind, CalibrationState, CalibrationStatus
from mavforge.domain.outcomes import OperationStatus
from mavforge.policy.permissions import Policy
from mavforge.services.calibration.accel import STUCK_WARNING, AccelWorkflow
from mavforge.services.calibration.compass import CompassWorkflow
from mavforge.services.calibration.operations import REGISTRY
from mavforge.testing.fake_calibration import FakeCalibrationConfig
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle
from mavforge.transport.dialect import mavlink

CAL = Policy.calibrate(allow_real_vehicle=False)


def connect(fake: FakeVehicle, policy: Policy = CAL) -> mavforge.Vehicle:
    return mavforge.connect(fake.connection_string, policy=policy)


def commands(fake: FakeVehicle) -> list[int]:
    return [m.command for m in fake.received if isinstance(m, mavlink.MAVLink_command_long_message)]


def run(
    vehicle: mavforge.Vehicle,
    kind: str,
    *,
    confirm: bool = True,
    workflow: object = None,
    cancel_after: float | None = None,
) -> CalibrationStatus:
    request = vehicle.calibration.request(kind)
    mavforge.approve_plan(request, approved_by="test")
    st = REGISTRY.start(vehicle.session, request, workflow=workflow)  # type: ignore[arg-type]
    started = time.monotonic()
    while not st.state.finished:
        if cancel_after is not None and time.monotonic() - started > cancel_after:
            return vehicle.calibration.cancel(st.operation_id)
        if confirm and st.awaiting_confirmation:
            st = vehicle.calibration.confirm(st.operation_id, st.awaiting_confirmation)
            continue
        time.sleep(0.05)
        st = vehicle.calibration.status(st.operation_id)
    return st


def cal_fake(make_fake: FakeFactory, **cal: object) -> FakeVehicle:
    return make_fake(calibration=FakeCalibrationConfig(**cal))  # type: ignore[arg-type]


# -- gyro / level -----------------------------------------------------------------------


def test_gyro_success_with_read_back(fake: FakeVehicle) -> None:
    with connect(fake) as vehicle:
        st = run(vehicle, "gyro")
    assert st.state is CalibrationState.SUCCEEDED
    assert st.result is not None
    assert st.result.params_after["INS_GYROFFS_X"] == pytest.approx(0.0012)
    assert st.result.params_before["INS_GYROFFS_X"] == pytest.approx(0.0011)


def test_gyro_failure_is_reported(make_fake: FakeFactory) -> None:
    fake = cal_fake(make_fake, gyro_result=mavlink.MAV_RESULT_FAILED)
    with connect(fake) as vehicle:
        st = run(vehicle, "gyro")
    assert st.state is CalibrationState.FAILED
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED


def test_level_rejected_explains_retry(make_fake: FakeFactory) -> None:
    fake = cal_fake(make_fake, level_result=mavlink.MAV_RESULT_TEMPORARILY_REJECTED)
    with connect(fake) as vehicle:
        st = run(vehicle, "level")
    assert st.result is not None
    assert st.result.status is OperationStatus.REJECTED
    assert "5 s" in st.result.summary


def test_level_success(fake: FakeVehicle) -> None:
    with connect(fake) as vehicle:
        st = run(vehicle, "level")
    assert st.result is not None
    assert st.result.params_after == {"AHRS_TRIM_X": 0.012, "AHRS_TRIM_Y": -0.008}


# -- baro ---------------------------------------------------------------------------------


def test_baro_success_with_read_back(fake: FakeVehicle) -> None:
    with connect(fake) as vehicle:
        st = run(vehicle, "baro")
    assert st.state is CalibrationState.SUCCEEDED
    assert st.result is not None
    assert st.result.params_before["BARO1_GND_PRESS"] == pytest.approx(101325.0)
    assert st.result.params_after["BARO1_GND_PRESS"] == pytest.approx(101180.0)


def test_baro_failure_is_reported(make_fake: FakeFactory) -> None:
    fake = cal_fake(make_fake, baro_result=mavlink.MAV_RESULT_FAILED)
    with connect(fake) as vehicle:
        st = run(vehicle, "baro")
    assert st.state is CalibrationState.FAILED
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED


def test_baro_rejected_explains_retry(make_fake: FakeFactory) -> None:
    fake = cal_fake(make_fake, baro_result=mavlink.MAV_RESULT_TEMPORARILY_REJECTED)
    with connect(fake) as vehicle:
        st = run(vehicle, "baro")
    assert st.result is not None
    assert st.result.status is OperationStatus.REJECTED
    assert "5 s" in st.result.summary


# -- compass ----------------------------------------------------------------------------


def test_compass_success_saves_and_verifies(make_fake: FakeFactory) -> None:
    fake = cal_fake(make_fake, mag_duration_s=0.6)
    with connect(fake) as vehicle:
        st = run(vehicle, "compass")
    assert st.state is CalibrationState.SUCCEEDED
    assert st.result is not None
    assert st.result.reboot_required
    assert st.result.params_after["COMPASS_OFS_X"] == 21.5
    assert any("match the reported offsets" in e for e in st.result.evidence)
    assert [c.status for c in st.compasses] == ["SUCCESS"]
    assert mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL in commands(fake)


def test_compass_failure_saves_nothing(make_fake: FakeFactory) -> None:
    fake = cal_fake(make_fake, mag_duration_s=0.4, mag_statuses=[4, 7])
    fake.set_param("COMPASS_DEV_ID2", 1234, fake.config.params["COMPASS_DEV_ID2"][1])
    with connect(fake) as vehicle:
        st = run(vehicle, "compass")
        assert vehicle.params.get("COMPASS_OFS_X").param.value == 5.0
    assert st.state is CalibrationState.FAILED
    assert st.result is not None
    assert "FAILED_RADIUS" in st.result.summary  # ArduPilot: BAD_RADIUS (status 7)
    assert "Nothing was saved" in st.result.summary
    assert mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL not in commands(fake)
    assert mavlink.MAV_CMD_DO_CANCEL_MAG_CAL in commands(fake)


def test_compass_accept_without_save_is_not_success(make_fake: FakeFactory) -> None:
    fake = cal_fake(make_fake, mag_duration_s=0.4, mag_save_on_accept=False)
    with connect(fake) as vehicle:
        st = run(vehicle, "compass")
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED
    assert "could not be confirmed" in st.result.summary


def test_compass_cancel_stops_vehicle_calibration(make_fake: FakeFactory) -> None:
    fake = cal_fake(make_fake, mag_duration_s=30)
    with connect(fake) as vehicle:
        st = run(vehicle, "compass", cancel_after=1.0)
        assert vehicle.params.get("COMPASS_OFS_X").param.value == 5.0
    assert st.state is CalibrationState.CANCELLED
    assert mavlink.MAV_CMD_DO_CANCEL_MAG_CAL in commands(fake)


def test_compass_timeout_cancels(make_fake: FakeFactory) -> None:
    fake = cal_fake(make_fake, mag_duration_s=30)
    with connect(fake) as vehicle:
        st = run(vehicle, "compass", workflow=CompassWorkflow(timeout=1.0))
    assert st.result is not None
    assert st.result.status is OperationStatus.TIMED_OUT
    assert mavlink.MAV_CMD_DO_CANCEL_MAG_CAL in commands(fake)


# -- accelerometer ----------------------------------------------------------------------


def test_accel_six_poses_with_confirmations(fake: FakeVehicle) -> None:
    with connect(fake) as vehicle:
        st = run(vehicle, "accel")
    assert st.state is CalibrationState.SUCCEEDED
    assert st.result is not None
    assert [e for e in st.result.evidence if e.startswith("Position")] == [
        f"Position {p} accepted" for p in ("LEVEL", "LEFT", "RIGHT", "NOSEDOWN", "NOSEUP", "BACK")
    ]
    assert "Calibration successful" in [m.text for m in st.vehicle_messages]


def test_accel_waits_for_the_human(fake: FakeVehicle) -> None:
    with connect(fake) as vehicle:
        request = vehicle.calibration.request("accel")
        mavforge.approve_plan(request)
        st = vehicle.calibration.start(request)
        deadline = time.monotonic() + 5
        while st.awaiting_confirmation is None and time.monotonic() < deadline:
            time.sleep(0.05)
            st = vehicle.calibration.status(st.operation_id)
        assert st.awaiting_confirmation == "LEVEL"
        assert st.instruction is not None
        assert "level" in st.instruction
        with pytest.raises(InvalidInputError, match="Expected confirmation of 'LEVEL'"):
            vehicle.calibration.confirm(st.operation_id, "BACK")
        time.sleep(0.5)  # no confirmation: the vehicle must not have been told to sample
        assert mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS not in commands(fake)
        st = vehicle.calibration.cancel(st.operation_id)
    assert st.state is CalibrationState.CANCELLED
    assert st.result is not None
    assert STUCK_WARNING in st.result.warnings  # honest: ArduPilot cannot cancel it


def test_accel_failure_and_silence(make_fake: FakeFactory) -> None:
    fake = cal_fake(make_fake, accel_fail=True)
    with connect(fake) as vehicle:
        assert run(vehicle, "accel").state is CalibrationState.FAILED
    quiet = cal_fake(make_fake, accel_stop_after_pose=2)
    with connect(quiet) as vehicle:
        st = run(vehicle, "accel", workflow=AccelWorkflow(silence_timeout=1.0))
    assert st.state is CalibrationState.UNKNOWN


# -- gates -------------------------------------------------------------------------------


def test_start_requires_approval_policy_and_disarmed(make_fake: FakeFactory) -> None:
    fake = make_fake()
    with connect(fake) as vehicle:
        request = vehicle.calibration.request("gyro")
        with pytest.raises(NotPermittedError, match="not approved"):
            vehicle.calibration.start(request)
    with connect(fake, Policy.read_only()) as vehicle:
        request = vehicle.calibration.request("gyro")
        mavforge.approve_plan(request)
        with pytest.raises(NotPermittedError, match="calibrate"):
            vehicle.calibration.start(request)
    armed = make_fake(armed=True)
    with connect(armed) as vehicle:
        request = vehicle.calibration.request("gyro")
        mavforge.approve_plan(request)
        with pytest.raises(NotPermittedError, match="armed"):
            vehicle.calibration.start(request)
    real = make_fake(simulated=False)
    with connect(real) as vehicle:
        request = vehicle.calibration.request("gyro")
        mavforge.approve_plan(request)
        with pytest.raises(NotPermittedError, match="real"):
            vehicle.calibration.start(request)
    for f in (fake, armed, real):
        assert mavlink.MAV_CMD_PREFLIGHT_CALIBRATION not in commands(f)


def test_requests_are_single_use_and_vehicle_bound(make_fake: FakeFactory) -> None:
    a = make_fake(firmware=(4, 6, 2, 255))
    b = make_fake(firmware=(4, 7, 2, 255))
    with connect(a) as vehicle:
        request = vehicle.calibration.request("gyro")
        mavforge.approve_plan(request)
    with connect(b) as vehicle, pytest.raises(NotPermittedError, match="different vehicle"):
        vehicle.calibration.start(request)
    with connect(a) as vehicle:
        vehicle.calibration.wait(vehicle.calibration.start(request).operation_id, 10)
        with pytest.raises(InvalidInputError, match="already"):
            mavforge.approve_plan(request)


def test_one_calibration_at_a_time(make_fake: FakeFactory) -> None:
    fake = cal_fake(make_fake, mag_duration_s=30)
    with connect(fake) as vehicle:
        first = vehicle.calibration.request("compass")
        mavforge.approve_plan(first)
        op = vehicle.calibration.start(first).operation_id
        second = vehicle.calibration.request("gyro")
        mavforge.approve_plan(second)
        with pytest.raises(InvalidInputError, match="already running"):
            vehicle.calibration.start(second)
        vehicle.calibration.cancel(op)


def test_unsupported_platform(make_fake: FakeFactory) -> None:
    fake = make_fake(autopilot=mavlink.MAV_AUTOPILOT_PX4)
    with connect(fake, Policy.calibrate(allow_real_vehicle=True)) as vehicle:
        assert not any(p.supported for p in vehicle.calibration.procedures())
        request = vehicle.calibration.request("gyro")
        mavforge.approve_plan(request)
        with pytest.raises(UnsupportedError):
            vehicle.calibration.start(request)


def test_calibration_is_audited(fake: FakeVehicle, isolated_state_dir: Path) -> None:
    with connect(fake) as vehicle:
        run(vehicle, "gyro")
    events = [
        json.loads(line) for line in (isolated_state_dir / "audit.jsonl").read_text().splitlines()
    ]
    kinds = [e["event"] for e in events]
    assert kinds.index("calibration_started") < kinds.index("calibration_finished")
    finished = next(e for e in events if e["event"] == "calibration_finished")
    assert finished["status"] == "succeeded"


def test_procedures_describe_every_workflow() -> None:
    procs = {p.kind: p for p in mavforge.calibration_procedures()}
    assert set(procs) == set(CalibrationKind)
    assert procs[CalibrationKind.ACCEL].available_via_mcp is False
    assert all(p.preconditions and p.human_steps and p.success_evidence for p in procs.values())


def test_compass_collects_every_report(make_fake: FakeFactory) -> None:
    """Regression (found in SITL): one REQUEST_MESSAGE makes ArduPilot send a report per
    finished compass; reading only the first reply never completed with 3 compasses."""
    fake = cal_fake(make_fake, mag_duration_s=0.4, mag_statuses=[4, 4, 4])
    for name in ("COMPASS_DEV_ID2", "COMPASS_DEV_ID3"):
        fake.set_param(name, 4321, fake.config.params[name][1])
    with connect(fake) as vehicle:
        st = run(vehicle, "compass")
    assert st.state is CalibrationState.SUCCEEDED, st.result
    assert [c.compass_id for c in st.compasses] == [0, 1, 2]
