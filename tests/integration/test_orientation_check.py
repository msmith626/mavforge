"""Board orientation check against the fake vehicle: poses, checks, plan, approval, apply."""

from __future__ import annotations

import math
import time
from collections.abc import Callable

import pytest

import mavforge
from mavforge.domain.errors import InvalidInputError
from mavforge.domain.models import CalibrationKind, CalibrationState, CalibrationStatus, ParamType
from mavforge.domain.outcomes import OperationStatus
from mavforge.policy.permissions import Policy
from mavforge.services.calibration.operations import REGISTRY
from mavforge.services.calibration.orientation import OrientationWorkflow
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle
from mavforge.transport.dialect import mavlink

CONFIGURE = Policy.configure(allow_real_vehicle=False)

type Vec = tuple[float, float, float]
_S, _C = 1000 * math.sin(math.radians(35)), 1000 * math.cos(math.radians(35))
# Specific force in the true body frame for each pose the check asks for (35 degree tilts).
BODY: dict[str, Vec] = {
    "level": (0, 0, -1000),
    "nose_up": (_S, 0, -_C),
    "right_down": (0, -_S, -_C),
}


def as_mounted(x: float, y: float, z: float) -> Vec:
    """Arrow forward, top up, AHRS_ORIENTATION 0: reported as is."""
    return (x, y, z)


def arrow_right(x: float, y: float, z: float) -> Vec:
    """Arrow toward the right wing, configured 0: board X = body right, board Y = body back."""
    return (y, -x, z)


def upside_down(x: float, y: float, z: float) -> Vec:
    """Arrow forward but top facing down (Roll180), configured 0."""
    return (x, -y, -z)


def forward_but_configured_yaw90(x: float, y: float, z: float) -> Vec:
    """Mounted arrow forward, but AHRS_ORIENTATION=2 rotates every sample by Yaw90."""
    return (-y, x, z)


def rolled(deg: float) -> Callable[[float, float, float], Vec]:
    """Board rolled ``deg`` about the vehicle's X axis (not a standard rotation)."""
    s, c = math.sin(math.radians(deg)), math.cos(math.radians(deg))
    return lambda x, y, z: (x, c * y + s * z, -s * y + c * z)


def param_sets(fake: FakeVehicle) -> list[str]:
    return [m.param_id for m in fake.received if isinstance(m, mavlink.MAVLink_param_set_message)]


def run(
    vehicle: mavforge.Vehicle,
    fake: FakeVehicle,
    mount: Callable[[float, float, float], Vec] = as_mounted,
    *,
    poses: dict[str, list[Vec]] | None = None,
    cancel_at: str | None = None,
) -> CalibrationStatus:
    """Play the person at the vehicle. ``poses`` overrides what each attempt of a step reads."""
    attempts: dict[str, int] = {}
    st = REGISTRY.start_observation(
        vehicle.session, CalibrationKind.ORIENTATION, workflow=OrientationWorkflow(5)
    )
    deadline = time.monotonic() + 30
    while not st.state.finished and time.monotonic() < deadline:
        step = st.awaiting_confirmation
        if step is not None:
            if step == cancel_at:
                return vehicle.calibration.cancel(st.operation_id)
            n = attempts.get(step, 0)
            attempts[step] = n + 1
            override = (poses or {}).get(step)
            reading = override[min(n, len(override) - 1)] if override else mount(*BODY[step])
            fake.set_accel(*reading)
            time.sleep(0.2)
            st = vehicle.calibration.confirm(st.operation_id, step)
            continue
        time.sleep(0.05)
        st = vehicle.calibration.status(st.operation_id)
    return vehicle.calibration.wait(st.operation_id, 10)


def requested(st: CalibrationStatus) -> dict[str, float | int]:
    assert st.result is not None
    assert st.result.plan is not None
    return {c.name: c.requested for c in st.result.plan.changes if not c.no_change}


def test_correct_mounting_needs_no_change(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake)
    assert st.state is CalibrationState.SUCCEEDED, st.result
    assert st.result is not None
    assert "matches how the board is mounted" in st.result.summary
    assert requested(st) == {}
    assert any("points toward the nose; its top faces up" in e for e in st.result.evidence)
    assert param_sets(fake) == []


def test_arrow_right_proposes_yaw90_and_applies_after_approval(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, arrow_right)
        assert st.state is CalibrationState.SUCCEEDED, st.result
        assert st.result is not None
        assert requested(st) == {"AHRS_ORIENTATION": 2}
        assert "AHRS_ORIENTATION 2 (Yaw90), not 0 (None)" in st.result.summary
        assert any("repeat the level calibration" in w for w in st.result.warnings)
        assert any("points toward the right; its top faces up" in e for e in st.result.evidence)
        assert param_sets(fake) == []  # the check itself never writes
        plan = st.result.plan
        assert plan is not None
        mavforge.approve_plan(plan, approved_by="test")
        assert vehicle.params.apply(plan).status is OperationStatus.SUCCEEDED
    assert fake.config.params["AHRS_ORIENTATION"][0] == 2


def test_upside_down_board(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, upside_down)
    assert requested(st) == {"AHRS_ORIENTATION": 8}
    assert st.result is not None
    assert any("its top faces down" in e for e in st.result.evidence)


def test_wrong_current_setting_is_accounted_for(fake: FakeVehicle) -> None:
    fake.set_param("AHRS_ORIENTATION", 2, ParamType.INT8)
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, forward_but_configured_yaw90)
    assert requested(st) == {"AHRS_ORIENTATION": 0}


def test_live_tilt_shows_what_the_autopilot_sees(fake: FakeVehicle) -> None:
    fake.set_accel(*arrow_right(*BODY["nose_up"]))
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = REGISTRY.start_observation(
            vehicle.session, CalibrationKind.ORIENTATION, workflow=OrientationWorkflow(5)
        )
        deadline = time.monotonic() + 5
        while st.tilt is None and time.monotonic() < deadline:
            time.sleep(0.05)
            st = vehicle.calibration.status(st.operation_id)
        vehicle.calibration.cancel(st.operation_id)
    # nose up on a board whose arrow points right reads as the right side going down
    assert st.tilt is not None
    assert (round(st.tilt.roll_deg), round(st.tilt.pitch_deg)) == (35, 0)


def test_too_little_tilt_is_asked_again(fake: FakeVehicle) -> None:
    small = (1000 * math.sin(math.radians(8)), 0, -1000 * math.cos(math.radians(8)))
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, poses={"nose_up": [small, BODY["nose_up"]]})
    assert st.state is CalibrationState.SUCCEEDED, st.result
    assert st.result is not None
    assert sum(e.startswith("Nose up:") for e in st.result.evidence) == 1  # only the retry


def test_retry_instruction_explains_the_problem(fake: FakeVehicle) -> None:
    small = (1000 * math.sin(math.radians(8)), 0, -1000 * math.cos(math.radians(8)))
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = REGISTRY.start_observation(
            vehicle.session, CalibrationKind.ORIENTATION, workflow=OrientationWorkflow(5)
        )
        for step, reading in (("level", BODY["level"]), ("nose_up", small)):
            st = _await(vehicle, st, step)
            fake.set_accel(*reading)
            time.sleep(0.2)
            st = vehicle.calibration.confirm(st.operation_id, step)
        st = _await(vehicle, st, "nose_up")
        vehicle.calibration.cancel(st.operation_id)
    assert st.instruction is not None
    assert st.instruction.startswith("The vehicle is tilted only 8 degrees from level")


def _await(vehicle: mavforge.Vehicle, st: CalibrationStatus, step: str) -> CalibrationStatus:
    deadline = time.monotonic() + 10
    while st.awaiting_confirmation != step and time.monotonic() < deadline:
        time.sleep(0.05)
        st = vehicle.calibration.status(st.operation_id)
    assert st.awaiting_confirmation == step, st
    return st


def test_shaking_vehicle_fails_after_three_attempts(make_fake: FakeFactory) -> None:
    fake = make_fake(accel_jitter_mg=150)
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake)
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED
    assert "The level pose could not be read: The vehicle was moving." in st.result.summary


def test_lowering_the_wrong_side_fails(fake: FakeVehicle) -> None:
    left_down = (0, _S, -_C)
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, poses={"right_down": [left_down]})
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED
    assert "tilts disagree" in st.result.summary
    assert st.result.plan is None


def test_board_not_square_to_the_vehicle_fails(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, rolled(22))
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED
    assert "22 degrees from the nearest standard rotation" in st.result.summary


def test_custom_rotation_is_not_checked(fake: FakeVehicle) -> None:
    fake.set_param("AHRS_ORIENTATION", 101, ParamType.INT8)
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake)
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED
    assert "custom rotation" in st.result.summary


def test_no_imu_data(make_fake: FakeFactory) -> None:
    fake = make_fake(unanswered_message_ids={mavlink.MAVLINK_MSG_ID_SCALED_IMU})
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake)
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED
    assert "did not send SCALED_IMU" in st.result.summary


def test_cancel_writes_nothing(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, arrow_right, cancel_at="right_down")
    assert st.state is CalibrationState.CANCELLED
    assert param_sets(fake) == []


def test_check_needs_only_read_permission(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:  # read-only default
        st = run(vehicle, fake, arrow_right)
    assert st.state is CalibrationState.SUCCEEDED, st.result


def test_api_entry_point(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = vehicle.calibration.start_orientation(offline=True)
        assert st.kind is CalibrationKind.ORIENTATION
        vehicle.calibration.cancel(st.operation_id)


def test_not_started_from_a_request(fake: FakeVehicle) -> None:
    with (
        mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle,
        pytest.raises(InvalidInputError, match="needs no request"),
    ):
        vehicle.calibration.request("orientation")
