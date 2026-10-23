"""Radio (RC) calibration against the fake vehicle: sweep, checks, plan, approval, apply."""

from __future__ import annotations

import time
from collections.abc import Sequence

import pytest

import mavforge
from mavforge.domain.errors import InvalidInputError, NotPermittedError
from mavforge.domain.models import CalibrationKind, CalibrationState, CalibrationStatus, ParamType
from mavforge.domain.outcomes import OperationStatus
from mavforge.policy.permissions import Policy
from mavforge.services.calibration.operations import REGISTRY
from mavforge.services.calibration.radio import RadioWorkflow
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle
from mavforge.transport.dialect import mavlink

CONFIGURE = Policy.configure(allow_real_vehicle=False)

# The person at the transmitter: sticks to every limit, then centred with throttle down.
LOW = (1102, 1098, 1095, 1104)
HIGH = (1895, 1901, 1906, 1893)
CENTRE = (1498, 1501, 1096, 1502)
SWEEP: list[Sequence[int]] = [LOW, HIGH, LOW, HIGH]


def param_sets(fake: FakeVehicle) -> list[str]:
    return [m.param_id for m in fake.received if isinstance(m, mavlink.MAVLink_param_set_message)]


def run(
    vehicle: mavforge.Vehicle,
    fake: FakeVehicle,
    *,
    sweep: Sequence[Sequence[int]] = SWEEP,
    centre: Sequence[int] = CENTRE,
    cancel_in_sweep: bool = False,
) -> CalibrationStatus:
    st = REGISTRY.start_observation(
        vehicle.session, CalibrationKind.RADIO, workflow=RadioWorkflow(5, 5)
    )
    deadline = time.monotonic() + 20
    while not st.state.finished and time.monotonic() < deadline:
        if st.awaiting_confirmation == "sweep_done":
            if cancel_in_sweep:
                return vehicle.calibration.cancel(st.operation_id)
            for frame in sweep:
                fake.set_rc(*frame)
                time.sleep(0.3)
            fake.set_rc(*centre)
            st = vehicle.calibration.confirm(st.operation_id, "sweep_done")
            continue
        if st.awaiting_confirmation == "centred":
            fake.set_rc(*centre)
            time.sleep(0.2)
            st = vehicle.calibration.confirm(st.operation_id, "centred")
            continue
        time.sleep(0.05)
        st = vehicle.calibration.status(st.operation_id)
    return vehicle.calibration.wait(st.operation_id, 10)


def test_sweep_proposes_a_plan_and_writes_nothing(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake)
    assert st.state is CalibrationState.SUCCEEDED, st.result
    assert st.result is not None
    plan = st.result.plan
    assert plan is not None
    assert not plan.blocked
    requested = {c.name: c.requested for c in plan.changes}
    assert requested == {
        "RC1_MIN": 1102, "RC1_MAX": 1895, "RC1_TRIM": 1498,
        "RC2_MIN": 1098, "RC2_MAX": 1901, "RC2_TRIM": 1501,
        "RC3_MIN": 1095, "RC3_MAX": 1906, "RC3_TRIM": 1096,
        "RC4_MIN": 1104, "RC4_MAX": 1893, "RC4_TRIM": 1502,
    }  # fmt: skip
    assert "needs approval" in st.result.summary
    assert any("RC5, RC6, RC7, RC8" in w for w in st.result.warnings)
    assert [c.moved for c in st.rc_channels] == [True] * 4 + [False] * 4
    assert param_sets(fake) == []  # the sweep itself never writes


def test_plan_applies_only_after_approval(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake)
        assert st.result is not None
        plan = st.result.plan
        assert plan is not None
        with pytest.raises(NotPermittedError, match="not approved"):
            vehicle.params.apply(plan)
        mavforge.approve_plan(plan, approved_by="test")
        result = vehicle.params.apply(plan)
        assert result.status is OperationStatus.SUCCEEDED
        assert vehicle.params.get("RC3_MAX").param.value == 1906


def test_switch_channel_is_included_when_swept(fake: FakeVehicle) -> None:
    for name, value in (("RC5_MIN", 1100), ("RC5_MAX", 1900), ("RC5_TRIM", 1500)):
        fake.set_param(name, value, ParamType.INT16)
    sweep = [(*LOW, 1000), (*HIGH, 2000), (*LOW, 1500)]
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, sweep=sweep, centre=(*CENTRE, 2000))
    assert st.result is not None
    assert st.result.plan is not None
    requested = {c.name: c.requested for c in st.result.plan.changes}
    assert (requested["RC5_MIN"], requested["RC5_MAX"], requested["RC5_TRIM"]) == (1000, 2000, 2000)


def test_jitter_on_an_unused_channel_is_not_movement(fake: FakeVehicle) -> None:
    fake.set_rc(*CENTRE, 1100, 1500)  # channel 6 unconnected: about 1500 +/- 2
    sweep = [(*LOW, 1100, 1498), (*HIGH, 1100, 1502), (*LOW, 1100, 1500)]
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, sweep=sweep)
    assert st.result is not None
    assert st.result.plan is not None
    assert not any(c.name.startswith("RC6") for c in st.result.plan.changes)
    assert any("not moved (travel 4 us)" in e for e in st.result.evidence)


def test_unmoved_primary_channel_fails(fake: FakeVehicle) -> None:
    sweep = [(*LOW[:3], 1500), (*HIGH[:3], 1500)]
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, sweep=sweep, centre=(*CENTRE[:3], 1500))
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED
    assert "RC4 (yaw) did not move" in st.result.summary
    assert st.result.plan is None


def test_uncentred_stick_fails(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, centre=(1800, *CENTRE[1:]))
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED
    assert "RC1 (roll) read 1800, not centred" in st.result.summary
    assert st.result.plan is None


def test_throttle_must_be_low(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, centre=(1498, 1501, 1500, 1502))
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED
    assert "RC3 (throttle) read 1500" in st.result.summary


def test_reversed_throttle_is_low_at_high_pwm(fake: FakeVehicle) -> None:
    fake.set_param("RC3_REVERSED", 1, ParamType.INT8)
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, centre=(1498, 1501, 1904, 1502))
    assert st.state is CalibrationState.SUCCEEDED, st.result


def test_follows_rcmap(fake: FakeVehicle) -> None:
    # throttle on channel 1, roll on channel 3 (e.g. a mode 1 transmitter mapping)
    fake.set_param("RCMAP_THROTTLE", 1, ParamType.INT8)
    fake.set_param("RCMAP_ROLL", 3, ParamType.INT8)
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, centre=(1096, 1501, 1498, 1502))
    assert st.state is CalibrationState.SUCCEEDED, st.result
    roles = {c.channel: c.role for c in st.rc_channels}
    assert (roles[1], roles[3]) == ("throttle", "roll")


def test_no_receiver(make_fake: FakeFactory) -> None:
    fake = make_fake(rc_inputs=[])
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake)
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED
    assert "no RC input" in st.result.summary


def test_receiver_in_failsafe_moves_nothing(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, sweep=[CENTRE], centre=CENTRE)
    assert st.result is not None
    assert st.result.status is OperationStatus.FAILED
    assert "No RC channel moved" in st.result.summary


def test_cancel_during_sweep_writes_nothing(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, cancel_in_sweep=True)
    assert st.state is CalibrationState.CANCELLED
    assert param_sets(fake) == []


def test_sweep_needs_only_read_permission_but_apply_needs_configure(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:  # read-only default
        st = run(vehicle, fake)
        assert st.result is not None
        plan = st.result.plan
        assert plan is not None
        mavforge.approve_plan(plan, approved_by="test")
        with pytest.raises(NotPermittedError):
            vehicle.params.apply(plan)
    assert param_sets(fake) == []


def test_radio_is_not_started_from_a_request(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        with pytest.raises(InvalidInputError, match="needs no request"):
            vehicle.calibration.request("radio")
        with pytest.raises(InvalidInputError, match="needs an approved request"):
            REGISTRY.start_observation(vehicle.session, CalibrationKind.GYRO)


def test_refused_when_armed(make_fake: FakeFactory) -> None:
    fake = make_fake(armed=True)
    with (
        mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle,
        pytest.raises(NotPermittedError, match="armed"),
    ):
        vehicle.calibration.start_radio()
