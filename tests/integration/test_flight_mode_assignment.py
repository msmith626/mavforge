"""Flight-mode assignment against the fake vehicle: sweep, plan, approval, apply."""

from __future__ import annotations

import time

import pytest

import mavforge
from mavforge.domain.errors import NotPermittedError
from mavforge.domain.models import CalibrationKind, CalibrationState, CalibrationStatus, ParamType
from mavforge.domain.outcomes import OperationStatus
from mavforge.policy.permissions import Policy
from mavforge.services.calibration.flight_modes import FlightModeWorkflow
from mavforge.services.calibration.operations import REGISTRY
from mavforge.testing.fake_vehicle import FakeVehicle

CONFIGURE = Policy.configure(allow_real_vehicle=False)
CHANNEL = 5  # matches FLTMODE_CH set in setup()

# PWM each nominal switch position produces; chosen inside the exact
# RC_Channel::read_6pos_switch bands for positions 1, 3 and 6 (bands 0, 2, 5).
POS1, POS3, POS6 = 1100, 1425, 2000


def setup(fake: FakeVehicle, current: dict[str, int] | None = None) -> None:
    fake.set_param("FLTMODE_CH", CHANNEL, ParamType.INT8)
    for name, value in (current or {f"FLTMODE{n}": 0 for n in range(1, 7)}).items():
        fake.set_param(name, value, ParamType.INT8)


def run(
    vehicle: mavforge.Vehicle,
    fake: FakeVehicle,
    requested: dict[str, float],
    *,
    pwms: tuple[int, ...] = (POS1, POS3, POS6),
) -> CalibrationStatus:
    st = REGISTRY.start_observation(
        vehicle.session,
        CalibrationKind.FLIGHT_MODES,
        workflow=FlightModeWorkflow(requested, 5),
    )
    deadline = time.monotonic() + 20
    while not st.state.finished and time.monotonic() < deadline:
        if st.awaiting_confirmation == "sweep_done":
            for pwm in pwms:
                fake.set_rc(1500, 1500, 1500, 1500, pwm)
                time.sleep(0.3)
            st = vehicle.calibration.confirm(st.operation_id, "sweep_done")
            continue
        time.sleep(0.05)
        st = vehicle.calibration.status(st.operation_id)
    return vehicle.calibration.wait(st.operation_id, 10)


def test_sweep_proposes_a_plan_and_writes_nothing(fake: FakeVehicle) -> None:
    setup(fake, {"FLTMODE1": 0, "FLTMODE3": 5, "FLTMODE6": 0})
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, {"FLTMODE1": 0, "FLTMODE3": 6})
    assert st.state is CalibrationState.SUCCEEDED, st.result
    assert st.result is not None
    plan = st.result.plan
    assert plan is not None
    assert not plan.blocked
    requested = {c.name: c.requested for c in plan.changes}
    assert requested == {"FLTMODE1": 0, "FLTMODE3": 6}  # FLTMODE1 unchanged: no_change
    changes = {c.name: c for c in plan.changes}
    assert changes["FLTMODE1"].no_change is True
    assert changes["FLTMODE3"].no_change is False
    assert "needs approval" in st.result.summary
    assert not any(w for w in st.result.warnings if "Not reached" in w)
    bands = {b.parameter: b for b in st.mode_bands}
    assert bands["FLTMODE1"].visited is True
    assert bands["FLTMODE3"].visited is True
    assert bands["FLTMODE6"].visited is True
    assert bands["FLTMODE2"].visited is False  # never swept into this band
    param_sets = [m for m in fake.received if m.get_type() == "PARAM_SET"]
    assert param_sets == []  # the sweep itself never writes


def test_plan_applies_only_after_approval(fake: FakeVehicle) -> None:
    setup(fake)
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, {"FLTMODE1": 5})
        assert st.result is not None
        plan = st.result.plan
        assert plan is not None
        with pytest.raises(NotPermittedError, match="not approved"):
            vehicle.params.apply(plan)
        mavforge.approve_plan(plan, approved_by="test")
        result = vehicle.params.apply(plan)
        assert result.status is OperationStatus.SUCCEEDED
        assert vehicle.params.get("FLTMODE1").param.value == 5


def test_requested_band_never_reached_is_flagged_but_still_proposed(fake: FakeVehicle) -> None:
    setup(fake)
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        # Only sweep positions 1 and 6 (bands 0 and 5); band 2 (FLTMODE3) is never visited.
        st = run(vehicle, fake, {"FLTMODE3": 5}, pwms=(POS1, POS6))
    assert st.result is not None
    assert st.result.plan is not None
    assert {c.name: c.requested for c in st.result.plan.changes} == {"FLTMODE3": 5}
    assert any("Not reached" in w and "FLTMODE3" in w for w in st.result.warnings)


def test_disabled_mode_channel_fails_without_touching_rc(fake: FakeVehicle) -> None:
    fake.set_param("FLTMODE_CH", 0, ParamType.INT8)
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, {"FLTMODE1": 5})
    assert st.state is CalibrationState.FAILED
    assert st.result is not None
    assert "FLTMODE_CH is 0" in st.result.summary


def test_unknown_parameter_name_fails(fake: FakeVehicle) -> None:
    setup(fake)
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, {"FLTMODE7": 5})
    assert st.state is CalibrationState.FAILED
    assert st.result is not None
    assert "Not a flight-mode parameter" in st.result.summary


def test_nothing_requested_fails_immediately(fake: FakeVehicle) -> None:
    setup(fake)
    with mavforge.connect(fake.connection_string, policy=CONFIGURE) as vehicle:
        st = run(vehicle, fake, {})
    assert st.state is CalibrationState.FAILED
    assert st.result is not None
    assert "nothing to check" in st.result.summary
