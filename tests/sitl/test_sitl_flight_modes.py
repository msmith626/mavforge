"""Flight-mode assignment against ArduCopter SITL.

Drives FLTMODE_CH (channel 5 on this image's defaults) through RC_CHANNELS_OVERRIDE,
the same mechanism test_sitl_radio.py uses for the stick sweep. MAVForge itself never
sends RC overrides; only this test harness does.
"""

from __future__ import annotations

import time

import pytest

import mavforge
from mavforge.domain.models import CalibrationState
from mavforge.policy.permissions import Policy

from .test_sitl_radio import IGNORE, transmitter

pytestmark = [pytest.mark.sitl, pytest.mark.timeout(180)]

SIM_ONLY = Policy.configure(allow_real_vehicle=False)
CHANNEL = 5  # FLTMODE_CH on this image's defaults
# PWM inside bands 0, 2 and 5 (RC_Channel::read_6pos_switch; confirmed in source, see
# docs/research.md, "Flight modes and failsafes").
POS1, POS3, POS6 = 1100, 1425, 2000


def test_flight_mode_sweep_plan_apply_and_restore(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=60, policy=SIM_ONLY) as vehicle:
        names = [f"FLTMODE{n}" for n in range(1, 7)]
        original = {n: vehicle.params.get(n).param.value for n in names}
        assert vehicle.params.get("FLTMODE_CH").param.value == CHANNEL
        with transmitter(vehicle) as frame:
            time.sleep(1)
            st = vehicle.calibration.start_flight_modes(
                {"FLTMODE1": 0, "FLTMODE3": 5}, offline=True
            )
            deadline = time.monotonic() + 60
            while not st.state.finished and time.monotonic() < deadline:
                if st.awaiting_confirmation == "sweep_done":
                    for pwm in (POS1, POS3, POS6):
                        frame.append((IGNORE, IGNORE, IGNORE, IGNORE, pwm))
                        time.sleep(0.6)
                    st = vehicle.calibration.confirm(st.operation_id, "sweep_done")
                else:
                    time.sleep(0.1)
                    st = vehicle.calibration.status(st.operation_id)
        assert st.state is CalibrationState.SUCCEEDED, st.result
        assert st.result is not None
        plan = st.result.plan
        assert plan is not None
        assert not plan.blocked
        requested = {c.name: c.requested for c in plan.changes if not c.no_change}
        assert requested == {"FLTMODE1": 0, "FLTMODE3": 5}  # both differ from defaults
        assert not any("Not reached" in w for w in st.result.warnings)
        mavforge.approve_plan(plan, approved_by="sitl-test")
        applied = vehicle.params.apply(plan)
        assert applied.status.value == "succeeded", applied
        assert vehicle.params.get("FLTMODE1").param.value == 0
        assert vehicle.params.get("FLTMODE3").param.value == 5

        restore = vehicle.params.plan(original, offline=True)
        mavforge.approve_plan(restore, approved_by="sitl-test")
        assert vehicle.params.apply(restore).status.value == "succeeded"
