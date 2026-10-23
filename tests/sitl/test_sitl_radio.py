"""Radio calibration against ArduCopter SITL.

The test plays the person at the transmitter with RC_CHANNELS_OVERRIDE, which ArduPilot
feeds into the same RC input that RC_CHANNELS reports. MAVForge itself never sends RC
overrides; only this test harness does.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Generator, Sequence
from contextlib import contextmanager

import pytest

import mavforge
from mavforge.domain.models import CalibrationState
from mavforge.policy.permissions import Policy
from mavforge.transport.dialect import mavlink

pytestmark = [pytest.mark.sitl, pytest.mark.timeout(180)]

SIM_ONLY = Policy.configure(allow_real_vehicle=False)
LOW = (1080, 1076, 1072, 1084)  # all more than TOLERANCE us from the SITL defaults
HIGH = (1916, 1924, 1930, 1912)
CENTRE = (1510, 1490, 1074, 1508)
IGNORE = 65535  # RC_CHANNELS_OVERRIDE: leave this channel alone


@contextmanager
def transmitter(vehicle: mavforge.Vehicle) -> Generator[list[Sequence[int]]]:
    """Send RC_CHANNELS_OVERRIDE at 10 Hz with whatever frame the list holds last."""
    frame: list[Sequence[int]] = [CENTRE]
    stop = threading.Event()
    sysid, compid = vehicle.session.target

    def loop() -> None:
        while not stop.wait(0.1):
            values = list(frame[-1]) + [IGNORE] * (18 - len(frame[-1]))
            vehicle.session.connection.send(
                mavlink.MAVLink_rc_channels_override_message(sysid, compid, *values)
            )

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    try:
        yield frame
    finally:
        stop.set()
        thread.join()


def test_radio_sweep_plan_apply_and_restore(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=60, policy=SIM_ONLY) as vehicle:
        names = [f"RC{c}_{k}" for c in range(1, 5) for k in ("MIN", "MAX", "TRIM")]
        original = {n: vehicle.params.get(n).param.value for n in names}
        with transmitter(vehicle) as frame:
            time.sleep(1)
            st = vehicle.calibration.start_radio()
            deadline = time.monotonic() + 60
            while not st.state.finished and time.monotonic() < deadline:
                if st.awaiting_confirmation == "sweep_done":
                    for f in (LOW, HIGH, LOW, HIGH, CENTRE):
                        frame.append(f)
                        time.sleep(0.6)
                    st = vehicle.calibration.confirm(st.operation_id, "sweep_done")
                elif st.awaiting_confirmation == "centred":
                    time.sleep(0.3)
                    st = vehicle.calibration.confirm(st.operation_id, "centred")
                else:
                    time.sleep(0.1)
                    st = vehicle.calibration.status(st.operation_id)
        assert st.state is CalibrationState.SUCCEEDED, st.result
        assert st.result is not None
        plan = st.result.plan
        assert plan is not None
        requested = {c.name: c.requested for c in plan.changes}
        for ch in range(4):
            n = ch + 1
            got = (requested[f"RC{n}_MIN"], requested[f"RC{n}_MAX"], requested[f"RC{n}_TRIM"])
            assert got == (LOW[ch], HIGH[ch], CENTRE[ch])
        mavforge.approve_plan(plan, approved_by="sitl-test")
        applied = vehicle.params.apply(plan)
        assert applied.status.value == "succeeded", applied
        assert vehicle.params.get("RC3_MAX").param.value == HIGH[2]

        restore = vehicle.params.plan(original)
        mavforge.approve_plan(restore, approved_by="sitl-test")
        assert vehicle.params.apply(restore).status.value == "succeeded"
