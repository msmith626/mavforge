"""setup_status against real ArduCopter SITL (copter.parm defaults). Read-only."""

from __future__ import annotations

import pytest

import mavforge
from mavforge.domain.models import SetupStepStatus

pytestmark = [pytest.mark.sitl, pytest.mark.timeout(240)]


def test_setup_status_sitl(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=60) as vehicle:
        report = vehicle.setup_status()
    by_step = {s.step: s.status for s in report.steps}
    # Can't be told from parameters alone, regardless of firmware defaults.
    assert by_step["orientation"] is SetupStepStatus.UNKNOWN
    assert by_step["gyro"] is SetupStepStatus.UNKNOWN
    assert by_step["level"] is SetupStepStatus.UNKNOWN
    # copter.parm defaults match test_sitl_diagnose.py: calibrated sensors, RC set,
    # a battery monitor configured without a failsafe action.
    assert by_step["ardupilot.accel_cal"] is SetupStepStatus.DONE
    assert by_step["ardupilot.compass_cal"] is SetupStepStatus.DONE
    assert by_step["ardupilot.rc_cal"] is SetupStepStatus.DONE
    assert by_step["ardupilot.frame_class"] is SetupStepStatus.DONE
    assert by_step["ardupilot.battery_failsafe"] is SetupStepStatus.RECOMMENDED
    # First unresolved step in ArduPilot's own setup order, not the most serious finding.
    assert report.next_step is not None
    assert report.next_step.step == "orientation"
