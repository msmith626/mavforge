"""Diagnostics against real ArduCopter SITL (copter.parm defaults). Read-only."""

from __future__ import annotations

import pytest

import mavforge
from mavforge.domain.models import FindingKind, FindingSource

pytestmark = [pytest.mark.sitl, pytest.mark.timeout(240)]


def test_diagnose_sitl(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=60) as vehicle:
        report = vehicle.diagnose()
    kinds = {f.check: f.kind for f in report.findings if f.source is FindingSource.PARAMETERS}
    # Real firmware acknowledges the pre-arm request (ArduPilot GCS_Common.cpp)
    assert report.prearm_requested is True
    # Whatever the vehicle reports (often start-up transients in SITL) is in PreArm form
    for f in report.findings:
        if f.source is FindingSource.VEHICLE:
            assert f.vehicle_text is not None
            assert f.vehicle_text.startswith("PreArm:")
    # copter.parm defaults: calibrated sensors, RC set, battery monitor without failsafe action
    assert kinds["ardupilot.arming_checks"] is FindingKind.PASSED
    assert kinds["ardupilot.accel_cal"] is FindingKind.PASSED
    assert kinds["ardupilot.compass_cal"] is FindingKind.PASSED
    assert kinds["ardupilot.rc_cal"] is FindingKind.PASSED
    assert kinds["ardupilot.frame_class"] is FindingKind.PASSED
    assert kinds["ardupilot.battery_failsafe"] is FindingKind.WARNING
