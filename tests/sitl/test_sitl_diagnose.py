"""Diagnostics against real ArduCopter SITL (copter.parm defaults)."""

from __future__ import annotations

import pytest

import mavforge
from mavforge.domain.models import FindingKind, FindingSource
from mavforge.policy.permissions import Policy

pytestmark = [pytest.mark.sitl, pytest.mark.timeout(240)]

SIM_ONLY = Policy.configure(allow_real_vehicle=False)


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


def test_propose_fixes_builds_a_plan_that_applies_cleanly(sitl_connection: str) -> None:
    """LOG_BACKEND_TYPE=0 is the one case propose_fixes is allowed to auto-propose a
    value for: no judgment call, unlike a battery failsafe action (see
    docs/research.md, "Flight modes and failsafes"). copter.parm's default is 1, so
    setting 0 and applying the proposed fix also restores the default."""
    with mavforge.connect(sitl_connection, timeout=60, policy=SIM_ONLY) as vehicle:
        original = vehicle.params.get("LOG_BACKEND_TYPE").param.value
        disable = vehicle.params.plan({"LOG_BACKEND_TYPE": 0})
        mavforge.approve_plan(disable, approved_by="sitl-test")
        assert vehicle.params.apply(disable).status.value == "succeeded"

        off = vehicle.diagnose()
        off_finding = next(f for f in off.findings if f.check == "ardupilot.logging")
        assert off_finding.kind.value == "warning"
        assert off_finding.proposed_changes == {"LOG_BACKEND_TYPE": 1}
        assert off_finding.proposed_plan is None  # not built unless asked

        on = vehicle.diagnose(propose_fixes=True)
        on_finding = next(f for f in on.findings if f.check == "ardupilot.logging")
        plan = on_finding.proposed_plan
        assert plan is not None
        assert not plan.blocked
        assert [(c.name, c.requested) for c in plan.changes] == [("LOG_BACKEND_TYPE", 1)]

        mavforge.approve_plan(plan, approved_by="sitl-test")
        applied = vehicle.params.apply(plan)
        assert applied.status.value == "succeeded", applied
        assert vehicle.params.get("LOG_BACKEND_TYPE").param.value == original == 1
