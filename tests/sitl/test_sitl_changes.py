"""Parameter writes against real ArduCopter SITL: plan, approve, apply, verify, restore.

These write to the throwaway SITL container only, and restore what they change.
"""

from __future__ import annotations

import pytest

import mavforge
from mavforge.domain.models import ChangeOutcome
from mavforge.domain.outcomes import OperationStatus
from mavforge.policy.permissions import Policy

pytestmark = [pytest.mark.sitl, pytest.mark.timeout(300)]

SIM_ONLY = Policy.configure(allow_real_vehicle=False)  # refuses if this is not SITL


def test_write_verify_and_restore(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=60, policy=SIM_ONLY) as vehicle:
        assert vehicle.identity.simulated is True
        before = {n: vehicle.params.get(n).param.value for n in ("RC1_TRIM", "BATT_FS_LOW_ACT")}
        plan = vehicle.params.plan({"RC1_TRIM": 1490, "BATT_FS_LOW_ACT": 2})
        assert not plan.blocked
        # ArduCopter 4.7.2 definitions are not published (as of 2026-10); latest is used
        assert plan.metadata_source is not None
        labels = {c.name: c.requested_label for c in plan.changes}
        assert labels["BATT_FS_LOW_ACT"] == "RTL"
        mavforge.approve_plan(plan, approved_by="sitl-test")
        result = vehicle.params.apply(plan)
        try:
            assert result.status is OperationStatus.SUCCEEDED
            assert {c.outcome for c in result.changes} == {ChangeOutcome.VERIFIED}
            assert vehicle.params.get("RC1_TRIM").param.value == 1490
        finally:
            assert result.pre_change_backup is not None
            undo = vehicle.params.plan_restore(result.pre_change_backup)
            mavforge.approve_plan(undo, approved_by="sitl-test")
            restored = vehicle.params.apply(undo)
        assert restored.status is OperationStatus.SUCCEEDED
        after = {n: vehicle.params.get(n).param.value for n in before}
        assert after == before


def test_firmware_rejects_unknown_parameter_at_plan_time(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=60, policy=SIM_ONLY) as vehicle:
        plan = vehicle.params.plan({"NO_SUCH_PARAM": 1}, offline=True)
    assert plan.blocked
