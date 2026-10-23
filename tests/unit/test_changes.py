from datetime import UTC, datetime, timedelta
from pathlib import Path

import pytest

from mavforge.domain.errors import InvalidInputError
from mavforge.domain.models import ChangePlan, ParamType, PlannedChange, VehicleIdentity
from mavforge.platforms.ardupilot.adapter import ArduPilotAdapter
from mavforge.platforms.metadata import ParamMeta
from mavforge.services import plan_store
from mavforge.services.changes import _check_value, parse_assignments  # pyright: ignore[reportPrivateUsage]


def test_parse_assignments() -> None:
    assert parse_assignments(["A=1", "B = 2.5"]) == {"A": 1.0, "B": 2.5}
    for bad in ("A", "=1", "A=x"):
        with pytest.raises(InvalidInputError):
            parse_assignments([bad])


def test_type_checks() -> None:
    assert _check_value(1.5, ParamType.INT8, None, False)[0] == [
        "INT8 parameter requires a whole number."
    ]
    assert "Outside the INT8 range" in _check_value(300, ParamType.INT8, None, False)[0][0]
    assert _check_value(float("nan"), ParamType.REAL32, None, False)[0]
    assert _check_value(0.3, ParamType.REAL32, None, False) == ([], [])


def test_metadata_checks_and_override() -> None:
    rng = ParamMeta(name="RC1_MIN", range=(800, 2200))
    issues, warnings = _check_value(700, ParamType.INT16, rng, False)
    assert issues
    assert "documented range" in issues[0]
    issues, warnings = _check_value(700, ParamType.INT16, rng, True)
    assert issues == []
    assert "explicit override" in warnings[0]
    enum = ParamMeta(name="THR_FAILSAFE", values={0.0: "Disabled", 1.0: "Enabled"})
    assert _check_value(5, ParamType.INT8, enum, False)[0]
    read_only = ParamMeta(name="INS_ACC_ID", read_only=True)
    assert _check_value(5, ParamType.INT32, read_only, True)[0] == [
        "The firmware marks this parameter read-only."
    ]
    bits = ParamMeta(name="ARMING_CHECK", bitmask={0: "All", 1: "Baro"})
    assert "not documented" in _check_value(8, ParamType.INT32, bits, False)[1][0]


def test_ardupilot_change_notes() -> None:
    notes = ArduPilotAdapter().change_notes
    assert notes("MAV_SYSID", 1, 2)[0]
    assert notes("FORMAT_VERSION", 120, 0)[0]
    assert "disables all pre-arm" in notes("ARMING_CHECK", 1, 0)[1][-1]
    assert notes("ARMING_CHECK", 0, 1) == ([], [])
    assert "failsafe" in notes("THR_FAILSAFE", 1, 0)[1][0]
    assert notes("BATT_FS_LOW_ACT", 0, 2) == ([], [])
    assert "serial port" in notes("SERIAL1_PROTOCOL", 2, 5)[1][0]
    assert "propellers" in notes("SERVO3_FUNCTION", 70, 0)[1][0]


def _plan() -> ChangePlan:
    draft = ChangePlan(
        plan_id="0" * 12,
        digest="",
        created_at=datetime.now(UTC),
        kind="set",
        vehicle=VehicleIdentity(
            system_id=1,
            component_id=1,
            autopilot="ARDUPILOTMEGA",
            mav_type="QUADROTOR",
            platform="ardupilot",
        ),
        changes=[
            PlannedChange(
                name="RC1_TRIM", current=1500, requested=1490, type=ParamType.INT16, no_change=False
            )
        ],
        blocked=False,
    )
    digest = plan_store.compute_digest(draft)
    return draft.model_copy(update={"plan_id": digest[:12], "digest": digest})


def test_plan_digest_detects_tampering(isolated_state_dir: Path) -> None:
    plan = _plan()
    path = plan_store.save_plan(plan)
    assert plan_store.load_plan(plan.plan_id) == plan
    path.write_text(path.read_text().replace("1490", "1990"))
    with pytest.raises(InvalidInputError, match="modified"):
        plan_store.load_plan(plan.plan_id)


def test_approval_expiry_and_single_use(isolated_state_dir: Path) -> None:
    plan = _plan()
    plan_store.save_plan(plan)
    assert plan_store.valid_approval(plan) is None
    plan_store.approve(plan, "test", ttl=timedelta(seconds=-1))
    assert plan_store.valid_approval(plan) is None  # expired
    plan_store.approve(plan, "test")
    assert plan_store.valid_approval(plan) is not None
    plan_store.consume(plan.plan_id)
    with pytest.raises(InvalidInputError, match="already been applied"):
        plan_store.consume(plan.plan_id)
    with pytest.raises(InvalidInputError, match="already been applied"):
        plan_store.approve(plan, "test")


def test_approval_bound_to_digest(isolated_state_dir: Path) -> None:
    plan = _plan()
    plan_store.save_plan(plan)
    plan_store.approve(plan, "test")
    other = plan.model_copy(update={"digest": "f" * 64})
    assert plan_store.valid_approval(other) is None


@pytest.mark.parametrize("bad", ["../x", "abc", "ABCDEF123456", "0123456789abcdef"])
def test_plan_ids_are_validated(bad: str) -> None:
    with pytest.raises(InvalidInputError):
        plan_store.load_plan(bad) if len(bad) == 12 else plan_store._path(bad)  # pyright: ignore[reportPrivateUsage]
