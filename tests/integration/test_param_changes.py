"""Plan -> approve -> apply -> verify against the fake vehicle, including every gate."""

import json
from pathlib import Path

import pytest

import mavforge
from mavforge.domain.errors import InvalidInputError, NotPermittedError
from mavforge.domain.models import ChangeOutcome, ParamType
from mavforge.domain.outcomes import OperationStatus
from mavforge.policy.permissions import Policy
from mavforge.services import plan_store
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle
from mavforge.transport.dialect import mavlink

RW = Policy.configure(allow_real_vehicle=False)


def connect(fake: FakeVehicle, policy: Policy = RW) -> mavforge.Vehicle:
    return mavforge.connect(fake.connection_string, policy=policy)


def sets(fake: FakeVehicle) -> list[str]:
    return [
        str(m.param_id).rstrip("\x00")
        for m in fake.received
        if isinstance(m, mavlink.MAVLink_param_set_message)
    ]


def outcomes(result: mavforge.api.ApplyResult) -> dict[str, ChangeOutcome]:  # type: ignore[name-defined]
    return {c.name: c.outcome for c in result.changes}


def test_set_verify_backup_and_audit(fake: FakeVehicle, isolated_state_dir: Path) -> None:
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490, "BATT_FS_LOW_ACT": 1, "LOG_BITMASK": 176126})
        assert not plan.blocked
        assert [c.no_change for c in plan.changes] == [False, False, True]
        mavforge.approve_plan(plan, approved_by="test")
        result = vehicle.params.apply(plan)
        assert vehicle.params.get("RC1_TRIM").param.value == 1490
    assert result.status is OperationStatus.SUCCEEDED
    assert outcomes(result) == {
        "RC1_TRIM": ChangeOutcome.VERIFIED,
        "BATT_FS_LOW_ACT": ChangeOutcome.VERIFIED,
        "LOG_BITMASK": ChangeOutcome.UNCHANGED,
    }
    assert sets(fake) == ["RC1_TRIM", "BATT_FS_LOW_ACT"]  # no-op is not written
    pre = mavforge.load_backup(isolated_state_dir / "backups" / result.pre_change_backup)  # type: ignore[operator]
    assert {p.name: p.value for p in pre.params}["RC1_TRIM"] == 1500
    events = [
        json.loads(line)["event"]
        for line in (isolated_state_dir / "audit.jsonl").read_text().splitlines()
    ]
    assert events.index("apply_started") < events.index("apply_finished")


def test_metadata_shapes_the_plan(fake: FakeVehicle) -> None:
    fake.set_param("THR_FAILSAFE", 1, ParamType.INT8)
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"THR_FAILSAFE": 0, "RC1_MIN": 700})
    by_name = {c.name: c for c in plan.changes}
    assert by_name["THR_FAILSAFE"].requested_label == "Disabled"
    assert by_name["THR_FAILSAFE"].current_label == "Enabled"
    assert any("disables the RC/throttle failsafe" in w for w in by_name["THR_FAILSAFE"].warnings)
    assert by_name["RC1_MIN"].range == (800, 2200)
    assert by_name["RC1_MIN"].issues  # outside documented range, no override
    assert plan.blocked
    assert plan.metadata_source is not None


def test_apply_requires_human_approval(fake: FakeVehicle) -> None:
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490})
        with pytest.raises(NotPermittedError, match="not approved"):
            vehicle.params.apply(plan)
    assert sets(fake) == []


def test_read_only_policy_cannot_apply(fake: FakeVehicle) -> None:
    with connect(fake, Policy.read_only()) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490})  # planning is allowed
        mavforge.approve_plan(plan)
        with pytest.raises(NotPermittedError, match="configure"):
            vehicle.params.apply(plan)
    assert sets(fake) == []


def test_real_vehicle_needs_explicit_policy(make_fake: FakeFactory) -> None:
    fake = make_fake(simulated=False)
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490})
        assert any(w.startswith("REAL VEHICLE") for w in plan.warnings)
        mavforge.approve_plan(plan)
        with pytest.raises(NotPermittedError, match="real"):
            vehicle.params.apply(plan)
    assert sets(fake) == []
    with connect(fake, Policy.configure(allow_real_vehicle=True)) as vehicle:
        assert vehicle.params.apply(plan).status is OperationStatus.SUCCEEDED


def test_armed_vehicle_is_refused(make_fake: FakeFactory) -> None:
    fake = make_fake(armed=True)
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490})
        mavforge.approve_plan(plan)
        with pytest.raises(NotPermittedError, match="armed"):
            vehicle.params.apply(plan)
    assert sets(fake) == []


def test_plan_is_bound_to_its_vehicle(make_fake: FakeFactory) -> None:
    a = make_fake(firmware=(4, 6, 2, 255))
    b = make_fake(firmware=(4, 7, 2, 255))
    with connect(a) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490})
    mavforge.approve_plan(plan)
    with connect(b) as vehicle, pytest.raises(NotPermittedError, match="different firmware"):
        vehicle.params.apply(plan)
    assert sets(b) == []


def test_tampered_plan_is_rejected(fake: FakeVehicle, isolated_state_dir: Path) -> None:
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490})
        mavforge.approve_plan(plan)
        path = isolated_state_dir / "plans" / f"{plan.plan_id}.json"
        path.write_text(path.read_text().replace("1490", "1900"))
        with pytest.raises(InvalidInputError, match="modified"):
            vehicle.params.apply(plan.plan_id)
    assert sets(fake) == []


def test_plans_are_single_use(fake: FakeVehicle) -> None:
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490})
        mavforge.approve_plan(plan)
        vehicle.params.apply(plan)
        with pytest.raises(InvalidInputError, match="already been applied"):
            vehicle.params.apply(plan)
    assert sets(fake) == ["RC1_TRIM"]


def test_value_changed_since_planning_is_skipped(fake: FakeVehicle) -> None:
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490, "RC1_MAX": 1950})
        mavforge.approve_plan(plan)
        fake.set_param("RC1_TRIM", 1520, ParamType.INT16)  # someone else changed it
        result = vehicle.params.apply(plan)
    assert outcomes(result) == {
        "RC1_TRIM": ChangeOutcome.SKIPPED,
        "RC1_MAX": ChangeOutcome.VERIFIED,
    }
    assert result.status is OperationStatus.PARTIAL
    assert sets(fake) == ["RC1_MAX"]


def test_write_failures_are_reported_honestly(make_fake: FakeFactory) -> None:
    fake = make_fake(
        param_write_errors={"RC1_MIN": mavlink.MAV_PARAM_ERROR_PERMISSION_DENIED},
        param_constraints={"RC1_MAX": (800, 1900)},
        ignore_param_set={"RC1_TRIM"},
        no_echo_param_set={"BATT_FS_LOW_ACT"},
    )
    with connect(fake) as vehicle:
        plan = vehicle.params.plan(
            {"RC1_MIN": 1000, "RC1_MAX": 1950, "RC1_TRIM": 1490, "BATT_FS_LOW_ACT": 1}
        )
        mavforge.approve_plan(plan)
        result = vehicle.params.apply(plan)
    by_name = {c.name: c for c in result.changes}
    assert by_name["RC1_MIN"].outcome is ChangeOutcome.FAILED
    assert "PERMISSION_DENIED" in (by_name["RC1_MIN"].detail or "")
    assert by_name["RC1_MAX"].outcome is ChangeOutcome.FAILED  # clamped by the firmware
    assert by_name["RC1_MAX"].after == 1900
    assert by_name["RC1_TRIM"].outcome is ChangeOutcome.FAILED  # read-back shows old value
    assert by_name["RC1_TRIM"].after == 1500
    assert by_name["BATT_FS_LOW_ACT"].outcome is ChangeOutcome.VERIFIED  # read-back, no echo
    assert result.status is OperationStatus.PARTIAL


def test_link_loss_mid_batch_is_unknown_not_failed(make_fake: FakeFactory) -> None:
    fake = make_fake(drop_link_after_sets=1)
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490, "RC1_MAX": 1950})
        mavforge.approve_plan(plan)
        result = vehicle.params.apply(plan)
    assert outcomes(result) == {
        "RC1_TRIM": ChangeOutcome.UNKNOWN,
        "RC1_MAX": ChangeOutcome.NOT_ATTEMPTED,
    }
    assert result.status is OperationStatus.UNKNOWN
    assert result.warnings
    assert plan_store.is_applied(plan.plan_id)


def test_retry_after_link_loss_resolves_the_rest(make_fake: FakeFactory) -> None:
    """The scenario the link-loss test above leaves unresolved, picked up from here:
    the link is back, and plan_retry only needs to cover what's actually outstanding."""
    fake = make_fake(drop_link_after_sets=1)
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490, "RC1_MAX": 1950})
        mavforge.approve_plan(plan)
        result = vehicle.params.apply(plan)
    assert outcomes(result) == {
        "RC1_TRIM": ChangeOutcome.UNKNOWN,
        "RC1_MAX": ChangeOutcome.NOT_ATTEMPTED,
    }
    fake.config.drop_link_after_sets = None  # the link is back
    with connect(fake) as vehicle:
        retry = vehicle.params.plan_retry(result)
        assert {c.name for c in retry.changes} == {"RC1_TRIM", "RC1_MAX"}
        mavforge.approve_plan(retry)
        retried = vehicle.params.apply(retry)
    # RC1_TRIM: the fake vehicle actually applied it before dropping the link (realistic -
    # a write can succeed even when its ack is lost), so the retry plan correctly sees
    # "already has this value" and doesn't write it again: UNCHANGED, not a redundant
    # VERIFIED. RC1_MAX genuinely never reached the vehicle the first time, so it does
    # get written now.
    assert outcomes(retried) == {
        "RC1_TRIM": ChangeOutcome.UNCHANGED,
        "RC1_MAX": ChangeOutcome.VERIFIED,
    }


def test_retry_by_plan_id_loads_the_recorded_result(make_fake: FakeFactory) -> None:
    """The CLI/MCP path: a separate call, only the plan id in hand, no ApplyResult
    object carried over in memory."""
    fake = make_fake(drop_link_after_sets=1)
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490, "RC1_MAX": 1950})
        mavforge.approve_plan(plan)
        vehicle.params.apply(plan)
    fake.config.drop_link_after_sets = None
    with connect(fake) as vehicle:
        retry = vehicle.params.plan_retry(plan.plan_id)  # string, not the ApplyResult
    assert {c.name for c in retry.changes} == {"RC1_TRIM", "RC1_MAX"}


def test_retry_excludes_failed_and_skipped_and_errors_if_nothing_left(
    make_fake: FakeFactory,
) -> None:
    fake = make_fake(param_write_errors={"RC1_MIN": mavlink.MAV_PARAM_ERROR_PERMISSION_DENIED})
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_MIN": 1000, "RC1_TRIM": 1490})
        mavforge.approve_plan(plan)
        result = vehicle.params.apply(plan)
    assert outcomes(result) == {
        "RC1_MIN": ChangeOutcome.FAILED,
        "RC1_TRIM": ChangeOutcome.VERIFIED,
    }
    with connect(fake) as vehicle, pytest.raises(InvalidInputError, match=r"[Nn]othing to retry"):
        vehicle.params.plan_retry(result)


def test_retry_unknown_plan_id_is_a_clear_error(fake: FakeVehicle) -> None:
    with connect(fake) as vehicle, pytest.raises(InvalidInputError, match=r"[Nn]o recorded result"):
        vehicle.params.plan_retry("0" * 12)


def test_audit_failure_blocks_writes(fake: FakeVehicle, isolated_state_dir: Path) -> None:
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490})
        mavforge.approve_plan(plan)
        (isolated_state_dir / "audit.jsonl").unlink()
        (isolated_state_dir / "audit.jsonl").mkdir()  # unwritable as a file
        with pytest.raises(NotPermittedError, match="audit"):
            vehicle.params.apply(plan)
    assert sets(fake) == []


def test_restore_from_pre_change_backup(fake: FakeVehicle) -> None:
    with connect(fake) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490, "BATT_FS_LOW_ACT": 1})
        mavforge.approve_plan(plan)
        applied = vehicle.params.apply(plan)
        assert applied.pre_change_backup is not None
        undo = vehicle.params.plan_restore(applied.pre_change_backup)
        assert undo.kind == "restore"
        assert {c.name: (c.current, c.requested) for c in undo.changes} == {
            "BATT_FS_LOW_ACT": (1, 2),
            "RC1_TRIM": (1490, 1500),
        }
        mavforge.approve_plan(undo)
        restored = vehicle.params.apply(undo)
        assert restored.status is OperationStatus.SUCCEEDED
        assert vehicle.params.get("RC1_TRIM").param.value == 1500


def test_restore_refuses_mismatched_or_anonymous_backups(
    make_fake: FakeFactory, tmp_path: Path
) -> None:
    other = make_fake(firmware=(4, 6, 2, 255))
    with mavforge.connect(other.connection_string) as vehicle:
        vehicle.params.backup(tmp_path / "other.json")
    (tmp_path / "anon.param").write_text("RC1_TRIM 1400\n")
    fake = make_fake()
    with connect(fake) as vehicle:
        with pytest.raises(InvalidInputError, match="does not clearly match"):
            vehicle.params.plan_restore(tmp_path / "other.json")
        with pytest.raises(InvalidInputError, match="does not clearly match"):
            vehicle.params.plan_restore(tmp_path / "anon.param")
        plan = vehicle.params.plan_restore(tmp_path / "anon.param", allow_mismatch=True)
        assert [c.name for c in plan.changes] == ["RC1_TRIM"]
        assert any(w.startswith("Identity:") for w in plan.warnings)
