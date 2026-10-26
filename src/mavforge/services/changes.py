"""Parameter changes: plan, then (human) approval, then apply with verification.

Planning reads current values, validates each requested value against its type and the
firmware's published metadata, and records risks for the approver. Applying re-checks
every gate (policy, approval, vehicle identity, disarmed state), takes a backup, refuses
values that changed since planning, writes one parameter at a time, and verifies each by
reading it back. A batch is not atomic; the result says exactly what happened to each.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from datetime import UTC, datetime
from pathlib import Path

from mavforge.domain.errors import (
    ConnectionFailedError,
    InvalidInputError,
    NotPermittedError,
    ParamNotFoundError,
)
from mavforge.domain.models import (
    AppliedChange,
    ApplyResult,
    ChangeOutcome,
    ChangePlan,
    ParamType,
    ParamValue,
    PlannedChange,
    VehicleIdentity,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.domain.values import format_value, normalize, values_equal
from mavforge.platforms.metadata import MetadataSet, ParamMeta
from mavforge.protocol import params as param_protocol
from mavforge.services import plan_store
from mavforge.services.backup import (
    build_backup,
    default_backup_stem,
    load_backup,
    write_new_default,
)
from mavforge.services.catalog import PARAMS_APPLY, PARAMS_PLAN, run_operation
from mavforge.services.diff import identity_warnings
from mavforge.services.session import Session
from mavforge.services.status import build_status

MAX_CHANGES = 2000
_INDIVIDUAL_READ_LIMIT = 20
_INT_BOUNDS: dict[ParamType, tuple[int, int]] = {
    ParamType.INT8: (-(2**7), 2**7 - 1),
    ParamType.UINT8: (0, 2**8 - 1),
    ParamType.INT16: (-(2**15), 2**15 - 1),
    ParamType.UINT16: (0, 2**16 - 1),
    ParamType.INT32: (-(2**31), 2**31 - 1),
    ParamType.UINT32: (0, 2**32 - 1),
}


def parse_assignments(items: list[str]) -> dict[str, float]:
    """``["NAME=VALUE", ...]`` from a command line into a mapping."""
    out: dict[str, float] = {}
    for item in items:
        name, sep, raw = item.partition("=")
        if not sep or not name.strip():
            raise InvalidInputError(f"Expected NAME=VALUE, got {item!r}")
        try:
            out[name.strip()] = float(raw)
        except ValueError:
            raise InvalidInputError(f"Invalid value in {item!r}") from None
    return out


# -- planning -----------------------------------------------------------------------


def _current_values(session: Session, names: list[str]) -> dict[str, ParamValue]:
    if len(names) <= _INDIVIDUAL_READ_LIMIT:
        found: dict[str, ParamValue] = {}
        for name in names:
            try:
                found[name] = param_protocol.read_param(
                    session.connection,
                    *session.target,
                    name,
                    timeout=session.options.request_timeout,
                )
            except ParamNotFoundError:
                continue
        return found
    download = param_protocol.fetch_all(session.connection, *session.target)
    return {p.name: p for p in download.params.values()}


def _label(meta: ParamMeta | None, value: float | int) -> str | None:
    if meta is None:
        return None
    if meta.values:
        return meta.values.get(float(value))
    if meta.bitmask:
        bits = [label for bit, label in sorted(meta.bitmask.items()) if int(value) & (1 << bit)]
        return ", ".join(bits) if bits else "none"
    return None


def _check_value(
    value: float, ptype: ParamType, meta: ParamMeta | None, allow_out_of_range: bool
) -> tuple[list[str], list[str]]:
    issues: list[str] = []
    warnings: list[str] = []
    if not math.isfinite(value):
        return ["Value must be a finite number."], []
    if ptype.is_integer:
        if not float(value).is_integer():
            issues.append(f"{ptype.value} parameter requires a whole number.")
        lo, hi = _INT_BOUNDS.get(ptype, (-(2**31), 2**31 - 1))
        if not lo <= value <= hi:
            issues.append(f"Outside the {ptype.value} range {lo}..{hi}.")
    if meta is None:
        return issues, warnings
    if meta.read_only:
        issues.append("The firmware marks this parameter read-only.")
    soft: list[str] = []
    if meta.range is not None and not meta.range[0] <= value <= meta.range[1]:
        soft.append(
            f"Outside the documented range "
            f"{format_value(meta.range[0])}..{format_value(meta.range[1])}."
        )
    if meta.values and not meta.bitmask and float(value) not in meta.values:
        soft.append("Not one of the documented values.")
    if soft:
        if allow_out_of_range:
            warnings.extend(s + " Allowed by explicit override." for s in soft)
        else:
            issues.extend(s + " Use the out-of-range override if this is intended." for s in soft)
    if meta.bitmask and ptype.is_integer:
        known = sum(1 << b for b in meta.bitmask)
        if int(value) & ~known:
            warnings.append("Sets bits that are not documented for this firmware.")
    if meta.volatile:
        warnings.append(
            "The firmware changes this parameter on its own; the value may not persist."
        )
    return issues, warnings


def plan_changes(
    session: Session,
    requested: Mapping[str, float | int],
    *,
    allow_out_of_range: bool = False,
    offline: bool = False,
    kind: str = "set",
    source: str | None = None,
    extra_warnings: list[str] | None = None,
) -> ChangePlan:
    """Validate requested values and record a reviewable plan. Does not change the vehicle."""

    def run() -> ChangePlan:
        if not requested:
            raise InvalidInputError("No parameter changes requested")
        if len(requested) > MAX_CHANGES:
            raise InvalidInputError(f"At most {MAX_CHANGES} changes per plan")
        normalized: dict[str, float] = {}
        for raw_name, value in requested.items():
            name = session.adapter.normalize_param_name(raw_name)
            if name in normalized:
                raise InvalidInputError(f"Parameter {name} requested more than once")
            normalized[name] = float(value)

        current = _current_values(session, list(normalized))
        metadata, meta_note = session.adapter.param_metadata(session.identity, offline=offline)
        changes = [
            _plan_one(session, name, value, current.get(name), metadata, allow_out_of_range)
            for name, value in normalized.items()
        ]
        warnings = list(extra_warnings or [])
        if session.identity.simulated is not True:
            warnings.append("REAL VEHICLE: these changes will be written to physical hardware.")
        if metadata is None or not metadata.version_matched:
            warnings.append(f"Parameter metadata: {meta_note}")
        if any(c.reboot_required and not c.no_change for c in changes):
            warnings.append("Some changes only take effect after the vehicle is rebooted.")
        draft = ChangePlan(
            plan_id="0" * 12,
            digest="",
            created_at=datetime.now(UTC),
            kind="restore" if kind == "restore" else "set",
            source=source,
            vehicle=session.identity,
            changes=changes,
            metadata_source=metadata.source if metadata else None,
            warnings=warnings,
            blocked=any(c.issues for c in changes),
        )
        digest = plan_store.compute_digest(draft)
        plan = draft.model_copy(update={"plan_id": digest[:12], "digest": digest})
        plan_store.save_plan(plan)
        return plan

    return run_operation(session, PARAMS_PLAN, run, changes=len(requested), kind=kind)


def _plan_one(
    session: Session,
    name: str,
    value: float,
    current: ParamValue | None,
    metadata: MetadataSet | None,
    allow_out_of_range: bool,
) -> PlannedChange:
    meta = metadata.get(name) if metadata else None
    if current is None:
        return PlannedChange(
            name=name,
            display_name=meta.display_name if meta else None,
            current=None,
            requested=value,
            type=ParamType.REAL32,
            no_change=False,
            issues=["The vehicle has no parameter with this name."],
        )
    fractional_int = current.type.is_integer and not float(value).is_integer()
    requested = value if fractional_int else normalize(value, current.type)
    issues, warnings = _check_value(value, current.type, meta, allow_out_of_range)
    notes_issues, notes_warnings = session.adapter.change_notes(name, current.value, requested)
    if current.precision_limited or (current.type.is_integer and abs(value) > 2**24):
        warnings.append("Large integer: MAVLink's float encoding may round this value.")
    return PlannedChange(
        name=name,
        display_name=meta.display_name if meta else None,
        current=current.value,
        requested=requested,
        type=current.type,
        units=meta.units if meta else None,
        range=meta.range if meta else None,
        current_label=_label(meta, current.value),
        requested_label=_label(meta, requested),
        reboot_required=meta.reboot_required if meta else None,
        no_change=values_equal(current.value, requested, current.type),
        issues=issues + notes_issues,
        warnings=warnings + notes_warnings,
    )


def plan_restore(
    session: Session,
    backup_path: Path,
    *,
    allow_mismatch: bool = False,
    offline: bool = False,
) -> ChangePlan:
    """Plan writing back every non-volatile value that differs from a backup."""
    backup = load_backup(backup_path)
    mismatch = identity_warnings(backup.vehicle, session.identity)
    if mismatch and not allow_mismatch:
        raise InvalidInputError(
            "The backup does not clearly match this vehicle; restoring could apply the wrong "
            "configuration. Re-run with the mismatch override only if you are sure.",
            details={"warnings": mismatch},
        )
    current = build_backup(session)
    now = {p.name: p for p in current.params}
    requested: dict[str, float | int] = {}
    skipped_missing: list[str] = []
    skipped_volatile = 0
    for p in backup.params:
        if p.name not in now:
            skipped_missing.append(p.name)
        elif values_equal(p.value, now[p.name].value, now[p.name].type):
            continue
        elif session.adapter.is_volatile_param(p.name):
            skipped_volatile += 1
        else:
            requested[p.name] = p.value
    if not requested:
        raise InvalidInputError("Nothing to restore: the vehicle already matches the backup")
    metadata, _ = session.adapter.param_metadata(session.identity, offline=offline)
    if metadata is not None:  # read-only values (device IDs) are hardware facts, not config
        read_only = [n for n in requested if (m := metadata.get(n)) is not None and m.read_only]
        for name in read_only:
            del requested[name]
    else:
        read_only = []
    warnings = [f"Identity: {w}" for w in mismatch]
    if not backup.complete:
        warnings.append("The backup is incomplete; parameters it lacks are not restored.")
    if skipped_missing:
        warnings.append(
            f"{len(skipped_missing)} parameter(s) in the backup do not exist on this vehicle "
            f"and are skipped (e.g. {', '.join(skipped_missing[:5])})."
        )
    if skipped_volatile:
        warnings.append(
            f"{skipped_volatile} volatile parameter(s) differ but are not restored "
            "(counters and boot-time calibration the firmware manages itself)."
        )
    if read_only:
        warnings.append(f"Read-only parameter(s) not restored: {', '.join(read_only[:10])}.")
    return plan_changes(
        session,
        requested,
        allow_out_of_range=True,  # restoring values the firmware previously held
        offline=offline,
        kind="restore",
        source=backup_path.name,
        extra_warnings=warnings,
    )


_RETRYABLE = frozenset({ChangeOutcome.UNKNOWN, ChangeOutcome.NOT_ATTEMPTED})
"""Only these mean "we don't actually know what happened, or definitely never tried" -
exactly a link-loss symptom. FAILED means the vehicle rejected the write or read-back
disagreed (retrying won't fix that on its own); SKIPPED means the baseline moved since
the plan was made (the drift-safety check, see apply_plan) - both need a person to look
again, not a blind retry."""


def plan_retry(session: Session, result: ApplyResult, *, offline: bool = False) -> ChangePlan:
    """Plan re-attempting whatever an earlier apply left unresolved (a link loss stopped
    it, typically): everything that was UNKNOWN or NOT_ATTEMPTED, at its original
    requested value. Goes through the same approve -> apply path as any other plan."""
    retry = {c.name: c.requested for c in result.changes if c.outcome in _RETRYABLE}
    if not retry:
        raise InvalidInputError(
            f"Nothing to retry in plan {result.plan_id}: every change already resolved "
            "(verified, unchanged, failed or skipped)."
        )
    return plan_changes(
        session,
        retry,
        offline=offline,
        kind="set",
        source=result.plan_id,
        extra_warnings=[f"Retrying unresolved changes from plan {result.plan_id}."],
    )


# -- applying -----------------------------------------------------------------------


def same_vehicle_problems(planned: VehicleIdentity, actual: VehicleIdentity) -> list[str]:
    problems: list[str] = []
    if (planned.system_id, planned.component_id) != (actual.system_id, actual.component_id):
        problems.append("different system/component ID")
    if planned.autopilot != actual.autopilot or planned.vehicle_family != actual.vehicle_family:
        problems.append("different autopilot or vehicle type")
    if (planned.firmware and planned.firmware.raw) != (actual.firmware and actual.firmware.raw):
        problems.append("different firmware version")
    if planned.uid and actual.uid and planned.uid != actual.uid:
        problems.append("different flight-controller board")
    return problems


def _overall(changes: list[AppliedChange]) -> OperationStatus:
    outcomes = {c.outcome for c in changes}
    good = {ChangeOutcome.VERIFIED, ChangeOutcome.UNCHANGED}
    if outcomes <= good:
        return OperationStatus.SUCCEEDED
    if ChangeOutcome.UNKNOWN in outcomes:
        return OperationStatus.UNKNOWN
    if outcomes & {ChangeOutcome.VERIFIED}:
        return OperationStatus.PARTIAL
    return OperationStatus.FAILED


def apply_plan(session: Session, plan_ref: str | Path) -> ApplyResult:
    """Apply an approved plan. Every safety gate is re-checked here, below all interfaces."""
    plan = plan_store.load_plan(plan_ref)

    def run() -> ApplyResult:
        session.policy.require_vehicle_writable(session.identity.simulated, PARAMS_APPLY.name)
        if plan.blocked:
            raise InvalidInputError(
                f"Plan {plan.plan_id} has blocking issues and cannot be applied"
            )
        approval = plan_store.valid_approval(plan)
        if approval is None:
            raise NotPermittedError(
                f"Plan {plan.plan_id} is not approved, or its approval expired. A human must "
                f"review and approve it, e.g. `mavforge approve {plan.plan_id}`.",
                details={"plan_id": plan.plan_id},
            )
        if problems := same_vehicle_problems(plan.vehicle, session.identity):
            raise NotPermittedError(
                f"Plan {plan.plan_id} was made for a different vehicle: {', '.join(problems)}",
                details={"plan_id": plan.plan_id},
            )
        if build_status(session).armed:
            raise NotPermittedError("The vehicle is armed. Disarm before changing parameters.")

        # Fail closed: nothing is written unless the intent is on the audit record first.
        _audit_or_refuse(
            session,
            "apply_started",
            plan_id=plan.plan_id,
            digest=plan.digest,
            approved_by=approval.approved_by,
            interface=session.interface,
            changes=[
                {"name": c.name, "current": c.current, "requested": c.requested}
                for c in plan.changes
                if not c.no_change
            ],
        )
        plan_store.consume(plan.plan_id)

        pre = build_backup(session)
        backup_path = write_new_default(
            default_backup_stem(session.identity, pre.created_at) + f"-pre-{plan.plan_id}",
            pre.model_dump_json(indent=2) + "\n",
        )
        before = {p.name: p.value for p in pre.params}
        results: list[AppliedChange] = []
        link_lost = False
        for change in plan.changes:
            now_value = before.get(change.name)
            if link_lost:
                results.append(
                    _record(
                        change,
                        now_value,
                        ChangeOutcome.NOT_ATTEMPTED,
                        None,
                        "Stopped after link loss",
                    )
                )
                continue
            if now_value is None:
                results.append(
                    _record(
                        change, None, ChangeOutcome.SKIPPED, None, "Parameter no longer present"
                    )
                )
                continue
            if change.current is None or not values_equal(now_value, change.current, change.type):
                results.append(
                    _record(
                        change,
                        now_value,
                        ChangeOutcome.SKIPPED,
                        now_value,
                        f"Changed since the plan was made (planned from "
                        f"{format_value(change.current or 0)}, now {format_value(now_value)})",
                    )
                )
                continue
            if change.no_change:
                results.append(_record(change, now_value, ChangeOutcome.UNCHANGED, now_value, None))
                continue
            try:
                written = param_protocol.write_param(
                    session.connection,
                    *session.target,
                    change.name,
                    change.requested,
                    change.type,
                    timeout=session.options.request_timeout,
                )
            except ConnectionFailedError:
                link_lost = True
                results.append(
                    _record(
                        change, now_value, ChangeOutcome.UNKNOWN, None, "Link lost during the write"
                    )
                )
                continue
            results.append(
                _record(change, now_value, written.outcome, written.after, written.detail)
            )

        result = ApplyResult(
            status=_overall(results),
            plan_id=plan.plan_id,
            vehicle=session.identity,
            approved_by=approval.approved_by,
            pre_change_backup=backup_path.name,
            changes=results,
            reboot_required=[
                c.name
                for c, r in zip(plan.changes, results, strict=True)
                if c.reboot_required and r.outcome is ChangeOutcome.VERIFIED
            ],
            warnings=(
                ["The link was lost; changes marked 'unknown' or 'not_attempted' need checking."]
                if link_lost
                else []
            ),
        )
        try:
            plan_store.record_result(result)
            session.audit.record(
                "apply_finished",
                plan_id=plan.plan_id,
                status=result.status.value,
                changes=[r.model_dump(mode="json") for r in results],
            )
        except OSError as exc:  # the vehicle has already been changed: report, don't hide
            result = result.model_copy(
                update={"warnings": [*result.warnings, f"Could not record the result: {exc}"]}
            )
        return result

    return run_operation(session, PARAMS_APPLY, run, plan_id=plan.plan_id)


def _audit_or_refuse(session: Session, event: str, **fields: object) -> None:
    try:
        session.audit.record(event, **fields)
    except OSError as exc:
        raise NotPermittedError(
            f"Cannot write the audit log ({exc}); refusing to change the vehicle without a record"
        ) from exc


def _record(
    change: PlannedChange,
    before: float | int | None,
    outcome: ChangeOutcome,
    after: float | int | None,
    detail: str | None,
) -> AppliedChange:
    return AppliedChange(
        name=change.name,
        before=before,
        requested=change.requested,
        after=after,
        outcome=outcome,
        detail=detail,
    )
