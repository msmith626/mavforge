"""Storage for change plans, their approvals and their (single) application.

Layout under ``<state>/plans/``::

    <plan_id>.json            the plan (immutable; content hashed into its digest)
    <plan_id>.approval.json   the approval, bound to the digest, with an expiry
    <plan_id>.applied.json    created atomically when application starts (single use)

Approvals can only be created through human-facing paths (CLI prompt, ``mavforge
approve``, MCP elicitation answered by the user, or a Python caller acting for a
human). No MCP tool creates one.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
from datetime import UTC, datetime, timedelta
from pathlib import Path

from pydantic import ValidationError

from mavforge.domain.errors import InvalidInputError
from mavforge.domain.models import (
    ApplyResult,
    Approval,
    CalibrationRequest,
    ChangePlan,
    PlanInfo,
    PlanListResult,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.policy.audit import default_state_dir

APPROVAL_TTL = timedelta(minutes=10)

Approvable = ChangePlan | CalibrationRequest
"""Anything a human approves before MAVForge acts on it."""
_PLAN_ID_RE = re.compile(r"^[0-9a-f]{12}$")


def plans_dir() -> Path:
    return default_state_dir() / "plans"


def compute_digest(plan: Approvable) -> str:
    content = plan.model_dump(mode="json", exclude={"plan_id", "digest"})
    canonical = json.dumps(content, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode()).hexdigest()


def _write_new(path: Path, text: str) -> None:
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        fh.write(text)


def _path(plan_id: str, suffix: str = "") -> Path:
    if not _PLAN_ID_RE.match(plan_id):
        raise InvalidInputError(f"Invalid plan id {plan_id!r}: expected 12 hex characters")
    return plans_dir() / f"{plan_id}{suffix}.json"


def save_plan(plan: Approvable) -> Path:
    path = _path(plan.plan_id)
    _write_new(path, plan.model_dump_json(indent=2) + "\n")
    return path


def load_approvable(reference: str | Path) -> Approvable:
    """Load a change plan or calibration request by id or path; verify its digest."""
    text_ref = str(reference)
    path = Path(reference) if not _PLAN_ID_RE.match(text_ref) else _path(text_ref)
    try:
        text = path.read_text(encoding="utf-8")
        kind = json.loads(text).get("format")
        model = CalibrationRequest if kind == "mavforge.calibration-request" else ChangePlan
        plan: Approvable = model.model_validate_json(text)
    except FileNotFoundError:
        raise InvalidInputError(f"No such plan: {reference}") from None
    except (OSError, ValueError, AttributeError) as exc:
        raise InvalidInputError(f"{path} is not a valid MAVForge plan") from exc
    if compute_digest(plan) != plan.digest or plan.digest[:12] != plan.plan_id:
        raise InvalidInputError(f"Plan {plan.plan_id} has been modified since it was created")
    if not _path(plan.plan_id).exists():
        # A plan file from elsewhere: store it so approvals and single use apply to it.
        save_plan(plan)
    return plan


def load_plan(reference: str | Path) -> ChangePlan:
    plan = load_approvable(reference)
    if not isinstance(plan, ChangePlan):
        raise InvalidInputError(f"{reference} is a calibration request, not a change plan")
    return plan


def load_request(reference: str | Path) -> CalibrationRequest:
    request = load_approvable(reference)
    if not isinstance(request, CalibrationRequest):
        raise InvalidInputError(f"{reference} is a change plan, not a calibration request")
    return request


def approve(plan: Approvable, approved_by: str, ttl: timedelta = APPROVAL_TTL) -> Approval:
    if plan.blocked:
        raise InvalidInputError(f"Plan {plan.plan_id} has blocking issues and cannot be approved")
    if is_applied(plan.plan_id):
        raise InvalidInputError(f"Plan {plan.plan_id} has already been applied")
    now = datetime.now(UTC)
    approval = Approval(
        plan_id=plan.plan_id,
        digest=plan.digest,
        approved_by=approved_by,
        approved_at=now,
        expires_at=now + ttl,
    )
    path = _path(plan.plan_id, ".approval")
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        fh.write(approval.model_dump_json(indent=2) + "\n")
    tmp.replace(path)
    return approval


def valid_approval(plan: Approvable) -> Approval | None:
    path = _path(plan.plan_id, ".approval")
    try:
        approval = Approval.model_validate_json(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, ValidationError):
        return None
    if approval.digest != plan.digest or approval.expires_at <= datetime.now(UTC):
        return None
    return approval


def is_applied(plan_id: str) -> bool:
    return _path(plan_id, ".applied").exists()


def consume(plan_id: str) -> None:
    """Mark the plan as being applied. Fails if it was ever applied before."""
    try:
        _write_new(_path(plan_id, ".applied"), '{"state": "in_progress"}\n')
    except FileExistsError:
        raise InvalidInputError(f"Plan {plan_id} has already been applied") from None


def record_result(result: ApplyResult) -> None:
    path = _path(result.plan_id, ".applied")
    path.write_text(result.model_dump_json(indent=2) + "\n", encoding="utf-8")


def list_plans() -> PlanListResult:
    infos: list[PlanInfo] = []
    directory = plans_dir()
    for path in sorted(directory.glob("*.json")) if directory.is_dir() else []:
        if not _PLAN_ID_RE.match(path.stem):
            continue
        try:
            plan = load_approvable(path.stem)
        except InvalidInputError:
            continue
        if not isinstance(plan, ChangePlan):
            continue
        v = plan.vehicle
        infos.append(
            PlanInfo(
                plan_id=plan.plan_id,
                created_at=plan.created_at,
                kind=plan.kind,
                vehicle=f"system {v.system_id} {v.autopilot} {v.firmware or ''}".strip(),
                change_count=sum(not c.no_change for c in plan.changes),
                blocked=plan.blocked,
                approved=valid_approval(plan) is not None,
                applied=is_applied(plan.plan_id),
            )
        )
    infos.sort(key=lambda i: i.created_at, reverse=True)
    return PlanListResult(status=OperationStatus.SUCCEEDED, plans=infos)
