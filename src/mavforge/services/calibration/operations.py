"""Calibration requests, approvals and the registry of running calibration operations.

Starting a calibration follows the same rules as a parameter change: a request is
created, a human approves it (CLI prompt, ``mavforge approve``, MCP elicitation, or a
Python caller acting for a person), and the service re-checks policy, approval, vehicle
identity and disarmed state before anything is sent. The workflow then runs in a
background thread so every interface can poll status, confirm steps and cancel.
"""

from __future__ import annotations

import logging
import secrets
import threading
from collections.abc import Callable
from datetime import UTC, datetime
from pathlib import Path

from mavforge.domain.errors import (
    ConnectionFailedError,
    InvalidInputError,
    NotPermittedError,
    UnsupportedError,
)
from mavforge.domain.models import (
    CalibrationKind,
    CalibrationProcedure,
    CalibrationRequest,
    CalibrationResult,
    CalibrationState,
    CalibrationStatus,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.policy.permissions import Permission
from mavforge.services import plan_store
from mavforge.services.calibration.accel import AccelWorkflow
from mavforge.services.calibration.base import CalibrationCancelled, Workflow, WorkflowContext
from mavforge.services.calibration.compass import CompassWorkflow
from mavforge.services.calibration.flight_modes import FlightModeWorkflow
from mavforge.services.calibration.orientation import OrientationWorkflow
from mavforge.services.calibration.radio import RadioWorkflow
from mavforge.services.calibration.simple import BaroWorkflow, GyroWorkflow, LevelWorkflow
from mavforge.services.changes import same_vehicle_problems
from mavforge.services.session import Session
from mavforge.services.status import build_status

log = logging.getLogger(__name__)

_NO_REQUEST = (
    "{kind} calibration writes nothing itself and needs no request: start it directly. "
    "Its result is a change plan, which needs approval before it is applied."
)

WORKFLOWS: dict[CalibrationKind, Callable[[], Workflow]] = {
    CalibrationKind.GYRO: GyroWorkflow,
    CalibrationKind.LEVEL: LevelWorkflow,
    CalibrationKind.BARO: BaroWorkflow,
    CalibrationKind.COMPASS: CompassWorkflow,
    CalibrationKind.ACCEL: AccelWorkflow,
    CalibrationKind.RADIO: RadioWorkflow,
    CalibrationKind.ORIENTATION: OrientationWorkflow,
    CalibrationKind.FLIGHT_MODES: FlightModeWorkflow,
}

_STATE_FOR_STATUS = {
    OperationStatus.SUCCEEDED: CalibrationState.SUCCEEDED,
    OperationStatus.CANCELLED: CalibrationState.CANCELLED,
    OperationStatus.UNKNOWN: CalibrationState.UNKNOWN,
}


def procedures(session: Session | None = None) -> list[CalibrationProcedure]:
    """Every calibration MAVForge knows, with support for the connected vehicle if given."""
    out: list[CalibrationProcedure] = []
    for kind, factory in WORKFLOWS.items():
        workflow = factory()
        proc = workflow.procedure()
        if session is not None:
            reason = None
            if kind not in session.adapter.supported_calibrations():
                reason = f"Not supported by the {session.adapter.name} platform adapter."
            else:
                reason = workflow.check_supported(session.identity)
            proc = proc.model_copy(
                update={"supported": reason is None, "reason": reason or proc.reason}
            )
        out.append(proc)
    return out


def create_request(session: Session, kind: CalibrationKind) -> CalibrationRequest:
    """Record an approvable request to run ``kind`` on this vehicle. Sends nothing."""
    if not WORKFLOWS[kind]().writes_vehicle:
        raise InvalidInputError(_NO_REQUEST.format(kind=kind.value))
    draft = CalibrationRequest(
        plan_id="0" * 12,
        digest="",
        created_at=datetime.now(UTC),
        kind=kind,
        vehicle=session.identity,
    )
    digest = plan_store.compute_digest(draft)
    request = draft.model_copy(update={"plan_id": digest[:12], "digest": digest})
    plan_store.save_plan(request)
    return request


class _Operation:
    def __init__(self, ctx: WorkflowContext, workflow: Workflow) -> None:
        self.ctx = ctx
        self.workflow = workflow
        self.thread = threading.Thread(
            target=self._run, name=f"calibration-{ctx.operation_id}", daemon=True
        )

    def _run(self) -> None:
        ctx, workflow = self.ctx, self.workflow
        try:
            result = workflow.run(ctx)
        except CalibrationCancelled:
            warnings = workflow.on_cancel(ctx)
            result = CalibrationResult(
                status=OperationStatus.CANCELLED,
                summary=f"{ctx.kind.value} calibration cancelled.",
                warnings=warnings,
            )
        except ConnectionFailedError as exc:
            result = CalibrationResult(
                status=OperationStatus.UNKNOWN,
                summary=f"The link was lost during calibration ({exc.message}); the outcome "
                "on the vehicle is unknown.",
            )
        except Exception as exc:  # report, never leave an operation hanging
            log.exception("calibration %s crashed", ctx.operation_id)
            result = CalibrationResult(
                status=OperationStatus.FAILED, summary=f"Internal error: {exc}"
            )
        state = _STATE_FOR_STATUS.get(result.status, CalibrationState.FAILED)
        ctx.update(state=state, result=result, instruction=None, awaiting_confirmation=None)
        try:
            ctx.session.audit.record(
                "calibration_finished",
                operation_id=ctx.operation_id,
                kind=ctx.kind.value,
                status=result.status.value,
                summary=result.summary,
                params_after=result.params_after,
                plan_id=result.plan.plan_id if result.plan else None,
            )
        except OSError as exc:
            log.error("Could not audit calibration result: %s", exc)


class OperationRegistry:
    """Process-wide registry of calibration operations."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._ops: dict[str, _Operation] = {}

    def start(
        self,
        session: Session,
        request_ref: str | Path | CalibrationRequest,
        *,
        workflow: Workflow | None = None,
    ) -> CalibrationStatus:
        request = (
            request_ref
            if isinstance(request_ref, CalibrationRequest)
            else plan_store.load_request(request_ref)
        )
        operation = f"calibrate_{request.kind.value}"
        session.policy.require(Permission.CALIBRATE, operation)
        session.policy.require_vehicle_writable(session.identity.simulated, operation)
        wf = workflow or WORKFLOWS[request.kind]()
        if not wf.writes_vehicle:
            raise InvalidInputError(_NO_REQUEST.format(kind=request.kind.value))
        self._check_runnable(session, request.kind, wf)
        if plan_store.valid_approval(request) is None:
            raise NotPermittedError(
                f"Calibration request {request.plan_id} is not approved, or its approval "
                f"expired. A human must approve it, e.g. `mavforge approve {request.plan_id}`.",
                details={"request_id": request.plan_id},
            )
        if problems := same_vehicle_problems(request.vehicle, session.identity):
            raise NotPermittedError(
                f"Request {request.plan_id} was made for a different vehicle: {', '.join(problems)}"
            )
        self._check_idle_and_disarmed(session)
        try:
            session.audit.record(
                "calibration_started",
                request_id=request.plan_id,
                kind=request.kind.value,
                interface=session.interface,
                vehicle_system_id=session.identity.system_id,
            )
        except OSError as exc:
            raise NotPermittedError(
                f"Cannot write the audit log ({exc}); refusing to calibrate without a record"
            ) from exc
        plan_store.consume(request.plan_id)
        return self._launch(session, request.kind, wf)

    def start_observation(
        self, session: Session, kind: CalibrationKind, *, workflow: Workflow | None = None
    ) -> CalibrationStatus:
        """Start a calibration that only observes the vehicle and proposes a change plan.

        Needs only read permission: the sweep writes nothing, and the plan it produces
        goes through the normal approve -> apply path.
        """
        session.policy.require(Permission.READ, f"calibrate_{kind.value}")
        wf = workflow or WORKFLOWS[kind]()
        if wf.writes_vehicle:
            raise InvalidInputError(
                f"{kind.value} calibration changes the vehicle and needs an approved request"
            )
        self._check_runnable(session, kind, wf)
        self._check_idle_and_disarmed(session)
        try:
            session.audit.record(
                "calibration_started",
                kind=kind.value,
                interface=session.interface,
                vehicle_system_id=session.identity.system_id,
            )
        except OSError as exc:  # nothing is written, so a missing record is not fatal
            log.error("Could not audit calibration start: %s", exc)
        return self._launch(session, kind, wf)

    def _check_runnable(self, session: Session, kind: CalibrationKind, wf: Workflow) -> None:
        if kind not in session.adapter.supported_calibrations():
            raise UnsupportedError(f"{kind.value} calibration is not supported here")
        if reason := wf.check_supported(session.identity):
            raise UnsupportedError(reason)
        if session.interface == "mcp" and not wf.via_mcp:
            raise UnsupportedError(
                f"{kind.value} calibration needs the person at the vehicle to confirm "
                f"each step; run `mavforge calibrate {kind.value}` there instead."
            )

    def _check_idle_and_disarmed(self, session: Session) -> None:
        with self._lock:
            if any(
                not op.ctx.state.finished and op.ctx.session is session for op in self._ops.values()
            ):
                raise InvalidInputError("A calibration is already running on this vehicle")
        if build_status(session).armed:
            raise NotPermittedError("The vehicle is armed. Disarm before calibrating.")

    def _launch(self, session: Session, kind: CalibrationKind, wf: Workflow) -> CalibrationStatus:
        op_id = secrets.token_hex(6)
        op = _Operation(WorkflowContext(session, op_id, kind), wf)
        with self._lock:
            self._ops[op_id] = op
        op.thread.start()
        return op.ctx.snapshot()

    def _get(self, operation_id: str) -> _Operation:
        with self._lock:
            op = self._ops.get(operation_id)
        if op is None:
            raise InvalidInputError(f"No calibration operation {operation_id!r}")
        return op

    def status(self, operation_id: str) -> CalibrationStatus:
        return self._get(operation_id).ctx.snapshot()

    def confirm(self, operation_id: str, step: str) -> CalibrationStatus:
        op = self._get(operation_id)
        op.ctx.confirm(step)
        return op.ctx.snapshot()

    def cancel(self, operation_id: str) -> CalibrationStatus:
        op = self._get(operation_id)
        op.ctx.cancel_requested.set()
        op.thread.join(timeout=10)
        return op.ctx.snapshot()

    def wait(self, operation_id: str, timeout: float | None = None) -> CalibrationStatus:
        op = self._get(operation_id)
        op.thread.join(timeout)
        return op.ctx.snapshot()


REGISTRY = OperationRegistry()
