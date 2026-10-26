"""The public Python API.

Example::

    import mavforge

    with mavforge.connect("tcp:127.0.0.1:5760") as vehicle:
        print(vehicle.info().vehicle.firmware)
        for p in vehicle.params.list("COMPASS_*").params:
            print(p.name, p.value)

The CLI and MCP server are thin layers over this same API.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from types import TracebackType
from typing import Self

from mavforge.domain.models import (
    ApplyResult,
    Approval,
    BackupListResult,
    BackupResult,
    CalibrationKind,
    CalibrationProcedure,
    CalibrationRequest,
    CalibrationStatus,
    CapabilitiesResult,
    ChangePlan,
    DiagnosticReport,
    InfoResult,
    ParamBackup,
    ParamDiffResult,
    ParamGetResult,
    ParamListResult,
    PlanListResult,
    SetupReport,
    StatusReport,
    VehicleIdentity,
)
from mavforge.policy.audit import AuditLog
from mavforge.policy.permissions import Policy
from mavforge.services import backup as backup_service
from mavforge.services import changes as changes_service
from mavforge.services import diagnostics as diagnostics_service
from mavforge.services import diff as diff_service
from mavforge.services import info as info_service
from mavforge.services import params as params_service
from mavforge.services import plan_store
from mavforge.services import setup as setup_service
from mavforge.services import status as status_service
from mavforge.services.calibration import operations as calibration_ops
from mavforge.services.calibration.flight_modes import FlightModeWorkflow
from mavforge.services.calibration.orientation import OrientationWorkflow
from mavforge.services.calibration.radio import RadioWorkflow
from mavforge.services.diagnostics import DEFAULT_PREARM_WAIT
from mavforge.services.session import (
    HEARTBEAT_TIMEOUT,
    ConnectOptions,
    Session,
    open_session,
)
from mavforge.transport.connection import DEFAULT_SOURCE_COMPONENT, DEFAULT_SOURCE_SYSTEM


class Params:
    def __init__(self, session: Session) -> None:
        self._session = session

    def list(self, pattern: str | None = None) -> ParamListResult:
        return params_service.list_params(self._session, pattern)

    def get(self, name: str) -> ParamGetResult:
        return params_service.get_param(self._session, name)

    def backup(
        self, path: str | Path | None = None, *, param_file: str | Path | None = None
    ) -> BackupResult:
        """Save all parameters with vehicle identity to a new JSON file.

        ``path`` defaults to a timestamped file in the MAVForge backup directory.
        ``param_file`` additionally exports a plain ``.param`` file. Never overwrites.
        """
        return backup_service.create_backup(
            self._session,
            Path(path) if path is not None else None,
            Path(param_file) if param_file is not None else None,
        )

    def plan(
        self,
        changes: Mapping[str, float | int],
        *,
        allow_out_of_range: bool = False,
        offline: bool = False,
    ) -> ChangePlan:
        """Validate ``{NAME: value}`` changes and save a plan. Changes nothing on the vehicle.

        The plan must be approved by a human (:func:`approve_plan`) before :meth:`apply`.
        """
        return changes_service.plan_changes(
            self._session, changes, allow_out_of_range=allow_out_of_range, offline=offline
        )

    def plan_restore(
        self, backup: str | Path, *, allow_mismatch: bool = False, offline: bool = False
    ) -> ChangePlan:
        """Plan restoring every non-volatile value that differs from a backup."""
        return changes_service.plan_restore(
            self._session,
            backup_service.resolve_reference(backup),
            allow_mismatch=allow_mismatch,
            offline=offline,
        )

    def apply(self, plan: str | Path | ChangePlan) -> ApplyResult:
        """Apply an approved plan, verifying every write. Requires the ``configure``
        permission (and ``allow_real_vehicle`` for hardware) in the session policy."""
        ref = plan.plan_id if isinstance(plan, ChangePlan) else plan
        return changes_service.apply_plan(self._session, ref)

    def plan_retry(self, result: ApplyResult | str, *, offline: bool = False) -> ChangePlan:
        """Plan re-attempting whatever an earlier :meth:`apply` left unresolved (a link
        loss stopped it, typically) at its original requested value. ``result`` is the
        ``ApplyResult`` itself, or the id of the plan that produced it. Goes through
        the same approve -> apply path as any other plan."""
        loaded = result if isinstance(result, ApplyResult) else plan_store.load_result(result)
        return changes_service.plan_retry(self._session, loaded, offline=offline)

    def diff(self, reference: str | Path) -> ParamDiffResult:
        """Compare the vehicle with a backup (path or backup id) or a ``.param`` file."""
        return diff_service.diff_vehicle(self._session, backup_service.resolve_reference(reference))


class Calibration:
    """Calibration workflows. Each run that changes the vehicle needs an approved request
    (see :func:`approve_plan`); the radio calibration and the orientation check instead
    propose an approvable plan."""

    def __init__(self, session: Session) -> None:
        self._session = session

    def procedures(self) -> list[CalibrationProcedure]:
        """What each calibration does, needs and changes, and whether this vehicle supports it."""
        return calibration_ops.procedures(self._session)

    def request(self, kind: CalibrationKind | str) -> CalibrationRequest:
        """Create a request to run ``kind``. A human must approve it before :meth:`start`."""
        return calibration_ops.create_request(self._session, CalibrationKind(kind))

    def start(self, request: str | Path | CalibrationRequest) -> CalibrationStatus:
        """Start an approved calibration in the background; poll :meth:`status`."""
        return calibration_ops.REGISTRY.start(self._session, request)

    def start_radio(self, *, offline: bool = False) -> CalibrationStatus:
        """Start a radio (RC) calibration sweep in the background; poll :meth:`status`.

        Needs no approval to start: it writes nothing. Its result carries a change plan
        (``result.plan``) that a human must approve before :meth:`Params.apply`.
        """
        return calibration_ops.REGISTRY.start_observation(
            self._session, CalibrationKind.RADIO, workflow=RadioWorkflow(offline=offline)
        )

    def start_orientation(self, *, offline: bool = False) -> CalibrationStatus:
        """Start a board orientation check (three poses) in the background.

        Like :meth:`start_radio`, it writes nothing; ``result.plan`` proposes
        AHRS_ORIENTATION, or keeps the current value if it already matches.
        """
        return calibration_ops.REGISTRY.start_observation(
            self._session,
            CalibrationKind.ORIENTATION,
            workflow=OrientationWorkflow(offline=offline),
        )

    def start_flight_modes(
        self, requested: dict[str, float], *, offline: bool = False
    ) -> CalibrationStatus:
        """Start a flight-mode check: watches FLTMODE_CH while you flip the switch, then
        proposes ``requested`` (FLTMODE1..FLTMODE6, e.g. ``{"FLTMODE3": 5}``) as a change
        plan once every requested position has been confirmed reachable.

        Like :meth:`start_radio`, it writes nothing; ``result.plan`` proposes the
        requested values, flagging any that the switch never actually produced.
        """
        return calibration_ops.REGISTRY.start_observation(
            self._session,
            CalibrationKind.FLIGHT_MODES,
            workflow=FlightModeWorkflow(requested, offline=offline),
        )

    def status(self, operation_id: str) -> CalibrationStatus:
        return calibration_ops.REGISTRY.status(operation_id)

    def confirm(self, operation_id: str, step: str) -> CalibrationStatus:
        """Confirm a physical step (e.g. an accelerometer pose) the person has completed."""
        return calibration_ops.REGISTRY.confirm(operation_id, step)

    def cancel(self, operation_id: str) -> CalibrationStatus:
        return calibration_ops.REGISTRY.cancel(operation_id)

    def wait(self, operation_id: str, timeout: float | None = None) -> CalibrationStatus:
        return calibration_ops.REGISTRY.wait(operation_id, timeout)


class Vehicle:
    """A connected vehicle. Create with :func:`connect`; close when done."""

    def __init__(self, session: Session) -> None:
        self.session = session
        self.params = Params(session)
        self.calibration = Calibration(session)

    @property
    def identity(self) -> VehicleIdentity:
        return self.session.identity

    def info(self) -> InfoResult:
        return info_service.info(self.session)

    def status(self) -> StatusReport:
        return status_service.snapshot(self.session)

    def diagnose(
        self, *, prearm_wait: float = DEFAULT_PREARM_WAIT, propose_fixes: bool = False
    ) -> DiagnosticReport:
        """Classified findings from pre-arm reports, sensor health and configuration rules.

        Read-only: also asks the firmware to run (not change) its pre-arm checks.
        ``propose_fixes``: also build a live ChangePlan for findings with a safe,
        judgment-free default (``Finding.proposed_plan``). Nothing is written: the
        plan still needs the normal approve -> apply path.
        """
        return diagnostics_service.diagnose(
            self.session, prearm_wait=prearm_wait, propose_fixes=propose_fixes
        )

    def setup_status(self) -> SetupReport:
        """ArduPilot's own first-time setup sequence, as a checklist derived from
        diagnose()'s findings: done / needed / recommended / unknown / not_applicable
        per step. Read-only; see services/setup.py for what each status means and why
        a few steps (board orientation, gyro, level) are always reported ``unknown``.
        """
        return setup_service.setup_status(self.session)

    @property
    def is_connected(self) -> bool:
        return self.session.connection.is_open

    def close(self) -> None:
        self.session.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()


def connect(
    connection: str,
    *,
    target_system: int | None = None,
    source_system: int = DEFAULT_SOURCE_SYSTEM,
    source_component: int = DEFAULT_SOURCE_COMPONENT,
    timeout: float = HEARTBEAT_TIMEOUT,
    policy: Policy | None = None,
    audit: AuditLog | None = None,
    interface: str = "api",
) -> Vehicle:
    """Connect, wait for a vehicle heartbeat, and identify the vehicle.

    ``policy`` defaults to read-only. ``audit`` defaults to the per-user audit log
    (``$MAVFORGE_STATE_DIR`` or ``$XDG_STATE_HOME/mavforge/audit.jsonl``).
    """
    options = ConnectOptions(
        connection=connection,
        target_system=target_system,
        source_system=source_system,
        source_component=source_component,
        heartbeat_timeout=timeout,
    )
    return Vehicle(open_session(options, policy=policy, audit=audit, interface=interface))


def capabilities(policy: Policy | None = None) -> CapabilitiesResult:
    return info_service.capabilities(policy)


def list_backups() -> BackupListResult:
    """Backups in the MAVForge backup directory (no vehicle needed)."""
    return backup_service.list_backups()


def load_backup(path: str | Path) -> ParamBackup:
    """Load a MAVForge JSON backup or a plain ``.param`` file."""
    return backup_service.load_backup(Path(path))


def diff_backups(reference: str | Path, current: str | Path) -> ParamDiffResult:
    """Compare two saved files (paths or backup ids) without contacting a vehicle."""
    return diff_service.diff_files(
        backup_service.resolve_reference(reference), backup_service.resolve_reference(current)
    )


def load_plan(plan: str | Path) -> ChangePlan:
    """Load a change plan by id or path (its content digest is verified)."""
    return plan_store.load_plan(plan)


def approve_plan(
    plan: str | Path | ChangePlan | CalibrationRequest, *, approved_by: str = "api"
) -> Approval:
    """Record a human's approval of a change plan or calibration request (valid for 10
    minutes, bound to its digest, single use).

    Call this only on behalf of a person who has reviewed it. AI agents using the MCP
    server have no way to call it.
    """
    loaded = (
        plan
        if isinstance(plan, ChangePlan | CalibrationRequest)
        else plan_store.load_approvable(plan)
    )
    return plan_store.approve(loaded, approved_by)


def calibration_procedures() -> list[CalibrationProcedure]:
    """Every calibration MAVForge knows (no vehicle needed)."""
    return calibration_ops.procedures()


def list_plans() -> PlanListResult:
    return plan_store.list_plans()
