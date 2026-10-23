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
    CapabilitiesResult,
    ChangePlan,
    DiagnosticReport,
    InfoResult,
    ParamBackup,
    ParamDiffResult,
    ParamGetResult,
    ParamListResult,
    PlanListResult,
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
from mavforge.services import status as status_service
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

    def diff(self, reference: str | Path) -> ParamDiffResult:
        """Compare the vehicle with a backup (path or backup id) or a ``.param`` file."""
        return diff_service.diff_vehicle(self._session, backup_service.resolve_reference(reference))


class Vehicle:
    """A connected vehicle. Create with :func:`connect`; close when done."""

    def __init__(self, session: Session) -> None:
        self.session = session
        self.params = Params(session)

    @property
    def identity(self) -> VehicleIdentity:
        return self.session.identity

    def info(self) -> InfoResult:
        return info_service.info(self.session)

    def status(self) -> StatusReport:
        return status_service.snapshot(self.session)

    def diagnose(self, *, prearm_wait: float = DEFAULT_PREARM_WAIT) -> DiagnosticReport:
        """Classified findings from pre-arm reports, sensor health and configuration rules.

        Read-only: also asks the firmware to run (not change) its pre-arm checks.
        """
        return diagnostics_service.diagnose(self.session, prearm_wait=prearm_wait)

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


def approve_plan(plan: str | Path | ChangePlan, *, approved_by: str = "api") -> Approval:
    """Record a human's approval of a plan (valid for 10 minutes, bound to its digest).

    Call this only on behalf of a person who has reviewed the plan. AI agents using the
    MCP server have no way to call it.
    """
    loaded = plan if isinstance(plan, ChangePlan) else plan_store.load_plan(plan)
    return plan_store.approve(loaded, approved_by)


def list_plans() -> PlanListResult:
    return plan_store.list_plans()
