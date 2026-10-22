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

from pathlib import Path
from types import TracebackType
from typing import Self

from mavforge.domain.models import (
    BackupListResult,
    BackupResult,
    CapabilitiesResult,
    InfoResult,
    ParamBackup,
    ParamDiffResult,
    ParamGetResult,
    ParamListResult,
    StatusReport,
    VehicleIdentity,
)
from mavforge.policy.audit import AuditLog
from mavforge.policy.permissions import Policy
from mavforge.services import backup as backup_service
from mavforge.services import diff as diff_service
from mavforge.services import info as info_service
from mavforge.services import params as params_service
from mavforge.services import status as status_service
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
