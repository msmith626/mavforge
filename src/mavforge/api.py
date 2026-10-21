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

from types import TracebackType
from typing import Self

from mavforge.domain.models import (
    CapabilitiesResult,
    InfoResult,
    ParamGetResult,
    ParamListResult,
    StatusReport,
    VehicleIdentity,
)
from mavforge.policy.audit import AuditLog
from mavforge.policy.permissions import Policy
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
