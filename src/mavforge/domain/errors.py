"""Exception hierarchy shared by every MAVForge interface.

Each error carries a stable machine-readable ``code`` so the CLI can map it to an exit
code and the MCP server can return it as structured data. Messages must never contain
secrets.
"""

from __future__ import annotations

from enum import IntEnum
from typing import Any


class ExitCode(IntEnum):
    """Process exit codes used by the ``mavforge`` CLI."""

    OK = 0
    FAILED = 1
    USAGE = 2
    CONNECTION = 3
    TIMEOUT = 4
    NOT_PERMITTED = 5
    UNSUPPORTED = 6
    PARTIAL = 7


class MAVForgeError(Exception):
    """Base class for all errors raised deliberately by MAVForge."""

    code: str = "error"
    exit_code: ExitCode = ExitCode.FAILED

    def __init__(self, message: str, *, details: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.message = message
        self.details: dict[str, Any] = details or {}

    def to_dict(self) -> dict[str, Any]:
        return {"error": self.code, "message": self.message, "details": self.details}


class InvalidInputError(MAVForgeError):
    """A caller supplied a malformed or unsupported argument."""

    code = "invalid_input"
    exit_code = ExitCode.USAGE


class ConnectionFailedError(MAVForgeError):
    """The MAVLink link could not be opened or was lost."""

    code = "connection_failed"
    exit_code = ExitCode.CONNECTION


class NoVehicleError(ConnectionFailedError):
    """The link opened but no suitable vehicle heartbeat was seen."""

    code = "no_vehicle"


class OperationTimeoutError(MAVForgeError):
    """No conclusive response arrived in time; the vehicle-side outcome is unknown."""

    code = "timeout"
    exit_code = ExitCode.TIMEOUT


class NotPermittedError(MAVForgeError):
    """The active policy does not grant the permission an operation requires."""

    code = "not_permitted"
    exit_code = ExitCode.NOT_PERMITTED


class UnsupportedError(MAVForgeError):
    """The vehicle, firmware or platform adapter does not support the operation."""

    code = "unsupported"
    exit_code = ExitCode.UNSUPPORTED


class ParamNotFoundError(MAVForgeError):
    """The vehicle reported (or implied) that a parameter does not exist."""

    code = "param_not_found"


class ProtocolError(MAVForgeError):
    """The vehicle responded in a way that violates the expected protocol."""

    code = "protocol_error"
