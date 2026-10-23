"""The catalogue of MAVForge operations and the wrapper that enforces policy and audit.

Every interface (Python API, CLI, MCP) lists capabilities from ``CATALOG`` and runs
operations through ``run_operation``, so permission checks and audit records cannot
drift between interfaces.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from mavforge.domain.errors import MAVForgeError
from mavforge.policy.permissions import Permission

if TYPE_CHECKING:
    from mavforge.services.session import Session

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class OperationSpec:
    name: str
    summary: str
    permission: Permission
    modifies_vehicle: bool
    available: bool = True


INFO = OperationSpec(
    "info",
    "Identify the connected vehicle: autopilot, firmware, board, IDs.",
    Permission.READ,
    False,
)
STATUS = OperationSpec(
    "status",
    "Snapshot of arming state, mode, sensor health, pre-arm flag, battery, GPS and messages.",
    Permission.READ,
    False,
)
PARAMS_LIST = OperationSpec(
    "params_list", "Download all parameters, optionally filtered by a glob.", Permission.READ, False
)
PARAMS_GET = OperationSpec("params_get", "Read one parameter by name.", Permission.READ, False)

PARAMS_BACKUP = OperationSpec(
    "params_backup",
    "Save all parameters, with vehicle identity and firmware, to a new local backup file.",
    Permission.READ,
    False,
)
PARAMS_DIFF = OperationSpec(
    "params_diff",
    "Compare the vehicle's parameters with a backup or .param file.",
    Permission.READ,
    False,
)

DIAGNOSE = OperationSpec(
    "diagnose",
    "Classified findings (fault / warning / suspected / not checked / passed) from "
    "vehicle-reported pre-arm failures, sensor health and configuration rules.",
    Permission.READ,
    False,
)

PARAMS_PLAN = OperationSpec(
    "params_plan",
    "Validate requested parameter values and record a reviewable change plan "
    "(does not change the vehicle).",
    Permission.READ,
    False,
)
PARAMS_APPLY = OperationSpec(
    "params_apply",
    "Apply a human-approved plan: backup, write each value, verify by read-back.",
    Permission.CONFIGURE,
    True,
)

_PLANNED = [
    OperationSpec(
        "calibrate", "Deterministic calibration workflows.", Permission.CALIBRATE, True, False
    ),
]

CATALOG: list[OperationSpec] = [
    INFO,
    STATUS,
    PARAMS_LIST,
    PARAMS_GET,
    PARAMS_BACKUP,
    PARAMS_DIFF,
    DIAGNOSE,
    PARAMS_PLAN,
    PARAMS_APPLY,
    *_PLANNED,
]


def run_operation[T](
    session: Session, spec: OperationSpec, fn: Callable[[], T], **audit_fields: Any
) -> T:
    """Check policy, run ``fn`` and record the outcome in the audit log."""
    session.policy.require(spec.permission, spec.name)
    started = time.monotonic()
    status = "error"
    error: dict[str, Any] | None = None
    try:
        result = fn()
        status = str(getattr(result, "status", "succeeded"))
        return result
    except MAVForgeError as exc:
        error = exc.to_dict()
        raise
    finally:
        try:
            session.audit.record(
                "operation",
                operation=spec.name,
                interface=session.interface,
                vehicle={
                    "system_id": session.identity.system_id,
                    "autopilot": session.identity.autopilot,
                    "firmware": str(session.identity.firmware)
                    if session.identity.firmware
                    else None,
                    "simulated": session.identity.simulated,
                },
                connection=session.connection.description,
                status=status,
                error=error,
                duration_s=round(time.monotonic() - started, 3),
                **audit_fields,
            )
        except OSError as exc:
            # Never mask the operation's real outcome or error. Modifying operations
            # fail closed *before* writing (see changes.apply_plan); this is the record
            # of how it ended.
            log.error("Could not write audit log entry for %s: %s", spec.name, exc)
