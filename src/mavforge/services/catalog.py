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
PARAMS_DIFF_DEFAULTS = OperationSpec(
    "params_diff_defaults",
    "Compare the vehicle's parameters with the firmware's own shipped defaults.",
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

SETUP_STATUS = OperationSpec(
    "setup_status",
    "ArduPilot's own first-time setup sequence, as a checklist derived from diagnose's "
    "findings (done / needed / recommended / unknown / not_applicable per step).",
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

CALIBRATE = OperationSpec(
    "calibrate",
    "Run an approved calibration (gyro, level, barometer, compass, 6-position accel) "
    "as a state machine with evidence-based success.",
    Permission.CALIBRATE,
    True,
)

CALIBRATE_RADIO = OperationSpec(
    "calibrate_radio",
    "Guided RC stick sweep that proposes RCn_MIN/MAX/TRIM as a change plan (writes "
    "nothing itself; the plan is applied through params_apply after approval).",
    Permission.READ,
    False,
)

CALIBRATE_ORIENTATION = OperationSpec(
    "calibrate_orientation",
    "Guided three-pose board orientation check that proposes AHRS_ORIENTATION as a change "
    "plan (writes nothing itself; the plan is applied through params_apply after approval).",
    Permission.READ,
    False,
)

CALIBRATE_FLIGHT_MODES = OperationSpec(
    "calibrate_flight_modes",
    "Guided FLTMODE_CH switch sweep that proposes FLTMODE1..6 as a change plan (writes "
    "nothing itself; the plan is applied through params_apply after approval).",
    Permission.READ,
    False,
)

_PLANNED: list[OperationSpec] = []

CATALOG: list[OperationSpec] = [
    INFO,
    STATUS,
    PARAMS_LIST,
    PARAMS_GET,
    PARAMS_BACKUP,
    PARAMS_DIFF,
    PARAMS_DIFF_DEFAULTS,
    DIAGNOSE,
    SETUP_STATUS,
    PARAMS_PLAN,
    PARAMS_APPLY,
    CALIBRATE,
    CALIBRATE_RADIO,
    CALIBRATE_ORIENTATION,
    CALIBRATE_FLIGHT_MODES,
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
