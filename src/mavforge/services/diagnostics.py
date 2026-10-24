"""Diagnostics: gather evidence, apply generic and platform rules, classify findings.

Evidence comes from three places. Vehicle-reported pre-arm failures are requested with
MAV_CMD_RUN_PREARM_CHECKS, which makes the firmware run its checks and report them
without changing any state. Sensor health comes from SYS_STATUS. Configuration rules
run over the full parameter set.
"""

from __future__ import annotations

import time
from datetime import UTC, datetime, timedelta

from mavforge.domain.errors import MAVForgeError
from mavforge.domain.models import (
    DiagnosticReport,
    Finding,
    FindingKind,
    FindingSource,
    ParamBackup,
    StatusReport,
    StatusText,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.platforms.rules import ParamView, finding, run_rule
from mavforge.protocol.commands import CommandOutcome, send_command
from mavforge.protocol.messages import request_messages
from mavforge.services.backup import build_backup
from mavforge.services.catalog import DIAGNOSE, run_operation
from mavforge.services.changes import plan_changes
from mavforge.services.session import Session
from mavforge.services.status import build_status
from mavforge.transport.dialect import Message, mavlink

PREARM_PREFIX = "PreArm:"
DEFAULT_PREARM_WAIT = 3.0
PREARM_QUIET = 1.0
"""Stop waiting once no new pre-arm message has arrived for this long."""
PREARM_REPEAT_PERIOD = timedelta(seconds=35)
SETTLE_TIME_S = 60
"""Shortly after boot, EKF/sensor/pre-arm failures are often start-up transients."""
"""ArduPilot repeats pre-arm failures every 30 s; older texts may be stale."""

_KIND_ORDER = list(FindingKind)


def request_prearm_report(session: Session, wait: float) -> tuple[CommandOutcome, list[StatusText]]:
    """Ask the firmware to report pre-arm failures; return the PreArm texts it sent."""
    conn = session.connection
    sysid, compid = session.target

    def match(msg: Message) -> bool:
        return (
            isinstance(msg, mavlink.MAVLink_statustext_message)
            and msg.get_srcSystem() == sysid
            and str(msg.text).startswith(PREARM_PREFIX)
        )

    started = datetime.now(UTC)
    with conn.subscribe(match) as sub:
        outcome = send_command(
            conn,
            sysid,
            compid,
            mavlink.MAV_CMD_RUN_PREARM_CHECKS,
            idempotent=True,
            retries=1,
        )
        if outcome.status is OperationStatus.SUCCEEDED:
            deadline = time.monotonic() + wait
            while (remaining := deadline - time.monotonic()) > 0:
                if sub.get(min(PREARM_QUIET, remaining)) is None:
                    break
    since = (
        started if outcome.status is OperationStatus.SUCCEEDED else started - PREARM_REPEAT_PERIOD
    )
    texts: dict[str, StatusText] = {}
    for text in conn.statustexts(sysid):
        if text.text.startswith(PREARM_PREFIX) and text.received_at >= since:
            texts.setdefault(text.text, text)
    return outcome, list(texts.values())


def _prearm_findings(
    status: StatusReport, outcome: CommandOutcome, texts: list[StatusText]
) -> list[Finding]:
    check = "vehicle.prearm"
    findings = [
        Finding(
            check=check,
            kind=FindingKind.FAULT,
            title="Pre-arm check failed (reported by vehicle)",
            detail="The firmware reported this pre-arm failure; see vehicle_text.",
            source=FindingSource.VEHICLE,
            vehicle_text=text.text,
        )
        for text in texts
    ]
    if outcome.status is not OperationStatus.SUCCEEDED:
        findings.append(
            finding(
                check,
                FindingKind.NOT_CHECKED,
                "Vehicle pre-arm report not available",
                f"The request to run pre-arm checks was not accepted "
                f"({outcome.result or outcome.status.value}). ArduPilot refuses it while armed.",
                source=FindingSource.MAVFORGE,
            )
        )
    elif status.prearm_checks_enabled is False:
        findings.append(
            finding(
                check,
                FindingKind.NOT_CHECKED,
                "Vehicle's own pre-arm checks are disabled",
                "The firmware reports no pre-arm checks enabled, so it cannot explain "
                "what would block arming. Only MAVForge's own rules apply.",
                source=FindingSource.MAVFORGE,
            )
        )
    elif status.prearm_checks_passing is False and not texts:
        findings.append(
            finding(
                check,
                FindingKind.FAULT,
                "Pre-arm checks failing (no reason received)",
                "SYS_STATUS reports pre-arm checks failing but no PreArm message arrived.",
                source=FindingSource.TELEMETRY,
            )
        )
    elif status.prearm_checks_passing is True and not texts:
        findings.append(
            finding(
                check,
                FindingKind.PASSED,
                "Vehicle pre-arm checks passing",
                "The firmware's enabled pre-arm checks pass.",
                source=FindingSource.TELEMETRY,
            )
        )
    return findings


def vehicle_uptime(session: Session) -> float | None:
    """Seconds since the vehicle booted (SYSTEM_TIME), or None if not reported."""
    got = request_messages(
        session.connection,
        *session.target,
        [mavlink.MAVLINK_MSG_ID_SYSTEM_TIME],
        timeout=session.options.request_timeout,
    )
    msg = got[mavlink.MAVLINK_MSG_ID_SYSTEM_TIME]
    if isinstance(msg, mavlink.MAVLink_system_time_message):
        return msg.time_boot_ms / 1000
    return None


def _uptime_findings(uptime: float | None, status: StatusReport) -> list[Finding]:
    # Seen on ArduPlane 4.6.2 right after a reboot: mode INITIALISING, and requests for
    # SYS_STATUS, GPS_RAW_INT and SYSTEM_TIME went unanswered.
    if status.mode == "INITIALISING" or status.system_status in {"UNINIT", "BOOT"}:
        return [
            finding(
                "vehicle.uptime",
                FindingKind.WARNING,
                "Vehicle is still starting up",
                f"The vehicle reports mode {status.mode or '?'} / state "
                f"{status.system_status}. Most sensor, EKF and pre-arm findings in this "
                "report are likely start-up transients.",
                source=FindingSource.TELEMETRY,
                evidence={"mode": status.mode, "system_status": status.system_status},
                recommendation="Wait about a minute and re-run diagnostics.",
            )
        ]
    if uptime is None or uptime >= SETTLE_TIME_S:
        return []
    return [
        finding(
            "vehicle.uptime",
            FindingKind.WARNING,
            "Vehicle started recently",
            f"The vehicle booted {uptime:.0f} s ago. Sensor, EKF and pre-arm findings "
            "may be start-up transients.",
            source=FindingSource.TELEMETRY,
            evidence={"uptime_s": round(uptime, 1)},
            recommendation=f"Re-run diagnostics once the vehicle has been up for "
            f"{SETTLE_TIME_S} s or more.",
        )
    ]


def _telemetry_findings(status: StatusReport, session: Session) -> list[Finding]:
    findings: list[Finding] = []
    unhealthy = [s for s in status.sensors if s.enabled and not s.healthy]
    for sensor in unhealthy:
        findings.append(
            finding(
                "vehicle.sensor_health",
                FindingKind.FAULT,
                f"{sensor.sensor} reports unhealthy",
                "SYS_STATUS reports this subsystem enabled but unhealthy right now. On a "
                "bench this can be expected (no GPS fix indoors, RC transmitter off, "
                "no terrain data without GPS).",
                source=FindingSource.TELEMETRY,
                evidence={"sensor": sensor.sensor},
            )
        )
    if status.sensors and not unhealthy:
        findings.append(
            finding(
                "vehicle.sensor_health",
                FindingKind.PASSED,
                "All enabled sensors report healthy",
                f"{sum(s.enabled for s in status.sensors)} enabled subsystems.",
                source=FindingSource.TELEMETRY,
            )
        )
    for item in status.unavailable:
        findings.append(
            finding(
                "vehicle.telemetry",
                FindingKind.NOT_CHECKED,
                f"{item} not received",
                f"The vehicle did not answer a request for {item}; "
                "checks based on it were skipped.",
                source=FindingSource.MAVFORGE,
            )
        )
    if status.armed:
        findings.append(
            finding(
                "vehicle.armed",
                FindingKind.WARNING,
                "Vehicle is armed",
                "Diagnostics are read-only, but configuration work should be done disarmed.",
                source=FindingSource.TELEMETRY,
            )
        )
    if session.identity.other_systems_seen:
        findings.append(
            finding(
                "link.multiple_vehicles",
                FindingKind.WARNING,
                "Multiple vehicles on this link",
                f"Also heard system IDs {session.identity.other_systems_seen}; pin "
                "--target-system to be sure which vehicle is diagnosed.",
                source=FindingSource.MAVFORGE,
            )
        )
    return findings


def _param_findings(session: Session, backup: ParamBackup) -> list[Finding]:
    view = ParamView({p.name: p.value for p in backup.params})
    findings: list[Finding] = []
    if not backup.complete:
        findings.append(
            finding(
                "vehicle.params",
                FindingKind.NOT_CHECKED,
                "Parameter download incomplete",
                f"{len(backup.missing_indices)} parameters never arrived; rules that need "
                "them are reported as not checked.",
                source=FindingSource.MAVFORGE,
            )
        )
    for rule in session.adapter.param_rules():
        findings.extend(run_rule(rule, session.identity, view))
    return findings


def _propose_fix(session: Session, f: Finding) -> Finding:
    """Build a live, reviewable ChangePlan for a finding's proposed_changes, if any.

    Nothing is written: like every other plan, it still needs the normal approve ->
    apply path. If the plan can't be built (e.g. the vehicle rejects the values), the
    finding is returned unchanged rather than failing the whole diagnose() call.
    """
    if not f.proposed_changes:
        return f
    try:
        plan = plan_changes(
            session,
            f.proposed_changes,
            extra_warnings=[f"Proposed by diagnose finding {f.check!r}: {f.title}."],
        )
    except MAVForgeError:
        return f
    return f.model_copy(update={"proposed_plan": plan})


def build_report(
    session: Session,
    status: StatusReport,
    backup: ParamBackup,
    outcome: CommandOutcome,
    texts: list[StatusText],
    uptime: float | None = None,
    *,
    propose_fixes: bool = False,
) -> DiagnosticReport:
    findings = (
        _uptime_findings(uptime, status)
        + _prearm_findings(status, outcome, texts)
        + _telemetry_findings(status, session)
        + _param_findings(session, backup)
    )
    if propose_fixes:
        findings = [_propose_fix(session, f) for f in findings]
    findings.sort(key=lambda f: _KIND_ORDER.index(f.kind))
    partial = (
        bool(status.unavailable)
        or not backup.complete
        or outcome.status is not OperationStatus.SUCCEEDED
    )
    return DiagnosticReport(
        status=OperationStatus.PARTIAL if partial else OperationStatus.SUCCEEDED,
        vehicle=session.identity,
        findings=findings,
        counts={kind: sum(f.kind is kind for f in findings) for kind in FindingKind},
        prearm_requested=outcome.status is OperationStatus.SUCCEEDED,
        link=session.link_stats(),
    )


def diagnose(
    session: Session,
    *,
    prearm_wait: float = DEFAULT_PREARM_WAIT,
    propose_fixes: bool = False,
) -> DiagnosticReport:
    """Classified findings from pre-arm reports, sensor health and configuration rules.

    ``propose_fixes``: for findings with a safe, judgment-free default, also build a
    live ChangePlan (``Finding.proposed_plan``). Off by default: diagnose() is otherwise
    a pure read, and building a plan re-reads current values and saves a plan file
    (see services.changes.plan_changes) as a side effect.
    """

    def run() -> DiagnosticReport:
        status = build_status(session)
        uptime = vehicle_uptime(session)
        outcome, texts = request_prearm_report(session, prearm_wait)
        backup = build_backup(session)
        return build_report(
            session, status, backup, outcome, texts, uptime, propose_fixes=propose_fixes
        )

    return run_operation(session, DIAGNOSE, run)
