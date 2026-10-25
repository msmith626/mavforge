"""Guided setup status: interprets diagnose's findings as ArduPilot's own first-time
setup sequence, rather than detecting anything new itself.

See docs/research.md, "Guided setup wizard": most of the "is this configured" signal
already exists, as diagnose findings. Two things genuinely can't be told from live
parameters alone and are reported ``unknown`` rather than guessed:

- Board orientation, gyro and level calibration: a value of exactly 0 is as likely to
  be correct as never-set. Unlike accel/compass offsets (practically never exactly
  zero after a real calibration -- see ``_accel_cal``/``_compass_cal``), a genuinely
  level, well-balanced board can legitimately have zero trim or gyro bias, so the same
  "exactly zero" test isn't safe for level/gyro either.
- A true "was this verified for this exact vehicle, and is it still valid" signal
  needs the audit log's read side, which doesn't exist yet (``AuditLog`` only appends
  today). This reads only live parameters; audit-backed verification is future work.
"""

from __future__ import annotations

from mavforge.domain.models import (
    DiagnosticReport,
    Finding,
    FindingKind,
    SetupReport,
    SetupStep,
    SetupStepStatus,
)
from mavforge.services import diagnostics as diagnostics_service
from mavforge.services.catalog import SETUP_STATUS, run_operation
from mavforge.services.session import Session

# Order matches ArduPilot's own setup sequence (docs/research.md, "Flight modes and
# failsafes", and the community sources it cites): orientation and sensor calibration
# first, then radio, then flight modes and failsafes, then everything else.
#
# (step id, title, command that addresses it, or None)
_STEPS: tuple[tuple[str, str, str | None], ...] = (
    ("orientation", "Board orientation", "mavforge calibrate orientation"),
    ("ardupilot.accel_cal", "Accelerometer calibration", "mavforge calibrate accel"),
    ("gyro", "Gyro calibration", "mavforge calibrate gyro"),
    ("level", "Level (AHRS trim)", "mavforge calibrate level"),
    ("ardupilot.compass_cal", "Compass calibration", "mavforge calibrate compass"),
    ("ardupilot.rc_cal", "Radio calibration", "mavforge calibrate radio"),
    (
        "ardupilot.flight_modes",
        "Flight mode assignment",
        "mavforge calibrate flight-modes FLTMODEn=VALUE ...",
    ),
    ("ardupilot.throttle_failsafe", "RC/throttle failsafe", "mavforge params set"),
    ("ardupilot.battery_failsafe", "Battery failsafe", "mavforge diagnose --propose-fixes"),
    ("ardupilot.gps_configured", "GPS configuration", "mavforge params set"),
    ("ardupilot.logging", "Onboard logging", "mavforge diagnose --propose-fixes"),
    ("ardupilot.arming_checks", "Pre-arm checks enabled", "mavforge params set"),
    ("ardupilot.frame_class", "Frame class", "mavforge params set"),
)

# These three have no diagnose rule, and deliberately don't get one: see the module
# docstring. Always reported unknown, regardless of what diagnose found.
_NO_RULE = {"orientation", "gyro", "level"}

_STATUS_FOR_KIND = {
    FindingKind.FAULT: SetupStepStatus.NEEDED,
    FindingKind.WARNING: SetupStepStatus.RECOMMENDED,
    FindingKind.SUSPECTED: SetupStepStatus.RECOMMENDED,
    FindingKind.PASSED: SetupStepStatus.DONE,
    FindingKind.NOT_CHECKED: SetupStepStatus.UNKNOWN,
}

_RELEVANT_STATUSES = frozenset(
    {SetupStepStatus.NEEDED, SetupStepStatus.RECOMMENDED, SetupStepStatus.UNKNOWN}
)


def _step_from_finding(step: str, title: str, run: str | None, finding: Finding) -> SetupStep:
    status = _STATUS_FOR_KIND.get(finding.kind, SetupStepStatus.UNKNOWN)
    return SetupStep(
        step=step,
        title=title,
        status=status,
        detail=finding.detail,
        run=run if status is not SetupStepStatus.DONE else None,
        finding=finding,
    )


def _steps_from_report(report: DiagnosticReport) -> list[SetupStep]:
    by_check = {f.check: f for f in report.findings}
    steps: list[SetupStep] = []
    for step, title, run in _STEPS:
        if step in _NO_RULE:
            steps.append(
                SetupStep(
                    step=step,
                    title=title,
                    status=SetupStepStatus.UNKNOWN,
                    detail="Can't be told from parameters alone; run the guided check "
                    "to know for sure.",
                    run=run,
                )
            )
            continue
        finding = by_check.get(step)
        if finding is None:
            steps.append(
                SetupStep(
                    step=step,
                    title=title,
                    status=SetupStepStatus.NOT_APPLICABLE,
                    detail="No rule for this vehicle family or firmware.",
                )
            )
            continue
        steps.append(_step_from_finding(step, title, run, finding))
    return steps


def setup_status(session: Session) -> SetupReport:
    """Read-only: one diagnose() call, interpreted as an ordered setup sequence."""

    def run() -> SetupReport:
        report = diagnostics_service.diagnose(session)
        steps = _steps_from_report(report)
        next_step = next((s for s in steps if s.status in _RELEVANT_STATUSES), None)
        return SetupReport(
            status=report.status,
            vehicle=report.vehicle,
            steps=steps,
            next_step=next_step,
        )

    return run_operation(session, SETUP_STATUS, run)
