"""setup_status's interpretation logic, against a synthetic DiagnosticReport.

No vehicle needed: this only exercises how Finding.kind maps to SetupStepStatus and
how the three no-rule steps (orientation, gyro, level) are always reported unknown.
See docs/research.md, "Guided setup wizard", for why.
"""

from __future__ import annotations

from mavforge.domain.models import (
    DiagnosticReport,
    Finding,
    FindingKind,
    FindingSource,
    LinkStats,
    SetupStepStatus,
    VehicleIdentity,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.services.setup import _steps_from_report

IDENTITY = VehicleIdentity(
    system_id=1,
    component_id=1,
    autopilot="ARDUPILOTMEGA",
    mav_type="FIXED_WING",
    vehicle_family="plane",
    platform="ardupilot",
)
LINK = LinkStats(
    connection="fake", messages_received=0, receive_errors=0, seconds_since_heartbeat=0
)


def report(findings: list[Finding]) -> DiagnosticReport:
    return DiagnosticReport(
        status=OperationStatus.SUCCEEDED,
        vehicle=IDENTITY,
        findings=findings,
        counts={k: sum(f.kind is k for f in findings) for k in FindingKind},
        prearm_requested=True,
        link=LINK,
    )


def finding(check: str, kind: FindingKind) -> Finding:
    return Finding(check=check, kind=kind, title="t", detail="d", source=FindingSource.PARAMETERS)


def by_step(findings: list[Finding]) -> dict[str, SetupStepStatus]:
    return {s.step: s.status for s in _steps_from_report(report(findings))}


def test_orientation_gyro_and_level_are_always_unknown() -> None:
    statuses = by_step([])
    assert statuses["orientation"] is SetupStepStatus.UNKNOWN
    assert statuses["gyro"] is SetupStepStatus.UNKNOWN
    assert statuses["level"] is SetupStepStatus.UNKNOWN


def test_missing_rule_is_not_applicable() -> None:
    # No ardupilot.frame_class finding at all (e.g. a plane): not_applicable, not unknown.
    assert by_step([])["ardupilot.frame_class"] is SetupStepStatus.NOT_APPLICABLE


def test_finding_kinds_map_to_the_right_status() -> None:
    statuses = by_step(
        [
            finding("ardupilot.compass_cal", FindingKind.FAULT),
            finding("ardupilot.battery_failsafe", FindingKind.WARNING),
            finding("ardupilot.rc_cal", FindingKind.SUSPECTED),
            finding("ardupilot.accel_cal", FindingKind.PASSED),
            finding("ardupilot.gps_configured", FindingKind.NOT_CHECKED),
        ]
    )
    assert statuses["ardupilot.compass_cal"] is SetupStepStatus.NEEDED
    assert statuses["ardupilot.battery_failsafe"] is SetupStepStatus.RECOMMENDED
    assert statuses["ardupilot.rc_cal"] is SetupStepStatus.RECOMMENDED
    assert statuses["ardupilot.accel_cal"] is SetupStepStatus.DONE
    assert statuses["ardupilot.gps_configured"] is SetupStepStatus.UNKNOWN


def test_flight_modes_passed_is_unknown_not_done() -> None:
    # Found on real hardware (2026-10-26): a non-zero FLTMODE_CH (-> PASSED from the
    # rule) doesn't mean the channel carries a real signal, unlike every other PASSED
    # finding. The SUSPECTED case (disabled) is unaffected and stays RECOMMENDED.
    statuses = by_step(
        [
            finding("ardupilot.flight_modes", FindingKind.PASSED),
            finding("ardupilot.accel_cal", FindingKind.PASSED),
        ]
    )
    assert statuses["ardupilot.flight_modes"] is SetupStepStatus.UNKNOWN
    assert statuses["ardupilot.accel_cal"] is SetupStepStatus.DONE  # unaffected

    suspected = by_step([finding("ardupilot.flight_modes", FindingKind.SUSPECTED)])
    assert suspected["ardupilot.flight_modes"] is SetupStepStatus.RECOMMENDED


def test_done_steps_carry_no_run_command() -> None:
    steps = _steps_from_report(report([finding("ardupilot.accel_cal", FindingKind.PASSED)]))
    accel = next(s for s in steps if s.step == "ardupilot.accel_cal")
    assert accel.run is None
    assert accel.finding is not None


def test_steps_are_reported_in_ardupilot_setup_order() -> None:
    steps = _steps_from_report(report([]))
    assert [s.step for s in steps][:3] == ["orientation", "ardupilot.accel_cal", "gyro"]
