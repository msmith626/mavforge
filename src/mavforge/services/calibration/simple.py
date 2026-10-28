"""Gyroscope, level (AHRS trim) and barometer calibration.

ArduPilot runs all three synchronously inside MAV_CMD_PREFLIGHT_CALIBRATION and only
ACKs when done (GCS_Common.cpp, EXPECT_DELAY_MS(30000)), so the command is sent once
with a long timeout and never retransmitted. None can be interrupted once started.
"""

from __future__ import annotations

from mavforge.domain.models import (
    CalibrationKind,
    CalibrationProcedure,
    CalibrationResult,
    CalibrationState,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.protocol.commands import send_command
from mavforge.services.calibration.base import Workflow, WorkflowContext
from mavforge.transport.dialect import mavlink

ACK_TIMEOUT = 35.0
GYRO_PARAMS = [f"INS_GYR{n}OFFS_{a}" for n in ("", "2", "3") for a in "XYZ"]
TRIM_PARAMS = ["AHRS_TRIM_X", "AHRS_TRIM_Y"]
BARO_PARAMS = [f"BARO{n}_GND_PRESS" for n in ("1", "2", "3")]


def preflight_command(ctx: WorkflowContext, params: list[float]) -> tuple[OperationStatus, str]:
    outcome = send_command(
        ctx.session.connection,
        *ctx.session.target,
        mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        params,
        timeout=ACK_TIMEOUT,
    )
    return outcome.status, outcome.result or "no acknowledgement"


class GyroWorkflow(Workflow):
    kind = CalibrationKind.GYRO

    def procedure(self) -> CalibrationProcedure:
        return CalibrationProcedure(
            kind=self.kind,
            title="Gyroscope calibration",
            summary="Measures gyro offsets while the vehicle is motionless.",
            preconditions=["Vehicle disarmed", "Vehicle completely still (not hand-held)"],
            human_steps=["Leave the vehicle untouched until the calibration finishes."],
            changes_parameters=["INS_GYROFFS_*", "INS_GYR2OFFS_*", "INS_GYR3OFFS_*"],
            typical_duration="a few seconds",
            success_evidence="Vehicle acknowledges ACCEPTED (it checks every gyro calibrated); "
            "offsets read back afterwards.",
            cancellation="Cannot be interrupted; it finishes within seconds.",
            available_via_mcp=True,
            supported=True,
        )

    def run(self, ctx: WorkflowContext) -> CalibrationResult:
        before = ctx.read_params(GYRO_PARAMS)
        ctx.update(
            state=CalibrationState.RUNNING,
            instruction="Keep the vehicle completely still.",
        )
        status, result = preflight_command(ctx, [1, 0, 0, 0, 0, 0, 0])
        if status is not OperationStatus.SUCCEEDED:
            return _command_failure("Gyro calibration", status, result, before)
        ctx.update(state=CalibrationState.VERIFYING, instruction=None)
        after = ctx.read_params(list(before))
        return CalibrationResult(
            status=OperationStatus.SUCCEEDED,
            summary="Gyro calibration completed.",
            params_before=before,
            params_after=after,
            evidence=[f"PREFLIGHT_CALIBRATION(gyro) acknowledged {result}", "Offsets read back"],
        )


class LevelWorkflow(Workflow):
    kind = CalibrationKind.LEVEL

    def procedure(self) -> CalibrationProcedure:
        return CalibrationProcedure(
            kind=self.kind,
            title="Level (AHRS trim) calibration",
            summary="Records the vehicle's level attitude as the AHRS trim reference. "
            "Runs a gyro calibration first.",
            preconditions=[
                "Vehicle disarmed",
                "Vehicle resting in its level flying attitude, completely still",
                "Not within 5 s of another accelerometer calibration",
            ],
            human_steps=[
                "Place the vehicle in its level flying attitude on a stable surface.",
                "Leave it untouched until the calibration finishes.",
            ],
            changes_parameters=["AHRS_TRIM_X", "AHRS_TRIM_Y", "INS_GYROFFS_*"],
            typical_duration="a few seconds",
            success_evidence="Vehicle acknowledges ACCEPTED; AHRS_TRIM_X/Y read back.",
            cancellation="Cannot be interrupted; it finishes within seconds.",
            available_via_mcp=True,
            supported=True,
        )

    def run(self, ctx: WorkflowContext) -> CalibrationResult:
        before = ctx.read_params(TRIM_PARAMS)
        ctx.update(
            state=CalibrationState.RUNNING,
            instruction="Keep the vehicle level and completely still.",
        )
        status, result = preflight_command(ctx, [0, 0, 0, 0, 2, 0, 0])
        if status is not OperationStatus.SUCCEEDED:
            return _command_failure("Level calibration", status, result, before)
        ctx.update(state=CalibrationState.VERIFYING, instruction=None)
        after = ctx.read_params(TRIM_PARAMS)
        return CalibrationResult(
            status=OperationStatus.SUCCEEDED,
            summary="Level calibration completed; new AHRS trim saved.",
            params_before=before,
            params_after=after,
            evidence=[f"PREFLIGHT_CALIBRATION(level) acknowledged {result}", "AHRS_TRIM read back"],
        )


class BaroWorkflow(Workflow):
    kind = CalibrationKind.BARO

    def procedure(self) -> CalibrationProcedure:
        return CalibrationProcedure(
            kind=self.kind,
            title="Barometer calibration",
            summary="Sets the current pressure as the ground-level (zero altitude) "
            "reference for each barometer.",
            preconditions=["Vehicle disarmed", "Vehicle stationary at its take-off location"],
            human_steps=["Leave the vehicle untouched until the calibration finishes."],
            changes_parameters=BARO_PARAMS,
            typical_duration="a few seconds",
            success_evidence="Vehicle acknowledges ACCEPTED; ground pressure read back.",
            cancellation="Cannot be interrupted; it finishes within seconds.",
            available_via_mcp=True,
            supported=True,
        )

    def run(self, ctx: WorkflowContext) -> CalibrationResult:
        before = ctx.read_params(BARO_PARAMS)
        ctx.update(
            state=CalibrationState.RUNNING,
            instruction="Keep the vehicle still at its current location.",
        )
        status, result = preflight_command(ctx, [0, 0, 1, 0, 0, 0, 0])
        if status is not OperationStatus.SUCCEEDED:
            return _command_failure("Barometer calibration", status, result, before)
        ctx.update(state=CalibrationState.VERIFYING, instruction=None)
        after = ctx.read_params(list(before))
        return CalibrationResult(
            status=OperationStatus.SUCCEEDED,
            summary="Barometer calibration completed; ground pressure updated.",
            params_before=before,
            params_after=after,
            evidence=[
                f"PREFLIGHT_CALIBRATION(baro) acknowledged {result}",
                "Ground pressure read back",
            ],
        )


def _command_failure(
    what: str, status: OperationStatus, result: str, before: dict[str, float | int]
) -> CalibrationResult:
    hints = {
        OperationStatus.REJECTED: " The vehicle is busy or another calibration ran within "
        "the last 5 s; wait and retry.",
        OperationStatus.FAILED: " Check the vehicle messages (e.g. 'Disarm to allow "
        "calibration', vehicle moved during sampling).",
        OperationStatus.UNKNOWN: " No acknowledgement arrived; the calibration may or may "
        "not have run. Check the link and read the parameters.",
    }
    return CalibrationResult(
        status=status,
        summary=f"{what} did not complete: vehicle answered {result}.{hints.get(status, '')}",
        params_before=before,
        evidence=[f"PREFLIGHT_CALIBRATION acknowledged {result}"],
    )
