"""Six-position accelerometer calibration (ArduPilot AP_AccelCal).

Sequence (AP_AccelCal.cpp, GCS_Common.cpp at Plane-4.6.2):

1. MAV_CMD_PREFLIGHT_CALIBRATION param5=1. ArduPilot first runs a gyro calibration
   synchronously, then ACKs and starts the accelerometer calibration.
2. The vehicle repeatedly (1 Hz) sends COMMAND_LONG MAV_CMD_ACCELCAL_VEHICLE_POS with
   param1 = the pose it wants (1..6). MAVForge asks the person to place the vehicle and
   confirm; only then does it echo the same command and pose back, which makes the
   vehicle sample.
3. The vehicle ends by sending param1 = ACCELCAL_VEHICLE_POS_SUCCESS (16777215) or
   _FAILED (16777216), plus STATUSTEXT "Calibration successful" / "Calibration FAILED".

There is no MAVLink command to cancel it: ArduPilot only cancels on arming. A cancelled
run leaves the autopilot waiting in calibration mode until it is rebooted.
"""

from __future__ import annotations

import time

from mavforge.domain.models import (
    CalibrationKind,
    CalibrationProcedure,
    CalibrationResult,
    CalibrationState,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.protocol.commands import send_command
from mavforge.services.calibration.base import Workflow, WorkflowContext
from mavforge.services.calibration.simple import ACK_TIMEOUT
from mavforge.transport.dialect import Message, mavlink

POS_SUCCESS = 16777215
POS_FAILED = 16777216
POSES: dict[int, tuple[str, str]] = {
    1: ("LEVEL", "level, in its normal flying attitude"),
    2: ("LEFT", "on its LEFT side"),
    3: ("RIGHT", "on its RIGHT side"),
    4: ("NOSEDOWN", "nose DOWN, pointing at the ground"),
    5: ("NOSEUP", "nose UP, pointing at the sky"),
    6: ("BACK", "on its BACK (upside down)"),
}
POSE_TIMEOUT = 600.0
SILENCE_TIMEOUT = 30.0
"""The vehicle re-requests the pose every second; silence this long means it stopped."""

STUCK_WARNING = (
    "ArduPilot cannot cancel an accelerometer calibration over MAVLink. The autopilot stays "
    "in calibration mode, waiting for the next position, until it is rebooted. Reboot it "
    "before use. Nothing is saved unless all six positions complete."
)


def _accel_params() -> list[str]:
    return [
        f"INS_ACC{i}{kind}_{a}" for i in ("", "2", "3") for kind in ("OFFS", "SCAL") for a in "XYZ"
    ]


class AccelWorkflow(Workflow):
    kind = CalibrationKind.ACCEL
    via_mcp = False

    def __init__(
        self, pose_timeout: float = POSE_TIMEOUT, silence_timeout: float = SILENCE_TIMEOUT
    ) -> None:
        self.pose_timeout = pose_timeout
        self.silence_timeout = silence_timeout

    def procedure(self) -> CalibrationProcedure:
        return CalibrationProcedure(
            kind=self.kind,
            title="Accelerometer calibration (6 positions)",
            summary="Calibrates accelerometer offsets and scale from six orientations.",
            preconditions=[
                "Vehicle disarmed",
                "A person at the vehicle who can place it in six orientations",
                "A flat, stable surface (a box helps for the nose-up/down positions)",
            ],
            human_steps=[
                f"When asked, place the vehicle {desc}, hold it completely still, then confirm."
                for _, desc in POSES.values()
            ],
            changes_parameters=["INS_ACC*OFFS_*", "INS_ACC*SCAL_*", "INS_GYROFFS_*"],
            typical_duration="2-5 minutes",
            success_evidence="Vehicle sends ACCELCAL_VEHICLE_POS_SUCCESS (and 'Calibration "
            "successful'); offsets read back non-zero.",
            cancellation=STUCK_WARNING,
            available_via_mcp=False,
            supported=True,
            reason="Each pose must be confirmed by the person holding the vehicle; use "
            "`mavforge calibrate accel` at the vehicle.",
        )

    def on_cancel(self, ctx: WorkflowContext) -> list[str]:
        return [STUCK_WARNING]

    def run(self, ctx: WorkflowContext) -> CalibrationResult:
        sysid, compid = ctx.session.target
        before = ctx.read_params(_accel_params())

        def position_request(msg: Message) -> bool:
            return (
                isinstance(msg, mavlink.MAVLink_command_long_message)
                and msg.command == mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS
                and msg.get_srcSystem() == sysid
            )

        with ctx.session.connection.subscribe(position_request) as requests:
            ctx.update(
                state=CalibrationState.RUNNING,
                instruction="Keep the vehicle still (gyro calibration first).",
            )
            outcome = send_command(
                ctx.session.connection,
                sysid,
                compid,
                mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                [0, 0, 0, 0, 1, 0, 0],
                timeout=ACK_TIMEOUT,
            )
            if outcome.status is not OperationStatus.SUCCEEDED:
                return CalibrationResult(
                    status=outcome.status,
                    summary=f"The vehicle did not start accelerometer calibration "
                    f"({outcome.result or 'no acknowledgement'}).",
                    params_before=before,
                )
            done: list[str] = []
            evidence: list[str] = []
            last_heard = time.monotonic()
            while True:
                ctx.check_cancelled()
                msg = requests.get(0.5)
                if msg is None:
                    if time.monotonic() - last_heard > self.silence_timeout:
                        return CalibrationResult(
                            status=OperationStatus.UNKNOWN,
                            summary="The vehicle stopped requesting positions; the outcome "
                            "is unknown.",
                            params_before=before,
                            warnings=[STUCK_WARNING],
                        )
                    continue
                last_heard = time.monotonic()
                assert isinstance(msg, mavlink.MAVLink_command_long_message)
                pose = int(msg.param1)
                if pose == POS_SUCCESS:
                    evidence.append("Vehicle reported ACCELCAL_VEHICLE_POS_SUCCESS")
                    break
                if pose == POS_FAILED:
                    return CalibrationResult(
                        status=OperationStatus.FAILED,
                        summary="The vehicle reported accelerometer calibration FAILED. Make "
                        "sure each position is held completely still and roughly square.",
                        params_before=before,
                        evidence=["Vehicle reported ACCELCAL_VEHICLE_POS_FAILED"],
                    )
                if pose not in POSES or POSES[pose][0] in done:
                    continue
                name, description = POSES[pose]
                instruction = (
                    f"Position {len(done) + 1} of 6: place the vehicle {description}, hold it "
                    "completely still, then confirm."
                )
                if not ctx.wait_for_confirmation(name, instruction, self.pose_timeout):
                    return CalibrationResult(
                        status=OperationStatus.TIMED_OUT,
                        summary=f"No confirmation for position {name} within "
                        f"{self.pose_timeout:.0f} s.",
                        params_before=before,
                        warnings=[STUCK_WARNING],
                    )
                ack = send_command(
                    ctx.session.connection,
                    sysid,
                    compid,
                    mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS,
                    [pose],
                    timeout=2.0,
                )
                if ack.status is OperationStatus.SUCCEEDED:
                    done.append(name)
                    evidence.append(f"Position {name} accepted")
                ctx.update(
                    state=CalibrationState.RUNNING,
                    instruction="Hold the vehicle still while it samples this position.",
                )
                last_heard = time.monotonic()

        ctx.update(
            state=CalibrationState.VERIFYING,
            instruction="Calibration finished; you can put the vehicle down.",
        )
        after = ctx.read_params(list(before))
        offsets = [after.get(f"INS_ACCOFFS_{a}") for a in "XYZ"]
        if not offsets or all(v in (None, 0) for v in offsets):
            return CalibrationResult(
                status=OperationStatus.UNKNOWN,
                summary="The vehicle reported success, but the saved offsets read back as zero.",
                params_before=before,
                params_after=after,
                evidence=evidence,
            )
        evidence.append("INS_ACCOFFS_* read back non-zero")
        return CalibrationResult(
            status=OperationStatus.SUCCEEDED,
            summary="Accelerometer calibration succeeded and was saved.",
            params_before=before,
            params_after=after,
            evidence=evidence,
        )
