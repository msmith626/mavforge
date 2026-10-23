"""Onboard compass calibration (ArduPilot AP_Compass_Calibration / CompassCalibrator).

Sequence (verified against ArduCopter 4.7.2 SITL, see docs/research.md):

1. MAV_CMD_DO_START_MAG_CAL(mask=0 all, retry=0, autosave=0, delay=0, autoreboot=0).
2. Poll MAG_CAL_PROGRESS (191) and MAG_CAL_REPORT (192) with MAV_CMD_REQUEST_MESSAGE.
   They travel in a telemetry stream MAVForge does not change, so are not otherwise sent.
   One request for 192 makes ArduPilot send a report for *every* finished compass
   (Compass::send_mag_cal_report loops over all instances), so the replies are collected
   from a subscription held open for the whole run, not as one reply per request.
3. When every calibrating compass reports, require MAG_CAL_SUCCESS from all of them.
   With autosave off nothing is saved yet (verified), so a failure changes nothing.
4. MAV_CMD_DO_ACCEPT_MAG_CAL(0) saves the results; read the offsets back and require
   them to match the reported offsets.
5. ArduPilot then needs a reboot before using the calibration (pre-arm: "Compass
   calibrated requires reboot"). MAVForge does not reboot; it tells the user.
"""

from __future__ import annotations

import math
import time

from mavforge.domain.models import (
    CalibrationKind,
    CalibrationProcedure,
    CalibrationResult,
    CalibrationState,
    CompassCalProgress,
    VehicleIdentity,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.protocol.commands import command_long, send_command
from mavforge.services.calibration.base import Workflow, WorkflowContext
from mavforge.transport.dialect import Message, enum_name, mavlink

POLL_INTERVAL = 0.5
DEFAULT_TIMEOUT = 300.0
SUCCESS = 4  # MAG_CAL_SUCCESS; every status >= 4 is terminal (FAILED, FAILED_* ...)
INSTANCES = ("", "2", "3")
OFFSET_TOLERANCE = 0.5  # milligauss; params are float32 copies of the reported floats

ROTATE = (
    "Rotate the vehicle slowly and steadily through every orientation: point the nose, "
    "each side, the top and the bottom towards the ground in turn, away from metal "
    "objects, until progress reaches 100%."
)

_FAILURE_HINTS = {
    "FAILED": "rotate more evenly through all orientations, away from metal and electronics",
    # ArduPilot's CompassCalibrator calls these BAD_ORIENTATION / BAD_RADIUS
    "FAILED_ORIENTATION": "the compass orientation looks wrong; check COMPASS_ORIENT* and mounting",
    "FAILED_RADIUS": "the field strength is implausible; move away from magnetic interference "
    "or check the sensor",
}


def _status_name(value: int) -> str:
    name = enum_name("MAG_CAL_STATUS", value, strip_prefix=False)
    return name.removeprefix("MAG_CAL_")


def _offset_names() -> list[str]:
    return [f"COMPASS_OFS{i}_{a}" for i in INSTANCES for a in "XYZ"]


class CompassWorkflow(Workflow):
    kind = CalibrationKind.COMPASS

    def __init__(self, timeout: float = DEFAULT_TIMEOUT) -> None:
        self.timeout = timeout

    def procedure(self) -> CalibrationProcedure:
        return CalibrationProcedure(
            kind=self.kind,
            title="Compass (magnetometer) calibration",
            summary="Onboard calibration of all compasses while you rotate the vehicle.",
            preconditions=[
                "Vehicle disarmed",
                "At least one compass detected (COMPASS_DEV_ID non-zero)",
                "Away from metal, vehicles, speakers and power cables",
                "Flight battery connected if the compass is near power wiring",
            ],
            human_steps=[ROTATE],
            changes_parameters=["COMPASS_OFS*", "COMPASS_DIA*", "COMPASS_ODI*", "COMPASS_SCALE*"],
            typical_duration="about 1 minute of rotation",
            success_evidence="Every compass reports MAG_CAL_SUCCESS; results accepted; saved "
            "offsets read back and match the reported offsets.",
            cancellation="Cancellable at any time (MAV_CMD_DO_CANCEL_MAG_CAL); nothing is "
            "saved unless every compass succeeded.",
            available_via_mcp=True,
            supported=True,
        )

    def check_supported(self, identity: VehicleIdentity) -> str | None:
        if "COMPASS_CALIBRATION" not in identity.protocol_capabilities:
            return "The vehicle does not advertise onboard compass calibration."
        return None

    def _command(self, ctx: WorkflowContext, command: int, params: list[float]) -> str:
        outcome = send_command(ctx.session.connection, *ctx.session.target, command, params)
        return outcome.result or "no acknowledgement"

    def on_cancel(self, ctx: WorkflowContext) -> list[str]:
        result = self._command(ctx, mavlink.MAV_CMD_DO_CANCEL_MAG_CAL, [0])
        if result != "ACCEPTED":
            return [
                f"The vehicle answered {result} to the cancel request; it may still be calibrating."
            ]
        return []

    def run(self, ctx: WorkflowContext) -> CalibrationResult:
        devices = ctx.read_params([f"COMPASS_DEV_ID{i}" for i in INSTANCES])
        before = ctx.read_params(_offset_names())
        if not any(devices.values()):
            return CalibrationResult(
                status=OperationStatus.FAILED,
                summary="No compass is detected (all COMPASS_DEV_ID are 0).",
                params_before=before,
            )
        started = self._command(ctx, mavlink.MAV_CMD_DO_START_MAG_CAL, [0, 0, 0, 0, 0])
        if started != "ACCEPTED":
            return CalibrationResult(
                status=OperationStatus.FAILED,
                summary=f"The vehicle refused to start compass calibration ({started}); see "
                "the vehicle messages.",
                params_before=before,
                evidence=[f"DO_START_MAG_CAL acknowledged {started}"],
            )
        ctx.update(state=CalibrationState.RUNNING, instruction=ROTATE, progress_pct=0)
        progress: dict[int, CompassCalProgress] = {}
        reports: dict[int, mavlink.MAVLink_mag_cal_report_message] = {}
        mask = 0
        deadline = time.monotonic() + self.timeout
        sysid, compid = ctx.session.target

        def cal_message(msg: Message) -> bool:
            return msg.get_srcSystem() == sysid and isinstance(
                msg,
                mavlink.MAVLink_mag_cal_progress_message | mavlink.MAVLink_mag_cal_report_message,
            )

        with ctx.session.connection.subscribe(cal_message) as sub:
            while True:
                if time.monotonic() > deadline:
                    self.on_cancel(ctx)
                    return CalibrationResult(
                        status=OperationStatus.TIMED_OUT,
                        summary=f"Compass calibration did not finish within {self.timeout:.0f} "
                        "s and was cancelled. Nothing was saved.",
                        params_before=before,
                    )
                for msg_id in (191, 192):
                    ctx.session.connection.send(
                        command_long(sysid, compid, mavlink.MAV_CMD_REQUEST_MESSAGE, [msg_id])
                    )
                poll_end = time.monotonic() + POLL_INTERVAL
                while (remaining := poll_end - time.monotonic()) > 0:
                    ctx.check_cancelled()
                    msg = sub.get(remaining)
                    if isinstance(msg, mavlink.MAVLink_mag_cal_progress_message):
                        mask |= msg.cal_mask
                        if msg.compass_id not in reports:
                            progress[msg.compass_id] = CompassCalProgress(
                                compass_id=msg.compass_id,
                                status=_status_name(msg.cal_status),
                                completion_pct=msg.completion_pct,
                            )
                    elif isinstance(msg, mavlink.MAVLink_mag_cal_report_message) and (
                        msg.cal_status >= SUCCESS
                    ):
                        mask |= msg.cal_mask
                        reports[msg.compass_id] = msg
                        progress[msg.compass_id] = CompassCalProgress(
                            compass_id=msg.compass_id,
                            status=_status_name(msg.cal_status),
                            completion_pct=100 if msg.cal_status == SUCCESS else None,
                            fitness=round(msg.fitness, 2),
                            offsets=(msg.ofs_x, msg.ofs_y, msg.ofs_z),
                        )
                ids = [i for i in range(8) if mask & (1 << i)] or sorted(progress)
                pcts = [progress[i].completion_pct or 0 for i in ids if i in progress]
                ctx.update(
                    compasses=[progress[i] for i in sorted(progress)],
                    progress_pct=min(pcts) if pcts else 0,
                )
                if ids and all(i in reports for i in ids):
                    break

        failed = {i: r for i, r in reports.items() if r.cal_status != SUCCESS}
        evidence = [
            f"compass {i}: {_status_name(r.cal_status)} fitness {r.fitness:.1f}"
            for i, r in sorted(reports.items())
        ]
        if failed:
            self.on_cancel(ctx)
            reasons = "; ".join(
                f"compass {i} {_status_name(r.cal_status)}: "
                f"{_FAILURE_HINTS.get(_status_name(r.cal_status), 'retry the calibration')}"
                for i, r in sorted(failed.items())
            )
            return CalibrationResult(
                status=OperationStatus.FAILED,
                summary=f"Compass calibration failed ({reasons}). Nothing was saved.",
                params_before=before,
                evidence=evidence,
            )

        ctx.update(state=CalibrationState.VERIFYING, instruction="You can stop rotating.")
        accepted = self._command(ctx, mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL, [0])
        after = ctx.read_params(_offset_names())
        evidence.append(f"DO_ACCEPT_MAG_CAL acknowledged {accepted}")
        saved = [_offsets(after, i) for i in INSTANCES]
        unmatched = [
            i
            for i, r in reports.items()
            if not any(
                s is not None and math.dist(s, (r.ofs_x, r.ofs_y, r.ofs_z)) < OFFSET_TOLERANCE
                for s in saved
            )
        ]
        if accepted != "ACCEPTED" or unmatched:
            return CalibrationResult(
                status=OperationStatus.UNKNOWN
                if accepted == "no acknowledgement"
                else OperationStatus.FAILED,
                summary="Every compass calibrated successfully, but saving could not be "
                f"confirmed (accept: {accepted}; unmatched compasses: {unmatched}).",
                params_before=before,
                params_after=after,
                evidence=evidence,
            )
        evidence.append("Saved COMPASS_OFS* read back and match the reported offsets")
        return CalibrationResult(
            status=OperationStatus.SUCCEEDED,
            summary=f"Compass calibration succeeded for {len(reports)} compass(es) and was saved.",
            params_before=before,
            params_after=after,
            evidence=evidence,
            reboot_required=True,
            warnings=[
                "Reboot the vehicle before flight: ArduPilot only uses the new compass "
                "calibration after a reboot (pre-arm: 'Compass calibrated requires reboot')."
            ],
        )


def _offsets(params: dict[str, float | int], instance: str) -> tuple[float, float, float] | None:
    names = [f"COMPASS_OFS{instance}_{a}" for a in "XYZ"]
    if not all(n in params for n in names):
        return None
    x, y, z = (float(params[n]) for n in names)
    return (x, y, z)
