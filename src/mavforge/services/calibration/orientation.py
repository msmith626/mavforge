"""Board orientation check: three guided poses that find the right AHRS_ORIENTATION.

ArduPilot rotates every accelerometer and gyro sample from the board's axes into the
vehicle's by AHRS_ORIENTATION (AP_InertialSensor_Backend::_rotate_and_correct_accel), and
reports the result in SCALED_IMU. MAVForge asks the person to hold the vehicle level,
nose up, then right side down. Gravity in each pose, as currently reported, gives the
vehicle's down, forward and right axes in the current frame; combining those with the
current setting gives the board's true mounting, which is matched against ArduPilot's
standard rotations. Like the radio calibration, nothing is written: the result is a
change plan.

Rotation semantics (verified against ArduPilot Plane-4.6.2 libraries/AP_Math/vector3.cpp
Vector3::rotate): rotation N maps a board-frame vector v to the body frame as
``Rz(yaw) @ Ry(pitch) @ Rx(roll) @ v`` with the angles in the rotation's name; e.g.
ROTATION_YAW_90 maps the board's X axis (its arrow) to the vehicle's right.
"""

from __future__ import annotations

import math
import time
from collections.abc import Sequence

from mavforge.domain.errors import MAVForgeError
from mavforge.domain.models import (
    CalibrationKind,
    CalibrationProcedure,
    CalibrationResult,
    CalibrationState,
    Tilt,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.protocol.commands import command_long
from mavforge.services.calibration.base import Workflow, WorkflowContext
from mavforge.services.changes import plan_changes
from mavforge.transport.dialect import Message, mavlink

type Vec = tuple[float, float, float]
type Mat = tuple[Vec, Vec, Vec]

POLL_INTERVAL = 0.1
"""SCALED_IMU is requested this often while waiting and sampling."""
FIRST_SAMPLE_TIMEOUT = 3.0
STEP_TIMEOUT = 180.0
SAMPLES = 10
"""Readings averaged per pose (about 1 s)."""
MAX_ATTEMPTS = 3
"""Times a pose is asked for again (not still, not tilted enough) before failing."""
GRAVITY_MG = (800.0, 1200.0)
"""Accepted magnitude of a still reading, in milli-g."""
MAX_SHAKE_MG = 80.0
"""A pose counts as still when no reading is further than this from the mean."""
MIN_TILT_DEG = 20.0
MAX_INCONSISTENCY_DEG = 30.0
"""Nose-up and right-side-down tilts must agree on the right axis within this."""
MAX_MOUNT_ERROR_DEG = 20.0
"""The best standard rotation must be within this of the measured mounting."""
SQUARE_WARNING_DEG = 10.0
"""Warn above this: ArduPilot's level trim absorbs at most 10 degrees of roll and pitch."""
AXIS_MATCH_DEG = 30.0

# ArduPilot's Rotation enum (AP_Math/rotations.h): value -> (roll, pitch, yaw) degrees.
ROTATIONS: dict[int, Vec] = {
    0: (0, 0, 0), 1: (0, 0, 45), 2: (0, 0, 90), 3: (0, 0, 135),
    4: (0, 0, 180), 5: (0, 0, 225), 6: (0, 0, 270), 7: (0, 0, 315),
    8: (180, 0, 0), 9: (180, 0, 45), 10: (180, 0, 90), 11: (180, 0, 135),
    12: (0, 180, 0), 13: (180, 0, 225), 14: (180, 0, 270), 15: (180, 0, 315),
    16: (90, 0, 0), 17: (90, 0, 45), 18: (90, 0, 90), 19: (90, 0, 135),
    20: (270, 0, 0), 21: (270, 0, 45), 22: (270, 0, 90), 23: (270, 0, 135),
    24: (0, 90, 0), 25: (0, 270, 0), 26: (0, 180, 90), 27: (0, 180, 270),
    28: (90, 90, 0), 29: (180, 90, 0), 30: (270, 90, 0), 31: (90, 180, 0),
    32: (270, 180, 0), 33: (90, 270, 0), 34: (180, 270, 0), 35: (270, 270, 0),
    36: (90, 180, 90), 37: (90, 0, 270), 38: (90, 68.8, 293.3), 39: (0, 315, 0),
    40: (90, 315, 0), 41: (0, 7, 0), 42: (45, 0, 0), 43: (315, 0, 0),
}  # fmt: skip

# ArduPilot's labels (AHRS_ORIENTATION values in the Plane 4.6.2 parameter metadata;
# 41 is not listed there). The order differs from the enum names; the rotation is the same.
LABELS: dict[int, str] = {
    0: "None", 1: "Yaw45", 2: "Yaw90", 3: "Yaw135", 4: "Yaw180", 5: "Yaw225", 6: "Yaw270",
    7: "Yaw315", 8: "Roll180", 9: "Yaw45Roll180", 10: "Yaw90Roll180", 11: "Yaw135Roll180",
    12: "Pitch180", 13: "Yaw225Roll180", 14: "Yaw270Roll180", 15: "Yaw315Roll180",
    16: "Roll90", 17: "Yaw45Roll90", 18: "Yaw90Roll90", 19: "Yaw135Roll90", 20: "Roll270",
    21: "Yaw45Roll270", 22: "Yaw90Roll270", 23: "Yaw135Roll270", 24: "Pitch90",
    25: "Pitch270", 26: "Yaw90Pitch180", 27: "Yaw270Pitch180", 28: "Pitch90Roll90",
    29: "Pitch90Roll180", 30: "Pitch90Roll270", 31: "Pitch180Roll90",
    32: "Pitch180Roll270", 33: "Pitch270Roll90", 34: "Pitch270Roll180",
    35: "Pitch270Roll270", 36: "Yaw90Pitch180Roll90", 37: "Yaw270Roll90",
    38: "Yaw293Pitch68Roll180", 39: "Pitch315", 40: "Pitch315Roll90", 41: "Pitch7",
    42: "Roll45", 43: "Roll315",
}  # fmt: skip
PROPOSABLE = frozenset(ROTATIONS) - {38, 41}
"""Rotations the check may propose. 38 and 41 exist for particular boards; a small tilt
like 41's 7 degrees belongs in the level trim, not the board orientation."""

LEVEL = "Rest the vehicle level, in its normal flying attitude, on a steady surface. Then confirm."
NOSE_UP = (
    "Raise the nose about 30 to 45 degrees, keeping the wings (or sides) level; the tail "
    "can stay on the surface. Hold it still, then confirm."
)
RIGHT_DOWN = (
    "Put the vehicle level again, then lower its right side (the right wing, seen from "
    "behind) about 30 to 45 degrees, keeping the nose level. Hold it still, then confirm."
)
SAMPLING = "Hold it still while the accelerometer is read."
POSE_NAMES = {"level": "Level", "nose_up": "Nose up", "right_down": "Right side down"}


def rotation_name(value: int) -> str:
    """ArduPilot's label for a rotation, e.g. 'Yaw90' or 'Yaw270Roll180'."""
    return LABELS[value]


def rotation_matrix(value: int) -> Mat:
    """Board-to-body matrix of an ArduPilot rotation: Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = (math.radians(a) for a in ROTATIONS[value])
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll), math.sin(roll), math.cos(pitch),
        math.sin(pitch), math.cos(yaw), math.sin(yaw),
    )  # fmt: skip
    return (
        (cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy),
        (cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy),
        (-sp, sr * cp, cr * cp),
    )


def _dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec, b: Vec) -> Vec:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _scale(a: Vec, k: float) -> Vec:
    return (a[0] * k, a[1] * k, a[2] * k)


def _add(a: Vec, b: Vec) -> Vec:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _norm(a: Vec) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a: Vec) -> Vec:
    return _scale(a, 1 / _norm(a))


def _angle(a: Vec, b: Vec) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, _dot(_unit(a), _unit(b))))))


def _transpose(m: Mat) -> Mat:
    return ((m[0][0], m[1][0], m[2][0]), (m[0][1], m[1][1], m[2][1]), (m[0][2], m[1][2], m[2][2]))


def _matmul(a: Mat, b: Mat) -> Mat:
    bt = _transpose(b)
    return (_apply(bt, a[0]), _apply(bt, a[1]), _apply(bt, a[2]))


def _apply(m: Mat, v: Vec) -> Vec:
    return (_dot(m[0], v), _dot(m[1], v), _dot(m[2], v))


def rotation_error_deg(a: Mat, b: Mat) -> float:
    """Angle of the rotation taking ``a`` to ``b``."""
    at, bt = _transpose(a), _transpose(b)
    trace = sum(_dot(at[i], bt[i]) for i in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1) / 2))))


def tilt_of(accel: Vec) -> Tilt:
    """Roll (right side down positive) and pitch (nose up positive) from a still reading."""
    x, y, z = accel
    return Tilt(
        roll_deg=round(math.degrees(math.atan2(-y, -z)), 1),
        pitch_deg=round(math.degrees(math.atan2(x, math.hypot(y, z))), 1),
    )


def _direction(v: Vec) -> str:
    """Name the body direction a unit vector points, if it is close to an axis."""
    names = {
        (1, 0, 0): "the nose", (-1, 0, 0): "the tail", (0, 1, 0): "the right",
        (0, -1, 0): "the left", (0, 0, 1): "down", (0, 0, -1): "up",
    }  # fmt: skip
    axis, name = min(names.items(), key=lambda kv: _angle(v, kv[0]))
    return name if _angle(v, axis) <= AXIS_MATCH_DEG else "an angle"


def solve(level: Vec, nose_up: Vec, right_down: Vec, current: int) -> tuple[Mat, float]:
    """The board's true board-to-body rotation, and how inconsistent the poses were.

    Readings are specific force in the current body frame: at rest it points up, so
    level gives -down; nose up adds +forward; right side down adds -right.
    """
    down = _unit(_scale(level, -1))
    forward = _unit(_sub(nose_up, _scale(down, _dot(nose_up, down))))
    right_measured = _unit(_scale(_sub(right_down, _scale(down, _dot(right_down, down))), -1))
    inconsistency = _angle(right_measured, _cross(down, forward))
    if inconsistency <= MAX_INCONSISTENCY_DEG:
        # use both tilts: average forward with the one the right-side tilt implies
        forward = _unit(_add(forward, _cross(right_measured, down)))
    right = _cross(down, forward)
    to_body: Mat = (forward, right, down)  # rows: current-frame vector -> true body frame
    return _matmul(to_body, rotation_matrix(current)), inconsistency


def best_rotation(mounting: Mat, current: int) -> tuple[int, float]:
    """Closest proposable rotation (the current value wins a tie, then the lowest)."""
    candidates = PROPOSABLE | {current}
    errors = {value: rotation_error_deg(rotation_matrix(value), mounting) for value in candidates}
    best = min(errors, key=lambda v: (round(errors[v], 3), v != current, v))
    return best, errors[best]


class OrientationWorkflow(Workflow):
    kind = CalibrationKind.ORIENTATION
    via_mcp = False
    writes_vehicle = False

    def __init__(self, step_timeout: float = STEP_TIMEOUT, *, offline: bool = False) -> None:
        self.step_timeout = step_timeout
        self.offline = offline

    def procedure(self) -> CalibrationProcedure:
        return CalibrationProcedure(
            kind=self.kind,
            title="Board orientation check",
            summary="Works out how the flight controller is mounted from three poses (level, "
            "nose up, right side down) and proposes AHRS_ORIENTATION as a change plan.",
            preconditions=[
                "Vehicle disarmed",
                "Motors unpowered or propellers removed: you will tilt the vehicle by hand",
            ],
            human_steps=[LEVEL, NOSE_UP, RIGHT_DOWN],
            changes_parameters=["AHRS_ORIENTATION (via an approved change plan)"],
            typical_duration="about 1 minute",
            success_evidence=f"Each pose held still and tilted at least {MIN_TILT_DEG:.0f} "
            f"degrees; the two tilts agree within {MAX_INCONSISTENCY_DEG:.0f} degrees and the "
            f"mounting is within {MAX_MOUNT_ERROR_DEG:.0f} degrees of a standard rotation. A "
            "change is written only through an approved plan with read-back.",
            cancellation="Cancellable at any time; nothing is written by the check itself.",
            available_via_mcp=False,
            proposes_plan=True,
            supported=True,
        )

    def run(self, ctx: WorkflowContext) -> CalibrationResult:
        read = ctx.read_params(["AHRS_ORIENTATION"])
        if "AHRS_ORIENTATION" not in read:
            return _failed("The vehicle has no AHRS_ORIENTATION parameter to check.")
        current = int(read["AHRS_ORIENTATION"])
        if current not in ROTATIONS:
            return _failed(
                f"AHRS_ORIENTATION is {current}, a custom rotation; MAVForge checks only the "
                "standard rotations (0-43). Nothing was written."
            )
        sysid, compid = ctx.session.target

        def imu_message(msg: Message) -> bool:
            return msg.get_srcSystem() == sysid and isinstance(
                msg, mavlink.MAVLink_scaled_imu_message
            )

        with ctx.session.connection.subscribe(imu_message) as sub:
            readings: list[Vec] = []

            def sample() -> None:
                """Request SCALED_IMU and read replies for one poll interval."""
                ctx.session.connection.send(
                    command_long(
                        sysid,
                        compid,
                        mavlink.MAV_CMD_REQUEST_MESSAGE,
                        [mavlink.MAVLINK_MSG_ID_SCALED_IMU],
                    )
                )
                end = time.monotonic() + POLL_INTERVAL
                while (remaining := end - time.monotonic()) > 0:
                    msg = sub.get(remaining)
                    if isinstance(msg, mavlink.MAVLink_scaled_imu_message):
                        accel: Vec = (float(msg.xacc), float(msg.yacc), float(msg.zacc))
                        readings.append(accel)
                        del readings[:-SAMPLES]
                        if _norm(accel) > 0:
                            ctx.update(tilt=tilt_of(accel))

            def collect() -> list[Vec] | None:
                readings.clear()
                deadline = time.monotonic() + FIRST_SAMPLE_TIMEOUT
                while len(readings) < SAMPLES:
                    ctx.check_cancelled()
                    if time.monotonic() > deadline:
                        return None
                    sample()
                return list(readings)

            first = collect()
            if first is None:
                return _failed(
                    "The vehicle did not send SCALED_IMU; cannot read the accelerometer."
                )

            poses: dict[str, Vec] = {}
            evidence: list[str] = [f"AHRS_ORIENTATION is {current} ({rotation_name(current)})"]
            steps = (("level", LEVEL), ("nose_up", NOSE_UP), ("right_down", RIGHT_DOWN))
            for step, instruction in steps:
                problem: str | None = None
                for _attempt in range(MAX_ATTEMPTS):
                    text = f"{problem} {instruction}" if problem else instruction
                    if not ctx.wait_for_confirmation(
                        step, text, self.step_timeout, poll=sample, after=SAMPLING
                    ):
                        return CalibrationResult(
                            status=OperationStatus.TIMED_OUT,
                            summary="Orientation check timed out waiting for the "
                            f"{POSE_NAMES[step].lower()} pose. "
                            "Nothing was written.",
                            evidence=evidence,
                        )
                    got = collect()
                    if got is None:
                        return _failed(
                            "SCALED_IMU stopped arriving while reading a pose. Nothing was "
                            "written.",
                            evidence,
                        )
                    mean, problem = _pose(got, poses.get("level"), step)
                    if problem is None:
                        poses[step] = mean
                        tilt = tilt_of(mean)
                        evidence.append(
                            f"{POSE_NAMES[step]}: accel {mean[0]:.0f}, {mean[1]:.0f}, "
                            f"{mean[2]:.0f} mG (seen as roll {round(tilt.roll_deg):+d}, "
                            f"pitch {round(tilt.pitch_deg):+d})"
                        )
                        break
                else:
                    return _failed(
                        f"The {POSE_NAMES[step].lower()} pose could not be read: {problem} "
                        "Nothing was written.",
                        evidence,
                    )
        ctx.update(state=CalibrationState.VERIFYING, instruction=None)
        return self._result(ctx, poses, current, evidence)

    def _result(
        self, ctx: WorkflowContext, poses: dict[str, Vec], current: int, evidence: list[str]
    ) -> CalibrationResult:
        mounting, inconsistency = solve(
            poses["level"], poses["nose_up"], poses["right_down"], current
        )
        if inconsistency > MAX_INCONSISTENCY_DEG:
            return _failed(
                f"The nose-up and right-side-down tilts disagree by {inconsistency:.0f} degrees. "
                "Check you raised the nose (not the tail) and lowered the right side (not the "
                "left), then run the check again. Nothing was written.",
                evidence,
            )
        best, error = best_rotation(mounting, current)
        arrow = _direction(_apply(mounting, (1, 0, 0)))
        top = _direction(_apply(mounting, (0, 0, -1)))
        evidence.append(
            f"The board's X axis (its arrow) points toward {arrow}; its top faces {top}"
        )
        evidence.append(
            f"Closest standard rotation: {best} ({rotation_name(best)}), {error:.0f} degrees "
            f"from the measured mounting; tilts agreed within {inconsistency:.0f} degrees"
        )
        if error > MAX_MOUNT_ERROR_DEG:
            return _failed(
                f"The board is mounted {error:.0f} degrees from the nearest standard rotation "
                f"({best}, {rotation_name(best)}). Mount it square to the vehicle, or set a "
                "custom rotation (not supported by MAVForge). Nothing was written.",
                evidence,
            )
        change = rotation_error_deg(rotation_matrix(best), rotation_matrix(current)) > 1
        warnings: list[str] = []
        if change:
            warnings.append(
                "After applying: reboot the autopilot, then repeat the level calibration (its "
                "trim was measured on the old axes) and the compass calibration."
            )
        if error > SQUARE_WARNING_DEG:
            warnings.append(
                f"The board is {error:.0f} degrees off the standard rotation. Check it is "
                "square to the airframe and that the level pose was really level; the level "
                f"calibration can absorb at most {SQUARE_WARNING_DEG:.0f} degrees."
            )
        requested = best if change else current
        try:
            plan = plan_changes(
                ctx.session,
                {"AHRS_ORIENTATION": requested},
                offline=self.offline,
                extra_warnings=[f"From orientation check {ctx.operation_id}.", *warnings],
            )
        except MAVForgeError as exc:
            return _failed(f"Could not build the change plan: {exc.message}", evidence)
        evidence.append(f"Change plan {plan.plan_id} recorded")
        if plan.blocked:
            return CalibrationResult(
                status=OperationStatus.FAILED,
                summary=f"AHRS_ORIENTATION {best} was rejected (plan {plan.plan_id} is "
                "blocked); nothing was written.",
                evidence=evidence,
                warnings=warnings,
                plan=plan,
            )
        summary = (
            f"The board is mounted as AHRS_ORIENTATION {best} ({rotation_name(best)}), not "
            f"{current} ({rotation_name(current)}). Nothing has been written: plan "
            f"{plan.plan_id} proposes the change and needs approval."
            if change
            else f"AHRS_ORIENTATION {current} ({rotation_name(current)}) matches how the "
            "board is mounted. No change needed."
        )
        return CalibrationResult(
            status=OperationStatus.SUCCEEDED,
            summary=summary,
            evidence=evidence,
            warnings=warnings,
            plan=plan,
        )


def _pose(readings: Sequence[Vec], level: Vec | None, step: str) -> tuple[Vec, str | None]:
    """Mean of a pose's readings, or why the pose is not usable."""
    n = len(readings)
    mean: Vec = (
        sum(r[0] for r in readings) / n,
        sum(r[1] for r in readings) / n,
        sum(r[2] for r in readings) / n,
    )
    if max(_norm(_sub(r, mean)) for r in readings) > MAX_SHAKE_MG:
        return mean, "The vehicle was moving."
    if not GRAVITY_MG[0] <= _norm(mean) <= GRAVITY_MG[1]:
        return mean, f"The accelerometer read {_norm(mean):.0f} mG, not about 1 g."
    if level is not None and (tilt := _angle(mean, level)) < MIN_TILT_DEG:
        return mean, (f"The vehicle is tilted only {tilt:.0f} degrees from level; tilt it further.")
    return mean, None


def _failed(summary: str, evidence: list[str] | None = None) -> CalibrationResult:
    return CalibrationResult(
        status=OperationStatus.FAILED, summary=summary, evidence=evidence or []
    )
