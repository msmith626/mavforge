"""Flight-mode assignment: a guided check of the FLTMODE_CH switch, then a change plan
for FLTMODE1-6.

ArduPilot reads the mode channel through RC_Channel::read_6pos_switch (confirmed
directly against ArduPilot source, tag Plane-4.6.2 -- see docs/research.md, "Flight
modes and failsafes"): six fixed PWM bands, not configurable, using the channel's raw
PWM (RCn_MIN/MAX on the mode channel don't move them). MAVForge watches the switch
while the person flips it through every position they use, to confirm each one
actually lands in a distinct band before proposing anything. Like radio calibration,
nothing is written itself: the result is a change plan for whichever FLTMODEn values
were requested.

Deliberately out of scope, the same reasoning as radio calibration and the orientation
check: which mode belongs on which switch position is the pilot's judgment call, not
something to infer from telemetry. The person supplies the target values (NAME=VALUE,
same syntax as `params set`); MAVForge's job here is only to verify the switch can
actually produce the positions being assigned.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from mavforge.domain.errors import MAVForgeError
from mavforge.domain.models import (
    CalibrationKind,
    CalibrationProcedure,
    CalibrationResult,
    CalibrationState,
    FlightModeBand,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.protocol.commands import command_long
from mavforge.services.calibration.base import Workflow, WorkflowContext
from mavforge.services.changes import plan_changes
from mavforge.transport.dialect import Message, mavlink

POLL_INTERVAL = 0.1
"""RC_CHANNELS is requested this often (10 Hz, matching the radio calibration)."""
FIRST_SAMPLE_TIMEOUT = 3.0
SWEEP_TIMEOUT = 300.0
VALID_PWM = (800, 2200)

# RC_Channel::read_6pos_switch (libraries/RC_Channel/RC_Channel.cpp, tag Plane-4.6.2):
# exact breakpoints. Boundaries are fixed and use raw receiver PWM.
_BREAKPOINTS = (1231, 1361, 1491, 1621, 1750)

PARAMETERS = tuple(f"FLTMODE{i}" for i in range(1, 7))

SWEEP = (
    "Flip the switch on FLTMODE_CH through every position you use, pausing on each for "
    "a second or two. Confirm once you've covered every position."
)


def band_of(pwm: int) -> int:
    """0-5, matching RC_Channel::read_6pos_switch exactly."""
    for i, edge in enumerate(_BREAKPOINTS):
        if pwm < edge:
            return i
    return 5


@dataclass
class _Band:
    band: int
    current: int | None = None
    pwm_seen: int | None = None
    visited: bool = False

    def view(self) -> FlightModeBand:
        return FlightModeBand(
            band=self.band,
            parameter=f"FLTMODE{self.band + 1}",
            current=self.current,
            pwm_seen=self.pwm_seen,
            visited=self.visited,
        )


class FlightModeWorkflow(Workflow):
    kind = CalibrationKind.FLIGHT_MODES
    via_mcp = False
    writes_vehicle = False

    def __init__(
        self,
        requested: dict[str, float] | None = None,
        sweep_timeout: float = SWEEP_TIMEOUT,
        *,
        offline: bool = False,
    ) -> None:
        self.requested = dict(requested or {})
        self.sweep_timeout = sweep_timeout
        self.offline = offline

    def procedure(self) -> CalibrationProcedure:
        return CalibrationProcedure(
            kind=self.kind,
            title="Flight mode assignment",
            summary="Checks that the FLTMODE_CH switch can actually produce every "
            "position you're assigning, then proposes FLTMODE1-FLTMODE6 as a change plan.",
            preconditions=[
                "Vehicle disarmed",
                "Transmitter on and the receiver bound, with RC input reaching the autopilot",
                "FLTMODE_CH set to the channel your mode switch is on",
            ],
            human_steps=[SWEEP],
            changes_parameters=["FLTMODE1..FLTMODE6 (via an approved change plan)"],
            typical_duration="about 30 seconds",
            success_evidence="Every FLTMODEn being set was confirmed reachable: the switch "
            "produced a PWM inside that band during the sweep. Values are then written "
            "only through an approved plan with read-back.",
            cancellation="Cancellable at any time; nothing is written by the check itself.",
            available_via_mcp=False,
            proposes_plan=True,
            supported=True,
        )

    def run(self, ctx: WorkflowContext) -> CalibrationResult:
        if not self.requested:
            return _failed("No FLTMODEn values were requested; nothing to check or propose.")
        unknown = [n for n in self.requested if n not in PARAMETERS]
        if unknown:
            return _failed(f"Not a flight-mode parameter: {', '.join(unknown)}.")

        read = ctx.read_params(["FLTMODE_CH", *PARAMETERS])
        if "FLTMODE_CH" not in read:
            return _failed("The vehicle has no FLTMODE_CH parameter.")
        channel = int(read["FLTMODE_CH"])
        if channel == 0:
            return _failed(
                "FLTMODE_CH is 0 (disabled): no RC channel is read for flight mode. Set "
                "FLTMODE_CH to your mode switch's channel first, then run this again."
            )

        bands = {
            i: _Band(i, current=int(v) if (v := read.get(f"FLTMODE{i + 1}")) is not None else None)
            for i in range(6)
        }
        sysid, compid = ctx.session.target

        def rc_message(msg: Message) -> bool:
            return msg.get_srcSystem() == sysid and isinstance(
                msg, mavlink.MAVLink_rc_channels_message
            )

        with ctx.session.connection.subscribe(rc_message) as sub:

            def sample() -> int:
                """Request RC_CHANNELS and read replies for one poll interval. Returns how many."""
                ctx.session.connection.send(
                    command_long(
                        sysid,
                        compid,
                        mavlink.MAV_CMD_REQUEST_MESSAGE,
                        [mavlink.MAVLINK_MSG_ID_RC_CHANNELS],
                    )
                )
                count = 0
                end = time.monotonic() + POLL_INTERVAL
                while (remaining := end - time.monotonic()) > 0:
                    msg = sub.get(remaining)
                    if not isinstance(msg, mavlink.MAVLink_rc_channels_message):
                        continue
                    count += 1
                    pwm = int(getattr(msg, f"chan{channel}_raw"))
                    if VALID_PWM[0] <= pwm <= VALID_PWM[1]:
                        b = bands[band_of(pwm)]
                        b.pwm_seen = pwm
                        b.visited = True
                return count

            def publish() -> None:
                ctx.update(mode_bands=[bands[i].view() for i in range(6)])

            def sweep_poll() -> None:
                sample()
                publish()

            def collect_first() -> bool:
                deadline = time.monotonic() + FIRST_SAMPLE_TIMEOUT
                got = 0
                while got == 0:
                    ctx.check_cancelled()
                    if time.monotonic() > deadline:
                        return False
                    got = sample()
                return True

            if not collect_first():
                return _failed("The vehicle did not send RC_CHANNELS; cannot read RC input.")
            publish()
            if not ctx.wait_for_confirmation(
                "sweep_done", SWEEP, self.sweep_timeout, poll=sweep_poll, after=None
            ):
                return CalibrationResult(
                    status=OperationStatus.TIMED_OUT,
                    summary="Flight-mode check timed out waiting for the sweep. Nothing was "
                    "written.",
                )
        ctx.update(state=CalibrationState.VERIFYING, instruction=None)
        return self._result(ctx, bands)

    def _result(self, ctx: WorkflowContext, bands: dict[int, _Band]) -> CalibrationResult:
        evidence = [
            f"FLTMODE{b.band + 1}: "
            + (f"band reached (PWM {b.pwm_seen})" if b.visited else "band NOT reached")
            + f"; currently {b.current}"
            for b in (bands[i] for i in range(6))
        ]
        unreached = [name for name in self.requested if not bands[int(name[7:]) - 1].visited]
        warnings = (
            [
                "Not reached during the sweep, proposed anyway: "
                + ", ".join(sorted(unreached))
                + ". Check the switch (or any mixing) actually produces this position before "
                "relying on it."
            ]
            if unreached
            else []
        )
        try:
            plan = plan_changes(
                ctx.session,
                self.requested,
                offline=self.offline,
                extra_warnings=[f"From flight-mode check {ctx.operation_id}.", *warnings],
            )
        except MAVForgeError as exc:
            return _failed(f"Could not build the change plan: {exc.message}", evidence)
        evidence.append(f"Change plan {plan.plan_id} recorded")
        if plan.blocked:
            return CalibrationResult(
                status=OperationStatus.FAILED,
                summary=f"The proposed values were rejected (plan {plan.plan_id} is "
                "blocked); nothing was written.",
                evidence=evidence,
                warnings=warnings,
                plan=plan,
            )
        pending = sum(not c.no_change for c in plan.changes)
        summary = (
            f"Plan {plan.plan_id} proposes {pending} change(s) and needs approval. Nothing "
            "has been written."
            if pending
            else "The vehicle already has these values."
        )
        return CalibrationResult(
            status=OperationStatus.SUCCEEDED,
            summary=summary,
            evidence=evidence,
            warnings=warnings,
            plan=plan,
        )


def _failed(summary: str, evidence: list[str] | None = None) -> CalibrationResult:
    return CalibrationResult(
        status=OperationStatus.FAILED, summary=summary, evidence=evidence or []
    )
