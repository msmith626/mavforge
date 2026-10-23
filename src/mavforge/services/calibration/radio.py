"""Radio (RC) calibration: a guided stick sweep that proposes RCn_MIN / RCn_MAX / RCn_TRIM.

The firmware takes no part. Like Mission Planner's ConfigRadioInput.cs, MAVForge watches
RC_CHANNELS while the person moves every stick and switch to its limits, then records
each channel with the sticks centred and the throttle down. MAVForge writes nothing
itself: the result is a parameter change plan that goes through the normal approve ->
apply -> read-back path.

Deliberate differences from Mission Planner (see docs/research.md):

- Each sample is range-checked per channel. Mission Planner gates all channels on
  channel 1 being inside 800..2200.
- A channel must travel at least MIN_TRAVEL us to count as moved. Mission Planner writes
  any channel with min != max, so an unconnected channel jittering 1500 +/- 2 gets a
  4 us range, and its noise then reads as full deflection.
- Channels that did not move are reported, not silently skipped.
- Roll, pitch and yaw must be centred, and the throttle low, when trims are captured.
  Mission Planner captures whatever the sticks show.
- A value within TOLERANCE us of the current one is left as it is.
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
    RcChannelRange,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.protocol.commands import command_long
from mavforge.services.calibration.base import Workflow, WorkflowContext
from mavforge.services.changes import plan_changes
from mavforge.transport.dialect import Message, mavlink

POLL_INTERVAL = 0.1
"""RC_CHANNELS is requested this often (10 Hz, Mission Planner's RC stream rate)."""
FIRST_SAMPLE_TIMEOUT = 3.0
SWEEP_TIMEOUT = 300.0
CENTRE_TIMEOUT = 120.0
CENTRE_SAMPLES = 5
VALID_PWM = (800, 2200)
"""Documented range of RCn_MIN / RCn_MAX / RCn_TRIM; samples outside it are ignored."""
MIN_TRAVEL = 200
"""Microseconds a channel must travel to count as moved (MAVForge's choice: a stick or
switch spans about 800 us, an unconnected channel jitters a few us)."""
CENTRE_BAND = 0.2
"""Roll/pitch/yaw trim must lie within this fraction of the travel from the midpoint."""
LOW_BAND = 0.1
"""Throttle trim must lie within this fraction of the travel from its low end."""
TOLERANCE = 3
"""A measured value within this many us of the current one keeps the current value, so a
radio that is already calibrated gets no changes for sampling noise (MAVForge's choice;
Mission Planner writes every value)."""
ROLES = ("ROLL", "PITCH", "THROTTLE", "YAW")
NO_CHANNEL = 65535  # RC_CHANNELS: UINT16_MAX marks an unused channel

SWEEP = (
    "Move every stick, switch and knob on the transmitter slowly to both ends of its "
    "travel, several times. Confirm when every channel you use shows as moved."
)
CENTRE = (
    "Centre the roll, pitch and yaw sticks, put the throttle all the way down, and leave "
    "the switches where you want them. Then confirm."
)


@dataclass
class _Channel:
    channel: int
    role: str | None = None
    current: int | None = None
    low: int | None = None
    high: int | None = None

    @property
    def span(self) -> tuple[int, int] | None:
        return None if self.low is None or self.high is None else (self.low, self.high)

    @property
    def travel(self) -> int:
        return 0 if self.span is None else self.span[1] - self.span[0]

    @property
    def moved(self) -> bool:
        return self.travel >= MIN_TRAVEL

    def view(self) -> RcChannelRange:
        return RcChannelRange(
            channel=self.channel,
            role=self.role,
            current=self.current,
            min=self.low,
            max=self.high,
            moved=self.moved,
        )


def _values(msg: mavlink.MAVLink_rc_channels_message) -> list[int | None]:
    """PWM per channel (index 0 = channel 1); None where the value is missing or invalid."""
    out: list[int | None] = []
    for i in range(1, min(msg.chancount, 18) + 1):
        value = int(getattr(msg, f"chan{i}_raw"))
        out.append(value if VALID_PWM[0] <= value <= VALID_PWM[1] else None)
    return out


class RadioWorkflow(Workflow):
    kind = CalibrationKind.RADIO
    via_mcp = False
    writes_vehicle = False

    def __init__(
        self,
        sweep_timeout: float = SWEEP_TIMEOUT,
        centre_timeout: float = CENTRE_TIMEOUT,
        *,
        offline: bool = False,
    ) -> None:
        self.sweep_timeout = sweep_timeout
        self.centre_timeout = centre_timeout
        self.offline = offline

    def procedure(self) -> CalibrationProcedure:
        return CalibrationProcedure(
            kind=self.kind,
            title="Radio (RC) calibration",
            summary="Records each RC channel's range and centre while you move the "
            "transmitter's sticks and switches, and proposes them as a change plan.",
            preconditions=[
                "Vehicle disarmed",
                "Motors unpowered or propellers removed: control surfaces and outputs may "
                "move with the sticks",
                "Transmitter on and the receiver bound, with RC input reaching the autopilot",
                "Transmitter trims centred and no mixes you don't fly with",
            ],
            human_steps=[SWEEP, CENTRE],
            changes_parameters=["RCn_MIN", "RCn_MAX", "RCn_TRIM (via an approved change plan)"],
            typical_duration="about 1 minute",
            success_evidence="Every primary channel (RCMAP roll/pitch/throttle/yaw) moved at "
            f"least {MIN_TRAVEL} us, sticks were centred and the throttle low at the end; "
            "the values are then written only through an approved plan with read-back.",
            cancellation="Cancellable at any time; nothing is written by the sweep itself.",
            available_via_mcp=False,
            proposes_plan=True,
            supported=True,
        )

    def run(self, ctx: WorkflowContext) -> CalibrationResult:
        maps = ctx.read_params([f"RCMAP_{r}" for r in ROLES])
        # ArduPilot's defaults are channels 1-4
        roles = {int(maps.get(f"RCMAP_{r}", i + 1)): r.lower() for i, r in enumerate(ROLES)}
        throttle = int(maps.get("RCMAP_THROTTLE", 3))
        reversed_name = f"RC{throttle}_REVERSED"
        reversed_throttle = bool(ctx.read_params([reversed_name]).get(reversed_name, 0))
        channels: dict[int, _Channel] = {}
        sysid, compid = ctx.session.target
        latest: list[list[int | None]] = []

        def rc_message(msg: Message) -> bool:
            return msg.get_srcSystem() == sysid and isinstance(
                msg, mavlink.MAVLink_rc_channels_message
            )

        with ctx.session.connection.subscribe(rc_message) as sub:

            def sample(track: bool = True) -> int:
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
                    values = _values(msg)
                    latest.append(values)
                    del latest[:-CENTRE_SAMPLES]
                    for i, value in enumerate(values):
                        ch = channels.setdefault(i + 1, _Channel(i + 1, roles.get(i + 1)))
                        ch.current = value
                        if track and value is not None:
                            ch.low = value if ch.low is None else min(ch.low, value)
                            ch.high = value if ch.high is None else max(ch.high, value)
                return count

            def publish() -> None:
                ctx.update(rc_channels=[channels[c].view() for c in sorted(channels)])

            def sweep_poll() -> None:
                sample()
                publish()

            def centre_poll() -> None:
                sample(track=False)

            def collect(count: int) -> bool:
                """Sample until ``count`` readings arrive; False if RC_CHANNELS stops."""
                latest.clear()
                deadline = time.monotonic() + FIRST_SAMPLE_TIMEOUT
                while len(latest) < count:
                    ctx.check_cancelled()
                    if time.monotonic() > deadline:
                        return False
                    sample(track=False)
                return True

            if not collect(1):
                return CalibrationResult(
                    status=OperationStatus.FAILED,
                    summary="The vehicle did not send RC_CHANNELS; cannot read RC input.",
                )
            if not any(v is not None for v in latest[-1]):
                return CalibrationResult(
                    status=OperationStatus.FAILED,
                    summary="The autopilot reports no RC input. Turn the transmitter on, "
                    "check the receiver is bound and connected, and try again.",
                )
            publish()
            if not ctx.wait_for_confirmation(
                "sweep_done", SWEEP, self.sweep_timeout, poll=sweep_poll, after=None
            ):
                return _timed_out("sweep")
            if not ctx.wait_for_confirmation(
                "centred",
                CENTRE,
                self.centre_timeout,
                poll=centre_poll,
                after="Reading the centre positions; keep the sticks still.",
            ):
                return _timed_out("centring")
            if not collect(CENTRE_SAMPLES):
                return CalibrationResult(
                    status=OperationStatus.FAILED,
                    summary="RC_CHANNELS stopped arriving while reading the centre "
                    "positions. Nothing was written.",
                )
        ctx.update(state=CalibrationState.VERIFYING, instruction=None)
        return self._result(ctx, channels, latest, roles, throttle, reversed_throttle)

    def _result(
        self,
        ctx: WorkflowContext,
        channels: dict[int, _Channel],
        centre_samples: list[list[int | None]],
        roles: dict[int, str],
        throttle: int,
        reversed_throttle: bool,
    ) -> CalibrationResult:
        trims: dict[int, int] = {}
        for ch in channels.values():
            readings = sorted(
                v
                for s in centre_samples
                if len(s) >= ch.channel and (v := s[ch.channel - 1]) is not None
            )
            if readings and ch.moved and (span := ch.span):
                # median of the samples, clamped to the sweep (as Mission Planner does)
                trims[ch.channel] = min(max(readings[len(readings) // 2], span[0]), span[1])
        evidence = [
            f"RC{c.channel}{f' ({c.role})' if c.role else ''}: "
            + (
                f"{c.low}..{c.high}, centre {trims.get(c.channel, '?')}"
                if c.moved
                else f"not moved (travel {c.travel} us)"
            )
            for c in sorted(channels.values(), key=lambda c: c.channel)
        ]
        if not any(c.moved for c in channels.values()):
            return CalibrationResult(
                status=OperationStatus.FAILED,
                summary="No RC channel moved. Is the transmitter on and bound? A receiver "
                "without a transmitter signal can keep sending fixed failsafe values. "
                "Nothing was written.",
                evidence=evidence,
            )
        problems: list[str] = []
        for number, role in sorted(roles.items()):
            ch = channels.get(number)
            span = ch.span if ch else None
            if ch is None or span is None or not ch.moved:
                problems.append(f"RC{number} ({role}) did not move; sweep it to both ends")
                continue
            low, high = span
            trim = trims.get(number)
            if trim is None:
                problems.append(f"RC{number} ({role}) had no reading when centred")
                continue
            low_end = high if reversed_throttle else low
            if role == "throttle":
                if abs(trim - low_end) > LOW_BAND * ch.travel:
                    problems.append(
                        f"RC{number} (throttle) read {trim} when it should be all the way "
                        "down; lower the throttle"
                    )
            elif abs(trim - (low + high) / 2) > CENTRE_BAND * ch.travel:
                problems.append(
                    f"RC{number} ({role}) read {trim}, not centred in {low}..{high}; "
                    "centre the stick (and the transmitter trim)"
                )
        if problems:
            return CalibrationResult(
                status=OperationStatus.FAILED,
                summary=f"Radio calibration not usable: {'; '.join(problems)}. "
                "Nothing was written; run it again.",
                evidence=evidence,
            )

        requested: dict[str, float | int] = {}
        for number in sorted(trims):
            if span := channels[number].span:
                requested |= {
                    f"RC{number}_MIN": span[0],
                    f"RC{number}_MAX": span[1],
                    f"RC{number}_TRIM": trims[number],
                }
        current = ctx.read_params(list(requested))
        kept: list[str] = []
        for number in sorted(trims):
            names = [f"RC{number}_{s}" for s in ("MIN", "MAX", "TRIM")]
            measured = [requested[n] for n in names]
            for name in names:
                if name in current and abs(current[name] - requested[name]) <= TOLERANCE:
                    requested[name] = int(current[name])
            low, high, trim = (requested[n] for n in names)
            if not low <= trim <= high:  # keeping the old trim must not put it outside the range
                requested[names[2]] = measured[2]
            kept += [
                f"{n} {requested[n]} (measured {m})"
                for n, m in zip(names, measured, strict=True)
                if requested[n] != m
            ]
        if kept:
            evidence.append(
                f"Within {TOLERANCE} us of the current value, kept as is: {', '.join(kept)}"
            )
        unmoved = [c.channel for c in channels.values() if not c.moved]
        warnings = (
            [
                "Not moved, left unchanged: "
                + ", ".join(f"RC{n}" for n in sorted(unmoved))
                + ". Run the calibration again if you use any of them."
            ]
            if unmoved
            else []
        )
        try:
            plan = plan_changes(
                ctx.session,
                requested,
                offline=self.offline,
                extra_warnings=[f"From radio calibration {ctx.operation_id}.", *warnings],
            )
        except MAVForgeError as exc:
            return CalibrationResult(
                status=OperationStatus.FAILED,
                summary=f"Could not build the change plan: {exc.message}",
                evidence=evidence,
            )
        evidence.append(f"Change plan {plan.plan_id} recorded")
        pending = sum(not c.no_change for c in plan.changes)
        if plan.blocked:
            return CalibrationResult(
                status=OperationStatus.FAILED,
                summary=f"The proposed values were rejected (plan {plan.plan_id} is blocked); "
                "nothing was written.",
                evidence=evidence,
                warnings=warnings,
                plan=plan,
            )
        summary = (
            f"Captured {len(trims)} channel(s). Nothing has been written: plan "
            f"{plan.plan_id} proposes {pending} change(s) and needs approval."
            if pending
            else f"Captured {len(trims)} channel(s); the vehicle already has these values."
        )
        return CalibrationResult(
            status=OperationStatus.SUCCEEDED,
            summary=summary,
            evidence=evidence,
            warnings=warnings,
            plan=plan,
        )


def _timed_out(step: str) -> CalibrationResult:
    return CalibrationResult(
        status=OperationStatus.TIMED_OUT,
        summary=f"Radio calibration timed out waiting for the {step} step. Nothing was written.",
    )
