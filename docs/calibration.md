# Calibration workflows

MAVForge runs each calibration as an explicit state machine
(`services/calibration/`):

- It follows ArduPilot's documented command sequence.
- It tells the person at the vehicle exactly what to do (`instruction`).
- It waits for them to confirm a physical step when the protocol needs that
  (`awaiting_confirmation`).
- It decides success only from the vehicle's terminal signal plus a parameter
  read-back. A command being sent or acknowledged is never enough.

Sources are listed in [research.md](research.md) ("Calibration" and "Calibration
behaviour verified").

## Common rules (all interfaces)

**Request and approval.** A calibration starts from an approved **calibration
request**. This uses the same digest-bound, 10-minute, single-use approvals as
parameter plans. The approval can come from:

- the CLI prompt or `--yes`;
- `mavforge approve <id>`;
- MCP elicitation;
- a Python caller acting for a person.

**Checks before starting.** The service checks, in this order:

1. The `calibrate` permission.
2. The real-vehicle opt-in.
3. The platform supports this calibration.
4. The request is approved.
5. The vehicle matches the one the request was made for.
6. No other calibration is running.
7. The vehicle is disarmed.

It then writes an audit record. If the record can't be written, the calibration
doesn't start (fail closed).

**How it runs.** Each calibration runs in a background operation. `status`,
`confirm` and `cancel` are available through the API, the CLI (live display) and
MCP (`calibration_status`, `calibration_cancel`).

**Outcome.** Every outcome is one of:
- `succeeded`: verified.
- `failed`: the vehicle said no.
- `unknown`: no conclusive evidence, for example the link was lost.
- `cancelled`.

The detailed result has evidence and before/after parameter values.

**Not supported.** MAVForge never reboots, arms or moves the vehicle. When ArduPilot
needs a reboot, the result says so.

## Support

| Workflow | ArduPilot | Fake vehicle | ArduCopter 4.7.2 SITL | Hardware | Via MCP |
|---|---|---|---|---|---|
| Gyro | yes | success + failure | **passed** | **passed** on ArduPlane 4.6.2 (one IMU; offsets moved by under 0.0004 rad/s) | yes |
| Level (AHRS trim) | yes | success + rejected | **passed** | **passed** on ArduPlane 4.6.2 (trim -0.12° roll, +0.23° pitch; board orientation confirmed by a tilt test) | yes |
| Compass (onboard) | yes, if `COMPASS_CALIBRATION` capability | success (1 and 3 compasses), per-compass failure, accept-without-save, cancel, timeout | **passed** (3 compasses, `calibration` model) | **passed**: ArduPlane 4.6.2, 1 compass, USB power only (2026-10-23) | yes |
| Accelerometer, 6 positions | yes | success, failure, silence, cancel | **passed** (`calibration` model) | not yet | **no** |
| Radio (RC) | yes (GCS-side sweep, proposes a change plan) | success, apply after approval, unmoved/uncentred/throttle-high, reversed throttle, RCMAP, jitter, no receiver, receiver in failsafe, cancel, armed | **passed** (`RC_CHANNELS_OVERRIDE` from the test) | **passed** on ArduPlane 4.6.2 (Spektrum DX7s, 7 channels; result matched the existing calibration to within 2 us) | **no** |
| Board orientation check | yes (three poses, proposes `AHRS_ORIENTATION` as a change plan) | correct, arrow right, upside down, wrong current setting, live tilt, too little tilt, shaking, wrong side, not square, custom rotation, no IMU data, cancel, permissions | **passed** (`SIM_IMU_ORIENT=2`: proposed 6, applied, re-check matched) | **passed** on ArduPlane 4.6.2, hand-held (0 matches, as confirmed by hand; heading within 5°, first pose 9° off level) | **no** |
| Flight mode assignment | yes (sweeps `FLTMODE_CH`, proposes `FLTMODE1..6` as a change plan) | plan proposed + applied, band reached/not reached, disabled mode channel, unknown parameter, no request, cancel, permissions | **passed** (drives the real mode channel via `RC_CHANNELS_OVERRIDE`, same mechanism as radio's SITL test) | not yet | **no** |

Other autopilots use the generic adapter, which declares no calibrations.

## Gyro

- **Needs:** the vehicle disarmed and completely still.
- **Sequence:** `MAV_CMD_PREFLIGHT_CALIBRATION` with p1=1.
  - ArduPilot calibrates synchronously and ACKs only when it's done; it allows up to
    30 s.
  - MAVForge sends the command once and waits 35 s. It never retransmits.
- **Success:** ACK `ACCEPTED` (the firmware checks every gyro calibrated), then
  `INS_GYR*OFFS_*` is read back.
- **Failure:**
  - ACK `FAILED`: the vehicle moved or is armed.
  - No ACK: reported as `unknown`.
- **Cancel:** not possible. It finishes in seconds.

## Level (AHRS trim)

- **Needs:** the vehicle disarmed, in its level flying attitude, and still.
- **Sequence:** `PREFLIGHT_CALIBRATION` with p5=2.
  - ArduPilot runs a gyro calibration first, samples for about 0.5 s, and saves
    `AHRS_TRIM_X/Y`.
  - It returns `TEMPORARILY_REJECTED` within 5 s of another accelerometer calibration.
- **Success:** ACK `ACCEPTED`, then `AHRS_TRIM_X/Y` is read back.
- **Limitation on QuadPlanes:** ArduPlane refuses with `Q_TRIM_PITCH` set; see the
  vehicle message.
- **Cancel:** not possible.

## Compass (onboard)

**Needs:**
- The vehicle disarmed.
- At least one `COMPASS_DEV_ID*` non-zero.
- The vehicle advertises `COMPASS_CALIBRATION`.
- No magnetic interference nearby (metal, computers, phones, speakers).
- The flight battery is not required; USB power is fine. ArduPilot's compass
  calibration guide doesn't require it. Motor-current interference is a separate,
  powered procedure (`compassmot`) that MAVForge doesn't implement.

**Sequence:**
1. `MAV_CMD_DO_START_MAG_CAL` with mask=0 (all compasses), retry=0, autosave=0,
   delay=0, autoreboot=0.
2. The person rotates the vehicle through every orientation.
3. MAVForge requests `MAG_CAL_PROGRESS` (191) and `MAG_CAL_REPORT` (192) every 0.5 s.
   - These travel in a telemetry stream MAVForge doesn't change, so they are not sent
     otherwise.
   - It collects every reply from a subscription. One report request makes ArduPilot
     send a report for every finished compass.
4. Once every compass in `cal_mask` has reported:
   - If any report isn't `SUCCESS`, MAVForge sends `DO_CANCEL_MAG_CAL` and returns
     `failed` with the status and a hint for each compass. With autosave off nothing
     was saved, which was verified on SITL.
   - If all succeeded, it sends `DO_ACCEPT_MAG_CAL`, then reads `COMPASS_OFS*` back.

**Success:** every compass reports `MAG_CAL_SUCCESS`, the accept is ACKed, **and** the
read-back offsets match the reported offsets within 0.5 mG.

**After success:** ArduPilot needs a reboot before it uses the new calibration
(pre-arm: "Compass calibrated requires reboot"). MAVForge reports this and does not
reboot.

**Cancel:** `DO_CANCEL_MAG_CAL`. A 300 s timeout also cancels.

## Accelerometer (6 positions)

**Needs:** the vehicle disarmed, and a person who can place it in six orientations.

**Sequence:**
1. `PREFLIGHT_CALIBRATION` with p5=1. ArduPilot runs a gyro calibration first, then
   ACKs.
2. The vehicle sends `COMMAND_LONG MAV_CMD_ACCELCAL_VEHICLE_POS` with p1 = the pose
   it wants (1–6), once a second.
3. MAVForge shows the pose and waits for the person to confirm it. Only then does it
   send `ACCELCAL_VEHICLE_POS` with the same pose, which makes the vehicle sample.
4. This repeats for all six poses. The vehicle then ends with
   `ACCELCAL_VEHICLE_POS_SUCCESS` (16777215) or `_FAILED` (16777216), plus
   STATUSTEXT.

**Success:** `POS_SUCCESS`, then `INS_ACCOFFS_*` read back non-zero.

**Cancel:** ArduPilot has **no MAVLink cancel** (it only cancels on arming). A
cancelled run leaves the autopilot waiting in calibration mode until it is rebooted.
MAVForge says so in the result and does not claim the vehicle was stopped.

**Not over MCP:** each pose must be confirmed by the person holding the vehicle, and
an agent can't truthfully confirm that. The service refuses MCP-initiated accel runs,
even for requests created elsewhere.

**Silence:** if the vehicle stops requesting poses for 30 s, the result is `unknown`.

## Radio (RC)

Radio calibration is different from the others. The autopilot takes no part, and
**MAVForge writes nothing during it.** It watches the RC input, then proposes
`RCn_MIN`, `RCn_MAX` and `RCn_TRIM` as an ordinary [change plan](../README.md). That
plan goes through the normal approve → apply → read-back path.

**Needs:**
- The vehicle disarmed.
- Motors unpowered or propellers removed. Control surfaces and outputs may move with
  the sticks.
- The transmitter on and the receiver bound, with RC input reaching the autopilot.
- Transmitter trims centred, and no mixes you don't fly with.

**Sequence:**
1. MAVForge requests `RC_CHANNELS` (65) with `MAV_CMD_REQUEST_MESSAGE` every 0.1 s. It
   doesn't change stream rates. Values outside 800..2200 (the documented range of the
   `RCn_*` parameters, and Mission Planner's sample gate) are ignored per channel.
2. **Sweep.** The person moves every stick, switch and knob to both ends. The live
   status lists each channel's min/max and whether it has moved. A channel counts as
   moved after 200 µs of travel. That threshold is MAVForge's choice: a stick spans
   about 800 µs, while an unconnected channel jitters a few µs. The person confirms
   (`sweep_done`).
3. **Centre.** The person centres roll, pitch and yaw, puts the throttle down, and
   confirms (`centred`). MAVForge takes the median of five readings as each channel's
   trim, clamped to its sweep.
4. **Checks**, using the channels in `RCMAP_ROLL/PITCH/THROTTLE/YAW`:
   - Each primary channel must have moved.
   - Roll, pitch and yaw trims must lie within 20% of their travel from the midpoint.
   - The throttle must lie within 10% of its low end, or of its high end when
     `RCn_REVERSED=1`.
   - If any check fails, the result is `failed` and no plan is made.
5. **Plan.** Every moved channel gets min/max/trim in a change plan. Channels that
   didn't move are listed and left unchanged. A measured value within 3 µs of the
   current one keeps the current value (listed in the evidence), unless keeping an old
   trim would put it outside the new min..max. So a radio that is already calibrated
   gets no changes and no writes for sampling noise. Mission Planner writes every
   value; on the hardware run, all four changes it would have made were 1–2 µs.

**Success:** a `succeeded` result means the sweep was usable and a plan was recorded
(`result.plan`). It does **not** mean anything was written. The values reach the
vehicle only when a human approves the plan and it is applied with a read-back.

**Interfaces:**
- **CLI:** `mavforge calibrate radio` guides the sweep live, shows the plan, and asks
  before writing. `--plan-only` saves the plan without writing.
- **Python:** `vehicle.calibration.start_radio()`, then `approve_plan(result.plan)` and
  `vehicle.params.apply(...)`.
- **Permissions:** starting the sweep needs only read permission. Applying the plan
  needs `configure`, plus the real-vehicle opt-in when it applies.
- **Not over MCP:** the person at the transmitter has to do the moving and the
  confirming.

**Cancel:** at any time. Nothing has been written.

## Board orientation check

`AHRS_ORIENTATION` tells ArduPilot how the flight controller is mounted. ArduPilot
rotates every accelerometer, gyro and internal-compass sample from the board's axes to
the vehicle's by it. If it is wrong, the autopilot sees a nose-up tilt as a roll, or
upside down. The level calibration can't detect a wrong yaw rotation, because turning a
flat board doesn't change which way is down. Like the radio calibration, the check
**writes nothing**: it proposes `AHRS_ORIENTATION` as a change plan.

**Needs:** the vehicle disarmed, motors unpowered or propellers removed.

**Sequence:**
1. MAVForge reads `AHRS_ORIENTATION`. Custom rotations (100 and up) aren't checked.
2. It requests `SCALED_IMU` (26) with `MAV_CMD_REQUEST_MESSAGE` every 0.1 s.
   ArduPilot reports the accelerometer there after applying `AHRS_ORIENTATION`. It
   shows the tilt the autopilot currently sees, live: "the autopilot sees: nose up 32°,
   right side down 2°". A new line is printed only once pitch or roll has moved 5° from
   the last one printed.
3. **Three poses**, each confirmed by the person:
   - **level**, in the normal flying attitude;
   - **nose up** 30–45°;
   - **right side down** 30–45°.

   Each pose averages 10 readings (about 1 s), and must meet three conditions:
   - **still:** no reading more than 80 mG from the mean;
   - **about 1 g;**
   - **tilted** at least 20° from level (nose up and right side down only).

   A pose that fails a condition is asked for again, up to three times, with the reason
   shown.
4. **Solve.** Gravity in the three poses gives the vehicle's down, forward and right
   axes in the frame ArduPilot currently uses. Combined with the current setting, that
   gives the board's true mounting.
   - The two tilts must agree on the right axis within 30°. This catches raising the
     tail or lowering the left side.
   - The mounting is matched against ArduPilot's rotations 0–43. Rotations 38 and 41
     exist for particular boards and are not proposed: a small tilt belongs in the level
     trim.
   - What doesn't fit the chosen rotation is split in two, and each part is judged on
     its own:
     - **Heading.** How far the two tilts put the board's arrow from the rotation's.
       The tilts measure this well, so it must be within 20°.
     - **First-pose tilt.** How far the first pose was from level with that rotation.
       Gravity alone cannot tell a vehicle resting slightly tilted from a board
       mounted slightly tilted, so this only rejects clear cases (over 15°), with a
       warning over 10° (the most the level calibration corrects). 15° stays well
       below 22.5°, where a 45° rotation would explain the reading as well as a
       square one.
   - When two values describe the same rotation, the current value is kept.
5. **Plan.** Either `AHRS_ORIENTATION` stays as it is ("matches how the board is
   mounted"), or the plan proposes the new value. The evidence says where the board's
   arrow points and which way its top faces.

**After changing it:** reboot, then repeat the level calibration. Both are from
ArduPilot's parameter description: "This option takes affect on next boot. After
changing you will need to re-level your vehicle." MAVForge also recommends repeating the
compass calibration. The accelerometer calibration stays valid, because ArduPilot
calibrates in the sensor frame, before the board rotation.

**Interfaces:**
- **CLI:** `mavforge calibrate orientation` (with `--yes`, `--plan-only`, `--offline`,
  as for radio).
- **Python:** `vehicle.calibration.start_orientation()`.
- **Permissions:** the check needs only read permission. Applying needs `configure`.
- **Not over MCP:** the person has to tilt the vehicle and confirm each pose.

**Limits:**
- The check can't tell if *both* tilts were reversed (tail raised *and* left side
  lowered). That looks like a board turned 180°. The live tilt line shows what the
  autopilot sees, and the evidence names where the arrow points; compare that with the
  board.
- External compasses have their own `COMPASS_ORIENTn` and aren't checked.

## Flight mode assignment

`FLTMODE_CH` selects a flight mode by six fixed PWM bands — not configurable, and not
moved by `RCn_MIN`/`RCn_MAX` on that channel (`RC_Channel::read_6pos_switch`, confirmed
directly against ArduPilot source; see [research.md](research.md#flight-modes-and-failsafes-planned-behaviour-for-m8)
for the exact breakpoints). Like radio calibration and the orientation check, nothing
is written by the check itself: the result is a change plan for whichever
`FLTMODEn` values were requested.

**Needs:** the vehicle disarmed; transmitter on and the receiver bound; `FLTMODE_CH`
already set to the channel the mode switch is on.

**You supply the target values up front** (`FLTMODEn=VALUE`, same syntax as
`params set`) — MAVForge never picks a mode for a switch position. Which mode belongs
where is the pilot's judgment call, the same reasoning as the other guided checks.

**Sequence:**
1. MAVForge reads `FLTMODE_CH`. `0` (disabled) fails immediately: no channel is read for
   flight mode at all.
2. It requests `RC_CHANNELS` (65) via `MAV_CMD_REQUEST_MESSAGE` at 10 Hz, discretising
   that channel's PWM into the six fixed bands as the person flips the switch, live.
3. **Sweep.** The person flips the switch through every position they use, pausing
   briefly on each, then confirms. For every `FLTMODEn` being requested, MAVForge
   checks whether its band was actually reached during the sweep.

**Plan.** Every requested `FLTMODEn` is proposed, whether or not its band was reached —
an unreached band is a warning ("check the switch/mixing"), not a reason to withhold the
value, since the person may be setting up a position they haven't wired yet.

**Success:** a `succeeded` result means the parameters named exist and a plan was
recorded (`result.plan`). As with radio calibration, it does **not** mean anything was
written: that needs the normal approval.

**Interfaces:**
- **CLI:** `mavforge calibrate flight-modes FLTMODE1=0 FLTMODE3=5 ...` (with `--yes`,
  `--plan-only`, `--offline`, as for radio).
- **Python:** `vehicle.calibration.start_flight_modes({"FLTMODE1": 0, ...})`.
- **Permissions:** the sweep needs only read permission. Applying needs `configure`.
- **Not over MCP:** which mode belongs on which switch position is the pilot's
  judgment call, not something to infer from telemetry — the same reasoning as radio
  calibration and the orientation check.

**Limits:** the sweep only validates that the switch *can* reach a requested band, not
that your transmitter's mixing or the receiver's failsafe behaviour matches what you
expect in flight.

## Recovery

- **Bad gyro, level or accel results.** Run the calibration again. Your earlier
  values are in `params_before` in the result and in the audit log. You can also
  restore them with `mavforge params restore <backup>`, from a backup you took
  beforehand.
- **Compass failure.** Nothing is saved on failure. Fix the cause the hint names
  (interference, orientation, uneven rotation), then run it again.
- **Interrupted runs.** After a cancelled or interrupted accelerometer calibration,
  reboot the autopilot before using it.
- **Bad radio values.** The apply step takes a backup first. Undo with
  `mavforge params restore <pre-change-backup-id>`, or run the calibration again.
