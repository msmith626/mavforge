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
| Gyro | yes | success + failure | **passed** | not yet | yes |
| Level (AHRS trim) | yes | success + rejected | **passed** | not yet | yes |
| Compass (onboard) | yes, if `COMPASS_CALIBRATION` capability | success (1 and 3 compasses), per-compass failure, accept-without-save, cancel, timeout | **passed** (3 compasses, `calibration` model) | **passed**: ArduPlane 4.6.2, 1 compass, USB power only (2026-10-23) | yes |
| Accelerometer, 6 positions | yes | success, failure, silence, cancel | **passed** (`calibration` model) | not yet | **no** |
| Radio (RC) | yes (GCS-side sweep, proposes a change plan) | success, apply after approval, unmoved/uncentred/throttle-high, reversed throttle, RCMAP, jitter, no receiver, receiver in failsafe, cancel, armed | **passed** (`RC_CHANNELS_OVERRIDE` from the test) | not yet | **no** |

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
   didn't move are listed and left unchanged.

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
