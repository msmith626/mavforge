# Research notes

These are the primary-source findings behind MAVForge's design, gathered in October 2026. Source line numbers refer to `master` at that time unless a tag is named. Each item is marked:

- **[verified]**: confirmed by MAVForge tests against ArduCopter 4.7.2 SITL
- **[source]**: read in source or documentation, but not yet exercised by MAVForge
- **[unverified]**: could not be confirmed

Mission Planner, ArduPilot and MAVProxy were used as *references*. No code was copied from them (see [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)).

## pymavlink (2.4.50, 2026-09-24, LGPLv3)

Sources: https://pypi.org/project/pymavlink/, https://github.com/ArduPilot/pymavlink (`mavutil.py`).

- **Process-global dialect.** The dialect is chosen from the `MAVLINK_DIALECT` and `MAVLINK20` environment variables at import time. If `MAVLINK_DIALECT` is unset it defaults to `all`. When pymavlink sees a 0xFD start byte, `mavfile` switches itself to MAVLink 2 by mutating `os.environ`.
  → MAVForge sets `ardupilotmega` + MAVLink 2 exactly once, in `mavforge/transport/dialect.py`. [source]
- **`recv_match` discards non-matching messages**, so two concurrent waiters lose each other's messages.
  → MAVForge uses one reader thread and a `MessageRouter` with per-subscriber queues. Covered by unit tests. [verified]
- **No `py.typed` marker; not thread-safe.** The default source IDs are 255/0. [source]
- **`mavlink_connection` executes certain inputs.** It opens log files, and it executes paths ending in `.elf` or containing `/bin/`. A bare `host:port` is treated as UDP.
  → `mavforge/transport/connstr.py` only accepts tcp/tcpin/udpin/udpout/serial. [source]
- **Prints to stdout** on some TCP errors. The MCP SDK v2 stdio transport claims fd 1 and diverts stray writes. [source]
- **TCP EOF is never reported.** `mavtcp.recv()` answers EOF and reset with `handle_eof()` / `handle_disconnect()`. Those print to stdout and call `reconnect()`, which does nothing when `autoreconnect=False`. The socket then stays readable at EOF forever: the loss is never reported, and every poll prints again. MAVForge replaces both handlers so they raise, which turns the loss into a link failure. Covered by a regression test. [verified]
- **SITL start-up window.** Once, ArduCopter SITL closed the first test connection shortly after a bare TCP readiness probe. The SITL fixture now waits for a full MAVLink session instead. Could not be reproduced in 7 later runs. [verified once]
- **Missing enum entries.** pymavlink 2.4.50's generated `ardupilotmega` dialect has no `MAV_RESULT_CANCELLED` (6), and no values 9 or 10, all of which are in upstream `common.xml`. MAVForge maps them by number. [verified locally]
- **Different `MAV_TYPE` names.** It lacks some newer `MAV_TYPE` names, e.g. it has `VTOL_DUOROTOR` rather than `VTOL_TAILSITTER_DUOROTOR`. Adapters resolve names defensively. [verified locally]

## MAVLink Command Protocol

Source: https://mavlink.io/en/services/command.html

- **COMMAND_LONG retransmission** increments `confirmation`. `COMMAND_ACK.result` uses `MAV_RESULT`. An `IN_PROGRESS` result carries `progress` and means the client should keep waiting. [source]
- **No numeric timeouts or retry counts** are specified; they are left to each flight stack. [source]
- **Mission Planner defaults** (`ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs`, `doCommandAsync`): 3 retries × 2000 ms. PREFLIGHT_CALIBRATION gets 1 retry and 25 s. [source]
- **MAVForge retries only commands the caller declares idempotent.** A lost ACK on a non-idempotent command is reported as `unknown`, not `failed`. [verified with the fake vehicle]
- **ArduPilot converts COMMAND_LONG to COMMAND_INT** for non-location commands (`GCS_Common.cpp`, `convert_COMMAND_LONG_to_COMMAND_INT`). This truncates params 5 and 6 to int32. [source]
- **`MAV_CMD_REQUEST_MESSAGE` (512)** is used for one-shot reads, so read-only use doesn't change stream rates. ArduPilot SITL answers requests for `AUTOPILOT_VERSION`, `SYS_STATUS`, `GPS_RAW_INT` and `SIMSTATE`. [verified]

## MAVLink Parameter Protocol and ArduPilot specifics

Sources: https://mavlink.io/en/services/parameter.html, ArduPilot `libraries/GCS_MAVLink/GCS_Param.cpp`, `libraries/AP_Param/AP_Param.cpp`.

- **"C cast" float encoding.** ArduPilot sends every value as a float, and advertises this with `MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT` [verified]. Integers with magnitude above 2^24 can be rounded; MAVForge flags them as `precision_limited`.
- **ArduPilot ignores `param_type` in PARAM_SET** and uses the type it has stored. [source]
- **Indices are not stable** across boots or firmware versions. [source]
- **Unknown names produce PARAM_ERROR.** ArduPilot 4.7.2 answers PARAM_REQUEST_READ for an unknown name with `PARAM_ERROR` (`MAV_PARAM_ERROR_DOES_NOT_EXIST`). [verified] Older firmware may simply not answer [unverified], so MAVForge's timeout message says the parameter may not exist.
- **Doc vs. source disagreement on write echoes.** mavlink.io says ArduPilot doesn't echo PARAM_VALUE after PARAM_SET, but current source does (`AP_Param::save` → `GCS_SEND_PARAM`). Future write support will wait for the echo **and** read the value back. [source]
- **Mission Planner's writes are only partly verified** (`setParamAsync`): it waits 700 ms for an echo with a matching name and retries 3×, but doesn't compare the echoed value. pymavlink's `mavparm.mavset` likewise checks only the name. [source]
- **Full download.**
  - Mission Planner re-requests the list after 4 s of silence, then fetches missing indices with `PARAM_REQUEST_READ`.
  - MAVForge does the same with a 1 s gap timeout and up to 3 stalled rounds.
  - Against SITL: 1369/1369 parameters received with nothing missing [verified]. Gap recovery is tested with the fake vehicle.
- **MAVFTP bulk download** of `@PARAM/param.pck?withdefaults=1` is the faster option. Its format is documented in ArduPilot `libraries/AP_Filesystem/README.md`. Planned. [source]
- **Parameter names change between firmware versions.** In 4.7, `SYSID_THISMAV` became `MAV_SYSID` and `SYSID_MYGCS` became `MAV_GCS_SYSID` [verified]. `ARMING_CHECK` was replaced by the inverted `ARMING_SKIPCHK` [source]. Any parameter metadata, diff or restore must therefore be keyed by firmware version.
- **Metadata** is published at `https://autotest.ardupilot.org/Parameters/<Vehicle>/apm.pdef.{xml,json}`, with versioned sets at `/Parameters/versioned/<Vehicle>/stable-X.Y.Z/`. It is generated from GPLv3 source, and I found no separate license statement [unverified].

## Parameters that change on their own ("volatile")

A diff between two reads of an unchanged vehicle should show nothing. Some ArduPilot parameters are rewritten by the firmware itself, so MAVForge flags them as `volatile`. It doesn't hide them.

- **ArduPilot metadata.** `apm.pdef.xml` (Copter stable-4.7.1, Plane stable-4.6.2) marks only these as `Volatile`: `BARO1..3_GND_PRESS`, `BARO_GND_TEMP` and `BARO_FIELD_ELV`, plus `TCAL_BARO_EXP`, `TCAL_TEMP_MAX` and `TCAL_TEMP_MIN` on Copter. No Plane/Copter `stable-4.7.2` metadata directory was published yet, and the versioned directories carry only XML. [verified]
- **SITL observation.** I read all parameters of ArduCopter 4.7.2 SITL after three successive reboots without wiping EEPROM, waiting 20 s after each boot. [verified]
  - First → second boot: `STAT_BOOTCNT`, `BARO2_GND_PRESS`, `INS_GYROFFS_{X,Y,Z}`, `INS_GYR2OFFS_{X,Y,Z}` and `INS_GYR2_CALTEMP` changed. The gyro offsets are re-measured at boot when `INS_GYR_CAL=1`.
  - Second → third boot: only `STAT_BOOTCNT` changed.
- **Flagged set.** The ArduPilot adapter flags the metadata set above, plus `STAT_BOOTCNT`, `STAT_RUNTIME`, `STAT_FLTTIME`, `INS_GYR*OFFS_*` and `INS_GYR*_CALTEMP`.
- **Not flagged.** Accelerometer and compass calibration values are deliberately *not* flagged: a change there is real.

## Identification

- **HEARTBEAT selection.** Mission Planner ignores GCS-type senders and prefers component 1. MAVForge does the same, and also reports any other autopilot system IDs it hears. [verified with the fake vehicle]
- **`AUTOPILOT_VERSION`.** `flight_sw_version` packs `major<<24 | minor<<16 | patch<<8 | type`, where type is 0 dev / 64 alpha / 128 beta / 192 rc / 255 official. `flight_custom_version` carries the first 8 characters of the git hash. Against SITL: `4.7.2` and `fb773567`, which matches `git-version.txt`. [verified]
- **SITL detection.** ArduPilot only sends `SIMSTATE` from simulator builds (`AP_SIM_ENABLED`). If a `SIMSTATE` arrives on request, MAVForge reports `simulated: true`. Otherwise it treats the vehicle as real hardware. Confirmed `true` on ArduCopter 4.7.2 SITL and `false` on an ArduPlane 4.6.2 flight controller [verified].

## Pre-arm and health

Source: ArduPilot `libraries/AP_Arming/AP_Arming.cpp`, `libraries/GCS_MAVLink/GCS.cpp`.

- **Pre-arm failures** are reported as STATUSTEXT `"PreArm: <reason>"`, every 30 s by default. `MAV_CMD_RUN_PREARM_CHECKS` (401) forces an immediate report; MAVForge will use it in diagnostics (planned). [source]
- **The SYS_STATUS bit `MAV_SYS_STATUS_PREARM_CHECK`** (0x10000000) is *present* always, *enabled* if any checks are enabled, and *healthy* when armed or when the checks are passing. Against SITL with no GPS fix, this bit reports failing. [verified]

## Calibration (planned; behaviour for future state machines)

Sources:
- ArduPilot `libraries/GCS_MAVLink/GCS_Common.cpp` (`handle_command_preflight_calibration`), `libraries/AP_AccelCal/AP_AccelCal.cpp`, `libraries/AP_Compass/AP_Compass_Calibration.cpp`
- Mission Planner `GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs`, `ConfigHWCompass2.cs`, `ConfigRadioInput.cs`, `ConfigESCCalibration.cs`

All items in this section are [source].

**General**
- If armed, ArduPilot sends STATUSTEXT `"Disarm to allow calibration"` and returns FAILED.
- Mission Planner itself performs no armed-state check.
- In `MAV_CMD_PREFLIGHT_CALIBRATION` (241), the first matching branch wins:

| Parameter | Effect |
|---|---|
| p1=1 | Gyro |
| p3=1 | Baro (plus airspeed → IN_PROGRESS) |
| p4 | RC |
| p5=1 | Full 6-pose accel |
| p5=2 | Level/trim |
| p5=4 | Simple accel |
| p5=76 | Force-save accel |
| p2=1 | Returns UNSUPPORTED; compass calibration uses `MAV_CMD_DO_START_MAG_CAL` |

**Accelerometer, 6-pose**
1. The vehicle sends STATUSTEXT `"Place vehicle <pose> and press any key."`.
2. It also sends a COMMAND_LONG `MAV_CMD_ACCELCAL_VEHICLE_POS` (42429) with p1 = pose (1–6), repeating every 1 s.
3. The GCS echoes the same command and pose once the human has positioned the vehicle.
4. The end is signalled by p1 = 16777215 (success) or 16777216 (failure), plus STATUSTEXT `"Calibration successful"`, `"Calibration FAILED"` or `"Calibration cancelled"`.

Mission Planner uses only the STATUSTEXT; MAVForge will require both.

**Compass (onboard)**
- `MAV_CMD_DO_START_MAG_CAL` (42424) takes mask, retry, autosave, delay and autoreboot. Progress arrives as `MAG_CAL_PROGRESS` (191), results as `MAG_CAL_REPORT` (192). Then `MAV_CMD_DO_ACCEPT_MAG_CAL` or `MAV_CMD_DO_CANCEL_MAG_CAL`.
- Mission Planner treats a compass as done when `autosaved == 1`. Its `cal_status` check is commented out.
- MAVForge will require `cal_status == MAG_CAL_SUCCESS` and then read the parameters back.
- The fitness threshold is `COMPASS_CAL_FIT`.

**Radio**
- A GCS-side sweep records min/max/trim, then writes `RCn_MIN`, `RCn_MAX` and `RCn_TRIM`.
- Mission Planner skips invalid channels without telling the user. MAVForge will report them.

**ESC**
- The `ESC_CALIBRATION` parameter plus a power cycle. This is high risk and deferred.

**SITL**
- `-M calibration` (`libraries/SITL/SIM_Calibration.cpp`) supports accel and compass calibration, with poses driven through SERVO5–8. ArduPilot's autotest `AccelCal` and `SITLCompassCalibration` tests are the model for MAVForge's future SITL calibration tests.

## MCP

Sources: https://modelcontextprotocol.io/specification/2026-07-28, https://github.com/modelcontextprotocol/python-sdk

- **Spec revision 2026-07-28.**
  - The protocol is stateless, and `server/discover` was added.
  - Elicitation works through multi-round-trip requests (MRTR).
  - Tasks moved to an extension.
  - HTTP+SSE is deprecated.
  - Tool annotations are untrusted hints.
- **SDK `mcp` 2.3.0.**
  - `MCPServer` replaces `FastMCP`.
  - Pydantic return types produce `outputSchema` and `structuredContent`.
  - `ToolAnnotations` takes snake_case fields.
  - Sync tools run in worker threads.
  - Only `ToolError` messages reach the client; other exceptions are masked.

  All of this was [verified] with a spike test and MAVForge's MCP tests.
- **Existing MAVLink MCP servers** focus on flight (arm, takeoff, goto). Examples: rmeadomavic/ardupilot-mcp, deepak61296/mavlink-mcp, hieutachi/mavlink-mcp, hfujikawa77/ardupilot-mcp-server-sandbox. MAVForge's focus is configuration, diagnostics and calibration, without flight tools. [source]

## SITL

- **Prebuilt binaries.** ArduPilot publishes official prebuilt SITL binaries, e.g. `https://firmware.ardupilot.org/Copter/stable-4.7.2/SITL_x86_64_linux_gnu/arducopter`. They are statically linked. [verified]
- **SERIAL0** is a TCP server on 5760 (+10 per `-I` instance) that accepts reconnects [verified]. SITL waits for the first client before it starts booting [source].
- **No official run image.** I found no maintained official ready-to-run Docker image, so MAVForge builds its own (`docker/sitl`). [source]

## Hardware observations

**ArduPlane 4.6.2, USB, 2026-10-21.** These come from `tests/hardware`, which uses the Python API and is read-only.

- **Identification.** `AUTOPILOT_VERSION` decoded as 4.6.2 official, git hash `1ebd4d99`.
  - `MAV_TYPE` is `FIXED_WING`, which the adapter maps to the `plane` family.
  - USB VID:PID is 0x1209:0x5741, ArduPilot's shared IDs, so they do not identify the board.
  - `board_version` 0x04520000.
- **Simulator detection.** No `SIMSTATE` arrived, so the vehicle is `simulated: false`.
- **Parameters and status.** The full parameter download completed with no missing indices. The status snapshot succeeded with the vehicle disarmed.
- **Serial dependency.** The first run found that pymavlink needs `pyserial` but doesn't declare it, so MAVForge now does. The test suite covers serial links via a pseudo-terminal.
- **Parameter set.** 1117 parameters. `SYSID_THISMAV` and `SYSID_MYGCS` still exist, which confirms that the rename to `MAV_SYSID` happened in 4.7.
- **Pre-arm bit with checks disabled.** With `ARMING_CHECK=0`, ArduPlane marks the `PREARM_CHECK` bit as *present* but *not enabled*. MAVForge now reports this as `prearm_checks_enabled: false` and shows "DISABLED on vehicle", where it previously said "not reported".
- **GPS without a fix.** With `NO_FIX` the receiver reported `eph` = 9999 (HDOP 99.99), which is a placeholder. MAVForge reports HDOP only when there is a 2D or 3D fix.
- **Bench state.** The board was on the bench with no RC receiver and no flight battery. It showed mode RTL with MAV_STATE CRITICAL and `RC_RECEIVER` unhealthy, which is consistent with a radio failsafe; the cause was not confirmed. The battery read about 0.14 V.
- **Backup and diff.** I backed up all 1117 parameters, then diffed the live plane against that backup a few seconds later. The only change was `STAT_RUNTIME`, which was correctly flagged as volatile. The exported `.param` file round-trips all 1117 values exactly.
- **Diagnostics.** `mavforge diagnose` reported:
  - **Faults:** an uncalibrated primary compass (`COMPASS_OFS_*` = 0, `COMPASS_LEARN=0`), plus three live bench states from SYS_STATUS: GPS, RC receiver and terrain unhealthy.
  - **Warnings:** pre-arm checks disabled, and no battery failsafe action.
  - **Passed:** accelerometer calibration, radio calibration (1102/1895/1498), throttle failsafe, GPS configured, logging.

## Diagnostics rules

These are the sources for the ArduPilot rules in `platforms/ardupilot/rules.py`. The code was read at tag `Plane-4.6.2`, and the parameter names and values were checked against `apm.pdef.xml` for Plane stable-4.6.2 and Copter stable-4.7.1.

- **Accelerometer.** `AP_InertialSensor::accel_calibrated_ok_all()` fails if any detected accelerometer's saved offset vector is exactly zero, or its scale is zero. The pre-arm message is `"3D Accel calibration needed"`. The rule checks only instances whose `INS_ACC{,2,3}_ID` parameter exists and is non-zero.
- **Compass.**
  - `AP_Arming::compass_checks()` requires configured compasses (`Compass::configured()`: offsets not all zero, and the device ID matches) on Copter, Heli and Blimp. On other vehicles it requires them unless `learn_offsets_enabled()`, which means `COMPASS_LEARN == 3` (`LEARN_INFLIGHT`, `AP_Compass.h`).
  - Secondary compasses with zero offsets are reported as *suspected*: `COMPASS_USE2/3` are priority-indexed, so whether those compasses are actually used can't be determined from parameters.
- **RC.**
  - `AP_Arming::rc_calibration_checks()` only rejects `RCn_MIN > RCn_TRIM` or `RCn_MAX < RCn_TRIM`, for channels with a live signal.
  - Firmware defaults (1100/1900/1500) on all four primary channels are a MAVForge inference. That's why the rule reports *suspected*, not *fault*.
- **Arming checks.**
  - `ARMING_CHECK` (≤4.6) is a bitmask where bit 0 means All.
  - `ARMING_SKIPCHK` (4.7) is inverted: 0 means skip nothing.
  - Bit names come from the metadata.
- **Failsafes.** The rule reads Plane `THR_FAILSAFE` (0 Disabled, 1 Enabled, 2 EnabledNoFailsafe) and Copter `FS_THR_ENABLE`. `BATT_FS_LOW_ACT` / `BATT_FS_CRT_ACT` = 0 means "None" on Plane and "Warn only" on Copter, so no action either way.
- **Pre-arm reasons.** `MAV_CMD_RUN_PREARM_CHECKS` returns ACCEPTED on SITL and real hardware, then STATUSTEXT `"PreArm: …"`. ArduPilot refuses it while armed.
  - Right after boot, SITL reports transient failures that cleared within about 15 s: `Accels inconsistent`, `EKF attitude is bad`, `AHRS: EKF3 not started`, `Compass 1 not healthy`, `Battery 1 unhealthy`.
  - For this reason MAVForge warns when `SYSTEM_TIME.time_boot_ms` is under 60 s.
- **Rejected rule: "value outside documented range".**
  - On the stock ArduPlane 4.6.2 configuration it flagged 17 parameters. Nearly all were firmware defaults where 0 means "disabled" but falls outside the documented range (e.g. `AIRSPEED_STALL`, `FENCE_TOTAL`), or board-specific pin numbers.
  - That is too noisy for diagnostics. Range validation belongs at write time instead (planned).

## Parameter writes

- **Write path.** `PARAM_SET` carries the value as a float and the stored type; ArduPilot uses its stored type.
  - **ArduCopter 4.7.2 SITL:** `RC1_TRIM` and `BATT_FS_LOW_ACT` were written, verified by `PARAM_REQUEST_READ`, and restored. Each write was echoed with `PARAM_VALUE`. [verified]
  - **ArduPlane 4.6.2 hardware (USB):** `BATT_CAPACITY` 3300→3400 and `FS_SHORT_TIMEOUT` 1.5→2.0 were written and verified by read-back. Both were then restored from the automatic pre-change backup and verified again. A later diff against the 2026-10-21 baseline showed no configuration differences, only volatile ones. The parameters were chosen with the board owner's consent as having no physical effect while disarmed on the bench. [verified]
- **Metadata availability** (2026-10-23):
  - `versioned/Plane/stable-4.6.2/apm.pdef.xml` exists: 4818 definitions, including `FS_SHORT_TIMEOUT` range 1..100.
  - `versioned/Copter/` stops at stable-4.7.1, so ArduCopter 4.7.2 falls back to the latest development definitions, and plans say so.
  - The versioned directories publish XML only.

  [verified]
- **MCP elicitation, `mcp` 2.3.0.** A tool parameter annotated `Annotated[..., Resolve(fn)]` is left out of the tool's input schema and is filled by the server-side resolver. An agent-supplied argument with the same name is ignored. A client without elicitation support can be detected through `ctx.client_capabilities.elicitation` and given a fallback. A declined elicitation aborts the call. [verified with spike and tests]

## Calibration behaviour verified

These were checked against ArduCopter 4.7.2 SITL. The source was read at tag `Plane-4.6.2`.

- **Gyro and level run synchronously.**
  - Both run inside `MAV_CMD_PREFLIGHT_CALIBRATION`, with `EXPECT_DELAY_MS(30000)`, and the ACK comes only when they finish.
  - Level calls `calibrate_gyros()` first. It returns `TEMPORARILY_REJECTED` within 5 s of a previous accelerometer calibration (`last_accel_cal_ms`).
  - Both passed in SITL. [verified]
- **Compass messages are not streamed to MAVForge.** Without stream configuration, `MAG_CAL_PROGRESS`/`MAG_CAL_REPORT` never arrived on the link. `MAV_CMD_REQUEST_MESSAGE` for 191/192 returns them. [verified]
- **One report request returns every finished compass.** `Compass::send_mag_cal_report()` loops over all instances and sends one report per finished compass. A single request therefore yields up to three reports, beginning after the last one sent.
  - Taking only the first reply returned the same compass every time.
  - Fixed by collecting from a subscription.
  - The fake vehicle now reproduces the loop, and a regression test covers it. [verified]
- **With autosave off, nothing is saved until accepted.** After every compass reported `MAG_CAL_SUCCESS`, `COMPASS_OFS*` were unchanged until `DO_ACCEPT_MAG_CAL`, after which they matched the reported offsets. [verified]
- **Status names differ between pymavlink and ArduPilot.** pymavlink's `MAG_CAL_STATUS` names statuses 6 and 7 `FAILED_ORIENTATION` and `FAILED_RADIUS`; ArduPilot's `CompassCalibrator` calls them `BAD_ORIENTATION` and `BAD_RADIUS`. Statuses 8–10 exist in the dialect but are not produced by ArduPilot 4.6.2. [verified locally]
- **Accelerometer calibration cannot be cancelled remotely.**
  - The vehicle sends `ACCELCAL_VEHICLE_POS` with target 0/0 at 1 Hz while it waits.
  - Its reply to the GCS echo comes from `handle_command_accelcal_vehicle_pos`.
  - `AP_AccelCal::cancel()` is only reached on arming (`AP_InertialSensor::acal_update`). There is no MAVLink cancel, and waiting for a pose has no timeout.
  - The full 6-pose run passed in SITL with the `calibration` model. [verified]
- **SITL `calibration` model.**
  - In `libraries/SITL/SIM_Calibration.cpp`, `SERVO5` selects the mode: 1150 attitude, 1250 autonomous compass-calibration rotation, ≥1300 angular velocity. `SERVO6`–`8` set the attitude.
  - `DO_SET_SERVO` works because `SERVO5_FUNCTION=0`.
  - MAVForge's SITL tests use this only to act as the person at the vehicle. [verified]
