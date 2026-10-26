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

**Radio** (Mission Planner `ConfigRadioInput.cs`, `BUT_Calibrateradio_Click`)
- A GCS-side sweep records min/max/trim, then writes `RCn_MIN`, `RCn_MAX` and `RCn_TRIM`. The firmware takes no part.
- Mission Planner:
  - Warns "Ensure your motor does not have power/no props!!!".
  - Requests the RC stream at 10 Hz.
  - Accepts a sample only if **channel 1** is inside 800..2200, then updates min/max for all 16 channels from it.
  - After "Ensure all your sticks are centered and throttle is down", reads trim once per channel and clamps it to that channel's min..max.
  - Writes a channel only if `min < max`, all values are non-zero, and trim is inside min..max. There is no minimum travel, so an unconnected channel jittering 1500 ± 2 is written with a 4 µs range. Its own results dialog notes unconnected channels "show about 1500 ± 2".
  - Skips invalid channels without telling the user. MAVForge reports them.

**ESC**
- The `ESC_CALIBRATION` parameter plus a power cycle. This is high risk and deferred.

**SITL**
- `-M calibration` (`libraries/SITL/SIM_Calibration.cpp`) supports accel and compass calibration, with poses driven through SERVO5–8. ArduPilot's autotest `AccelCal` and `SITLCompassCalibration` tests are the model for MAVForge's future SITL calibration tests.

## Flight modes and failsafes (planned; behaviour for M8)

Sources:
- ArduPilot `libraries/RC_Channel/RC_Channel.cpp` (`read_6pos_switch`), tag `Plane-4.6.2`
- `https://autotest.ardupilot.org/Parameters/versioned/Plane/stable-4.6.2/apm.pdef.xml` — the
  same metadata source MAVForge's own change plans cite (`metadata_source` field)

All items in this section are **[source]** unless marked otherwise.

**This is two different shapes of work, not one feature** — the single most important
scoping finding. Conflating them would produce the wrong architecture for one of them.

**Flight-mode assignment is a guided workflow**, the same shape as radio calibration
(M6) and the orientation check (M7): it needs to watch live `RC_CHANNELS` while a
person moves a switch, so it belongs as a new `Workflow` subclass
(`CalibrationKind.FLIGHT_MODES`), not a diagnostics rule.
- `FLTMODE_CH`: 0 = disabled (no mode channel read at all), 1–16 = that RC channel.
- `RC_Channel::read_6pos_switch` maps the channel's raw PWM to one of six positions
  with these exact breakpoints (confirmed directly in source, not inferred from the
  nominal 1165/1295/.../1815 centre values the docs quote):

  | PWM | Position | Parameter |
  |---|---|---|
  | < 1231 | 0 | `FLTMODE1` |
  | 1231–1360 | 1 | `FLTMODE2` |
  | 1361–1490 | 2 | `FLTMODE3` |
  | 1491–1620 | 3 | `FLTMODE4` |
  | 1621–1749 | 4 | `FLTMODE5` |
  | ≥ 1750 | 5 | `FLTMODE6` |

  Boundaries are fixed, not configurable, and use raw receiver PWM — `RCn_MIN`/`RCn_MAX`
  on the mode channel don't move them. **[unverified]**: the ~0.2 s debounce before a
  position change takes effect is widely documented but not yet confirmed against this
  same source file.
- A guided workflow for this would, like radio calibration, request `RC_CHANNELS` at
  10 Hz on `FLTMODE_CH`, show the live PWM and which of the six bands it falls in as the
  person flips the switch, let them pick a mode per band (from the firmware's own
  `FLTMODE1` value-list metadata, same as every other `ChangePlan`), and propose
  `FLTMODE1`–`FLTMODE6` as a plan — writing nothing itself, same as M6/M7.
- Same reasoning as M6/M7 for **not via MCP**: which mode belongs on which switch
  position is the pilot's judgment call, not something to infer from telemetry alone.

**Failsafe review is a diagnostics extension, not a guided workflow** — there is no
physical action for a person to perform; it is a static read of already-known
parameters, the exact shape `diagnose` already has. It should extend
`services/diagnostics.py` / `platforms/ardupilot/rules.py`, not add a new
`CalibrationKind`.
- `FS_SHORT_ACTN` (short failsafe, ≥`FS_SHORT_TIMEOUT`, default 1.5 s): `0`
  CIRCLE/no-change-if-already-in-AUTO|GUIDED|LOITER, `1` CIRCLE, `2` FBWA at zero
  throttle, `3` Disable, `4` FBWB.
- `FS_LONG_ACTN` (long failsafe, ≥`FS_LONG_TIMEOUT`, default 5 s): `0` Continue, `1`
  RTL, `2` Glide, `3` Deploy Parachute, `4` Auto.
- `FS_GCS_ENABL`: `0` Disabled, `1` Heartbeat, `2` HeartbeatAndREMRSSI, `3`
  HeartbeatAndAUTO — triggers after `FS_LONG_TIMEOUT` with no MAVLink heartbeat.
  ArduPilot's own parameter description warns that enabling it can let the motor keep
  running on the ground after contact is lost, and recommends `ARMING_REQUIRED` on
  electric planes if it's on. **Directly relevant to MAVForge**: the project's own
  design choice to never send a HEARTBEAT (README, "Safety model") means a vehicle with
  `FS_GCS_ENABL` enabled will, after `FS_LONG_TIMEOUT`, treat MAVForge's own presence as
  no different from no GCS at all — worth calling out explicitly in whatever this rule
  reports, not just checking the parameter is non-zero.
- Battery failsafe (`BATT_FS_LOW_ACT`, `BATT_FS_CRT_ACT`, `BATT_LOW_VOLT`,
  `BATT_CRT_VOLT`, and the `_MAH` equivalents) is already on the roadmap as a
  Diagnostics item ("failsafe thresholds against battery capacity") — this session's
  bench vehicle is the motivating case: an action configured with its threshold left at
  the default `0`, so it could never trigger. The rule needs to check the action and its
  threshold together, not either alone.
- This needs one piece of new plumbing the existing rules don't have: a way for a
  finding to carry a proposed fix. `Finding` (`domain/models.py`) has no field for one
  today. The natural shape is a `proposed_plan: ChangePlan | None` field, built by
  calling `services.changes.plan_changes()` (a plain function, already independent of
  the calibration machinery) from within the rule, the same way radio/orientation build
  their plan at the end of a run. This is exactly the existing roadmap item "findings
  that propose a parameter plan" — this review is its first real use, not a separate
  thing.
- **Open question, deliberately not decided here**: should this be available via MCP?
  Unlike flight-mode assignment there is no live human action involved — it's exactly
  as read-only as `diagnose` itself, which already is MCP-available. An agent proposing
  a plan is not the same as an agent approving one (MCP still can't do that), so there
  isn't an obvious safety reason to exclude it the way M6/M7 are excluded. Worth a
  deliberate decision when this is built, not an assumption either way.

## Guided setup wizard (planned; behaviour for M9)

The key finding here isn't from an external source — it's from reading MAVForge's own
code: **most of the "is this done" signal the wizard needs already exists**, as
`diagnose`'s findings. M9 is mostly plumbing to interpret that signal as a sequence,
not a new detection engine.

**What already answers "is this configured" today**, by `Finding.check`:
- `ardupilot.accel_cal`, `ardupilot.compass_cal`: `FAULT` if uncalibrated.
- `ardupilot.rc_cal`: `SUSPECTED` ("probably never done") if every primary channel is
  still at the firmware default.
- `ardupilot.throttle_failsafe`, `ardupilot.battery_failsafe`: `WARNING` if disabled,
  and — since M8b — `battery_failsafe` can carry a `proposed_plan` for the "no action"
  case specifically.
- `ardupilot.gps_configured`, `ardupilot.logging`, `ardupilot.arming_checks`,
  `ardupilot.frame_class` (copter/heli only): all `WARNING`/`PASSED` from a single
  parameter read, no live sensor interaction needed.

**What has no passive signal at all today**, and why:
- **Gyro and level calibration (M5)**: `INS_GYROFFS_*`/`AHRS_TRIM_X/Y` existing and
  non-zero *could* be checked (the same shape as the existing `accel_cal` rule), but
  no rule does this yet. A real gap worth filling as a small addition alongside M9, not
  a reason to block it.
- **Board orientation (M7)**: fundamentally can't be answered from parameters alone.
  `AHRS_ORIENTATION=0` is exactly as likely to be "correct for this board" as "never
  set" — only the gravity-based check (three poses) tells them apart. At best, a new
  rule could flag "`AHRS_ORIENTATION` is still the firmware default" as `SUSPECTED`,
  the same honesty `_rc_cal` already uses for "probably never done."
- **Flight-mode assignment (M8a)**: same shape as orientation — `FLTMODE_CH=0`
  (disabled) is a clean, confident signal ("not set up"), but whether the six
  `FLTMODEn` values are *right* can't be derived from parameters, only from flying or
  from the M8a sweep. A rule could add the `FLTMODE_CH=0` case cleanly; "are the six
  values sensible" has to stay a recommendation to run the check, not a verdict.

**What a true "was this verified for this exact vehicle" signal would need, and why
it's deliberately out of scope for a first version:** the audit log
(`~/.local/state/mavforge/audit.jsonl`, `policy/audit.py`) already records every
operation's outcome per vehicle identity — in principle, "did a successful orientation
check run on *this* vehicle since its last reboot" is answerable from it. But today
`AuditLog` is write-only: `record()` appends; nothing reads it back. Building that
read side (parse JSONL, filter by vehicle identity and event, define what counts as
"still valid" — a parameter changing since the last run should invalidate it) is real,
separate work. **First version of M9 should rely only on live parameters** (what
`diagnose` already provides) and treat orientation/flight-modes as "unknown, run the
check" rather than attempting a false done/not-done verdict from an audit trail that
doesn't exist yet. Audit-backed verification is a good second version, not a blocker.

**Shape that falls out of this:**
- A `setup_status(session)` service, read-only, same risk tier as `diagnose` (in fact
  built from one call to it): walks a small, ordered table of setup steps
  (orientation → calibration → radio → flight modes → failsafes/logging/GPS/arming),
  maps each to a `Finding.check` id where one exists, and reports `done` / `needed` /
  `recommended` / `unknown` / `not_applicable` accordingly. Only ArduPilot has the
  rule depth for this; the generic adapter reports everything `not_applicable`.
- **CLI `mavforge setup` is a dashboard, not an orchestrator.** It shows the ordered
  checklist and, for anything not `done`, the exact command to run next
  (`mavforge calibrate orientation`, etc.) — it does not itself drive six different
  guided workflows through one invocation. Building an auto-walk-through would mean
  reimplementing each workflow's confirmation/approval flow control inside `setup`
  itself; a dashboard that points at the real commands is far less to get wrong, and
  every one of those commands already has its own tests and approval path.
- **MCP `setup_status`**: read-only, same tier as `diagnose` — an AI assistant gets the
  checklist and the vehicle's own knowledge of what it needs, then drives the
  conversation using the *existing* tools (gyro/level/compass over MCP; radio,
  orientation, flight-modes and accel still point the person at the CLI, exactly as
  `calibration_request` already explains for each).

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
- **Servo output ranges.** `AP_Arming::servo_checks()` (`AP_Arming.cpp`, tag `Plane-4.6.2`) only examines a channel whose `SERVOn_FUNCTION` is assigned (`get_function() > k_none`, i.e. not 0) — an unassigned output isn't driving anything, so the firmware never looks at its MIN/MAX/TRIM either. For each assigned channel it rejects `SERVOn_MIN > SERVOn_TRIM` ("SERVOn_MIN is greater than SERVOn_TRIM") and `SERVOn_MAX < SERVOn_TRIM` ("SERVOn_MAX is less than SERVOn_TRIM") — the exact same shape as the existing RC-channel rule, just for outputs instead of inputs, and with "function assigned" standing in for RC's "live signal" gate (both are ways to skip a channel the firmware itself doesn't care about). Channels checked: whichever `SERVOn_FUNCTION` (n=1..16) are actually present on the vehicle; a vehicle with none present is `NOT_CHECKED`, not silently `PASSED`. Not vehicle-family-gated: `SRV_Channel` is a universal ArduPilot concept.
  - **Rejected sub-check: "no SERVOx_FUNCTION set to MotorX".** The official Plane pre-arm docs list this (at least one expected motor output unconfigured), but it needs knowing how many motor outputs a given `FRAME_CLASS`/`FRAME_TYPE` requires — board/frame-specific knowledge this rule doesn't have and wasn't worth guessing at for this pass.
- **Battery failsafe threshold ordering.** `AP_BattMonitor_Backend::arming_checks()` (`libraries/AP_BattMonitor/AP_BattMonitor_Backend.cpp`, tag `Plane-4.6.2`) refuses to arm when both of a pair of thresholds are set (non-zero) and the low one isn't strictly above the critical one: `fs_voltage_inversion` (`BATT_LOW_VOLT`/`BATT_CRT_VOLT`) and `fs_capacity_inversion` (`BATT_LOW_MAH`/`BATT_CRT_MAH`), messages `"voltage failsafe critical >= low"` / `"capacity failsafe critical >= low"`. This check is independent of whether `BATT_FS_LOW_ACT`/`BATT_FS_CRT_ACT` are even configured — pure parameter-value comparison — so the rule checks it before (and regardless of) the existing action/threshold findings. Classified FAULT: this is a configuration the firmware's own `arming_checks()` actually rejects.
- **Airspeed (Plane only).** `AP_Arming::airspeed_checks()` (`libraries/AP_Arming/AP_Arming.cpp`, tag `Plane-4.6.2`), gated by `ARMING_CHECK` bit 9: for each sensor, only rejects arming on `enabled(i) && use(i) && !healthy(i)` ("Airspeed %d not healthy"). A sensor that isn't `enabled` (`ARSPD_TYPE=0`) is skipped entirely, so `ARSPD_USE=1` with no sensor configured is *not* caught by this check — ArduPilot then substitutes an estimated airspeed as if it were a real reading for speed-controlled modes, which the [Airspeed Parameters Setup](https://ardupilot.org/plane/docs/airspeed-parameters-setup.html) docs warn "can result in erratic, or even dangerous, operation." The rule flags exactly that gap. A separate `AIRSPEED_MIN too low` check exists in `ArduPlane/AP_Arming.cpp` against an internal `MIN_AIRSPEED_MIN` constant, not gated by any `ARMING_CHECK` bit; its exact value wasn't found in the fetched source, so it was deliberately left unimplemented rather than guessed.
- **Pre-arm reasons.** `MAV_CMD_RUN_PREARM_CHECKS` returns ACCEPTED on SITL and real hardware, then STATUSTEXT `"PreArm: …"`. ArduPilot refuses it while armed.
  - Right after boot, SITL reports transient failures that cleared within about 15 s: `Accels inconsistent`, `EKF attitude is bad`, `AHRS: EKF3 not started`, `Compass 1 not healthy`, `Battery 1 unhealthy`.
  - For this reason MAVForge warns when `SYSTEM_TIME.time_boot_ms` is under 60 s.
- **Rejected rule: "duplicate serial-port protocol".** `AP_Arming::serial_protocol_checks()`
  (`libraries/AP_Arming/AP_Arming.cpp`, tag `Plane-4.6.2`) rejects arming when a second
  `SERIALn_PROTOCOL=23` (RCIN) port exists (`have_serial(SerialProtocol_RCIN, 1)`),
  unconditionally — `AP_SerialManager::have_serial()`/`find_protocol_instance()` has no
  exception anywhere in it. But the official
  [Multiple Radio Control Receivers](https://ardupilot.org/copter/docs/common-multiple-rx.html)
  page documents configuring a *second* receiver with that exact same value (`SERIALx_PROTOCOL=23`),
  gated by `RC_OPTIONS` bit 10, on boards with an IOMCU co-processor. Couldn't resolve
  from the fetched source whether the IOMCU path bypasses the main `AP_SerialManager`
  (plausible: the IOMCU has its own co-processor and own serial handling) or whether
  this is a genuine firmware inconsistency. Implementing "two ports = RCIN is a fault"
  without resolving that risks a false positive on a legitimate, documented setup, so
  it was left out rather than guessed at.
- **Rejected rule: "value outside documented range".**
  - On the stock ArduPlane 4.6.2 configuration it flagged 17 parameters. Nearly all were firmware defaults where 0 means "disabled" but falls outside the documented range (e.g. `AIRSPEED_STALL`, `FENCE_TOTAL`), or board-specific pin numbers.
  - That is too noisy for diagnostics. Range validation belongs at write time instead (planned).

## MAVFTP bulk parameter download

- **`FILE_TRANSFER_PROTOCOL` message and opcodes.** Per
  [mavlink.io](https://mavlink.io/en/services/ftp.html): 12-byte header (`seq_number`
  uint16, `session`, `opcode`, `size`, `req_opcode`, `burst_complete`, one padding
  byte, `offset` uint32), up to 239 bytes of `data`. Opcodes used here: `OpenFileRO`=4,
  `ReadFile`=5, `TerminateSession`=1, `ACK`=128, `NAK`=129 (`data[0]` is the error code;
  `EOF`=6, `Fail`=1, `FileNotFound`=10, `InvalidSession`=4, `UnknownCommand`=7).
  MAVForge implements sequential reads only, not burst reads: simpler and more
  reliable to get right, at the cost of one round trip per 239-byte chunk instead of
  pipelining - still far fewer round trips than one `PARAM_VALUE` per parameter.
- **Sequence numbers: the spec and real ArduPilot disagree.** mavlink.io states "the
  server replies with the same number as the request." **This is wrong for
  ArduPilot.** `GCS_FTP.cpp::setup_reply()` (checked at tag `Copter-4.7.2`):
  `reply.seq_number = request.seq_number + 1`. Found by writing the client strictly
  to the documented spec, which then timed out against real ArduCopter 4.7.2 SITL
  with *no visible error* - a raw pymavlink probe showed the ACK arriving correctly,
  just with `seq = request_seq + 1`, silently discarded by the client's own
  same-seq-only filter. The fake vehicle's FTP test double was built on the same wrong
  assumption and had to be fixed too, since it "agreed with itself" either way - this
  is exactly why the SITL layer of evidence exists, not just the fake vehicle.
  [verified against real ArduCopter 4.7.2 SITL]
- **`@PARAM/param.pck` binary format.** Per
  [mavlink.io](https://mavlink.io/en/services/parameter_ftp.html): 6-byte header
  (`magic` `0x671B`/no-defaults or `0x671C`/with-defaults, `num_params`,
  `total_params`, all little-endian uint16), then entries: byte 0 low nibble = type
  (1=INT8, 2=INT16, 3=INT32, 4=REAL32 - ArduPilot's internal codes, not
  `MAV_PARAM_TYPE`), high nibble = flags (bit 0: a default value follows); byte 1 low
  nibble = `common_len` (characters shared with the previous name), high nibble =
  `name_len-1`; then the non-shared name suffix, then the value, then optionally the
  default. `common_len + name_len` must not exceed 16. A zero first byte is block
  padding (inserted so a value doesn't straddle a 239-byte transfer chunk) and must be
  skipped one byte at a time. Unlike `PARAM_VALUE`'s "C cast" float encoding, INT32
  entries here are genuine binary `int32_t` - no precision loss for large integers.
  MAVForge only decodes (`protocol/param_pack.py`); encoding (needed only to upload, a
  deliberately out-of-scope direction - see roadmap) was not built. [verified against
  real ArduCopter 4.7.2 SITL: `param.pck` decodes to exactly the same name/value set
  as `PARAM_REQUEST_LIST`, modulo parameters the ArduPilot adapter already treats as
  volatile]
- **Fallback, not a replacement.** `services/backup.py::fetch_all_params()` attempts
  MAVFTP only when `AUTOPILOT_VERSION.capabilities` has the FTP bit (already surfaced
  as `"FTP" in identity.protocol_capabilities`), and falls back to the existing
  `PARAM_REQUEST_LIST` path (`protocol/params.py::fetch_all()`) on any FTP-specific
  failure. Confirmed against real SITL that the fallback is *not* taken when FTP
  works (a monkeypatch makes the fallback explode if it's ever called) - this is the
  thing that was actually broken until the seq_number fix above, now confirmed fixed.
- **Writes deliberately excluded.** `write_param()`'s PARAM_SET-with-read-back
  verification is core to this project's safety model; MAVFTP's `WriteFile` path has
  no equivalent per-parameter verification, so it was not built.

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
- **Compass calibration on hardware** (ArduPlane 4.6.2, USB power only, flight battery not connected, rotated by hand, 2026-10-23). [verified]
  - Progress was polled through `RUNNING_STEP_ONE` → `RUNNING_STEP_TWO` → `SUCCESS`.
  - One compass, fitness 11.6 against the default `COMPASS_CAL_FIT` of 16.
  - Offsets 406.8 / −213.3 / 90.7 (length 468) were accepted and verified by read-back. That is within the default `COMPASS_OFFS_MAX` of 1800 (`AP_COMPASS_OFFSETS_MAX_DEFAULT`).
  - `diagnose` now mirrors the firmware's "Compass offsets too high" pre-arm check: offset length greater than `COMPASS_OFFS_MAX`.
- **Radio calibration in SITL** (ArduCopter 4.7.2, 2026-10-23). [verified]
  - `MAV_CMD_REQUEST_MESSAGE(RC_CHANNELS)` is answered, so MAVForge can poll at 10 Hz without changing stream rates.
  - `RC_CHANNELS_OVERRIDE` from the test harness (source system 255) shows up in `RC_CHANNELS`. The sweep captured the overridden PWM values exactly.
  - The resulting plan was applied, verified by read-back, and restored.
- **Diagnose right after a reboot** (ArduPlane 4.6.2, USB, 2026-10-23). [verified]
  - The heartbeat reported mode `INITIALISING`. `SYS_STATUS`, `GPS_RAW_INT` and `SYSTEM_TIME` requests went unanswered.
  - The pre-arm reasons were start-up states: `EKF3 not started`, `waiting for home`, `Baro`/`Compass not healthy`, `INITIALISING mode not armable`.
  - About a minute later, the same check showed only bench conditions (GPS bad fix, radio failsafe → RTL) and `EKF3 Yaw inconsistent 119 deg. Wait`.
  - `diagnose` now warns "Vehicle is still starting up" whenever the heartbeat says so, even when uptime is not available.

## Board orientation (`AHRS_ORIENTATION`)

All of this is from ArduPilot source at tag `Plane-4.6.2`, unless marked otherwise.

- **Where the rotation is applied.** `AP_InertialSensor_Backend::_rotate_and_correct_accel()`:
  1. rotates by the per-sensor orientation;
  2. applies offsets and scale, in the sensor frame;
  3. rotates by `_board_orientation` ("rotate to body frame").

  The gyro path is the same. So the accelerometer calibration stays valid when
  `AHRS_ORIENTATION` changes, and the level trim (`AHRS_TRIM_*`, measured on body axes)
  doesn't.
- **What `SCALED_IMU` reports.** `GCS_MAVLINK::send_scaled_imu()` sends
  `ins.get_accel(instance)`, the corrected body-frame vector, in milli-g. `RAW_IMU` uses
  the same values. `MAV_CMD_REQUEST_MESSAGE` maps 26 to `MSG_SCALED_IMU`. [verified in
  SITL]
- **Rotation semantics.** `Vector3::rotate()` in `AP_Math/vector3.cpp` maps board
  vectors to body vectors. Each standard rotation equals `Rz(yaw) @ Ry(pitch) @ Rx(roll)`
  with the angles in its enum name, e.g. `ROTATION_YAW_90` is `(x, y, z) -> (-y, x, z)`.
  So with Yaw90, the board's X axis (its arrow) points to the vehicle's right.
  `tests/unit/test_orientation_math.py` checks MAVForge's matrices against the
  transcribed `rotate()` cases. [verified]
- **Labels.** The parameter metadata's labels list rotations in the opposite order to
  the enum names ("Yaw90Roll180" for `ROTATION_ROLL_180_YAW_90`), and its description
  says the label gives "the order of rotations". The matrices are identical. MAVForge
  displays the metadata labels.
  - Value 38's label reads "Yaw293Pitch68Roll180", but the code comment and matrix say
    roll 90. MAVForge uses the matrix and never proposes 38.
  - Value 41 (Pitch7) isn't listed in Plane's metadata.
- **When it takes effect.** The parameter description says "This option takes affect
  on next boot. After changing you will need to re-level your vehicle." ArduCopter
  4.7.2 SITL applied a change without a reboot. MAVForge still tells the person to
  reboot, as documented. [verified in SITL]
- **SITL.** `AP_InertialSensor_SITL` rotates the simulated samples by `SIM_IMU_ORIENT`
  before ArduPilot applies `AHRS_ORIENTATION`. The correct setting is therefore the
  inverse: `SIM_IMU_ORIENT=2` (Yaw90) needs `AHRS_ORIENTATION=6` (Yaw270).
  - The SITL test checks this, applies 6, and re-runs the check, which then reports a
    match. [verified]
- **Ground truth on hardware.** On the ArduPlane 4.6.2 bench board, `AHRS_ORIENTATION=0`
  was confirmed by hand on 2026-10-23: the arrow points at the nose, and nose up and
  right wing down read correctly in `ATTITUDE`.
- **First hardware run (2026-10-23, hand-held).** Result: 0 matches.
  - The first pose read 8° nose down and 4° left side down. In the first version, that
    went straight into a single "10° from the standard rotation" figure, and a warning
    blamed the board.
  - Split as the check now does, it was a 4.6° heading error plus a 9.4° first-pose
    tilt.
  - The readings are a unit test (`test_hand_held_bench_run_on_arduplane`).
  - The live tilt line also printed on every crossing of a 5° rounding boundary (dozens
    of lines while holding still). It now prints on a 5° change from the last line.
