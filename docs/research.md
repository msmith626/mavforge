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
