# Roadmap

Each phase ends green: `ruff`, `pyright --strict`, `pytest`, and `pytest -m sitl`.
Capabilities are claimed only once they are tested.

Status as of 2026-10-23.

## Done

| Milestone | Deliverables | Evidence |
|---|---|---|
| **M1** | Scaffold; transport and router; fake vehicle; identify, status, params list/get through API, CLI and MCP; audit; policy; Docker SITL | Unit, integration and parity tests; SITL against ArduCopter 4.7.2; read-only hardware tests on ArduPlane 4.6.2 |
| **M2a** | Parameter backup (JSON with identity, owner-only, never overwritten, optional `.param` export); diff against live vehicle, backup or plain `.param`; offline diff; float32-aware equality; volatile flagging; API, CLI and MCP | Unit, fake-vehicle, parity and SITL tests |
| **M3** | `diagnose`: vehicle pre-arm reasons (`RUN_PREARM_CHECKS`), SYS_STATUS sensor health, recent-boot warning, ArduPilot rules (arming checks, accel/compass/RC calibration, compass offsets, throttle and battery failsafes, GPS, logging, frame class) with certainty classes; API, CLI (`--fail-on`) and MCP | Unit rule tests, fake-vehicle scenarios including a bench plane, parity, SITL; ArduPlane 4.6.2 hardware |
| **M4** | Parameter changes: plan (type, bounds, versioned `apm.pdef` metadata cache, safety notes) → human approval (CLI prompt / `mavforge approve` / MCP elicitation; digest-bound, expiring, single-use) → apply (policy, identity, disarmed, fail-closed audit, pre-change backup, drift skip, write + read-back) → per-parameter outcomes; restore from backup with identity gate; MCP write tools behind `--allow-configure` / `--allow-real-vehicle` | Fault-injection tests (rejected, clamped, ignored writes, link loss, drift, tamper, expiry, forgery); MCP authorization tests; parity; SITL write + restore; write + restore on ArduPlane 4.6.2 hardware |
| **M5** | Calibration: gyro, level, onboard compass, 6-position accel as state machines (`services/calibration/`); approved single-use requests; background operations with status/confirm/cancel; evidence-based success; CLI live guidance; MCP behind `--allow-calibrate` (accel CLI/API only) | Fake-vehicle success/failure/cancel/timeout tests, MCP authorization tests, parity, all four on ArduCopter 4.7.2 SITL; compass, gyro and level on ArduPlane 4.6.2 hardware |
| **M6** | Radio (RC) calibration: guided `RC_CHANNELS` sweep (10 Hz `REQUEST_MESSAGE`, per-channel range gate, 200 µs travel threshold), centred/throttle-low checks via `RCMAP_*` and `RCn_REVERSED`, 3 µs tolerance against current values; result is a `ChangePlan` applied through the normal approval; sweep needs read permission only; CLI live guidance, `--plan-only`; not via MCP | Fake-vehicle and CLI tests, MCP refusal, ArduCopter 4.7.2 SITL sweep → apply → restore; ArduPlane 4.6.2 hardware with a Spektrum DX7s |
| **M7** | Board orientation check: three guided poses (level, nose up, right side down) from `SCALED_IMU`, live tilt as the autopilot sees it, retries with the reason, solve for the true mounting and match ArduPilot's rotations (labels from its metadata), heading and first-pose tilt judged separately; result is a `ChangePlan` for `AHRS_ORIENTATION`; read permission only; not via MCP | Rotation matrices checked against ArduPilot's `Vector3::rotate`; fake-vehicle and CLI tests; MCP refusal; ArduCopter 4.7.2 SITL with `SIM_IMU_ORIENT=2` (proposes 6, applied, re-check matches); hand-held run on ArduPlane 4.6.2 hardware |

## Remaining

Roughly in order of value. Each item keeps the same test levels: unit, fake vehicle,
parity where an interface is involved, SITL, and hardware where it matters.

### M8: Flight modes and failsafes
The natural next step after M5–M7: every sensor and radio prerequisite is now
calibrated, which is where ArduPilot's own setup sequence turns to flight modes and
failsafes. Scoped in detail in [research.md](research.md#flight-modes-and-failsafes-planned-behaviour-for-m8)
(exact `RC_Channel::read_6pos_switch` PWM breakpoints and `FS_*`/`BATT_FS_*` value
metadata, both pulled from ArduPilot source and the firmware's own `apm.pdef.xml`, not
guessed). **This is two architecturally different pieces of work, not one feature:**

- **M8a — Guided flight-mode assignment.** A new guided `Workflow`
  (`CalibrationKind.FLIGHT_MODES`), the same shape as radio calibration (M6): live
  `RC_CHANNELS` monitoring of `FLTMODE_CH` to show the PWM and which of the six fixed
  bands it falls in as the person flips the switch, they pick a mode for each of
  `FLTMODE1`–`FLTMODE6`, result is a `ChangePlan` through the normal approval. Never
  switches modes live — that stays flight control, out of scope. Not via MCP, same
  reasoning as M6/M7: which mode belongs on which switch position is the pilot's call.
- **M8b — Failsafe review.** Not a guided workflow — there's no physical action
  involved, so this is a `diagnose` extension (`services/diagnostics.py` /
  `platforms/ardupilot/rules.py`), reading `FS_SHORT_ACTN`, `FS_LONG_ACTN`,
  `FS_GCS_ENABL`, and battery failsafe (`BATT_FS_LOW_ACT`/`_CRT_ACT` together with
  `BATT_LOW_VOLT`/`BATT_CRT_VOLT` — this session's bench vehicle is the motivating
  case: an action configured with its threshold left at the default `0`, so neither
  could ever trigger). Needs one schema change the existing rules don't have: a
  `proposed_plan` field on `Finding`, built with the already-independent
  `services.changes.plan_changes()`. This is the first real use of the existing
  Diagnostics item "findings that propose a parameter plan," not a separate thing.
  Whether this one is MCP-available is an open question worth a deliberate decision,
  not an assumption: unlike M8a there's no live human action involved, so the usual
  M6/M7 exclusion reasoning doesn't automatically carry over — though MCP still could
  never approve the plan it proposes.
  - **Implemented** (2026-10-24): `Finding.proposed_changes`/`proposed_plan`,
    `diagnose(propose_fixes=...)` (off by default — building a plan re-reads live
    values and saves a plan file, a side effect a normally pure-read `diagnose` call
    shouldn't have unasked), the extended `_battery_failsafe` rule (catches exactly the
    action-without-threshold gap found by hand this session), and one demonstration
    case with a genuinely safe default (`LOG_BACKEND_TYPE`). Deliberately does **not**
    propose a specific battery voltage: that needs cell-count information MAVForge has
    no way to know, the same reasoning applied live, by a person, earlier this session.
    Exposed via MCP (the open question above, now answered): `propose_fixes` only ever
    builds a plan, never applies one, so it stays as read-only in effect as `diagnose`
    itself. CLI `--propose-fixes`; API and MCP parameter of the same name. Covered by
    unit (rules), fake-vehicle integration, CLI and MCP tests — **not yet** run against
    SITL or hardware, so not yet claimed as fully done by this project's own bar.

### M9: Guided vehicle setup ("wizard")
Ties M1–M8 together into one guided first-time setup, instead of requiring a person
(or an AI assistant) to already know ArduPilot's setup order and which steps apply to
their vehicle. This session is the motivating example: a first-time owner, walked
through orientation, calibration, radio, flight modes and failsafes one conversation
at a time, with the AI assistant supplying the sequence and the "why" from its own
general knowledge rather than from anything MAVForge itself knows or verifies.
- A setup-status view, derived from the vehicle's actual capabilities and current
  parameters (not a fixed checklist): which steps are done (with evidence, as
  calibration already records), which are needed, which don't apply (e.g. no airspeed
  sensor fitted), and which are optional-but-recommended (e.g. battery failsafe
  thresholds, terrain) — the battery-failsafe gap found by hand in this session
  (actions configured, thresholds left at 0, so neither could ever trigger) is exactly
  the shape of thing this view exists to catch without a person needing to think to
  check it.
- CLI: a `mavforge setup` that walks the sequence end to end, reusing every existing
  mechanism (the calibration state machines, the radio sweep, the orientation check,
  flight-mode/failsafe plans from M8) and skipping whatever the vehicle already has
  evidence for.
- MCP: the same status/sequence exposed as a tool, so an AI assistant drives the
  conversation from MAVForge's own knowledge of what this specific vehicle needs and
  in what order, with every actual change still going through the normal
  approve → apply → read-back path. The assistant becomes the conversational layer
  over a checklist MAVForge owns and can verify, rather than the sole source of what
  "setup" even means for this vehicle.
- Overlaps with the "Findings that propose a parameter plan" item under Diagnostics
  below: a rule that notices a configured failsafe action with no threshold (or any
  other finding with an obvious fix) and proposes the plan itself is a building block
  this wizard would call directly, rather than relying on a person or an AI to notice.
- Only ArduPilot/plane has the engineering depth for this today (the generic adapter
  is read-only); the checklist content, like calibration and radio, would need to be
  per-platform-family rather than universal.

### Release (6)
- First CI run on GitHub. `.github/workflows/ci.yml` runs checks on Python 3.12–3.14
  and a SITL job. It has never run on GitHub, and only Python 3.12 has been tested
  locally.
- API, CLI and MCP reference docs; a compatibility matrix backed by test evidence;
  CONTRIBUTING; version 0.1.0 (and possibly a PyPI release).
- A fresh-clone quick start, verified on a second machine.

### Hardening (5)
- Recover from link loss *during* a long operation (calibration). The MCP server
  already reconnects between calls, and link loss is detected and reported.
- Wrong-vehicle tests beyond the current ones (selection when several vehicles or GCS
  heartbeats are present).
- Security review against `docs/risks.md`.
- MCP over localhost HTTP (stdio only today).

### Diagnostics (3, remaining)
- More rules, each sourced from firmware or metadata:
  - servo and motor output functions;
  - airspeed setup for planes;
  - serial-port protocol conflicts;
  - failsafe thresholds against battery capacity.
- Optional comparison against a saved baseline inside the report.
- Findings that propose a parameter plan (never applied automatically).

### Parameters (2, remaining)
- MAVFTP `@PARAM/param.pck` bulk download, which is much faster than
  `PARAM_REQUEST_LIST`, with the current method as a fallback.
- Diff against firmware defaults (`param.pck?withdefaults=1`).

### Calibration (4, remaining)
- 6-position accelerometer calibration on hardware. It passes in SITL but hasn't been
  run on a real board.
- Large-vehicle MagCal (`MAV_CMD_FIXED_MAG_CAL_YAW`); barometer calibration.
- ESC calibration: deferred (high risk).

## Hardware evidence

Bench board: ArduPlane 4.6.2 over USB, unconfigured airframe, indoors (no GPS fix).

| Workflow | Result | Date |
|---|---|---|
| Read-only tests (info, status, params) | passed | 2026-10-21 |
| `diagnose` | runs; findings match bench conditions | 2026-10-21 |
| Parameter write + restore (harmless parameters) | passed | 2026-10-23 |
| Compass calibration | passed, USB power only | 2026-10-23 |
| Gyro, level calibration | passed | 2026-10-23 |
| Radio calibration (Spektrum DX7s, 7 channels) | passed; matched the existing calibration within 2 µs | 2026-10-23 |
| Board orientation check | passed; `AHRS_ORIENTATION=0` matches, confirmed by hand | 2026-10-23 |
| Wireless link (WiFi, onboard module) | passed; `info`, `status`, full 1117-param download, no gaps. Bluetooth (BLE and classic SPP) ruled out — see [docs/wireless.md](wireless.md) | 2026-10-23 |
| 6-position accelerometer calibration | not yet run | |
