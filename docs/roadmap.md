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
| 6-position accelerometer calibration | not yet run | |
