# Roadmap

Each phase ends green: `ruff`, `pyright --strict`, `pytest`, and `pytest -m sitl`.
Capabilities are claimed only once they are tested.

| Phase | Deliverables | Exit criteria |
|---|---|---|
| **M1 (done)** | Scaffold; transport and router; fake vehicle; identify, status, params list/get through API, CLI and MCP; audit; policy; Docker SITL | Unit, integration and parity tests pass; SITL tests pass against ArduCopter 4.7.2 |
| **M2a (done)** | Parameter backup (JSON with identity, owner-only, never overwritten, optional `.param` export); diff against live vehicle, backup or plain `.param`; offline diff; float32-aware equality; volatile flagging; through API, CLI and MCP | Unit, fake-vehicle, parity and SITL tests |
| 2. Params and config | MAVFTP `param.pck` download; versioned `apm.pdef` metadata cache; diff vs firmware defaults; plan → approve → apply with read-back verification and per-parameter partial-failure reporting; restore with compatibility gate; `mavforge approve` | Fault-injection tests (lost echo, PARAM_ERROR, link loss mid-batch) give correct `partial` / `unknown`; SITL backup → modify → restore round trip |
| **M3 (done)** | `diagnose`: vehicle pre-arm reasons (`RUN_PREARM_CHECKS`), SYS_STATUS sensor health, recent-boot warning, ArduPilot rules (arming checks, accel/compass/RC calibration, throttle and battery failsafes, GPS, logging, frame class) with certainty classes; API, CLI (`--fail-on`) and MCP | Unit rule tests, fake-vehicle scenarios incl. the bench plane, parity, SITL; run on ArduPlane 4.6.2 hardware |
| 3. Diagnostics (remaining) | More rules, each sourced from firmware or metadata: servo/motor output functions, airspeed setup for planes, serial-port protocol conflicts, failsafe thresholds vs battery capacity. Optional comparison against a saved baseline inside the report. Findings that propose a parameter plan (never auto-applied) | Rule unit tests; SITL with induced faults (SIM_* params) produces the expected findings |
| **M4 (done)** | Parameter changes: plan (type, bounds, versioned `apm.pdef` metadata, safety notes) → human approval (CLI prompt / `mavforge approve` / MCP elicitation; digest-bound, expiring, single-use) → apply (policy, identity, disarmed, fail-closed audit, pre-change backup, drift skip, write + read-back) → per-parameter outcomes; restore from backup with identity gate; MCP write tools behind `--allow-configure` / `--allow-real-vehicle` | Fault-injection tests (rejected, clamped, ignored writes, link loss, drift, tamper, expiry, forgery); parity; SITL write+restore; consented write+restore on ArduPlane 4.6.2 |
| **M5 (done)** | Calibration: gyro, level, onboard compass, 6-position accel as state machines (`services/calibration/`); approved single-use requests; background operations with status/confirm/cancel; evidence-based success; CLI live guidance; MCP behind `--allow-calibrate` (accel CLI/API only) | Fake-vehicle success/failure/cancel/timeout tests, MCP authorization tests, parity, all four passing on ArduCopter 4.7.2 SITL |
| **M6 (done)** | Radio (RC) calibration: guided `RC_CHANNELS` sweep (10 Hz `REQUEST_MESSAGE`, per-channel range gate, 200 µs travel threshold), centred/throttle-low checks via `RCMAP_*` and `RCn_REVERSED`, result is a `ChangePlan` applied through the normal approval; sweep needs read permission only; CLI live guidance, `--plan-only`; not via MCP | Fake-vehicle tests (success, apply after approval, unmoved, uncentred, throttle high, reversed, RCMAP, jitter, no receiver, cancel, armed, permissions), CLI tests, MCP refusal, ArduCopter 4.7.2 SITL sweep → apply → restore; hardware: ArduPlane 4.6.2 with a Spektrum DX7s (2026-10-23) |
| **M7 (done)** | Board orientation check: three guided poses (level, nose up, right side down) from `SCALED_IMU`, live tilt as the autopilot sees it, retries with the reason, solve for the true mounting and match ArduPilot's rotations, result is a `ChangePlan` for `AHRS_ORIENTATION`; read permission only; not via MCP | Rotation matrices checked against ArduPilot's `Vector3::rotate`; fake-vehicle tests (mountings, wrong current setting, retries, shaking, wrong side, not square, custom, no IMU, cancel); CLI test; MCP refusal; ArduCopter 4.7.2 SITL with `SIM_IMU_ORIENT=2`: proposes 6, applied, re-check matches |
| 4. Calibration (remaining) | Large-vehicle MagCal (`MAV_CMD_FIXED_MAG_CAL_YAW`); barometer; ESC deferred (high risk). Hardware validation of the existing workflows | Same test levels; hardware runs with the owner present |
| 5. Hardening | Reconnect during long operations, wrong-vehicle tests, security review, MCP authorization tests, localhost HTTP transport | Security checklist complete; parity tests cover every write path |
| 6. Release | API / CLI / MCP reference docs, compatibility matrix backed by test evidence, 0.1.0 | Fresh-clone quick start verified |

## Next milestone

Pick one of these:

- **Hardware validation** on your ArduPlane 4.6.2 board: gyro, level and radio calibration
  passed on 2026-10-23. Still to do: the orientation check (expected result: 0 matches,
  as confirmed by hand) and the 6-position accelerometer calibration.
- **Hardening:** reconnect during long operations, and the localhost HTTP MCP transport.
- **More diagnostics rules:** servo output functions, plane airspeed setup, serial
  protocol conflicts, failsafe thresholds against battery capacity.
