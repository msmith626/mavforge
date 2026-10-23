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
| 4. Calibration | State machines, in order: gyro → level/trim → 6-pose accel → onboard compass → radio. ESC deferred. Per-workflow spec docs; operation registry with cancel; human-step prompts (interactive CLI; MCP poll + elicitation) | Fake-vehicle tests for success / fail / cancel / timeout per state; SITL `-M calibration` tests for accel and compass |
| 5. Hardening | Reconnect during long operations, wrong-vehicle tests, security review, MCP authorization tests, localhost HTTP transport | Security checklist complete; parity tests cover every write path |
| 6. Release | API / CLI / MCP reference docs, compatibility matrix backed by test evidence, 0.1.0 | Fresh-clone quick start verified |

## Next milestone

Calibration (Phase 4), starting with the workflows that need no physical handling:
1. Gyro, with a state machine and evidence-based success.
2. Level/trim.
3. Onboard compass calibration: `MAG_CAL_PROGRESS`/`REPORT`, success only on
   `MAG_CAL_SUCCESS`, then a parameter read-back.
4. The 6-pose accelerometer calibration, with human prompts between poses.

Each workflow runs as an operation that can be cancelled, gets a spec document, and has
fake-vehicle and SITL `-M calibration` tests. Your plane's compass is the obvious first
hardware candidate.
