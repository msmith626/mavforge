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
| 4. Calibration | State machines, in order: gyro → level/trim → 6-pose accel → onboard compass → radio. ESC deferred. Per-workflow spec docs; operation registry with cancel; human-step prompts (interactive CLI; MCP poll + elicitation) | Fake-vehicle tests for success / fail / cancel / timeout per state; SITL `-M calibration` tests for accel and compass |
| 5. Hardening | Reconnect during long operations, wrong-vehicle tests, security review, MCP authorization tests, localhost HTTP transport | Security checklist complete; parity tests cover every write path |
| 6. Release | API / CLI / MCP reference docs, compatibility matrix backed by test evidence, 0.1.0 | Fresh-clone quick start verified |

## Next milestone

Parameter changes (Phase 2 writes): plan, then approve, then apply, then verify.
- **Plan.** A plan lists the proposed values with the current value, the metadata range,
  and any reboot-required flag for each parameter.
- **Approve.** Approval comes from a human, through MCP elicitation or
  `mavforge approve`. It is bound to the plan's hash and to the vehicle's identity.
- **Apply and verify.** Each written value is read back. Partial application is reported
  per parameter.
- **Restore.** Restoring from a backup reuses the same path, with a compatibility check
  first.

Diagnostics findings will be able to propose plans, for example "set `ARMING_CHECK=1`".
They will never apply anything automatically.
