# Roadmap

Each phase ends green: `ruff`, `pyright --strict`, `pytest`, and `pytest -m sitl`.
Capabilities are claimed only once they are tested.

| Phase | Deliverables | Exit criteria |
|---|---|---|
| **M1 (done)** | Scaffold; transport and router; fake vehicle; identify, status, params list/get through API, CLI and MCP; audit; policy; Docker SITL | Unit, integration and parity tests pass; SITL tests pass against ArduCopter 4.7.2 |
| **M2a (done)** | Parameter backup (JSON with identity, owner-only, never overwritten, optional `.param` export); diff against live vehicle, backup or plain `.param`; offline diff; float32-aware equality; volatile flagging; through API, CLI and MCP | Unit, fake-vehicle, parity and SITL tests |
| 2. Params and config | MAVFTP `param.pck` download; versioned `apm.pdef` metadata cache; diff vs firmware defaults; plan → approve → apply with read-back verification and per-parameter partial-failure reporting; restore with compatibility gate; `mavforge approve` | Fault-injection tests (lost echo, PARAM_ERROR, link loss mid-batch) give correct `partial` / `unknown`; SITL backup → modify → restore round trip |
| 3. Diagnostics | `RUN_PREARM_CHECKS` + STATUSTEXT window; sensor, GPS, battery, RC, compass and accel status; parameter consistency rules; baseline diff; findings classified as confirmed / warning / suspected / uncheckable; airworthiness disclaimer | Rule tests on fixtures; SITL with induced faults (SIM_* params) produces the expected findings |
| 4. Calibration | State machines, in order: gyro → level/trim → 6-pose accel → onboard compass → radio. ESC deferred. Per-workflow spec docs; operation registry with cancel; human-step prompts (interactive CLI; MCP poll + elicitation) | Fake-vehicle tests for success / fail / cancel / timeout per state; SITL `-M calibration` tests for accel and compass |
| 5. Hardening | Reconnect during long operations, wrong-vehicle tests, security review, MCP authorization tests, localhost HTTP transport | Security checklist complete; parity tests cover every write path |
| 6. Release | API / CLI / MCP reference docs, compatibility matrix backed by test evidence, 0.1.0 | Fresh-clone quick start verified |

## Next milestone

Diagnostics (Phase 3) before writes. An unconfigured airframe has many problems, and a
classified findings report is the most useful read-only tool for working through them.
That report covers pre-arm reasons, RC/GPS/compass/accel calibration state, and failsafe
configuration. Parameter writes (plan → approve → apply) follow.
