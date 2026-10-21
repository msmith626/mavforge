# Roadmap

Each phase ends green: `ruff`, `pyright --strict`, `pytest`, and `pytest -m sitl`.
Capabilities are claimed only once they are tested.

| Phase | Deliverables | Exit criteria |
|---|---|---|
| **M1 (done)** | Scaffold; transport and router; fake vehicle; identify, status, params list/get through API, CLI and MCP; audit; policy; Docker SITL | Unit, integration and parity tests pass; SITL tests pass against ArduCopter 4.7.2 |
| 2. Params and config | MAVFTP `param.pck` download; versioned `apm.pdef` metadata cache; backup (params + identity + firmware, JSON and `.param`); diff vs backup and vs defaults; plan → approve → apply with read-back verification and per-parameter partial-failure reporting; restore with compatibility gate; `mavforge approve` | Fault-injection tests (lost echo, PARAM_ERROR, link loss mid-batch) give correct `partial` / `unknown`; SITL backup → modify → restore round trip |
| 3. Diagnostics | `RUN_PREARM_CHECKS` + STATUSTEXT window; sensor, GPS, battery, RC, compass and accel status; parameter consistency rules; baseline diff; findings classified as confirmed / warning / suspected / uncheckable; airworthiness disclaimer | Rule tests on fixtures; SITL with induced faults (SIM_* params) produces the expected findings |
| 4. Calibration | State machines, in order: gyro → level/trim → 6-pose accel → onboard compass → radio. ESC deferred. Per-workflow spec docs; operation registry with cancel; human-step prompts (interactive CLI; MCP poll + elicitation) | Fake-vehicle tests for success / fail / cancel / timeout per state; SITL `-M calibration` tests for accel and compass |
| 5. Hardening | Reconnect during long operations, wrong-vehicle tests, security review, MCP authorization tests, localhost HTTP transport | Security checklist complete; parity tests cover every write path |
| 6. Release | API / CLI / MCP reference docs, compatibility matrix backed by test evidence, 0.1.0 | Fresh-clone quick start verified |

## Next milestone (M2a): parameter backup and diff

This is read-only work that builds on M1 and sets up safe writes:

1. `vehicle.params.backup(path)`: a JSON document with schema version, vehicle
   identity, firmware, timestamp and all parameters. It also writes a Mission
   Planner / MAVProxy-compatible `.param` file.
2. `vehicle.params.diff(backup)`: classifies each parameter as changed, missing on the
   vehicle, or new on the vehicle, and warns on identity or firmware mismatch.
3. The CLI and MCP equivalents, plus parity tests.
4. A SITL test: back up, diff with no changes, then diff against a doctored backup.
