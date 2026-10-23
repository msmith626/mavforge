# Architecture

```
 Interfaces      mavforge.api (Vehicle)   mavforge.cli (Typer)   mavforge.mcp (MCPServer)
                         \                       |                       /
                          `---- all call the Python API; no MAVLink logic here ----'
 Services        services/{session,info,status,params}.py ── catalog.run_operation
                         |                         (policy check + audit record)
 Policy          policy/permissions.py (Policy, Permission)   policy/audit.py (AuditLog)
 Domain          domain/{models,outcomes,errors}.py   (Pydantic models = shared contract)
 Platforms       platforms/base.py (PlatformAdapter, select_adapter)
                 platforms/ardupilot/adapter.py   platforms/generic.py
 Protocol        protocol/commands.py   protocol/messages.py   protocol/params.py
 Transport       transport/connection.py (reader thread, send lock, caches)
                 transport/router.py (fan-out subscriptions)   transport/connstr.py
                 transport/dialect.py (the only pymavlink dialect setup)
 Test doubles    testing/fake_vehicle.py (real MAVLink bytes over localhost TCP)
```

## Key decisions

1. **One implementation, several interfaces.**
   - Every operation is a service function that runs through
     `services.catalog.run_operation`. That function checks the active `Policy` and
     writes an audit record.
   - The CLI and MCP server call the same `Vehicle` API that library users get.
   - Parity tests check that the API, CLI and MCP give identical results and audit
     records, and that policy denials apply to all of them.
2. **Synchronous core, single reader.**
   - pymavlink is neither thread-safe nor asynchronous, and `recv_match` drops
     messages.
   - Each `MavlinkConnection` therefore owns one reader thread, which hands messages to
     subscribers.
   - Callers subscribe *before* sending a request, so a fast reply cannot be missed.
   - MCP runs these synchronous tools in worker threads.
3. **Platform adapters instead of `if autopilot == ...` checks.**
   - Mode names, parameter-name normalisation, the simulator probe and declared
     capabilities all live in the adapter.
   - Unknown autopilots fall back to the generic read-only adapter.
4. **Honest outcomes.**
   - `OperationStatus` never treats "sent" as "succeeded".
   - Partial parameter downloads list the indices that never arrived.
   - A missing reply in a status snapshot is listed under `unavailable`.
   - A lost ACK on a non-idempotent command is `unknown`.
5. **Read-only requests.** Telemetry is fetched with `MAV_CMD_REQUEST_MESSAGE`, which
   leaves stream rates (`SRx_*`, message intervals) unchanged.
6. **No heartbeats; distinct source IDs.**
   - MAVForge never sends HEARTBEAT, so it cannot keep a vehicle's GCS failsafe from
     triggering.
   - It sends as system 255, component 25 (`MAV_COMP_ID_USER1`). That is distinct from
     MAVProxy (component 0) and Mission Planner/QGC (component 190), so ACKs addressed
     to MAVForge can be told apart.
   - Both IDs are configurable.
7. **The operator owns the link.** The MCP server takes its connection from the command
   line or environment. No tool accepts a connection string.

## Safety and permission model

- **Permission tiers.** `Permission` has four tiers:
  - `read` (the default).
  - `configure`: parameter changes.
  - `calibrate` and `maintenance`: reserved.

  There is deliberately no tier for arming or flight. `Policy.allow_real_vehicle`
  (default off) gates every modifying operation on vehicles not identified as SITL.
- **Parameter changes** (`services/changes.py`, `services/plan_store.py`):
  1. **Plan.** Reads current values, then validates type, integer bounds, and the
     metadata for the vehicle's exact firmware: range, documented values, read-only.
     Records adapter safety notes (disabling failsafes or arming checks, serial
     ports, output functions). Some changes are blocked outright: system ID and
     `FORMAT_VERSION`. The plan is saved immutably, with its id taken from the
     SHA-256 of its content.
  2. **Approve.** The approval names the plan's digest and lasts 10 minutes. It can only
     come from a human-facing path: the CLI prompt or `--yes`, `mavforge approve`, MCP
     elicitation (through a resolver parameter hidden from the agent), or a Python caller.
  3. **Apply** re-checks every gate below all interfaces, in this order:
     - the `configure` policy and the real-vehicle flag;
     - the plan is unblocked and has a valid approval;
     - the vehicle is the same: system and component ID, autopilot, firmware, board UID;
     - the vehicle is disarmed;
     - an `apply_started` audit record is written; if it can't be, nothing is written
       (fail closed).

     It then consumes the plan (an atomic exclusive file create makes it single-use)
     and backs up all parameters. Each value is checked for drift since planning (if it
     changed, that parameter is skipped), written with `PARAM_SET`, and verified by
     reading it back. A lost link marks the change in flight `unknown` and the rest
     `not_attempted`.
  4. **Restore** is a plan built from a backup's non-volatile differences. It is refused
     if the backup's identity or firmware doesn't match, unless explicitly overridden.
- **Calibration** (`services/calibration/`; details in [calibration.md](calibration.md)):
  - Each workflow is a state machine (`Workflow`), run by an `OperationRegistry` in a
    background thread.
  - It starts from an approved, single-use `CalibrationRequest`, under the same gates
    as parameter changes, plus one calibration at a time.
  - A `WorkflowContext` carries the person's `instruction`, an atomic step
    confirmation, cancellation, and live status.
  - Success requires the vehicle's terminal signal and a parameter read-back.
  - Workflows that need physical confirmations (accel, radio) are refused for
    MCP-initiated runs in the service layer.
  - A workflow with `writes_vehicle = False` (radio) starts without a request, needing
    only read permission (`OperationRegistry.start_observation`). It returns a
    `ChangePlan` in `result.plan`, and that plan needs the usual approval to apply.
- **Untrusted vehicle text.**
  - The CLI renders vehicle-originated text inertly: no rich markup, and control
    characters are replaced.
  - MCP marks this text as untrusted in its schemas and server instructions.
- **Audit log.**
  - Location: `$MAVFORGE_STATE_DIR` or `$XDG_STATE_HOME/mavforge/audit.jsonl`.
  - The file is created with mode 0600 and credentials are redacted.
  - Audit failures before a write block the write (fail closed). After the writes, an
    audit failure never hides the outcome: it is logged and added to the result as a
    warning.

## Extending

**Adding an operation:**
1. Add an `OperationSpec` to `services/catalog.py`.
2. Implement a service function that calls `run_operation`.
3. Expose it on `Vehicle`.
4. Add a CLI command and an MCP tool that call the API.
5. Extend `tests/integration/test_parity.py`.

**Adding a platform:** subclass `PlatformAdapter`, register it in
`platforms/base.select_adapter`, and declare only the capabilities you have tested.
