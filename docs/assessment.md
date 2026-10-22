# Phase 1 assessment (October 2026)

## Starting point

`~/devel/mavforge` was empty when the project began: no code, history or conventions,
so everything here is greenfield. This document records why MAVForge exists, how it
relates to other tools, and what milestone M1 delivered. Research findings and their
sources are in [research.md](research.md).

## Positioning

Cells for other tools come from reading their source and documentation
([research.md](research.md)); "not assessed" means I didn't check.

| | MAVForge | MAVProxy | Mission Planner | MethodicConfigurator | Flight-focused MAVLink MCP servers |
|---|---|---|---|---|---|
| Primary form | Library + one-shot CLI + MCP | Interactive console GCS + router | Windows GUI GCS | Tkinter GUI wizard | MCP server |
| Typed Python API | yes | no (module API) | no (C#) | internal only | no |
| Machine-readable output and exit codes | yes | no | no | no | JSON over MCP |
| Parameter writes verified by value read-back | planned | name echo only (pymavlink `mavset`) | name echo only | not assessed | not assessed |
| Backups bound to vehicle and firmware identity | yes | no | no | not assessed | no |
| Plan → approve → apply, with audit log | planned (audit log done) | no | no | records a reason per change | `confirm` flags |
| Calibration as deterministic state machines with evidence | planned | relays prompts | UI-driven | no | no |
| AI agent access under a shared policy | yes (read-only so far) | no | no | no | yes, but flight-oriented |
| Missions, maps, flight control, arming | **no, by design** | yes | yes | no | yes |
| License | GPL-3.0-or-later | GPLv3 | GPLv3 | GPL-3.0-or-later | MIT or GPL-3.0 |

**Where tools overlap.** MAVForge overlaps with MAVProxy and Mission Planner on
parameter reads and writes and on triggering calibrations.

**What MAVForge does instead.** Its value is verification, auditability,
scriptability and safe agent access. It does not try to be an interactive ground
station.

**Running both.** MAVForge can share a link with MAVProxy: see [mavproxy.md](mavproxy.md).

## Capabilities after M1

**Done:**
- Typed domain models and outcomes. `OperationStatus` separates succeeded, partial,
  failed, timed-out, unknown, rejected and unsupported results.
- An error hierarchy that maps to CLI exit codes and to MCP tool errors.
- A connection-string validator that rejects pymavlink's file and exec forms.
- A single-reader MAVLink connection with a fan-out router.
- A command client with idempotent-only retries.
- A parameter read client with gap recovery.
- Vehicle selection and identification: AUTOPILOT_VERSION, git hash, SITL detection,
  and a multi-vehicle warning.
- An ArduPilot platform adapter and a generic read-only fallback.
- A central policy (read-only default) and a JSON-lines audit log.
- Python API, CLI and MCP server over the same services.
- Tests:
  - 86 default tests: unit tests, plus integration tests against a MAVLink-speaking
    fake vehicle (including parity tests).
  - 7 SITL tests against ArduCopter 4.7.2.
  - Opt-in hardware smoke tests, passed on one ArduPlane 4.6.2 flight controller over USB.

**Not done** (see [roadmap.md](roadmap.md)):
- Parameter writes and restore (backup and diff are done: milestone M2a).
- MAVFTP parameter download.
- Parameter metadata.
- Diagnostics.
- Calibration workflows.
- Approval flow.
- Reconnect-aware long operations.
- HTTP MCP transport.
- Hardware coverage beyond one board (ArduPlane 4.6.2, read-only, Python API).
- Any non-ArduPilot platform adapter.

## Minimum viable end-to-end use case (M1)

> An engineer or an agent connects to an ArduPilot vehicle, identifies it, sees its
> health, and inspects any parameter. Results are identical through the API, the CLI
> and MCP, and every run is audited.

Status: done and verified against ArduCopter 4.7.2 SITL.
