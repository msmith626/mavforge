# Security review

## 2026-10-26: re-verification against the risk register, plus M8/M9 new-code audit

Scope: re-verify all 17 risks in [risks.md](risks.md) against current code (not just
re-read the register's own claims), and specifically scrutinize the three substantial
pieces added this session — M8a (flight-mode assignment), M8b (`propose_fixes`), M9
(`setup_status`) — for anything that bypasses the existing safety architecture.

**Note on methodology:** this was launched via the project's `security-review` skill,
but its auto-gathered context came back empty (clean working tree, nothing to diff)
and its built-in checklist (SQLi, XSS, JWT, crypto, deserialization) targets a generic
web-app threat model that doesn't map onto MAVForge — a local CLI/MCP tool with no web
surface, no SQL, no HTML rendering, no auth tokens. Running that checklist against an
empty diff would have produced nothing useful. This review instead did what the
roadmap item actually asked for, against the register MAVForge itself maintains.

### Re-verification of the 17 tracked risks

All 17 mitigations were checked against current code. Risks #2–#4, #7, #11, #12, #16,
#17 are the ones M8/M9 code could plausibly have weakened, since they touch vehicle
targeting, writes, calibration, and human approval — these got the closest look:

| # | Risk | Still holds? | What was checked |
|---|---|---|---|
| 2 | Wrong vehicle on a multi-vehicle link | **Yes** | `plan_changes()` binds every `ChangePlan.vehicle` to `session.identity` uniformly; `FlightModeWorkflow`/`_propose_fix` call the same function, no parallel path |
| 3 | Agent approves its own change | **Yes** | `propose_fixes` and `start_flight_modes` both only ever reach the *plan* stage (`Permission.READ`); `params_apply` independently requires `Permission.CONFIGURE` **and** `plan_store.valid_approval()` — confirmed by reading `apply_plan()` and `catalog.py` directly, not assumed |
| 4 | Link lost mid-write; batches not atomic | **Yes** | New code doesn't add a write path — `propose_fixes` builds a plan via the same `plan_changes()`/`apply_plan()` machinery as every other change |
| 7 | Calibration reported successful when it wasn't | **Yes** | `FlightModeWorkflow` requires a real reached-band per `FLTMODEn` (or an explicit warning when not reached) before proposing a plan, same evidence-first pattern as radio/orientation |
| 11 | Hardware tests changing a real vehicle | **Yes** | This session's manual hardware writes (`FLTMODE_CH`, `BATT_FS_*`, `LOG_BACKEND_TYPE`) all took an automatic pre-change backup and were explicitly consented per-command, same as the existing manual-write precedent |
| 12 | MAVForge masking a GCS failsafe | **Yes** | No new code sends a HEARTBEAT; `FlightModeWorkflow`/`_propose_fix` only read/request existing messages |
| 16 | Agent confirming a physical step that never happened | **Yes** | `FlightModeWorkflow.via_mcp = False`; confirmed enforced at the **service layer** (`_check_runnable`: `if session.interface == "mcp" and not wf.via_mcp: raise`), not just the CLI's refusal dict — defense in depth, same as radio/orientation |
| 17 | Cancel claims that don't hold | **Yes** (unaffected) | Not touched by new code |

The remaining risks (#1, #5, #6, #8–#10, #13–#15) are infrastructure-level (dialect
state, float encoding, metadata versioning, MCP transport, licensing, stdout hygiene,
XML parsing) and nothing in M8/M9 touches those surfaces at all.

### New code, specifically: does anything bypass the existing gates?

Checked by reading the actual code, not by re-describing the design intent:

- **`diagnose(propose_fixes=True)`** (`DIAGNOSE`/`SETUP_STATUS` are both
  `Permission.READ` in `catalog.py`) only calls `plan_changes()` — which itself
  requires nothing beyond `Permission.READ` (`PARAMS_PLAN` is `Permission.READ` too)
  and never writes to the vehicle. Writing requires a separate `params_apply` call,
  gated at `Permission.CONFIGURE` plus a real approval. An MCP agent with only the
  default read-only policy cannot turn `propose_fixes` into an actual write.
- **MCP tool registrations** for `diagnose` and `setup_status` both carry
  `annotations=READ_ONLY` — confirmed directly in `mcp/server.py`, not assumed from
  the docstrings.
- **No new dangerous constructs**: grepped `setup.py`, `flight_modes.py`, the
  `rules.py` additions, `diagnostics.py`, `cli/app.py` and `mcp/server.py` for
  `eval`/`exec`/`subprocess`/`pickle`/`yaml.load`/`os.system`/`shell=True` — none found.
- **CLI parsing** for `calibrate flight-modes FLTMODEn=VALUE` reuses the existing
  `parse_assignments()` (`float(raw)` only) verbatim — no new injection surface.
- **Information disclosure**: a `proposed_plan` only ever shows values a caller could
  already see via `diagnose`'s own evidence fields or a plain `params_get` —
  `propose_fixes` doesn't surface anything a read-only caller couldn't already read.

### Result

**No findings at HIGH or MEDIUM confidence.** The new work (M8a, M8b, M9) was built by
composing with the existing plan/approve/apply and calibration-request/approve/start
machinery rather than adding a parallel path, and that composition holds up under
direct inspection — not just by design intent. No new row is needed in
[risks.md](risks.md); all 17 existing entries remain accurately described by the
current code.
