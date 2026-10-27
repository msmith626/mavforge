# Roadmap

Each phase ends green: `ruff`, `pyright --strict`, `pytest`, and `pytest -m sitl`.
Capabilities are claimed only once they are tested.

Status as of 2026-10-26.

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
  - **Implemented** (2026-10-24): `services/calibration/flight_modes.py`
    (`FlightModeWorkflow`), a single guided sweep step that discretises `FLTMODE_CH`'s
    live PWM into the six fixed bands (breakpoints confirmed directly against
    `RC_Channel::read_6pos_switch`) and checks whether each requested `FLTMODEn`'s band
    was actually reached. The person supplies the target values up front
    (`FLTMODEn=VALUE`, the same syntax as `params set`) — MAVForge never picks a mode
    for a position, consistent with the "pilot's judgment call" reasoning above. CLI
    `calibrate flight-modes FLTMODE1=0 FLTMODE3=5 ...`; Python
    `vehicle.calibration.start_flight_modes({...})`. Covered by unit tests (the exact
    breakpoints, including contiguity across the full PWM range) and fake-vehicle
    integration tests (plan proposed, applied after approval, a requested-but-unreached
    band flagged as a warning not a failure, disabled mode channel, unknown parameter
    name, nothing requested, MCP refusal). **SITL** (2026-10-24): a new
    `test_sitl_flight_modes.py` drives the real mode channel (`FLTMODE_CH=5` on this
    image) via `RC_CHANNELS_OVERRIDE` — same mechanism as the existing radio SITL test —
    sweeps three positions, confirms the proposed plan matches and applies cleanly
    against real ArduCopter 4.7.2 firmware. **Hardware** (2026-10-26): run on the
    ArduPlane 4.6.2 bench board. Found two real things, not simulated ones:
    - A genuine CLI bug — the live-progress renderer only knew about `rc_channels`
      (radio) and `tilt` (orientation); `mode_bands` was tracked internally the whole
      time but never displayed. Fixed, and confirmed live on hardware afterward (the
      "reached: ..." line updating three times as the switch was flipped).
    - A real misconfiguration on the vehicle itself: `FLTMODE_CH` was `8`, a channel
      its Spektrum DX7s doesn't drive at all (confirmed separately with a live RC
      monitor and a full `calibrate radio` sweep — RC8 never moved). The flight-mode
      switch did nothing. The actual 3-position switch was on RC6 the whole time;
      `FLTMODE1/4/6` (RTL/FBWA/MANUAL) all confirmed reachable once `FLTMODE_CH` was
      corrected to `6`.
    - **A real gap this exposed in `ardupilot.flight_modes` (M9's rule, not M8a's):**
      `mavforge setup` reported this step `done` the entire time `FLTMODE_CH=8` was
      pointing at a dead channel. The rule only checks `FLTMODE_CH != 0`, which cannot
      tell "a channel number is set" apart from "that channel actually carries a
      signal" — the exact distinction that mattered here. **Fixed same day**: a PASSED
      finding for this one step now downgrades to `unknown` in `setup_status` (the
      SUSPECTED/disabled case is unaffected and stays reliable) — see M9, below.
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
    unit (rules), fake-vehicle integration, CLI and MCP tests. **SITL** (2026-10-24):
    extended `test_sitl_diagnose.py` — disables `LOG_BACKEND_TYPE` on real ArduCopter
    4.7.2, confirms `propose_fixes=False` leaves `proposed_plan` unbuilt, then confirms
    `propose_fixes=True` builds a real plan that applies cleanly and restores the
    firmware's own default (`LOG_BACKEND_TYPE=1`). **Hardware** (2026-10-26): the same
    sequence, by hand, on the ArduPlane 4.6.2 bench board — disabled, confirmed
    `propose_fixes=False` leaves `proposed_plan` unbuilt and `=True` builds a real plan,
    applied and verified back to `LOG_BACKEND_TYPE=1`. M8 (both halves) now has fake
    vehicle, SITL and hardware evidence.

### M9: Guided vehicle setup ("wizard")
Ties M1–M8 together into one guided first-time setup, instead of requiring a person
(or an AI assistant) to already know ArduPilot's setup order and which steps apply to
their vehicle. This session is the motivating example: a first-time owner, walked
through orientation, calibration, radio, flight modes and failsafes one conversation
at a time, with the AI assistant supplying the sequence and the "why" from its own
general knowledge rather than from anything MAVForge itself knows or verifies.
Scoped in detail in [research.md](research.md#guided-setup-wizard-planned-behaviour-for-m9).
The key finding there: most of the "is this done" signal **already exists**, as
`diagnose`'s findings (`ardupilot.accel_cal`, `.compass_cal`, `.rc_cal`,
`.battery_failsafe`, ...) — M9 is mostly plumbing to read that signal as a sequence,
not a new detection engine. Two things genuinely don't exist yet and are scoped
accordingly, not treated as blockers:
- **Orientation and flight-mode "done-ness" can't come from parameters alone** —
  `AHRS_ORIENTATION=0`/`FLTMODE_CH=0` are exactly as likely to be "correct" as "never
  set." These stay `unknown, run the check` rather than a false verdict (`FLTMODE_CH=0`
  specifically is a clean "not set up" signal and could get its own small rule, mirroring
  `_rc_cal`'s existing "probably never done" pattern).
- **A true "was this verified for this exact vehicle" signal needs the audit log's
  read side**, which doesn't exist (`AuditLog` is write-only today — `record()` appends,
  nothing reads it back). Building that is real, separate work; v1 relies only on live
  parameters and leaves audit-backed verification as a deliberate second version.
- `setup_status(session)`: read-only, same risk tier as `diagnose` (built from one call
  to it), reports `done`/`needed`/`recommended`/`unknown`/`not_applicable` per step.
- **CLI `mavforge setup` is a dashboard, not an orchestrator**: shows the checklist and
  the exact next command to run, rather than driving six workflows' confirmation flows
  itself. Every command it points at already has its own tests and approval path.
- **MCP `setup_status`**: read-only, same tier as `diagnose`. An AI assistant gets the
  checklist from MAVForge itself, then drives the conversation with the *existing*
  tools — gyro/level/compass over MCP; radio, orientation, flight-modes and accel still
  point at the CLI, exactly as `calibration_request` already explains per kind.
- Only ArduPilot/plane has the engineering depth for this today; the generic adapter
  reports everything `not_applicable`.
- **Implemented** (2026-10-24): `services/setup.py` (`setup_status`), built from exactly
  the design above — a single `diagnose()` call reinterpreted against a small ordered
  table. Along the way, added the one new rule the design called for
  (`ardupilot.flight_modes`: `FLTMODE_CH=0` → `SUSPECTED`, mirroring `_rc_cal`'s
  pattern) and fixed a real gap found while building M8a: `CALIBRATE_FLIGHT_MODES` had
  never been added to the capabilities catalog, so `mavforge capabilities` didn't list
  it at all. CLI `mavforge setup` (a dashboard: the checklist plus the exact next
  command, never running a workflow itself); API `vehicle.setup_status()`; MCP
  `setup_status`, read-only. Covered by unit tests (the pure interpretation logic,
  against a synthetic `DiagnosticReport`), fake-vehicle integration tests (fully
  configured, an unconfigured plane, the new `FLTMODE_CH=0` case), CLI, MCP, and
  parity (API/CLI/MCP agree) — **and** a SITL test, since SITL now runs on this Mac
  (see Release, below): confirmed against real ArduCopter 4.7.2 firmware that
  orientation/gyro/level correctly stay `unknown` and the sequence order is right.
  **Hardware** (2026-10-26): the `ardupilot.flight_modes`-passed-means-unknown fix
  above, directly motivated by and verified against the real ArduPlane bench board —
  see M8a's hardware bullet.

### Release (6)
- **CI on GitHub: already running, this item is stale.** `.github/workflows/ci.yml`
  triggers on every push to `main`; checked directly against GitHub's own API
  (2026-10-26), not assumed: it has run on every push since the first one this session
  made (`179c842`, 2026-10-24) and passed every time — all three Python matrix legs
  (3.12/3.13/3.14) plus the `sitl` job (building `docker/sitl` and running
  `pytest -m sitl` on GitHub's own runners). This item was simply never checked before
  being written down as undone. What's still genuinely open: no branch protection
  requiring it to pass, and no status badge in this README.
- SITL run on this Mac for the first time (2026-10-24), Python 3.14.7, under Colima
  (`--vm-type=vz --vz-rosetta`, native arm64 VM; Rosetta accelerates the `linux/amd64`
  SITL image rather than emulating a full x86_64 VM, which is far slower): all 17
  existing SITL tests passed against real ArduCopter 4.7.2 firmware that day. SITL
  coverage for M8a and M8b specifically followed shortly after (2026-10-24; see their
  own entries above) — 20 SITL tests pass as of 2026-10-26.
- API, CLI and MCP reference docs; a compatibility matrix backed by test evidence;
  CONTRIBUTING; version 0.1.0 (and possibly a PyPI release).
- A fresh-clone quick start, verified on a second machine.

### Hardening (5)
- **Recover from link loss *during* a long operation: re-scoped, partly done
  (2026-10-26).** Checked the actual current behavior before assuming the gap: both
  calibration (`operations.py`) and `apply_plan` already detect link loss cleanly and
  report it (`UNKNOWN`/`NOT_ATTEMPTED`, no hang, no blind retry) — that part was
  already done, just not reflected here. The MCP server's `VehicleProvider` already
  reconnects, but only *between* calls, which doesn't help an operation already running
  in a background thread. The real remaining gap split into two very different sizes:
  automatic reconnect-and-resume for an *in-progress* sweep (radio/orientation/
  flight-modes) is a genuinely new feature with real connection-lifecycle design
  questions, deliberately deferred rather than designed on the spot. The small,
  bounded piece — a retry helper for `apply_plan`'s partial results — is done: a new
  `plan_retry()` (`services/changes.py`) builds a plan from only what an earlier apply
  left `UNKNOWN`/`NOT_ATTEMPTED`, at its original requested value. `FAILED` (the
  vehicle rejected it, or read-back disagreed) and `SKIPPED` (the baseline moved since
  planning) are deliberately excluded — both need a person to look again, not a blind
  repeat. Needed a new `plan_store.load_result()` (the `.applied.json` file was
  already written, nothing read it back before). CLI `mavforge params retry <plan-id>`
  (same shape as `params restore`); API `vehicle.params.plan_retry()` (takes either the
  `ApplyResult` in hand or just the plan id); MCP `params_plan_retry`, gated the same
  as `params_plan`/`params_plan_restore`. Covered by fake-vehicle integration tests
  (including the realistic case: a write can actually succeed even when its
  acknowledgment is lost, so retrying it correctly comes back `UNCHANGED`, not a
  redundant `VERIFIED`), CLI and MCP tests. No SITL test: there's no clean,
  non-flaky way to deterministically sever a real SITL connection mid-write, and the
  *existing* link-loss-detection feature has no SITL coverage for the same reason —
  consistent with that precedent, not a new gap.
- **Wrong-vehicle tests beyond the current ones: done (2026-10-26).** Read the actual
  selection code (`session.py`'s `wait_for_vehicle`) before writing tests against an
  assumed heuristic: it returns the *first* `MAV_COMP_ID_AUTOPILOT1` heartbeat seen
  (falling back to the first autopilot-type heartbeat of any component if none ever
  arrives), not "lowest sysid" or similar — ambiguous multi-vehicle selection is
  genuinely arrival-order-dependent by design, which a new test would need unreliable
  timing control to exercise directly, so it wasn't forced. Three new tests that don't
  need that: (1) the fallback-to-non-AUTOPILOT1 path, previously uncovered — every
  existing test's vehicle happened to use the default component id; (2) GCS heartbeats
  and multiple other real vehicles present *together*, not just each scenario alone;
  (3) the most directly safety-relevant one — a plan made for one vehicle on a
  multi-vehicle link is refused (`same_vehicle_problems`, risk #2) when applied against
  a *different* real vehicle reachable on that same link, not just against an
  unrelated fake-vehicle instance as the one existing identity-mismatch test
  (`test_calibration.py`) already covered.
- **Security review against `docs/risks.md`: done (2026-10-26).** All 17 risks
  re-verified against current code (not just the register's own claims), specifically
  scrutinizing the M8/M9 work for anything bypassing the existing permission/approval
  architecture — it doesn't; every new write path composes with `plan_changes()` /
  `apply_plan()` rather than adding a parallel one. No findings, no new risk needed.
  Full writeup: [security-review.md](security-review.md). Worth another pass whenever
  MCP-over-HTTP (below) actually ships — that's the one item left in this section
  that changes the attack surface rather than extending existing machinery.
- MCP over localhost HTTP (stdio only today).

### Diagnostics (0, remaining)
- **Findings that propose a parameter plan: done.** Built for M8b
  (`propose_fixes`/`Finding.proposed_plan`), already in production use for the
  `logging` and (as of 2026-10-26) `airspeed_setup` rules — the ones with a complete,
  self-contained safe default. Rules needing operator judgment (battery thresholds)
  deliberately don't propose one.
- **Airspeed setup for planes: done (2026-10-26).** New rule `ardupilot.airspeed_setup`:
  flags `ARSPD_USE` enabled with no sensor configured (`ARSPD_TYPE=0`) as a WARNING,
  not a FAULT — confirmed from `AP_Arming::airspeed_checks()` source that ArduPilot's
  own pre-arm check only examines sensors already marked `enabled`, so it silently
  skips exactly this case rather than rejecting it. A second check found in the same
  source file (`AIRSPEED_MIN too low`, against an internal `MIN_AIRSPEED_MIN`
  constant) was deliberately left unimplemented: that constant's exact value wasn't
  found in the fetched source, and this project doesn't encode a diagnose threshold
  it can't verify. See `docs/research.md` ("Diagnostics rules").
- **Failsafe thresholds against battery capacity: done (2026-10-26), narrower than the
  original wording.** Found the real firmware check first:
  `AP_BattMonitor_Backend::arming_checks()` doesn't compare a failsafe threshold
  against total `BATT_CAPACITY` — it compares the low and critical thresholds
  *against each other* (`BATT_LOW_VOLT` must exceed `BATT_CRT_VOLT`; same for the
  `_MAH` pair), and refuses to arm if they're inverted or equal, independent of
  whether an action is even configured. That's the actual, verifiable firmware
  behavior, so that's what the extended `ardupilot.battery_failsafe` rule checks —
  as a FAULT, since ArduPilot's own arming check genuinely rejects it. See
  `docs/research.md` ("Diagnostics rules").
- **Serial-port protocol conflicts: investigated, rejected (2026-10-26).** The one
  firmware-level check found (`AP_Arming::serial_protocol_checks()`: a second
  `SERIALn_PROTOCOL=23`/RCIN port fails arming) turned out to collide with an
  officially-documented feature (a *second* receiver, intentionally configured with
  that same value, gated by `RC_OPTIONS` bit 10). Couldn't confirm from the source
  fetched whether that path bypasses the check via the IOMCU or is a genuine firmware
  inconsistency — implementing the simple version risks a false-positive FAULT on a
  legitimate setup, so it wasn't built. See `docs/research.md` ("Rejected rule:
  duplicate serial-port protocol").
- **Servo output functions: done (2026-10-26), narrower than the full doc list.**
  `AP_Arming::servo_checks()` only examines channels with an assigned
  `SERVOn_FUNCTION`, rejecting `SERVOn_MIN > SERVOn_TRIM` or `SERVOn_MAX < SERVOn_TRIM`
  — same shape as the existing RC-channel rule, for outputs instead of inputs. Not
  gated by vehicle family (`SRV_Channel` is universal). Deliberately left out: the
  documented "no SERVOx_FUNCTION set to MotorX" check, which needs frame-specific
  motor-count knowledge this rule doesn't have. See `docs/research.md` ("Diagnostics
  rules").
- **Optional comparison against a saved baseline inside the report: done
  (2026-10-26).** Built on the existing diff machinery (`services/diff.py`'s pure
  `compare()`, already used by `params diff`) rather than a new comparison engine:
  `diagnose(baseline=...)` loads the baseline eagerly, before any vehicle contact (a
  bad file fails fast, not after already downloading the whole parameter set and
  waiting on pre-arm), then runs the same `compare()` against the freshly-downloaded
  backup it builds anyway for the parameter rules. Surfaced as `DiagnosticReport.baseline`
  (`ParamDiffResult | None`), reusing the CLI's existing diff renderer rather than a
  new one. MCP takes `baseline_backup_id` (a backup id only, resolved through the same
  `resolve_backup_id_safely()` guard `params_diff` already uses — agents don't get an
  arbitrary filesystem path); CLI/API take a path or backup id, same as `params diff`
  itself. Covered at the service (direct comparison against `params.diff()`'s own
  result), CLI, and MCP (including the path-traversal-refusal test, mirrored from
  `params_diff`'s existing one) layers.

### Parameters (0, remaining)
- **MAVFTP `@PARAM/param.pck` bulk download: done (2026-10-26).** New
  `protocol/ftp.py` (sequential OpenFileRO/ReadFile/TerminateSession client) and
  `protocol/param_pack.py` (pure decoder for the packed format). Wired into
  `services/backup.py::fetch_all_params()`, now the single funnel used by
  `build_backup()`, `params.list()` and `changes.py`'s by-name lookup: tries FTP when
  the vehicle reports the capability, falls back to the existing
  `PARAM_REQUEST_LIST` method (`protocol/params.py::fetch_all()`) on any FTP-specific
  failure. Uploads deliberately excluded - `write_param()`'s per-parameter read-back
  verification has no MAVFTP equivalent and is core to this project's safety model.

  Found a real discrepancy between the mavlink.io spec and ArduPilot's actual
  behavior: the client, built strictly to the documented spec, timed out against real
  ArduCopter 4.7.2 SITL with no visible error. `GCS_FTP.cpp` (ArduPilot's own source)
  confirmed why: `reply.seq_number = request.seq_number + 1`, not "the same number as
  the request" as mavlink.io states. The fake vehicle's FTP test double had been
  built on the same wrong assumption and had to be fixed too - exactly the scenario
  the SITL layer of evidence exists for, since a fake vehicle can "agree with itself"
  on a wrong assumption indefinitely. See `docs/research.md` ("MAVFTP bulk parameter
  download") for the full writeup.

  Tested at all four layers: `protocol/param_pack.py` against hand-built bytes from
  the spec (unit), the FTP client against the fake vehicle's new FTP server
  (multi-chunk, server-side failure, capability-absent, timeout), real ArduCopter
  4.7.2 SITL (`param.pck` decodes to exactly the same parameter set
  `PARAM_REQUEST_LIST` gets; a monkeypatch confirms the fallback is never actually
  taken when FTP works), and real ArduPlane 4.6.2 hardware over USB serial (same
  exact-match check, plus a real-world number: **1.11s via FTP vs 16.28s via
  PARAM_REQUEST_LIST for the same 1117 parameters**, about 15x). Two pre-existing
  tests (`test_incomplete_backup_is_partial_and_marked`,
  `test_list_recovers_dropped_parameters`) tested PARAM_REQUEST_LIST-specific
  fault-injection knobs that have no MAVFTP equivalent; pinned to
  `ftp_enabled=False` rather than taught to the fake FTP server, since an incomplete
  FTP transfer isn't "partial data," it's an exception that triggers the fallback.
- **Diff against firmware defaults: done (2026-10-26).** Checked
  `AP_Filesystem_Param.cpp::pack_param()` directly (not just the wiki, after the
  seq_number mismatch above showed the wiki isn't always reliable here): ArduPilot
  only attaches a default to an entry when it was requested *and* the live value
  differs from it. So one `param.pck?withdefaults=1` download answers the whole
  comparison on its own - every name in the response's decoded `defaults` map is a
  changed parameter with its default right there; every absent name already equals
  its (unmodeled) default. No second request needed.

  MAVFTP-only, with no fallback: there's no `PARAM_REQUEST_LIST` equivalent of "what
  is this parameter's default", so `services/diff.py::diff_against_defaults()`
  raises `UnsupportedError` when the vehicle doesn't report FTP support, rather than
  silently doing something else. New API `vehicle.params.diff_defaults()`, CLI
  `mavforge params diff-defaults`, MCP `params_diff_defaults` (all reuse
  `ParamDiffResult`/the existing diff renderer - this is the same shape as `params
  diff`, just against the firmware instead of a saved file).

  `protocol/param_pack.py`'s decoder extended to capture `defaults: dict[str,
  float|int]` (previously deliberately skipped, see M-earlier-this-session's writeup
  above); the fake vehicle's `.pck` encoder and FTP handler extended to model
  `?withdefaults=1` and a configurable `firmware_defaults` knob, matching the real
  "only attach when it differs" semantics exactly.

  Tested at all four layers again: the decoder against hand-built bytes (3 more unit
  tests), the fake vehicle (5 integration tests: found, unchanged, "listed but equal
  behaves like absent," volatile marking, unsupported-without-FTP), real ArduCopter
  4.7.2 SITL (42 of 1369 parameters differ from stock - sensor device IDs, baro
  calibration, compass priorities, all deliberately set by SITL's own
  `default_params/copter.parm`), and real ArduPlane 4.6.2 hardware (76 of 1117
  differ, cross-checked against a plain backup's live values - exact match -
  including `FLTMODE_CH`: default `8`, current `6`, the exact fix made earlier on
  this same vehicle, now correctly shown as a deliberate change rather than an
  oversight). See `docs/research.md` ("MAVFTP bulk parameter download") for the full
  writeup.

### Calibration (3, remaining)
- **6-position accelerometer calibration on hardware: done (2026-10-27).** Ran
  `mavforge calibrate accel` against the real ArduPlane 4.6.2 (over WiFi this time,
  not USB). All six positions (level, left, right, nose down, nose up, back/upside
  down) accepted; vehicle reported `ACCELCAL_VEHICLE_POS_SUCCESS`; offsets and scale
  read back non-zero and changed from their prior values. No code changes needed -
  this item was purely the hardware-evidence gap SITL coverage already had.
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
| Battery failsafe (`params set`, over WiFi) | passed; `BATT_FS_LOW_ACT`/`BATT_FS_CRT_ACT`/`BATT_LOW_VOLT`/`BATT_CRT_VOLT` set, verified by read-back | 2026-10-24 |
| `mavforge setup` (M9) | passed; correctly showed orientation/gyro/level `unknown`, everything else `done`, over USB | 2026-10-25 |
| Flight mode assignment (Spektrum DX7s) | passed, after finding and fixing a real misconfiguration: `FLTMODE_CH=8` pointed at a dead channel (confirmed with a live RC monitor and a `calibrate radio` sweep — RC8 never moved). Corrected to the real 3-position switch (RC6); RTL/FBWA/MANUAL all confirmed reachable. Also fixed a CLI bug found in the process: live sweep progress was never rendered for this check (worked internally, displayed nothing) | 2026-10-26 |
| `diagnose --propose-fixes` (M8b) | passed; disabled `LOG_BACKEND_TYPE`, confirmed the plan builds only with the flag, applied and verified back to its original value | 2026-10-26 |
| MAVFTP bulk param download | passed; 1117 params, exact match against `PARAM_REQUEST_LIST` (only `STAT_RUNTIME` differed, a known-volatile counter); **1.11s via FTP vs 16.28s via `PARAM_REQUEST_LIST`** on this real USB-serial link, about 15x; `backup()` confirmed to use FTP, not fall back | 2026-10-26 |
| Diff against firmware defaults | passed; 76 of 1117 params differ from stock, cross-checked against a plain backup's live values (exact match); correctly surfaced `FLTMODE_CH` (default 8, current 6) as a deliberate change - the exact fix made earlier on this same vehicle | 2026-10-26 |
| 6-position accelerometer calibration | passed, over WiFi; all six positions (level, left, right, nose down, nose up, back) accepted, `ACCELCAL_VEHICLE_POS_SUCCESS`, `INS_ACCOFFS_*`/`INS_ACCSCAL_*` read back non-zero and changed | 2026-10-27 |
