# Risk register

**Last reviewed: 2026-10-26** — all 17 risks below re-verified against current code,
including the M8/M9 work (flight-mode assignment, `propose_fixes`, `setup_status`); no
new risk found. Full writeup: [security-review.md](security-review.md).

| # | Risk | Likelihood / impact | Mitigation | Status |
|---|---|---|---|---|
| 1 | pymavlink's global dialect state and `recv_match` dropping messages corrupt concurrent operations | High / High | Dialect set once (`transport/dialect.py`). Single reader thread + `MessageRouter` | Done; router unit tests |
| 2 | Connecting to the wrong vehicle on a multi-vehicle link | Medium / High | Ignore GCS heartbeats, prefer compid 1, `--target-system` pinning, `other_systems_seen` warning, identity recorded in every result and audit entry. Writes will require pinning when several vehicles are seen | Read side done |
| 3 | An agent approves its own change, or prompt injection via STATUSTEXT or docs | Medium / High | Approvals bound to the plan hash and issued only by a human channel; vehicle text typed as untrusted; inert CLI rendering | Done: digest-bound, expiring, single-use approvals; MCP approval parameter hidden from agents (forgery test) |
| 4 | Link lost during a write; parameter batches are not atomic | Medium / High | Record before / requested / after values per parameter; `unknown` status on timeout; read-back verification; no blind retries of non-idempotent commands | Done: per-parameter outcomes, link-loss test, drift skip, pre-change backup |
| 5 | Float encoding rounds large int32 parameters | Low / Medium | `precision_limited` flag; type-aware comparisons in diff and verify | Flag done |
| 6 | Parameter names or semantics change between firmware versions (e.g. 4.7 `SYSID_THISMAV` → `MAV_SYSID`) | High / Medium | Metadata keyed by firmware version; restore blocked on identity or firmware mismatch unless explicitly overridden with warnings | Done: metadata keyed by exact firmware (fallback flagged); restore identity gate |
| 7 | Calibration reported as successful when it wasn't | Medium / High | Require a terminal vehicle signal **and** a parameter read-back postcondition | Done: per-workflow evidence ([calibration.md](calibration.md)); compass offsets must match the reported values |
| 8 | MCP SDK v2 and spec 2026-07-28 are new | Medium / Low | Spike-tested; MCP layer kept thin; dependency pinned `mcp>=2.3,<3` | Done |
| 9 | Network-exposed MCP server | Low / High | stdio only for now. HTTP will bind to localhost by default with a warning; OAuth resource-server work only if remote use is requested | stdio only |
| 10 | Licensing: accidental copying from GPL or unlicensed sources | Low / Medium | MAVForge is GPL-3.0-or-later, compatible with pymavlink (LGPLv3) and ArduPilot / MP / MAVProxy (GPLv3). Prefer independent implementation; record any adaptation in THIRD_PARTY_NOTICES. Download apm.pdef metadata at runtime rather than vendoring it | No third-party code copied |
| 11 | Hardware tests changing a real vehicle | Low / High | `hardware` marker, opt-in env var, read-only automated tests; writes need `allow_real_vehicle` policy; manual write tests only with the owner's consent, harmless parameters, and restore | Done (one consented manual write + restore on ArduPlane 4.6.2) |
| 12 | MAVForge masking a GCS failsafe | Low / High | Never send HEARTBEAT | Done; tested |
| 13 | Stray stdout output corrupting MCP stdio | Low / Medium | MCP SDK v2 diverts fd 1; logging goes to stderr | Done; stdio tests |
| 14 | pymavlink TCP EOF handling prints to stdout and never reports the loss | Found / High | Transport converts TCP EOF/reset into a link failure; regression test | Done |
| 15 | Metadata downloaded from the network is untrusted | Low / Medium | HTTPS only, 32 MB cap, defusedxml (entity attacks rejected, tested), cache under the state dir, `--offline` | Done |
| 16 | An agent confirming a physical calibration step that never happened | Medium / High | The 6-pose accel calibration is refused for MCP in the service layer; MCP has no confirm tool; human approval is needed to start any calibration | Done |
| 17 | Cancel claims that don't hold (accel calibration cannot be cancelled over MAVLink) | Medium / Medium | The result states the autopilot stays in calibration mode until rebooted | Done |
