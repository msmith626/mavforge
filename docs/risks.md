# Risk register

| # | Risk | Likelihood / impact | Mitigation | Status |
|---|---|---|---|---|
| 1 | pymavlink's global dialect state and `recv_match` dropping messages corrupt concurrent operations | High / High | Dialect set once (`transport/dialect.py`). Single reader thread + `MessageRouter` | Done; router unit tests |
| 2 | Connecting to the wrong vehicle on a multi-vehicle link | Medium / High | Ignore GCS heartbeats, prefer compid 1, `--target-system` pinning, `other_systems_seen` warning, identity recorded in every result and audit entry. Writes will require pinning when several vehicles are seen | Read side done |
| 3 | An agent approves its own change, or prompt injection via STATUSTEXT or docs | Medium / High | Approvals bound to the plan hash and issued only by a human channel; vehicle text typed as untrusted; inert CLI rendering | Rendering done; approvals planned |
| 4 | Link lost during a write; parameter batches are not atomic | Medium / High | Record before / requested / after values per parameter; `unknown` status on timeout; read-back verification; no blind retries of non-idempotent commands | Command semantics done; writes planned |
| 5 | Float encoding rounds large int32 parameters | Low / Medium | `precision_limited` flag; type-aware comparisons in diff and verify | Flag done |
| 6 | Parameter names or semantics change between firmware versions (e.g. 4.7 `SYSID_THISMAV` → `MAV_SYSID`) | High / Medium | Metadata keyed by firmware version; restore blocked on identity or firmware mismatch unless explicitly overridden with warnings | Observed in SITL; mitigation planned |
| 7 | Calibration reported as successful when it wasn't | Medium / High | Require a terminal vehicle signal **and** a parameter read-back postcondition | Planned (see research.md) |
| 8 | MCP SDK v2 and spec 2026-07-28 are new | Medium / Low | Spike-tested; MCP layer kept thin; dependency pinned `mcp>=2.3,<3` | Done |
| 9 | Network-exposed MCP server | Low / High | stdio only for now. HTTP will bind to localhost by default with a warning; OAuth resource-server work only if remote use is requested | stdio only |
| 10 | Licensing: accidental copying from GPL or unlicensed sources | Low / Medium | MAVForge is GPL-3.0-or-later, compatible with pymavlink (LGPLv3) and ArduPilot / MP / MAVProxy (GPLv3). Prefer independent implementation; record any adaptation in THIRD_PARTY_NOTICES. Download apm.pdef metadata at runtime rather than vendoring it | No third-party code copied |
| 11 | Hardware tests changing a real vehicle | Low / High | `hardware` marker, opt-in env var, read-only tests only; future write tests need an extra env var and must never arm | Done |
| 12 | MAVForge masking a GCS failsafe | Low / High | Never send HEARTBEAT | Done; tested |
| 13 | Stray stdout output corrupting MCP stdio | Low / Medium | MCP SDK v2 diverts fd 1; logging goes to stderr | Done; stdio tests |
