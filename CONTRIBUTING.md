# Contributing to MAVForge

## Setup

```sh
uv sync
uv run ruff format . && uv run ruff check . && uv run pyright && uv run pytest
```

Run `uv run pytest -m sitl` too (needs Docker) when you touch transport, protocol or
services.

## Ground rules

- **One implementation.** Interfaces (CLI, MCP) call the Python API, never pymavlink
  directly. New operations go through `services.catalog.run_operation` and get a
  parity test.
- **Honest outcomes.** Never report `succeeded` for something that was only sent or
  ACKed. Use `partial` or `unknown` when that's the truth.
- **Evidence for claims.** Don't document an autopilot, firmware version or workflow as
  supported without a test (fake, SITL or hardware) that shows it. Record new
  protocol findings in `docs/research.md` with their source.
- **Safety.**
  - No arming, motor-test or flight features.
  - Write operations need a policy permission, human approval, and read-back
    verification.
  - Hardware tests are opt-in and read-only unless explicitly gated.
- **Licensing.**
  - Contributions are GPL-3.0-or-later.
  - If you adapt code from another project, keep its notice and add it to
    `THIRD_PARTY_NOTICES.md`.
  - Prefer implementing from the MAVLink specification.
- **Style.**
  - `pyright --strict` and ruff must stay clean.
  - Keep modules focused, and match the comment density of the code around you.
