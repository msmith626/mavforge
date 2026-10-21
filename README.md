# MAVForge

*Intelligent tools for flight controllers.*

MAVForge is an open-source engineering toolkit for configuring, diagnosing and (soon)
calibrating MAVLink flight controllers, starting with ArduPilot. One tested core is
exposed three ways:

- a **Python API** for scripts and other programs
- a **one-shot CLI** (`mavforge`) with `--json` output and stable exit codes
- an **MCP server** (`mavforge-mcp`) so AI agents can use the same operations under
  the same safety policy

MAVForge complements interactive ground stations such as MAVProxy and Mission Planner.
It does not replace them: there is no console, map, mission planning or flight control.
See [docs/mavproxy.md](docs/mavproxy.md).

> **Status: pre-alpha (0.1.0.dev0).** Read-only operations only. Parameter changes,
> backups, diagnostics and calibration are planned; see [docs/roadmap.md](docs/roadmap.md).
> Nothing MAVForge reports means a vehicle is safe to fly.

## What works today

| Operation | API | CLI | MCP tool | Verified against |
|---|---|---|---|---|
| Identify vehicle (autopilot, firmware, git hash, board, capabilities, SITL detection) | `vehicle.info()` | `mavforge info` | `vehicle_info` | fake vehicle, ArduCopter 4.7.2 SITL |
| Status snapshot (armed, mode, sensors, pre-arm flag, battery, GPS, STATUSTEXT) | `vehicle.status()` | `mavforge status` | `vehicle_status` | fake vehicle, ArduCopter 4.7.2 SITL |
| Download all parameters, glob filter, gap recovery | `vehicle.params.list()` | `mavforge params list` | `params_list` | fake vehicle, ArduCopter 4.7.2 SITL (1369 params) |
| Read one parameter | `vehicle.params.get()` | `mavforge params get` | `params_get` | fake vehicle, ArduCopter 4.7.2 SITL |
| List operations and permissions | `mavforge.capabilities()` | `mavforge capabilities` | `mavforge_capabilities` | unit tests |

Real-hardware testing is opt-in ([tests/hardware](tests/hardware)) and has not yet been
reported for any board. Only ArduPilot has a platform adapter. Other autopilots get a
generic read-only adapter that has not been tested.

## Install (development)

Requires Python ≥ 3.12 and [uv](https://docs.astral.sh/uv/).

```sh
git clone <this repo> mavforge && cd mavforge
uv sync
uv run mavforge --help
```

## Quick start

```sh
# ArduPilot SITL in Docker (see docs/sitl.md)
docker build -t mavforge-sitl:4.7.2 docker/sitl
docker run --rm -p 127.0.0.1:5760:5760 mavforge-sitl:4.7.2 &

uv run mavforge -c tcp:127.0.0.1:5760 --timeout 30 info
uv run mavforge -c tcp:127.0.0.1:5760 status
uv run mavforge -c tcp:127.0.0.1:5760 params list 'COMPASS_*'
uv run mavforge -c tcp:127.0.0.1:5760 --json params get FRAME_CLASS
```

To use a real flight controller over USB, run `mavforge -c /dev/ttyACM0 info`.

Global options go before the command. You can also put the connection in
`MAVFORGE_CONNECTION` instead of passing `-c`. Exit codes:

| Code | Meaning |
|---|---|
| 0 | OK |
| 1 | Failed |
| 2 | Usage error |
| 3 | Connection failed |
| 4 | Timeout or unknown outcome |
| 5 | Not permitted |
| 6 | Unsupported |
| 7 | Partial result |

### Python API

```python
import mavforge

with mavforge.connect("tcp:127.0.0.1:5760", timeout=30) as vehicle:
    print(vehicle.identity.firmware)  # e.g. 4.7.2
    result = vehicle.params.list("INS_*")
    if result.status != "succeeded":  # partial downloads are reported, not hidden
        print("missing:", result.missing_indices)
    for p in result.params:
        print(p.name, p.value, p.type)
```

### MCP (AI agents)

```sh
uv run mavforge-mcp --connection tcp:127.0.0.1:5760
```

See [docs/mcp-setup.md](docs/mcp-setup.md) for client configuration.

## Safety model (short version)

- **Read-only by default.** The current release has no operation that changes the
  vehicle. Future write operations will need explicit permissions, a human approval
  that an agent cannot fabricate, and verification by read-back.
- **No arming, motor tests or flight control**, by design.
- **The operator picks the vehicle link** when starting the MCP server; agents can't
  choose it.
- **Every operation is audited** to `~/.local/state/mavforge/audit.jsonl` (mode 0600),
  with the interface (api/cli/mcp), the vehicle identity and the outcome.
- **Vehicle text is treated as untrusted.** It is rendered inertly in the terminal and
  marked as data for agents.
- **MAVForge never sends HEARTBEATs**, so it cannot mask a GCS-failsafe condition.

Details: [docs/architecture.md](docs/architecture.md), [docs/risks.md](docs/risks.md).

## Development

```sh
uv run ruff format --check . && uv run ruff check . && uv run pyright
uv run pytest            # unit + integration (fake vehicle), no hardware needed
uv run pytest -m sitl    # ArduCopter SITL in Docker
MAVFORGE_HW_CONNECTION=/dev/ttyACM0 uv run pytest -m hardware   # opt-in, read-only
```

## Documentation

- [Assessment](docs/assessment.md): project state, positioning and gaps
- [Research notes](docs/research.md): MAVLink, ArduPilot, Mission Planner and MCP findings, with sources
- [Architecture](docs/architecture.md) · [Risks](docs/risks.md) · [Roadmap](docs/roadmap.md)
- [SITL](docs/sitl.md) · [MCP setup](docs/mcp-setup.md) · [MAVForge and MAVProxy](docs/mavproxy.md)
- [Contributing](CONTRIBUTING.md) · [Third-party notices](THIRD_PARTY_NOTICES.md)

## License

GPL-3.0-or-later. See [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
