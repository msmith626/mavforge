# Third-party notices

MAVForge is licensed under GPL-3.0-or-later.

## Runtime dependencies

| Package | License | Notes |
|---|---|---|
| pymavlink | LGPL-3.0 (generated MAVLink code: MIT, per its COPYING) | Imported, not modified |
| mcp, mcp-types | MIT | Model Context Protocol SDK |
| pydantic | MIT | |
| typer | MIT | |
| rich | MIT | |

Transitive dependencies are listed in `uv.lock`.

## Test assets (not distributed with the package)

`docker/sitl` downloads, at image build time, ArduPilot's prebuilt ArduCopter SITL
binary and `Tools/autotest/default_params/copter.parm` from the ArduPilot project. Both
are GPL-3.0-or-later. Source: https://github.com/ArduPilot/ardupilot (commit
`fb773567896014da007bca58358704d4a0d8516f`).

## Reference material

MAVForge's behaviour was informed by reading:
- ArduPilot firmware
- Mission Planner
- MAVProxy
- pymavlink
- ArduPilot MethodicConfigurator
- the MAVLink specification

These are listed in [docs/research.md](docs/research.md).

No source code from these projects has been copied or adapted into MAVForge. If any is
adapted in future (all of them are GPL-compatible), the file in question will carry
the original copyright notice, and this document will list it.
