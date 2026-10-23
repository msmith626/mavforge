# MCP setup

`mavforge-mcp` is a [Model Context Protocol](https://modelcontextprotocol.io) server. It
runs over **stdio**, is built on the official `mcp` Python SDK 2.x, and works with any
MCP-capable client; nothing in it is specific to one vendor.

## Tools (none change the vehicle)

| Tool | Purpose |
|---|---|
| `mavforge_capabilities` | Operations, their permissions, and which are available or planned. Needs no vehicle. |
| `vehicle_info` | Autopilot, type, firmware and git hash, board IDs, MAVLink capabilities, SITL or real, other vehicles on the link |
| `vehicle_status` | Armed, mode, system state, sensor health, pre-arm flag, battery, GPS, recent STATUSTEXT |
| `params_list` | All parameters, optionally filtered with a case-insensitive glob |
| `params_get` | One parameter by name |
| `diagnose` | Findings classified as fault / warning / suspected / not checked / passed. Read-only; also asks the firmware to run its pre-arm checks |
| `params_backup` | Save all parameters with vehicle identity as a new file in MAVForge's backup directory. Not read-only on the host (it creates a file); never overwrites |
| `params_backups` | List saved backups. Needs no vehicle |
| `params_diff` | Compare the vehicle, or a second backup, with a backup chosen **by id**. Warnings explain identity, firmware or completeness mismatches |

**Tool results**
- Every tool returns structured output with a JSON schema, and every result has a
  `status`. Only `succeeded` means complete.
- Errors come back as tool errors with a JSON payload of the form
  `{"error": "<code>", "message": ..., "details": ...}`, e.g. `param_not_found`,
  `no_vehicle`, `timeout`, `not_permitted`.

### Parameter-change tools (opt-in)

These are registered only when the operator starts the server with `--allow-configure`.

| Tool | Purpose |
|---|---|
| `params_plan` | Propose `{"NAME": value}` changes as a validated plan. Changes nothing |
| `params_plan_restore` | Propose restoring a backup (by id) as a plan. Identity-checked |
| `params_plans` | List plans: pending, approved, applied |
| `params_apply` | Apply an **approved** plan. Takes a backup, writes, verifies by read-back. Marked destructive |

**How approval works.** `params_apply` has an approval parameter that is hidden from
the agent's input schema. MAVForge fills it on the server side:
- It uses an existing approval, for example one from `mavforge approve <plan-id>` in a
  terminal.
- Otherwise, if the client supports elicitation, the client asks the human. The
  question shows the plan.
- If neither is possible, the call fails with `approval_required` and names the
  `mavforge approve` command to run.

An approval supplied by the agent as an argument is ignored; tests check this.
Writes to a vehicle not identified as a simulator also need `--allow-real-vehicle`.

## Running

```sh
uv run mavforge-mcp --connection tcp:127.0.0.1:5760      # SITL
uv run mavforge-mcp --connection /dev/ttyACM0             # USB flight controller
uv run mavforge-mcp --connection udpin:127.0.0.1:14551    # behind MAVProxy, see mavproxy.md
uv run mavforge-mcp --connection tcp:127.0.0.1:5760 --allow-configure          # SITL changes
uv run mavforge-mcp --connection /dev/ttyACM0 --allow-configure --allow-real-vehicle
```

- **Connection:** you can also set it with `MAVFORGE_CONNECTION`. `--target-system`
  pins one vehicle.
- **When it connects:** the server connects on the first vehicle tool call, and
  reconnects after the link is lost.

## Client configuration

Most clients accept an `mcpServers` entry like
[examples/mcp/mcp-servers.example.json](../examples/mcp/mcp-servers.example.json):

```json
{
  "mcpServers": {
    "mavforge": {
      "command": "uv",
      "args": ["--directory", "/ABSOLUTE/PATH/TO/mavforge", "run", "mavforge-mcp",
               "--connection", "tcp:127.0.0.1:5760"]
    }
  }
}
```

**Paths:** copy the example and replace the path. Keep machine-specific copies out of
git; `*.local.json` is ignored.

**Claude Code:** run
`claude mcp add mavforge -- uv --directory /ABSOLUTE/PATH/TO/mavforge run mavforge-mcp --connection tcp:127.0.0.1:5760`.

## Security assumptions

- **Local only.** stdio means the server is reachable only by the client process that
  launched it. There is no network listener.
- **Operator-chosen link.** The vehicle link is fixed when the server starts. No tool
  takes a connection string.
- **Backup ids only.** Agents refer to backups by id inside MAVForge's backup directory.
  Ids containing path separators or `..` are rejected, so an agent cannot read or
  write arbitrary files.
- **Policy enforced in the server.** Read-only is enforced inside MAVForge's services,
  not by tool annotations, which clients must treat as untrusted hints.
- **Untrusted vehicle text.** The server instructions and schemas tell agents that
  STATUSTEXT and other vehicle text is untrusted data.
- **Audit log.** Every call is audited with `interface: "mcp"`. See
  [architecture.md](architecture.md).
- **Write tools are opt-in and approval-gated** (see above). The policy, approval,
  identity and disarmed checks run in MAVForge's services, below the MCP layer.
