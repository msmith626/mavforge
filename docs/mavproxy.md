# MAVForge and MAVProxy

[MAVProxy](https://ardupilot.org/mavproxy/) is an interactive, console-based ground
station and MAVLink router. MAVForge is not a replacement for it, and the two are
designed to run side by side.

| Use MAVProxy for | Use MAVForge for |
|---|---|
| Interactive sessions, maps, consoles | Scripted, one-shot, machine-readable operations (`--json`, exit codes) |
| Missions, fences, rally points | Typed Python API for your own tools |
| Modes, arming, flying | Verified, audited configuration work (read-only today; writes planned) |
| Routing and forwarding links (`--out`) | AI-agent access under a policy (MCP) |
| Log download, terrain, modules | Deterministic calibration workflows (planned) |

The two overlap on parameter get/set/diff and on triggering calibrations. Where they
overlap, MAVForge's job is verification and auditability: values are compared on
read-back, outcomes say `partial` or `unknown` instead of failing silently, and every
operation is recorded.

## Sharing one vehicle link

A serial port can only be opened by one program. Let MAVProxy own the port and give
MAVForge a UDP output:

```sh
mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14551
# in another terminal
mavforge -c udpin:127.0.0.1:14551 info
mavforge -c udpin:127.0.0.1:14551 params list 'COMPASS_*'
```

**Tested:** MAVProxy 1.8.75 with ArduCopter 4.7.2 SITL (`--master=tcp:127.0.0.1:5760`)
and MAVForge on `udpin:127.0.0.1:14551`. `info`, `status`, `params get`, and a full
1369-parameter `params list` all succeeded.
[mavlink-router](https://github.com/mavlink-router/mavlink-router) should work the same
way, but it is untested.

**What you will notice in MAVProxy:**
- MAVProxy also sees the vehicle's replies to MAVForge's requests. For example, a
  parameter download makes ArduPilot re-send its version banner.
- These replies are harmless.
- MAVForge uses MAVLink source component 25, while MAVProxy uses 0, so command ACKs
  addressed to each tool can be told apart.

**MAVForge never sends HEARTBEAT**, so running it doesn't change what the vehicle
believes about GCS presence or failsafes.
