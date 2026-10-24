# Wireless links

MAVForge doesn't implement anything wireless-specific: a connection string is a
connection string ([connstr.py](../src/mavforge/transport/connstr.py)). Any transport
that ends up looking like `udpin:HOST:PORT`, `udpout:HOST:PORT`, `tcp:HOST:PORT` or a
serial device works exactly as it does over USB. This doc records what was actually
tried against one board — a SpeedyBee F405 WING APP (ArduPlane) — and the macOS side
of getting there, since several things about both are not obvious from the manuals.

## The board's wireless modes

The wireless top board (separate from the FC board, wired to **UART6** /
`SERIAL6`) has four RF modes, cycled by holding **BOOT for ~6 s** while the FC is
powered (not while powering on — that's DFU mode instead). A fast yellow LED flash
confirms the mode actually switched.

| LED | Mode | Carries MAVLink to ArduPilot? |
|---|---|---|
| Green, slow flash | Bluetooth BLE | No — SpeedyBee-app/INAV path only |
| **White, slow flash** | **WiFi (AP)** | **Yes** |
| Blue, slow flash | Classic Bluetooth SPP | No, on this board (see below) |
| Off | Wireless off | — |

ArduPilot's own board page and SpeedyBee's manual both say to use WiFi for ArduPilot
and BLE for INAV. ArduPilot's page is explicit that Bluetooth "has not been proven to
work with all BT dongles or cell-phones," while WiFi "has been tested with a variety
of systems." That held up exactly on this unit.

Before any of this, `SERIAL6_PROTOCOL=2` (MAVLink2) and `SERIAL6_BAUD=115` (115200)
must be set — confirmed already-correct on this board's factory defaults via
`mavforge -c /dev/cu.usbmodemXXXX params list 'SERIAL6_*'`.

## Bluetooth: dead end on this board

**BLE** (green) was ruled out without testing: BLE has no Serial Port Profile, so
macOS never creates a `/dev/cu.*` node for it. It would need a GATT-speaking bridge
program, and it's the SpeedyBee-app/INAV path on this board regardless.

**Classic Bluetooth SPP** (blue) looked more promising — macOS does expose paired SPP
devices as `/dev/cu.*`/`/dev/tty.*` nodes — and was tested directly:

- Paired from System Settings as `SpeedyBeeF405Wing-SPP`. The link was **stable**:
  `system_profiler SPBluetoothDataType` showed it staying connected, and reading the
  device node held open cleanly for 15 s with no drop. (This rules out the commonly-
  seen ESP32 `BluetoothSerial` macOS bug, where a wrong Class-of-Device makes
  `bluetoothaudiod` tear the link down — that specific failure mode does not apply
  here.)
- In that 15 s, **44,078 bytes arrived, every one of them `0x00`.** No MAVLink v1/v2
  frames, no text — a clean, continuous null stream. (Script used:
  `mavsniff.py`, written for this investigation; it walks the buffer for `0xFD`/`0xFE`
  frame headers and reports what it finds. A synthetic-frame test and an all-zero-input
  test both confirm the parser behaves correctly — this wasn't a parsing bug.)
- `SERIAL6_PROTOCOL`/`SERIAL6_BAUD` were separately confirmed correct over USB at the
  same time, ruling out a configuration cause.

So: the SPP link itself is solid, but UART6 doesn't come through it as a MAVLink
stream — the module appears to use classic SPP for something else (its own
handshake with the SpeedyBee app, most likely) rather than a transparent UART bridge.
This matches two independent reports on the ArduPilot forum for this exact board:
["bluetooth mode is not compatible with
ardupilot"](https://discuss.ardupilot.org/t/cant-connect-mp-on-laptop-via-wifi-to-speedybee-f405-wing/102308/16)
and ["With Bluetooth it's not working at
all"](https://discuss.ardupilot.org/t/cant-connect-mp-on-laptop-via-wifi-to-speedybee-f405-wing/102308/5).
Not a one-off, and not fixable from the Mac side — **don't re-investigate Bluetooth on
this board without new evidence that the firmware changed.**

## WiFi: works

1. Over USB, confirm (or set) `SERIAL6_PROTOCOL=2`, `SERIAL6_BAUD=115`, reboot.
2. Hold BOOT until the wireless LED is slow-flashing **white**.
3. Join the FC's access point from the Mac. On this unit the SSID was
   `SpeedyBee F405Wing` (open, no password); the WING MINI variant instead advertises
   `SpeedyBee eFLY-WIFI` / password `88888888`. The FC's module answers as the gateway
   (`192.168.1.1` on this unit).
4. Connect: `mavforge -c udpin:0.0.0.0:14550 --timeout 20 info` (the Mac listens; the
   module sends first, same as Mission Planner/QGC).

**Hardware evidence (2026-10-23):** `info`, `status`, and a full parameter download —
**1117/1117 parameters, no gaps, ~31 s** — all succeeded over this link, on the same
vehicle identity as the USB connection. Forum reports for this board describe WiFi as
unstable and prone to stalling partway through a parameter download; that was not
reproduced here, plausibly because testing was at close (desk) range against
SpeedyBee's own quoted 10–20 m limit.

**Trade-offs to know about:**
- **Range is short** — this is a bench/pad-configuration link, not in-flight telemetry.
  Use a SiK radio or an ESP32 `mavesp8266` bridge on the `UART4` telemetry pads for
  anything longer-range or flight-reliable.
- **The Mac's WiFi interface leaves your LAN** while joined to the FC's isolated AP.
  Harmless if another interface (e.g. Ethernet) is carrying your default route — check
  with `networksetup -listnetworkserviceorder` and `ifconfig` before assuming you're
  still online elsewhere. Switch back afterwards:
  `networksetup -setairportnetwork en1 "<home network>" "<password>"`.
- **The mode doesn't stick as "the" mode forever** — if you ever cycle back to
  BLE/SPP/off for another tool, you'll need to hold BOOT back to white again next time.

## macOS gotcha: an empty Wi-Fi network list

While chasing this down, the Mac's Wi-Fi network list went empty — not even the home
network or neighbors showed up, with Wi-Fi power confirmed "On" the whole time. This
is unrelated to the flight controller: it's a known macOS issue where **Location
Services** blocks the Wi-Fi scan (System Settings → Privacy & Security → Location
Services → System Services → "Wi-Fi Networking"). A reboot resolved it here; toggling
Location Services off/on (then restarting) is the documented fix when a reboot isn't
convenient. Worth remembering before assuming a wireless module isn't broadcasting.

## Reproducing this

Diagnostic scripts written for this investigation (not part of the package — ad hoc,
kept here for reference):

```sh
./btwatch.sh watch     # poll Bluetooth device state + /dev serial nodes, print only changes
./btwatch.sh log       # stream bluetoothd/bluetoothaudiod while pairing
python3 mavsniff.py /dev/cu.<device> <seconds>   # open a serial device, classify what arrives
```

`mavsniff.py`'s frame parser was checked against synthetic MAVLink v1/v2 frames
(including a signed v2 frame and interleaved garbage) and against an all-zero buffer,
before trusting its "no frames" verdict on the real capture.

## Related code fixes

Two issues turned up in MAVForge itself while testing against real wireless links,
both fixed:

- **[connstr.py](../src/mavforge/transport/connstr.py)** rejected serial device paths
  containing spaces, which macOS uses for paired Bluetooth SPP device names (e.g.
  `/dev/cu.SpeedyBee F405 WING-SPPDev`). Fixed by allowing spaces inside the `/dev/`
  branch of `_DEVICE_RE` only — the `/dev/` anchor still keeps `/bin/sh`, `/etc/passwd`
  and similar unsafe paths rejected.
- **[connection.py](../src/mavforge/transport/connection.py)**'s `open_failure_hint`
  only ever suggested the Linux `dialout` group fix for a permission error, which is
  actively misleading on macOS (there is no such group). It's now platform-aware
  (`platform` parameter, defaulting to `sys.platform`, injectable for deterministic
  tests regardless of which OS runs them) and also recognises `Resource busy` —
  pyserial's message for "another program already has this port open," the single
  most common real-world macOS failure, which previously fell through to the generic
  (and wrong) "check the device is plugged in" hint.
