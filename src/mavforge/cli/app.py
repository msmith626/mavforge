"""The ``mavforge`` command-line interface.

Every command is a one-shot call into the Python API (``mavforge.connect``), rendered
either as tables or, with ``--json``, as the same Pydantic models the API and MCP
server return. Exit codes are listed in ``mavforge --help``.
"""

from __future__ import annotations

import logging
import re
import sys
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Annotated, Any, NoReturn

import typer
from pydantic import BaseModel
from rich.console import Console
from rich.table import Table
from rich.text import Text

import mavforge
from mavforge.domain.errors import ExitCode, MAVForgeError
from mavforge.domain.models import (
    BackupListResult,
    BackupResult,
    CapabilitiesResult,
    InfoResult,
    ParamDiffResult,
    ParamGetResult,
    ParamListResult,
    ParamPresence,
    ParamValue,
    StatusReport,
)
from mavforge.domain.outcomes import OperationStatus, exit_code_for
from mavforge.domain.values import format_value
from mavforge.transport.connection import DEFAULT_SOURCE_COMPONENT, DEFAULT_SOURCE_SYSTEM

EXIT_CODES_HELP = (
    "Exit codes: 0 ok, 1 failed, 2 usage error, 3 connection failed, 4 timeout/unknown "
    "outcome, 5 not permitted, 6 unsupported, 7 partial result."
)

app = typer.Typer(
    name="mavforge",
    help="MAVForge: verified configuration, diagnostics and calibration tools for "
    "MAVLink flight controllers.\n\n"
    "Global options go before the command, e.g. "
    "'mavforge -c tcp:127.0.0.1:5760 status'.\n\n" + EXIT_CODES_HELP,
    no_args_is_help=True,
    rich_markup_mode=None,
    pretty_exceptions_enable=False,
)
params_app = typer.Typer(
    help="Inspect, back up and compare vehicle parameters (read-only on the vehicle).",
    no_args_is_help=True,
)
app.add_typer(params_app, name="params")

out = Console(highlight=False, soft_wrap=True)
err = Console(stderr=True, highlight=False)

_CONTROL_CHARS = re.compile(r"[\x00-\x08\x0b-\x1f\x7f-\x9f]")


def safe(text: str) -> Text:
    """Render vehicle-originated text inertly: no rich markup, no terminal escapes."""
    return Text(_CONTROL_CHARS.sub("?", text))


@dataclass(frozen=True)
class GlobalOptions:
    connection: str | None
    target_system: int | None
    source_system: int
    source_component: int
    timeout: float
    json: bool


@app.callback()
def _global_options(
    ctx: typer.Context,
    connection: Annotated[
        str | None,
        typer.Option(
            "--connection",
            "-c",
            envvar="MAVFORGE_CONNECTION",
            help="MAVLink link: tcp:HOST:PORT, udpin:HOST:PORT, udpout:HOST:PORT, "
            "/dev/ttyACM0[,BAUD] or serial:DEVICE[:BAUD].",
        ),
    ] = None,
    target_system: Annotated[
        int | None,
        typer.Option(
            "--target-system", "-t", min=1, max=255, help="Only talk to this vehicle system ID."
        ),
    ] = None,
    source_system: Annotated[
        int, typer.Option(min=1, max=255, help="MAVLink system ID MAVForge sends as.")
    ] = DEFAULT_SOURCE_SYSTEM,
    source_component: Annotated[
        int, typer.Option(min=0, max=255, help="MAVLink component ID MAVForge sends as.")
    ] = DEFAULT_SOURCE_COMPONENT,
    timeout: Annotated[
        float, typer.Option(min=0.1, help="Seconds to wait for a vehicle heartbeat.")
    ] = 5.0,
    json_output: Annotated[
        bool, typer.Option("--json", help="Print machine-readable JSON on stdout.")
    ] = False,
    verbose: Annotated[bool, typer.Option("--verbose", "-v", help="Log diagnostics.")] = False,
) -> None:
    logging.basicConfig(level=logging.DEBUG if verbose else logging.WARNING, stream=sys.stderr)
    ctx.obj = GlobalOptions(
        connection=connection,
        target_system=target_system,
        source_system=source_system,
        source_component=source_component,
        timeout=timeout,
        json=json_output,
    )


def _fail(opts: GlobalOptions, exc: MAVForgeError) -> NoReturn:
    if opts.json:
        out.print_json(data=exc.to_dict())
    else:
        err.print(Text(f"error: {exc.message}", style="red"))
    raise typer.Exit(int(exc.exit_code))


def _emit[R: BaseModel](opts: GlobalOptions, result: R, render: Callable[[R], None]) -> NoReturn:
    if opts.json:
        out.print_json(result.model_dump_json())
    else:
        render(result)
    status: OperationStatus = getattr(result, "status", OperationStatus.SUCCEEDED)
    raise typer.Exit(int(exit_code_for(status)))


def _with_vehicle[R: BaseModel](
    ctx: typer.Context, action: Callable[[mavforge.Vehicle], R], render: Callable[[R], None]
) -> NoReturn:
    opts: GlobalOptions = ctx.obj
    if not opts.connection:
        err.print("error: no connection given; use -c/--connection or set MAVFORGE_CONNECTION")
        raise typer.Exit(int(ExitCode.USAGE))
    try:
        with mavforge.connect(
            opts.connection,
            target_system=opts.target_system,
            source_system=opts.source_system,
            source_component=opts.source_component,
            timeout=opts.timeout,
            interface="cli",
        ) as vehicle:
            result = action(vehicle)
    except MAVForgeError as exc:
        _fail(opts, exc)
    _emit(opts, result, render)


def _offline[R: BaseModel](
    ctx: typer.Context, action: Callable[[], R], render: Callable[[R], None]
) -> NoReturn:
    """Run an operation that needs no vehicle connection."""
    opts: GlobalOptions = ctx.obj
    try:
        result = action()
    except MAVForgeError as exc:
        _fail(opts, exc)
    _emit(opts, result, render)


# -- renderers ----------------------------------------------------------------------


def _kv_table(rows: list[tuple[str, Any]]) -> Table:
    table = Table(show_header=False, box=None, pad_edge=False)
    table.add_column(style="bold")
    table.add_column()
    for key, value in rows:
        table.add_row(key, value if isinstance(value, Text) else safe(str(value)))
    return table


def _status_line(status: OperationStatus) -> None:
    if status is not OperationStatus.SUCCEEDED:
        err.print(Text(f"result: {status.value}", style="yellow"))


def render_info(result: InfoResult) -> None:
    v = result.vehicle
    rows: list[tuple[str, Any]] = [
        ("Vehicle", f"system {v.system_id}, component {v.component_id}"),
        ("Autopilot", f"{v.autopilot} ({v.platform} adapter)"),
        ("Type", f"{v.mav_type}" + (f" ({v.vehicle_family})" if v.vehicle_family else "")),
        (
            "Firmware",
            f"{v.firmware or 'unknown'}"
            + (f" [{v.firmware_git_hash}]" if v.firmware_git_hash else ""),
        ),
        ("Simulated", {True: "yes (SITL)", False: "no", None: "unknown"}[v.simulated]),
        ("Board", f"version 0x{v.board_version:08x}" if v.board_version is not None else "unknown"),
        ("UID", v.uid or "unknown"),
        ("Capabilities", ", ".join(v.protocol_capabilities) or "none reported"),
        ("Link", f"{result.link.connection}, {result.link.messages_received} msgs received"),
    ]
    if v.other_systems_seen:
        rows.append(("Other vehicles", ", ".join(map(str, v.other_systems_seen))))
    out.print(_kv_table(rows))
    if v.other_systems_seen:
        err.print("warning: multiple vehicles on this link; pin one with --target-system")


def render_status(r: StatusReport) -> None:
    if r.prearm_checks_enabled is False:
        prearm = Text("DISABLED on vehicle (no pre-arm checks enabled)", style="red")
    else:
        prearm = Text(
            {True: "passing", False: "FAILING", None: "not reported"}[r.prearm_checks_passing]
        )
    rows: list[tuple[str, Any]] = [
        (
            "Vehicle",
            f"system {r.vehicle.system_id} {r.vehicle.autopilot} {r.vehicle.firmware or ''}",
        ),
        ("Armed", "ARMED" if r.armed else "disarmed"),
        ("Mode", r.mode or "unknown"),
        ("System state", r.system_status),
        ("Pre-arm checks", prearm),
    ]
    if r.battery:
        b = r.battery
        rows.append(
            (
                "Battery",
                f"{b.voltage_v if b.voltage_v is not None else '?'} V, "
                f"{b.current_a if b.current_a is not None else '?'} A, "
                f"{b.remaining_pct if b.remaining_pct is not None else '?'}%",
            )
        )
    if r.gps:
        hdop = f", HDOP {r.gps.hdop}" if r.gps.hdop is not None else ""
        rows.append(("GPS", f"{r.gps.fix_type}, {r.gps.satellites_visible} sats{hdop}"))
    out.print(_kv_table(rows))
    if r.sensors:
        table = Table(title="Sensors", title_justify="left")
        table.add_column("Sensor")
        table.add_column("Enabled")
        table.add_column("Healthy")
        for s in r.sensors:
            healthy = Text("yes") if s.healthy else Text("NO", style="red")
            table.add_row(s.sensor, "yes" if s.enabled else "no", healthy)
        out.print(table)
    if r.recent_statustext:
        out.print(Text("Messages from vehicle (since connect):", style="bold"))
        for t in r.recent_statustext:
            out.print(Text(f"  [{t.severity}] ").append(safe(t.text)))
    if r.unavailable:
        err.print(f"unavailable: {', '.join(r.unavailable)}")
    out.print(
        Text(
            "Note: these checks do not establish that the vehicle is safe to fly.",
            style="dim",
        )
    )
    _status_line(r.status)


def _format_value(p: ParamValue) -> str:
    return format_value(p.value) + (" (~rounded)" if p.precision_limited else "")


def render_param_list(r: ParamListResult) -> None:
    table = Table(box=None, pad_edge=False)
    table.add_column("Name", style="bold")
    table.add_column("Value", justify="right")
    table.add_column("Type")
    for p in r.params:
        table.add_row(safe(p.name), _format_value(p), p.type.value)
    out.print(table)
    summary = f"{len(r.params)} shown, {r.received}/{r.total_on_vehicle} received from vehicle"
    err.print(summary)
    if r.missing_indices:
        err.print(
            Text(f"warning: {len(r.missing_indices)} parameters never arrived", style="yellow")
        )
    _status_line(r.status)


def render_param_get(r: ParamGetResult) -> None:
    out.print(Text(f"{r.param.name} = {_format_value(r.param)}  ({r.param.type.value})"))


def render_backup(r: BackupResult) -> None:
    out.print(Text(f"Saved {r.param_count} parameters to {r.path}"))
    if r.param_file:
        out.print(Text(f"Exported .param file: {r.param_file}"))
    out.print(Text(f"Backup id: {r.backup_id}"))
    if not r.complete:
        err.print(
            Text("warning: backup is INCOMPLETE; some parameters never arrived", style="yellow")
        )
    _status_line(r.status)


def render_backups(r: BackupListResult) -> None:
    if not r.backups:
        out.print(f"No backups in {r.directory}")
    else:
        table = Table(box=None, pad_edge=False)
        table.add_column("Backup id", style="bold")
        table.add_column("Created (UTC)")
        table.add_column("Vehicle")
        table.add_column("Params", justify="right")
        for b in r.backups:
            count = str(b.param_count) + ("" if b.complete else " (incomplete)")
            table.add_row(b.backup_id, f"{b.created_at:%Y-%m-%d %H:%M:%S}", b.vehicle, count)
        out.print(table)
    for name in r.unreadable:
        err.print(Text(f"warning: {name} is not a valid backup", style="yellow"))


def _names(title: str, items: list[ParamPresence], limit: int = 20) -> None:
    if not items:
        return
    shown = ", ".join(f"{p.name}={format_value(p.value)}" for p in items[:limit])
    more = f", ... and {len(items) - limit} more (use --json)" if len(items) > limit else ""
    out.print(Text(f"{title} ({len(items)}): ", style="bold").append(safe(shown + more)))


def make_render_diff(include_volatile: bool) -> Callable[[ParamDiffResult], None]:
    def render(r: ParamDiffResult) -> None:
        out.print(Text(f"Baseline: {r.reference}   Compared: {r.current}", style="bold"))
        for warning in r.warnings:
            err.print(Text(f"warning: {warning}", style="yellow"))
        changed = [c for c in r.changed if include_volatile or not c.volatile]
        hidden = len(r.changed) - len(changed)
        if changed:
            table = Table(box=None, pad_edge=False)
            table.add_column("Changed", style="bold")
            table.add_column("Baseline", justify="right")
            table.add_column("Now", justify="right")
            table.add_column("")
            for c in changed:
                note = "volatile" if c.volatile else ""
                table.add_row(
                    safe(c.name), format_value(c.reference), format_value(c.current), note
                )
            out.print(table)
        else:
            out.print("No configuration differences in shared parameters.")
        if hidden:
            out.print(
                Text(
                    f"{hidden} volatile change(s) hidden (counters, boot-time "
                    "calibration); use --all to show them.",
                    style="dim",
                )
            )
        _names("Only in baseline", r.only_in_reference)
        _names("Only in compared", r.only_in_current)
        out.print(f"Unchanged: {r.unchanged}")
        _status_line(r.status)

    return render


def render_capabilities(r: CapabilitiesResult) -> None:
    table = Table(title=f"MAVForge {r.mavforge_version}", title_justify="left")
    table.add_column("Operation")
    table.add_column("Permission")
    table.add_column("Modifies vehicle")
    table.add_column("Status")
    table.add_column("Summary")
    for op in r.operations:
        table.add_row(
            op.name,
            op.permission,
            "yes" if op.modifies_vehicle else "no",
            "available" if op.available else "planned",
            op.summary,
        )
    out.print(table)
    out.print(f"Platform adapters: {', '.join(r.platforms)} (others: generic read-only)")
    out.print(f"Granted permissions: {', '.join(r.granted_permissions)}")


# -- commands -----------------------------------------------------------------------


@app.command()
def info(ctx: typer.Context) -> None:
    """Identify the connected vehicle: autopilot, firmware, board and IDs."""
    _with_vehicle(ctx, lambda v: v.info(), render_info)


@app.command()
def status(ctx: typer.Context) -> None:
    """Show arming state, mode, sensor health, pre-arm flag, battery, GPS and messages."""
    _with_vehicle(ctx, lambda v: v.status(), render_status)


@app.command()
def capabilities(ctx: typer.Context) -> None:
    """List MAVForge operations, their permissions and availability (no vehicle needed)."""
    _emit(ctx.obj, mavforge.capabilities(), render_capabilities)


@params_app.command("list")
def params_list(
    ctx: typer.Context,
    pattern: Annotated[
        str | None, typer.Argument(help="Case-insensitive glob, e.g. 'COMPASS_*'.")
    ] = None,
) -> None:
    """Download all parameters and print them (optionally filtered)."""
    _with_vehicle(ctx, lambda v: v.params.list(pattern), render_param_list)


@params_app.command("get")
def params_get(
    ctx: typer.Context, name: Annotated[str, typer.Argument(help="Parameter name.")]
) -> None:
    """Read one parameter by name."""
    _with_vehicle(ctx, lambda v: v.params.get(name), render_param_get)


@params_app.command("backup")
def params_backup(
    ctx: typer.Context,
    output: Annotated[
        Path | None,
        typer.Option(
            "--output", "-o", help="JSON backup path (default: MAVForge backup directory)."
        ),
    ] = None,
    param_file: Annotated[
        Path | None,
        typer.Option("--param-file", help="Also export a plain .param file (NAME VALUE)."),
    ] = None,
) -> None:
    """Save all parameters, with vehicle identity and firmware, to a new backup file."""
    _with_vehicle(ctx, lambda v: v.params.backup(output, param_file=param_file), render_backup)


@params_app.command("backups")
def params_backups(ctx: typer.Context) -> None:
    """List backups in the MAVForge backup directory (no vehicle needed)."""
    _offline(ctx, mavforge.list_backups, render_backups)


@params_app.command("diff")
def params_diff(
    ctx: typer.Context,
    baseline: Annotated[
        str, typer.Argument(help="Backup id, MAVForge JSON backup, or .param file.")
    ],
    against: Annotated[
        str | None,
        typer.Option(
            "--against", help="Compare with this backup/file instead of the live vehicle."
        ),
    ] = None,
    include_volatile: Annotated[
        bool, typer.Option("--all", "-a", help="Also show volatile parameter changes.")
    ] = False,
) -> None:
    """Compare the vehicle (or another backup) with a baseline backup or .param file."""
    render = make_render_diff(include_volatile)
    if against is not None:
        _offline(ctx, lambda: mavforge.diff_backups(baseline, against), render)
    _with_vehicle(ctx, lambda v: v.params.diff(baseline), render)


def main() -> None:
    app()
