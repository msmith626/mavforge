"""The ``mavforge`` command-line interface.

Every command is a one-shot call into the Python API (``mavforge.connect``), rendered
either as tables or, with ``--json``, as the same Pydantic models the API and MCP
server return. Exit codes are listed in ``mavforge --help``.
"""

from __future__ import annotations

import logging
import re
import sys
import time
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
from mavforge.domain.errors import ExitCode, MAVForgeError, NotApprovedError
from mavforge.domain.models import (
    ApplyResult,
    BackupListResult,
    BackupResult,
    CalibrationKind,
    CalibrationProcedure,
    CalibrationProcedureList,
    CalibrationState,
    CalibrationStatus,
    CapabilitiesResult,
    ChangeOutcome,
    ChangePlan,
    DiagnosticReport,
    FindingKind,
    InfoResult,
    ParamDiffResult,
    ParamGetResult,
    ParamListResult,
    ParamPresence,
    ParamValue,
    PlanListResult,
    StatusReport,
)
from mavforge.domain.outcomes import OperationStatus, exit_code_for
from mavforge.domain.values import format_value
from mavforge.policy.permissions import Policy
from mavforge.services.calibration.operations import procedures as calibration_procedures
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
calibrate_app = typer.Typer(
    help="Calibrate sensors with guided, verified workflows (asks before starting).",
    no_args_is_help=True,
)
app.add_typer(calibrate_app, name="calibrate")

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


def _status_exit_code(result: BaseModel) -> ExitCode:
    status: OperationStatus = getattr(result, "status", OperationStatus.SUCCEEDED)
    return exit_code_for(status)


def _emit[R: BaseModel](
    opts: GlobalOptions,
    result: R,
    render: Callable[[R], None],
    exit_code: Callable[[R], ExitCode] = _status_exit_code,
) -> NoReturn:
    if opts.json:
        out.print_json(result.model_dump_json())
    else:
        render(result)
    raise typer.Exit(int(exit_code(result)))


def _with_vehicle[R: BaseModel](
    ctx: typer.Context,
    action: Callable[[mavforge.Vehicle], R],
    render: Callable[[R], None],
    exit_code: Callable[[R], ExitCode] = _status_exit_code,
    policy: Policy | None = None,
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
            policy=policy,
            interface="cli",
        ) as vehicle:
            result = action(vehicle)
    except MAVForgeError as exc:
        _fail(opts, exc)
    _emit(opts, result, render, exit_code)


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


_KIND_STYLE = {
    FindingKind.FAULT: ("FAULT", "bold red"),
    FindingKind.WARNING: ("WARNING", "yellow"),
    FindingKind.SUSPECTED: ("SUSPECTED", "magenta"),
    FindingKind.NOT_CHECKED: ("NOT CHECKED", "dim"),
    FindingKind.PASSED: ("passed", "green"),
}


def make_render_diagnose(show_passed: bool) -> Callable[[DiagnosticReport], None]:
    def render(r: DiagnosticReport) -> None:
        out.print(
            Text(
                f"Diagnostics for {r.vehicle.autopilot} system {r.vehicle.system_id} "
                f"({r.vehicle.vehicle_family or r.vehicle.mav_type}) firmware "
                f"{r.vehicle.firmware or 'unknown'}",
                style="bold",
            )
        )
        for f in r.findings:
            if f.kind is FindingKind.PASSED and not show_passed:
                continue
            label, style = _KIND_STYLE[f.kind]
            line = Text(f"[{label}] ", style=style).append(Text(f.title, style="bold"))
            out.print(line)
            out.print(Text(f"    {f.detail}"))
            if f.vehicle_text:
                out.print(Text("    vehicle says: ").append(safe(f.vehicle_text)))
            if f.recommendation:
                out.print(Text(f"    -> {f.recommendation}", style="cyan"))
        counts = ", ".join(f"{n} {k.value}" for k, n in r.counts.items())
        out.print(Text(f"Summary: {counts}", style="bold"))
        if not show_passed and r.counts.get(FindingKind.PASSED):
            out.print(Text("Use --show-passed to list passed checks.", style="dim"))
        out.print(Text(r.disclaimer, style="dim"))
        _status_line(r.status)

    return render


def _value(value: float | int, label: str | None) -> str:
    return format_value(value) + (f" ({label})" if label else "")


def render_plan(plan: ChangePlan) -> None:
    """Show a plan to the human who will approve it. Goes to stderr in --json mode."""
    console = err
    v = plan.vehicle
    console.print(
        Text(
            f"Change plan {plan.plan_id} ({plan.kind}"
            + (f" from {plan.source}" if plan.source else "")
            + f") for system {v.system_id} {v.autopilot} {v.vehicle_family or v.mav_type} "
            f"firmware {v.firmware or 'unknown'}",
            style="bold",
        )
    )
    table = Table(box=None, pad_edge=False)
    for column in ("Parameter", "Current", "New", "Range", "Reboot", ""):
        table.add_column(column)
    for c in plan.changes:
        rng = f"{format_value(c.range[0])}..{format_value(c.range[1])}" if c.range else ""
        reboot = {True: "yes", False: "", None: "?"}[c.reboot_required]
        mark = Text("no change", style="dim") if c.no_change else Text("")
        if c.issues:
            mark = Text("BLOCKED", style="bold red")
        elif c.warnings:
            mark = Text("warning", style="yellow")
        table.add_row(
            Text(c.name, style="bold"),
            _value(c.current, c.current_label) if c.current is not None else "(none)",
            _value(c.requested, c.requested_label),
            rng,
            reboot,
            mark,
        )
    console.print(table)
    for c in plan.changes:
        for issue in c.issues:
            console.print(Text(f"  {c.name}: {issue}", style="red"))
        for warning in c.warnings:
            console.print(Text(f"  {c.name}: {warning}", style="yellow"))
    for warning in plan.warnings:
        style = "bold red" if warning.startswith("REAL VEHICLE") else "yellow"
        console.print(Text(f"! {warning}", style=style))


def render_plan_saved(plan: ChangePlan) -> None:
    render_plan(plan)
    if plan.blocked:
        err.print(Text("This plan is blocked and cannot be approved.", style="bold red"))
        return
    out.print(Text(f"Plan {plan.plan_id} saved. Review it, then:"))
    out.print(Text(f"  mavforge approve {plan.plan_id}"))
    out.print(Text(f"  mavforge -c <connection> params apply {plan.plan_id}"))


_OUTCOME_STYLE = {
    ChangeOutcome.VERIFIED: "green",
    ChangeOutcome.UNCHANGED: "dim",
    ChangeOutcome.FAILED: "bold red",
    ChangeOutcome.UNKNOWN: "bold yellow",
    ChangeOutcome.SKIPPED: "yellow",
    ChangeOutcome.NOT_ATTEMPTED: "yellow",
}


def render_apply(r: ApplyResult) -> None:
    table = Table(box=None, pad_edge=False)
    for column in ("Parameter", "Before", "Requested", "After", "Result", "Detail"):
        table.add_column(column)
    for c in r.changes:
        table.add_row(
            Text(c.name, style="bold"),
            format_value(c.before) if c.before is not None else "?",
            format_value(c.requested),
            format_value(c.after) if c.after is not None else "?",
            Text(c.outcome.value, style=_OUTCOME_STYLE[c.outcome]),
            c.detail or "",
        )
    out.print(table)
    out.print(
        Text(
            f"Result: {r.status.value} (plan {r.plan_id}, approved via {r.approved_by})",
            style="bold",
        )
    )
    if r.pre_change_backup:
        out.print(Text(f"Backup taken before the change: {r.pre_change_backup}"))
        out.print(
            Text(
                f"  undo with: mavforge -c <connection> params restore {r.pre_change_backup}",
                style="dim",
            )
        )
    if r.reboot_required:
        out.print(
            Text(
                f"Reboot the vehicle for these to take effect: {', '.join(r.reboot_required)}",
                style="yellow",
            )
        )
    for warning in r.warnings:
        err.print(Text(f"warning: {warning}", style="yellow"))


def render_plans(r: PlanListResult) -> None:
    if not r.plans:
        out.print("No change plans.")
        return
    table = Table(box=None, pad_edge=False)
    for column in ("Plan", "Created (UTC)", "Kind", "Vehicle", "Changes", "State"):
        table.add_column(column)
    for p in r.plans:
        state = (
            "applied"
            if p.applied
            else "blocked"
            if p.blocked
            else "approved"
            if p.approved
            else "pending"
        )
        table.add_row(
            p.plan_id,
            f"{p.created_at:%Y-%m-%d %H:%M:%S}",
            p.kind,
            p.vehicle,
            str(p.change_count),
            state,
        )
    out.print(table)


def _confirm_and_approve(plan: ChangePlan, yes: bool) -> None:
    """Show the plan and record the human's approval, or raise NotApprovedError."""
    render_plan(plan)
    if plan.blocked:
        raise NotApprovedError(f"Plan {plan.plan_id} has blocking issues; nothing was changed")
    if all(c.no_change for c in plan.changes):
        raise NotApprovedError("Every requested value is already set; nothing to change")
    if not yes:
        count = sum(not c.no_change for c in plan.changes)
        real = plan.vehicle.simulated is not True
        question = f"Write {count} parameter(s) to the {'REAL ' if real else ''}vehicle?"
        try:
            confirmed = typer.confirm(question, default=False, err=True)
        except typer.Abort:
            confirmed = False
        if not confirmed:
            raise NotApprovedError(f"Plan {plan.plan_id} was not approved; nothing was changed")
    mavforge.approve_plan(plan, approved_by="cli-yes-flag" if yes else "cli-prompt")


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


@app.command()
def diagnose(
    ctx: typer.Context,
    show_passed: Annotated[
        bool, typer.Option("--show-passed", help="Also list checks that passed.")
    ] = False,
    prearm_wait: Annotated[
        float,
        typer.Option(min=0.5, max=30, help="Seconds to collect pre-arm messages."),
    ] = 3.0,
    fail_on: Annotated[
        str | None,
        typer.Option(
            "--fail-on",
            help="Exit 1 if any finding is at least this serious: 'fault' or 'warning'.",
        ),
    ] = None,
) -> None:
    """Find configuration and readiness problems, classified by certainty (read-only)."""
    if fail_on not in (None, "fault", "warning"):
        raise typer.BadParameter("must be 'fault' or 'warning'", param_hint="--fail-on")
    serious = {"fault": {FindingKind.FAULT}, "warning": {FindingKind.FAULT, FindingKind.WARNING}}

    def exit_code(report: DiagnosticReport) -> ExitCode:
        if fail_on and any(f.kind in serious[fail_on] for f in report.findings):
            return ExitCode.FAILED
        return _status_exit_code(report)

    _with_vehicle(
        ctx,
        lambda v: v.diagnose(prearm_wait=prearm_wait),
        make_render_diagnose(show_passed),
        exit_code,
    )


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


_CALIBRATE_POLICY = Policy.calibrate(allow_real_vehicle=True)


def render_procedure(proc: CalibrationProcedure, console: Console = out) -> None:
    support = "" if proc.supported else f"  (not available: {proc.reason})"
    console.print(Text(f"{proc.title} [{proc.kind.value}]{support}", style="bold"))
    console.print(Text(f"  {proc.summary} Takes {proc.typical_duration}."))
    console.print(Text("  Before you start:", style="bold"))
    for item in proc.preconditions:
        console.print(Text(f"    - {item}"))
    console.print(Text("  You will:", style="bold"))
    for item in proc.human_steps:
        console.print(Text(f"    - {item}"))
    console.print(Text(f"  Changes: {', '.join(proc.changes_parameters)}"))
    console.print(Text(f"  Success means: {proc.success_evidence}", style="dim"))
    console.print(Text(f"  Cancelling: {proc.cancellation}", style="dim"))


def render_procedures(procs: list[CalibrationProcedure]) -> None:
    for proc in procs:
        render_procedure(proc)
        out.print()


def render_calibration(st: CalibrationStatus) -> None:
    r = st.result
    if r is None:
        out.print(Text(f"Calibration {st.operation_id} is {st.state.value}."))
        return
    style = "green" if r.status is OperationStatus.SUCCEEDED else "bold red"
    out.print(Text(f"{st.kind.value} calibration: {r.status.value}", style=style))
    out.print(Text(r.summary))
    for item in r.evidence:
        out.print(Text(f"  evidence: {item}", style="dim"))
    changed = {
        k: (r.params_before.get(k), v)
        for k, v in r.params_after.items()
        if r.params_before.get(k) is None or format_value(r.params_before[k]) != format_value(v)
    }
    if changed:
        table = Table(box=None, pad_edge=False)
        for column in ("Parameter", "Before", "After"):
            table.add_column(column)
        for name, (before, after) in changed.items():
            table.add_row(
                name, format_value(before) if before is not None else "?", format_value(after)
            )
        out.print(table)
    for warning in r.warnings:
        out.print(Text(f"! {warning}", style="bold yellow"))


def _calibration_exit(st: CalibrationStatus) -> ExitCode:
    return exit_code_for(st.result.status) if st.result else ExitCode.TIMEOUT


def _progress_line(st: CalibrationStatus) -> str:
    line = st.instruction or st.state.value
    if st.compasses:
        line += "  " + "  ".join(
            f"compass {c.compass_id}: "
            f"{c.completion_pct if c.completion_pct is not None else '?'}% {c.status}"
            for c in st.compasses
        )
    return line


def _run_calibration(
    vehicle: mavforge.Vehicle, kind: CalibrationKind, yes: bool
) -> CalibrationStatus:
    proc = next(p for p in vehicle.calibration.procedures() if p.kind is kind)
    render_procedure(proc, err)
    if not proc.supported:
        raise NotApprovedError(f"{proc.title} is not available: {proc.reason}")
    request = vehicle.calibration.request(kind)
    if not yes:
        real = "REAL " if vehicle.identity.simulated is not True else ""
        try:
            ok = typer.confirm(
                f"Start {kind.value} calibration on the {real}vehicle?", default=False, err=True
            )
        except typer.Abort:
            ok = False
        if not ok:
            raise NotApprovedError("Calibration not started")
    mavforge.approve_plan(request, approved_by="cli-yes-flag" if yes else "cli-prompt")
    st = vehicle.calibration.start(request)
    op = st.operation_id
    last_line = ""
    try:
        while not st.state.finished:
            if (line := _progress_line(st)) != last_line:
                err.print(Text(line))
                last_line = line
            if st.state is CalibrationState.WAITING_FOR_USER and st.awaiting_confirmation:
                typer.prompt(
                    "Press Enter when the vehicle is in position (Ctrl-C to cancel)",
                    default="",
                    show_default=False,
                    err=True,
                )
                st = vehicle.calibration.confirm(op, st.awaiting_confirmation)
                continue
            time.sleep(0.25)
            st = vehicle.calibration.status(op)
    except (KeyboardInterrupt, typer.Abort):
        err.print(Text("Cancelling...", style="yellow"))
        vehicle.calibration.cancel(op)
    return vehicle.calibration.wait(op, 30)


def _calibrate_command(kind: CalibrationKind) -> Callable[..., None]:
    def command(
        ctx: typer.Context,
        yes: Annotated[bool, typer.Option("--yes", "-y", help="Start without asking.")] = False,
    ) -> None:
        _with_vehicle(
            ctx,
            lambda v: _run_calibration(v, kind, yes),
            render_calibration,
            _calibration_exit,
            policy=_CALIBRATE_POLICY,
        )

    proc = next(p for p in calibration_procedures() if p.kind is kind)
    command.__doc__ = f"{proc.title}: {proc.summary}"
    return command


for _kind in CalibrationKind:
    calibrate_app.command(_kind.value)(_calibrate_command(_kind))


@calibrate_app.command("list")
def calibrate_list(ctx: typer.Context) -> None:
    """Describe every calibration: preconditions, steps, effects (no vehicle needed)."""
    _offline(
        ctx,
        lambda: CalibrationProcedureList(procedures=calibration_procedures()),
        lambda r: render_procedures(r.procedures),
    )


_WRITE_POLICY = Policy.configure(allow_real_vehicle=True)
"""CLI write commands act for the human at the terminal, who reviews and confirms every
plan (marked REAL VEHICLE when applicable) before anything is written."""


@params_app.command("set")
def params_set(
    ctx: typer.Context,
    assignments: Annotated[list[str], typer.Argument(help="One or more NAME=VALUE.")],
    yes: Annotated[bool, typer.Option("--yes", "-y", help="Approve without prompting.")] = False,
    allow_out_of_range: Annotated[
        bool, typer.Option(help="Allow values outside the documented range or value list.")
    ] = False,
    offline: Annotated[bool, typer.Option(help="Do not download parameter metadata.")] = False,
) -> None:
    """Change parameters: show the plan, ask for approval, write and verify each value."""
    requested = _parse_or_fail(ctx, assignments)

    def act(vehicle: mavforge.Vehicle) -> ApplyResult:
        plan = vehicle.params.plan(
            requested, allow_out_of_range=allow_out_of_range, offline=offline
        )
        _confirm_and_approve(plan, yes)
        return vehicle.params.apply(plan)

    _with_vehicle(ctx, act, render_apply, policy=_WRITE_POLICY)


@params_app.command("plan")
def params_plan(
    ctx: typer.Context,
    assignments: Annotated[list[str], typer.Argument(help="One or more NAME=VALUE.")],
    allow_out_of_range: Annotated[
        bool, typer.Option(help="Allow values outside the documented range or value list.")
    ] = False,
    offline: Annotated[bool, typer.Option(help="Do not download parameter metadata.")] = False,
) -> None:
    """Validate changes and save a plan for later approval and apply (changes nothing)."""
    requested = _parse_or_fail(ctx, assignments)
    _with_vehicle(
        ctx,
        lambda v: v.params.plan(requested, allow_out_of_range=allow_out_of_range, offline=offline),
        render_plan_saved,
    )


@params_app.command("restore")
def params_restore(
    ctx: typer.Context,
    backup: Annotated[str, typer.Argument(help="Backup id, MAVForge JSON backup, or .param file.")],
    yes: Annotated[bool, typer.Option("--yes", "-y", help="Approve without prompting.")] = False,
    allow_mismatch: Annotated[
        bool,
        typer.Option(help="Restore even if the backup's vehicle/firmware differs (dangerous)."),
    ] = False,
    offline: Annotated[bool, typer.Option(help="Do not download parameter metadata.")] = False,
) -> None:
    """Write back every non-volatile value that differs from a backup, then verify."""

    def act(vehicle: mavforge.Vehicle) -> ApplyResult:
        plan = vehicle.params.plan_restore(backup, allow_mismatch=allow_mismatch, offline=offline)
        _confirm_and_approve(plan, yes)
        return vehicle.params.apply(plan)

    _with_vehicle(ctx, act, render_apply, policy=_WRITE_POLICY)


@params_app.command("apply")
def params_apply(
    ctx: typer.Context,
    plan_ref: Annotated[str, typer.Argument(metavar="PLAN", help="Plan id or plan file.")],
    yes: Annotated[bool, typer.Option("--yes", "-y", help="Approve without prompting.")] = False,
) -> None:
    """Apply a saved plan (asks for approval unless it was already approved)."""

    def act(vehicle: mavforge.Vehicle) -> ApplyResult:
        plan = mavforge.load_plan(plan_ref)
        if not _is_approved(plan):
            _confirm_and_approve(plan, yes)
        return vehicle.params.apply(plan)

    _with_vehicle(ctx, act, render_apply, policy=_WRITE_POLICY)


@params_app.command("plans")
def params_plans(ctx: typer.Context) -> None:
    """List change plans and whether they are pending, approved or applied."""
    _offline(ctx, mavforge.list_plans, render_plans)


@app.command()
def approve(
    ctx: typer.Context,
    plan_ref: Annotated[str, typer.Argument(metavar="PLAN", help="Plan id or plan file.")],
    yes: Annotated[bool, typer.Option("--yes", "-y", help="Approve without prompting.")] = False,
) -> None:
    """Review and approve a change plan, e.g. one proposed by an AI agent over MCP.

    The approval is bound to that exact plan and expires after 10 minutes.
    """

    def act() -> ChangePlan:
        plan = mavforge.load_plan(plan_ref)
        _confirm_and_approve(plan, yes)
        out.print(Text(f"Approved plan {plan.plan_id} (valid for 10 minutes).", style="green"))
        return plan

    _offline(ctx, act, lambda _: None)


def _is_approved(plan: ChangePlan) -> bool:
    from mavforge.services.plan_store import valid_approval

    return valid_approval(plan) is not None


def _parse_or_fail(ctx: typer.Context, assignments: list[str]) -> dict[str, float]:
    from mavforge.services.changes import parse_assignments

    try:
        return parse_assignments(assignments)
    except MAVForgeError as exc:
        _fail(ctx.obj, exc)


def main() -> None:
    app()
