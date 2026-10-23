"""ArduPilot SITL fixtures.

SITL tests are deselected by default; run them with ``pytest -m sitl``. They need either
Docker (the ``docker/sitl`` image is built on first use) or an already-running SITL
given by ``MAVFORGE_SITL_CONNECTION`` (e.g. ``tcp:127.0.0.1:5760``).
"""

from __future__ import annotations

import os
import shutil
import socket
import subprocess
import time
import uuid
from collections.abc import Generator, Iterator
from contextlib import contextmanager
from pathlib import Path

import pytest

SITL_IMAGE = os.environ.get("MAVFORGE_SITL_IMAGE", "mavforge-sitl:4.7.2")
DOCKER_CONTEXT = Path(__file__).resolve().parents[2] / "docker" / "sitl"
BOOT_TIMEOUT = 60.0


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _docker(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["docker", *args], check=check, capture_output=True, text=True)


def _wait_for_vehicle(connection: str, timeout: float) -> None:
    """Wait until a full MAVLink session works, not merely an open TCP port.

    A bare TCP probe becomes SITL's first SERIAL0 client while it is still starting; once,
    SITL then closed the next test's connection (detected as a lost link). Waiting for a
    real heartbeat and identification avoids that start-up window.
    """
    import mavforge
    from mavforge.domain.errors import ConnectionFailedError

    deadline = time.monotonic() + timeout
    while True:
        try:
            with mavforge.connect(connection, timeout=10):
                return
        except ConnectionFailedError:
            if time.monotonic() > deadline:
                raise
            time.sleep(0.5)


@contextmanager
def _sitl_container(*args: str) -> Generator[str]:
    """Run the SITL image (optionally with arducopter arguments) and yield its link."""
    if shutil.which("docker") is None:
        pytest.skip("Docker not available and MAVFORGE_SITL_CONNECTION not set")
    if _docker("image", "inspect", SITL_IMAGE, check=False).returncode != 0:
        _docker("build", "-t", SITL_IMAGE, str(DOCKER_CONTEXT))
    port = _free_port()
    name = f"mavforge-sitl-{uuid.uuid4().hex[:8]}"
    _docker("run", "-d", "--rm", "--name", name, "-p", f"127.0.0.1:{port}:5760", SITL_IMAGE, *args)
    try:
        connection = f"tcp:127.0.0.1:{port}"
        _wait_for_vehicle(connection, BOOT_TIMEOUT)
        yield connection
    finally:
        _docker("rm", "-f", name, check=False)


@pytest.fixture(scope="session")
def sitl_connection() -> Iterator[str]:
    """Connection string for a fresh ArduCopter SITL (wiped EEPROM) for this session."""
    if external := os.environ.get("MAVFORGE_SITL_CONNECTION"):
        yield external
        return
    with _sitl_container() as connection:
        yield connection


@pytest.fixture(scope="module")
def sitl_calibration_connection() -> Iterator[str]:
    """ArduCopter SITL using the 'calibration' model, whose attitude and rotation can be
    driven through SERVO5-8 (libraries/SITL/SIM_Calibration.cpp), as ArduPilot's autotest
    does for accelerometer and compass calibration."""
    with _sitl_container(
        "--model",
        "calibration",
        "--speedup",
        "1",
        "-I0",
        "-w",
        "--home",
        "-35.363262,149.165237,584,353",
        "--defaults",
        "/opt/ardupilot/copter.parm",
    ) as connection:
        yield connection
