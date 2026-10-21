"""Opt-in tests against a real flight controller.

Run only when you intend to, with the vehicle disarmed and props removed:

    MAVFORGE_HW_CONNECTION=/dev/ttyACM0 uv run pytest -m hardware

These tests are read-only. They never write parameters, calibrate, arm or reboot.
"""

from __future__ import annotations

import os

import pytest


@pytest.fixture(scope="session")
def hw_connection() -> str:
    connection = os.environ.get("MAVFORGE_HW_CONNECTION")
    if not connection:
        pytest.skip("Set MAVFORGE_HW_CONNECTION to run hardware tests")
    return connection
