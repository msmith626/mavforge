"""band_of(): pure function, no vehicle needed.

Breakpoints confirmed directly against ArduPilot source (RC_Channel::read_6pos_switch,
libraries/RC_Channel/RC_Channel.cpp, tag Plane-4.6.2) - see docs/research.md, "Flight
modes and failsafes". Not inferred from the nominal centre PWM values the wiki quotes.
"""

import itertools

import pytest

from mavforge.services.calibration.flight_modes import band_of


@pytest.mark.parametrize(
    ("pwm", "band"),
    [
        (800, 0),
        (1230, 0),
        (1231, 1),  # exact lower breakpoint of band 1
        (1295, 1),  # nominal centre of position 2
        (1360, 1),
        (1361, 2),
        (1425, 2),  # nominal centre of position 3
        (1490, 2),
        (1491, 3),
        (1555, 3),  # nominal centre of position 4
        (1620, 3),
        (1621, 4),
        (1685, 4),  # nominal centre of position 5
        (1749, 4),
        (1750, 5),  # exact upper breakpoint into band 5
        (1815, 5),  # nominal centre of position 6
        (2200, 5),
    ],
)
def test_band_of_matches_read_6pos_switch_breakpoints(pwm: int, band: int) -> None:
    assert band_of(pwm) == band


def test_bands_are_contiguous_and_cover_every_pwm_from_800_to_2200() -> None:
    bands = [band_of(pwm) for pwm in range(800, 2201)]
    assert set(bands) == {0, 1, 2, 3, 4, 5}
    # Non-decreasing: PWM only ever moves to the same or next band as it rises.
    assert all(b <= nxt for b, nxt in itertools.pairwise(bands))
