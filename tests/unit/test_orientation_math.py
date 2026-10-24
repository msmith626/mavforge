"""Board orientation maths, checked against ArduPilot's own rotation code."""

from __future__ import annotations

import math
from collections.abc import Callable

import pytest

from mavforge.services.calibration.orientation import (
    LABELS,
    ROTATIONS,
    Vec,
    best_rotation,
    rotation_matrix,
    rotation_name,
    solve,
    tilt_of,
)

H = math.sqrt(0.5)

# Transcribed from ArduPilot Plane-4.6.2 libraries/AP_Math/vector3.cpp, Vector3::rotate.
AP_ROTATE: dict[int, Callable[[float, float, float], Vec]] = {
    0: lambda x, y, z: (x, y, z),
    1: lambda x, y, z: (H * (x - y), H * (x + y), z),
    2: lambda x, y, z: (-y, x, z),
    4: lambda x, y, z: (-x, -y, z),
    6: lambda x, y, z: (y, -x, z),
    8: lambda x, y, z: (x, -y, -z),
    10: lambda x, y, z: (y, x, -z),
    12: lambda x, y, z: (-x, y, -z),
    14: lambda x, y, z: (-y, -x, -z),
    16: lambda x, y, z: (x, -z, y),
    20: lambda x, y, z: (x, z, -y),
    24: lambda x, y, z: (z, y, -x),
    25: lambda x, y, z: (-z, y, x),
    26: lambda x, y, z: (-y, -x, -z),
    27: lambda x, y, z: (y, x, -z),
    # ROLL_90 then PITCH_90: (x, -z, y) -> (y, -z, -x)
    28: lambda x, y, z: (y, -z, -x),
    # ROLL_90, PITCH_180, YAW_90: (x, -z, y) -> (-x, -z, -y) -> (z, -x, -y)
    36: lambda x, y, z: (z, -x, -y),
    # ROLL_90 then YAW_270: (x, -z, y) -> (-z, -x, y)
    37: lambda x, y, z: (-z, -x, y),
    38: lambda x, y, z: (
        0.14303897231223747 * x + 0.36877648650320383 * y - 0.9184463813430871 * z,
        -0.3321327777966474 * x - 0.8562894214664188 * y - 0.3955455025629652 * z,
        -0.9323238012155122 * x + 0.3616245700820924 * y,
    ),
    39: lambda x, y, z: (H * (x - z), y, H * (x + z)),
    41: lambda x, y, z: (
        0.992546151641322 * x + 0.12186934340514748 * z,
        y,
        -0.12186934340514748 * x + 0.992546151641322 * z,
    ),
    42: lambda x, y, z: (x, H * (y - z), H * (y + z)),
    43: lambda x, y, z: (x, H * (y + z), H * (z - y)),
}


@pytest.mark.parametrize("value", sorted(AP_ROTATE))
def test_rotation_matrices_match_ardupilot(value: int) -> None:
    m = rotation_matrix(value)
    for v in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.3, -0.5, 0.8)):
        ours = tuple(sum(m[r][c] * v[c] for c in range(3)) for r in range(3))
        theirs = AP_ROTATE[value](*v)
        # rotation 38 is stored to 0.1 degree in our table
        assert ours == pytest.approx(theirs, abs=2e-3 if value == 38 else 1e-9), value


def test_every_rotation_is_proper() -> None:
    for value in ROTATIONS:
        m = rotation_matrix(value)
        det = (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )
        assert det == pytest.approx(1.0), value


def test_names_follow_ardupilot() -> None:
    assert rotation_name(0) == "None"
    assert rotation_name(2) == "Yaw90"
    assert rotation_name(14) == "Yaw270Roll180"
    assert set(LABELS) == set(ROTATIONS)


def test_tilt_signs() -> None:
    s, c = 1000 * math.sin(math.radians(30)), 1000 * math.cos(math.radians(30))
    assert tilt_of((0, 0, -1000)).model_dump() == {"roll_deg": 0.0, "pitch_deg": 0.0}
    assert tilt_of((s, 0, -c)).pitch_deg == pytest.approx(30)  # nose up
    assert tilt_of((0, -s, -c)).roll_deg == pytest.approx(30)  # right side down


def body_poses(deg: float = 35) -> tuple[Vec, Vec, Vec]:
    """Specific force (mG) in the true body frame: level, nose up, right side down."""
    s, c = 1000 * math.sin(math.radians(deg)), 1000 * math.cos(math.radians(deg))
    return (0, 0, -1000), (s, 0, -c), (0, -s, -c)


def reported(mounted: int, configured: int, body: Vec) -> Vec:
    """What ArduPilot reports: the board-frame reading rotated by the configured value."""
    m, cfg = rotation_matrix(mounted), rotation_matrix(configured)
    board = tuple(sum(m[r][c] * body[r] for r in range(3)) for c in range(3))  # m^T @ body
    return tuple(sum(cfg[r][c] * board[c] for c in range(3)) for r in range(3))  # type: ignore[return-value]


@pytest.mark.parametrize(
    ("mounted", "configured"),
    [(0, 0), (2, 0), (6, 0), (4, 0), (8, 0), (16, 0), (24, 0), (0, 2), (2, 4), (42, 0), (38, 38)],
)
def test_solve_finds_the_mounting(mounted: int, configured: int) -> None:
    poses = [reported(mounted, configured, b) for b in body_poses()]
    mounting, inconsistency = solve(*poses, current=configured)
    assert inconsistency == pytest.approx(0, abs=1e-6)
    best, error = best_rotation(mounting, configured)
    assert (best, round(error, 3)) == (mounted, 0)


def test_board_arrow_right_hand_derived() -> None:
    # Independent of the matrices: arrow toward the right wing, top up, configured None.
    # Board X = body right, board Y = body back: the reading is (a_y, -a_x, a_z).
    poses = [(b[1], -b[0], b[2]) for b in body_poses()]
    best, _ = best_rotation(solve(*poses, current=0)[0], 0)
    assert best == 2  # Yaw90, as ArduPilot documents for an arrow pointing right


def test_equivalent_rotation_keeps_the_current_value() -> None:
    # 26 (Pitch180Yaw90) is the same rotation as 14 (Roll180Yaw270)
    poses = [reported(14, 26, b) for b in body_poses()]
    assert best_rotation(solve(*poses, current=26)[0], 26)[0] == 26


def test_tilting_the_wrong_side_is_inconsistent() -> None:
    level, nose_up, right_down = body_poses()
    left_down = (right_down[0], -right_down[1], right_down[2])
    assert solve(level, nose_up, left_down, current=0)[1] > 90


def test_a_slight_nose_up_mount_is_not_pitch7() -> None:
    # board pitched 6 degrees nose up: the level trim's job, not rotation 41 (Pitch7)
    s, c = math.sin(math.radians(6)), math.cos(math.radians(6))
    pitched = ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    best, error = best_rotation(pitched, 0)
    assert (best, round(error)) == (0, 6)
