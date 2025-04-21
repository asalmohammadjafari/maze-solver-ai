import numpy as np

from game import _resolve_position


def test_resolve_position_prefers_row_col_when_open():
    grid = np.array(
        [
            [1, 1, 1],
            [1, 1, 0],
            [1, 0, 1],
        ],
        dtype=int,
    )
    assert _resolve_position((1, 1), grid) == ((1, 1), False)


def test_resolve_position_falls_back_to_col_row_when_direct_blocked():
    grid = np.array(
        [
            [1, 1, 1],
            [1, 0, 0],
            [1, 1, 1],
        ],
        dtype=int,
    )
    # direct (1,2) is blocked, swapped (2,1) is open
    assert _resolve_position((1, 2), grid) == ((2, 1), True)


def test_resolve_position_uses_swapped_when_direct_out_of_bounds():
    grid = np.array(
        [
            [1, 1, 1],
            [1, 1, 1],
        ],
        dtype=int,
    )
    # direct (2,1) out of bounds for rows, swapped (1,2) valid
    assert _resolve_position((2, 1), grid) == ((1, 2), True)
