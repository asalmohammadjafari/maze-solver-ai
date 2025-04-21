import numpy as np
import pytest

from Algorithm import (
    A_Star_Algorithm,
    BFS_Algorithm,
    DFS_Algorithm,
    Greedy_Best_First_Algorithm,
    IDS_Algorithm,
    Uniform_Cost_Search_Algorithm,
)


ALL_ALGORITHMS = [
    BFS_Algorithm,
    DFS_Algorithm,
    IDS_Algorithm,
    Uniform_Cost_Search_Algorithm,
    Greedy_Best_First_Algorithm,
    A_Star_Algorithm,
]


def _prepare_grid(raw_grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> np.ndarray:
    grid = raw_grid.copy()
    grid[start] = 2
    grid[goal] = 3
    return grid


def _run_algorithm(algorithm_cls, raw_grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int]):
    grid = _prepare_grid(raw_grid, start, goal)
    solver = algorithm_cls(start_pos=start, goal_pos=goal, grid_dim=(grid.shape[0] - 1, grid.shape[1] - 1))
    path, done, _ = solver.update(grid)
    return path, done


def assert_valid_path(path: list[tuple[int, int]], raw_grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> None:
    assert path, "Path should not be empty."
    assert path[0] == start, "Path must start at the start node."
    assert path[-1] == goal, "Path must end at the goal node."

    for i in range(1, len(path)):
        prev_pos = path[i - 1]
        curr_pos = path[i]
        manhattan_step = abs(prev_pos[0] - curr_pos[0]) + abs(prev_pos[1] - curr_pos[1])
        assert manhattan_step == 1, "Each move must go to an adjacent cell."
        assert raw_grid[curr_pos] != 0, "Path cannot pass through walls."


def _shortest_distance(raw_grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> int | None:
    from collections import deque

    if start == goal:
        return 0

    queue = deque([(start, 0)])
    visited = {start}

    while queue:
        (x, y), dist = queue.popleft()
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if not (0 <= nx < raw_grid.shape[0] and 0 <= ny < raw_grid.shape[1]):
                continue
            if raw_grid[nx, ny] == 0 or (nx, ny) in visited:
                continue
            if (nx, ny) == goal:
                return dist + 1
            visited.add((nx, ny))
            queue.append(((nx, ny), dist + 1))

    return None


def test_all_algorithms_find_valid_path_on_maze_with_multiple_paths():
    raw_grid = np.array(
        [
            [1, 1, 1, 1, 1],
            [0, 0, 1, 0, 1],
            [1, 1, 1, 0, 1],
            [1, 0, 0, 0, 1],
            [1, 1, 1, 1, 1],
        ],
        dtype=int,
    )
    start, goal = (0, 0), (4, 4)

    for algorithm_cls in ALL_ALGORITHMS:
        path, done = _run_algorithm(algorithm_cls, raw_grid, start, goal)
        assert done is True
        assert_valid_path(path, raw_grid, start, goal)


def test_bfs_returns_shortest_path_in_unweighted_maze():
    raw_grid = np.array(
        [
            [1, 1, 1, 1, 1],
            [0, 0, 1, 0, 1],
            [1, 1, 1, 0, 1],
            [1, 0, 0, 0, 1],
            [1, 1, 1, 1, 1],
        ],
        dtype=int,
    )
    start, goal = (0, 0), (4, 4)

    path, done = _run_algorithm(BFS_Algorithm, raw_grid, start, goal)
    assert done is True
    assert_valid_path(path, raw_grid, start, goal)

    shortest = _shortest_distance(raw_grid, start, goal)
    assert shortest is not None
    assert len(path) - 1 == shortest


def test_ucs_uses_cumulative_cost_and_matches_shortest_path():
    raw_grid = np.array(
        [
            [1, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 0, 1],
            [1, 0, 0, 1, 0, 1],
            [1, 1, 1, 1, 1, 1],
        ],
        dtype=int,
    )
    start, goal = (0, 0), (4, 5)

    ucs_path, done = _run_algorithm(Uniform_Cost_Search_Algorithm, raw_grid, start, goal)
    assert done is True
    assert_valid_path(ucs_path, raw_grid, start, goal)

    bfs_path, _ = _run_algorithm(BFS_Algorithm, raw_grid, start, goal)
    assert len(ucs_path) == len(bfs_path)


def test_a_star_uses_g_plus_h_and_matches_bfs_optimality():
    raw_grid = np.array(
        [
            [1, 1, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 1, 0, 1],
            [1, 1, 1, 0, 1, 1, 1],
            [1, 0, 1, 1, 1, 0, 0],
            [1, 1, 1, 1, 1, 1, 1],
        ],
        dtype=int,
    )
    start, goal = (0, 0), (6, 6)

    astar_path, done = _run_algorithm(A_Star_Algorithm, raw_grid, start, goal)
    assert done is True
    assert_valid_path(astar_path, raw_grid, start, goal)

    bfs_path, _ = _run_algorithm(BFS_Algorithm, raw_grid, start, goal)
    assert len(astar_path) == len(bfs_path)


def test_ids_returns_valid_solution_and_terminates():
    raw_grid = np.array(
        [
            [1, 1, 1, 1],
            [1, 0, 0, 1],
            [1, 1, 1, 1],
            [1, 0, 1, 1],
        ],
        dtype=int,
    )
    start, goal = (0, 0), (3, 3)

    path, done = _run_algorithm(IDS_Algorithm, raw_grid, start, goal)
    assert done is True
    assert_valid_path(path, raw_grid, start, goal)


def test_no_path_exists_for_all_algorithms():
    raw_grid = np.array(
        [
            [1, 0, 1],
            [0, 0, 0],
            [1, 0, 1],
        ],
        dtype=int,
    )
    start, goal = (0, 0), (2, 2)

    for algorithm_cls in ALL_ALGORITHMS:
        path, done = _run_algorithm(algorithm_cls, raw_grid, start, goal)
        assert done is True
        assert path == []


def test_start_equals_goal_for_all_algorithms():
    raw_grid = np.array([[1, 1], [1, 1]], dtype=int)
    start = goal = (0, 0)

    for algorithm_cls in ALL_ALGORITHMS:
        path, done = _run_algorithm(algorithm_cls, raw_grid, start, goal)
        assert done is True
        assert path == [start]


def test_small_single_cell_maze_for_all_algorithms():
    raw_grid = np.array([[1]], dtype=int)
    start = goal = (0, 0)

    for algorithm_cls in ALL_ALGORITHMS:
        path, done = _run_algorithm(algorithm_cls, raw_grid, start, goal)
        assert done is True
        assert path == [start]


@pytest.mark.parametrize("algorithm_cls", ALL_ALGORITHMS)
def test_blocked_cells_are_never_in_path(algorithm_cls):
    raw_grid = np.array(
        [
            [1, 1, 0, 1],
            [0, 1, 0, 1],
            [1, 1, 1, 1],
            [1, 0, 0, 1],
        ],
        dtype=int,
    )
    start, goal = (0, 0), (3, 3)

    path, done = _run_algorithm(algorithm_cls, raw_grid, start, goal)
    assert done is True
    assert_valid_path(path, raw_grid, start, goal)
