import os
import time

import numpy as np
import pygame
from Algorithm import (
    A_Star_Geometric_Algorithm,
    DFS_Algorithm,
    BFS_Algorithm,
    IDS_Algorithm,
    Uniform_Cost_Search_Algorithm,
    Greedy_Best_First_Algorithm,
    A_Star_Algorithm,
)


def _in_bounds(pos, num_rows, num_columns):
    return 0 <= pos[0] < num_rows and 0 <= pos[1] < num_columns


def _resolve_position(pos, grid):
    """
    Resolve CLI coordinates against grid indexing.

    Primary convention is (row, col). For backward compatibility with historical
    CLI usage, if that lands on a wall but swapped coordinates are open and in
    bounds, fallback to swapped (col, row -> row, col).
    """

    num_rows, num_columns = grid.shape
    direct = (int(pos[0]), int(pos[1]))
    swapped = (int(pos[1]), int(pos[0]))

    direct_in_bounds = _in_bounds(direct, num_rows, num_columns)
    swapped_in_bounds = _in_bounds(swapped, num_rows, num_columns)

    if direct_in_bounds and grid[*direct] != 0:
        return direct, False

    if swapped_in_bounds and grid[*swapped] != 0:
        return swapped, swapped != direct

    if direct_in_bounds:
        return direct, False
    if swapped_in_bounds:
        return swapped, swapped != direct
    raise IndexError


def solve_maze(map_address, algorithm, start_pos=None, goal_pos=None):
    grid = np.genfromtxt(map_address, delimiter=",", dtype=int)
    num_rows, num_columns = grid.shape[0], grid.shape[1]
    empty_block_count = np.count_nonzero(grid == 1)

    # Define start & goal positions
    start_pos = (0, 0) if start_pos is None else start_pos
    goal_pos = (num_rows - 1, num_columns - 1) if goal_pos is None else goal_pos
    try:
        start_pos, start_swapped = _resolve_position(start_pos, grid)
        goal_pos, goal_swapped = _resolve_position(goal_pos, grid)
        if start_swapped or goal_swapped:
            print(
                "Note: Interpreting coordinates using compatibility mode "
                "(col,row -> row,col)."
            )
        if grid[*start_pos] == 0 or grid[*goal_pos] == 0:
            print("Error: Start or goal position is blocked by a wall.")
            return None
        grid[*start_pos] = 2
        grid[*goal_pos] = 3
    except IndexError:
        print("Error: Start or goal position is out of the maze!")
        return None
    grid_dim = (num_rows - 1, num_columns - 1)

    black, white, green, red, grey, blue, magenta = (
        (0, 0, 0),
        (255, 255, 255),
        (50, 205, 50),
        (255, 99, 71),
        (211, 211, 211),
        (0, 94, 255),
        (255, 0, 255),
    )
    idx_to_color = [black, white, green, red, blue, magenta]

    window_size = [min(800, num_columns * 15), min(800, num_rows * 15)]
    margin = 1
    height = (window_size[1] - margin * (num_rows - 1)) / num_rows
    width = (window_size[0] - margin * (num_columns - 1)) / num_columns

    pygame.init()

    screen = pygame.display.set_mode(window_size)

    pygame.display.set_caption(f"{algorithm} Pathfinder. Solving: {map_address}")

    headless_mode = os.environ.get("SDL_VIDEODRIVER") == "dummy"
    done = False
    run = headless_mode
    close = False
    start_t0 = time.time() if run else None

    clock = pygame.time.Clock()

    digital_twin = None
    solution = []

    if algorithm == "BFS":
        digital_twin = BFS_Algorithm(
            start_pos=start_pos, goal_pos=goal_pos, grid_dim=grid_dim
        )
    elif algorithm == "DFS":
        digital_twin = DFS_Algorithm(
            start_pos=start_pos, goal_pos=goal_pos, grid_dim=grid_dim
        )
    elif algorithm == "IDS":
        digital_twin = IDS_Algorithm(
            start_pos=start_pos, goal_pos=goal_pos, grid_dim=grid_dim
        )
    elif algorithm == "A_Star":
        digital_twin = A_Star_Algorithm(
            start_pos=start_pos, goal_pos=goal_pos, grid_dim=grid_dim
        )
    elif algorithm == "UCS":
        digital_twin = Uniform_Cost_Search_Algorithm(
            start_pos=start_pos, goal_pos=goal_pos, grid_dim=grid_dim
        )
    elif algorithm == "GBFS":
        digital_twin = Greedy_Best_First_Algorithm(
            start_pos=start_pos, goal_pos=goal_pos, grid_dim=grid_dim
        )
    elif algorithm == "ASG":
        digital_twin = A_Star_Geometric_Algorithm(
            start_pos=start_pos, goal_pos=goal_pos, grid_dim=grid_dim
        )
    else:
        return None

    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

            elif event.type == pygame.KEYDOWN:
                run = True
                start_t0 = time.time()

        screen.fill(grey)

        for row in range(num_rows):
            for column in range(num_columns):
                color = idx_to_color[grid[row, column]]
                pygame.draw.rect(
                    screen,
                    color,
                    [
                        (margin + width) * column + margin,
                        (margin + height) * row + margin,
                        width,
                        height,
                    ],
                )

        clock.tick(60)
        pygame.display.flip()

        if run:
            solution, done, grid = digital_twin.update(grid=grid)

        if done:
            print(f"Total empty block numbers: {empty_block_count}")
            print(f"Explored block numbers: {np.count_nonzero(grid == 4)}")
            for pos in solution:
                if pos not in (start_pos, goal_pos):
                    grid[pos[0], pos[1]] = 5

            screen.fill(grey)

            for row in range(num_rows):
                for column in range(num_columns):
                    color = idx_to_color[grid[row, column]]
                    pygame.draw.rect(
                        screen,
                        color,
                        [
                            (margin + width) * column + margin,
                            (margin + height) * row + margin,
                            width,
                            height,
                        ],
                    )

            clock.tick(60)
            pygame.display.flip()

    if solution:
        print(f"Your maze solved with {algorithm} algorithm.")
    else:
        print(f"No path found with {algorithm} algorithm.")
    if start_t0 is not None:
        print(f"--- finished {time.time()-start_t0:.3f} s---")
    if not headless_mode:
        while not close:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    close = True

                elif event.type == pygame.KEYDOWN:
                    close = True
    pygame.quit()
