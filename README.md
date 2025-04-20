# MazeSolver-AI

This project implements AI algorithms (BFS, DFS, IDS, A*, A* Geometric) to solve mazes. It includes a maze generator with obstacles and uses **pygame** to visualize the solution.

## Installation
Run the following to install dependencies:

```bash
pip install -r requirements.txt
```

## Usage

Run the solver with:

```bash
python main.py -a <algorithm> -m <map_number> -s <start_pos> -g <goal_pos>
```

### Example:

```bash
python main.py -a A_Star -m 3 -s '1,2' -g '6,4'
```

## Maze Generation

Generate random mazes with obstacles using `maze_generator.py`:

```bash
python maze_generator.py
```

## Algorithms

- **BFS**: Explores neighbors level by level (shortest path).
- **DFS**: Explores as far as possible before backtracking.
- **IDS**: Runs DFS with increasing depth limits.
- **A* Search**: Uses Euclidean distance for heuristic-based search.
- **A* Geometric**: Uses Manhattan distance for grids with only horizontal/vertical movement.

## Visualization

The maze solver uses **pygame** to visualize the path, coloring grid cells based on their state (empty, obstacle, start, goal, or solution).
