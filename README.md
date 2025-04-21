# 🧩 Maze Solver AI

Interactive visualization of classical AI search algorithms on grid-based mazes using `pygame`.

## 🧭 Overview

The solver compares uninformed and heuristic-based search strategies for 4-directional maze navigation with unit step cost.

## ⚙️ Algorithms

| CLI Name | Algorithm | Behavior |
|---|---|---|
| `BFS` | Breadth-First Search | Finds the shortest path in unweighted mazes |
| `DFS` | Depth-First Search | Explores deeply with visited-state tracking |
| `IDS` | Iterative Deepening Search | Runs depth-limited DFS with increasing limits |
| `UCS` | Uniform Cost Search | Expands by cumulative path cost |
| `GBFS` | Greedy Best-First Search | Expands by Manhattan-distance heuristic |
| `A_Star` | A* Search | Uses `f(n) = g(n) + h(n)` |

## 🚀 Usage

```bash
pip install -r requirements.txt
python main.py -a A_Star -m 5 -s "1,2" -g "18,17"
```

## 🧪 Tests

```bash
python -m pytest -q
```

The test suite validates path correctness and optimality guarantees for BFS, UCS, and A*.

## 📂 Project Structure

```text
maze-solver-ai/
├── Algorithm.py
├── game.py
├── main.py
├── maze_generator.py
├── mazes/
└── tests/
```