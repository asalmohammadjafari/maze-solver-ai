from __future__ import annotations

import heapq
from collections import deque
from math import inf


Position = tuple[int, int]


class _SearchBase:
    """Shared utilities for grid-based search algorithms."""

    def __init__(self, start_pos: Position, goal_pos: Position, grid_dim: Position):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim

    def get_successors(self, x: int, y: int) -> list[Position]:
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

    def is_valid_cell(self, pos: Position) -> bool:
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def _is_walkable(self, grid, pos: Position) -> bool:
        return self.is_valid_cell(pos) and grid[pos[0], pos[1]] in (1, 2, 3, 4, 5)

    def _mark_explored(self, grid, pos: Position) -> None:
        if pos not in (self.start_pos, self.goal_pos) and grid[pos[0], pos[1]] == 1:
            grid[pos[0], pos[1]] = 4

    def _build_path(self, parent: dict[Position, Position | None], end: Position) -> list[Position]:
        path: list[Position] = []
        cursor: Position | None = end
        while cursor is not None:
            path.append(cursor)
            cursor = parent[cursor]
        path.reverse()
        return path

    def _start_equals_goal(self, grid):
        if self.start_pos == self.goal_pos:
            return [self.start_pos], True, grid
        return None


class BFS_Algorithm(_SearchBase):
    def update(self, grid):
        trivial = self._start_equals_goal(grid)
        if trivial is not None:
            return trivial

        queue = deque([self.start_pos])
        visited = {self.start_pos}
        parent: dict[Position, Position | None] = {self.start_pos: None}

        while queue:
            current = queue.popleft()
            if current == self.goal_pos:
                return self._build_path(parent, current), True, grid

            for successor in self.get_successors(*current):
                if successor in visited or not self._is_walkable(grid, successor):
                    continue
                visited.add(successor)
                parent[successor] = current
                queue.append(successor)

            self._mark_explored(grid, current)

        return [], True, grid


class DFS_Algorithm(_SearchBase):
    def update(self, grid):
        trivial = self._start_equals_goal(grid)
        if trivial is not None:
            return trivial

        stack = [self.start_pos]
        visited = {self.start_pos}
        parent: dict[Position, Position | None] = {self.start_pos: None}

        while stack:
            current = stack.pop()
            if current == self.goal_pos:
                return self._build_path(parent, current), True, grid

            # Reverse order keeps traversal deterministic with stack LIFO behavior.
            for successor in reversed(self.get_successors(*current)):
                if successor in visited or not self._is_walkable(grid, successor):
                    continue
                visited.add(successor)
                parent[successor] = current
                stack.append(successor)

            self._mark_explored(grid, current)

        return [], True, grid


class IDS_Algorithm(_SearchBase):
    def _depth_limited_search(self, grid, depth_limit: int):
        explored: set[Position] = set()
        best_remaining_depth: dict[Position, int] = {}

        def dfs(node: Position, depth: int, active_path: set[Position]) -> list[Position] | None:
            explored.add(node)
            if node == self.goal_pos:
                return [node]
            if depth >= depth_limit:
                return None

            remaining_depth = depth_limit - depth
            if best_remaining_depth.get(node, -1) >= remaining_depth:
                return None
            best_remaining_depth[node] = remaining_depth

            for successor in self.get_successors(*node):
                if not self._is_walkable(grid, successor) or successor in active_path:
                    continue
                active_path.add(successor)
                suffix = dfs(successor, depth + 1, active_path)
                if suffix is not None:
                    return [node] + suffix
                active_path.remove(successor)

            return None

        initial_path = {self.start_pos}
        found_path = dfs(self.start_pos, depth=0, active_path=initial_path)
        return found_path, explored

    def update(self, grid):
        trivial = self._start_equals_goal(grid)
        if trivial is not None:
            return trivial

        max_depth = (self.grid_dim[0] + 1) * (self.grid_dim[1] + 1)
        for depth_limit in range(max_depth + 1):
            path, explored = self._depth_limited_search(grid, depth_limit)
            for pos in explored:
                self._mark_explored(grid, pos)
            if path is not None:
                return path, True, grid

        return [], True, grid


class Uniform_Cost_Search_Algorithm(_SearchBase):
    def update(self, grid):
        trivial = self._start_equals_goal(grid)
        if trivial is not None:
            return trivial

        parent: dict[Position, Position | None] = {self.start_pos: None}
        best_cost: dict[Position, int] = {self.start_pos: 0}
        open_heap: list[tuple[int, Position]] = [(0, self.start_pos)]

        while open_heap:
            cost_so_far, current = heapq.heappop(open_heap)
            if cost_so_far > best_cost.get(current, inf):
                continue

            if current == self.goal_pos:
                return self._build_path(parent, current), True, grid

            self._mark_explored(grid, current)

            for successor in self.get_successors(*current):
                if not self._is_walkable(grid, successor):
                    continue
                next_cost = cost_so_far + 1
                if next_cost < best_cost.get(successor, inf):
                    best_cost[successor] = next_cost
                    parent[successor] = current
                    heapq.heappush(open_heap, (next_cost, successor))

        return [], True, grid


class Greedy_Best_First_Algorithm(_SearchBase):
    def calculate_heuristic(self, pos: Position) -> int:
        return abs(pos[0] - self.goal_pos[0]) + abs(pos[1] - self.goal_pos[1])

    def update(self, grid):
        trivial = self._start_equals_goal(grid)
        if trivial is not None:
            return trivial

        parent: dict[Position, Position | None] = {self.start_pos: None}
        visited = {self.start_pos}
        open_heap: list[tuple[int, Position]] = [
            (self.calculate_heuristic(self.start_pos), self.start_pos)
        ]

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == self.goal_pos:
                return self._build_path(parent, current), True, grid

            self._mark_explored(grid, current)

            for successor in self.get_successors(*current):
                if successor in visited or not self._is_walkable(grid, successor):
                    continue
                visited.add(successor)
                parent[successor] = current
                heapq.heappush(open_heap, (self.calculate_heuristic(successor), successor))

        return [], True, grid


class A_Star_Algorithm(_SearchBase):
    def calculate_heuristic(self, pos: Position) -> int:
        return abs(pos[0] - self.goal_pos[0]) + abs(pos[1] - self.goal_pos[1])

    def update(self, grid):
        trivial = self._start_equals_goal(grid)
        if trivial is not None:
            return trivial

        parent: dict[Position, Position | None] = {self.start_pos: None}
        g_cost: dict[Position, int] = {self.start_pos: 0}
        open_heap: list[tuple[int, int, Position]] = [
            (self.calculate_heuristic(self.start_pos), 0, self.start_pos)
        ]

        while open_heap:
            _, cost_so_far, current = heapq.heappop(open_heap)
            if cost_so_far > g_cost.get(current, inf):
                continue

            if current == self.goal_pos:
                return self._build_path(parent, current), True, grid

            self._mark_explored(grid, current)

            for successor in self.get_successors(*current):
                if not self._is_walkable(grid, successor):
                    continue

                next_cost = cost_so_far + 1
                if next_cost < g_cost.get(successor, inf):
                    g_cost[successor] = next_cost
                    parent[successor] = current
                    next_f = next_cost + self.calculate_heuristic(successor)
                    heapq.heappush(open_heap, (next_f, next_cost, successor))

        return [], True, grid


class A_Star_Geometric_Algorithm(Greedy_Best_First_Algorithm):
    """Backward-compatible alias for the original ASG option."""


__all__ = [
    "BFS_Algorithm",
    "DFS_Algorithm",
    "IDS_Algorithm",
    "Uniform_Cost_Search_Algorithm",
    "Greedy_Best_First_Algorithm",
    "A_Star_Algorithm",
    "A_Star_Geometric_Algorithm",
]
