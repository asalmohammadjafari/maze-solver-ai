from collections import deque
from _helpers import Node, Stack, Queue, PriorityQueue
import sys
import heapq

sys.setrecursionlimit(10000)

class BFS_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim
        self.queue = Queue()
        self.queue.push(Node(pos=start_pos, parent=None))

    def get_successors(self, x, y):
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def backtrack_solution(self, curr_node):
        return self._backtrack(curr_node)

    def _backtrack(self, curr_node):
        return [] if curr_node.parent is None else self._backtrack(curr_node.parent) + [curr_node.position()]

    def update(self, grid):
        curr_state = self.queue.pop()
        x, y = curr_state.position()
        done = False
        solution_path = []

        for step in self.get_successors(x, y):
            if self.is_valid_cell(step) and grid[step[0], step[1]] in [1, 3]:
                self.queue.push(Node(pos=step, parent=curr_state))

                if step == self.goal_pos:
                    done = True
                    solution_path = self.backtrack_solution(curr_state)
                    break

            grid[x, y] = 4

        return solution_path, done, grid


class DFS_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim
        self.stack = Stack()
        self.stack.push(Node(pos=start_pos, parent=None))

    def get_successors(self, x, y):
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def backtrack_solution(self, curr_node):
        return self._backtrack(curr_node)

    def _backtrack(self, curr_node):
        return [] if curr_node.parent is None else self._backtrack(curr_node.parent) + [curr_node.position()]

    def update(self, grid):
        curr_state = self.stack.pop()
        x, y = curr_state.position()
        done = False
        solution_path = []

        for step in self.get_successors(x, y):
            if self.is_valid_cell(step) and grid[step[0], step[1]] in [1, 3]:
                self.stack.push(Node(pos=step, parent=curr_state))

                if step == self.goal_pos:
                    done = True
                    solution_path = self.backtrack_solution(curr_state)
                    break

            grid[x, y] = 4

        return solution_path, done, grid


class IDS_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim
        self.stack = Stack()
        self.stack.push(Node(pos=start_pos, parent=None))
        self.max_depth = 0

    def get_successors(self, x, y):
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def backtrack_solution(self, curr_node):
        return self._backtrack(curr_node)

    def _backtrack(self, curr_node):
        return [] if curr_node.parent is None else self._backtrack(curr_node.parent) + [curr_node.position()]

    def dls(self, grid, depth):
        curr_state = self.stack.pop()
        x, y = curr_state.position()
        done = False
        solution_path = []

        for step in self.get_successors(x, y):
            if self.max_depth < depth:
                continue
            elif self.is_valid_cell(step) and grid[step[0], step[1]] in [1, 3]:
                self.stack.push(Node(pos=step, parent=curr_state))

                if step == self.goal_pos:
                    done = True
                    solution_path = self.backtrack_solution(curr_state)
                    break

            grid[x, y] = 4
        return solution_path, done, grid

    def update(self, grid):
        res = self.dls(grid, self.max_depth)
        if res[1]:
            return res
        else:
            self.max_depth += 1
            return self.update(grid)


class A_Star_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim

    def calculate_heuristic(self, pos):
        return ((pos[0] - self.goal_pos[0]) ** 2 + (pos[1] - self.goal_pos[1]) ** 2) ** 0.5

    def get_successors(self, x, y):
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1), (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1),
                (x - 1, y + 1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def backtrack_solution(self, curr_node):
        return self._backtrack(curr_node)

    def _backtrack(self, curr_node):
        return [] if curr_node.parent is None else self._backtrack(curr_node.parent) + [curr_node.position()]

    def find_lowest_cost(self, node_list):
        best_node = node_list[0]
        for node in node_list[1:]:
            if node.cost < best_node.cost:
                best_node = node

        return best_node

    def update(self, grid):
        done = False
        solution_path = []
        open_list = []
        start_node = Node(self.start_pos, parent=None, cost=0)
        heapq.heappush(open_list, start_node)
        closed_list = set()

        while open_list:
            q = heapq.heappop(open_list)

            if q.position() in closed_list:
                continue

            closed_list.add(q.position())

            if q.position() == self.goal_pos:
                solution_path = self.backtrack_solution(q)
                done = True
                break

            successors_h = sorted(
                [(self.calculate_heuristic(step), step) for step in self.get_successors(*q.position())])

            for successor in successors_h:
                if self.is_valid_cell(successor[1]) and grid[successor[1][0], successor[1][1]] in [1, 3]:
                    n = Node(successor[1], parent=q, cost=successor[0])
                    heapq.heappush(open_list, n)
            grid[*q.position()] = 4

        return solution_path, done, grid


class A_Star_Geometric_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim

    def calculate_heuristic(self, pos):
        return abs(pos[0] - self.goal_pos[0]) + abs(pos[1] - self.goal_pos[1])

    def get_successors(self, x, y):
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1), (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1),
                (x - 1, y + 1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def backtrack_solution(self, curr_node):
        return self._backtrack(curr_node)

    def _backtrack(self, curr_node):
        return [] if curr_node.parent is None else self._backtrack(curr_node.parent) + [curr_node.position()]

    def find_lowest_cost(self, node_list):
        best_node = node_list[0]
        for node in node_list[1:]:
            if node.cost < best_node.cost:
                best_node = node

        return best_node

    def update(self, grid):
        done = False
        solution_path = []
        open_list = []
        start_node = Node(self.start_pos, parent=None, cost=0)
        heapq.heappush(open_list, start_node)
        closed_list = set()

        while open_list:
            q = heapq.heappop(open_list)

            if q.position() in closed_list:
                continue

            closed_list.add(q.position())

            if q.position() == self.goal_pos:
                solution_path = self.backtrack_solution(q)
                done = True
                break

            successors_h = sorted(
                [(self.calculate_heuristic(step), step) for step in self.get_successors(*q.position())])

            for successor in successors_h:
                if self.is_valid_cell(successor[1]) and grid[successor[1][0], successor[1][1]] in [1, 3]:
                    n = Node(successor[1], parent=q, cost=successor[0])
                    heapq.heappush(open_list, n)

            grid[q.position()[0], q.position()[1]] = 4

        return solution_path, done, grid
