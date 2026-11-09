"""
Daniel Graham
620171166
"""
from collections import deque
import heapq

#(1 = wall, 0 = open space, S = start at row 1 col 1, G = goal at row 8 col 8, 0-indexed.)
BENCHMARK_MAZE = [
    "1111111111",
    "1S01001001",
    "1000101101",
    "1110010001",
    "1010000001",
    "1000001101",
    "1000000001",
    "1110000101",
    "10010000G1",
    "1111111111",
]


def load_grid(rows):

    grid, start, goal = [], None, None
    for r, line in enumerate(rows):
        row = []
        for c, ch in enumerate(line):
            if ch == '1':
                row.append(1)
            elif ch == '0':
                row.append(0)
            elif ch == 'S':
                row.append(0)
                start = (r, c)
            elif ch == 'G':
                row.append(0)
                goal = (r, c)
        grid.append(row)
    return grid, start, goal


def manhattan_distance(node_a, node_b):
    return abs(node_a[0] - node_b[0]) + abs(node_a[1] - node_b[1])


def get_valid_neighbors(maze, node):
    """
    Generates valid 4-connected neighboring coordinates (up, down, left, right)
    that are within bounds and represent open spaces (0).
    """
    r, c = node
    # Define 4 cardinal directions (Delta Row, Delta Col)
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        nr, nc = r + dr, c + dc
        # Check boundary limits and ensure cell is open (0)
        if 0 <= nr < len(maze) and 0 <= nc < len(maze[0]) and maze[nr][nc] == 0:
            yield (nr, nc)


def reconstruct_path(came_from_map, start_node, goal_node):
    """
    Backtracks through the 'came_from' mapping dictionary
    to reconstruct the shortest or valid path from start to goal.
    """
    path = [goal_node]
    while path[-1] != start_node:
        path.append(came_from_map[path[-1]])
    path.reverse()
    return path


#Breadth-First Search (BFS)
def bfs(maze, start, goal):
    frontier = deque([start])  # FIFO queue for BFS exploration
    came_from = {start: None}  # tracking dictionary for path reconstruction
    visited_order = []  # Sequence of nodes expanded
    expanded = 0  # Node expansion counter

    while frontier:
        node = frontier.popleft()
        visited_order.append(node)
        expanded += 1

        # Check goal reached
        if node == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, visited_order, {
                'nodes_expanded': expanded,
                'path_length': len(path),
                'path_cost': len(path) - 1,
            }

        # Explore valid neighboring cells
        for nb in get_valid_neighbors(maze, node):
            if nb not in came_from:
                came_from[nb] = node
                frontier.append(nb)

    # Return empty path and infinity cost if no route exists
    return ([], visited_order, {
        'nodes_expanded': expanded,
        'path_length': 0,
        'path_cost': float('inf'),
    })


#Depth-First Search (DFS)
def dfs(maze, start, goal):

    stack = [start]  # LIFO stack for DFS exploration
    came_from = {start: None}
    visited_order = []
    expanded = 0

    while stack:
        node = stack.pop()

        # Skip if node was already expanded
        if node in visited_order:
            continue

        visited_order.append(node)
        expanded += 1

        if node == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, visited_order, {
                'nodes_expanded': expanded,
                'path_length': len(path),
                'path_cost': len(path) - 1,
            }

        for nb in get_valid_neighbors(maze, node):
            if nb not in came_from:
                came_from[nb] = node
                stack.append(nb)

    return ([], visited_order, {
        'nodes_expanded': expanded,
        'path_length': 0,
        'path_cost': float('inf'),
    })


#A* Search
def astar(maze, start, goal, heuristic=manhattan_distance):
    """
    Performs A* Search utilizing a priority queue and a heuristic function.
    Guarantees optimal path finding with admissible heuristics.
    """
    counter = 0  # Tie-breaker counter to prevent tuple comparison errors in heapq

    # Priority queue stores tuples: (f_score, tie_breaker, node_coordinates)
    open_heap = [(heuristic(start, goal), counter, start)]
    came_from = {start: None}
    g_cost = {start: 0}  # Cost from start node to current node
    visited_order = []
    expanded = 0
    closed_set = set()  # Track fully evaluated nodes

    while open_heap:
        f, _, node = heapq.heappop(open_heap)

        if node in closed_set:
            continue
        closed_set.add(node)
        visited_order.append(node)
        expanded += 1

        if node == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, visited_order, {
                'nodes_expanded': expanded,
                'path_length': len(path),
                'path_cost': g_cost[node],
            }

        for nb in get_valid_neighbors(maze, node):
            tentative_g_cost = g_cost[node] + 1

            # Update path if a cheaper route to neighbor is found
            if nb not in g_cost or tentative_g_cost < g_cost[nb]:
                g_cost[nb] = tentative_g_cost
                came_from[nb] = node
                counter += 1
                f_score = tentative_g_cost + heuristic(nb, goal)
                heapq.heappush(open_heap, (f_score, counter, nb))

    return ([], visited_order, {
        'nodes_expanded': expanded,
        'path_length': 0,
        'path_cost': float('inf'),
    })


#Greedy Best-First Search (GBFS)
def gbfs(maze, start, goal, heuristic=manhattan_distance):
    """
    Performs Greedy Best-First Search on the maze grid.
    Prioritizes expansion only based on the heuristic h(n).
    """
    counter = 0  # Tie-breaker to prevent comparison errors in heapq

    open_heap = [(heuristic(start, goal), counter, start)]
    came_from = {start: None}
    visited_order = []
    expanded = 0
    closed_set = set()

    while open_heap:
        h, _, node = heapq.heappop(open_heap)

        if node in closed_set:
            continue
        closed_set.add(node)
        visited_order.append(node)
        expanded += 1

        if node == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, visited_order, {
                'nodes_expanded': expanded,
                'path_length': len(path),
                'path_cost': len(path) - 1,
            }

        for nb in get_valid_neighbors(maze, node):
            if nb not in came_from and nb not in closed_set:
                came_from[nb] = node
                counter += 1
                heapq.heappush(open_heap, (heuristic(nb, goal), counter, nb))

    return ([], visited_order, {
        'nodes_expanded': expanded,
        'path_length': 0,
        'path_cost': float('inf'),
    })