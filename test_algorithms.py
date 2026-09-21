"""
Comprehensive DSA Test Suite for Graph Pathfinding
Tests:
- Adjacency List Graph Data Structure & Metrics (|V|, |E|, density, degrees)
- Dijkstra's Algorithm (Min-Heap Priority Queue, edge relaxation)
- A* Search Algorithm (Min-Heap, Euclidean heuristic, f = g + h)
- Greedy Best-First Search (Heuristic-only min-heap)
- Breadth-First Search (FIFO Queue deque, unweighted minimum hops)
- Obstacle avoidance & unreachable goal handling
"""

import math
import heapq
from collections import deque
import unittest


class Graph:
    def __init__(self):
        self.nodes = {}  # id -> {'label': str, 'x': float, 'y': float, 'obstacle': bool}
        self.adj = {}    # id -> [(neighbor_id, weight)]
    
    def add_node(self, node_id, label, x, y, obstacle=False):
        self.nodes[node_id] = {'label': label, 'x': float(x), 'y': float(y), 'obstacle': obstacle}
        if node_id not in self.adj:
            self.adj[node_id] = []

    def add_edge(self, u, v, weight):
        w = max(0.1, round(float(weight), 1))
        self.adj[u].append((v, w))
        self.adj[v].append((u, w))

    def get_density(self):
        v = len(self.nodes)
        if v <= 1:
            return 0.0
        e = sum(len(neighbors) for neighbors in self.adj.values()) // 2
        return round((2.0 * e) / (v * (v - 1)), 3)

    def get_avg_degree(self):
        v = len(self.nodes)
        if v == 0:
            return 0.0
        e = sum(len(neighbors) for neighbors in self.adj.values()) // 2
        return round((2.0 * e) / v, 2)


def euclidean_heuristic(graph, u, goal):
    nu = graph.nodes[u]
    ng = graph.nodes[goal]
    return round(math.hypot(nu['x'] - ng['x'], nu['y'] - ng['y']) / 10.0, 1)


def run_dijkstra(graph, start_id, goal_id):
    if start_id not in graph.nodes or goal_id not in graph.nodes:
        return None, float('inf'), 0
    if graph.nodes[start_id]['obstacle'] or graph.nodes[goal_id]['obstacle']:
        return None, float('inf'), 0

    distances = {node: float('inf') for node in graph.nodes}
    distances[start_id] = 0.0
    came_from = {}
    visited = set()
    counter = 0

    # Min-Heap Priority Queue: (dist, tie_breaker, node_id)
    pq = [(0.0, counter, start_id)]
    nodes_explored = 0

    while pq:
        dist, _, current = heapq.heappop(pq)
        if current in visited:
            continue
        visited.add(current)
        nodes_explored += 1

        if current == goal_id:
            path = []
            curr = goal_id
            while curr is not None:
                path.append(curr)
                curr = came_from.get(curr)
            path.reverse()
            return path, distances[goal_id], nodes_explored

        for neighbor, weight in graph.adj.get(current, []):
            if graph.nodes[neighbor]['obstacle']:
                continue
            new_dist = round(dist + weight, 1)
            if new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                came_from[neighbor] = current
                counter += 1
                heapq.heappush(pq, (new_dist, counter, neighbor))

    return None, float('inf'), nodes_explored


def run_astar(graph, start_id, goal_id):
    if start_id not in graph.nodes or goal_id not in graph.nodes:
        return None, float('inf'), 0
    if graph.nodes[start_id]['obstacle'] or graph.nodes[goal_id]['obstacle']:
        return None, float('inf'), 0

    g_score = {node: float('inf') for node in graph.nodes}
    g_score[start_id] = 0.0
    came_from = {}
    visited = set()
    counter = 0

    h_start = euclidean_heuristic(graph, start_id, goal_id)
    # Min-Heap: (f_score, tie_breaker, g_score, node_id)
    pq = [(h_start, counter, 0.0, start_id)]
    nodes_explored = 0

    while pq:
        f, _, g, current = heapq.heappop(pq)
        if current in visited:
            continue
        visited.add(current)
        nodes_explored += 1

        if current == goal_id:
            path = []
            curr = goal_id
            while curr is not None:
                path.append(curr)
                curr = came_from.get(curr)
            path.reverse()
            return path, g_score[goal_id], nodes_explored

        for neighbor, weight in graph.adj.get(current, []):
            if graph.nodes[neighbor]['obstacle']:
                continue
            tentative_g = round(g + weight, 1)
            if tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                h = euclidean_heuristic(graph, neighbor, goal_id)
                f_val = round(tentative_g + h, 1)
                counter += 1
                heapq.heappush(pq, (f_val, counter, tentative_g, neighbor))

    return None, float('inf'), nodes_explored


def run_greedy_best_first(graph, start_id, goal_id):
    if start_id not in graph.nodes or goal_id not in graph.nodes:
        return None, float('inf'), 0
    if graph.nodes[start_id]['obstacle'] or graph.nodes[goal_id]['obstacle']:
        return None, float('inf'), 0

    came_from = {}
    visited = set()
    counter = 0

    h_start = euclidean_heuristic(graph, start_id, goal_id)
    # Min-Heap ordered strictly by heuristic h(n)
    pq = [(h_start, counter, start_id)]
    nodes_explored = 0

    while pq:
        h, _, current = heapq.heappop(pq)
        if current in visited:
            continue
        visited.add(current)
        nodes_explored += 1

        if current == goal_id:
            path = []
            curr = goal_id
            while curr is not None:
                path.append(curr)
                curr = came_from.get(curr)
            path.reverse()
            # Calculate total distance of reconstructed path
            total_dist = 0.0
            for i in range(len(path) - 1):
                for nbr, w in graph.adj.get(path[i], []):
                    if nbr == path[i + 1]:
                        total_dist += w
                        break
            return path, round(total_dist, 1), nodes_explored

        for neighbor, weight in graph.adj.get(current, []):
            if graph.nodes[neighbor]['obstacle'] or neighbor in visited:
                continue
            if neighbor not in came_from:
                came_from[neighbor] = current
                h_val = euclidean_heuristic(graph, neighbor, goal_id)
                counter += 1
                heapq.heappush(pq, (h_val, counter, neighbor))

    return None, float('inf'), nodes_explored


def run_bfs(graph, start_id, goal_id):
    if start_id not in graph.nodes or goal_id not in graph.nodes:
        return None, float('inf'), 0
    if graph.nodes[start_id]['obstacle'] or graph.nodes[goal_id]['obstacle']:
        return None, float('inf'), 0

    # FIFO Queue
    queue = deque([start_id])
    visited = {start_id}
    came_from = {}
    nodes_explored = 0

    while queue:
        current = queue.popleft()
        nodes_explored += 1

        if current == goal_id:
            path = []
            curr = goal_id
            while curr is not None:
                path.append(curr)
                curr = came_from.get(curr)
            path.reverse()
            # Calculate total distance of the BFS path
            total_dist = 0.0
            for i in range(len(path) - 1):
                for nbr, w in graph.adj.get(path[i], []):
                    if nbr == path[i + 1]:
                        total_dist += w
                        break
            return path, round(total_dist, 1), nodes_explored

        for neighbor, weight in graph.adj.get(current, []):
            if graph.nodes[neighbor]['obstacle'] or neighbor in visited:
                continue
            visited.add(neighbor)
            came_from[neighbor] = current
            queue.append(neighbor)

    return None, float('inf'), nodes_explored


class TestDSAPathfinding(unittest.TestCase):
    def setUp(self):
        self.g = Graph()
        # Diamond network:
        # A (0,0) -> B (100, -50, weight 12) -> D (200, 0, weight 12)
        #         -> C (100,  50, weight 5)  -> D (200, 0, weight 5)
        self.g.add_node('A', 'A', 0, 0)
        self.g.add_node('B', 'B', 100, -50)
        self.g.add_node('C', 'C', 100, 50)
        self.g.add_node('D', 'D', 200, 0)

        self.g.add_edge('A', 'B', 12.0)
        self.g.add_edge('B', 'D', 12.0)
        self.g.add_edge('A', 'C', 5.0)
        self.g.add_edge('C', 'D', 5.0)

    def test_graph_metrics(self):
        self.assertEqual(len(self.g.nodes), 4)
        # 4 edges, 4 vertices -> density = 2*4 / (4*3) = 8/12 = 0.667
        self.assertAlmostEqual(self.g.get_density(), 0.667, delta=0.01)
        self.assertEqual(self.g.get_avg_degree(), 2.0)

    def test_dijkstra_shortest_path(self):
        path, dist, explored = run_dijkstra(self.g, 'A', 'D')
        self.assertEqual(path, ['A', 'C', 'D'])
        self.assertAlmostEqual(dist, 10.0)

    def test_astar_shortest_path(self):
        path, dist, explored = run_astar(self.g, 'A', 'D')
        self.assertEqual(path, ['A', 'C', 'D'])
        self.assertAlmostEqual(dist, 10.0)

    def test_greedy_best_first(self):
        path, dist, explored = run_greedy_best_first(self.g, 'A', 'D')
        self.assertIsNotNone(path)
        self.assertEqual(path[0], 'A')
        self.assertEqual(path[-1], 'D')

    def test_bfs_path(self):
        path, dist, explored = run_bfs(self.g, 'A', 'D')
        self.assertIsNotNone(path)
        self.assertEqual(path[0], 'A')
        self.assertEqual(path[-1], 'D')

    def test_obstacle_avoidance(self):
        # Block node C
        self.g.nodes['C']['obstacle'] = True
        path, dist, explored = run_dijkstra(self.g, 'A', 'D')
        self.assertEqual(path, ['A', 'B', 'D'])
        self.assertAlmostEqual(dist, 24.0)

    def test_disconnected_goal(self):
        self.g.add_node('E', 'E', 300, 0)
        path, dist, explored = run_dijkstra(self.g, 'A', 'E')
        self.assertIsNone(path)
        self.assertEqual(dist, float('inf'))


if __name__ == '__main__':
    unittest.main()
