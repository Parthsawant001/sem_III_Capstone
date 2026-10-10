#!/usr/bin/env python3
"""
Interactive Graph Pathfinding Visualizer (DSA Edition)
100% Pure Python 3 & Tkinter Desktop Application. Zero web dependencies.

Core DSA Concepts Implemented:
- Graph Data Structure: Adjacency List representation with degree & density metrics.
- Min-Heap Priority Queue: Python `heapq` with real-time heap state inspection.
- FIFO Queue: Python `collections.deque` for unweighted breadth-first exploration.
- 4 Pathfinding Algorithms:
    1. Dijkstra's Algorithm (Single-Source Shortest Path, Min-Heap relaxation)
    2. A* Search Algorithm (Informed Heuristic Search: f(n) = g(n) + h(n))
    3. Greedy Best-First Search (Heuristic-only prioritization: f(n) = h(n))
    4. Breadth-First Search (BFS level-order unweighted shortest path)
- Real-time step-by-step animation via Tkinter `.after()` loop (non-blocking).
- Interactive Canvas: Node creation, edge weight modal, drag-and-drop, right-click context menu.
- DSA Inspector: Detailed view of Adjacency List and Graph Theory metrics (|V|, |E|, density).
"""
import sys
import os
import ast
import pprint
import time
import datetime
import shutil
import math
import heapq
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox

# ==============================================================================
# COLOR PALETTE & DESIGN SYSTEM (DARK THEME)
# ==============================================================================
THEME = {
    "bg_app": "#0B0F19",          # Deep Obsidian app background
    "bg_panel": "#111827",        # Sidebar & Header charcoal
    "bg_card": "#1F2937",         # Inner cards / containers
    "bg_card_inner": "#161E2E",   # Nested metric boxes
    "border": "#374151",          # Subtle divider border
    "border_light": "#4B5563",
    "text_primary": "#F9FAFB",    # High-contrast white
    "text_secondary": "#9CA3AF",  # Muted slate text
    "text_accent": "#38BDF8",     # Sky blue highlight
    
    # Canvas & Graph Elements
    "canvas_bg": "#0D1322",       # Canvas deep midnight navy
    "canvas_grid": "#1E293B",     # Subtle grid dots
    "node_default_fill": "#1E293B",
    "node_default_stroke": "#38BDF8", # Bright Sky Blue
    
    "node_start_fill": "#064E3B",
    "node_start_stroke": "#10B981",    # Emerald Green
    
    "node_goal_fill": "#7F1D1D",
    "node_goal_stroke": "#EF4444",     # Bright Crimson Red
    
    "node_obstacle_fill": "#181D26",
    "node_obstacle_stroke": "#64748B", # Dim slate obstacle
    "node_obstacle_cross": "#EF4444",
    
    "node_frontier_fill": "#78350F",
    "node_frontier_stroke": "#F59E0B", # Amber / Golden Yellow
    
    "node_visited_fill": "#312E81",
    "node_visited_stroke": "#818CF8",  # Periwinkle / Purple
    
    "node_active_fill": "#0E7490",
    "node_active_stroke": "#06B6D4",   # Glowing Cyan (Current expanding)
    
    "node_path_fill": "#78350F",       # Golden Amber
    "node_path_stroke": "#FACC15",     # Vibrant Gold Yellow
    
    # Edges
    "edge_default": "#475569",
    "edge_examining": "#22C55E",       # Electric Green (Actively searched road)
    "edge_searched": "#10B981",        # Emerald Green (Road taken in exploration)
    "edge_path": "#FACC15",            # High-Contrast Vibrant Yellow (Final Shortest Path)
    "edge_badge_bg": "#1F2937",
    "edge_badge_text": "#E2E8F0",
    
    # Buttons
    "btn_bg": "#1F2937",
    "btn_hover": "#374151",
    "btn_text": "#F3F4F6",
}

FONT_FAMILY = "Segoe UI" if sys.platform == "win32" else "Helvetica"
FONT_TITLE = (FONT_FAMILY, 12, "bold")
FONT_SUBTITLE = (FONT_FAMILY, 9, "bold")
FONT_REGULAR = (FONT_FAMILY, 9)
FONT_SMALL = (FONT_FAMILY, 8)
FONT_BOLD = (FONT_FAMILY, 9, "bold")
FONT_NODE = (FONT_FAMILY, 11, "bold")
FONT_MONO = ("Consolas", 9)
FONT_MONO_SMALL = ("Consolas", 8)


# ==============================================================================
# DATA STRUCTURES: GRAPH & ADJACENCY LIST
# ==============================================================================
class Node:
    def __init__(self, node_id, label, x, y, is_start=False, is_goal=False, is_obstacle=False):
        self.id = node_id
        self.label = label
        self.x = float(x)
        self.y = float(y)
        self.is_start = is_start
        self.is_goal = is_goal
        self.is_obstacle = is_obstacle
        self.radius = 24

class Graph:
    """
    Adjacency List Graph representation with coordinate geometry.
    Maintains nodes dict and undirected edges dict.
    """
    def __init__(self):
        self.nodes = {}  # id -> Node
        self.edges = {}  # tuple(sorted(u, v)) -> weight
        self._next_id_int = 0

    def get_next_label(self):
        existing = {n.label for n in self.nodes.values()}
        val = 0
        while True:
            temp = val
            label = ""
            while True:
                label = chr(65 + (temp % 26)) + label
                temp = temp // 26 - 1
                if temp < 0:
                    break
            if label not in existing:
                return label
            val += 1

    def add_node(self, x, y, label=None, is_start=False, is_goal=False, is_obstacle=False):
        if label is None:
            label = self.get_next_label()
        node_id = f"N_{label}"
        counter = 1
        while node_id in self.nodes:
            node_id = f"N_{label}_{counter}"
            counter += 1
            
        node = Node(node_id, label, x, y, is_start, is_goal, is_obstacle)
        self.nodes[node_id] = node
        return node

    def remove_node(self, node_id):
        if node_id in self.nodes:
            to_del = [e for e in self.edges if node_id in e]
            for e in to_del:
                del self.edges[e]
            del self.nodes[node_id]

    def add_edge(self, u_id, v_id, weight):
        if u_id == v_id or u_id not in self.nodes or v_id not in self.nodes:
            return False
        key = tuple(sorted([u_id, v_id]))
        self.edges[key] = max(0.1, round(float(weight), 1))
        return True

    def remove_edge(self, u_id, v_id):
        key = tuple(sorted([u_id, v_id]))
        if key in self.edges:
            del self.edges[key]

    def get_neighbors(self, node_id):
        neighbors = []
        for (u, v), weight in self.edges.items():
            if u == node_id and v in self.nodes:
                neighbors.append((v, weight))
            elif v == node_id and u in self.nodes:
                neighbors.append((u, weight))
        return neighbors

    def get_adjacency_list_dict(self):
        """Returns adjacency list representation for inspection."""
        adj = {node.label: [] for node in self.nodes.values()}
        for (u_id, v_id), weight in self.edges.items():
            if u_id in self.nodes and v_id in self.nodes:
                u_lbl = self.nodes[u_id].label
                v_lbl = self.nodes[v_id].label
                adj[u_lbl].append((v_lbl, weight))
                adj[v_lbl].append((u_lbl, weight))
        return adj

    def get_graph_density(self):
        v = len(self.nodes)
        if v <= 1:
            return 0.0
        e = len(self.edges)
        return round((2.0 * e) / (v * (v - 1)), 3)

    def get_avg_degree(self):
        v = len(self.nodes)
        if v == 0:
            return 0.0
        return round((2.0 * len(self.edges)) / v, 2)

    def get_start_node(self):
        for n in self.nodes.values():
            if n.is_start:
                return n
        return None

    def get_goal_node(self):
        for n in self.nodes.values():
            if n.is_goal:
                return n
        return None

    def clear_all(self):
        self.nodes.clear()
        self.edges.clear()
        self._next_id_int = 0


# ==============================================================================
# DSA PATHFINDING ALGORITHMS (GENERATORS)
# ==============================================================================
def euclidean_distance(graph, u_id, v_id):
    nu = graph.nodes[u_id]
    nv = graph.nodes[v_id]
    raw = math.hypot(nu.x - nv.x, nu.y - nv.y)
    return round(raw / 10.0, 1)


def dijkstra_step_generator(graph, start_id, goal_id):
    """
    Dijkstra's Single-Source Shortest Path Algorithm
    DSA Structure: Min-Heap Priority Queue (heapq)
    Complexity: Time O((V + E) log V), Space O(V)
    """
    distances = {n_id: float('inf') for n_id in graph.nodes}
    distances[start_id] = 0.0
    came_from = {}
    visited = set()
    counter = 0

    # Min-Heap elements: (dist, tie_breaker, node_id)
    pq = [(0.0, counter, start_id)]
    explored_count = 0
    total_steps = 0

    start_lbl = graph.nodes[start_id].label
    goal_lbl = graph.nodes[goal_id].label

    yield ("LOG", None, f"[DSA Dijkstra] Starting Single-Source Shortest Path ({start_lbl} -> {goal_lbl})")
    yield ("FRONTIER_ADD", (start_id, 0.0, 0.0, [f"({start_lbl}: 0.0)"]), f"[Min-Heap] Pushed Start {start_lbl} with priority 0.0")

    while pq:
        dist, _, current = heapq.heappop(pq)
        if current in visited:
            continue
        visited.add(current)
        explored_count += 1
        curr_node = graph.nodes[current]

        # Snapshot of heap
        heap_snapshot = [f"({graph.nodes[item[2]].label}: {item[0]:.1f})" for item in sorted(pq)[:5]]
        yield ("VISIT_NODE", (current, dist, heap_snapshot), f"[Min-Heap Pop] Settling node {curr_node.label} (d[{curr_node.label}] = {dist:.1f})")
        total_steps += 1

        if current == goal_id:
            path = []
            curr = goal_id
            while curr is not None:
                path.append(curr)
                curr = came_from.get(curr)
            path.reverse()

            yield ("LOG", None, f"[DSA Done] Goal {goal_lbl} reached with optimal distance {dist:.1f}")
            yield ("PATH_FOUND", {
                "path": path,
                "total_dist": dist,
                "explored_count": explored_count,
                "total_steps": total_steps
            }, f"[Shortest Path] {' -> '.join(graph.nodes[n].label for n in path)}")
            return

        for neighbor, weight in graph.get_neighbors(current):
            total_steps += 1
            n_node = graph.nodes[neighbor]

            if n_node.is_obstacle:
                yield ("EXAMINE_EDGE", (current, neighbor, False), f"[Obstacle] Skipped {n_node.label} (Blocked)")
                continue
            if neighbor in visited:
                yield ("EXAMINE_EDGE", (current, neighbor, False), f"[Closed] Node {n_node.label} already settled")
                continue

            tentative_dist = round(dist + weight, 1)
            yield ("EXAMINE_EDGE", (current, neighbor, True), f"[Edge Relaxation] d[{curr_node.label}] + w({curr_node.label},{n_node.label}) = {dist:.1f} + {weight:.1f} = {tentative_dist:.1f}")

            if tentative_dist < distances[neighbor]:
                old_d = distances[neighbor]
                distances[neighbor] = tentative_dist
                came_from[neighbor] = current
                counter += 1
                heapq.heappush(pq, (tentative_dist, counter, neighbor))
                heap_snap = [f"({graph.nodes[item[2]].label}: {item[0]:.1f})" for item in sorted(pq)[:5]]
                yield ("FRONTIER_ADD", (neighbor, tentative_dist, tentative_dist, heap_snap),
                       f"[Relaxed] d[{n_node.label}]: {old_d} -> {tentative_dist:.1f}. Pushed to Min-Heap.")

        yield ("CLOSE_NODE", current, f"[Node Closed] Completed adjacent edges of {curr_node.label}")

    yield ("NO_PATH", {"explored_count": explored_count, "total_steps": total_steps}, f"[Error] No path found: Goal {goal_lbl} is unreachable.")


def astar_step_generator(graph, start_id, goal_id):
    """
    A* Search Algorithm
    DSA Structure: Min-Heap Priority Queue (heapq) ordered by f(n) = g(n) + h(n)
    Complexity: Time O((V + E) log V), Space O(V)
    """
    g_score = {n_id: float('inf') for n_id in graph.nodes}
    g_score[start_id] = 0.0
    came_from = {}
    visited = set()
    counter = 0

    h_start = euclidean_distance(graph, start_id, goal_id)
    # Min-Heap elements: (f_score, tie_breaker, g_score, node_id)
    pq = [(h_start, counter, 0.0, start_id)]
    explored_count = 0
    total_steps = 0

    start_lbl = graph.nodes[start_id].label
    goal_lbl = graph.nodes[goal_id].label

    yield ("LOG", None, f"[DSA A* Search] Starting Informed Search with Euclidean Heuristic ({start_lbl} -> {goal_lbl})")
    yield ("FRONTIER_ADD", (start_id, 0.0, h_start, [f"({start_lbl}: f={h_start:.1f})"]),
           f"[A* Init] Start {start_lbl}: g=0.0, h={h_start:.1f} -> f={h_start:.1f}")

    while pq:
        f, _, g, current = heapq.heappop(pq)
        if current in visited:
            continue
        visited.add(current)
        explored_count += 1
        curr_node = graph.nodes[current]

        heap_snapshot = [f"({graph.nodes[item[3]].label}: f={item[0]:.1f})" for item in sorted(pq)[:5]]
        yield ("VISIT_NODE", (current, g, heap_snapshot), f"[A* Expand] Node {curr_node.label} (g={g:.1f}, f={f:.1f})")
        total_steps += 1

        if current == goal_id:
            path = []
            curr = goal_id
            while curr is not None:
                path.append(curr)
                curr = came_from.get(curr)
            path.reverse()

            yield ("LOG", None, f"[A* Optimal Path] Reached goal {goal_lbl} with cost {g:.1f}")
            yield ("PATH_FOUND", {
                "path": path,
                "total_dist": g,
                "explored_count": explored_count,
                "total_steps": total_steps
            }, f"[A* Solution] {' -> '.join(graph.nodes[n].label for n in path)}")
            return

        for neighbor, weight in graph.get_neighbors(current):
            total_steps += 1
            n_node = graph.nodes[neighbor]

            if n_node.is_obstacle:
                yield ("EXAMINE_EDGE", (current, neighbor, False), f"[Obstacle] Skipped {n_node.label}")
                continue
            if neighbor in visited:
                yield ("EXAMINE_EDGE", (current, neighbor, False), f"[Settled] {n_node.label} in closed set")
                continue

            tentative_g = round(g + weight, 1)
            yield ("EXAMINE_EDGE", (current, neighbor, True), f"[Evaluating] {curr_node.label}->{n_node.label}: tentative g = {tentative_g:.1f}")

            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                h = euclidean_distance(graph, neighbor, goal_id)
                new_f = round(tentative_g + h, 1)
                counter += 1
                heapq.heappush(pq, (new_f, counter, tentative_g, neighbor))
                heap_snap = [f"({graph.nodes[item[3]].label}: f={item[0]:.1f})" for item in sorted(pq)[:5]]
                yield ("FRONTIER_ADD", (neighbor, tentative_g, new_f, heap_snap),
                       f"[A* Relax] {n_node.label}: g={tentative_g:.1f} + h={h:.1f} => f={new_f:.1f}")

        yield ("CLOSE_NODE", current, f"[Closed Set] Finished expanding {curr_node.label}")

    yield ("NO_PATH", {"explored_count": explored_count, "total_steps": total_steps}, f"[Error] Goal {goal_lbl} unreachable.")


def greedy_bfs_step_generator(graph, start_id, goal_id):
    """
    Greedy Best-First Search Algorithm
    DSA Structure: Min-Heap Priority Queue ordered solely by heuristic h(n)
    Complexity: Time O(V log V), Space O(V)
    """
    came_from = {}
    visited = set()
    counter = 0

    h_start = euclidean_distance(graph, start_id, goal_id)
    # Min-Heap elements: (h_score, tie_breaker, node_id)
    pq = [(h_start, counter, start_id)]
    explored_count = 0
    total_steps = 0

    start_lbl = graph.nodes[start_id].label
    goal_lbl = graph.nodes[goal_id].label

    yield ("LOG", None, f"[DSA Greedy BFS] Heuristic-Only Search Ordering by h(n) ({start_lbl} -> {goal_lbl})")
    yield ("FRONTIER_ADD", (start_id, 0.0, h_start, [f"({start_lbl}: h={h_start:.1f})"]),
           f"[Greedy Init] Pushed Start {start_lbl} with heuristic h={h_start:.1f}")

    while pq:
        h, _, current = heapq.heappop(pq)
        if current in visited:
            continue
        visited.add(current)
        explored_count += 1
        curr_node = graph.nodes[current]

        heap_snapshot = [f"({graph.nodes[item[2]].label}: h={item[0]:.1f})" for item in sorted(pq)[:5]]
        yield ("VISIT_NODE", (current, h, heap_snapshot), f"[Greedy Pop] Expanding node {curr_node.label} (Lowest h = {h:.1f})")
        total_steps += 1

        if current == goal_id:
            path = []
            curr = goal_id
            while curr is not None:
                path.append(curr)
                curr = came_from.get(curr)
            path.reverse()

            total_dist = 0.0
            for i in range(len(path) - 1):
                for nbr, w in graph.get_neighbors(path[i]):
                    if nbr == path[i + 1]:
                        total_dist += w
                        break

            yield ("LOG", None, f"[Greedy Done] Goal reached! Path distance: {total_dist:.1f}")
            yield ("PATH_FOUND", {
                "path": path,
                "total_dist": round(total_dist, 1),
                "explored_count": explored_count,
                "total_steps": total_steps
            }, f"[Greedy Path] {' -> '.join(graph.nodes[n].label for n in path)}")
            return

        for neighbor, weight in graph.get_neighbors(current):
            total_steps += 1
            n_node = graph.nodes[neighbor]

            if n_node.is_obstacle:
                yield ("EXAMINE_EDGE", (current, neighbor, False), f"[Obstacle] Skipped {n_node.label}")
                continue
            if neighbor in visited:
                yield ("EXAMINE_EDGE", (current, neighbor, False), f"[Visited] {n_node.label}")
                continue

            yield ("EXAMINE_EDGE", (current, neighbor, True), f"[Greedy Inspect] Road {curr_node.label} -> {n_node.label} (w={weight:g})")

            if neighbor not in came_from:
                came_from[neighbor] = current
                h_val = euclidean_distance(graph, neighbor, goal_id)
                counter += 1
                heapq.heappush(pq, (h_val, counter, neighbor))
                heap_snap = [f"({graph.nodes[item[2]].label}: h={item[0]:.1f})" for item in sorted(pq)[:5]]
                yield ("FRONTIER_ADD", (neighbor, 0.0, h_val, heap_snap),
                       f"[Greedy Push] {n_node.label}: heuristic h={h_val:.1f}")

        yield ("CLOSE_NODE", current, f"[Closed] Finished {curr_node.label}")

    yield ("NO_PATH", {"explored_count": explored_count, "total_steps": total_steps}, f"[Error] Goal {goal_lbl} unreachable.")


def bfs_step_generator(graph, start_id, goal_id):
    """
    Breadth-First Search (BFS) Algorithm
    DSA Structure: FIFO Queue (collections.deque)
    Complexity: Time O(V + E), Space O(V)
    Finds shortest unweighted path (minimum hops).
    """
    queue = deque([start_id])
    visited = {start_id}
    came_from = {}
    explored_count = 0
    total_steps = 0

    start_lbl = graph.nodes[start_id].label
    goal_lbl = graph.nodes[goal_id].label

    yield ("LOG", None, f"[DSA BFS] Unweighted Level-Order Traversal with FIFO Queue ({start_lbl} -> {goal_lbl})")
    yield ("FRONTIER_ADD", (start_id, 0.0, 0.0, [start_lbl]), f"[FIFO Enqueue] Start {start_lbl}")

    while queue:
        current = queue.popleft()
        explored_count += 1
        curr_node = graph.nodes[current]

        q_snap = [graph.nodes[qid].label for qid in list(queue)[:5]]
        yield ("VISIT_NODE", (current, 0.0, q_snap), f"[FIFO Dequeue] Visiting node {curr_node.label} (Level-Order)")
        total_steps += 1

        if current == goal_id:
            path = []
            curr = goal_id
            while curr is not None:
                path.append(curr)
                curr = came_from.get(curr)
            path.reverse()

            total_dist = 0.0
            for i in range(len(path) - 1):
                for nbr, w in graph.get_neighbors(path[i]):
                    if nbr == path[i + 1]:
                        total_dist += w
                        break

            yield ("LOG", None, f"[BFS Done] Goal {goal_lbl} reached in {len(path)-1} hops (Cost: {total_dist:.1f})")
            yield ("PATH_FOUND", {
                "path": path,
                "total_dist": round(total_dist, 1),
                "explored_count": explored_count,
                "total_steps": total_steps
            }, f"[BFS Shortest Hops] {' -> '.join(graph.nodes[n].label for n in path)}")
            return

        for neighbor, weight in graph.get_neighbors(current):
            total_steps += 1
            n_node = graph.nodes[neighbor]

            if n_node.is_obstacle:
                yield ("EXAMINE_EDGE", (current, neighbor, False), f"[Obstacle] Skipped {n_node.label}")
                continue
            if neighbor in visited:
                yield ("EXAMINE_EDGE", (current, neighbor, False), f"[Visited] {n_node.label}")
                continue

            yield ("EXAMINE_EDGE", (current, neighbor, True), f"[BFS Traversal] Exploring road {curr_node.label} -> {n_node.label} (w={weight:g})")

            visited.add(neighbor)
            came_from[neighbor] = current
            queue.append(neighbor)
            q_snap = [graph.nodes[qid].label for qid in list(queue)[:5]]
            yield ("FRONTIER_ADD", (neighbor, 0.0, 0.0, q_snap), f"[FIFO Enqueue] Enqueued {n_node.label} into queue")

        yield ("CLOSE_NODE", current, f"[Settled] Finished {curr_node.label}")

    yield ("NO_PATH", {"explored_count": explored_count, "total_steps": total_steps}, f"[Error] Goal {goal_lbl} unreachable.")


# ==============================================================================
# CUSTOM DARK POPUP MODALS
# ==============================================================================
class DarkWeightDialog(tk.Toplevel):
    def __init__(self, parent, u_label, v_label, default_weight=10.0, is_edit=False):
        super().__init__(parent)
        self.result = None
        self.title("Edit Road Distance / Weight" if is_edit else "Set Road Distance / Weight")
        self.configure(bg=THEME["bg_card"])
        self.resizable(False, False)
        self.transient(parent)
        self.grab_set()

        w, h = 360, 210
        x = parent.winfo_rootx() + (parent.winfo_width() - w) // 2
        y = parent.winfo_rooty() + (parent.winfo_height() - h) // 2
        self.geometry(f"{w}x{h}+{max(50, x)}+{max(50, y)}")

        action_title = f"✏️ Edit Road: {u_label} <--> {v_label}" if is_edit else f"Connect Road: {u_label} <--> {v_label}"
        tk.Label(
            self,
            text=action_title,
            font=FONT_TITLE,
            bg=THEME["bg_card"],
            fg="#F59E0B" if is_edit else THEME["text_primary"]
        ).pack(pady=(16, 4))

        prompt_text = "Update edge distance (weight > 0):" if is_edit else "Enter edge distance (weight > 0):"
        tk.Label(
            self,
            text=prompt_text,
            font=FONT_REGULAR,
            bg=THEME["bg_card"],
            fg=THEME["text_secondary"]
        ).pack(pady=(0, 10))

        self.entry_var = tk.StringVar(value=str(round(default_weight, 1)))
        self.entry = tk.Entry(
            self,
            textvariable=self.entry_var,
            font=(FONT_FAMILY, 12, "bold"),
            bg="#111827",
            fg=THEME["text_accent"],
            insertbackground="#38BDF8",
            justify="center",
            relief="solid",
            bd=1
        )
        self.entry.pack(ipady=4, padx=30, fill="x")
        self.entry.select_range(0, tk.END)
        self.entry.focus_set()

        sub_info = f"(Current road length: {default_weight:g})" if is_edit else f"(Geometric Euclidean distance is ~{default_weight:.1f})"
        tk.Label(
            self,
            text=sub_info,
            font=FONT_SMALL,
            bg=THEME["bg_card"],
            fg=THEME["text_secondary"]
        ).pack(pady=(4, 12))

        btn_frame = tk.Frame(self, bg=THEME["bg_card"])
        btn_frame.pack(fill="x", padx=20, pady=(0, 12))

        cancel_btn = tk.Button(
            btn_frame,
            text="Cancel",
            font=FONT_BOLD,
            bg=THEME["btn_bg"],
            fg=THEME["text_secondary"],
            activebackground=THEME["btn_hover"],
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            command=self.on_cancel,
            cursor="hand2"
        )
        cancel_btn.pack(side="left", expand=True, fill="x", padx=(0, 6), ipady=5)

        confirm_btn = tk.Button(
            btn_frame,
            text="Update Length" if is_edit else "Save Road",
            font=FONT_BOLD,
            bg="#D97706" if is_edit else "#2563EB",
            fg="#FFFFFF",
            activebackground="#B45309" if is_edit else "#1D4ED8",
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            command=self.on_confirm,
            cursor="hand2"
        )
        confirm_btn.pack(side="right", expand=True, fill="x", padx=(6, 0), ipady=5)

        self.bind("<Return>", lambda e: self.on_confirm())
        self.bind("<Escape>", lambda e: self.on_cancel())
        self.wait_window(self)

    def on_confirm(self):
        val_str = self.entry_var.get().strip()
        try:
            val = float(val_str)
            if val <= 0:
                messagebox.showerror("Invalid Weight", "Distance must be > 0.", parent=self)
                return
            self.result = round(val, 1)
            self.destroy()
        except ValueError:
            messagebox.showerror("Invalid Number", "Please enter a valid numeric value.", parent=self)

    def on_cancel(self):
        self.result = None
        self.destroy()


class DSAExecutionLogDialog(tk.Toplevel):
    """Floating/hovering real-time DSA Algorithm Execution Log dialog."""
    def __init__(self, parent, initial_history=None, on_clear_cb=None, on_close_cb=None):
        super().__init__(parent)
        self.title("DSA Algorithm Execution Log")
        self.configure(bg=THEME["bg_card"])
        self.transient(parent)
        self.on_clear_cb = on_clear_cb
        self.on_close_cb = on_close_cb
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        w, h = 540, 440
        # Position floating nicely offset near top right
        x = parent.winfo_rootx() + max(40, parent.winfo_width() - w - 24)
        y = parent.winfo_rooty() + 80
        self.geometry(f"{w}x{h}+{max(50, x)}+{max(50, y)}")

        # Header Bar
        header = tk.Frame(self, bg=THEME["bg_card"], padx=14, pady=10)
        header.pack(fill="x")

        tk.Label(
            header,
            text="📜 DSA EXECUTION LOG",
            font=FONT_TITLE,
            bg=THEME["bg_card"],
            fg="#38BDF8"
        ).pack(side="left")

        self.count_var = tk.StringVar(value="0 entries")
        tk.Label(
            header,
            textvariable=self.count_var,
            font=FONT_SMALL,
            bg="#1E293B",
            fg="#94A3B8",
            padx=8,
            pady=2
        ).pack(side="left", padx=(12, 0))

        tk.Button(
            header,
            text="Close",
            font=FONT_SMALL,
            bg=THEME["btn_bg"],
            fg=THEME["text_secondary"],
            activebackground=THEME["btn_hover"],
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=8,
            pady=3,
            cursor="hand2",
            command=self.on_close
        ).pack(side="right", padx=(6, 0))

        tk.Button(
            header,
            text="Clear",
            font=FONT_SMALL,
            bg=THEME["btn_bg"],
            fg=THEME["text_secondary"],
            activebackground=THEME["btn_hover"],
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=8,
            pady=3,
            cursor="hand2",
            command=self.on_clear
        ).pack(side="right")

        # Scrollable Monospace Text Area
        body = tk.Frame(self, bg=THEME["bg_card"], padx=14, pady=6)
        body.pack(fill="both", expand=True, pady=(0, 10))

        self.text_area = tk.Text(
            body,
            bg="#0D1322",
            fg="#E2E8F0",
            insertbackground="#38BDF8",
            font=FONT_MONO,
            bd=1,
            relief="solid",
            wrap="word",
            state="normal"
        )
        self.text_area.pack(side="left", fill="both", expand=True)

        scrollbar = tk.Scrollbar(body, command=self.text_area.yview, bg=THEME["bg_card"])
        scrollbar.pack(side="right", fill="y")
        self.text_area.configure(yscrollcommand=scrollbar.set)

        self.entry_count = 0
        if initial_history:
            for line in initial_history:
                self.text_area.insert(tk.END, line + "\n")
                self.entry_count += 1
            self.text_area.see(tk.END)
            self.count_var.set(f"{self.entry_count} entries")

        self.text_area.config(state="disabled")

        # Bottom tip
        tip_bar = tk.Frame(self, bg="#0F172A", padx=14, pady=6)
        tip_bar.pack(fill="x", side="bottom")
        tk.Label(
            tip_bar,
            text="Live DSA Telemetry • Queue snapshots, relaxations & tree expansions stream here.",
            font=FONT_SMALL,
            bg="#0F172A",
            fg="#64748B"
        ).pack(side="left")

    def append_entry(self, text):
        self.text_area.config(state="normal")
        self.text_area.insert(tk.END, text + "\n")
        self.text_area.see(tk.END)
        self.text_area.config(state="disabled")
        self.entry_count += 1
        self.count_var.set(f"{self.entry_count} entries")

    def clear_entries(self):
        self.text_area.config(state="normal")
        self.text_area.delete("1.0", tk.END)
        self.text_area.config(state="disabled")
        self.entry_count = 0
        self.count_var.set("0 entries")

    def on_clear(self):
        self.clear_entries()
        if self.on_clear_cb:
            self.on_clear_cb()

    def on_close(self):
        if self.on_close_cb:
            self.on_close_cb()
        self.destroy()


class DSAInspectorDialog(tk.Toplevel):
    """Modal displaying full Adjacency List and Graph Theory metrics."""
    def __init__(self, parent, graph):
        super().__init__(parent)
        self.title("DSA Graph Theory Inspector")
        self.configure(bg=THEME["bg_card"])
        self.transient(parent)
        self.grab_set()

        w, h = 560, 480
        x = parent.winfo_rootx() + (parent.winfo_width() - w) // 2
        y = parent.winfo_rooty() + (parent.winfo_height() - h) // 2
        self.geometry(f"{w}x{h}+{max(50, x)}+{max(50, y)}")

        tk.Label(
            self,
            text="DSA GRAPH THEORY & ADJACENCY LIST",
            font=FONT_TITLE,
            bg=THEME["bg_card"],
            fg="#38BDF8"
        ).pack(pady=(16, 4))

        # Metrics bar
        v_count = len(graph.nodes)
        e_count = len(graph.edges)
        density = graph.get_graph_density()
        avg_deg = graph.get_avg_degree()

        metrics_frame = tk.Frame(self, bg=THEME["bg_card_inner"], padx=12, pady=8)
        metrics_frame.pack(fill="x", padx=16, pady=8)

        stats = [
            (f"|V| (Vertices): {v_count}", "#34D399"),
            (f"|E| (Edges): {e_count}", "#38BDF8"),
            (f"Density: {density}", "#FBBF24"),
            (f"Avg Degree: {avg_deg}", "#A78BFA"),
        ]
        for idx, (txt, col) in enumerate(stats):
            tk.Label(metrics_frame, text=txt, font=FONT_BOLD, bg=THEME["bg_card_inner"], fg=col).grid(row=0, column=idx, padx=10)

        # Adjacency List representation
        tk.Label(
            self,
            text="Adjacency List in Memory (adj[u] = [(v, weight), ...]):",
            font=FONT_SUBTITLE,
            bg=THEME["bg_card"],
            fg=THEME["text_secondary"]
        ).pack(anchor="w", padx=16, pady=(8, 4))

        adj_dict = graph.get_adjacency_list_dict()
        text_area = tk.Text(self, bg="#0D1322", fg="#E2E8F0", font=FONT_MONO, bd=1, relief="solid")
        text_area.pack(fill="both", expand=True, padx=16, pady=4)

        if not adj_dict:
            text_area.insert(tk.END, "# Empty graph: No vertices or edges present.")
        else:
            text_area.insert(tk.END, "adjacency_list = {\n")
            for node_lbl, nbrs in sorted(adj_dict.items()):
                nbr_str = ", ".join(f"('{nbr}', {w})" for nbr, w in nbrs)
                text_area.insert(tk.END, f"    '{node_lbl}': [{nbr_str}],\n")
            text_area.insert(tk.END, "}\n")

        text_area.config(state="disabled")

        close_btn = tk.Button(
            self,
            text="Close Inspector",
            font=FONT_BOLD,
            bg="#2563EB",
            fg="#FFFFFF",
            relief="flat",
            bd=0,
            command=self.destroy,
            cursor="hand2"
        )
        close_btn.pack(pady=12, ipady=4, ipadx=16)


# ==============================================================================
# DRAFTS SYSTEM: STORAGE, THUMBNAIL RASTERIZER & DIALOG
# ==============================================================================
DRAFTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drafts")
DRAFTS_FILE = os.path.join(DRAFTS_DIR, "drafts_data.py")
MAX_DRAFTS = 10


def generate_draft_thumbnail(nodes, edges, out_path, w=260, h=150):
    """
    Pure Python rasterizer that renders a thumbnail image of the graph
    and exports it as a PNG image file.
    """
    if not nodes:
        return False

    pad = 22
    xs = [n["x"] for n in nodes]
    ys = [n["y"] for n in nodes]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(1.0, max_x - min_x)
    span_y = max(1.0, max_y - min_y)
    scale = min((w - 2 * pad) / span_x, (h - 2 * pad) / span_y)
    scale = min(scale, 1.2)  # Cap scale to avoid over-stretching small graphs
    off_x = pad + (w - 2 * pad - span_x * scale) / 2.0 - min_x * scale
    off_y = pad + (h - 2 * pad - span_y * scale) / 2.0 - min_y * scale

    buf = bytearray([13, 19, 34] * (w * h))  # Midnight navy #0D1322

    def put_pixel(x, y, r, g, b):
        if 0 <= x < w and 0 <= y < h:
            i = (y * w + x) * 3
            buf[i] = r
            buf[i + 1] = g
            buf[i + 2] = b

    # Outer border #374151
    for x in range(w):
        put_pixel(x, 0, 55, 65, 81)
        put_pixel(x, h - 1, 55, 65, 81)
    for y in range(h):
        put_pixel(0, y, 55, 65, 81)
        put_pixel(w - 1, y, 55, 65, 81)

    def draw_thick_line(x0, y0, x1, y1, r, g, b, thickness=2):
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            for tx in range(-thickness // 2, thickness // 2 + 1):
                for ty in range(-thickness // 2, thickness // 2 + 1):
                    put_pixel(x0 + tx, y0 + ty, r, g, b)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    node_map = {n["id"]: n for n in nodes}
    for e in edges:
        u = node_map.get(e["u_id"])
        v = node_map.get(e["v_id"])
        if u and v:
            ux = int(u["x"] * scale + off_x)
            uy = int(u["y"] * scale + off_y)
            vx = int(v["x"] * scale + off_x)
            vy = int(v["y"] * scale + off_y)
            draw_thick_line(ux, uy, vx, vy, 71, 85, 105, thickness=2)

    rad = 8
    for n in nodes:
        cx = int(n["x"] * scale + off_x)
        cy = int(n["y"] * scale + off_y)
        if n.get("is_start"):
            fill_col = (16, 185, 129)   # Emerald
        elif n.get("is_goal"):
            fill_col = (239, 68, 68)    # Crimson
        elif n.get("is_obstacle"):
            fill_col = (100, 116, 139)  # Dim Slate
        else:
            fill_col = (56, 189, 248)   # Sky Blue

        for dy in range(-rad, rad + 1):
            for dx in range(-rad, rad + 1):
                d2 = dx * dx + dy * dy
                if d2 <= rad * rad:
                    if d2 >= (rad - 1) * (rad - 1):
                        put_pixel(cx + dx, cy + dy, 241, 245, 249)
                    else:
                        put_pixel(cx + dx, cy + dy, *fill_col)

    ppm_path = out_path + ".ppm"
    try:
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(ppm_path, "wb") as f:
            f.write(f"P6\n{w} {h}\n255\n".encode("ascii") + buf)

        # Convert to PNG using PhotoImage
        img = tk.PhotoImage(file=ppm_path)
        img.write(out_path, format="png")
        if os.path.exists(ppm_path):
            os.remove(ppm_path)
        return True
    except Exception:
        # Fallback to PPM
        if os.path.exists(ppm_path):
            try:
                if not os.path.exists(out_path):
                    shutil.copyfile(ppm_path, out_path)
                os.remove(ppm_path)
            except Exception:
                pass
        return False


class DraftManager:
    """
    Manages persistent storage of graph drafts across app restarts.
    Stores up to the last 10 drafts in a pure Python file with preview images.
    """
    def __init__(self, drafts_dir=None):
        self.drafts_dir = drafts_dir if drafts_dir is not None else DRAFTS_DIR
        self.drafts_file = os.path.join(self.drafts_dir, "drafts_data.py")
        self.max_drafts = MAX_DRAFTS
        self.drafts = []
        os.makedirs(self.drafts_dir, exist_ok=True)
        self.load_drafts()

    def load_drafts(self):
        if os.path.exists(self.drafts_file):
            try:
                with open(self.drafts_file, "r", encoding="utf-8") as f:
                    code_text = f.read()
                tree = ast.parse(code_text)
                for stmt in tree.body:
                    if isinstance(stmt, ast.Assign):
                        for target in stmt.targets:
                            if isinstance(target, ast.Name) and target.id == "DRAFTS":
                                val = ast.literal_eval(stmt.value)
                                if isinstance(val, list):
                                    self.drafts = val[:self.max_drafts]
                                    return self.drafts
                self.drafts = []
            except Exception:
                self.drafts = []
        else:
            self.drafts = []
        return self.drafts

    def save_to_disk(self):
        try:
            os.makedirs(self.drafts_dir, exist_ok=True)
            formatted_data = pprint.pformat(self.drafts, indent=4, width=100)
            content = (
                '"""\n'
                'Persistent Graph Drafts Storage (Pure Python).\n'
                'Automatically maintained by the Pathfinding Visualizer.\n'
                '"""\n\n'
                f'DRAFTS = {formatted_data}\n'
            )
            with open(self.drafts_file, "w", encoding="utf-8") as f:
                f.write(content)
        except Exception as e:
            print(f"[Error saving drafts to Python file]: {e}")

    def get_drafts(self):
        return self.drafts

    def get_next_draft_number(self):
        existing_nums = []
        for d in self.drafts:
            name = d.get("name", "")
            if name.startswith("Draft "):
                parts = name.split()
                if len(parts) >= 2 and parts[1].isdigit():
                    existing_nums.append(int(parts[1]))
        if existing_nums:
            return max(existing_nums) + 1
        return len(self.drafts) + 1

    def save_draft(self, graph, name=None):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        time_id = int(time.time() * 1000)
        draft_id = f"draft_{time_id}"

        if not name:
            draft_num = self.get_next_draft_number()
            name = f"Draft {draft_num}"

        nodes_data = []
        for n in graph.nodes.values():
            nodes_data.append({
                "id": n.id,
                "label": n.label,
                "x": float(n.x),
                "y": float(n.y),
                "is_start": bool(n.is_start),
                "is_goal": bool(n.is_goal),
                "is_obstacle": bool(n.is_obstacle)
            })

        edges_data = []
        for (u_id, v_id), weight in graph.edges.items():
            edges_data.append({
                "u_id": u_id,
                "v_id": v_id,
                "weight": float(weight)
            })

        start_node = graph.get_start_node()
        goal_node = graph.get_goal_node()
        start_label = start_node.label if start_node else None
        goal_label = goal_node.label if goal_node else None

        img_filename = f"{draft_id}.png"
        img_path = os.path.join(self.drafts_dir, img_filename)

        # Generate image preview
        generate_draft_thumbnail(nodes_data, edges_data, img_path, w=260, h=150)

        draft_item = {
            "id": draft_id,
            "name": name,
            "created_at": timestamp,
            "node_count": len(nodes_data),
            "edge_count": len(edges_data),
            "start_label": start_label,
            "goal_label": goal_label,
            "image_path": img_path,
            "nodes": nodes_data,
            "edges": edges_data
        }

        # Add to beginning of drafts (newest first)
        self.drafts.insert(0, draft_item)

        # Keep strictly last 10 drafts
        while len(self.drafts) > self.max_drafts:
            old_draft = self.drafts.pop()
            old_img = old_draft.get("image_path")
            if old_img and os.path.exists(old_img):
                try:
                    os.remove(old_img)
                except Exception:
                    pass

        self.save_to_disk()
        return draft_item

    def delete_draft(self, draft_id):
        to_del = None
        for d in self.drafts:
            if d.get("id") == draft_id:
                to_del = d
                break
        if to_del:
            self.drafts.remove(to_del)
            img_path = to_del.get("image_path")
            if img_path and os.path.exists(img_path):
                try:
                    os.remove(img_path)
                except Exception:
                    pass
            self.save_to_disk()
            return True
        return False


class DraftsDialog(tk.Toplevel):
    """
    Visual gallery window displaying saved map drafts (last 10).
    Clicking any draft's image or 'Open Map' button loads that map into the visualizer.
    """
    def __init__(self, parent, draft_manager, on_select_draft=None):
        super().__init__(parent)
        self.parent = parent
        self.draft_manager = draft_manager
        self.on_select_draft = on_select_draft
        self.title("Saved Map Drafts (Last 10)")
        self.configure(bg=THEME["bg_app"])
        self.transient(parent)
        self.photos = []

        w, h = 820, 620
        x = parent.winfo_rootx() + (parent.winfo_width() - w) // 2
        y = parent.winfo_rooty() + (parent.winfo_height() - h) // 2
        self.geometry(f"{w}x{h}+{max(40, x)}+{max(40, y)}")
        self.minsize(680, 500)

        # Header Bar
        header = tk.Frame(self, bg=THEME["bg_panel"], padx=18, pady=12)
        header.pack(fill="x", side="top")

        title_frame = tk.Frame(header, bg=THEME["bg_panel"])
        title_frame.pack(side="left")

        tk.Label(
            title_frame,
            text="📁 SAVED MAP DRAFTS",
            font=FONT_TITLE,
            bg=THEME["bg_panel"],
            fg="#38BDF8"
        ).pack(anchor="w")

        tk.Label(
            title_frame,
            text="Click any map preview image to open and edit that graph. Stores last 10 drafts.",
            font=FONT_SMALL,
            bg=THEME["bg_panel"],
            fg=THEME["text_secondary"]
        ).pack(anchor="w", pady=(2, 0))

        right_frame = tk.Frame(header, bg=THEME["bg_panel"])
        right_frame.pack(side="right")

        self.count_badge = tk.Label(
            right_frame,
            text="",
            font=FONT_BOLD,
            bg="#1F2937",
            fg="#34D399",
            padx=10,
            pady=4
        )
        self.count_badge.pack(side="left", padx=(0, 10))

        tk.Button(
            right_frame,
            text="✕ Close",
            font=FONT_SMALL,
            bg=THEME["btn_bg"],
            fg=THEME["text_secondary"],
            activebackground=THEME["btn_hover"],
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=10,
            pady=4,
            cursor="hand2",
            command=self.destroy
        ).pack(side="left")

        # Scrollable container
        container = tk.Frame(self, bg=THEME["bg_app"])
        container.pack(fill="both", expand=True, padx=16, pady=12)

        self.canvas = tk.Canvas(container, bg=THEME["bg_app"], bd=0, highlightthickness=0)
        self.scrollbar = tk.Scrollbar(container, orient="vertical", command=self.canvas.yview, bg=THEME["bg_panel"])
        self.scrollable_frame = tk.Frame(self.canvas, bg=THEME["bg_app"])

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )
        self.canvas_window = self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")

        self.canvas.bind(
            "<Configure>",
            lambda e: self.canvas.itemconfig(self.canvas_window, width=e.width)
        )

        self.bind_mousewheel(self)
        self.refresh_drafts()

    def bind_mousewheel(self, widget):
        widget.bind("<MouseWheel>", self._on_mousewheel, add="+")
        for child in widget.winfo_children():
            self.bind_mousewheel(child)

    def _on_mousewheel(self, event):
        if self.canvas.winfo_exists():
            self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def refresh_drafts(self):
        for widget in self.scrollable_frame.winfo_children():
            widget.destroy()
        self.photos.clear()

        drafts = self.draft_manager.get_drafts()
        count = len(drafts)
        self.count_badge.config(text=f"{count} / 10 Saved")

        if not drafts:
            empty_box = tk.Frame(self.scrollable_frame, bg=THEME["bg_card"], padx=30, pady=40)
            empty_box.pack(fill="x", pady=40, padx=20)

            tk.Label(
                empty_box,
                text="📭",
                font=(FONT_FAMILY, 32),
                bg=THEME["bg_card"],
                fg=THEME["text_secondary"]
            ).pack(pady=(0, 10))

            tk.Label(
                empty_box,
                text="No Drafts Saved Yet",
                font=FONT_TITLE,
                bg=THEME["bg_card"],
                fg=THEME["text_primary"]
            ).pack()

            tk.Label(
                empty_box,
                text="Design or modify a graph on the canvas and click '💾 Save Draft' beside Center.\nDrafts are automatically stored and persisted across restarts!",
                font=FONT_REGULAR,
                bg=THEME["bg_card"],
                fg=THEME["text_secondary"],
                justify="center"
            ).pack(pady=(8, 0))
            return

        grid_frame = tk.Frame(self.scrollable_frame, bg=THEME["bg_app"])
        grid_frame.pack(fill="both", expand=True)
        grid_frame.columnconfigure(0, weight=1)
        grid_frame.columnconfigure(1, weight=1)

        for idx, draft in enumerate(drafts):
            row = idx // 2
            col = idx % 2

            card = tk.Frame(
                grid_frame,
                bg=THEME["bg_card"],
                bd=1,
                relief="solid",
                highlightthickness=1,
                highlightbackground=THEME["border"],
                padx=12,
                pady=12
            )
            card.grid(row=row, column=col, padx=8, pady=8, sticky="nsew")

            # Header metadata
            top_meta = tk.Frame(card, bg=THEME["bg_card"])
            top_meta.pack(fill="x", pady=(0, 8))

            draft_name = draft.get("name", f"Draft {idx + 1}")
            tk.Label(
                top_meta,
                text=draft_name,
                font=FONT_BOLD,
                bg=THEME["bg_card"],
                fg=THEME["text_primary"]
            ).pack(side="left")

            created_time = draft.get("created_at", "")
            tk.Label(
                top_meta,
                text=created_time,
                font=FONT_SMALL,
                bg=THEME["bg_card"],
                fg=THEME["text_secondary"]
            ).pack(side="right")

            # Image Preview Container
            img_container = tk.Frame(
                card,
                bg="#0D1322",
                bd=1,
                relief="solid",
                highlightthickness=1,
                highlightbackground=THEME["border"]
            )
            img_container.pack(fill="x", pady=4)

            img_path = draft.get("image_path", "")
            img_loaded = False

            if img_path and os.path.exists(img_path):
                try:
                    photo = tk.PhotoImage(file=img_path)
                    self.photos.append(photo)
                    img_lbl = tk.Label(img_container, image=photo, bg="#0D1322", cursor="hand2")
                    img_lbl.image = photo
                    img_lbl.pack(padx=2, pady=2, fill="both")
                    img_lbl.bind("<Button-1>", lambda e, d=draft: self.select_draft(d))
                    img_loaded = True
                except Exception:
                    img_loaded = False

            if not img_loaded:
                fallback_canvas = tk.Canvas(
                    img_container,
                    width=260,
                    height=150,
                    bg="#0D1322",
                    bd=0,
                    highlightthickness=0,
                    cursor="hand2"
                )
                fallback_canvas.pack(padx=2, pady=2)
                self.draw_mini_graph(fallback_canvas, draft.get("nodes", []), draft.get("edges", []))
                fallback_canvas.bind("<Button-1>", lambda e, d=draft: self.select_draft(d))

            click_hint = tk.Label(
                card,
                text="🖱️ Click image to open map",
                font=(FONT_FAMILY, 8, "italic"),
                bg=THEME["bg_card"],
                fg="#38BDF8",
                cursor="hand2"
            )
            click_hint.pack(pady=(4, 6))
            click_hint.bind("<Button-1>", lambda e, d=draft: self.select_draft(d))

            # Bottom stats
            stats_frame = tk.Frame(card, bg=THEME["bg_card"])
            stats_frame.pack(fill="x", pady=(2, 8))

            v_count = draft.get("node_count", len(draft.get("nodes", [])))
            e_count = draft.get("edge_count", len(draft.get("edges", [])))
            start_lbl = draft.get("start_label") or "-"
            goal_lbl = draft.get("goal_label") or "-"

            stat_txt = f"|V|: {v_count}  |E|: {e_count}  Start: {start_lbl}  Goal: {goal_lbl}"
            tk.Label(
                stats_frame,
                text=stat_txt,
                font=FONT_SMALL,
                bg=THEME["bg_card"],
                fg=THEME["text_secondary"]
            ).pack(anchor="w")

            btn_frame = tk.Frame(card, bg=THEME["bg_card"])
            btn_frame.pack(fill="x")

            open_btn = tk.Button(
                btn_frame,
                text="📂 Open Map",
                font=FONT_BOLD,
                bg="#2563EB",
                fg="#FFFFFF",
                activebackground="#1D4ED8",
                activeforeground="#FFFFFF",
                relief="flat",
                bd=0,
                padx=10,
                pady=4,
                cursor="hand2",
                command=lambda d=draft: self.select_draft(d)
            )
            open_btn.pack(side="left")

            del_btn = tk.Button(
                btn_frame,
                text="🗑️ Delete",
                font=FONT_SMALL,
                bg=THEME["btn_bg"],
                fg="#EF4444",
                activebackground="#991B1B",
                activeforeground="#FFFFFF",
                relief="flat",
                bd=0,
                padx=8,
                pady=4,
                cursor="hand2",
                command=lambda d=draft: self.delete_draft_item(d)
            )
            del_btn.pack(side="right")

    def draw_mini_graph(self, canvas, nodes, edges):
        if not nodes:
            canvas.create_text(130, 75, text="Empty Graph", fill="#64748B", font=FONT_SMALL)
            return
        w, h = 260, 150
        pad = 20
        xs = [n["x"] for n in nodes]
        ys = [n["y"] for n in nodes]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max(1.0, max_x - min_x)
        span_y = max(1.0, max_y - min_y)
        scale = min((w - 2 * pad) / span_x, (h - 2 * pad) / span_y)
        scale = min(scale, 1.2)
        off_x = pad + (w - 2 * pad - span_x * scale) / 2 - min_x * scale
        off_y = pad + (h - 2 * pad - span_y * scale) / 2 - min_y * scale

        node_map = {n["id"]: n for n in nodes}
        for e in edges:
            u = node_map.get(e["u_id"])
            v = node_map.get(e["v_id"])
            if u and v:
                ux = u["x"] * scale + off_x
                uy = u["y"] * scale + off_y
                vx = v["x"] * scale + off_x
                vy = v["y"] * scale + off_y
                canvas.create_line(ux, uy, vx, vy, fill="#475569", width=2)

        rad = 7
        for n in nodes:
            cx = n["x"] * scale + off_x
            cy = n["y"] * scale + off_y
            if n.get("is_start"):
                fill = "#10B981"
            elif n.get("is_goal"):
                fill = "#EF4444"
            elif n.get("is_obstacle"):
                fill = "#64748B"
            else:
                fill = "#38BDF8"
            canvas.create_oval(cx - rad, cy - rad, cx + rad, cy + rad, fill=fill, outline="#FFFFFF", width=1)
            canvas.create_text(cx, cy, text=n.get("label", "")[:2], fill="#0B0F19", font=(FONT_FAMILY, 6, "bold"))

    def select_draft(self, draft):
        if self.on_select_draft:
            self.on_select_draft(draft)
        self.destroy()

    def delete_draft_item(self, draft):
        draft_name = draft.get("name", "Draft")
        if messagebox.askyesno("Delete Draft", f"Are you sure you want to delete '{draft_name}'?", parent=self):
            self.draft_manager.delete_draft(draft.get("id"))
            self.refresh_drafts()


# ==============================================================================
# MAIN APPLICATION GUI CLASS
# ==============================================================================
class PathfindingVisualizerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Interactive Graph Pathfinding Visualizer (DSA Edition)")
        self.root.geometry("1280x820")
        self.root.minsize(1080, 700)
        self.root.configure(bg=THEME["bg_app"])

        # Core Graph
        self.graph = Graph()
        self.current_mode = "ADD_NODE"
        self.edge_start_node = None

        # Dragging
        self.dragged_node = None
        self.drag_offset_x = 0
        self.drag_offset_y = 0

        # Animation & Generator
        self.anim_generator = None
        self.anim_running = False
        self.anim_paused = False
        self.anim_speed_ms = 120
        self.timer_id = None

        # Highlights
        self.frontier_nodes = set()
        self.visited_nodes = set()
        self.active_node = None
        self.examining_edge = None
        self.searched_edges = set()
        self.final_path = []
        self.final_path_edges = set()

        # Telemetry
        self.metric_dist = 0.0
        self.metric_explored = 0
        self.metric_steps = 0
        self.metric_path_seq = []
        self.last_heap_state = "Empty"
        self.endless_timer_id = None
        self.simulation_active = False

        # Logs & Floating Inspector Window
        self.log_history = []
        self.log_dialog = None

        # Drafts System (Persistent storage up to 10 drafts)
        self.draft_manager = DraftManager()
        self.drafts_dialog = None

        # Build UI
        self.create_header_toolbar()
        self.create_main_content()
        self.create_status_bar()
        self.root.bind("<space>", lambda e: self.toggle_pause())

        # Load initial preset
        self.has_initial_centered = False
        self.load_preset_simple()
        self.root.after(60, self.center_graph)

    # --------------------------------------------------------------------------
    # TOP TOOLBAR
    # --------------------------------------------------------------------------
    def create_header_toolbar(self):
        self.header_frame = tk.Frame(self.root, bg=THEME["bg_panel"], bd=0)
        self.header_frame.pack(side="top", fill="x")

        # ----------------------------------------------------------------------
        # ROW 1: System Title, Algorithm Selector, Presets & Execution Controls
        # ----------------------------------------------------------------------
        row1 = tk.Frame(self.header_frame, bg=THEME["bg_panel"])
        row1.pack(side="top", fill="x", padx=12, pady=(6, 4))

        # RIGHT: Execution Controls (Pack FIRST so always anchored & never clipped)
        actions_box = tk.Frame(row1, bg=THEME["bg_panel"])
        actions_box.pack(side="right", padx=(4, 0))

        self.btn_run = tk.Button(
            actions_box,
            text="▶ RUN",
            font=(FONT_FAMILY, 9, "bold"),
            bg="#059669",
            fg="#FFFFFF",
            activebackground="#047857",
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=10,
            pady=4,
            cursor="hand2",
            command=self.start_visualization
        )
        self.btn_run.pack(side="left", padx=2)

        self.btn_pause = tk.Button(
            actions_box,
            text="⏸ Pause",
            font=FONT_BOLD,
            bg=THEME["btn_bg"],
            fg="#FCD34D",
            activebackground=THEME["btn_hover"],
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=6,
            pady=4,
            cursor="hand2",
            command=self.toggle_pause,
            state="disabled"
        )
        self.btn_pause.pack(side="left", padx=2)

        self.btn_step = tk.Button(
            actions_box,
            text="⏭ Step",
            font=FONT_BOLD,
            bg=THEME["btn_bg"],
            fg="#FBBF24",
            activebackground=THEME["btn_hover"],
            activeforeground="#FDE68A",
            relief="flat",
            bd=0,
            padx=6,
            pady=4,
            cursor="hand2",
            command=self.step_execution
        )
        self.btn_step.pack(side="left", padx=2)

        self.btn_simulate = tk.Button(
            actions_box,
            text="🚗 Simulate",
            font=FONT_BOLD,
            bg=THEME["btn_bg"],
            fg="#60A5FA",
            activebackground=THEME["btn_hover"],
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=6,
            pady=4,
            cursor="hand2",
            command=self.start_route_simulation,
            state="disabled"
        )
        self.btn_simulate.pack(side="left", padx=2)

        self.endless_var = tk.BooleanVar(value=True)
        self.chk_endless = tk.Checkbutton(
            actions_box,
            text="Loop",
            variable=self.endless_var,
            font=FONT_SMALL,
            bg=THEME["bg_panel"],
            fg=THEME["text_primary"],
            selectcolor=THEME["btn_bg"],
            activebackground=THEME["bg_panel"],
            activeforeground="#FFFFFF"
        )
        self.chk_endless.pack(side="left", padx=(2, 6))

        # Clean vertical separator before Reset / Clear actions
        tk.Frame(actions_box, bg=THEME["border"], width=1, height=20).pack(side="left", fill="y", padx=3, pady=2)

        # ADJUSTED RESET BUTTON - Generously sized, prominent, and completely visible
        self.btn_reset = tk.Button(
            actions_box,
            text="🔄 Reset",
            font=FONT_BOLD,
            bg="#1E293B",
            fg="#38BDF8",
            activebackground="#334155",
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=10,
            pady=4,
            cursor="hand2",
            command=self.reset_search_path
        )
        self.btn_reset.pack(side="left", padx=3)

        self.btn_clear = tk.Button(
            actions_box,
            text="🗑 Clear",
            font=FONT_BOLD,
            bg="#1E293B",
            fg="#F87171",
            activebackground="#991B1B",
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=10,
            pady=4,
            cursor="hand2",
            command=self.clear_all_canvas
        )
        self.btn_clear.pack(side="left", padx=3)

        # LEFT: Title Box
        title_box = tk.Frame(row1, bg=THEME["bg_panel"])
        title_box.pack(side="left", padx=(0, 6))

        tk.Label(
            title_box,
            text="PATHFINDING VISUALIZER",
            font=(FONT_FAMILY, 10, "bold"),
            bg=THEME["bg_panel"],
            fg=THEME["text_primary"]
        ).pack(anchor="w")

        tk.Label(
            title_box,
            text="DSA Graph Engine • Python & Tkinter",
            font=FONT_SMALL,
            bg=THEME["bg_panel"],
            fg=THEME["text_accent"]
        ).pack(anchor="w")

        tk.Frame(row1, bg=THEME["border"], width=1, height=22).pack(side="left", fill="y", padx=6, pady=4)

        # Algorithm Selector
        algo_box = tk.Frame(row1, bg=THEME["bg_panel"])
        algo_box.pack(side="left", padx=4)

        tk.Label(
            algo_box,
            text="ALGORITHM",
            font=(FONT_FAMILY, 8, "bold"),
            bg=THEME["bg_panel"],
            fg=THEME["text_secondary"]
        ).pack(anchor="w")

        self.algo_var = tk.StringVar(value="Dijkstra's Algorithm")
        self.algo_combo = ttk.Combobox(
            algo_box,
            textvariable=self.algo_var,
            values=[
                "Dijkstra's Algorithm",
                "A* Search Algorithm",
                "Greedy Best-First Search",
                "Breadth-First Search (BFS)"
            ],
            state="readonly",
            width=18,
            font=FONT_BOLD
        )
        self.algo_combo.set("Dijkstra's Algorithm")
        self.algo_combo.pack(pady=(2, 0))
        self.algo_combo.bind("<<ComboboxSelected>>", lambda e: self.on_algo_change())

        tk.Frame(row1, bg=THEME["border"], width=1, height=22).pack(side="left", fill="y", padx=6, pady=4)

        # Presets, Inspector & Floating Log Buttons
        aux_box = tk.Frame(row1, bg=THEME["bg_panel"])
        aux_box.pack(side="left", padx=2)

        preset_mb = tk.Menubutton(
            aux_box,
            text="🎲 Presets ▾",
            font=FONT_BOLD,
            bg=THEME["btn_bg"],
            fg=THEME["btn_text"],
            activebackground=THEME["btn_hover"],
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=6,
            pady=4,
            cursor="hand2"
        )
        preset_menu = tk.Menu(preset_mb, tearoff=0, bg=THEME["bg_card"], fg=THEME["text_primary"], activebackground="#2563EB", font=FONT_REGULAR)
        preset_menu.add_command(label="1. Simple Road Network", command=self.load_preset_simple)
        preset_menu.add_command(label="2. Diamond & Detours", command=self.load_preset_diamond)
        preset_menu.add_command(label="3. Obstacle Maze Barrier", command=self.load_preset_obstacle_wall)
        preset_menu.add_command(label="4. Grid Road Map", command=self.load_preset_grid)
        preset_menu.add_separator()
        preset_menu.add_command(label="🎯 Center in Middle", command=self.center_graph)
        preset_mb["menu"] = preset_menu
        preset_mb.pack(side="left", padx=2)

        self.btn_inspector = tk.Button(
            aux_box,
            text="📊 Inspector",
            font=FONT_BOLD,
            bg=THEME["btn_bg"],
            fg="#38BDF8",
            activebackground=THEME["btn_hover"],
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=6,
            pady=4,
            cursor="hand2",
            command=self.open_dsa_inspector
        )
        self.btn_inspector.pack(side="left", padx=2)

        self.btn_header_log = tk.Button(
            aux_box,
            text="📜 Log",
            font=FONT_BOLD,
            bg=THEME["btn_bg"],
            fg="#38BDF8",
            activebackground=THEME["btn_hover"],
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=6,
            pady=4,
            cursor="hand2",
            command=self.open_dsa_log
        )
        self.btn_header_log.pack(side="left", padx=2)

        self.btn_drafts = tk.Button(
            aux_box,
            text="📁 Drafts",
            font=FONT_BOLD,
            bg=THEME["btn_bg"],
            fg="#38BDF8",
            activebackground=THEME["btn_hover"],
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=6,
            pady=4,
            cursor="hand2",
            command=self.open_drafts_window
        )
        self.btn_drafts.pack(side="left", padx=2)

        # ----------------------------------------------------------------------
        # ROW 2: Canvas Mode Tools & Live Mode Hint
        # ----------------------------------------------------------------------
        row2 = tk.Frame(self.header_frame, bg="#0F172A", height=36)
        row2.pack(side="top", fill="x", padx=0, pady=(2, 0))

        modes_box = tk.Frame(row2, bg="#0F172A")
        modes_box.pack(side="left", padx=10, pady=3)

        tk.Label(
            modes_box,
            text="CANVAS TOOLS:",
            font=(FONT_FAMILY, 8, "bold"),
            bg="#0F172A",
            fg=THEME["text_secondary"]
        ).pack(side="left", padx=(0, 6))

        self.mode_buttons = {}
        modes = [
            ("ADD_NODE", "➕ Add Node", "#3B82F6"),
            ("ADD_EDGE", "🔗 Add Road", "#6366F1"),
            ("SET_START", "🟢 Set Start", "#10B981"),
            ("SET_GOAL", "🔴 Set Goal", "#EF4444"),
            ("TOGGLE_OBSTACLE", "🚫 Obstacle", "#64748B"),
            ("MOVE_NODE", "✋ Drag Node", "#8B5CF6"),
            ("DELETE_NODE", "🗑️ Delete Node", "#DC2626"),
        ]

        for mode_key, text, accent_color in modes:
            btn = tk.Button(
                modes_box,
                text=text,
                font=FONT_BOLD,
                bg=THEME["btn_bg"],
                fg=THEME["btn_text"],
                activebackground=accent_color,
                activeforeground="#FFFFFF",
                relief="flat",
                bd=0,
                padx=8,
                pady=4,
                cursor="hand2",
                command=lambda m=mode_key: self.set_mode(m)
            )
            btn.pack(side="left", padx=2)
            self.mode_buttons[mode_key] = (btn, accent_color)

        self.update_mode_button_styles()

        # Re-center button to easily bring the entire pattern to the middle of the screen
        self.btn_center = tk.Button(
            modes_box,
            text="🎯 Center",
            font=FONT_BOLD,
            bg=THEME["btn_bg"],
            fg="#38BDF8",
            activebackground="#0284C7",
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=8,
            pady=4,
            cursor="hand2",
            command=self.center_graph
        )
        self.btn_center.pack(side="left", padx=(4, 2))

        self.btn_save_draft = tk.Button(
            modes_box,
            text="💾 Save Draft",
            font=FONT_BOLD,
            bg=THEME["btn_bg"],
            fg="#10B981",
            activebackground="#059669",
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=8,
            pady=4,
            cursor="hand2",
            command=self.save_draft
        )
        self.btn_save_draft.pack(side="left", padx=2)

        # Canvas Hint displayed on right side of Row 2
        self.canvas_hint_var = tk.StringVar(value="Mode: Click anywhere on the dark canvas to place a Node.")
        self.canvas_hint_bar = tk.Label(
            row2,
            textvariable=self.canvas_hint_var,
            font=FONT_SMALL,
            bg="#0F172A",
            fg=THEME["text_accent"],
            anchor="e",
            padx=14
        )
        self.canvas_hint_bar.pack(side="right", fill="y")

    # --------------------------------------------------------------------------
    # MAIN WORKSPACE
    # --------------------------------------------------------------------------
    def create_main_content(self):
        self.content_frame = tk.Frame(self.root, bg=THEME["bg_app"])
        self.content_frame.pack(fill="both", expand=True, padx=12, pady=8)

        # Left Canvas
        canvas_container = tk.Frame(self.content_frame, bg=THEME["canvas_bg"], bd=1, relief="solid")
        canvas_container.pack(side="left", fill="both", expand=True)

        self.canvas = tk.Canvas(
            canvas_container,
            bg=THEME["canvas_bg"],
            bd=0,
            highlightthickness=0,
            cursor="crosshair"
        )
        self.canvas.pack(fill="both", expand=True)


        # Bindings
        self.canvas.bind("<Button-1>", self.on_canvas_left_click)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)
        self.canvas.bind("<Button-3>", self.on_canvas_right_click)
        self.canvas.bind("<Configure>", self.on_canvas_configure)
        self.canvas.bind("<Motion>", self.on_canvas_motion)

        # Right Telemetry Sidebar
        self.sidebar_frame = tk.Frame(self.content_frame, bg=THEME["bg_panel"], width=350)
        self.sidebar_frame.pack(side="right", fill="y", padx=(10, 0))
        self.sidebar_frame.pack_propagate(False)

        self.create_telemetry_sidebar()

    # --------------------------------------------------------------------------
    # SIDEBAR TELEMETRY & DSA MONITOR
    # --------------------------------------------------------------------------
    def create_telemetry_sidebar(self):
        sidebar = self.sidebar_frame

        # Engine Status Badge
        status_card = tk.Frame(sidebar, bg=THEME["bg_card"], padx=10, pady=8)
        status_card.pack(fill="x", padx=8, pady=(8, 4))

        tk.Label(
            status_card,
            text="ENGINE STATUS",
            font=(FONT_FAMILY, 8, "bold"),
            bg=THEME["bg_card"],
            fg=THEME["text_secondary"]
        ).pack(anchor="w")

        self.status_badge_var = tk.StringVar(value="READY TO RUN")
        self.current_badge_bg = "#064E3B"
        self.current_badge_fg = "#34D399"
        self.status_badge = tk.Button(
            status_card,
            textvariable=self.status_badge_var,
            font=(FONT_FAMILY, 10, "bold"),
            bg="#064E3B",
            fg="#34D399",
            activebackground="#059669",
            activeforeground="#FFFFFF",
            relief="flat",
            bd=0,
            padx=10,
            pady=5,
            cursor="hand2",
            command=self.on_status_badge_click
        )
        self.status_badge.pack(anchor="w", pady=(3, 0), fill="x")
        self.status_badge.bind("<Enter>", self.on_status_badge_enter)
        self.status_badge.bind("<Leave>", self.on_status_badge_leave)

        # Quick Execution Controls in Sidebar
        status_controls = tk.Frame(status_card, bg=THEME["bg_card"])
        status_controls.pack(fill="x", pady=(5, 0))

        self.btn_sidebar_pause = tk.Button(
            status_controls,
            text="⏸ Pause",
            font=(FONT_FAMILY, 9, "bold"),
            bg="#1F2937",
            fg="#FBBF24",
            activebackground="#374151",
            activeforeground="#FDE68A",
            relief="flat",
            bd=0,
            padx=10,
            pady=5,
            cursor="hand2",
            command=self.toggle_pause
        )
        self.btn_sidebar_pause.pack(side="left", fill="x", expand=True, padx=(0, 3))

        self.btn_sidebar_step = tk.Button(
            status_controls,
            text="⏭ Step",
            font=(FONT_FAMILY, 9, "bold"),
            bg="#1F2937",
            fg="#38BDF8",
            activebackground="#374151",
            activeforeground="#7DD3FC",
            relief="flat",
            bd=0,
            padx=10,
            pady=5,
            cursor="hand2",
            command=self.step_execution
        )
        self.btn_sidebar_step.pack(side="left", fill="x", expand=True, padx=(3, 0))

        # DSA Data Structure Monitor Card
        dsa_card = tk.Frame(sidebar, bg=THEME["bg_card"], padx=10, pady=6)
        dsa_card.pack(fill="x", padx=8, pady=4)

        tk.Label(
            dsa_card,
            text="DSA DATA STRUCTURE MONITOR",
            font=(FONT_FAMILY, 8, "bold"),
            bg=THEME["bg_card"],
            fg="#38BDF8"
        ).pack(anchor="w")

        # Active DS & Complexity
        self.dsa_type_var = tk.StringVar(value="Data Structure: Min-Heap (heapq)")
        tk.Label(
            dsa_card,
            textvariable=self.dsa_type_var,
            font=FONT_BOLD,
            bg=THEME["bg_card"],
            fg=THEME["text_primary"]
        ).pack(anchor="w", pady=(2, 0))

        self.dsa_complexity_var = tk.StringVar(value="Complexity: Time O((V+E) log V) | Space O(V)")
        tk.Label(
            dsa_card,
            textvariable=self.dsa_complexity_var,
            font=FONT_MONO_SMALL,
            bg=THEME["bg_card"],
            fg="#FBBF24"
        ).pack(anchor="w")

        # Queue / Heap Live Snapshot
        queue_box = tk.Frame(dsa_card, bg=THEME["bg_card_inner"], padx=6, pady=4)
        queue_box.pack(fill="x", pady=(4, 0))
        tk.Label(queue_box, text="Live Priority Queue State:", font=FONT_SMALL, bg=THEME["bg_card_inner"], fg=THEME["text_secondary"]).pack(anchor="w")
        self.queue_snapshot_var = tk.StringVar(value="Queue: [ Empty ]")
        tk.Label(
            queue_box,
            textvariable=self.queue_snapshot_var,
            font=FONT_MONO_SMALL,
            bg=THEME["bg_card_inner"],
            fg="#38BDF8",
            wraplength=310,
            justify="left"
        ).pack(anchor="w")

        # Speed Control
        speed_card = tk.Frame(sidebar, bg=THEME["bg_card"], padx=10, pady=6)
        speed_card.pack(fill="x", padx=8, pady=4)

        speed_hdr = tk.Frame(speed_card, bg=THEME["bg_card"])
        speed_hdr.pack(fill="x")
        tk.Label(speed_hdr, text="ANIMATION SPEED", font=(FONT_FAMILY, 8, "bold"), bg=THEME["bg_card"], fg=THEME["text_secondary"]).pack(side="left")
        self.speed_label_var = tk.StringVar(value=f"{self.anim_speed_ms} ms")
        tk.Label(speed_hdr, textvariable=self.speed_label_var, font=FONT_SMALL, bg=THEME["bg_card"], fg=THEME["text_accent"]).pack(side="right")

        self.speed_slider = tk.Scale(
            speed_card,
            from_=10,
            to=500,
            orient="horizontal",
            bg=THEME["bg_card"],
            fg=THEME["text_primary"],
            troughcolor="#111827",
            highlightthickness=0,
            bd=0,
            showvalue=False,
            command=self.on_speed_change
        )
        self.speed_slider.set(self.anim_speed_ms)
        self.speed_slider.pack(fill="x", pady=(2, 0))

        # Real-time Metrics Card
        metrics_card = tk.Frame(sidebar, bg=THEME["bg_card"], padx=10, pady=6)
        metrics_card.pack(fill="x", padx=8, pady=4)

        tk.Label(metrics_card, text="REAL-TIME METRICS", font=(FONT_FAMILY, 8, "bold"), bg=THEME["bg_card"], fg=THEME["text_secondary"]).pack(anchor="w", pady=(0, 4))

        grid_frame = tk.Frame(metrics_card, bg=THEME["bg_card"])
        grid_frame.pack(fill="x")

        # 1. Distance
        m1 = tk.Frame(grid_frame, bg=THEME["bg_card_inner"], padx=6, pady=4)
        m1.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        tk.Label(m1, text="Shortest Distance", font=FONT_SMALL, bg=THEME["bg_card_inner"], fg=THEME["text_secondary"]).pack(anchor="w")
        self.val_dist_var = tk.StringVar(value="--")
        tk.Label(m1, textvariable=self.val_dist_var, font=(FONT_FAMILY, 11, "bold"), bg=THEME["bg_card_inner"], fg="#38BDF8").pack(anchor="w")

        # 2. Explored
        m2 = tk.Frame(grid_frame, bg=THEME["bg_card_inner"], padx=6, pady=4)
        m2.grid(row=0, column=1, sticky="nsew", padx=2, pady=2)
        tk.Label(m2, text="Nodes Explored", font=FONT_SMALL, bg=THEME["bg_card_inner"], fg=THEME["text_secondary"]).pack(anchor="w")
        self.val_explored_var = tk.StringVar(value="0")
        tk.Label(m2, textvariable=self.val_explored_var, font=(FONT_FAMILY, 11, "bold"), bg=THEME["bg_card_inner"], fg="#A78BFA").pack(anchor="w")

        # 3. Steps
        m3 = tk.Frame(grid_frame, bg=THEME["bg_card_inner"], padx=6, pady=4)
        m3.grid(row=1, column=0, sticky="nsew", padx=2, pady=2)
        tk.Label(m3, text="Algorithm Steps", font=FONT_SMALL, bg=THEME["bg_card_inner"], fg=THEME["text_secondary"]).pack(anchor="w")
        self.val_steps_var = tk.StringVar(value="0")
        tk.Label(m3, textvariable=self.val_steps_var, font=(FONT_FAMILY, 11, "bold"), bg=THEME["bg_card_inner"], fg="#FBBF24").pack(anchor="w")

        # 4. Frontier
        m4 = tk.Frame(grid_frame, bg=THEME["bg_card_inner"], padx=6, pady=4)
        m4.grid(row=1, column=1, sticky="nsew", padx=2, pady=2)
        tk.Label(m4, text="Frontier Count", font=FONT_SMALL, bg=THEME["bg_card_inner"], fg=THEME["text_secondary"]).pack(anchor="w")
        self.val_frontier_var = tk.StringVar(value="0")
        tk.Label(m4, textvariable=self.val_frontier_var, font=(FONT_FAMILY, 11, "bold"), bg=THEME["bg_card_inner"], fg="#F472B6").pack(anchor="w")

        grid_frame.columnconfigure(0, weight=1)
        grid_frame.columnconfigure(1, weight=1)

        # Path Sequence Box
        seq_frame = tk.Frame(metrics_card, bg=THEME["bg_card_inner"], padx=6, pady=4)
        seq_frame.pack(fill="x", pady=(4, 0))
        tk.Label(seq_frame, text="Shortest Path Sequence:", font=FONT_SMALL, bg=THEME["bg_card_inner"], fg=THEME["text_secondary"]).pack(anchor="w")
        self.val_sequence_var = tk.StringVar(value="None")
        tk.Label(
            seq_frame,
            textvariable=self.val_sequence_var,
            font=(FONT_FAMILY, 9, "bold"),
            bg=THEME["bg_card_inner"],
            fg="#FACC15",
            wraplength=310,
            justify="left"
        ).pack(anchor="w", pady=(1, 0))

        # Color Legend Box
        legend_card = tk.Frame(sidebar, bg=THEME["bg_card"], padx=10, pady=5)
        legend_card.pack(fill="x", padx=8, pady=4)

        tk.Label(legend_card, text="COLOR LEGEND", font=(FONT_FAMILY, 8, "bold"), bg=THEME["bg_card"], fg=THEME["text_secondary"]).pack(anchor="w")
        legend_grid = tk.Frame(legend_card, bg=THEME["bg_card"])
        legend_grid.pack(fill="x", pady=(2, 0))

        legends = [
            ("🟢 Start Node", "#10B981"),
            ("🔴 Goal Node", "#EF4444"),
            ("🟡 Frontier / Open", "#F59E0B"),
            ("🟣 Visited / Settled", "#818CF8"),
            ("🔵 Current Active", "#06B6D4"),
            ("✨ Shortest Path", "#FACC15"),
            ("🛣️ Road Searched", "#22C55E"),
            ("🚫 Obstacle (Blocked)", "#64748B"),
        ]
        for idx, (lbl_txt, col) in enumerate(legends):
            r = idx // 2
            c = idx % 2
            item_f = tk.Frame(legend_grid, bg=THEME["bg_card"])
            item_f.grid(row=r, column=c, sticky="w", padx=2, pady=1)
            tk.Label(item_f, text="■", fg=col, bg=THEME["bg_card"], font=(FONT_FAMILY, 8, "bold")).pack(side="left")
            tk.Label(item_f, text=lbl_txt[2:], fg=THEME["text_secondary"], bg=THEME["bg_card"], font=FONT_SMALL).pack(side="left", padx=2)

        # Interactive Tools Card (Clean UI replacement for bulky inline log)
        tools_card = tk.Frame(sidebar, bg=THEME["bg_card"], padx=10, pady=8)
        tools_card.pack(fill="x", padx=8, pady=(4, 8))

        tk.Label(
            tools_card,
            text="ENGINE TOOLS",
            font=(FONT_FAMILY, 8, "bold"),
            bg=THEME["bg_card"],
            fg=THEME["text_secondary"]
        ).pack(anchor="w", pady=(0, 6))

        tools_btn_frame = tk.Frame(tools_card, bg=THEME["bg_card"])
        tools_btn_frame.pack(fill="x")

        # Floating DSA Execution Log
        self.btn_open_log = tk.Button(
            tools_btn_frame,
            text="📜 DSA Execution Log",
            font=FONT_BOLD,
            bg="#1E293B",
            fg="#38BDF8",
            activebackground="#334155",
            activeforeground="#7DD3FC",
            relief="flat",
            bd=0,
            padx=8,
            pady=7,
            cursor="hand2",
            command=self.open_dsa_log
        )
        self.btn_open_log.pack(fill="x")

    # --------------------------------------------------------------------------
    # BOTTOM STATUS BAR
    # --------------------------------------------------------------------------
    def create_status_bar(self):
        self.status_bar = tk.Frame(self.root, bg=THEME["bg_panel"], height=26)
        self.status_bar.pack(side="bottom", fill="x")

        self.status_msg_var = tk.StringVar(value="Tip: Right-click any node for options (Start / Goal / Obstacle / Delete). Drag node to move.")
        tk.Label(
            self.status_bar,
            textvariable=self.status_msg_var,
            font=FONT_SMALL,
            bg=THEME["bg_panel"],
            fg=THEME["text_secondary"],
            anchor="w",
            padx=12
        ).pack(side="left")

        self.node_edge_count_var = tk.StringVar(value="Vertices |V|: 0 | Edges |E|: 0 | Density: 0.0")
        tk.Label(
            self.status_bar,
            textvariable=self.node_edge_count_var,
            font=FONT_SMALL,
            bg=THEME["bg_panel"],
            fg=THEME["text_accent"],
            anchor="e",
            padx=12
        ).pack(side="right")

    # --------------------------------------------------------------------------
    # MODE SWITCHING & DSA INSPECTOR
    # --------------------------------------------------------------------------
    def set_mode(self, mode):
        self.current_mode = mode
        self.edge_start_node = None
        self.update_mode_button_styles()

        hints = {
            "ADD_NODE": "Mode: Click anywhere on the dark canvas to place a new Node.",
            "ADD_EDGE": "Mode: Click 1st Node, then click 2nd Node to connect them with a Road.",
            "SET_START": "Mode: Click any node to designate it as the START node (Green).",
            "SET_GOAL": "Mode: Click any node to designate it as the GOAL node (Red).",
            "TOGGLE_OBSTACLE": "Mode: Click any node to toggle it as an OBSTACLE (Blocked).",
            "MOVE_NODE": "Mode: Click and drag any node to reposition it smoothly.",
            "DELETE_NODE": "Mode: Click any node to delete it and its connected roads.",
        }
        self.canvas_hint_var.set(hints.get(mode, ""))

        cursors = {
            "ADD_NODE": "crosshair",
            "ADD_EDGE": "crosshair",
            "SET_START": "hand2",
            "SET_GOAL": "hand2",
            "TOGGLE_OBSTACLE": "pirate" if sys.platform != "win32" else "hand2",
            "MOVE_NODE": "fleur",
            "DELETE_NODE": "hand2",
        }
        self.canvas.config(cursor=cursors.get(mode, "arrow"))
        self.redraw()

    def update_mode_button_styles(self):
        for key, (btn, active_color) in self.mode_buttons.items():
            if key == self.current_mode:
                btn.config(bg=active_color, fg="#FFFFFF")
            else:
                btn.config(bg=THEME["btn_bg"], fg=THEME["btn_text"])

    def on_canvas_motion(self, event):
        if self.current_mode != "MOVE_NODE":
            if self.find_distance_badge_at(event.x, event.y) is not None:
                self.canvas.config(cursor="hand2")
            else:
                cursors = {
                    "ADD_NODE": "crosshair",
                    "ADD_EDGE": "crosshair",
                    "SET_START": "hand2",
                    "SET_GOAL": "hand2",
                    "TOGGLE_OBSTACLE": "pirate" if sys.platform != "win32" else "hand2",
                    "DELETE_NODE": "hand2",
                }
                self.canvas.config(cursor=cursors.get(self.current_mode, "arrow"))

    def on_speed_change(self, val):
        self.anim_speed_ms = int(val)
        self.speed_label_var.set(f"{self.anim_speed_ms} ms")

    def on_algo_change(self):
        algo = self.algo_var.get()
        complexities = {
            "Dijkstra's Algorithm": ("Data Structure: Min-Heap (heapq)", "Complexity: Time O((V+E) log V) | Space O(V)"),
            "A* Search Algorithm": ("Data Structure: Min-Heap (heapq)", "Complexity: Time O((V+E) log V) | Space O(V)"),
            "Greedy Best-First Search": ("Data Structure: Min-Heap (heapq)", "Complexity: Time O(V log V) | Space O(V)"),
            "Breadth-First Search (BFS)": ("Data Structure: FIFO Queue (deque)", "Complexity: Time O(V + E) | Space O(V)"),
        }
        ds_info, comp_info = complexities.get(algo, ("Data Structure: Queue", "Complexity: O(V+E)"))
        self.dsa_type_var.set(ds_info)
        self.dsa_complexity_var.set(comp_info)
        self.append_log(f"Switched Algorithm: {algo}")

    def open_dsa_inspector(self):
        DSAInspectorDialog(self.root, self.graph)

    def open_dsa_log(self):
        """Open or bring to focus the floating/hovering DSA Execution Log dialog."""
        if self.log_dialog is not None and self.log_dialog.winfo_exists():
            self.log_dialog.lift()
            self.log_dialog.focus_force()
        else:
            self.log_dialog = DSAExecutionLogDialog(
                self.root,
                initial_history=self.log_history,
                on_clear_cb=self.clear_log,
                on_close_cb=self.on_log_dialog_closed
            )

    def on_log_dialog_closed(self):
        self.log_dialog = None

    def append_log(self, text):
        self.log_history.append(text)
        if self.log_dialog is not None and self.log_dialog.winfo_exists():
            self.log_dialog.append_entry(text)

    def clear_log(self):
        self.log_history.clear()
        if self.log_dialog is not None and self.log_dialog.winfo_exists():
            self.log_dialog.clear_entries()

    # --------------------------------------------------------------------------
    # DRAFTS SYSTEM METHODS
    # --------------------------------------------------------------------------
    def save_draft(self):
        """Saves current graph/map into Drafts (persists up to 10 drafts)."""
        if not self.graph.nodes:
            messagebox.showwarning(
                "Empty Map",
                "Cannot save an empty map. Please add nodes to the canvas or load a preset first.",
                parent=self.root
            )
            return None

        draft = self.draft_manager.save_draft(self.graph)
        draft_name = draft.get("name", "Draft")
        v_count = len(self.graph.nodes)
        e_count = len(self.graph.edges)

        self.canvas_hint_var.set(f"Saved to Drafts: '{draft_name}' ({v_count} vertices, {e_count} roads).")
        self.append_log(f"[Drafts] Successfully saved '{draft_name}' ({v_count} vertices, {e_count} roads). Total stored: {len(self.draft_manager.get_drafts())}/10.")

        if hasattr(self, "drafts_dialog") and self.drafts_dialog and self.drafts_dialog.winfo_exists():
            self.drafts_dialog.refresh_drafts()

        messagebox.showinfo(
            "Draft Saved",
            f"Draft '{draft_name}' saved successfully!\n\nVertices: {v_count} | Roads: {e_count}\nStored in Drafts ({len(self.draft_manager.get_drafts())}/10).",
            parent=self.root
        )
        return draft

    def open_drafts_window(self):
        """Opens or brings to focus the Drafts window displaying saved maps."""
        if hasattr(self, "drafts_dialog") and self.drafts_dialog and self.drafts_dialog.winfo_exists():
            self.drafts_dialog.lift()
            self.drafts_dialog.focus_force()
        else:
            self.drafts_dialog = DraftsDialog(self.root, self.draft_manager, on_select_draft=self.load_draft)

    def load_draft(self, draft):
        """Loads a saved draft's graph into the main editor."""
        if not draft or "nodes" not in draft:
            return

        self.reset_search_path()
        self.graph.clear_all()

        for n_data in draft.get("nodes", []):
            node = Node(
                node_id=n_data["id"],
                label=n_data["label"],
                x=float(n_data["x"]),
                y=float(n_data["y"]),
                is_start=bool(n_data.get("is_start", False)),
                is_goal=bool(n_data.get("is_goal", False)),
                is_obstacle=bool(n_data.get("is_obstacle", False))
            )
            self.graph.nodes[node.id] = node

        for e_data in draft.get("edges", []):
            u_id = e_data["u_id"]
            v_id = e_data["v_id"]
            weight = e_data["weight"]
            if u_id in self.graph.nodes and v_id in self.graph.nodes:
                self.graph.add_edge(u_id, v_id, weight)

        draft_name = draft.get("name", "Saved Map")
        v_count = len(self.graph.nodes)
        e_count = len(self.graph.edges)

        self.clear_log()
        self.append_log(f"Loaded Draft: {draft_name} ({v_count} vertices, {e_count} roads).")
        self.canvas_hint_var.set(f"Loaded draft: {draft_name}. Ready to edit or run pathfinding.")
        self.redraw()

        if hasattr(self, "drafts_dialog") and self.drafts_dialog and self.drafts_dialog.winfo_exists():
            self.drafts_dialog.destroy()
            self.drafts_dialog = None

    # --------------------------------------------------------------------------
    # CANVAS INTERACTIONS
    # --------------------------------------------------------------------------
    def find_node_at(self, x, y):
        for node in reversed(list(self.graph.nodes.values())):
            dist = math.hypot(node.x - x, node.y - y)
            if dist <= node.radius + 4:
                return node
        return None

    def find_distance_badge_at(self, x, y):
        """Check if (x, y) falls directly on any midpoint distance number badge."""
        for (u_id, v_id), weight in self.graph.edges.items():
            if u_id not in self.graph.nodes or v_id not in self.graph.nodes:
                continue
            u = self.graph.nodes[u_id]
            v = self.graph.nodes[v_id]
            mx = (u.x + v.x) / 2
            my = (u.y + v.y) / 2
            weight_text = f"{weight:g}"
            bw = max(28, len(weight_text) * 8 + 14)
            bh = 22
            if abs(x - mx) <= (bw / 2 + 6) and abs(y - my) <= (bh / 2 + 6):
                return tuple(sorted([u_id, v_id]))
        return None

    def find_edge_at(self, x, y):
        """Find an edge near (x, y) - either by clicking its distance badge or road segment."""
        badge = self.find_distance_badge_at(x, y)
        if badge is not None:
            return badge

        # Check road line segment (proximity within 12px)
        for (u_id, v_id), weight in self.graph.edges.items():
            if u_id not in self.graph.nodes or v_id not in self.graph.nodes:
                continue
            u = self.graph.nodes[u_id]
            v = self.graph.nodes[v_id]
            dx = v.x - u.x
            dy = v.y - u.y
            length_sq = dx * dx + dy * dy
            if length_sq > 0:
                t = max(0.0, min(1.0, ((x - u.x) * dx + (y - u.y) * dy) / length_sq))
                px = u.x + t * dx
                py = u.y + t * dy
                dist = math.hypot(x - px, y - py)
                if dist <= 12.0:
                    return tuple(sorted([u_id, v_id]))
        return None

    def edit_road_length(self, edge_tuple):
        """Open modal to edit distance/weight of an existing edge."""
        u_id, v_id = edge_tuple
        key = tuple(sorted([u_id, v_id]))
        if key not in self.graph.edges:
            return
        curr_weight = self.graph.edges[key]
        u = self.graph.nodes[u_id]
        v = self.graph.nodes[v_id]

        dlg = DarkWeightDialog(self.root, u.label, v.label, default_weight=curr_weight, is_edit=True)
        if dlg.result is not None:
            self.graph.add_edge(u_id, v_id, dlg.result)
            self.append_log(f"[Road Edited] Updated road length between '{u.label}' and '{v.label}' to {dlg.result:g}")
            self.reset_search_path()
            self.redraw()

    def context_delete_edge(self, edge_tuple):
        u_id, v_id = edge_tuple
        self.graph.remove_edge(u_id, v_id)
        u_lbl = self.graph.nodes[u_id].label if u_id in self.graph.nodes else str(u_id)
        v_lbl = self.graph.nodes[v_id].label if v_id in self.graph.nodes else str(v_id)
        self.append_log(f"Deleted road between '{u_lbl}' and '{v_lbl}'.")
        self.reset_search_path()
        self.redraw()

    def on_canvas_left_click(self, event):
        x, y = event.x, event.y

        # Direct click on road distance number badge -> immediately edit its length in ANY mode!
        clicked_badge = self.find_distance_badge_at(x, y)
        if clicked_badge is not None:
            self.edit_road_length(clicked_badge)
            return

        clicked_node = self.find_node_at(x, y)
        clicked_edge = self.find_edge_at(x, y)

        if self.current_mode == "ADD_NODE":
            if clicked_node is None:
                new_node = self.graph.add_node(x, y)
                if len(self.graph.nodes) == 1:
                    new_node.is_start = True
                elif len(self.graph.nodes) == 2:
                    new_node.is_goal = True
                self.append_log(f"[Adjacency List] Added vertex '{new_node.label}' at ({int(x)}, {int(y)})")
                self.redraw()

        elif self.current_mode == "ADD_EDGE":
            if clicked_node is not None:
                if self.edge_start_node is None:
                    self.edge_start_node = clicked_node
                    self.canvas_hint_var.set(f"Selected {clicked_node.label}. Click 2nd Node to connect road.")
                    self.redraw()
                else:
                    first = self.edge_start_node
                    second = clicked_node
                    self.edge_start_node = None
                    self.canvas_hint_var.set("Mode: Click 1st Node, then click 2nd Node to connect road.")

                    if first.id == second.id:
                        messagebox.showwarning("Self Loop", "Self loops are disallowed.", parent=self.root)
                        self.redraw()
                        return

                    euc_dist = euclidean_distance(self.graph, first.id, second.id)
                    dlg = DarkWeightDialog(self.root, first.label, second.label, default_weight=euc_dist)
                    if dlg.result is not None:
                        self.graph.add_edge(first.id, second.id, dlg.result)
                        self.append_log(f"[Graph Edge] Added weighted road: ({first.label}, {second.label}, weight={dlg.result})")
                    self.redraw()
            else:
                if self.edge_start_node is not None:
                    self.edge_start_node = None
                    self.redraw()

        elif self.current_mode == "SET_START":
            if clicked_node is not None:
                if clicked_node.is_obstacle:
                    messagebox.showwarning("Obstacle", "Obstacle node cannot be Start.")
                    return
                for n in self.graph.nodes.values():
                    n.is_start = False
                clicked_node.is_start = True
                clicked_node.is_goal = False
                self.append_log(f"Set Start vertex to '{clicked_node.label}'.")
                self.redraw()

        elif self.current_mode == "SET_GOAL":
            if clicked_node is not None:
                if clicked_node.is_obstacle:
                    messagebox.showwarning("Obstacle", "Obstacle node cannot be Goal.")
                    return
                for n in self.graph.nodes.values():
                    n.is_goal = False
                clicked_node.is_goal = True
                clicked_node.is_start = False
                self.append_log(f"Set Goal vertex to '{clicked_node.label}'.")
                self.redraw()

        elif self.current_mode == "TOGGLE_OBSTACLE":
            if clicked_node is not None:
                if clicked_node.is_start or clicked_node.is_goal:
                    messagebox.showwarning("Protected", "Cannot turn Start or Goal into an obstacle.")
                    return
                clicked_node.is_obstacle = not clicked_node.is_obstacle
                state_str = "BLOCKED (Obstacle)" if clicked_node.is_obstacle else "UNBLOCKED"
                self.append_log(f"Vertex '{clicked_node.label}' is now {state_str}.")
                self.redraw()

        elif self.current_mode == "MOVE_NODE":
            if clicked_node is not None:
                self.dragged_node = clicked_node
                self.drag_offset_x = clicked_node.x - x
                self.drag_offset_y = clicked_node.y - y

        elif self.current_mode == "DELETE_NODE":
            if clicked_node is not None:
                self.delete_node(clicked_node)

    def on_canvas_drag(self, event):
        if self.current_mode == "MOVE_NODE" and self.dragged_node is not None:
            cw = self.canvas.winfo_width()
            ch = self.canvas.winfo_height()
            self.dragged_node.x = max(30, min(cw - 30, event.x + self.drag_offset_x))
            self.dragged_node.y = max(30, min(ch - 30, event.y + self.drag_offset_y))
            self.redraw()

    def on_canvas_release(self, event):
        if self.dragged_node is not None:
            self.dragged_node = None
            self.redraw()

    def on_canvas_right_click(self, event):
        x, y = event.x, event.y
        clicked_node = self.find_node_at(x, y)
        clicked_edge = self.find_edge_at(x, y)

        menu = tk.Menu(self.root, tearoff=0, bg=THEME["bg_card"], fg=THEME["text_primary"], activebackground="#2563EB", font=FONT_REGULAR)

        if clicked_node is not None:
            node = clicked_node
            menu.add_command(label=f"Vertex: {node.label}", state="disabled")
            menu.add_separator()
            menu.add_command(label="🟢 Set as Start", command=lambda: self.context_set_start(node))
            menu.add_command(label="🔴 Set as Goal", command=lambda: self.context_set_goal(node))
            obs_text = "✅ Unblock (Remove Obstacle)" if node.is_obstacle else "🚫 Block (Set as Obstacle)"
            menu.add_command(label=obs_text, command=lambda: self.context_toggle_obstacle(node))
            menu.add_separator()
            menu.add_command(label="❌ Delete Node", command=lambda: self.context_delete_node(node))
        elif clicked_edge is not None:
            u_id, v_id = clicked_edge
            u = self.graph.nodes[u_id]
            v = self.graph.nodes[v_id]
            w = self.graph.edges.get(clicked_edge, 0.0)
            menu.add_command(label=f"Road: {u.label} <--> {v.label} (Length={w:g})", state="disabled")
            menu.add_separator()
            menu.add_command(label="✏️ Edit Road Length", command=lambda e=clicked_edge: self.edit_road_length(e))
            menu.add_command(label="🗑️ Delete Road", command=lambda e=clicked_edge: self.context_delete_edge(e))
        else:
            menu.add_command(label="🎯 Center in Middle", command=self.center_graph)
            menu.add_command(label="➕ Add Node Here", command=lambda: self.context_add_node(x, y))
            menu.add_command(label="📜 Open DSA Log", command=self.open_dsa_log)
            menu.add_command(label="📊 View Adjacency List", command=self.open_dsa_inspector)
            menu.add_command(label="🔄 Reset Path", command=self.reset_search_path)

        menu.tk_popup(event.x_root, event.y_root)

    def context_set_start(self, node):
        if node.is_obstacle:
            messagebox.showwarning("Obstacle", "Obstacle node cannot be Start.")
            return
        for n in self.graph.nodes.values():
            n.is_start = False
        node.is_start = True
        node.is_goal = False
        self.redraw()

    def context_set_goal(self, node):
        if node.is_obstacle:
            messagebox.showwarning("Obstacle", "Obstacle node cannot be Goal.")
            return
        for n in self.graph.nodes.values():
            n.is_goal = False
        node.is_goal = True
        node.is_start = False
        self.redraw()

    def context_toggle_obstacle(self, node):
        if node.is_start or node.is_goal:
            messagebox.showwarning("Protected", "Cannot turn Start or Goal into obstacle.")
            return
        node.is_obstacle = not node.is_obstacle
        self.redraw()

    def delete_node(self, node):
        if self.edge_start_node == node:
            self.edge_start_node = None
        self.graph.remove_node(node.id)
        self.append_log(f"Deleted Vertex '{node.label}' (and connected roads).")
        self.reset_search_path()

    def context_delete_node(self, node):
        self.delete_node(node)

    def context_add_node(self, x, y):
        self.graph.add_node(x, y)
        self.redraw()

    # --------------------------------------------------------------------------
    # CANVAS RENDERING
    # --------------------------------------------------------------------------
    def redraw(self):
        self.canvas.delete("all")
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()

        # Grid dots
        grid_step = 40
        if cw > 50 and ch > 50:
            for gx in range(20, cw, grid_step):
                for gy in range(20, ch, grid_step):
                    self.canvas.create_oval(
                        gx - 1, gy - 1, gx + 1, gy + 1,
                        fill=THEME["canvas_grid"],
                        outline=""
                    )

        # Edges
        for (u_id, v_id), weight in self.graph.edges.items():
            if u_id not in self.graph.nodes or v_id not in self.graph.nodes:
                continue
            u = self.graph.nodes[u_id]
            v = self.graph.nodes[v_id]

            edge_tuple = tuple(sorted([u_id, v_id]))
            is_shortest_path = edge_tuple in self.final_path_edges
            is_examining = (self.examining_edge == edge_tuple)
            is_searched = (edge_tuple in self.searched_edges)

            if is_shortest_path:
                # Glowing warm amber/gold halo behind final shortest path
                self.canvas.create_line(
                    u.x, u.y, v.x, v.y,
                    fill="#78350F",
                    width=10,
                    capstyle="round"
                )
                line_color = THEME["edge_path"]
                line_width = 5
            elif is_examining:
                # Glowing outer beam for actively searched road
                self.canvas.create_line(
                    u.x, u.y, v.x, v.y,
                    fill="#064E3B",
                    width=8,
                    capstyle="round"
                )
                line_color = THEME["edge_examining"]
                line_width = 4
            elif is_searched:
                # Road taken in exploration tree
                line_color = THEME["edge_searched"]
                line_width = 3
            else:
                line_color = THEME["edge_default"]
                line_width = 2

            self.canvas.create_line(
                u.x, u.y, v.x, v.y,
                fill=line_color,
                width=line_width,
                capstyle="round"
            )

            # Midpoint Pill Badge
            mx = (u.x + v.x) / 2
            my = (u.y + v.y) / 2
            weight_text = f"{weight:g}"

            bw = max(24, len(weight_text) * 8 + 10)
            bh = 18

            if is_shortest_path:
                badge_bg = "#451A03"
                badge_outline = "#FACC15"
                badge_text_col = "#FEF08A"
            elif is_examining:
                badge_bg = "#064E3B"
                badge_outline = "#22C55E"
                badge_text_col = "#6EE7B7"
            elif is_searched:
                badge_bg = "#064E3B"
                badge_outline = "#059669"
                badge_text_col = "#A7F3D0"
            else:
                badge_bg = THEME["edge_badge_bg"]
                badge_outline = THEME["border"]
                badge_text_col = THEME["edge_badge_text"]

            self.canvas.create_rectangle(
                mx - bw / 2, my - bh / 2, mx + bw / 2, my + bh / 2,
                fill=badge_bg,
                outline=badge_outline,
                width=1
            )
            self.canvas.create_text(
                mx, my,
                text=weight_text,
                fill=badge_text_col,
                font=FONT_SMALL
            )

            # Active pulsing beacon on the road currently being examined
            if is_examining:
                self.canvas.create_oval(
                    mx - 5, my - 5, mx + 5, my + 5,
                    fill="#FFFFFF",
                    outline="#22C55E",
                    width=2
                )

        # Selection ring for edge creation
        if self.current_mode == "ADD_EDGE" and self.edge_start_node is not None:
            sn = self.edge_start_node
            r = sn.radius + 6
            self.canvas.create_oval(
                sn.x - r, sn.y - r, sn.x + r, sn.y + r,
                outline="#F59E0B",
                width=2,
                dash=(4, 4)
            )

        # Nodes
        for node in self.graph.nodes.values():
            self.draw_node(node)

        # Update status bar with graph theory metrics
        density = self.graph.get_graph_density()
        self.node_edge_count_var.set(
            f"Vertices |V|: {len(self.graph.nodes)} | Edges |E|: {len(self.graph.edges)} | Density: {density}"
        )

    def draw_node(self, node):
        x, y = node.x, node.y
        r = node.radius

        is_path = node.id in self.final_path
        is_active = (node.id == self.active_node)
        is_frontier = (node.id in self.frontier_nodes)
        is_visited = (node.id in self.visited_nodes)

        if node.is_obstacle:
            fill_col = THEME["node_obstacle_fill"]
            stroke_col = THEME["node_obstacle_stroke"]
            glow_col = None
        elif node.is_start:
            fill_col = THEME["node_start_fill"]
            stroke_col = THEME["node_start_stroke"]
            glow_col = "#10B981"
        elif node.is_goal:
            fill_col = THEME["node_goal_fill"]
            stroke_col = THEME["node_goal_stroke"]
            glow_col = "#EF4444"
        else:
            fill_col = THEME["node_default_fill"]
            stroke_col = THEME["node_default_stroke"]
            glow_col = None

        if is_path:
            if not (node.is_start or node.is_goal):
                fill_col = THEME["node_path_fill"]
                stroke_col = THEME["node_path_stroke"]
            glow_col = "#FACC15"
        elif is_active:
            fill_col = THEME["node_active_fill"]
            stroke_col = THEME["node_active_stroke"]
            glow_col = "#06B6D4"
        elif is_visited and not (node.is_start or node.is_goal):
            fill_col = THEME["node_visited_fill"]
            stroke_col = THEME["node_visited_stroke"]
        elif is_frontier and not (node.is_start or node.is_goal):
            fill_col = THEME["node_frontier_fill"]
            stroke_col = THEME["node_frontier_stroke"]
            glow_col = "#F59E0B"

        if glow_col is not None:
            glow_r = r + 5
            self.canvas.create_oval(
                x - glow_r, y - glow_r, x + glow_r, y + glow_r,
                outline=glow_col,
                width=2
            )

        self.canvas.create_oval(
            x - r, y - r, x + r, y + r,
            fill=fill_col,
            outline=stroke_col,
            width=3
        )

        self.canvas.create_text(
            x, y,
            text=node.label,
            fill=THEME["text_primary"],
            font=FONT_NODE
        )

        if node.is_start:
            self.canvas.create_text(
                x, y - r - 10,
                text="START",
                fill="#10B981",
                font=(FONT_FAMILY, 8, "bold")
            )
        elif node.is_goal:
            self.canvas.create_text(
                x, y - r - 10,
                text="GOAL",
                fill="#EF4444",
                font=(FONT_FAMILY, 8, "bold")
            )
        elif node.is_obstacle:
            d = r * 0.55
            self.canvas.create_line(
                x - d, y + d, x + d, y - d,
                fill=THEME["node_obstacle_cross"],
                width=3
            )
            self.canvas.create_text(
                x, y + r + 10,
                text="BLOCKED",
                fill="#94A3B8",
                font=FONT_SMALL
            )

    # --------------------------------------------------------------------------
    # EXECUTION ENGINE
    # --------------------------------------------------------------------------
    def start_visualization(self):
        start_node = self.graph.get_start_node()
        goal_node = self.graph.get_goal_node()

        if start_node is None or goal_node is None:
            messagebox.showwarning(
                "Missing Start or Goal",
                "Please designate both a Start (Green) and Goal (Red) vertex.\n\n"
                "Tip: Click 'Set Start' or 'Set Goal' in the top bar, then click any node.",
                parent=self.root
            )
            return

        if start_node.id == goal_node.id:
            messagebox.showinfo("Start equals Goal", "Start and Goal are the same node! Distance is 0.0.", parent=self.root)
            return

        if start_node.is_obstacle or goal_node.is_obstacle:
            messagebox.showerror("Obstacle Alert", "Start or Goal cannot be an obstacle.", parent=self.root)
            return

        self.reset_search_visuals()

        algo_name = self.algo_var.get()
        if "A*" in algo_name:
            self.anim_generator = astar_step_generator(self.graph, start_node.id, goal_node.id)
        elif "Greedy" in algo_name:
            self.anim_generator = greedy_bfs_step_generator(self.graph, start_node.id, goal_node.id)
        elif "Breadth" in algo_name or "BFS" in algo_name:
            self.anim_generator = bfs_step_generator(self.graph, start_node.id, goal_node.id)
        else:
            self.anim_generator = dijkstra_step_generator(self.graph, start_node.id, goal_node.id)

        self.anim_running = True
        self.anim_paused = False
        self.btn_run.config(state="disabled")
        self.btn_pause.config(state="normal")
        self.update_pause_buttons(paused=False)
        self.set_status_badge("SEARCHING...", "#78350F", "#FBBF24")

        self.schedule_next_step()

    def schedule_next_step(self):
        if self.timer_id is not None:
            self.root.after_cancel(self.timer_id)
            self.timer_id = None

        if self.anim_running and not self.anim_paused:
            self.timer_id = self.root.after(self.anim_speed_ms, self.step_execution_loop)

    def step_execution_loop(self):
        if not self.anim_running or self.anim_paused:
            return
        has_more = self.step_execution()
        if has_more and self.anim_running and not self.anim_paused:
            self.schedule_next_step()

    def step_execution(self):
        if self.anim_generator is None:
            start_node = self.graph.get_start_node()
            goal_node = self.graph.get_goal_node()
            if start_node is None or goal_node is None:
                messagebox.showwarning("Missing Start/Goal", "Please set Start and Goal nodes first.", parent=self.root)
                return False
            self.reset_search_visuals()
            algo_name = self.algo_var.get()
            if "A*" in algo_name:
                self.anim_generator = astar_step_generator(self.graph, start_node.id, goal_node.id)
            elif "Greedy" in algo_name:
                self.anim_generator = greedy_bfs_step_generator(self.graph, start_node.id, goal_node.id)
            elif "Breadth" in algo_name or "BFS" in algo_name:
                self.anim_generator = bfs_step_generator(self.graph, start_node.id, goal_node.id)
            else:
                self.anim_generator = dijkstra_step_generator(self.graph, start_node.id, goal_node.id)

            self.anim_running = True
            self.anim_paused = True
            self.btn_pause.config(state="normal")
            self.update_pause_buttons(paused=True)
            self.set_status_badge("STEPPING (PAUSED)", "#1E293B", "#38BDF8")

        try:
            event_type, data, log_msg = next(self.anim_generator)
        except StopIteration:
            self.finish_animation(found=False)
            return False

        if log_msg:
            self.append_log(log_msg)

        if event_type == "FRONTIER_ADD":
            node_id, g_val, f_val, snap = data
            self.frontier_nodes.add(node_id)
            self.val_frontier_var.set(str(len(self.frontier_nodes)))
            self.queue_snapshot_var.set(f"Queue: {snap}")

        elif event_type == "VISIT_NODE":
            node_id, current_dist, snap = data
            self.active_node = node_id
            if node_id in self.frontier_nodes:
                self.frontier_nodes.remove(node_id)
            self.visited_nodes.add(node_id)
            self.val_explored_var.set(str(len(self.visited_nodes)))
            self.val_frontier_var.set(str(len(self.frontier_nodes)))
            self.queue_snapshot_var.set(f"Queue: {snap}")

        elif event_type == "EXAMINE_EDGE":
            u, v, is_valid = data
            self.examining_edge = tuple(sorted([u, v]))
            if is_valid:
                self.searched_edges.add(self.examining_edge)

        elif event_type == "CLOSE_NODE":
            node_id = data
            if self.active_node == node_id:
                self.active_node = None
            self.examining_edge = None

        elif event_type == "PATH_FOUND":
            self.finish_animation(found=True, result_data=data)
            return False

        elif event_type == "NO_PATH":
            self.finish_animation(found=False, result_data=data)
            return False

        self.metric_steps += 1
        self.val_steps_var.set(str(self.metric_steps))
        self.redraw()
        self.canvas.update_idletasks()
        return True

    def finish_animation(self, found=True, result_data=None):
        self.anim_running = False
        self.anim_paused = False
        self.active_node = None
        self.examining_edge = None
        self.btn_run.config(state="normal")
        self.btn_pause.config(state="disabled")
        self.update_pause_buttons(paused=False)

        if found and result_data:
            path = result_data["path"]
            total_dist = result_data["total_dist"]
            explored = result_data["explored_count"]
            steps = result_data["total_steps"]

            self.final_path = path
            self.final_path_edges = set()
            for i in range(len(path) - 1):
                self.final_path_edges.add(tuple(sorted([path[i], path[i + 1]])))

            self.metric_dist = total_dist
            self.metric_explored = explored
            self.metric_steps = steps
            self.metric_path_seq = path

            self.val_dist_var.set(f"{total_dist:.1f}")
            self.val_explored_var.set(f"{explored} / {len(self.graph.nodes)}")
            self.val_steps_var.set(str(steps))

            # Point-wise details
            details = []
            for i in range(len(path) - 1):
                u = path[i]
                v = path[i+1]
                edge_weight = self.graph.edges.get(tuple(sorted([u, v])), 0.0)
                details.append(f"{self.graph.nodes[u].label} ({edge_weight:g})")
            details.append(f"{self.graph.nodes[path[-1]].label}")
            
            detail_str = " ➔ ".join(details)
            self.val_sequence_var.set(detail_str)
            self.append_log(f"[SUCCESS] Shortest Path: {detail_str} | Total Cost: {total_dist:.1f}")

            self.set_status_badge("PATH FOUND! 🎉", "#064E3B", "#34D399")
            self.btn_simulate.config(state="normal")
        else:
            self.set_status_badge("NO PATH EXISTS", "#7F1D1D", "#F87171")
            self.val_dist_var.set("Infinity")
            self.val_sequence_var.set("Unreachable")
            self.append_log("[FAILED] Goal is unreachable from Start.")
            self.btn_simulate.config(state="disabled")
            # Do not show messagebox in endless mode to avoid blocking
            if not self.endless_var.get():
                messagebox.showinfo("Search Finished", "No path exists between Start and Goal nodes!", parent=self.root)

        self.redraw()
        self.canvas.update_idletasks()
        
        if self.endless_var.get():
            self.endless_timer_id = self.root.after(1500, self.auto_restart)

    def auto_restart(self):
        if not self.anim_paused and not self.simulation_active and len(self.graph.nodes) > 0:
            self.start_visualization()
            
    def start_route_simulation(self):
        if not self.final_path: return
        self.btn_simulate.config(state="disabled")
        if hasattr(self, "endless_timer_id") and self.endless_timer_id:
            self.root.after_cancel(self.endless_timer_id)
            self.endless_timer_id = None
            
        self.simulation_active = True
        self.sim_dot = self.canvas.create_oval(0, 0, 0, 0, fill="#FBBF24", outline="#FFFFFF", width=2)
        self.append_log("[Simulation] Starting moving dot traversal along the route...")
        self.animate_dot(0, 0.0)

    def animate_dot(self, segment_idx, progress):
        if not self.simulation_active or segment_idx >= len(self.final_path) - 1:
            if self.simulation_active:
                self.append_log("[Simulation] Traversal Complete.")
                self.canvas.delete(self.sim_dot)
            self.simulation_active = False
            self.btn_simulate.config(state="normal")
            if self.endless_var.get():
                self.endless_timer_id = self.root.after(1500, self.auto_restart)
            return

        u = self.graph.nodes[self.final_path[segment_idx]]
        v = self.graph.nodes[self.final_path[segment_idx + 1]]
        
        x = u.x + (v.x - u.x) * progress
        y = u.y + (v.y - u.y) * progress
        r = 8
        self.canvas.coords(self.sim_dot, x-r, y-r, x+r, y+r)
        
        if progress < 1.0:
            self.root.after(30, lambda: self.animate_dot(segment_idx, min(1.0, progress + 0.1)))
        else:
            self.canvas.create_line(u.x, u.y, v.x, v.y, fill="#FACC15", width=4, dash=(4,4), capstyle="round")
            self.root.after(50, lambda: self.animate_dot(segment_idx + 1, 0.0))

    def update_pause_buttons(self, paused=True):
        if paused:
            txt = "▶ Resume"
            fg = "#34D399"
            bg = "#064E3B"
        else:
            txt = "⏸ Pause"
            fg = "#FBBF24"
            bg = "#1F2937"
        if hasattr(self, "btn_sidebar_pause"):
            self.btn_sidebar_pause.config(text=txt, fg=fg, bg=bg)
        if hasattr(self, "btn_pause"):
            self.btn_pause.config(text=txt)

    def toggle_pause(self):
        if not self.anim_running:
            self.step_execution()
            return
        self.anim_paused = not self.anim_paused
        if self.anim_paused:
            if self.timer_id is not None:
                self.root.after_cancel(self.timer_id)
                self.timer_id = None
            self.update_pause_buttons(paused=True)
            self.set_status_badge("PAUSED", "#1F2937", "#FBBF24")
            self.append_log("Visualization Paused. Click Resume, Step, or hit Spacebar to continue.")
        else:
            self.update_pause_buttons(paused=False)
            self.set_status_badge("SEARCHING...", "#78350F", "#FBBF24")
            self.append_log("Visualization Resumed.")
            self.schedule_next_step()

    def reset_search_visuals(self):
        self.frontier_nodes.clear()
        self.visited_nodes.clear()
        self.active_node = None
        self.examining_edge = None
        self.searched_edges.clear()
        self.final_path.clear()
        self.final_path_edges.clear()
        self.metric_dist = 0.0
        self.metric_explored = 0
        self.metric_steps = 0
        self.metric_path_seq = []
        self.val_dist_var.set("--")
        self.val_explored_var.set("0")
        self.val_steps_var.set("0")
        self.val_frontier_var.set("0")
        self.val_sequence_var.set("None")
        self.queue_snapshot_var.set("Queue: [ Empty ]")

    def reset_search_path(self):
        if self.timer_id is not None:
            self.root.after_cancel(self.timer_id)
            self.timer_id = None
        if self.endless_timer_id is not None:
            self.root.after_cancel(self.endless_timer_id)
            self.endless_timer_id = None
        self.anim_running = False
        self.anim_paused = False
        self.simulation_active = False
        self.anim_generator = None
        self.btn_run.config(state="normal")
        self.btn_pause.config(state="disabled")
        self.update_pause_buttons(paused=False)
        self.btn_simulate.config(state="disabled")
        self.reset_search_visuals()
        self.set_status_badge("READY TO RUN", "#064E3B", "#34D399")
        self.append_log("Reset search paths and highlights.")
        self.redraw()

    def clear_all_canvas(self):
        if messagebox.askyesno("Clear All", "Are you sure you want to clear the entire graph?", parent=self.root):
            self.reset_search_path()
            self.graph.clear_all()
            self.clear_log()
            self.append_log("Canvas cleared. Ready for new graph creation.")
            self.redraw()

    def on_status_badge_click(self):
        if not self.anim_running:
            self.start_visualization()
        else:
            self.toggle_pause()

    def on_status_badge_enter(self, event=None):
        if not self.anim_running:
            self.status_badge.config(bg="#059669")

    def on_status_badge_leave(self, event=None):
        bg = getattr(self, "current_badge_bg", "#064E3B")
        self.status_badge.config(bg=bg)

    def set_status_badge(self, text, bg_col, fg_col):
        self.current_badge_bg = bg_col
        self.current_badge_fg = fg_col
        self.status_badge_var.set(text)
        self.status_badge.config(bg=bg_col, fg=fg_col)

    def on_canvas_configure(self, event=None):
        if not self.has_initial_centered and self.canvas.winfo_width() > 100 and self.canvas.winfo_height() > 100:
            self.has_initial_centered = True
            self.center_graph()
        else:
            self.redraw()

    def center_graph(self):
        """Dynamically center the entire graph pattern in the middle of the canvas.
        Preserves all relative Euclidean distances, edge weights, and topological structure.
        """
        if not self.graph.nodes:
            return

        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()

        # If canvas has not been mapped/rendered yet by Tkinter, re-try shortly
        if cw <= 100 or ch <= 100:
            self.root.after(40, self.center_graph)
            return

        nodes = list(self.graph.nodes.values())
        min_x = min(n.x for n in nodes)
        max_x = max(n.x for n in nodes)
        min_y = min(n.y for n in nodes)
        max_y = max(n.y for n in nodes)

        graph_w = max_x - min_x
        graph_h = max_y - min_y

        curr_cx = (min_x + max_x) / 2.0
        curr_cy = (min_y + max_y) / 2.0

        target_cx = cw / 2.0
        target_cy = ch / 2.0

        # Safety scaling if graph exceeds available canvas bounds
        avail_w = max(100.0, cw - 80.0)
        avail_h = max(100.0, ch - 80.0)
        scale = 1.0
        if graph_w > avail_w or graph_h > avail_h:
            scale_w = avail_w / graph_w if graph_w > 0 else 1.0
            scale_h = avail_h / graph_h if graph_h > 0 else 1.0
            scale = min(scale_w, scale_h)

        if scale < 1.0:
            for node in nodes:
                node.x = curr_cx + (node.x - curr_cx) * scale
                node.y = curr_cy + (node.y - curr_cy) * scale

        dx = target_cx - curr_cx
        dy = target_cy - curr_cy

        if abs(dx) > 1.0 or abs(dy) > 1.0 or scale < 1.0:
            for node in nodes:
                node.x += dx
                node.y += dy
            self.redraw()
            self.canvas_hint_var.set(f"Graph pattern centered in middle ({int(cw)}x{int(ch)}).")

    # --------------------------------------------------------------------------
    # PRESETS
    # --------------------------------------------------------------------------
    def load_preset_simple(self):
        self.reset_search_path()
        self.graph.clear_all()

        n_a = self.graph.add_node(100, 240, label="A", is_start=True)
        n_b = self.graph.add_node(240, 140, label="B")
        n_c = self.graph.add_node(240, 340, label="C")
        n_d = self.graph.add_node(420, 140, label="D")
        n_e = self.graph.add_node(420, 340, label="E")
        n_f = self.graph.add_node(560, 240, label="F", is_goal=True)

        self.graph.add_edge(n_a.id, n_b.id, 8.0)
        self.graph.add_edge(n_a.id, n_c.id, 5.0)
        self.graph.add_edge(n_b.id, n_d.id, 6.0)
        self.graph.add_edge(n_c.id, n_e.id, 4.0)
        self.graph.add_edge(n_b.id, n_c.id, 2.0)
        self.graph.add_edge(n_d.id, n_e.id, 3.0)
        self.graph.add_edge(n_d.id, n_f.id, 7.0)
        self.graph.add_edge(n_e.id, n_f.id, 6.0)

        self.clear_log()
        self.append_log("Loaded Preset 1: Simple Road Network (6 vertices, 8 edges).")
        self.redraw()
        self.center_graph()

    def load_preset_diamond(self):
        self.reset_search_path()
        self.graph.clear_all()

        n_s = self.graph.add_node(80, 250, label="Start", is_start=True)
        n_t1 = self.graph.add_node(220, 120, label="T1")
        n_t2 = self.graph.add_node(380, 120, label="T2")
        n_b1 = self.graph.add_node(220, 380, label="B1")
        n_b2 = self.graph.add_node(380, 380, label="B2")
        n_mid = self.graph.add_node(300, 250, label="Center")
        n_g = self.graph.add_node(520, 250, label="Goal", is_goal=True)

        self.graph.add_edge(n_s.id, n_t1.id, 15.0)
        self.graph.add_edge(n_t1.id, n_t2.id, 14.0)
        self.graph.add_edge(n_t2.id, n_g.id, 16.0)

        self.graph.add_edge(n_s.id, n_b1.id, 6.0)
        self.graph.add_edge(n_b1.id, n_mid.id, 4.0)
        self.graph.add_edge(n_mid.id, n_b2.id, 5.0)
        self.graph.add_edge(n_b2.id, n_g.id, 6.0)

        self.graph.add_edge(n_t1.id, n_mid.id, 8.0)
        self.graph.add_edge(n_mid.id, n_t2.id, 9.0)

        self.clear_log()
        self.append_log("Loaded Preset 2: Diamond & Detours.")
        self.redraw()
        self.center_graph()

    def load_preset_obstacle_wall(self):
        self.reset_search_path()
        self.graph.clear_all()

        n_s = self.graph.add_node(90, 250, label="Start", is_start=True)
        n_g = self.graph.add_node(580, 250, label="Goal", is_goal=True)

        obs1 = self.graph.add_node(340, 190, label="Obs1", is_obstacle=True)
        obs2 = self.graph.add_node(340, 250, label="Obs2", is_obstacle=True)
        obs3 = self.graph.add_node(340, 310, label="Obs3", is_obstacle=True)

        top1 = self.graph.add_node(230, 100, label="North1")
        top2 = self.graph.add_node(440, 100, label="North2")
        bot1 = self.graph.add_node(230, 400, label="South1")
        bot2 = self.graph.add_node(440, 400, label="South2")

        self.graph.add_edge(n_s.id, top1.id, 12.0)
        self.graph.add_edge(top1.id, top2.id, 10.0)
        self.graph.add_edge(top2.id, n_g.id, 11.0)

        self.graph.add_edge(n_s.id, bot1.id, 14.0)
        self.graph.add_edge(bot1.id, bot2.id, 13.0)
        self.graph.add_edge(bot2.id, n_g.id, 15.0)

        self.graph.add_edge(n_s.id, obs2.id, 5.0)
        self.graph.add_edge(obs2.id, n_g.id, 5.0)

        self.clear_log()
        self.append_log("Loaded Preset 3: Obstacle Maze Barrier.")
        self.redraw()
        self.center_graph()

    def load_preset_grid(self):
        self.reset_search_path()
        self.graph.clear_all()

        grid = {}
        cols, rows = 4, 3
        start_x, start_y = 100, 140
        dx, dy = 140, 120

        for r in range(rows):
            for c in range(cols):
                label = chr(65 + (r * cols + c))
                x = start_x + c * dx
                y = start_y + r * dy
                is_start = (r == 0 and c == 0)
                is_goal = (r == rows - 1 and c == cols - 1)
                is_obs = (r == 1 and c == 1)
                n = self.graph.add_node(x, y, label=label, is_start=is_start, is_goal=is_goal, is_obstacle=is_obs)
                grid[(r, c)] = n

        for r in range(rows):
            for c in range(cols):
                curr = grid[(r, c)]
                if c + 1 < cols:
                    right = grid[(r, c + 1)]
                    self.graph.add_edge(curr.id, right.id, 10.0)
                if r + 1 < rows:
                    down = grid[(r + 1, c)]
                    self.graph.add_edge(curr.id, down.id, 10.0)

        self.clear_log()
        self.append_log("Loaded Preset 4: Grid Road Map (Great for Dijkstra vs A* vs BFS).")
        self.redraw()
        self.center_graph()


# ==============================================================================
# MAIN ENTRY POINT
# ==============================================================================
def main():
    root = tk.Tk()
    app = PathfindingVisualizerApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
