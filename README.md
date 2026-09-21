# Interactive Graph Pathfinding Visualizer (DSA Edition)

A single-page, fully working desktop application built **100% purely with Python 3 and Tkinter**. No HTML, CSS, JavaScript, web servers, or external browser dependencies.

---

## DSA Concepts & Core Theoretical Framework

This application is built from the ground up to demonstrate fundamental **Data Structures & Algorithms (DSA)** in computer science:

### 1. Graph Data Structure
- **Adjacency List Representation**:
  ```python
  adj = {
      'A': [('B', 8.0), ('C', 5.0)],
      'B': [('A', 8.0), ('D', 6.0), ('C', 2.0)],
      ...
  }
  ```
- **Graph Metrics**:
  - Vertex Count $|V|$ and Edge Count $|E|$.
  - **Graph Density**: $D = \frac{2|E|}{|V|(|V|-1)}$ for undirected graphs.
  - **Average Vertex Degree**: $\bar{d} = \frac{2|E|}{|V|}$.

### 2. Priority Queue (Min-Heap)
- Implemented using Python's native `heapq` module.
- Supports efficient $O(\log |V|)$ extraction of the minimum distance vertex and $O(\log |V|)$ key relaxation.
- **Live Min-Heap Monitor**: Real-time side panel display showing current elements inside the priority queue array during search execution.

### 3. Pathfinding Algorithms Implemented

| Algorithm | Data Structure | Formula / Evaluation | Time Complexity | Space Complexity | Guarantees Shortest Path? |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Dijkstra's Algorithm** | Min-Heap (`heapq`) | $d[v] = \min(d[v], d[u] + w(u,v))$ | $O((V + E) \log V)$ | $O(V)$ | **Yes** (Non-negative weights) |
| **A\* Search Algorithm** | Min-Heap (`heapq`) | $f(n) = g(n) + h(n)$ (Euclidean) | $O((V + E) \log V)$ | $O(V)$ | **Yes** (Admissible heuristic) |
| **Greedy Best-First** | Min-Heap (`heapq`) | $f(n) = h(n)$ | $O(V \log V)$ | $O(V)$ | No (Heuristic trade-off) |
| **Breadth-First Search** | FIFO Queue (`deque`) | Level-Order Queue traversal | $O(V + E)$ | $O(V)$ | **Yes** (Unweighted / minimum hops) |

---

## Interactive Features & GUI Controls

- **Canvas Modes**:
  - ➕ **Add Node**: Click on canvas to spawn vertices auto-labeled `A, B, C... Z, AA...`.
  - 🔗 **Add Road**: Click 1st node then 2nd node to draw an edge. A dark popup modal prompts for custom edge distance (pre-fills with geometric Euclidean distance).
  - ✏️ **Direct Road Length Editing**: Simply click directly on any road distance number badge to edit its length/weight via a sleek dialog modal (hovering shows hand cursor).
  - 🟢 **Set Start Node**: Mark any vertex as Start (Emerald Green).
  - 🔴 **Set Goal Node**: Mark any vertex as Goal (Crimson Red).
  - 🚫 **Toggle Obstacle**: Block vertices (Dark Slate with barrier slash); algorithms will route around them.
  - ✋ **Drag-and-Drop Repositioning**: Click and drag vertices; connecting edges and midpoint weight badges recalculate smoothly in real time.
  - 🗑️ **Delete Node**: Click any vertex to remove it and its connected roads from the graph.
  - 🎯 **Center in Middle**: Click `🎯 Center` on the canvas toolbar (or in the Presets menu / right-click canvas menu) to instantly and symmetrically align the entire pattern in the dead center of the screen with equal margins on all four sides. Automatically centers when any preset is loaded.
  - 🖱 **Right-Click Context Menu**: Instant actions on vertices (*Set Start*, *Set Goal*, *Toggle Obstacle*, *Delete Vertex*), roads (*Edit Road Length*, *Delete Road*), and empty canvas (*Center in Middle*, *Add Node*, *Open DSA Log*).
- **Interactive Engine Tools**:
  - 📜 **Floating DSA Execution Log**: Dedicated floating/hovering window (`tk.Toplevel`) opened from the sidebar. Streams execution events, queue snapshots, and mathematical relaxation equations in real time with entry count badges and clear/close controls.
- **DSA Inspector (`📊 DSA Inspector`)**:
  - One-click modal displaying the exact Python Adjacency List dictionary, $|V|$, $|E|$, density, and average degree.
- **Execution Controls**:
  - `Ready to Run Button / Run (▶)`: Starts step-by-step visual animation in real time (click the prominent **READY TO RUN** engine status button or the top **▶ RUN** button).
  - `Real-Time Search & Yellow Path Highlighting`: As the algorithm searches, roads being actively examined and taken illuminate in vibrant green (`#22C55E` / `#10B981`). Once the goal is reached, the final correct path is highlighted in brilliant **electric yellow** (`#FACC15`) with an amber glowing halo for maximum contrast against the green explored roads.
  - `Pause (⏸) / Resume`: Pause search execution at any step in real time via the dedicated **`⏸ Pause`** button under Engine Status, the top header bar, or by pressing **Spacebar**. Click again to resume.
  - `Step (⏭)`: Manually step forward by 1 algorithm event (queue pop, relaxation, or neighbor check) directly from the sidebar or header.
  - `Reset (🔄)`: Clears search highlights while preserving graph structure.
  - `Clear (🗑)`: Resets canvas to empty.
  - `Presets (🎲)`: Pre-configured sample graphs (*Simple Road*, *Diamond Detours*, *Obstacle Maze*, *Grid Network*).
  - `Speed Slider`: Native slider from 10 ms (rapid) to 500 ms (detailed inspection).
- **Telemetry Dashboard**:
  - Shortest Path Distance, Nodes Explored count & percentage, Algorithm Steps count.
  - Path Sequence display (e.g. `A ➔ C ➔ E ➔ F`).
  - Live Priority Queue Snapshot.
  - Theoretical Complexity indicators.

---

## Requirements

- **Python 3.8+** (Standard installation with Tkinter included).
- **Zero external packages**: No `pip install` required.

---

## How to Run

Launch the desktop visualizer:

```powershell
python app.py
```

Run the automated test suites:

```powershell
python test_algorithms.py
python test_gui_integration.py
```
