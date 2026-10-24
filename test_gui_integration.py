"""
GUI & Logic Integration Test (DSA Edition)
Validates end-to-end functionality of PathfindingVisualizerApp in pure Python:
- Preset loading
- Dijkstra execution & path reconstruction
- A* execution & path reconstruction
- Greedy Best-First Search execution
- Breadth-First Search (BFS) execution
- Depth-First Search (DFS) execution
- Obstacle bypass verification
- Unreachable node handling
- DSA Inspector dialog initialization
"""

import os
import tempfile
import tkinter as tk
import unittest
from unittest.mock import patch, MagicMock
from app import (
    PathfindingVisualizerApp,
    DSAInspectorDialog,
    DSAExecutionLogDialog,
    DarkWeightDialog,
    DraftManager,
    DraftsDialog
)


class TestVisualizerIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = tk.Tk()
        cls.root.withdraw()
        cls.patcher_info = patch("tkinter.messagebox.showinfo")
        cls.patcher_warn = patch("tkinter.messagebox.showwarning")
        cls.patcher_err = patch("tkinter.messagebox.showerror")
        cls.patcher_info.start()
        cls.patcher_warn.start()
        cls.patcher_err.start()
        cls.app = PathfindingVisualizerApp(cls.root)

    @classmethod
    def tearDownClass(cls):
        cls.patcher_info.stop()
        cls.patcher_warn.stop()
        cls.patcher_err.stop()
        cls.root.destroy()

    def test_preset_simple_dijkstra_completion(self):
        self.app.load_preset_simple()
        self.app.algo_var.set("Dijkstra's Algorithm")
        self.app.start_visualization()

        steps = 0
        while self.app.anim_running and steps < 200:
            self.app.step_execution()
            steps += 1

        self.assertFalse(self.app.anim_running)
        self.assertTrue(len(self.app.final_path) > 0)
        expected_labels = ["A", "C", "E", "F"]
        actual_labels = [self.app.graph.nodes[n].label for n in self.app.final_path]
        self.assertEqual(actual_labels, expected_labels)
        self.assertAlmostEqual(self.app.metric_dist, 15.0, delta=0.1)

    def test_preset_simple_astar_completion(self):
        self.app.load_preset_simple()
        self.app.algo_var.set("A* Search Algorithm")
        self.app.start_visualization()

        steps = 0
        while self.app.anim_running and steps < 200:
            self.app.step_execution()
            steps += 1

        self.assertFalse(self.app.anim_running)
        self.assertTrue(len(self.app.final_path) > 0)
        expected_labels = ["A", "C", "E", "F"]
        actual_labels = [self.app.graph.nodes[n].label for n in self.app.final_path]
        self.assertEqual(actual_labels, expected_labels)
        self.assertAlmostEqual(self.app.metric_dist, 15.0, delta=0.1)

    def test_preset_simple_greedy_completion(self):
        self.app.load_preset_simple()
        self.app.algo_var.set("Greedy Best-First Search")
        self.app.start_visualization()

        steps = 0
        while self.app.anim_running and steps < 200:
            self.app.step_execution()
            steps += 1

        self.assertFalse(self.app.anim_running)
        self.assertTrue(len(self.app.final_path) > 0)
        self.assertEqual(self.app.final_path[0], self.app.graph.get_start_node().id)
        self.assertEqual(self.app.final_path[-1], self.app.graph.get_goal_node().id)

    def test_preset_simple_bfs_completion(self):
        self.app.load_preset_simple()
        self.app.algo_var.set("Breadth-First Search (BFS)")
        self.app.start_visualization()

        steps = 0
        while self.app.anim_running and steps < 200:
            self.app.step_execution()
            steps += 1

        self.assertFalse(self.app.anim_running)
        self.assertTrue(len(self.app.final_path) > 0)
        self.assertEqual(self.app.final_path[0], self.app.graph.get_start_node().id)
        self.assertEqual(self.app.final_path[-1], self.app.graph.get_goal_node().id)

    def test_preset_simple_dfs_completion(self):
        self.app.load_preset_simple()
        self.app.algo_var.set("Depth-First Search (DFS)")
        self.app.start_visualization()

        steps = 0
        while self.app.anim_running and steps < 200:
            self.app.step_execution()
            steps += 1

        self.assertFalse(self.app.anim_running)
        self.assertTrue(len(self.app.final_path) > 0)
        self.assertEqual(self.app.final_path[0], self.app.graph.get_start_node().id)
        self.assertEqual(self.app.final_path[-1], self.app.graph.get_goal_node().id)
        self.assertIn("Stack", self.app.queue_title_var.get())

    def test_obstacle_wall_bypassed(self):
        self.app.load_preset_obstacle_wall()
        self.app.algo_var.set("Dijkstra's Algorithm")
        self.app.start_visualization()

        steps = 0
        while self.app.anim_running and steps < 300:
            self.app.step_execution()
            steps += 1

        self.assertFalse(self.app.anim_running)
        self.assertTrue(len(self.app.final_path) > 0)
        for n_id in self.app.final_path:
            self.assertFalse(self.app.graph.nodes[n_id].is_obstacle)

    def test_unreachable_node_handling(self):
        self.app.load_preset_simple()
        goal = self.app.graph.get_goal_node()
        self.app.graph.remove_node(goal.id)
        self.app.graph.add_node(700, 300, label="IsolatedGoal", is_goal=True)

        self.app.algo_var.set("Dijkstra's Algorithm")
        self.app.start_visualization()

        steps = 0
        while self.app.anim_running and steps < 200:
            self.app.step_execution()
            steps += 1

        self.assertFalse(self.app.anim_running)
        self.assertEqual(len(self.app.final_path), 0)
        self.assertEqual(self.app.val_dist_var.get(), "Infinity")

    def test_dsa_inspector_dialog(self):
        self.app.load_preset_simple()
        dlg = DSAInspectorDialog(self.root, self.app.graph)
        self.assertIsNotNone(dlg)
        dlg.destroy()

    def test_delete_node_button_and_mode(self):
        # Verify button exists in mode_buttons
        self.assertIn("DELETE_NODE", self.app.mode_buttons)
        btn, accent_col = self.app.mode_buttons["DELETE_NODE"]
        self.assertIn("Delete Node", btn.cget("text"))

        # Switch to DELETE_NODE mode
        self.app.set_mode("DELETE_NODE")
        self.assertEqual(self.app.current_mode, "DELETE_NODE")

        # Load preset and pick a node to delete
        self.app.load_preset_simple()
        initial_node_count = len(self.app.graph.nodes)
        initial_edge_count = len(self.app.graph.edges)
        node_b = next(n for n in self.app.graph.nodes.values() if n.label == "B")
        incident_edges = [e for e in self.app.graph.edges if node_b.id in e]
        self.assertTrue(len(incident_edges) > 0)

        # Simulate left-click event at node B's coordinates
        class FakeEvent:
            x = int(node_b.x)
            y = int(node_b.y)

        self.app.on_canvas_left_click(FakeEvent())

        # Verify node B was deleted along with its edges
        self.assertEqual(len(self.app.graph.nodes), initial_node_count - 1)
        self.assertNotIn(node_b.id, self.app.graph.nodes)
        for e in incident_edges:
            self.assertNotIn(e, self.app.graph.edges)
        self.assertEqual(len(self.app.graph.edges), initial_edge_count - len(incident_edges))

    def test_ready_to_run_button_and_green_road_animation(self):
        self.app.load_preset_simple()
        self.app.reset_search_path()

        # Check badge is a button with READY TO RUN text
        self.assertIn("READY TO RUN", self.app.status_badge_var.get())
        self.assertFalse(self.app.anim_running)

        # Click / invoke the READY TO RUN button
        self.app.status_badge.invoke()

        # Verify animation process has started
        self.assertTrue(self.app.anim_running)
        self.assertEqual(self.app.status_badge_var.get(), "SEARCHING...")

        # Step through animation and verify roads being searched are collected in searched_edges
        steps = 0
        while self.app.anim_running and steps < 200:
            self.app.step_execution()
            steps += 1

        # Search should find the path and complete
        self.assertFalse(self.app.anim_running)
        self.assertTrue(len(self.app.final_path) > 0)
        self.assertTrue(len(self.app.searched_edges) > 0)
        # Verify all searched edges belong to the graph
        for edge in self.app.searched_edges:
            self.assertIn(edge, self.app.graph.edges)

    def test_sidebar_pause_button_and_resume(self):
        self.app.load_preset_simple()
        self.app.reset_search_path()

        self.assertTrue(hasattr(self.app, "btn_sidebar_pause"))
        self.assertTrue(hasattr(self.app, "btn_sidebar_step"))

        # Start visualization
        self.app.start_visualization()
        self.assertTrue(self.app.anim_running)
        self.assertFalse(self.app.anim_paused)

        # Click Pause button in sidebar
        self.app.btn_sidebar_pause.invoke()
        self.assertTrue(self.app.anim_paused)
        self.assertEqual(self.app.status_badge_var.get(), "PAUSED")
        self.assertIn("Resume", self.app.btn_sidebar_pause.cget("text"))

        # Step forward while paused
        prev_steps = self.app.metric_steps
        self.app.btn_sidebar_step.invoke()
        self.assertEqual(self.app.metric_steps, prev_steps + 1)
        self.assertTrue(self.app.anim_paused)

        # Resume execution
        self.app.btn_sidebar_pause.invoke()
        self.assertFalse(self.app.anim_paused)
        self.assertEqual(self.app.status_badge_var.get(), "SEARCHING...")
        self.assertIn("Pause", self.app.btn_sidebar_pause.cget("text"))

        # Clean up
        self.app.reset_search_path()

    def test_floating_dsa_log_dialog(self):
        self.app.load_preset_simple()

        # Check buttons exist in sidebar and toolbar
        self.assertTrue(hasattr(self.app, "btn_open_log"))
        self.assertTrue(hasattr(self.app, "btn_header_log"))
        self.assertIn("DSA Execution Log", self.app.btn_open_log.cget("text"))

        # Add a log entry to history
        test_msg = "DSA Test Log Entry for Floating Box"
        self.app.append_log(test_msg)
        self.assertIn(test_msg, self.app.log_history)

        # Open the floating dialog
        self.app.open_dsa_log()
        self.assertIsNotNone(self.app.log_dialog)
        self.assertIsInstance(self.app.log_dialog, DSAExecutionLogDialog)

        # Verify initial history is loaded into text area
        log_content = self.app.log_dialog.text_area.get("1.0", tk.END)
        self.assertIn(test_msg, log_content)

        # Stream a new real-time log while dialog is open
        stream_msg = "Real-time Priority Queue Step"
        self.app.append_log(stream_msg)
        log_content = self.app.log_dialog.text_area.get("1.0", tk.END)
        self.assertIn(stream_msg, log_content)

        # Test clear log
        self.app.clear_log()
        self.assertEqual(len(self.app.log_history), 0)
        cleared_content = self.app.log_dialog.text_area.get("1.0", tk.END).strip()
        self.assertEqual(cleared_content, "")

        # Close dialog
        self.app.log_dialog.on_close()
        self.assertIsNone(self.app.log_dialog)

    def test_edit_road_length_by_direct_badge_click(self):
        self.app.load_preset_simple()

        # Verify Road Length mode button was removed from mode_buttons and sidebar
        self.assertNotIn("EDIT_EDGE", self.app.mode_buttons)
        self.assertFalse(hasattr(self.app, "btn_edit_road"))

        # Select an edge to edit (A <-> C)
        node_a = next(n for n in self.app.graph.nodes.values() if n.label == "A")
        node_c = next(n for n in self.app.graph.nodes.values() if n.label == "C")
        edge_key = tuple(sorted([node_a.id, node_c.id]))
        self.assertIn(edge_key, self.app.graph.edges)
        original_weight = self.app.graph.edges[edge_key]

        # Calculate midpoint badge position
        mid_x = int((node_a.x + node_c.x) / 2)
        mid_y = int((node_a.y + node_c.y) / 2)

        # Verify find_distance_badge_at detects edge by midpoint badge
        found_edge = self.app.find_distance_badge_at(mid_x, mid_y)
        self.assertEqual(found_edge, edge_key)

        # Mock DarkWeightDialog to simulate updating the road weight to 42.5
        with patch("app.DarkWeightDialog") as MockDialog:
            instance = MagicMock()
            instance.result = 42.5
            MockDialog.return_value = instance

            class FakeClickEvent:
                x = mid_x
                y = mid_y

            # Direct left-click on the road distance badge in normal ADD_NODE mode
            self.app.on_canvas_left_click(FakeClickEvent())

            # Verify DarkWeightDialog was invoked directly with is_edit=True
            MockDialog.assert_called_once()
            _, kwargs = MockDialog.call_args
            self.assertTrue(kwargs.get("is_edit", False))

        # Verify the weight in graph.edges was successfully updated
        self.assertEqual(self.app.graph.edges[edge_key], 42.5)
        self.assertNotEqual(self.app.graph.edges[edge_key], original_weight)

    def test_center_graph_method_and_button(self):
        # 1. Verify Center button exists and has correct text
        self.assertTrue(hasattr(self.app, "btn_center"))
        self.assertIn("Center", self.app.btn_center.cget("text"))

        # 2. Simulate canvas geometry
        self.app.canvas.winfo_width = MagicMock(return_value=900)
        self.app.canvas.winfo_height = MagicMock(return_value=700)

        # 3. Load preset 4 (Grid) and center it
        self.app.load_preset_grid()
        self.app.center_graph()

        # Calculate bounding box
        nodes = list(self.app.graph.nodes.values())
        min_x = min(n.x for n in nodes)
        max_x = max(n.x for n in nodes)
        min_y = min(n.y for n in nodes)
        max_y = max(n.y for n in nodes)

        cx = (min_x + max_x) / 2.0
        cy = (min_y + max_y) / 2.0

        # Center should be at (450, 350)
        self.assertAlmostEqual(cx, 450.0, delta=1.5)
        self.assertAlmostEqual(cy, 350.0, delta=1.5)

        # Margins should be equal
        left_margin = min_x
        right_margin = 900 - max_x
        top_margin = min_y
        bottom_margin = 700 - max_y

        self.assertAlmostEqual(left_margin, right_margin, delta=1.5)
        self.assertAlmostEqual(top_margin, bottom_margin, delta=1.5)

        # 4. Test moving nodes off-center and clicking the Center button
        for n in nodes:
            n.x -= 200
            n.y += 300

        self.app.btn_center.invoke()

        new_min_x = min(n.x for n in nodes)
        new_max_x = max(n.x for n in nodes)
        new_min_y = min(n.y for n in nodes)
        new_max_y = max(n.y for n in nodes)
        new_cx = (new_min_x + new_max_x) / 2.0
        new_cy = (new_min_y + new_max_y) / 2.0

        self.assertAlmostEqual(new_cx, 450.0, delta=1.5)
        self.assertAlmostEqual(new_cy, 350.0, delta=1.5)

    def test_draft_buttons_existence_and_placement(self):
        # 1. Verify Drafts button exists and has correct text
        self.assertTrue(hasattr(self.app, "btn_drafts"))
        self.assertIn("Drafts", self.app.btn_drafts.cget("text"))

        # 2. Verify Save Draft button exists and has correct text
        self.assertTrue(hasattr(self.app, "btn_save_draft"))
        self.assertIn("Save Draft", self.app.btn_save_draft.cget("text"))

    def test_save_draft_empty_graph(self):
        # Empty graph should not save a draft
        self.app.graph.clear_all()
        result = self.app.save_draft()
        self.assertIsNone(result)

    def test_save_and_load_draft_workflow(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            temp_mgr = DraftManager(drafts_dir=tmp_dir)
            self.app.draft_manager = temp_mgr

            # 1. Load preset simple and customize it
            self.app.load_preset_simple()
            original_node_labels = sorted(n.label for n in self.app.graph.nodes.values())
            original_edge_count = len(self.app.graph.edges)
            start_node = self.app.graph.get_start_node()
            goal_node = self.app.graph.get_goal_node()

            # 2. Save draft
            saved_draft = self.app.save_draft()
            self.assertIsNotNone(saved_draft)
            self.assertEqual(len(temp_mgr.get_drafts()), 1)
            self.assertEqual(saved_draft["node_count"], len(original_node_labels))
            self.assertEqual(saved_draft["edge_count"], original_edge_count)
            self.assertEqual(saved_draft["start_label"], start_node.label)
            self.assertEqual(saved_draft["goal_label"], goal_node.label)

            # 3. Verify image file exists
            img_path = saved_draft.get("image_path")
            self.assertTrue(os.path.exists(img_path))

            # 4. Clear graph completely
            self.app.graph.clear_all()
            self.assertEqual(len(self.app.graph.nodes), 0)

            # 5. Load draft back
            self.app.load_draft(saved_draft)
            restored_labels = sorted(n.label for n in self.app.graph.nodes.values())
            self.assertEqual(restored_labels, original_node_labels)
            self.assertEqual(len(self.app.graph.edges), original_edge_count)
            self.assertEqual(self.app.graph.get_start_node().label, start_node.label)
            self.assertEqual(self.app.graph.get_goal_node().label, goal_node.label)

    def test_drafts_max_10_limit_and_rotation(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            temp_mgr = DraftManager(drafts_dir=tmp_dir)
            self.app.draft_manager = temp_mgr
            self.app.load_preset_simple()

            # Save 12 drafts
            for _ in range(12):
                self.app.save_draft()

            drafts = temp_mgr.get_drafts()
            self.assertEqual(len(drafts), 10)
            # Ensure the latest draft is at index 0
            self.assertEqual(drafts[0]["name"], "Draft 12")

    def test_drafts_persistence_across_restarts(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            mgr1 = DraftManager(drafts_dir=tmp_dir)
            self.app.draft_manager = mgr1
            self.app.load_preset_diamond()
            draft1 = self.app.save_draft()

            self.app.load_preset_obstacle_wall()
            draft2 = self.app.save_draft()

            self.assertEqual(len(mgr1.get_drafts()), 2)

            # Simulate app restart with a fresh DraftManager pointing to the same folder
            mgr_restarted = DraftManager(drafts_dir=tmp_dir)
            reloaded_drafts = mgr_restarted.get_drafts()
            self.assertEqual(len(reloaded_drafts), 2)
            self.assertEqual(reloaded_drafts[0]["id"], draft2["id"])
            self.assertEqual(reloaded_drafts[1]["id"], draft1["id"])

    def test_drafts_window_open_and_interaction(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            temp_mgr = DraftManager(drafts_dir=tmp_dir)
            self.app.draft_manager = temp_mgr
            self.app.load_preset_simple()
            self.app.save_draft()

            # Open drafts window
            self.app.open_drafts_window()
            self.assertIsNotNone(self.app.drafts_dialog)
            self.assertTrue(self.app.drafts_dialog.winfo_exists())

            # Simulate selecting the draft from the dialog
            draft = temp_mgr.get_drafts()[0]
            self.app.drafts_dialog.select_draft(draft)
            self.assertIsNone(self.app.drafts_dialog)

    def test_preset_binary_tree_structure(self):
        self.app.load_preset_binary_tree()
        self.assertEqual(len(self.app.graph.nodes), 15)
        self.assertEqual(len(self.app.graph.edges), 14)
        start = self.app.graph.get_start_node()
        goal = self.app.graph.get_goal_node()
        self.assertIsNotNone(start)
        self.assertIsNotNone(goal)
        self.assertEqual(start.label, "A")
        self.assertEqual(goal.label, "O")

    def test_preset_binary_tree_bfs_completion(self):
        self.app.load_preset_binary_tree()
        self.app.algo_var.set("Breadth-First Search (BFS)")
        self.app.start_visualization()

        steps = 0
        while self.app.anim_running and steps < 250:
            self.app.step_execution()
            steps += 1

        self.assertFalse(self.app.anim_running)
        self.assertTrue(len(self.app.final_path) > 0)
        start = self.app.graph.get_start_node()
        goal = self.app.graph.get_goal_node()
        self.assertEqual(self.app.final_path[0], start.id)
        self.assertEqual(self.app.final_path[-1], goal.id)
        path_labels = [self.app.graph.nodes[n].label for n in self.app.final_path]
        self.assertEqual(path_labels, ["A", "C", "G", "O"])

    def test_bfs_no_extra_routes_after_goal(self):
        # On Preset 1, node D discovers goal F. Node E should never be expanded.
        self.app.load_preset_simple()
        self.app.algo_var.set("Breadth-First Search (BFS)")
        self.app.start_visualization()

        steps = 0
        while self.app.anim_running and steps < 200:
            self.app.step_execution()
            steps += 1

        self.assertFalse(self.app.anim_running)
        # Verify node E is not in visited nodes because goal was reached before expanding E
        e_nodes = [nid for nid, n in self.app.graph.nodes.items() if n.label == "E"]
        self.assertTrue(len(e_nodes) == 1)
        self.assertNotIn(e_nodes[0], self.app.visited_nodes)


if __name__ == "__main__":
    unittest.main()

