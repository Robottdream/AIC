"""Exercise static estimates and full-plan fallback without a running ROS graph."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/main_controller'))
from main_controller.static_routes import StaticRouteCosts
from test_time_optimization import methods, path


class StaticCostsTests(unittest.TestCase):
    def test_wall_detour_and_disconnected_targets(self):
        cells = np.zeros((30, 30), dtype=np.int8)
        cells[:23, 15] = 100
        costs = StaticRouteCosts()
        costs.update(30, 30, .2, (0., 0.), cells.ravel())
        distance = costs.costs_from((2., 2.), [(4., 2.)])[0]
        self.assertGreater(distance, 5.)  # Euclidean sorting would miss the wall.
        cells[:, 15] = 100
        costs.update(30, 30, .2, (0., 0.), cells.ravel())
        self.assertEqual(costs.costs_from((2., 2.), [(4., 2.)]), [math.inf])

    def test_cache_invalidates_on_content_and_geometry_not_republication(self):
        cells = np.zeros((20, 20), dtype=np.int8)
        costs = StaticRouteCosts()
        self.assertTrue(costs.update(20, 20, .2, (0., 0.), cells.ravel()))
        targets = [(2., 2.), (3., 2.)]
        before = costs.costs_from((1., 1.), targets)
        self.assertFalse(costs.update(20, 20, .2, (0., 0.), cells.ravel()))
        self.assertEqual(before, costs.costs_from((1., 1.), targets))
        self.assertEqual(costs.field_builds, 1)
        costs.costs_from((2., 1.), targets, cache=False)
        self.assertEqual(len(costs.fields), 1)
        cells[:, 8] = -1
        self.assertTrue(costs.update(20, 20, .2, (0., 0.), cells.ravel()))
        self.assertEqual(len(costs.fields), 0)
        self.assertEqual(costs.costs_from((1., 1.), targets), [math.inf, math.inf])
        self.assertTrue(costs.update(20, 20, .2, (1., 0.), cells.ravel()))
        self.assertEqual(len(costs.fields), 0)

    def test_boundaries_invalid_points_and_diagonal_corner(self):
        costs = StaticRouteCosts()
        self.assertIsNone(costs.costs_from((0, 0), [(1, 1)]))
        cells = np.zeros((4, 4), dtype=np.int8)
        cells[1, 2] = cells[2, 1] = 100
        costs.update(4, 4, 1., (0., 0.), cells.ravel())
        a, b = 1 * 4 + 1, 2 * 4 + 2
        self.assertEqual(costs.graph[a, b], 0)  # no diagonal through touching corners
        for point in ((-.01, 1.), (4., 1.), (math.nan, 1.), (2.1, 1.1)):
            self.assertEqual(costs.costs_from(point, [(1., 1.)]), [math.inf])


def controller():
    names = {'_plan_next_candidate', '_rank_static_candidates', '_choose_route_with_lookahead',
             '_prefer_initial_west_red', '_remaining_same_area_tasks', '_approach_penalty',
             'get_grasp_candidates', '_plan_result', '_path_length', '_endpoint_turn_cost'}
    C = methods('main_controller/main_controller/main_controller_node.py', 'MainControllerNode', names)
    C._plan_next_candidate.__globals__['ComputePathToPose'] = NS(Goal=NS)
    C._plan_result.__globals__['GoalStatus'] = NS(STATUS_SUCCEEDED=4)
    node = C()
    node.selection_id = 1
    node.current_step = 'SELECT_BLOCK'
    node.current_task = dict(color='blue', to='B')
    node.current_assignment = dict(area_pos=(0., 0.))
    node.optimized_tasks = []
    node.current_robot_pose = (0., 0.)
    node.current_robot_yaw = 0.
    node.turn_cost_weight = .2
    node.stable_layout_hints = False
    node.GRASP_OFFSET = .55
    node.selection_best = None
    node.selection_options = []
    node.selection_static_future = None
    node.selection_checked = node.selection_requests = 0
    node.selection_started = 0.
    node._selection_request_started = node._selection_request_accepted = 0.
    node.selection_fast = True
    node.selection_batch_size = node.selection_batch_end = 2
    node.selection_candidates = [((float(i), 1., i), 0, (float(i), 1., 0.)) for i in range(5)]
    node.get_logger = lambda: NS(info=lambda *_: None)
    node._pose_stamped = lambda *pose: pose
    node.requests = []
    node.plan_client = NS(send_goal_async=lambda req: node.requests.append(req) or
                          NS(add_done_callback=lambda *_: None))
    node.navigate_to_block = lambda: setattr(node, 'navigated', True)
    node._stop_failed_task = lambda reason: setattr(node, 'failed', reason)
    return node


class SelectionTests(unittest.TestCase):
    def test_outbound_success_requires_a_separate_return_success_before_selection(self):
        node = controller()
        node._active_candidate = ((2., 1., 2), 0, (2., 1., 0.))
        advanced = []
        node._plan_next_candidate = lambda sid: advanced.append(sid)
        result = NS(result=lambda: NS(status=4, result=NS(path=path((0., 0.), (2., 1.)))))
        node._plan_result(result, 1, False)
        self.assertIsNone(node.selection_best)
        self.assertEqual(advanced, [])
        self.assertTrue(node.requests[-1].use_start)
        self.assertEqual(node.requests[-1].start, (2., 1., 0.))
        node._plan_result(NS(result=lambda: NS(status=6, result=NS(path=path()))), 1, True)
        self.assertIsNone(node.selection_best)
        self.assertEqual(advanced, [1])
        node._plan_result(result, 1, True)
        self.assertIsNotNone(node.selection_best)
        self.assertEqual(advanced, [1, 1])

    def test_failed_first_batch_continues_and_success_stops(self):
        node = controller()
        for _ in range(3):
            node._plan_next_candidate(1)  # simulate three failed candidates
        self.assertEqual(node.selection_checked, 3)
        self.assertEqual(node.selection_batch_end, 4)
        block, direction, _ = node._active_candidate
        node.selection_best = (4., block, direction)
        node.selection_options = [(4., 2., block, direction)]
        node._plan_next_candidate(1)  # second candidate in the fallback batch
        node._plan_next_candidate(1)
        self.assertTrue(node.navigated)
        self.assertEqual(node.selection_checked, 4)
        self.assertEqual(len(node.selection_candidates), 1)

    def test_all_failures_exhaust_every_candidate_and_stale_callback_is_ignored(self):
        node = controller()
        node._plan_next_candidate(0)
        self.assertEqual(node.requests, [])
        for _ in range(6):
            node._plan_next_candidate(1)
        self.assertEqual(node.selection_checked, 5)
        self.assertTrue(node.failed)

    def test_no_map_falls_back_and_infinite_estimates_do_not_drop_candidates(self):
        node = controller()
        node.static_routes = StaticRouteCosts()
        self.assertFalse(node._rank_static_candidates())
        costs = [math.inf, 3., 4., 5., 6.]
        node.static_routes = NS(costs_from=lambda *args, **kwargs: costs)
        self.assertTrue(node._rank_static_candidates())
        self.assertEqual(len(node.selection_candidates), 5)
        self.assertEqual(node.selection_candidates[-1][0][2], 0)

    def test_initial_west_red_preference_and_task_order_preserved(self):
        node = controller()
        node.current_task = dict(color='red', to='A', original_num=3, current_index=1)
        node.optimized_tasks = [dict(task=dict(color='red', to='A')),
                                dict(task=dict(color='blue', to='B'))]
        node.static_routes = NS(costs_from=lambda *args, **kwargs: [1., 2., 3., 20., 4.])
        before = list(node.optimized_tasks)
        node._rank_static_candidates()
        self.assertEqual(node.selection_candidates[0][0][2], 3)
        self.assertEqual(node.optimized_tasks, before)
        self.assertEqual(node._remaining_same_area_tasks(), 1)


if __name__ == '__main__':
    unittest.main()
