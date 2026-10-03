"""Algorithm regression tests without requiring a running ROS graph."""
import ast
import json
import math
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1] / '源码（国赛）/src'


def methods(relative, class_name, names):
    tree = ast.parse((ROOT / relative).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    ns = dict(math=math, json=json, String=lambda **kw: NS(**kw))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(relative), 'exec'), ns)
    return ns[class_name]


def path(*points):
    return NS(poses=[NS(pose=NS(position=NS(x=x, y=y))) for x, y in points])


class TimingTests(unittest.TestCase):
    def test_alignment_cost_prefers_facing_approach(self):
        C = methods('main_controller/main_controller/main_controller_node.py',
                    'MainControllerNode', {'_endpoint_turn_cost'})
        p = path((0, 0), (.02, .01), (.5, 0), (1, 0))
        self.assertAlmostEqual(C._endpoint_turn_cost(p, 0, 0), 0)
        self.assertAlmostEqual(C._endpoint_turn_cost(p, 0, math.pi), math.pi)
        self.assertAlmostEqual(C._endpoint_turn_cost(path((0, 0), (-1, 0)),
                                                    -math.pi, math.pi), 0)
        self.assertEqual(C._endpoint_turn_cost(path((0, 0)), 0, 0), 0)

    def navigator(self):
        C = methods('nav_simple/nav_simple/simple_navigator.py', 'SimpleNav2Navigator',
                    {'obstacle_callback', '_predicted_obstacle_y', '_crossing_wait'})
        node = C()
        node.now = 1.0
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(node.now * 1e9)))
        node.obstacle_samples = deque(maxlen=50)
        node.get_logger = lambda: NS(info=lambda *_: None, warn=lambda *_: None)
        node.current_target_cube = 'red_cube_3'
        node.crossing_max_wait = 12.0
        return node

    def test_paused_clock_and_reset(self):
        node = self.navigator()
        node.obstacle_callback(NS(position=NS(y=0.0)))
        node.obstacle_callback(NS(position=NS(y=0.1)))
        self.assertEqual(len(node.obstacle_samples), 1)
        node.now = 0.1
        node.obstacle_callback(NS(position=NS(y=1.0)))
        self.assertEqual(list(node.obstacle_samples), [(0.1, 1.0)])
        node.now = .3
        node.obstacle_callback(NS(position=NS(y=float('nan'))))
        self.assertEqual(len(node.obstacle_samples), 1)

    def test_distant_crossing_starts_without_pickup_wait(self):
        node = self.navigator()
        node.now = 2.0
        node.obstacle_samples.extend([(1.5, 1.66), (2., 1.6)])
        goal = NS(pose=NS(pose=NS(position=NS(x=2.593086, y=-5.727858))))
        distant = path((-8.32, .8), (-2.5, .8), (0, .8))
        self.assertEqual(node._crossing_wait(goal, distant), 0.0)
        near = path((-3.5, .8), (-2.5, .8), (0, .8))
        node.crossing_max_wait = 24.0
        self.assertGreater(node._crossing_wait(goal, near), 0.0)
        node.now = 4.0
        self.assertEqual(node._crossing_wait(goal, near), 0.0)

    def test_near_crossing_can_wait_until_obstacle_clears(self):
        node = self.navigator()
        node.crossing_max_wait = 24.0
        node.now = 2.0
        node.obstacle_samples.extend([(1.5, 2.37), (2.0, 2.31)])
        goal = NS(pose=NS(pose=NS(position=NS(x=2.59, y=-5.73))))
        wait = node._crossing_wait(goal, path((-3.5, 1.52), (-2.5, 1.52), (1.18, 1.52)))
        self.assertGreaterEqual(wait, 18.0)
        self.assertLessEqual(wait, 20.0)
        node.crossing_max_wait = 12.0
        self.assertEqual(node._crossing_wait(goal, path((-3.5, 1.52), (-2.5, 1.52), (1.18, 1.52))), 0.0)

    def test_slow_moving_obstacle_still_predicted(self):
        node = self.navigator()
        node.now = 2.0
        node.obstacle_samples.extend([(1.5, 0.24), (2., 0.30)])
        goal = NS(pose=NS(pose=NS(position=NS(x=2.593086, y=-5.727858))))
        # Slow but measurable motion must not be replaced with a guessed direction.
        calls = []
        node._predicted_obstacle_y = lambda y, v, t: calls.append(v) or 3.0
        node._crossing_wait(goal, path((-3.5, .8), (-2.5, .8)))
        self.assertTrue(calls)
        self.assertAlmostEqual(calls[0], 0.12)

    def test_arm_failure_published_after_reset(self):
        C = methods('arm_action_integration/arm_action_integration/arm_grab_place_node.py',
                    'ArmGrabPlaceNode', {'_reset_execution', '_detach_done_cb'})
        node = C()
        node.current_task = 'grab'
        node.is_executing = True
        node.get_logger = lambda: NS(info=lambda *_: None, error=lambda *_: None)
        messages = []
        node.arm_status_pub = NS(publish=lambda msg: messages.append((msg.data, node.is_executing)))
        node._reset_execution()
        self.assertEqual(messages, [('grasp_failed', False)])
        node.current_task = 'place'
        node.detach_attempts = 3
        node._detach_done_cb(NS(result=lambda: NS(success=False, message='failure')), lambda: None)
        self.assertEqual(messages[-1], ('place_failed', False))

    def test_release_rechecks_and_cancels_on_new_goal(self):
        C = methods('nav_simple/nav_simple/simple_navigator.py', 'SimpleNav2Navigator',
                    {'plan_result_callback'})
        # Method uses GoalStatus as a global, supplied without a ROS dependency.
        C.plan_result_callback.__globals__['GoalStatus'] = NS(STATUS_SUCCEEDED=4)
        node = C()
        node.request_id = 1
        node.crossing_max_wait = 12.0
        node.emergency_stop_active = False
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=0))
        node.get_logger = lambda: NS(error=lambda *_: None)
        waits = iter([4., 0.])
        node._crossing_wait = lambda *_: next(waits)
        callbacks, started, canceled = [], [], []
        node.create_timer = lambda period, callback: callbacks.append(callback)
        node._cancel_pending_start = lambda: canceled.append(True)
        node._start_navigation = lambda *args: started.append(args)
        response = NS(status=4, result=NS(path=path((0, 0), (1, 0))))
        node.plan_result_callback(NS(result=lambda: response), 'goal', 1)
        callbacks[0]()
        self.assertEqual(started, [('goal', 1)])
        node.request_id = 2
        callbacks[0]()
        self.assertEqual(len(started), 1)
        self.assertEqual(len(canceled), 2)


if __name__ == '__main__':
    unittest.main()
