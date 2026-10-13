import unittest
from types import SimpleNamespace as NS
from test_time_optimization import methods

class TimedArrivalTests(unittest.TestCase):

    def navigator(self, measured):
        C = methods('nav_simple/nav_simple/simple_navigator.py', 'SimpleNav2Navigator', {'result_callback'})
        C.result_callback.__globals__['GoalStatus'] = NS(STATUS_SUCCEEDED=4, STATUS_ABORTED=6, STATUS_CANCELED=5)
        n = C()
        n.request_id = 1
        n.safe_route = []
        n.current_goal_handle = None
        n.world_pose = measured
        n.goal_pose = NS(pose=NS(position=NS(x=3.45, y=-11.0)))
        n.get_clock = lambda : NS(now=lambda : NS(nanoseconds=2000000000))
        n.near_goal_started = None
        n.get_logger = lambda : NS(error=lambda _: None, info=lambda _: None)
        messages = []
        n.status_pub = NS(publish=lambda m: messages.append(m.data))
        return (n, messages)

    def test_false_success_and_stale_position_cannot_trigger_grasp(self):
        for (measured, expected) in [((2.0, 0.0, 0.0), 'failed: arrival_position_mismatch'), ((1.0, 3.45, -11.0), 'failed: arrival_pose_unavailable')]:
            (n, messages) = self.navigator(measured)
            n.result_callback(NS(result=lambda : NS(status=4)), 1)
            self.assertEqual(messages, [expected])

    def test_actual_fresh_arrival_reports_success(self):
        (n, messages) = self.navigator((2.0, 3.33, -10.95))
        n.result_callback(NS(result=lambda : NS(status=4)), 1)
        self.assertEqual(messages, ['succeeded'])
