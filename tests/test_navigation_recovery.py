import math
import time
import unittest
from types import SimpleNamespace as NS
from test_time_optimization import methods


class NavigationRecoveryTests(unittest.TestCase):
    def navigator(self):
        C = methods('nav_simple/nav_simple/simple_navigator.py', 'SimpleNav2Navigator',
                    {'_recover_navigation', '_navigation_recovery_tick', '_cancel_pending_start',
                     '_recovery_plan_response', '_recovery_plan_result'})
        C._recover_navigation.__globals__.update(
            Twist=lambda: NS(), Empty=NS(Request=lambda: NS()),
            NavigateToPose=NS(Goal=lambda: NS()), ComputePathToPose=NS(Goal=lambda: NS()),
            GoalStatus=NS(STATUS_SUCCEEDED=4))
        node = C()
        node.request_id = 3
        node.emergency_stop_active = False
        node.navigation_retries = 0
        node.pending_start_timer = None
        node.recovery_context = None
        node.goal_pose = NS(header=NS(), pose=NS(position=NS(x=9., y=10.)))
        node.now = 20.
        node.get_clock = lambda: NS(now=lambda: NS(
            nanoseconds=int(node.now*1e9), to_msg=lambda: node.now))
        node.get_logger = lambda: NS(info=lambda *_: None, warn=lambda *_: None,
                                    error=lambda *_: None)
        node.stop_pub = NS(publish=lambda _: None)
        node.timer_canceled = False
        def cancel():
            node.timer_canceled = True
        node.create_timer = lambda *_: NS(cancel=cancel)
        node.destroy_timer = lambda _: None
        node.messages = []
        node.status_pub = NS(publish=lambda msg: node.messages.append(msg.data))
        node.scans = []
        node.nomotion_client = NS(service_is_ready=lambda: True,
            call_async=lambda _: node.scans.append('scan') or NS(done=lambda: True))
        node.launches = []
        node.queries = []
        node.plan_result = NS(status=4, result=NS(path=NS(header=NS(frame_id='map'),
            poses=[NS(pose=NS(position=NS(x=9., y=10.)))])))
        def completed(value):
            return NS(result=lambda: value, add_done_callback=lambda cb: cb(completed(value)))
        def query(goal):
            node.queries.append(goal)
            handle = NS(accepted=True, get_result_async=lambda: completed(node.plan_result),
                        cancel_goal_async=lambda: None)
            return completed(handle)
        node.plan_client = NS(send_goal_async=query)
        node._start_navigation = lambda goal, request: node.launches.append((goal, request))
        node.world_pose = (20., 1., 2.)
        node.world_motion = (20., 0., 0., 0.)
        node.localization_pose = (20., 1., 2., 0.)
        return node

    def test_requires_new_consistent_scan_and_stationary_chassis(self):
        n = self.navigator()
        self.assertTrue(n._recover_navigation(3, 'aborted'))
        n._navigation_recovery_tick()  # Old AMCL pose cannot restart driving.
        self.assertFalse(n.launches)
        self.assertEqual(n.scans, ['scan'])
        n.now = 20.2
        n.world_pose = (20.2, 1., 2.)
        n.world_motion = (20.2, 0., .5, 0.)
        n.localization_pose = (20.2, 1., 2., 0.)
        n._navigation_recovery_tick()
        self.assertFalse(n.launches)
        n.world_motion = (20.2, 0., 0., 0.)
        n.localization_pose = (20.2, 2., 2., 0.)
        n._navigation_recovery_tick()
        self.assertFalse(n.launches)
        n.localization_pose = (20.2, 1., 2., .1)
        n._navigation_recovery_tick()
        self.assertFalse(n.launches)  # A fresh route is also required.
        self.assertEqual(len(n.queries), 1)
        n._navigation_recovery_tick()
        self.assertEqual(len(n.launches), 1)
        self.assertEqual(n.launches[0][1], 3)
        self.assertTrue(n.timer_canceled)
        self.assertIsNone(n.recovery_context)
        self.assertFalse(n.messages)

    def test_temporary_no_path_waits_for_new_route_without_exhausting_navigation_retries(self):
        n = self.navigator()
        n._recover_navigation(3, 'aborted')
        n.now = 20.2
        n.world_pose = (20.2, 1., 2.)
        n.world_motion = (20.2, 0., 0., 0.)
        n.localization_pose = (20.2, 1., 2., 0.)
        n.plan_result.status = 6
        n._navigation_recovery_tick()
        n._navigation_recovery_tick()
        self.assertEqual(len(n.queries), 1)  # Backoff prevents rapid repeated actions.
        self.assertFalse(n.launches)
        self.assertFalse(n.messages)
        self.assertEqual(n.navigation_retries, 1)
        n.recovery_context['next_plan'] = time.monotonic()-1
        n.plan_result.status = 4
        n._navigation_recovery_tick()
        n._navigation_recovery_tick()
        self.assertEqual(len(n.queries), 2)
        self.assertEqual(len(n.launches), 1)
        self.assertEqual(n.navigation_retries, 1)

    def test_wrong_goal_path_does_not_restart_driving(self):
        n = self.navigator()
        n._recover_navigation(3, 'aborted')
        n.now = 20.2
        n.world_pose = (20.2, 1., 2.)
        n.world_motion = (20.2, 0., 0., 0.)
        n.localization_pose = (20.2, 1., 2., 0.)
        n.plan_result.result.path.poses[-1].pose.position.x = 0.
        n._navigation_recovery_tick()
        n._navigation_recovery_tick()
        self.assertFalse(n.launches)

    def test_timeout_and_retry_budget_fail_closed(self):
        n = self.navigator()
        n._recover_navigation(3, 'aborted')
        n.recovery_context['deadline'] = time.monotonic()-1
        n._navigation_recovery_tick()
        self.assertEqual(n.messages, ['failed: localization_recovery_timeout'])
        self.assertFalse(n.launches)
        self.assertTrue(n._recover_navigation(3, 'aborted'))
        n._cancel_pending_start()
        self.assertFalse(n._recover_navigation(3, 'aborted'))

    def test_old_request_and_emergency_stop_cannot_restart(self):
        n = self.navigator()
        self.assertFalse(n._recover_navigation(2, 'aborted'))
        n._recover_navigation(3, 'aborted')
        n.request_id = 4
        n._navigation_recovery_tick()
        self.assertIsNone(n.recovery_context)
        self.assertFalse(n.launches)
        n.emergency_stop_active = True
        self.assertFalse(n._recover_navigation(4, 'aborted'))

    def test_late_planning_acceptance_after_cancel_is_canceled(self):
        n = self.navigator()
        n._recover_navigation(3, 'aborted')
        old_context = n.recovery_context
        canceled = []
        handle = NS(accepted=True, cancel_goal_async=lambda: canceled.append(True))
        n._cancel_pending_start()
        n._recovery_plan_response(NS(result=lambda: handle), old_context)
        self.assertEqual(canceled, [True])
        self.assertFalse(n.launches)

    def test_expired_recovery_cannot_restart_even_with_valid_pose_and_path(self):
        n = self.navigator()
        n._recover_navigation(3, 'aborted')
        n.now = 20.2
        n.world_pose = (20.2, 1., 2.)
        n.world_motion = (20.2, 0., 0., 0.)
        n.localization_pose = (20.2, 1., 2., 0.)
        n.recovery_context['plan_good_until'] = math.inf
        n.recovery_context['deadline'] = time.monotonic()-1
        n._navigation_recovery_tick()
        self.assertFalse(n.launches)
        self.assertEqual(n.messages, ['failed: localization_recovery_timeout'])
