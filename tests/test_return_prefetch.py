"""Prefetch is speculative and bound to the moving pickup leg, never a goal."""
from types import SimpleNamespace as NS
import unittest
from test_time_optimization import methods


def controller():
    C = methods('main_controller/main_controller/main_controller_node.py',
                'MainControllerNode', {'_prefetch_return_route', '_cancel_return_prefetch',
                                       'get_grasp_candidates', '_nav_status_callback_impl'})
    C._prefetch_return_route.__globals__['Float64MultiArray'] = NS
    node = C()
    node.return_prefetch_enabled = True
    node.current_step = 'NAV_TO_BLOCK'
    node.current_assignment = dict(block_pos=(2., 3.), area_pos=(7., 1.))
    node.selected_block = (2., 3., 'red_cube_2', 1)
    node.current_grasp_candidate_index = 0
    node.GRASP_OFFSET = .55
    node.selection_id = 4
    node.current_task = dict(to='A')
    node.return_prefetch_tag = None
    node.messages = []
    node.return_prefetch_pub = NS(publish=lambda msg: node.messages.append(msg.data))
    node.get_logger = lambda: NS(info=lambda *_: None)
    return node, C


class ReturnPrefetchTests(unittest.TestCase):
    def test_only_measured_moving_pickup_leg_starts_once_and_does_not_send_navigation(self):
        node, C = controller()
        C._nav_status_callback_impl(lambda: node, NS(data='navigating'))
        self.assertEqual(node.messages, [])
        C._nav_status_callback_impl(lambda: node, NS(data='moving'))
        self.assertEqual(node.messages, [[1.45, 3., 0., 7., 1., 0.]])
        C._nav_status_callback_impl(lambda: node, NS(data='moving'))
        self.assertEqual(len(node.messages), 1)
        node.current_step = 'NAV_TO_AREA'
        C._nav_status_callback_impl(lambda: node, NS(data='moving'))
        self.assertEqual(len(node.messages), 1)

    def test_direction_retry_and_cancellation_invalidate_old_work(self):
        node, _ = controller()
        node._prefetch_return_route()
        old = node.return_prefetch_tag
        node._cancel_return_prefetch()
        self.assertIsNone(node.return_prefetch_tag)
        self.assertEqual(node.messages[-1], [])
        node.current_grasp_candidate_index = 1
        node._prefetch_return_route()
        self.assertNotEqual(node.return_prefetch_tag, old)
        self.assertEqual(node.messages[-1][:2], [2., 2.45])

    def test_disabled_missing_assignment_or_other_stage_is_noop(self):
        node, _ = controller()
        node.return_prefetch_enabled = False
        node._prefetch_return_route()
        node.return_prefetch_enabled = True
        node.current_assignment = None
        node._prefetch_return_route()
        self.assertEqual(node.messages, [])

    def test_motion_notification_requires_an_active_goal_and_measured_speed(self):
        C = methods('nav_simple/nav_simple/simple_navigator.py', 'SimpleNav2Navigator',
                    {'_world_callback'})
        node = C();node.current_goal_handle = None;node.emergency_stop_active = False
        node.request_id = 2;node.moving_request_id = None
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1000000000))
        node.get_logger = lambda: NS(info=lambda *_: None)
        messages = [];node.status_pub = NS(publish=lambda msg: messages.append(msg.data))
        msg = NS(name=['six_arm'],pose=[NS(position=NS(x=0.,y=0.))],
                 twist=[NS(linear=NS(x=.2,y=0.))])
        node._world_callback(msg)
        self.assertEqual(messages, [])
        node.current_goal_handle = object()
        msg.twist[0].linear.x = .05
        node._world_callback(msg)
        self.assertEqual(messages, [])
        msg.twist[0].linear.x = .2
        node._world_callback(msg);node._world_callback(msg)
        self.assertEqual(messages, ['moving'])
        node.request_id += 1;node.emergency_stop_active = True
        node._world_callback(msg)
        self.assertEqual(messages, ['moving'])


if __name__ == '__main__':
    unittest.main()
