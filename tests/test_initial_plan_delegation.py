"""Guarded-tree deployment and legacy/new navigation request behavior."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET
from test_time_optimization import methods

ROOT = Path(__file__).resolve().parents[1]


def configure(navigation):
    source = ROOT/'tools/project_launcher.py'
    tree = ast.parse(source.read_text())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                    and n.name == 'configure_first_plan_tree')
    namespace = dict(Path=Path, ROOT=ROOT)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace[function.name](navigation)


class DelegationTests(unittest.TestCase):
    def config(self, plugin='aic_timed_planner/TimeAwarePlanner', **bt):
        return dict(planner_server=dict(ros__parameters=dict(GridBased=dict(plugin=plugin))),
                    bt_navigator=dict(ros__parameters=dict(plugin_lib_names=['existing'], **bt)))

    def test_only_explicit_guarded_tree_enables_delegation_and_registers_plugin_once(self):
        config = self.config()
        expected = str(ROOT/'tools/continuous_navigation.xml')
        self.assertEqual(configure(config), expected)
        self.assertEqual(configure(config), expected)
        self.assertEqual(config['bt_navigator']['ros__parameters']['plugin_lib_names'],
                         ['existing', 'aic_current_goal_navigation_bt_node'])
        custom = self.config(default_nav_to_pose_bt_xml='/custom/tree.xml')
        self.assertEqual(configure(custom), '')
        self.assertEqual(custom['bt_navigator']['ros__parameters']['plugin_lib_names'], ['existing'])
        self.assertEqual(configure(self.config(plugin='nav2_navfn_planner/NavfnPlanner')), '')

    def test_guarded_tree_has_a_single_initial_and_rolling_compute_child(self):
        tree = ET.parse(ROOT/'tools/continuous_navigation.xml')
        guarded = tree.find('.//CurrentGoalNavigation')
        self.assertIsNotNone(guarded)
        self.assertEqual([n.tag for n in guarded], ['ComputePathToPose', 'RecoveryNode'])
        self.assertEqual(len(tree.findall('.//ComputePathToPose')), 1)
        self.assertEqual(guarded.get('path'), '{path}')
        self.assertEqual(guarded[0].get('path'), guarded.get('candidate_path'))
        self.assertEqual(guarded.find('.//FollowPath').get('path'), '{path}')
        self.assertIsNone(tree.find('.//ForceSuccess'))

    def node(self, tree):
        C = methods('nav_simple/nav_simple/simple_navigator.py', 'SimpleNav2Navigator', {'send_goal'})
        namespace = C.send_goal.__globals__
        namespace.update(PoseStamped=lambda: NS(header=NS(),pose=NS(position=NS(),orientation=NS())),
                         NavigateToPose=NS(Goal=NS), Quaternion=NS,
                         ComputePathToPose=NS(Goal=NS),
                         quaternion_from_euler=lambda a,b,y: (0.,0.,math.sin(y/2),math.cos(y/2)))
        node = C();node.request_id=2;node.emergency_stop_active=False;node.initial_plan_bt_xml=tree
        node.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda: 'fresh_stamp'))
        node.get_logger=lambda:NS(info=lambda *_: None)
        node.requests=[];node.started=[]
        node.plan_client=NS(send_goal_async=lambda r: node.requests.append(r) or
                            NS(add_done_callback=lambda *_: None))
        node._start_navigation=lambda *args:node.started.append(args)
        return node

    def test_guarded_request_sends_original_goal_and_tree_without_discarded_precheck(self):
        node=self.node('/guarded/tree.xml');node.send_goal(1.,2.,.3,2)
        self.assertEqual(node.requests,[]);self.assertEqual(len(node.started),1)
        goal, request_id=node.started[0]
        self.assertEqual(goal.behavior_tree,'/guarded/tree.xml')
        self.assertEqual((goal.pose.pose.position.x,goal.pose.pose.position.y,request_id),(1.,2.,2))
        self.assertEqual(goal.pose.header.stamp,'fresh_stamp')

    def test_legacy_request_still_prechecks_and_stale_or_stopped_requests_do_nothing(self):
        node=self.node('');node.send_goal(1.,2.,0.,2)
        self.assertEqual(len(node.requests),1);self.assertEqual(node.started,[])
        node.send_goal(1.,2.,0.,1)
        node.emergency_stop_active=True;node.send_goal(1.,2.,0.,2)
        self.assertEqual(len(node.requests),1);self.assertEqual(node.started,[])


if __name__ == '__main__': unittest.main()
