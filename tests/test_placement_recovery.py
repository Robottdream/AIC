"""Regressions for measured cargo placement and safe recovery sequencing."""
import math
import threading
import unittest
from types import SimpleNamespace as NS
from test_time_optimization import methods

ARM = 'arm_action_integration/arm_action_integration/arm_grab_place_node.py'
MAIN = 'main_controller/main_controller/main_controller_node.py'

class PlacementRecoveryTests(unittest.TestCase):
    def test_logged_fourth_cube_can_be_placed_without_moving_base(self):
        C = methods(ARM, 'ArmGrabPlaceNode', {'_placement_yaw_candidates'})
        pivot, cube, center = (-7.84479,-5.6169), (-7.204,-5.753), (-7.4,-5.4)
        angles = C._placement_yaw_candidates(cube,pivot,center,(.5,.25),0.)
        self.assertTrue(angles)
        angle=angles[0];vx,vy=cube[0]-pivot[0],cube[1]-pivot[1]
        x=pivot[0]+math.cos(angle)*vx-math.sin(angle)*vy
        y=pivot[1]+math.sin(angle)*vx+math.cos(angle)*vy
        self.assertLessEqual(abs(x-center[0]),.42)
        self.assertLessEqual(abs(y-center[1]),.17)
        self.assertLess(abs(angle),.5)

    def test_far_or_nonfinite_cube_has_no_arm_solution(self):
        C=methods(ARM,'ArmGrabPlaceNode',{'_placement_yaw_candidates'})
        for cube in ((3.,0.),(float('nan'),0.)):
            self.assertEqual(C._placement_yaw_candidates(cube,(0.,0.),(.7,0.),(.5,.25),0.),[])

    def test_arm_sweep_rejects_wall_unknown_or_missing_map(self):
        C=methods(ARM,'ArmGrabPlaceNode',{'_arm_sweep_clear'})
        node=C();node.static_map=None
        self.assertFalse(node._arm_sweep_clear((.65,-.14),(0.,0.),.3))
        grid=NS(info=NS(resolution=.05,width=80,height=80,
            origin=NS(position=NS(x=-2.,y=-2.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=[0]*6400)
        node.static_map=grid
        self.assertTrue(node._arm_sweep_clear((.65,-.14),(0.,0.),.3))
        for occupied in (100,-1):
            grid.data=[0]*6400
            for y in range(35,45):
                for x in range(49,55):grid.data[y*80+x]=occupied
            self.assertFalse(node._arm_sweep_clear((.65,-.14),(0.,0.),.3))

    def test_reposition_waits_for_arm_and_gripper_to_stow(self):
        C=methods(ARM,'ArmGrabPlaceNode',{'_prepare_place_reposition'})
        node=C();node.is_executing=True;node.current_task='place';node.place_release_lock=threading.Lock()
        node._stop_alignment=lambda:None;node.arm_trajectory={'lift':[0.]*6}
        node.get_logger=lambda:NS(warn=lambda *_:None)
        actions={};messages=[]
        node.send_arm_action=lambda target,done:actions.update(arm=done)
        node.send_gripper_action=lambda target,done:actions.update(gripper=done)
        node.arm_status_pub=NS(publish=lambda msg:messages.append(msg.data))
        node._prepare_place_reposition(-.196,.353)
        self.assertEqual(messages,[])
        actions['arm']();self.assertEqual(messages,[])
        actions['gripper']();self.assertEqual(messages,['place_reposition:-0.1960:0.3530'])
        self.assertFalse(node.is_executing)
        actions['arm']();self.assertEqual(len(messages),1)

    def test_base_reposition_uses_actual_pose_and_preserves_heading(self):
        C=methods(MAIN,'MainControllerNode',{'_arm_status_callback_impl','_cancel_return_prefetch'})
        C._cancel_return_prefetch.__globals__['Float64MultiArray']=NS
        node=C();node.current_step='PLACE';node.place_timer=None
        node.current_place_retry=1;node.max_place_retry=3
        node.current_robot_pose=(-7.83,-5.578);node.current_robot_yaw=.101
        node.area_nav_targets=[(-7.95,-5.56,0.)];node.area_nav_target_index=0
        node.get_logger=lambda:NS(warn=lambda *_:None)
        canceled=[];node.return_prefetch_pub=NS(publish=lambda msg:canceled.append(msg.data))
        sent=[];node._send_current_area_target=lambda:sent.append(node.area_nav_targets[-1])
        C._arm_status_callback_impl(lambda:node,NS(data='place_reposition:-0.196:0.353'))
        self.assertEqual(len(sent),1)
        self.assertAlmostEqual(sent[0][0],-8.026)
        self.assertAlmostEqual(sent[0][1],-5.228)
        self.assertEqual(sent[0][2],.101)
        self.assertEqual(canceled,[[]])


if __name__=='__main__':unittest.main()
