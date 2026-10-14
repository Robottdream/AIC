import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/nav_simple'))
from nav_simple.crossing_passage import choose_passage, track_distance, static_clearance
from nav_simple.trajectory_prediction import crossing_conflicts, predict_position
from test_time_optimization import methods


class SafeCrossingTests(unittest.TestCase):
    def grid(self):
        return NS(info=NS(resolution=.05,width=200,height=200,
            origin=NS(position=NS(x=-5.,y=-5.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),
            data=[0]*40000)

    def track(self):
        return dict(name='left',points=(-3.65,1.,-1.4,1.),size=.5,speed=.35)

    def test_wait_point_is_outside_track_despite_arrival_tolerance(self):
        track=self.track();p=choose_passage([(0.,0.),(-3.7,1.),(-4.,2.5)],track,self.grid())
        self.assertIsNotNone(p)
        self.assertGreater(track_distance(p['entry'],track)-.15,.95)
        self.assertGreater(track_distance(p['exit'],track)-.15,.95)
        self.assertLess(track_distance((-1.1186,.5917),track),.95)

    def test_unknown_or_blocked_corridor_is_not_used(self):
        g=self.grid();g.data=[100]*len(g.data)
        self.assertIsNone(choose_passage([(0,0),(-4,2.5)],self.track(),g))
        self.assertIsNone(choose_passage([(0,0),(-4,2.5)],self.track(),None))

    def test_path_far_from_track_needs_no_crossing_stage(self):
        self.assertIsNone(choose_passage([(-8.,0.),(-8.,3.)],self.track(),self.grid()))

    def test_permanent_lane_avoids_short_track_crossing(self):
        track=self.track();track['points']=(-3.65,.6,-2.4,.6)
        passage=choose_passage([(0.,0.),(-3.,.6),(-4.,3.)],track,self.grid())
        self.assertTrue(passage['bypass'])
        for i in range(101):
            a,b=passage['entry'],passage['exit']
            point=(a[0]+i/100*(b[0]-a[0]),a[1]+i/100*(b[1]-a[1]))
            self.assertGreaterEqual(track_distance(point,track)-.15,1.049)

    def test_stale_nearby_obstacle_does_not_open_gate(self):
        t=self.track()
        self.assertEqual(crossing_conflicts([(-3.,0.),(-3.,2.)], [t], {'left':[]},
            1.,0.,.8,3.5),{'left'})
        self.assertEqual(predict_position(t,[(0.,-3.,1.),(.4,-3.,1.)],.4,2.),(-3.,1.))

    def test_static_clearance_includes_unknown_cells_and_rotated_origin(self):
        g=self.grid()
        # Unknown wall at local x=5.6 m, 0.6 m from the test position.
        for row in range(g.info.height):
            g.data[row*g.info.width+112]=-1
        self.assertAlmostEqual(static_clearance(g,(0.,0.)),.575,delta=.03)
        g.info.origin.orientation=NS(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5))
        self.assertAlmostEqual(static_clearance(g,(-10.,0.)),.575,delta=.03)
        self.assertEqual(static_clearance(g,(20.,20.)),0.)

    def test_gate_uses_same_world_segment_as_entry_guard(self):
        C=methods('nav_simple/nav_simple/simple_navigator.py','SimpleNav2Navigator',{'_scene_crossing_wait'})
        n=C();n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000))
        n.world_pose=(1.,-3.,0.);n.route_stage=dict(kind='cross',
            passage=dict(track=self.track(),exit=(-3.,2.)))
        n.scene_obstacle_samples={};n.scene_tracks=[];n.crossing_acceleration=3.5;n.crossing_cruise_speed=2.2
        captured=[]
        C._scene_crossing_wait.__globals__['crossing_conflicts']=lambda points,*args:captured.append(points) or set()
        path=NS(poses=[NS(pose=NS(position=NS(x=8.,y=8.)))])
        self.assertEqual(n._scene_crossing_wait(path),0.)
        self.assertIn([(-3.,0.),(-3.,2.)],captured)
        n.world_pose=(0.,-3.,0.)
        self.assertGreater(n._scene_crossing_wait(path),0.)

    def test_crossing_profile_is_confirmed_before_departure(self):
        C=methods('nav_simple/nav_simple/simple_navigator.py','SimpleNav2Navigator',{'_update_controller_profile'})
        env=C._update_controller_profile.__globals__
        env['static_clearance']=lambda *_:.5
        env['Parameter']=lambda name,value:NS(to_parameter_msg=lambda:NS(name=name,value=value))
        env['SetParametersAtomically']=NS(Request=lambda:NS(parameters=[]))
        n=C();n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000))
        n.world_pose=(1.,0.,0.);n.static_map=NS();n.goal_pose=NS()
        n.slow_near_static_obstacles=True
        n.controller_profile='near';n.controller_profile_pending=False
        n.controller_profile_retry_at=0.;n.crossing_cruise_speed=2.2
        n.route_stage={'kind':'entry'};ready=[];requests=[];callbacks=[]
        f=NS(add_done_callback=lambda cb:callbacks.append(cb),
             result=lambda:NS(result=NS(successful=True)))
        n.controller_parameters=NS(service_is_ready=lambda:True,
            call_async=lambda request:requests.append(request) or f)
        n.get_logger=lambda:NS(info=lambda _:None,warn=lambda _:None)
        n._update_controller_profile(lambda:ready.append('entry'))
        self.assertEqual(ready,['entry']);self.assertEqual(requests,[])
        n.route_stage={'kind':'cross'}
        n._update_controller_profile(lambda:ready.append('cross'))
        self.assertEqual(ready,['entry'])
        self.assertEqual({p.name:p.value for p in requests[0].parameters}[
            'FollowPath.max_lookahead_dist'],.45)
        self.assertEqual({p.name:p.value for p in requests[0].parameters}[
            'FollowPath.desired_linear_vel'],.8)
        callbacks[0](f)
        self.assertEqual(n.controller_profile,'cross')
        self.assertEqual(ready,['entry','cross'])

    def test_disabling_wall_speed_cap_preserves_short_lookahead_and_cruise(self):
        C=methods('nav_simple/nav_simple/simple_navigator.py','SimpleNav2Navigator',{'_update_controller_profile'})
        env=C._update_controller_profile.__globals__
        env['static_clearance']=lambda *_:.5
        env['Parameter']=lambda name,value:NS(to_parameter_msg=lambda:NS(name=name,value=value))
        env['SetParametersAtomically']=NS(Request=lambda:NS(parameters=[]))
        n=C();n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000))
        n.world_pose=(1.,0.,0.);n.static_map=NS();n.goal_pose=NS()
        n.controller_profile='open';n.controller_profile_pending=False
        n.controller_profile_retry_at=0.;n.crossing_cruise_speed=2.2
        n.slow_near_static_obstacles=False;n.route_stage=None
        requests=[];callbacks=[];ready=[]
        f=NS(add_done_callback=lambda cb:callbacks.append(cb),
             result=lambda:NS(result=NS(successful=True)))
        n.controller_parameters=NS(service_is_ready=lambda:True,
            call_async=lambda request:requests.append(request) or f)
        n.get_logger=lambda:NS(info=lambda _:None,warn=lambda _:None)
        n._update_controller_profile(lambda:ready.append(True))
        values={p.name:p.value for p in requests[0].parameters}
        self.assertEqual(values['FollowPath.desired_linear_vel'],2.2)
        self.assertEqual(values['FollowPath.max_lookahead_dist'],.45)
        self.assertEqual(values['FollowPath.lookahead_time'],.4)
        self.assertEqual(ready,[])
        callbacks[0](f)
        self.assertEqual(n.controller_profile,'near')
        self.assertEqual(ready,[True])

    def test_intermediate_arrival_does_not_report_delivery_target_reached(self):
        C=methods('nav_simple/nav_simple/simple_navigator.py','SimpleNav2Navigator',{'result_callback'})
        C.result_callback.__globals__['GoalStatus']=NS(STATUS_SUCCEEDED=4,STATUS_ABORTED=6,STATUS_CANCELED=5)
        n=C();n.request_id=1;n.safe_route=[{'kind':'entry'},{'kind':'cross'}]
        n.current_goal_handle=None;sent=[];status=[]
        n._send_route_stage=lambda:sent.append(n.safe_route[0]['kind'])
        n.status_pub=NS(publish=lambda msg:status.append(msg.data))
        n.result_callback(NS(result=lambda:NS(status=4)),1)
        self.assertEqual(sent,['cross']);self.assertEqual(status,[])


if __name__=='__main__':unittest.main()
