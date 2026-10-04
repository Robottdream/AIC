"""Compare fixed-heading lateral/forward Nav2 routes without task selection."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); p.add_argument('--cycles',type=int,default=3)
    a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(); n=rclpy.create_node('model_route_probe'); n.set_parameters([Parameter('use_sim_time',value=True)])
    state={}; rows=[]
    def odom(m):
        q=m.pose.pose.orientation
        state['pose']=[m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        state['actual']=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]
    subs=[n.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)]
    for topic in ['cmd_vel_nav','cmd_vel']:
        subs.append(n.create_subscription(Twist,'/'+topic,lambda m,t=topic:state.update({t:[m.linear.x,m.linear.y,m.angular.z]}),10))
    client=ActionClient(n,NavigateToPose,'/navigate_to_pose')
    def wait(f,limit):
        end=time.monotonic()+limit
        while not f.done() and time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.02)
        if not f.done(): raise TimeoutError('Action request timeout')
        return f.result()
    assert client.wait_for_server(timeout_sec=15)
    end=time.monotonic()+10
    while 'pose' not in state and time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.05)
    assert 'pose' in state
    end=time.monotonic()+10
    while n.get_clock().now().nanoseconds==0 and time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.02)
    assert n.get_clock().now().nanoseconds>0, 'No simulation clock'
    results=[]
    # The same map-frame targets, tolerances and scalar speed limits in both runs.
    for cycle in range(a.cycles):
        for kind,x,y in [('lateral',0.,1.),('return_lateral',0.,0.),('forward',1.,0.),('return_forward',0.,0.)]:
            g=NavigateToPose.Goal(); g.pose.header.frame_id='map'; g.pose.header.stamp=n.get_clock().now().to_msg()
            g.pose.pose.position.x=x; g.pose.pose.position.y=y; g.pose.pose.orientation.w=1.
            initial=dict(state); start=time.monotonic(); sim=n.get_clock().now().nanoseconds/1e9
            handle=wait(client.send_goal_async(g),10); assert handle.accepted
            f=handle.get_result_async(); end=start+60
            local=[]
            while not f.done() and time.monotonic()<end:
                rclpy.spin_once(n,timeout_sec=.02)
                local.append(dict(cycle=cycle+1,kind=kind,wall=time.monotonic()-start,**state))
            if not f.done(): wait(handle.cancel_goal_async(),5)
            result=dict(cycle=cycle+1,kind=kind,status=f.result().status if f.done() else 'timeout',seconds=time.monotonic()-start,sim_seconds=n.get_clock().now().nanoseconds/1e9-sim,initial=initial,final=dict(state),max_lateral_cmd=max((abs(r.get('cmd_vel',[0,0,0])[1]) for r in local),default=0.))
            results.append(result); rows.extend(local); print(json.dumps(result),flush=True)
            (a.output/'results.json').write_text(json.dumps(results,indent=2))
            if result['status']!=4: break
            pause=time.monotonic()+1
            while time.monotonic()<pause: rclpy.spin_once(n,timeout_sec=.02)
        if results[-1]['status']!=4: break
    (a.output/'trace.json').write_text(json.dumps(rows))
    n.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
