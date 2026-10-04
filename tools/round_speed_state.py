"""Read current competition state before testing translation limits."""
import json
import math
import time
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters

rclpy.init()
n = rclpy.create_node('round_speed_state_probe')
state = {}
def odom(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    state['pose'] = [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
    state['speed'] = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
subs = [n.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data),
        n.create_subscription(Int32, '/cur', lambda m: state.update(stage=m.data), 10),
        n.create_subscription(String, '/navigation/dynamic_guard', lambda m: state.update(guard=json.loads(m.data)), 10),
        n.create_subscription(Twist, '/cmd_vel', lambda m: state.update(command=[m.linear.x,m.linear.y,m.angular.z]), 10)]
def forecasts(m):
    data=json.loads(m.data)
    state['prediction']={'source':data.get('source'),'tracks':len(data['tracks']),'frame':data['frame']}
subs.append(n.create_subscription(String,'/navigation/predicted_obstacles',forecasts,10))
until = time.monotonic()+5
while time.monotonic()<until:
    rclpy.spin_once(n, timeout_sec=.05)
state['parameters']={}
for node,names in [('controller_server',['FollowPath.max_speed_xy','FollowPath.max_vel_x','FollowPath.max_vel_y','FollowPath.acc_lim_x','FollowPath.acc_lim_y','FollowPath.sim_time','FollowPath.critics','FollowPath.trajectory_generator_name','FollowPath.vx_samples','FollowPath.vy_samples']),
                   ('velocity_smoother',['max_velocity','min_velocity','max_accel','max_decel']),
                   ('scan_velocity_guard',['max_linear_speed','predictive_enabled','collision_footprint_radius']),
                   ('local_costmap/local_costmap',['width','height']),
                   ('global_costmap/global_costmap',['plugins'])]:
    client=n.create_client(GetParameters,'/'+node+'/get_parameters')
    assert client.wait_for_service(timeout_sec=10),node
    f=client.call_async(GetParameters.Request(names=names))
    rclpy.spin_until_future_complete(n,f,timeout_sec=10)
    assert f.done() and f.result(),node
    values={}
    for name,v in zip(names,f.result().values):
        values[name]=(v.double_value if v.type==3 else v.bool_value if v.type==1 else
                      v.integer_value if v.type==2 else list(v.double_array_value) if v.type==8 else
                      list(v.string_array_value) if v.type==9 else v.string_value)
    state['parameters'][node]=values
print(json.dumps(state, ensure_ascii=False), flush=True)
n.destroy_node()
rclpy.shutdown()
