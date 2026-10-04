"""Read predictor parameters/diagnostics and compare actual scan clustering."""
import json
import time
from pathlib import Path
import sys
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformException
from tf2_msgs.msg import TFMessage
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/yzbot/bot_navigation/scripts'))
from dynamic_obstacle_tracker import clusters
rclpy.init(); n=rclpy.create_node('prediction_live_check')
n.set_parameters([Parameter('use_sim_time',value=True)])
buffer=Buffer(node=n)
state={'diagnostics':[],'clusters':{}}
state['tf_checks']={'success':0,'errors':[]}
def tf(m):
    for t in m.transforms: buffer.set_transform(t,'probe')
def fixed(m):
    for t in m.transforms: buffer.set_transform_static(t,'probe')
subs=[n.create_subscription(String,'/navigation/dynamic_guard',lambda m:state['diagnostics'].append(json.loads(m.data)),10)]
subs.append(n.create_subscription(TFMessage,'/navigation/tf',tf,100))
subs.append(n.create_subscription(TFMessage,'/navigation/tf_static',fixed,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
def scan(m,topic):
    groups=clusters(m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max,(0,0,0))
    state['clusters'][topic]={'groups':len(groups),'max_span_m':max((g[1] for g in groups),default=0.),'finite_ranges':sum(__import__('math').isfinite(x) for x in m.ranges)}
    if topic=='/scan':
        try:
            buffer.lookup_transform('odom',m.header.frame_id,rclpy.time.Time.from_msg(m.header.stamp))
            buffer.lookup_transform('odom','base_footprint',rclpy.time.Time.from_msg(m.header.stamp))
            state['tf_checks']['success']+=1
        except TransformException as e:
            if len(state['tf_checks']['errors'])<4: state['tf_checks']['errors'].append(str(e))
for t in ('/scan','/scan_left','/scan_right'): subs.append(n.create_subscription(LaserScan,t,lambda m,t=t:scan(m,t),qos_profile_sensor_data))
client=n.create_client(GetParameters,'/scan_velocity_guard/get_parameters')
assert client.wait_for_service(timeout_sec=15),'Guard parameter service unavailable'
request=GetParameters.Request(); request.names=['predictive_enabled']
future=client.call_async(request)
end=time.monotonic()+12
while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.05)
assert future.done(),'GetParameters timeout'
state['predictive_enabled']=future.result().values[0].bool_value
assert state['predictive_enabled'],'Prediction disabled in running node'
assert state['diagnostics'] and any(m.get('reason')!='disabled' for m in state['diagnostics']),'No active prediction diagnostics'
state['diagnostic_samples']=len(state['diagnostics'])
assert max(m.get('tracking_updates',0) for m in state['diagnostics'])>min(m.get('tracking_updates',0) for m in state['diagnostics']), 'Scan tracker is not updating'
assert max(m.get('prediction_checks',0) for m in state['diagnostics'])>0, 'Predictor never evaluated a velocity command'
state['diagnostics']=state['diagnostics'][-3:]
print(json.dumps(state,ensure_ascii=False,indent=2))
n.destroy_node(); rclpy.shutdown()
