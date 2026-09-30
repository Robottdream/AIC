"""Time ordered competition tasks using wall/simulation clocks and record IMU/odometry.
Run after the one-click launcher finishes: --output DIR [--limit N].
"""
import json
import math
import time
import sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PolygonStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from nav2_msgs.action import NavigateToPose

OUTPUT = Path('/mnt/e/workspace/AIC/log/speed-probe')
COMMANDS = ['抓取1个蓝色去B', '抓取1个红色去A', '抓取1个蓝色去A',
            '抓取1个红色去C', '抓取1个蓝色去C', '抓取1个红色去B']
START_INDEX = 1
if '--two-blue' in sys.argv:
    COMMANDS = ['抓取2个蓝色去B']
if '--command' in sys.argv:
    COMMANDS = [sys.argv[sys.argv.index('--command')+1]]
if '--task6' in sys.argv:
    OUTPUT = OUTPUT/'task6'
    COMMANDS = [COMMANDS[-1]]
    START_INDEX = 6
if '--corner' in sys.argv:
    COMMANDS = ['抓取1个红色去C']
    START_INDEX = 4
if '--c-regression' in sys.argv:
    COMMANDS = ['抓取1个红色去C','抓取1个蓝色去C']
    START_INDEX = 4
if '--blue-c' in sys.argv:
    COMMANDS = ['抓取1个蓝色去C']
    START_INDEX = 5
if '--limit' in sys.argv:
    COMMANDS = COMMANDS[:int(sys.argv[sys.argv.index('--limit')+1])]
if '--output' in sys.argv:
    OUTPUT = Path(sys.argv[sys.argv.index('--output')+1])

class Probe(Node):
    def __init__(self):
        super().__init__('multi_task_timing_probe')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        OUTPUT.mkdir(parents=True, exist_ok=True)
        self.out = (OUTPUT/'trace.jsonl').open('w')
        self.start = time.monotonic()
        self.task_start = self.start
        self.task_index = 0
        self.placed = None
        self.place_count = 0
        self.placed_sim = None
        self.last_nav_status = None
        self.active_seen = False
        self.latest = {}
        self.subs = []
        for topic in ['/command','/chat','/manual_nav_target','/current_target_cube',
                      '/nav_status','/arm_status','/nav_done_cargo','/nav_done_area']:
            self.subs.append(self.create_subscription(String, topic, lambda m,t=topic:self.event(t,m.data),10))
        self.subs.append(self.create_subscription(Int32,'/cur',self.stage,10))
        self.subs.append(self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:self.pose('amcl',m.pose.pose),qos_profile_sensor_data))
        self.subs.append(self.create_subscription(Odometry,'/odom',lambda m:(self.pose('odom',m.pose.pose),self.latest.update(actual_velocity=[m.twist.twist.linear.x,m.twist.twist.angular.z])),qos_profile_sensor_data))
        self.subs.append(self.create_subscription(Twist,'/cmd_vel',lambda m:self.latest.update(cmd=[m.linear.x,m.angular.z]),qos_profile_sensor_data))
        self.subs.append(self.create_subscription(NavigateToPose.Impl.FeedbackMessage,'/navigate_to_pose/_action/feedback',self.feedback,qos_profile_sensor_data))
        self.subs.append(self.create_subscription(Imu,'/imu',self.imu,qos_profile_sensor_data))
        for scope in ('local','global'):
            self.subs.append(self.create_subscription(PolygonStamped,f'/{scope}_costmap/published_footprint',lambda m,k=scope:self.latest.update({k+'_clock':m.header.stamp.sec+m.header.stamp.nanosec/1e9}),qos_profile_sensor_data))
        self.pub=self.create_publisher(String,'/command',10)
        self.nav_pub=self.create_publisher(String,'/manual_nav_target',10)
        self.create_timer(.2,lambda:self.write(dict(type='sample',**self.latest)))

    def stage(self,m):
        self.latest['stage']=m.data
        if m.data>0:self.active_seen=True

    def pose(self,key,p):
        q=p.orientation
        self.latest[key]=[p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]

    def imu(self,m):
        q=m.orientation
        self.latest['tilt']=[math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))]

    def feedback(self,m):
        f=m.feedback
        self.latest['feedback']=dict(distance=f.distance_remaining,recoveries=f.number_of_recoveries)
        self.latest['feedback_wall']=time.monotonic()-self.start
        self.pose('nav_pose',f.current_pose.pose)

    def write(self,d):
        if self.task_index==0:return
        d.update(task=self.task_index,wall=time.monotonic()-self.start,
                 elapsed=time.monotonic()-self.task_start,sim=self.get_clock().now().nanoseconds/1e9)
        self.out.write(json.dumps(d,ensure_ascii=False)+'\n');self.out.flush()

    def event(self,topic,data):
        self.write(dict(type='event',topic=topic,data=data))
        if topic=='/nav_status':self.last_nav_status=data
        if self.task_index==0 and topic in ('/manual_nav_target','/nav_status'):
            print('PREPOSITION EVENT',topic,data,flush=True)
        if self.task_index:
            print(f'T{self.task_index} {time.monotonic()-self.task_start:.2f}s {topic}: {data}',flush=True)
        if topic=='/manual_nav_target':
            self.latest['target']=json.loads(data)
            for key in ['feedback','nav_pose','feedback_wall']:self.latest.pop(key,None)
        if topic=='/arm_status' and data=='place_succeeded':
            self.placed=time.monotonic()
            self.placed_sim=self.get_clock().now().nanoseconds/1e9
            self.place_count+=1

rclpy.init()
node=Probe()
deadline=time.monotonic()+20
while node.pub.get_subscription_count()<2 and time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.1)
if node.pub.get_subscription_count()<2:raise RuntimeError('Parser not ready')
deadline=time.monotonic()+10
while node.get_clock().now().nanoseconds==0 and time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.1)
if node.get_clock().now().nanoseconds==0:raise RuntimeError('Simulation clock not ready')
if '--corner' in sys.argv or '--preposition-c' in sys.argv:
    deadline=time.monotonic()+20
    while node.nav_pub.get_subscription_count()<2 and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
    if node.nav_pub.get_subscription_count()<2:raise RuntimeError('Navigator not ready')
    preposition = {'type':'custom','x':-7.423777,'y':-7.785160,'yaw':0.0} if '--preposition-c' in sys.argv else {'type':'custom','x':2.593086,'y':-5.727858,'yaw':0.0}
    print('PREPOSITION MATCHES',node.nav_pub.get_subscription_count(),flush=True)
    for attempt in range(1,4):
        node.last_nav_status=None
        node.nav_pub.publish(String(data=json.dumps(preposition)))
        print('PREPOSITION: navigating to parking point',preposition,'attempt',attempt,flush=True)
        deadline=time.monotonic()+90
        while node.last_nav_status is None and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        if node.last_nav_status=='succeeded':break
        print('PREPOSITION: retry after',node.last_nav_status,flush=True)
        time.sleep(3)
    if node.last_nav_status!='succeeded':
        raise RuntimeError('Preposition failed: '+str(node.last_nav_status))
    print('PREPOSITION: arrived',flush=True)
results=[]
for i,command in enumerate(COMMANDS,START_INDEX):
    node.task_index=i;node.task_start=time.monotonic();node.placed=None;node.active_seen=False
    node.place_count=0;node.placed_sim=None
    task_sim_start=node.get_clock().now().nanoseconds/1e9
    placed_sim=None
    node.write(dict(type='publish',command=command))
    print(f'BEGIN TASK {i}: {command}',flush=True)
    node.pub.publish(String(data=command))
    status='timeout'
    while time.monotonic()-node.task_start<300:
        rclpy.spin_once(node,timeout_sec=.1)
        if node.placed and placed_sim is None:placed_sim=node.get_clock().now().nanoseconds/1e9
        if node.placed and time.monotonic()-node.placed>3 and node.latest.get('stage')==0:
            status='success';break
        if node.active_seen and node.latest.get('stage')==0 and not node.placed and time.monotonic()-node.task_start>10:
            status='failed';break
    elapsed=(node.placed or time.monotonic())-node.task_start
    result=dict(task=i,command=command,status=status,seconds=elapsed,
                places=node.place_count,
                sim_seconds=(node.placed_sim or node.get_clock().now().nanoseconds/1e9)-task_sim_start)
    results.append(result);node.write(dict(type='task_end',**result))
    (OUTPUT/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    print('TASK RESULT '+json.dumps(result,ensure_ascii=False),flush=True)
    if status!='success':break
node.out.close();node.destroy_node();rclpy.shutdown()
