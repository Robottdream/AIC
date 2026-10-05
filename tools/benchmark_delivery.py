import os, subprocess, signal, time, json, argparse, re
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
parser = argparse.ArgumentParser(description="Gazebo delivery benchmark; excludes language parsing latency")
parser.add_argument('--log-dir', required=True)
parser.add_argument('--timeout', type=float, default=380.)
parser.add_argument('--use-running', action='store_true', help='Use the one-click stack; do not start/stop modules')
args = parser.parse_args()
log = Path(args.log_dir).resolve()
log.mkdir(parents=True, exist_ok=False)
nav_share = Path(get_package_share_directory('bot_navigation'))
processes=[]
if not args.use_running and subprocess.run(['pgrep', '-x', 'gzserver'], stdout=subprocess.DEVNULL).returncode == 0:
 raise SystemExit('Existing gzserver detected; stop the existing simulation before benchmarking.')
def interrupted(*_):
 raise KeyboardInterrupt
signal.signal(signal.SIGTERM, interrupted)
def start(name,args):
 f=(log/(name+'.log')).open('w')
 child_env=os.environ.copy()
 if name=='nav2' and child_env.get('AIC_TF2_FIX_LIB'):
  tf_fix=child_env['AIC_TF2_FIX_LIB']
  if not Path(tf_fix).is_file():raise RuntimeError(f'TF fix library missing: {tf_fix}')
  child_env['LD_PRELOAD']=tf_fix+(' '+child_env['LD_PRELOAD'] if child_env.get('LD_PRELOAD') else '')
 p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=child_env)
 processes.append((p,f)); print(name,p.pid,flush=True)
try:
 if not args.use_running:
  start('gazebo',['ros2','launch','mybot','gazebo_world.launch.py'])
  time.sleep(12)
  for gazebo_process in ('gzserver', 'gzclient'):
   if subprocess.run(['pgrep', '-x', gazebo_process], stdout=subprocess.DEVNULL).returncode != 0:
    raise RuntimeError(f'{gazebo_process} failed to start; see gazebo.log')
  start('nav2',['ros2','launch','nav2_bringup','bringup_launch.py','use_sim_time:=true','map:='+str(nav_share/'maps/map.yaml'),'params_file:='+str(nav_share/'param/originbot_nav2.yaml')])
 else:
  (log/'main.log').symlink_to(Path(__file__).resolve().parents[1]/'log/run/09_main.log')
 rclpy.init();node=Node('algorithm_trial')
 clock_ns=[None];clock_wall=[None]
 def on_clock(msg):
  clock_ns[0]=msg.clock.sec*1000000000+msg.clock.nanosec
  clock_wall[0]=time.monotonic()
 node.create_subscription(Clock,'/clock',on_clock,QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE))
 speed_samples=[]
 tilt_samples=[]
 angular_samples=[]
 instability=[None]
 def on_odom(msg):
  velocity=msg.twist.twist.linear
  speed_samples.append((velocity.x**2+velocity.y**2)**0.5)
  import math
  q=msg.pose.pose.orientation
  roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
  pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
  tilt_samples.append(math.degrees(max(abs(roll),abs(pitch))))
  angular_samples.append(abs(msg.twist.twist.angular.z))
  p=msg.pose.pose.position
  if tilt_samples[-1]>5.0 or abs(p.x)>20.0 or abs(p.y)>20.0 or p.z>0.3:
   instability[0]=dict(position=(p.x,p.y,p.z),tilt_deg=tilt_samples[-1])
 node.create_subscription(Odometry,'/odom',on_odom,QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE))
 nav_command_speeds=[]
 base_command_speeds=[]
 node.create_subscription(Twist,'/cmd_vel_nav',lambda msg:nav_command_speeds.append(abs(msg.linear.x)),10)
 node.create_subscription(Twist,'/cmd_vel',lambda msg:base_command_speeds.append(abs(msg.linear.x)),10)
 latest_amcl=[None]
 def on_amcl(msg):
  pose=msg.pose.pose
  q=pose.orientation
  yaw=__import__('math').atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  latest_amcl[0]=(pose.position.x,pose.position.y,yaw)
 node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',on_amcl,10)
 nav=ActionClient(node,NavigateToPose,'/navigate_to_pose')
 if not nav.wait_for_server(timeout_sec=70.): raise RuntimeError('Nav2 not ready')
 if not args.use_running:
  start('navigator',['ros2','run','nav_simple','simple_navigator'])
  start('arm',['ros2','run','arm_action_integration','arm_grab_place_node'])
  start('main',['ros2','run','main_controller','main_controller_node'])
 else:
  previous=(log/'main.log').read_text(errors='replace')
  if '物块标记为已抓取' in previous:
   raise RuntimeError('Benchmark requires a fresh stack with all cubes at spawn positions')
 pub=node.create_publisher(String,'/chat',10)
 until=time.monotonic()+20
 while pub.get_subscription_count()==0 and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.2)
 if not pub.get_subscription_count():raise RuntimeError('main not ready')
 time.sleep(3)
 after_sleep=time.monotonic();until=after_sleep+10
 while (clock_wall[0] is None or clock_wall[0]<after_sleep) and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.2)
 if clock_wall[0] is None or clock_wall[0]<after_sleep:raise RuntimeError('Fresh Gazebo /clock not available')
 speed_samples.clear()
 began=time.monotonic();sim_began=clock_ns[0]
 pub.publish(String(data=json.dumps([dict(color='red',num=3,to='A'),dict(color='blue',num=2,to='B')])))
 print(f'START wall={began:.3f} sim={sim_began/1e9:.3f}',flush=True)
 last_counts=(-1,-1)
 last_arrivals=0
 arrivals=[]
 placements=[]
 while time.monotonic()-began<args.timeout:
  rclpy.spin_once(node,timeout_sec=.5)
  s=(log/'main.log').read_text(errors='replace')
  if instability[0]:
   print(f'ABORT_INSTABILITY {instability[0]}',flush=True)
   break
  counts=(s.count('机械臂抓取成功'),s.count('机械臂放置成功'))
  arrival_count=s.count('到达目标区域，准备放置')
  if arrival_count>last_arrivals:
   try:
    robot_at_arrival=tuple(map(float,subprocess.check_output(['gz','model','-m','six_arm','-p'],timeout=4,text=True).split()[:6]))
   except Exception as exc:
    print(f'ARRIVAL_POSE_QUERY_FAILED: {exc}',flush=True)
    robot_at_arrival=None
   snapshot={'place_index':arrival_count,'robot_ground_truth':robot_at_arrival,'robot_amcl':latest_amcl[0]}
   arrivals.append(snapshot)
   print(f'ARRIVAL {snapshot}',flush=True)
   last_arrivals=arrival_count
  if counts[1]>max(last_counts[1],0):
   chosen=re.findall(r'选定可达物块：((?:red|blue)_cube_\d+)',s)
   for index in range(max(last_counts[1],0),counts[1]):
    cube=chosen[index] if index<len(chosen) else None
    try:
     position=tuple(map(float,subprocess.check_output(['gz','model','-m',cube,'-p'],timeout=4,text=True).split()[:3]))
    except Exception as exc:
     print(f'POSITION_QUERY_FAILED {cube}: {exc}',flush=True)
     position=None
    try:
     robot_pose=tuple(map(float,subprocess.check_output(['gz','model','-m','six_arm','-p'],timeout=4,text=True).split()[:6]))
    except Exception as exc:
     print(f'ROBOT_POSE_QUERY_FAILED: {exc}',flush=True)
     robot_pose=None
    snapshot={'place_index':index+1,'cube':cube,'position_at_confirmation':position,'robot_ground_truth':robot_pose,'robot_amcl':latest_amcl[0]}
    placements.append(snapshot)
    print(f'PLACEMENT {snapshot}',flush=True)
  if counts!=last_counts:
   print(f'PROGRESS grabs={counts[0]} places={counts[1]} wall={time.monotonic()-began:.1f}s sim={(clock_ns[0]-sim_began)/1e9:.1f}s',flush=True)
   last_counts=counts
  if '所有优化任务执行完成' in s:break
  if '停止以避免误报完成' in s or '目标区域导航失败：' in s:break
  if any(p.poll() is not None for p,_ in processes):raise RuntimeError('child exited')
 elapsed=time.monotonic()-began
 moving_samples=[v for v in speed_samples if v>0.05]
 fraction_above_062=sum(v>=0.62 for v in moving_samples)/len(moving_samples) if moving_samples else 0.0
 mean_moving_speed=sum(moving_samples)/len(moving_samples) if moving_samples else 0.0
 result=dict(timing_scope='parsed /chat publish to fifth place confirmation; excludes language parsing',wall_seconds=elapsed,sim_seconds=(clock_ns[0]-sim_began)/1e9,peak_odom_linear_mps=max(speed_samples,default=0.0),moving_fraction_at_least_062=fraction_above_062,mean_moving_speed_mps=mean_moving_speed,peak_nav_command_mps=max(nav_command_speeds,default=0.0),peak_base_command_mps=max(base_command_speeds,default=0.0),seconds=elapsed,complete='所有优化任务执行完成' in s,grasp_count=s.count('机械臂抓取成功'),place_count=s.count('机械臂放置成功'),arrivals=arrivals,placements=placements)
 result['final_positions']={p['cube']:p['position_at_confirmation'] for p in placements if p['cube'] and p['position_at_confirmation']}
 result['max_tilt_deg']=max(tilt_samples,default=0.0)
 result['aborted_for_instability']=instability[0]
 result['peak_angular_radps']=max(angular_samples,default=0.0)
 for cube in sorted(set(re.findall(r'选定可达物块：((?:red|blue)_cube_\d+)', s))):
  try:
   out=subprocess.check_output(['gz','model','-m',cube,'-p'],timeout=8,text=True)
   (log/(cube+'.pose')).write_text(out)
   result.setdefault('final_positions',{})[cube]=tuple(map(float,out.split()[:3]))
  except Exception as e:print(e,flush=True)
 zones={'red':(3.143086,-5.807858),'blue':(-1.196544,-6.485499)}
 result['zone_checks']={}
 for cube,pos in result.get('final_positions',{}).items():
  center=zones[cube.split('_')[0]]
  # Cubes are 0.03 m across; count only positions fully inside the visual zone.
  result['zone_checks'][cube]=abs(pos[0]-center[0])<=0.485 and abs(pos[1]-center[1])<=0.235
 result['all_five_in_zones']=len(result['zone_checks'])==5 and all(result['zone_checks'].values())
 result['height_checks']={cube:0.02<=pos[2]<=0.055 for cube,pos in result.get('final_positions',{}).items()}
 result['all_five_on_zone_surface']=result['all_five_in_zones'] and all(result['height_checks'].values())
 (log/'result.json').write_text(json.dumps(result,indent=2));print(result,flush=True)
finally:
 for p,f in reversed(processes):
  try:os.killpg(p.pid,signal.SIGTERM)
  except ProcessLookupError:pass
 time.sleep(2)
 for p,f in processes:
  try:os.killpg(p.pid,signal.SIGKILL)
  except ProcessLookupError:pass
  f.close()
 if rclpy.ok(): rclpy.shutdown()
