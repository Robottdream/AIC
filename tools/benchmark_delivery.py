import os, subprocess, signal, time, json, argparse, re, math
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
parser = argparse.ArgumentParser(description="Gazebo delivery benchmark; excludes language parsing latency")
parser.add_argument('--log-dir', required=True)
parser.add_argument('--timeout', type=float, default=380.)
parser.add_argument('--use-running', action='store_true', help='Use the one-click stack; stop it on detected instability')
args = parser.parse_args()
start_delay = float(os.environ.get('AIC_BENCHMARK_START_DELAY', '3'))
if not math.isfinite(start_delay) or not 0 <= start_delay <= 30:
 raise ValueError('AIC_BENCHMARK_START_DELAY must be between 0 and 30 seconds')
log = Path(args.log_dir).resolve()
log.mkdir(parents=True, exist_ok=False)
nav_share = Path(get_package_share_directory('bot_navigation'))
world_file = os.environ.get('AIC_WORLD')
map_file = os.environ.get('AIC_MAP', str(nav_share/'maps/map.yaml'))
nav_params = os.environ.get('AIC_NAV_PARAMS', str(nav_share/'param/originbot_nav2.yaml'))
task_config = os.environ.get('AIC_TASK_CONFIG')
node_arguments = ['--ros-args', '--params-file', task_config] if task_config else []
scene_metadata_file = Path(map_file).parent/'layout.json'
scene_metadata = json.loads(scene_metadata_file.read_text()) if scene_metadata_file.exists() else {}
zones = {'red':(3.143086,-5.807858),'blue':(-1.196544,-6.485499)}
area_centers = {'A':zones['red'], 'B':zones['blue'], 'C':(-6.873777,-7.785160)}
benchmark_tasks = json.loads(os.environ.get('AIC_BENCHMARK_TASKS',
 '[{"color":"red","num":3,"to":"A"},{"color":"blue","num":2,"to":"B"}]'))
if (len(benchmark_tasks)!=2 or {t['color'] for t in benchmark_tasks}!={'red','blue'}
 or sum(t['num'] for t in benchmark_tasks)!=5
 or any(not isinstance(t['num'],int) or t['num']<1 or t['to'] not in area_centers for t in benchmark_tasks)):
 raise ValueError('Benchmark needs two color assignments, at least one each, five total, and areas A/B/C')
zone_size = (1.0, 0.5)
if task_config:
 import yaml
 config = yaml.safe_load(Path(task_config).read_text())
 centers = config['arm_grab_place_node']['ros__parameters']['zone_centers']
 if len(centers) != 6 or not all(__import__('math').isfinite(float(v)) for v in centers):
  raise ValueError('zone_centers requires six finite coordinates')
 zones = {'red':tuple(centers[:2]), 'blue':tuple(centers[2:4])}
 area_centers = {key:tuple(centers[2*i:2*i+2]) for i,key in enumerate(('A','B','C'))}
 zone_size = tuple(config['arm_grab_place_node']['ros__parameters'].get('zone_size', zone_size))
zones = {task['color']:area_centers[task['to']] for task in benchmark_tasks}
processes=[]
def robot_overlaps_obstacle(sample, obstacle):
 name=obstacle['name']
 if name not in sample['obstacles']:return False
 ox,oy=sample['obstacles'][name];dx,dy=ox-sample['x'],oy-sample['y']
 c,s=math.cos(sample['yaw']),math.sin(sample['yaw']);half=obstacle['size']/2
 if obstacle.get('shape')=='cylinder':
  return math.hypot(max(abs(dx*c+dy*s)-.26,0),max(abs(-dx*s+dy*c)-.31,0))<=half
 return (abs(dx)<=half+.26*abs(c)+.31*abs(s)
  and abs(dy)<=half+.26*abs(s)+.31*abs(c)
  and abs(dx*c+dy*s)<=.26+half*(abs(c)+abs(s))
  and abs(-dx*s+dy*c)<=.31+half*(abs(c)+abs(s)))
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
  start('gazebo',['ros2','launch','mybot','gazebo_world.launch.py'] + ([f'world:={world_file}'] if world_file else []))
  time.sleep(12)
  for gazebo_process in ('gzserver', 'gzclient'):
   if subprocess.run(['pgrep', '-x', gazebo_process], stdout=subprocess.DEVNULL).returncode != 0:
    raise RuntimeError(f'{gazebo_process} failed to start; see gazebo.log')
  start('nav2',['ros2','launch','nav2_bringup','bringup_launch.py','use_sim_time:=true','map:='+map_file,'params_file:='+nav_params])
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
 collision=[None]
 motion_samples=[]
 latest_commands={}
 obstacle_poses={}
 for obstacle in scene_metadata.get('obstacles', []):
  name=obstacle['name']
  def obstacle_callback(msg, name=name):
   obstacle_poses[name]=(msg.position.x,msg.position.y)
  node.create_subscription(Pose,f'/{name}/current_pose',obstacle_callback,
   QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE))
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
  motion_samples.append(dict(sim=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
   wall=time.monotonic(),x=p.x,y=p.y,z=p.z,
   yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
   v=speed_samples[-1],w=msg.twist.twist.angular.z,tilt_deg=tilt_samples[-1],
   obstacles=dict(obstacle_poses),**latest_commands))
  if collision[0] is None:
   for obstacle in scene_metadata.get('obstacles', []):
    if robot_overlaps_obstacle(motion_samples[-1],obstacle):
     collision[0]=dict(obstacle=obstacle['name'],sim_seconds=motion_samples[-1]['sim'],
                      robot=(p.x,p.y),obstacle_position=obstacle_poses[obstacle['name']])
     break
  if tilt_samples[-1]>5.0 or abs(p.x)>20.0 or abs(p.y)>20.0 or p.z>0.3:
   instability[0]=dict(position=(p.x,p.y,p.z),tilt_deg=tilt_samples[-1])
 node.create_subscription(Odometry,'/odom',on_odom,QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE))
 nav_command_speeds=[]
 base_command_speeds=[]
 def nav_command(msg):
  nav_command_speeds.append(abs(msg.linear.x))
  latest_commands.update(nav_v=msg.linear.x,nav_w=msg.angular.z)
 def base_command(msg):
  base_command_speeds.append(abs(msg.linear.x))
  latest_commands.update(cmd_v=msg.linear.x,cmd_w=msg.angular.z)
 node.create_subscription(Twist,'/cmd_vel_nav',nav_command,10)
 node.create_subscription(Twist,'/cmd_vel',base_command,10)
 latest_amcl=[None]
 def on_amcl(msg):
  pose=msg.pose.pose
  q=pose.orientation
  yaw=__import__('math').atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  latest_amcl[0]=(pose.position.x,pose.position.y,yaw)
 node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',on_amcl,10)
 nav=ActionClient(node,NavigateToPose,'/navigate_to_pose')
 if not nav.wait_for_server(timeout_sec=70.): raise RuntimeError('Nav2 not ready')
 until=time.monotonic()+45.
 while latest_amcl[0] is None and time.monotonic()<until:
  rclpy.spin_once(node,timeout_sec=.2)
 if latest_amcl[0] is None:
  raise RuntimeError('Map localization not ready; check map path and nav2.log')
 if not args.use_running:
  start('navigator',['ros2','run','nav_simple','simple_navigator',*node_arguments])
  start('arm',['ros2','run','arm_action_integration','arm_grab_place_node',*node_arguments])
  start('main',['ros2','run','main_controller','main_controller_node',*node_arguments])
 else:
  previous=(log/'main.log').read_text(errors='replace')
  if '物块标记为已抓取' in previous:
   raise RuntimeError('Benchmark requires a fresh stack with all cubes at spawn positions')
 pub=node.create_publisher(String,'/chat',10)
 until=time.monotonic()+20
 while pub.get_subscription_count()==0 and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.2)
 if not pub.get_subscription_count():raise RuntimeError('main not ready')
 time.sleep(start_delay)
 after_sleep=time.monotonic();until=after_sleep+10
 while (clock_wall[0] is None or clock_wall[0]<after_sleep) and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.2)
 if clock_wall[0] is None or clock_wall[0]<after_sleep:raise RuntimeError('Fresh Gazebo /clock not available')
 until=time.monotonic()+5
 while not motion_samples and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
 if not motion_samples:raise RuntimeError('Gazebo odometry not available')
 speed_samples.clear()
 tilt_samples.clear();angular_samples.clear();motion_samples.clear()
 began=time.monotonic();sim_began=clock_ns[0]
 pub.publish(String(data=json.dumps(benchmark_tasks)))
 print(f'START wall={began:.3f} sim={sim_began/1e9:.3f}',flush=True)
 last_counts=(-1,-1)
 last_arrivals=0
 arrivals=[]
 placements=[]
 last_trace_save=began
 while time.monotonic()-began<args.timeout:
  rclpy.spin_once(node,timeout_sec=.5)
  if time.monotonic()-last_trace_save>=5.:
   temporary_trace=log/'motion.partial.tmp'
   temporary_trace.write_text(json.dumps(motion_samples))
   temporary_trace.replace(log/'motion.partial.json')
   last_trace_save=time.monotonic()
  s=(log/'main.log').read_text(errors='replace')
  if collision[0]:
   collision[0]['wall_seconds']=time.monotonic()-began
   print(f'ABORT_COLLISION {collision[0]}',flush=True)
   if args.use_running:
    subprocess.run(['python3',str(Path(__file__).resolve().parent/'project_launcher.py'),'stop'],check=True,timeout=30)
   break
  if instability[0]:
   print(f'ABORT_INSTABILITY {instability[0]}',flush=True)
   instability[0]['wall_seconds']=time.monotonic()-began
   # Save the trace before stopping Gazebo; the task controller otherwise keeps running.
   (log/'motion.json').write_text(json.dumps(motion_samples,indent=2))
   if args.use_running:
    subprocess.run(['python3',str(Path(__file__).resolve().parent/'project_launcher.py'),'stop'],check=True,timeout=30)
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
 elapsed=instability[0]['wall_seconds'] if instability[0] else time.monotonic()-began
 moving_samples=[v for v in speed_samples if v>0.05]
 fraction_above_062=sum(v>=0.62 for v in moving_samples)/len(moving_samples) if moving_samples else 0.0
 mean_moving_speed=sum(moving_samples)/len(moving_samples) if moving_samples else 0.0
 result=dict(timing_scope='parsed /chat publish to fifth place confirmation; excludes language parsing',wall_seconds=elapsed,sim_seconds=(clock_ns[0]-sim_began)/1e9,peak_odom_linear_mps=max(speed_samples,default=0.0),moving_fraction_at_least_062=fraction_above_062,mean_moving_speed_mps=mean_moving_speed,peak_nav_command_mps=max(nav_command_speeds,default=0.0),peak_base_command_mps=max(base_command_speeds,default=0.0),seconds=elapsed,complete=not instability[0] and not collision[0] and '所有优化任务执行完成' in s,grasp_count=s.count('机械臂抓取成功'),place_count=s.count('机械臂放置成功'),arrivals=arrivals,placements=placements)
 result['aborted_for_collision']=collision[0]
 result['task_start_delay_seconds']=start_delay
 navigator_log = log/'navigator.log'
 prediction_log = navigator_log.read_text(errors='replace') if navigator_log.exists() else ''
 result['prediction_checks'] = dict(
  configured_tracks=re.findall(r'已接入障碍轨迹预测: (\S+)',prediction_log),
  actual_wait_seconds=[float(v) for v in re.findall(r'轨迹预测放行，实际等待=([0-9.]+)s',prediction_log)],
  wait_timeout_count=prediction_log.count('轨迹预测等待超时'))
 result['final_positions']={p['cube']:p['position_at_confirmation'] for p in placements if p['cube'] and p['position_at_confirmation']}
 result['max_tilt_deg']=max(tilt_samples,default=0.0)
 result['aborted_for_instability']=instability[0]
 result['peak_angular_radps']=max(angular_samples,default=0.0)
 for cube in ([] if instability[0] else sorted(set(re.findall(r'选定可达物块：((?:red|blue)_cube_\d+)', s)))):
  try:
   out=subprocess.check_output(['gz','model','-m',cube,'-p'],timeout=8,text=True)
   (log/(cube+'.pose')).write_text(out)
   result.setdefault('final_positions',{})[cube]=tuple(map(float,out.split()[:3]))
  except Exception as e:print(e,flush=True)
 result['scene'] = dict(world=world_file, map=map_file, nav_params=nav_params, task_config=task_config, zones=zones, zone_size=zone_size, tasks=benchmark_tasks)
 result['obstacle_checks']={}
 for obstacle in scene_metadata.get('obstacles', []):
  import math
  name=obstacle['name'];sx,sy=obstacle['start'];ex,ey=obstacle['end']
  track_length_squared=(ex-sx)**2+(ey-sy)**2
  center_distances=[];track_distances=[];footprint_overlaps=0
  for sample in motion_samples:
   x,y=sample['x'],sample['y']
   fraction=max(0.,min(1.,((x-sx)*(ex-sx)+(y-sy)*(ey-sy))/track_length_squared))
   track_distances.append(math.hypot(x-sx-fraction*(ex-sx),y-sy-fraction*(ey-sy)))
   if name not in sample['obstacles']:continue
   ox,oy=sample['obstacles'][name];dx,dy=ox-x,oy-y
   center_distances.append(math.hypot(dx,dy))
   footprint_overlaps+=int(robot_overlaps_obstacle(sample,obstacle))
  result['obstacle_checks'][name]=dict(
   min_center_distance_m=min(center_distances,default=None),
   min_track_distance_m=min(track_distances,default=None),
   nav_footprint_overlap_samples=footprint_overlaps)
 result['zone_checks']={}
 for cube,pos in result.get('final_positions',{}).items():
  center=zones[cube.split('_')[0]]
  # Cubes are 0.03 m across; count only positions fully inside the visual zone.
  result['zone_checks'][cube]=abs(pos[0]-center[0])<=zone_size[0]/2-.015 and abs(pos[1]-center[1])<=zone_size[1]/2-.015
 result['all_five_in_zones']=len(result['zone_checks'])==5 and all(result['zone_checks'].values())
 result['height_checks']={cube:0.02<=pos[2]<=0.055 for cube,pos in result.get('final_positions',{}).items()}
 result['all_five_on_zone_surface']=result['all_five_in_zones'] and all(result['height_checks'].values())
 (log/'motion.json').write_text(json.dumps(motion_samples,indent=2))
 (log/'result.json').write_text(json.dumps(result,indent=2));print(result,flush=True)
finally:
 if 'motion_samples' in locals():
  (log/'motion.json').write_text(json.dumps(motion_samples,indent=2))
 for p,f in reversed(processes):
  try:os.killpg(p.pid,signal.SIGTERM)
  except ProcessLookupError:pass
 time.sleep(2)
 for p,f in processes:
  try:os.killpg(p.pid,signal.SIGKILL)
  except ProcessLookupError:pass
  f.close()
 if rclpy.ok(): rclpy.shutdown()
