import os, subprocess, signal, time, json, argparse, re
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
parser = argparse.ArgumentParser(description="Gazebo delivery benchmark; excludes language parsing latency")
parser.add_argument('--log-dir', required=True)
parser.add_argument('--timeout', type=float, default=380.)
args = parser.parse_args()
log = Path(args.log_dir).resolve()
log.mkdir(parents=True, exist_ok=False)
nav_share = Path(get_package_share_directory('bot_navigation'))
processes=[]
if subprocess.run(['pgrep', '-x', 'gzserver'], stdout=subprocess.DEVNULL).returncode == 0:
 raise SystemExit('Existing gzserver detected; stop the existing simulation before benchmarking.')
def interrupted(*_):
 raise KeyboardInterrupt
signal.signal(signal.SIGTERM, interrupted)
def start(name,args):
 f=(log/(name+'.log')).open('w')
 p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 processes.append((p,f)); print(name,p.pid,flush=True)
try:
 start('gazebo',['ros2','launch','mybot','gazebo_world.launch.py'])
 time.sleep(12)
 start('nav2',['ros2','launch','nav2_bringup','bringup_launch.py','use_sim_time:=true','map:='+str(nav_share/'maps/map.yaml'),'params_file:='+str(nav_share/'param/originbot_nav2.yaml')])
 rclpy.init();node=Node('algorithm_trial')
 nav=ActionClient(node,NavigateToPose,'/navigate_to_pose')
 if not nav.wait_for_server(timeout_sec=70.): raise RuntimeError('Nav2 not ready')
 start('navigator',['ros2','run','nav_simple','simple_navigator'])
 start('arm',['ros2','run','arm_action_integration','arm_grab_place_node'])
 start('main',['ros2','run','main_controller','main_controller_node'])
 pub=node.create_publisher(String,'/chat',10)
 until=time.monotonic()+20
 while pub.get_subscription_count()==0 and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.2)
 if not pub.get_subscription_count():raise RuntimeError('main not ready')
 time.sleep(3)
 began=time.monotonic()
 pub.publish(String(data=json.dumps([dict(color='red',num=3,to='A'),dict(color='blue',num=2,to='B')])))
 print('START',flush=True)
 while time.monotonic()-began<args.timeout:
  rclpy.spin_once(node,timeout_sec=.5)
  s=(log/'main.log').read_text(errors='replace')
  if '所有优化任务执行完成' in s:break
  if any(p.poll() is not None for p,_ in processes):raise RuntimeError('child exited')
 elapsed=time.monotonic()-began
 result=dict(timing_scope='parsed /chat publish to fifth place confirmation; excludes language parsing',seconds=elapsed,complete='所有优化任务执行完成' in s,grasp_count=s.count('机械臂抓取成功'),place_count=s.count('机械臂放置成功'))
 (log/'result.json').write_text(json.dumps(result,indent=2));print(result,flush=True)
 for cube in sorted(set(re.findall(r'选定可达物块：((?:red|blue)_cube_\d+)', s))):
  try:
   out=subprocess.check_output(['gz','model','-m',cube,'-p'],timeout=8,text=True)
   (log/(cube+'.pose')).write_text(out)
  except Exception as e:print(e,flush=True)
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
