"""Record a deliberately stopped stalled task and pause the active navigator."""
import json
import time
from pathlib import Path
import rclpy
from std_msgs.msg import String

root=Path.home()/'aic_mecanum_evidence_20261002'
folder=root/'new-v3-tasks'
results=json.loads((folder/'results.json').read_text())
trace=[json.loads(line) for line in (folder/'trace.jsonl').read_text().splitlines()]
last=trace[-1]
if not any(r['task']==last['task'] for r in results):
    samples=[r for r in trace if r['task']==last['task']]
    row={'task':last['task'],'command':'抓取1个红色去A','status':'stalled_test_stopped',
         'seconds':last['elapsed'],'places':0,'sim_seconds':last['sim']-samples[0]['sim'],
         'reason':'Sustained zero command near B exit after repeated recoveries; stopped by test operator, not a 300 s timeout',
         'last_pose':last.get('odom'),'recoveries':last.get('feedback',{}).get('recoveries')}
    results.append(row)
    (folder/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
    print(json.dumps(row,ensure_ascii=False))
rclpy.init(); node=rclpy.create_node('finish_mecanum_probe')
pub=node.create_publisher(String,'/manual_nav_target',10)
deadline=time.monotonic()+5
while pub.get_subscription_count()==0 and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.05)
pub.publish(String(data=json.dumps({'type':'pause'})))
deadline=time.monotonic()+1
while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.05)
node.destroy_node(); rclpy.shutdown()
