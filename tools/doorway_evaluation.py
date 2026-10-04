import json,time
from pathlib import Path
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from dwb_msgs.msg import LocalPlanEvaluation
from nav_msgs.msg import Path as PathMessage
from std_msgs.msg import String

rclpy.init();n=rclpy.create_node('doorway_evaluation_probe')
print([t for t in n.get_topic_names_and_types() if any(s in t[0] for s in ('evaluation','local_plan','transformed','global_plan'))],flush=True)
client=n.create_client(SetParameters,'/controller_server/set_parameters');assert client.wait_for_service(timeout_sec=10)
f=client.call_async(SetParameters.Request(parameters=[Parameter('FollowPath.debug_trajectory_details',value=True).to_parameter_msg(),Parameter('FollowPath.publish_evaluation',value=True).to_parameter_msg()]))
rclpy.spin_until_future_complete(n,f,timeout_sec=10);print(f.result(),flush=True)
state={};subs=[]
def evaluation(m):
    state['best']=m.best_index
    state['trajectories']=[{'v':[t.traj.velocity.x,t.traj.velocity.y,t.traj.velocity.theta],
                            'total':t.total,'critics':[{ 'name':s.name,'raw':s.raw_score,'scale':s.scale} for s in t.scores]} for t in m.twists]
for name,types in n.get_topic_names_and_types():
    if 'dwb_msgs/msg/LocalPlanEvaluation' in types:subs.append(n.create_subscription(LocalPlanEvaluation,name,evaluation,10))
    if 'nav_msgs/msg/Path' in types and any(k in name for k in ('local_plan','transformed','received_global')):
        subs.append(n.create_subscription(PathMessage,name,lambda m,k=name:state.update({k:[[p.pose.position.x,p.pose.position.y] for p in m.poses]}),10))
subs.append(n.create_subscription(String,'/navigation/predictive_planner',lambda m:state.update(prediction=json.loads(m.data)),10))
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
out=Path('/home/polarbear/aic_doorway_evidence_20261003/evaluation.json');out.write_text(json.dumps(state,indent=2)+'\n')
print(json.dumps({k:v for k,v in state.items() if k!='trajectories'},indent=2),flush=True)
if 'trajectories' in state:
    print(json.dumps(sorted(state['trajectories'],key=lambda x:x['total'])[:9],indent=2),flush=True)
n.destroy_node();rclpy.shutdown()
