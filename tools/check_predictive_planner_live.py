"""Read-only proof of the installed predictor/critic/marker chain."""
import json
import time
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

rclpy.init()
n = rclpy.create_node('predictive_planner_live_check')
state = {'forecasts': [], 'planner': [], 'guard': [], 'marker_messages': 0,
         'prediction_markers': 0}
def markers(msg):
    state['marker_messages'] += 1
    state['prediction_markers'] = max(state['prediction_markers'],
                                     sum(m.action == m.ADD for m in msg.markers))
subscriptions = [n.create_subscription(String, '/navigation/'+topic,
    lambda m, key=key: state[key].append(json.loads(m.data)), 10)
    for topic, key in [('predicted_obstacles', 'forecasts'),
                       ('predictive_planner', 'planner'), ('dynamic_guard', 'guard')]]
subscriptions.append(n.create_subscription(MarkerArray, '/navigation/obstacle_predictions', markers, 10))
parameters = {}
for node, names in [('/controller_server', ['FollowPath.critics',
    'FollowPath.PredictiveObstacle.class', 'FollowPath.PredictiveObstacle.scale',
    'FollowPath.sim_time', 'FollowPath.discretize_by_time', 'FollowPath.time_granularity']),
    ('/scan_velocity_guard', ['predictive_enabled', 'publish_lidar_forecasts']),
    ('/global_costmap/global_costmap', ['plugins', 'known_trajectory_layer.plugin', 'known_trajectory_layer.enabled'])]:
    client = n.create_client(GetParameters, node+'/get_parameters')
    assert client.wait_for_service(timeout_sec=10), node
    request = GetParameters.Request(); request.names = names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(n, future, timeout_sec=10)
    assert future.done() and future.result(), 'Parameter query failed'
    parameters[node] = {}
    for key, value in zip(names, future.result().values):
        parameters[node][key] = (list(value.string_array_value) if value.type == 9 else
            value.string_value if value.type == 4 else value.double_value if value.type == 3
            else value.bool_value)
deadline = time.monotonic()+15
while time.monotonic() < deadline:
    rclpy.spin_once(n, timeout_sec=.1)
assert 'PredictiveObstacle' in parameters['/controller_server']['FollowPath.critics']
assert parameters['/scan_velocity_guard']['predictive_enabled']
assert not parameters['/scan_velocity_guard']['publish_lidar_forecasts']
assert 'known_trajectory_layer' in parameters['/global_costmap/global_costmap']['plugins']
assert state['forecasts'] and state['marker_messages'], 'No forecast/display stream'
assert state['guard'][-1]['tracking_updates'] > state['guard'][0]['tracking_updates'], 'Tracker stopped'
assert max(len(m['tracks']) for m in state['forecasts']) == 2, 'Missing known scene obstacle forecasts'
assert all(m.get('source') == 'known_scene_routes' for m in state['forecasts']), 'Mixed forecast publishers'
assert state['prediction_markers'] == 2, 'Known prediction paths not displayed'
report = {'parameters': parameters, 'forecast_messages': len(state['forecasts']),
          'max_tracks': max(len(m['tracks']) for m in state['forecasts']),
          'marker_messages': state['marker_messages'], 'prediction_markers': state['prediction_markers'],
          'planner_samples': len(state['planner']), 'last_planner': state['planner'][-3:],
          'last_guard': state['guard'][-3:], 'last_forecast': state['forecasts'][-1]}
out = Path('/home/polarbear/aic_predictive_planner_evidence_20261003')
(out/'live.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({key: report[key] for key in ('parameters', 'forecast_messages',
    'max_tracks', 'prediction_markers', 'planner_samples')}, indent=2))
n.destroy_node(); rclpy.shutdown()
