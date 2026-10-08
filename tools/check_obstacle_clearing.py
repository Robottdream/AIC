"""Reproduce a moving obstacle's residual cell in installed Nav2, without Gazebo.

All sensor/TF/output topics are isolated; no robot motion commands are sent.
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def trial(node, enabled, folder, beams=361):
    params = {'use_sim_time': False, 'global_frame': 'ghost_map',
              'robot_base_frame': 'ghost_laser', 'width': 10, 'height': 10,
              'origin_x': -5.0, 'origin_y': -5.0, 'resolution': .05,
              'update_frequency': 10.0, 'publish_frequency': 10.0,
              'always_send_full_costmap': True, 'track_unknown_space': False,
              'plugins': ['obstacle_layer'],
              'obstacle_layer': {'plugin': 'nav2_costmap_2d::ObstacleLayer',
                  'observation_sources': 'scan', 'scan': {
                      'topic': '/ghost/scan', 'data_type': 'LaserScan',
                      'marking': True, 'clearing': True, 'inf_is_valid': enabled,
                      'max_obstacle_height': 2.0, 'obstacle_max_range': 6.0,
                      'raytrace_max_range': 7.0}}}
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.safe_dump({'/**': {'ros__parameters': params}}, f)
        filename = f.name
    log = (folder / ('enabled.log' if enabled else 'baseline.log')).open('w')
    env = os.environ.copy()
    if env.get('AIC_TF2_FIX_LIB'):
        env['LD_PRELOAD'] = env['AIC_TF2_FIX_LIB']
    process = subprocess.Popen(['/opt/ros/humble/lib/nav2_costmap_2d/nav2_costmap_2d',
        '--ros-args', '--params-file', filename,
        '-r', '/tf:=/ghost/tf', '-r', '/tf_static:=/ghost/tf_static'],
        stdout=log, stderr=subprocess.STDOUT, env=env)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    tf_pub = node.create_publisher(TFMessage, '/ghost/tf_static', qos)
    scan_pub = node.create_publisher(LaserScan, '/ghost/scan',
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    maps = []
    sub = node.create_subscription(OccupancyGrid, '/costmap/costmap', maps.append, qos)
    client = node.create_client(ChangeState, '/costmap/costmap/change_state')
    try:
        transform = TransformStamped()
        transform.header.frame_id = 'ghost_map'
        transform.child_frame_id = 'ghost_laser'
        transform.transform.translation.z = .2
        transform.transform.rotation.w = 1.0
        tf_pub.publish(TFMessage(transforms=[transform]))
        assert client.wait_for_service(timeout_sec=15)
        for transition in (1, 3):
            req = ChangeState.Request()
            req.transition.id = transition
            future = client.call_async(req)
            deadline = time.monotonic()+15
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            print('transition', transition, future.result(), flush=True)
            assert future.result() and future.result().success

        def scan_for(seconds, old_hit, sweep=False):
            started = time.monotonic()
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                scan = LaserScan()
                scan.header.frame_id = 'ghost_laser'
                scan.header.stamp = node.get_clock().now().to_msg()
                scan.angle_min = -math.pi
                scan.angle_max = math.pi
                scan.angle_increment = 2*math.pi/(beams-1)
                scan.range_min = .1
                scan.range_max = 30.0
                scan.ranges = [float('inf')]*beams
                # An obstacle at x=2 moves away; the y=2 obstacle stays.
                if old_hit:
                    scan.ranges[(beams-1)//2] = 2.0
                scan.ranges[3*(beams-1)//4] = 2.0
                if sweep:
                    # A 0.75 m circular obstacle traverses the left corridor.
                    cx, cy = -4.75, 3-5*min(1, (time.monotonic()-started)/seconds)
                    for index in range(beams):
                        angle = scan.angle_min+index*scan.angle_increment
                        projection = cx*math.cos(angle)+cy*math.sin(angle)
                        disc = .375**2-(cx*cx+cy*cy-projection*projection)
                        if projection > 0 and disc >= 0:
                            scan.ranges[index] = projection-math.sqrt(disc)
                scan_pub.publish(scan)
                rclpy.spin_once(node, timeout_sec=.1)
            assert maps, 'Costmap not received'
            grid = maps[-1]
            def cost(x, y):
                ix = int((x-grid.info.origin.position.x)/grid.info.resolution)
                iy = int((y-grid.info.origin.position.y)/grid.info.resolution)
                # Beam float precision may put the endpoint in an adjacent cell.
                return max(grid.data[yy*grid.info.width+xx]
                    for yy in range(iy-1, iy+2) for xx in range(ix-1, ix+2))
            trail = sum(grid.data[iy*grid.info.width+ix] == 100
                for iy in range(grid.info.height) for ix in range(grid.info.width)
                if -5 < grid.info.origin.position.x+(ix+.5)*grid.info.resolution < -4
                and -2.5 < grid.info.origin.position.y+(iy+.5)*grid.info.resolution < 3.5)
            return {'old_obstacle': cost(2, 0), 'remaining_obstacle': cost(0, 2),
                    'corridor_lethal_cells': trail}
        deadline = time.monotonic() + 15
        while scan_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert scan_pub.get_subscription_count(), 'Scan subscription not discovered'
        before = scan_for(5, True)
        after = scan_for(2, False)
        print('observations', before, after, flush=True)
        assert before['old_obstacle'] == 100, 'Initial obstacle not marked'
        assert after['remaining_obstacle'] == 100, 'Real obstacle erased'
        assert after['old_obstacle'] == (0 if enabled else 100), 'Unexpected clearing result'
        scan_for(3, False, sweep=True)
        trail_after = scan_for(2, False)
        assert trail_after['remaining_obstacle'] == 100, 'Real obstacle erased after sweep'
        if enabled and beams == 1441:
            assert trail_after['corridor_lethal_cells'] == 0, 'Dense rays left a trail'
        return {'inf_is_valid': enabled, 'beams': beams, 'before': before,
                'after': after, 'trail_after': trail_after, 'pass': True}
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        Path(filename).unlink()
        node.destroy_subscription(sub)
        node.destroy_client(client)
        node.destroy_publisher(scan_pub)
        node.destroy_publisher(tf_pub)


def main():
    folder = Path(__file__).resolve().parents[1]/'log/ghost-obstacles-20261005'
    folder.mkdir(parents=True, exist_ok=True)
    if len(sys.argv) == 1:
        for flag in ('false', 'true', 'dense'):
            subprocess.run([sys.executable, __file__, flag], check=True)
        report = [json.loads((folder/(flag+'.json')).read_text()) for flag in ('false', 'true', 'dense')]
        (folder/'clearing-check.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        return
    enabled = sys.argv[1] != 'false'
    # Separate DDS domains avoid cached lifecycle endpoints between trials.
    os.environ['ROS_DOMAIN_ID'] = str({'false': 83, 'true': 84, 'dense': 85}[sys.argv[1]])
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['CYCLONEDDS_URI'] = '''<CycloneDDS><Domain id="any">
      <General><Interfaces><NetworkInterface address="127.0.0.1"/></Interfaces>
      <AllowMulticast>false</AllowMulticast><MaxMessageSize>1400B</MaxMessageSize>
      <FragmentSize>1200B</FragmentSize></General>
      <Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>128</MaxAutoParticipantIndex>
      <Peers><Peer Address="127.0.0.1"/></Peers></Discovery>
      </Domain></CycloneDDS>'''
    rclpy.init()
    node = rclpy.create_node('ghost_obstacle_test')
    try:
        report = trial(node, enabled, folder, 1441 if sys.argv[1] == 'dense' else 361)
        (folder/(sys.argv[1]+'.json')).write_text(json.dumps(report, indent=2)+'\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
