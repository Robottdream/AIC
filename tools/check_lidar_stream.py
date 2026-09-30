"""Measure both live lidar streams and real-time factor without moving the robot."""
import json
import time
from pathlib import Path
import rclpy
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('lidar_stream_check')
    scans = {key: [] for key in ('scan', 'scan_high')}
    clocks = []
    for key in scans:
        node.create_subscription(LaserScan, '/'+key,
            lambda m, k=key: scans[k].append((time.monotonic(),
                m.header.stamp.sec+m.header.stamp.nanosec/1e9, len(m.ranges))),
            qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock',
        lambda m: clocks.append((time.monotonic(), m.clock.sec+m.clock.nanosec/1e9)),
        qos_profile_sensor_data)
    deadline = time.monotonic()+20
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    report = {}
    for key, rows in scans.items():
        assert len(rows) > 10, 'Missing lidar stream: '+key
        report[key] = {'messages': len(rows), 'beams': sorted({r[2] for r in rows}),
            'wall_hz': (len(rows)-1)/(rows[-1][0]-rows[0][0]),
            'sim_hz': (len(rows)-1)/(rows[-1][1]-rows[0][1])}
        assert report[key]['beams'] == [1441], 'Unexpected lidar resolution'
    assert len(clocks) > 2, 'Missing simulation clock'
    report['real_time_factor'] = (clocks[-1][1]-clocks[0][1])/(clocks[-1][0]-clocks[0][0])
    folder = Path(__file__).resolve().parents[1]/'log/ghost-obstacles-20260930'
    folder.mkdir(parents=True, exist_ok=True)
    (folder/'live-lidar-check.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
