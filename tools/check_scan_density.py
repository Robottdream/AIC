"""Read actual Gazebo scan geometry and rate without moving the robot."""
import json
from pathlib import Path
import time
import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('scan_density_check')
    samples = []
    node.create_subscription(LaserScan, '/scan',
        lambda msg: samples.append((time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                                   len(msg.ranges), msg.angle_increment)), qos_profile_sensor_data)
    deadline = time.monotonic() + 15
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
        assert len(samples) > 10, 'Not enough actual scans received'
        count = len(samples) - 1
        report = {'scan_count': len(samples), 'beam_counts': sorted(set(s[2] for s in samples)),
                  'sim_hz': count / (samples[-1][1] - samples[0][1]),
                  'wall_hz': count / (samples[-1][0] - samples[0][0]),
                  'angle_increment_rad': samples[-1][3]}
        assert report['beam_counts'] == [1441], report
        folder = Path(__file__).resolve().parents[1] / 'log/ghost-obstacles-20261005'
        folder.mkdir(parents=True, exist_ok=True)
        (folder / 'live-scan.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
