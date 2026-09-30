"""Verify cross-process delivery and late-joining reliable history before launch."""
import json
import subprocess
import sys
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node = rclpy.create_node('aic_transport_sub' if '--subscriber' in sys.argv else 'aic_transport_pub')
    if '--subscriber' in sys.argv:
        received = []
        scans = []
        scan_times = []
        small_scans = []
        live = []
        node.create_subscription(String, '/aic_transport_check', lambda m:received.append(m.data), qos)
        node.create_subscription(LaserScan, '/aic_transport_scan',
            lambda m:(scans.append(len(m.ranges)), scan_times.append(time.monotonic())), qos_profile_sensor_data)
        node.create_subscription(LaserScan, '/aic_transport_small_scan', lambda m:small_scans.append(len(m.ranges)), qos_profile_sensor_data)
        node.create_subscription(String, '/aic_transport_live', lambda m:live.append(m.data), qos_profile_sensor_data)
        deadline = time.monotonic()+15
        while (not received or len(scans)<5 or len(small_scans)<5 or len(live)<5) and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        success = received == ['retained-before-join'] and len(scans)>=5 and scans[-1] == 360 and len(live)>=5 and len(small_scans)>=5
        print(json.dumps({'retained_cross_process_delivery':received == ['retained-before-join'],
                          'live_scan_cross_process_delivery':bool(scans) and scans[-1] == 360}), flush=True)
        print(json.dumps({'live_best_effort_string':bool(live), 'scan_counts':scans[:3],
                          'small_scan_counts':small_scans[:3], 'scan_samples':len(scans),
                          'scan_hz':(len(scans)-1)/(scan_times[-1]-scan_times[0]) if len(scans)>1 else None}), flush=True)
    else:
        pub = node.create_publisher(String, '/aic_transport_check', qos)
        scan_pub = node.create_publisher(LaserScan, '/aic_transport_scan', qos_profile_sensor_data)
        small_scan_pub = node.create_publisher(LaserScan, '/aic_transport_small_scan', qos_profile_sensor_data)
        live_pub = node.create_publisher(String, '/aic_transport_live', qos_profile_sensor_data)
        pub.publish(String(data='retained-before-join'))
        end = time.monotonic()+1
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.1)
        child = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--subscriber'],
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        deadline = time.monotonic()+20
        while child.poll() is None and time.monotonic()<deadline:
            scan = LaserScan()
            scan.ranges = [2.0]*360
            scan.intensities = [0.0]*360
            scan_pub.publish(scan)
            scan.ranges = [2.0]
            scan.intensities = [0.0]
            small_scan_pub.publish(scan)
            live_pub.publish(String(data='live'))
            rclpy.spin_once(node, timeout_sec=.1)
        if child.poll() is None:
            child.kill()
        output, errors = child.communicate()
        print(output, end='')
        if errors:
            print(errors, file=sys.stderr, end='')
        success = child.returncode == 0
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
