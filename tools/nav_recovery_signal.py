"""Notify the persistent navigator before stopping/restarting Nav2."""
import sys
import time
import rclpy
from std_msgs.msg import Bool

rclpy.init()
node = rclpy.create_node('navigation_recovery_signal')
pub = node.create_publisher(Bool, '/navigation/recovering', 10)
deadline = time.monotonic()+3
while pub.get_subscription_count() == 0 and time.monotonic()<deadline:
    rclpy.spin_once(node, timeout_sec=.05)
for _ in range(5):
    pub.publish(Bool(data=sys.argv[1] == 'true'))
    rclpy.spin_once(node, timeout_sec=.2)
node.destroy_node()
rclpy.shutdown()
