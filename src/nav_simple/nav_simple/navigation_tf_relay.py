"""Relay navigation's planar TF, preserving source timestamps exactly.

Enabled only for the simulation's static map/odom alignment. No timestamp
rewriting or periodic replay: upstream staleness remains visible.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage


class NavigationTfRelay(Node):
    def __init__(self):
        super().__init__('navigation_tf_relay')
        self.fixed = {}
        dynamic = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        fixed = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.dynamic_pub = self.create_publisher(TFMessage, '/navigation/tf', dynamic)
        self.fixed_pub = self.create_publisher(TFMessage, '/navigation/tf_static', fixed)
        self.create_subscription(TFMessage, '/tf', self.on_dynamic, dynamic)
        self.create_subscription(TFMessage, '/tf_static', self.on_fixed, fixed)

    def on_dynamic(self, message):
        root = [t for t in message.transforms
                if t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint']
        if root:
            self.dynamic_pub.publish(TFMessage(transforms=root))

    def on_fixed(self, message):
        for transform in message.transforms:
            self.fixed[transform.child_frame_id] = transform
        self.fixed_pub.publish(TFMessage(transforms=list(self.fixed.values())))


def main(args=None):
    rclpy.init(args=args)
    node = NavigationTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
