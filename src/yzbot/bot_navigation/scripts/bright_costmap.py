#!/usr/bin/env python3
"""Publish bright RViz-only map copies without exterior scan artifacts.

Nav2 continues to use /map and /global_costmap/costmap unchanged. The visual
copies paint unknown cells white and suppress cost values beyond the static
map's outer boundary, while retaining a narrow halo around the outer walls.
"""

from array import array
from copy import copy

import numpy as np
from PIL import Image, ImageDraw, ImageFilter
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
WALL_HALO_METERS = 0.65


def convex_hull(points):
    """Return the convex outer boundary of occupied static-map cells."""
    points = sorted(points)
    if len(points) < 3:
        return points

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    lower = []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


class BrightCostmap(Node):
    def __init__(self):
        super().__init__('bright_costmap')
        self.interior_mask = None
        self.map_geometry = None
        self.map_pub = self.create_publisher(OccupancyGrid, '/visualization/map', MAP_QOS)
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, '/visualization/global_costmap', MAP_QOS
        )
        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_QOS)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.on_costmap, MAP_QOS
        )

    @staticmethod
    def brighten(message):
        result = copy(message)
        # OccupancyGrid encodes unknown as signed -1 (byte 0xff).
        result.data = array('b', bytes(message.data).replace(b'\xff', b'\x00'))
        return result

    def on_map(self, message):
        width, height = message.info.width, message.info.height
        cells = np.frombuffer(bytes(message.data), dtype=np.int8).reshape(height, width)
        ys, xs = np.nonzero(cells == 100)
        hull = convex_hull(zip(xs.tolist(), ys.tolist()))
        if len(hull) >= 3:
            mask_image = Image.new('L', (width, height), 0)
            ImageDraw.Draw(mask_image).polygon(hull, fill=255)
            halo_cells = max(1, round(WALL_HALO_METERS / message.info.resolution))
            mask_image = mask_image.filter(ImageFilter.MaxFilter(2 * halo_cells + 1))
            self.interior_mask = np.asarray(mask_image, dtype=bool).ravel()
            self.map_geometry = (
                width, height, message.info.resolution,
                message.info.origin.position.x, message.info.origin.position.y,
            )
        else:
            self.get_logger().warning('Static map has no outer wall boundary')
        self.map_pub.publish(self.brighten(message))

    def on_costmap(self, message):
        geometry = (
            message.info.width, message.info.height, message.info.resolution,
            message.info.origin.position.x, message.info.origin.position.y,
        )
        if self.interior_mask is None or geometry != self.map_geometry:
            return
        cells = np.frombuffer(bytes(message.data), dtype=np.uint8).copy()
        cells[cells == 255] = 0  # unknown -> white in the RViz costmap palette
        cells[~self.interior_mask] = 0  # exterior is display-only white
        result = copy(message)
        result.data = array('b', cells.tobytes())
        self.costmap_pub.publish(result)


def main():
    rclpy.init()
    node = BrightCostmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
