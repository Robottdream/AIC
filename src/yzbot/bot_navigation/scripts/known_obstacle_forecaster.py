#!/usr/bin/env python3
"""Scene-specific route prior, synchronized with Gazebo obstacle phase.

Unlike lidar tracking this intentionally knows the competition world's routes.
Current poses synchronize the phase; no route has to be learned from scans.
"""
import json
import math
import time
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException

ROUTES = {
    'obstacle2': {'id': 2, 'start': (-9.0, -4.2), 'end': (-5.0, -3.3), 'speed': .35},
    'obstacle3': {'id': 3, 'start': (-2.5, 3.0), 'end': (-2.5, -2.0), 'speed': .35},
}
RADIUS = math.hypot(.375, .375)

def forecast_route(route, position, velocity, direction, horizon=4.0):
    """Mirror the plugin's target switch at 0.2m and finite acceleration."""
    x, y = position
    vx, vy = velocity
    ax, ay = route['start']; bx, by = route['end']
    dx, dy = bx-ax, by-ay
    length2 = dx*dx+dy*dy
    samples = [{'t': 0.0, 'center': [x, y]}]
    for i in range(1, round(horizon/.1)+1):
        tx, ty = (bx, by) if direction > 0 else (ax, ay)
        if math.hypot(tx-x, ty-y) < .2:
            direction *= -1
            tx, ty = (bx, by) if direction > 0 else (ax, ay)
        distance = math.hypot(tx-x, ty-y)
        fraction = max(0.0, min(1.0, ((x-ax)*dx+(y-ay)*dy)/length2))
        nx, ny = ax+fraction*dx, ay+fraction*dy
        speed = min(route['speed'], 2.0*distance)
        desired = ((tx-x)/max(distance, 1e-8)*speed + max(-.25, min(.25, (nx-x)*.8)),
                   (ty-y)/max(distance, 1e-8)*speed + max(-.25, min(.25, (ny-y)*.8)))
        delta = math.hypot(desired[0]-vx, desired[1]-vy)
        factor = min(1.0, .5/max(delta, 1e-8))  # force1.5N / mass0.3kg, dt=.1
        vx += (desired[0]-vx)*factor
        vy += (desired[1]-vy)*factor
        x += vx*.1; y += vy*.1
        samples.append({'t': round(i*.1, 3), 'center': [x, y]})
    return samples

class KnownObstacleForecaster(Node):
    def __init__(self):
        super().__init__('known_obstacle_forecaster')
        self.states = {}
        self.history = {name: deque(maxlen=12) for name in ROUTES}
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.local = self.create_publisher(String, '/navigation/predicted_obstacles', 1)
        self.global_pub = self.create_publisher(String, '/navigation/known_scene_predictions', 1)
        self.markers = self.create_publisher(MarkerArray, '/navigation/obstacle_predictions', 1)
        self.status = self.create_publisher(String, '/navigation/known_prediction', 10)
        self.subs = [self.create_subscription(Pose, '/'+name+'/current_pose',
            lambda msg, key=name: self.observe(key, msg),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)) for name in ROUTES]
        self.create_timer(.1, self.publish)

    def observe(self, name, msg):
        stamp = self.get_clock().now().nanoseconds/1e9
        xy = (msg.position.x, msg.position.y)
        if not all(math.isfinite(v) for v in xy):
            return
        history = self.history[name]
        if history and stamp < history[-1][0]:
            history.clear()
        if history and stamp-history[-1][0] < .04:
            return
        history.append((stamp, xy))
        while len(history) > 2 and stamp-history[0][0] > .3:
            history.popleft()
        velocity = (0.0, 0.0)
        if len(history) >= 2 and stamp-history[0][0] > .05:
            dt = stamp-history[0][0]
            velocity = tuple((xy[i]-history[0][1][i])/dt for i in (0, 1))
        if math.hypot(*velocity) > 1.2:
            history.clear()  # teleport/reset is not a velocity estimate
            self.states.pop(name, None)
            return
        route = ROUTES[name]
        direction = self.states.get(name, {}).get('direction', 1)
        projection = sum(velocity[i]*(route['end'][i]-route['start'][i]) for i in (0, 1))
        if abs(projection) > .03:
            direction = 1 if projection > 0 else -1
        if math.dist(xy, route['end']) < .2:
            direction = -1
        elif math.dist(xy, route['start']) < .2:
            direction = 1
        self.states[name] = dict(position=xy, velocity=velocity, direction=direction,
                                 stamp=stamp, wall=time.monotonic())

    def publish(self):
        now = self.get_clock().now()
        stamp = now.nanoseconds/1e9
        tracks = []
        for name, state in self.states.items():
            if not 0 <= stamp-state['stamp'] < .35 or time.monotonic()-state['wall'] > .6:
                continue
            # Shift the samples to the publication time, retaining the route turns.
            age = stamp-state['stamp']
            samples = forecast_route(ROUTES[name],
                tuple(state['position'][i]+state['velocity'][i]*age for i in (0, 1)),
                state['velocity'], state['direction'])
            tracks.append(dict(id=ROUTES[name]['id'], name=name, center=samples[0]['center'],
                velocity=list(state['velocity']), radius=RADIUS, samples=samples))
        payload = dict(version=1, enabled=True, source='known_scene_routes',
                       frame='map', stamp=stamp, tracks=tracks)
        self.global_pub.publish(String(data=json.dumps(payload)))
        local_tracks = []
        frame_error = None
        try:
            transform = self.buffer.lookup_transform('odom', 'map', rclpy.time.Time()).transform
            q = transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            def convert(p):
                return [transform.translation.x+c*p[0]-s*p[1],
                        transform.translation.y+s*p[0]+c*p[1]]
            for track in tracks:
                vx, vy = track['velocity']
                local_tracks.append(dict(track, center=convert(track['center']),
                    velocity=[c*vx-s*vy, s*vx+c*vy],
                    samples=[dict(p, center=convert(p['center'])) for p in track['samples']]))
        except TransformException as error:
            frame_error = str(error)
        self.local.publish(String(data=json.dumps(dict(payload, frame='odom',
            enabled=frame_error is None, tracks=local_tracks))))
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = 'map'; clear.header.stamp = now.to_msg()
        markers = [clear]
        for track in tracks:
            line = Marker()
            line.header = clear.header
            line.ns = 'known_route_forecast'; line.id = track['id']
            line.type = Marker.LINE_STRIP; line.action = Marker.ADD
            line.pose.orientation.w = 1.0; line.scale.x = .035
            line.color.r, line.color.g, line.color.a = 1.0, .45, 1.0
            line.lifetime.nanosec = 400000000
            line.points = [Point(x=p['center'][0], y=p['center'][1], z=.12) for p in track['samples']]
            markers.append(line)
        self.markers.publish(MarkerArray(markers=markers))
        self.status.publish(String(data=json.dumps(dict(source=payload['source'], stamp=stamp,
            tracks=len(tracks), local_tracks=len(local_tracks), frame_error=frame_error))))

def main():
    rclpy.init(); node = KnownObstacleForecaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
