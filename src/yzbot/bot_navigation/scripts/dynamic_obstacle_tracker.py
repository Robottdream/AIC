"""Small lidar-only tracker and constant-velocity collision forecast.

Points and velocities are expressed in odom, so vehicle motion is compensated
before matching clusters. No Gazebo names/poses enter the decision.
"""
from collections import deque
from dataclasses import dataclass, field
import math


def to_world(point, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * point[0] - s * point[1], y + s * point[0] + c * point[1])


def to_body(point, pose):
    x, y, yaw = pose
    dx, dy = point[0] - x, point[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * dx + s * dy, -s * dx + c * dy)


def clusters(ranges, angle_min, angle_step, range_min, range_max, scan_pose):
    groups, current = [], []
    def finish():
        if len(current) >= 3:
            groups.append(current[:])
        current.clear()
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance <= min(range_max, 4.5):
            finish()
            continue
        angle = angle_min + index * angle_step
        p = (distance * math.cos(angle), distance * math.sin(angle))
        if current and math.dist(p, current[-1]) > 0.20:
            finish()
        current.append(p)
    finish()
    if len(groups) > 1 and math.dist(groups[0][0], groups[-1][-1]) < 0.20:
        groups[0] = groups[-1] + groups[0]
        groups.pop()
    result = []
    for points in groups:
        world = [to_world(p, scan_pose) for p in points]
        span = math.hypot(max(p[0] for p in world) - min(p[0] for p in world),
                          max(p[1] for p in world) - min(p[1] for p in world))
        # Reject long walls and small grasp blocks; partial/occluded objects may
        # be omitted here but remain covered by the existing collision monitor.
        if 0.35 <= span <= 1.30:
            center = tuple(sum(p[i] for p in world) / len(world) for i in (0, 1))
            result.append((center, span, world))
    return result


@dataclass
class Track:
    ident: int
    center: tuple
    span: float
    points: list
    history: deque = field(default_factory=lambda: deque(maxlen=24))
    shapes: deque = field(default_factory=lambda: deque(maxlen=24))
    velocity: tuple = (0.0, 0.0)
    residual: float = float('inf')
    stable: bool = False

    def update(self, stamp, center, span, points):
        self.center, self.span, self.points = center, span, points
        # Use a robust observed boundary anchor for velocity. The centroid of
        # an L-shaped visible surface shifts when one face becomes occluded,
        # even when the object's motion along that axis is zero.
        k = max(0, int((len(points)-1)*0.05))
        anchor = tuple(sorted(p[i] for p in points)[k] for i in (0, 1))
        self.history.append((stamp, *anchor))
        self.shapes.append((stamp, points))
        while len(self.history) > 2 and stamp - self.history[0][0] > 0.8:
            self.history.popleft()
        while len(self.shapes) > 2 and stamp - self.shapes[0][0] > 0.8:
            self.shapes.popleft()
        self.stable = False
        if len(self.history) < 5 or stamp - self.history[0][0] < 0.45:
            return
        n = len(self.history)
        tm = sum(p[0] for p in self.history) / n
        xm = sum(p[1] for p in self.history) / n
        ym = sum(p[2] for p in self.history) / n
        denominator = sum((p[0] - tm)**2 for p in self.history)
        if denominator < 1e-8:
            return
        vx = sum((p[0] - tm) * (p[1] - xm) for p in self.history) / denominator
        vy = sum((p[0] - tm) * (p[2] - ym) for p in self.history) / denominator
        self.residual = math.sqrt(sum((p[1]-xm-vx*(p[0]-tm))**2 +
            (p[2]-ym-vy*(p[0]-tm))**2 for p in self.history) / n)
        self.velocity = (vx, vy)
        self.stable = self.residual < 0.035 and 0.12 <= math.hypot(vx, vy) <= 1.2
        if self.stable:
            old = self.shapes[0][1]
            # A centroid can slide along an unmoving visible face as the car
            # changes viewpoint. Reject that interpretation when either point
            # set still largely lies on the previously observed world surface.
            tolerance = max(0.04, self.span/max(1, min(len(old), len(points))-1))
            forward = sum(min(math.dist(p,q) for q in old) <= tolerance for p in points)/len(points)
            backward = sum(min(math.dist(p,q) for q in points) <= tolerance for p in old)/len(old)
            if max(forward, backward) > 0.8:
                self.stable = False


class Tracker:
    def __init__(self):
        self.tracks = []
        self.next_id = 1
        self.last_stamp = None

    def update(self, stamp, observations):
        if self.last_stamp is not None and (stamp <= self.last_stamp or stamp - self.last_stamp > 0.4):
            self.tracks.clear()
        self.last_stamp = stamp
        previous = self.tracks
        matched = set()
        updated = []
        # Greedy one-to-one matching, ordered by innovation, prevents two
        # detections from contributing to the same velocity estimate.
        candidates = []
        for i, (center, span, _) in enumerate(observations):
            for j, track in enumerate(previous):
                dt = stamp - track.history[-1][0]
                predicted = (track.center[0]+track.velocity[0]*dt,
                             track.center[1]+track.velocity[1]*dt)
                distance = math.dist(center, predicted)
                if distance < 0.25 and abs(span-track.span) < 0.20:
                    candidates.append((distance, i, j))
        associations = {}
        for _, i, j in sorted(candidates):
            if i not in associations and j not in matched:
                associations[i] = previous[j]
                matched.add(j)
        for i, (center, span, points) in enumerate(observations):
            track = associations.get(i)
            if track is None:
                track = Track(self.next_id, center, span, points)
                self.next_id += 1
            track.update(stamp, center, span, points)
            updated.append(track)
        # Drop unseen objects immediately; a stale forecast must not become a
        # persistent imaginary obstacle in the costmap.
        self.tracks = updated

    def collision(self, robot_pose, linear, angular, horizon=1.8, observation_age=0.0):
        moving = [track for track in self.tracks if track.stable]
        if not moving:
            return None
        # Keep the measured point set, not its centroid, to retain the observed
        # object surface. An 8 cm uncertainty margin covers estimation error.
        for step in range(19):
            t = step * horizon / 18
            if abs(angular) < 1e-5:
                relative_pose = (linear*t, 0.0, 0.0)
            else:
                angle = angular*t
                relative_pose = (linear/angular*math.sin(angle),
                                 linear/angular*(1-math.cos(angle)), angle)
            xy = to_world(relative_pose[:2], robot_pose)
            future_pose = (*xy, robot_pose[2]+relative_pose[2])
            for track in moving:
                hits = 0
                for point in track.points:
                    predicted = (point[0]+track.velocity[0]*(t+observation_age),
                                 point[1]+track.velocity[1]*(t+observation_age))
                    x, y = to_body(predicted, future_pose)
                    if -0.34 <= x <= 0.40 and abs(y) <= 0.356:
                        hits += 1
                if hits >= 3:
                    return {'track': track.ident, 'time': round(t, 2),
                            'velocity': [round(v, 3) for v in track.velocity]}
        return None
