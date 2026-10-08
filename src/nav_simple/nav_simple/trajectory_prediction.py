"""Short-term crossing estimates for measured motion on known 2-D tracks."""
import math


def predict_position(track, samples, now, seconds):
    if len(samples) < 2 or now - samples[-1][0] > 0.5:
        return None
    sx, sy, ex, ey = track['points']
    length = math.hypot(ex-sx, ey-sy)
    ux, uy = (ex-sx)/length, (ey-sy)/length
    latest = samples[-1]
    old = next((p for p in reversed(samples) if latest[0]-p[0] >= .35), samples[0])
    dt = latest[0]-old[0]
    if dt < .15:
        return None
    velocity = ((latest[1]-old[1])*ux + (latest[2]-old[2])*uy)/dt
    if abs(velocity) < .02:
        return None
    velocity = math.copysign(min(track['speed'], max(.12, abs(velocity))), velocity)
    # The Gazebo plugin reverses within 0.2 m of each endpoint.
    lower, upper = .18, length-.18
    along = min(upper, max(lower, (latest[1]-sx)*ux+(latest[2]-sy)*uy))
    span = upper-lower
    phase = (along-lower + velocity*(max(0., now-latest[0])+seconds)) % (2*span)
    along = lower + (phase if phase <= span else 2*span-phase)
    return sx+along*ux, sy+along*uy


def crossing_conflicts(points, tracks, histories, now, delay, speed, acceleration):
    """Return conflicts on the first six seconds of a planned path.

    Timing uncertainty covers curved-path slowdown. This is an anticipatory
    start gate; live laser/costmap collision checks still govern motion.
    """
    conflicts = set()
    distance = 0.
    ramp_distance = speed*speed/(2*acceleration)
    for before, after in zip(points, points[1:]):
        dx, dy = after[0]-before[0], after[1]-before[1]
        segment = math.hypot(dx, dy)
        count = max(1, math.ceil(segment/.15))
        for i in range(count+1):
            fraction = i/count
            travelled = distance+fraction*segment
            arrival = (math.sqrt(2*travelled/acceleration) if travelled <= ramp_distance
                       else speed/acceleration+(travelled-ramp_distance)/speed)
            if arrival > 6.:
                return conflicts
            x, y = before[0]+fraction*dx, before[1]+fraction*dy
            uncertainty = .65+.12*arrival
            for track in tracks:
                sx, sy, ex, ey = track['points']
                length_squared = (ex-sx)**2+(ey-sy)**2
                along = max(0., min(1., ((x-sx)*(ex-sx)+(y-sy)*(ey-sy))/length_squared))
                radius = math.hypot(.26, .31)+track['size']/math.sqrt(2)+.20
                if math.hypot(x-sx-along*(ex-sx), y-sy-along*(ey-sy)) >= radius:
                    continue
                samples = histories[track['name']]
                for tick in range(9):
                    offset = uncertainty*(tick/4.-1.)
                    predicted = predict_position(track, samples, now, max(0., delay+arrival+offset))
                    if predicted is not None and math.hypot(x-predicted[0], y-predicted[1]) < radius:
                        conflicts.add(track['name'])
                        break
        distance += segment
    return conflicts
