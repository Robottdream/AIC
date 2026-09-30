"""Ray-cast regression for ego compensation, closing motion and stale tracks."""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/yzbot/bot_navigation/scripts'))
from dynamic_obstacle_tracker import Tracker, clusters


def scan_box(pose, center):
    ranges = []
    for i in range(360):
        angle = -math.pi + i*2*math.pi/360 + pose[2]
        dx, dy = math.cos(angle), math.sin(angle)
        lower, upper = 0.0, 4.5
        for origin, direction, midpoint in zip(pose[:2], (dx, dy), center):
            if abs(direction) < 1e-9:
                if abs(origin-midpoint) > .375:
                    upper = -1
                    break
            else:
                a = (midpoint-.375-origin)/direction
                b = (midpoint+.375-origin)/direction
                lower, upper = max(lower, min(a,b)), min(upper, max(a,b))
        ranges.append(lower if upper >= lower else float('inf'))
    return clusters(ranges, -math.pi, 2*math.pi/360, .01, 4.5, pose)


def run_case(robot, obstacle):
    tracker = Tracker()
    for i in range(13):
        t = i*.1
        pose = robot(t)
        tracker.update(t, scan_box(pose, obstacle(t)))
    return tracker, pose


def main():
    report = {}
    tracker, pose = run_case(lambda t:(.3*t, -.1*t, .2*t), lambda t:(2., 1.))
    assert not any(t.stable for t in tracker.tracks), 'Static object mistaken for moving after ego compensation'
    report['static_with_vehicle_translation_rotation'] = True
    count = 0
    for vx in (0.0, 0.3, 0.65):
        for vy in (0.0, 0.2):
            for angular in (0.0, 0.5, 1.0):
                for center in ((.9,.6), (1.5,.3), (3.,1.), (-1.,-.5)):
                    candidate, _ = run_case(lambda t:(vx*t,vy*t,angular*t), lambda t:center)
                    assert not any(track.stable for track in candidate.tracks), (vx,vy,angular,center)
                    count += 1
    report['static_viewpoint_cases_without_false_motion'] = count
    tracker, pose = run_case(lambda t:(0.,0.,0.), lambda t:(1.8-.3*t,0.))
    moving = [t for t in tracker.tracks if t.stable]
    assert moving and abs(moving[0].velocity[0]+.3)<.10, 'Head-on velocity inaccurate'
    assert tracker.collision(pose, .6, 0.) is not None, 'Head-on collision missed'
    report['head_on_detected'] = True
    tracker, pose = run_case(lambda t:(0.,0.,0.), lambda t:(1.1,.9-.5*t))
    assert tracker.collision(pose, .6, 0.) is not None, 'Crossing collision missed'
    report['crossing_detected'] = True
    tracker, pose = run_case(lambda t:(0.,0.,0.), lambda t:(1.1,.9+.5*t))
    assert any(t.stable for t in tracker.tracks), 'Receding object not tracked'
    assert tracker.collision(pose, .6, 0.) is None, 'Receding object blocks route'
    report['receding_does_not_stop'] = True
    tracker.update(1.3, [])
    assert not tracker.tracks, 'Occluded observations left stale prediction'
    report['lost_observation_expires'] = True
    tracker.update(.1, scan_box((0.,0.,0.), (1.,0.)))
    assert not any(t.stable for t in tracker.tracks), 'Time reset kept a velocity estimate'
    report['clock_reset_discards_velocity'] = True
    out = ROOT/'log/dynamic-prediction-20260930'
    out.mkdir(parents=True, exist_ok=True)
    (out/'tracker-check.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
