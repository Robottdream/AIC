"""Record TF-derived costmap pose timestamps against simulation time, including idle.

The historical file/output names say clocks, but footprints do not expose the
costmap node's own clock. Missing footprint scopes also count as stale samples.
"""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PolygonStamped


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=240)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('navigation_clock_probe')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    stamps = {}
    for scope in ('local', 'global'):
        node.create_subscription(PolygonStamped, '/'+scope+'_costmap/published_footprint',
            lambda m, k=scope: stamps.update({k:(m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                                                 time.monotonic())}), qos_profile_sensor_data)
    start = time.monotonic()
    rows = []
    last_sim = None
    sim_received = start
    with (args.output/'clocks.jsonl').open('w') as stream:
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            sim = node.get_clock().now().nanoseconds/1e9
            now = time.monotonic()
            if sim != last_sim:
                last_sim = sim
                sim_received = now
            if rows and now-start-rows[-1]['wall'] < .5:
                continue
            row = {'wall':now-start, 'sim':sim, 'clock_wall_age':now-sim_received,
                   'lag':{k:sim-v[0] for k,v in stamps.items()},
                   'receive_age':{k:now-v[1] for k,v in stamps.items()}}
            rows.append(row)
            stream.write(json.dumps(row)+'\n')
    usable = [r for r in rows if r['wall'] > 5]
    summary = {'seconds':time.monotonic()-start, 'samples':len(usable),
               'max_sim_clock_wall_age':max((r['clock_wall_age'] for r in usable), default=None),
               'max_lag':{k:max((r['lag'][k] for r in usable if k in r['lag']), default=None)
                          for k in ('local','global')},
               'stale_samples':sum(len(r['lag']) != 2 or
                                   any(v>1 or v<-.1 for v in r['lag'].values()) or
                                   any(v>1 for v in r['receive_age'].values()) or
                                   r['clock_wall_age']>1 for r in usable)}
    (args.output/'clocks-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
