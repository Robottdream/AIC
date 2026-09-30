"""Summarize speed_probe traces without mixing wall time and simulation time."""
import json
import math
import sys
from pathlib import Path


folder = Path(sys.argv[1])
rows = json.loads((folder / 'results.json').read_text())
trace = [json.loads(line) for line in (folder / 'trace.jsonl').read_text().splitlines()]
for row in rows:
    samples = [s for s in trace if s['task'] == row['task'] and s['type'] == 'sample']
    moving = sorted(s['cmd'][0] for s in samples if s.get('cmd', [0])[0] > 0.05)
    row['command_peak_mps'] = max(moving, default=0)
    row['command_p90_mps'] = moving[int((len(moving) - 1) * 0.9)] if moving else 0
    row['actual_peak_mps'] = max((abs(s['actual_velocity'][0]) for s in samples
                                  if 'actual_velocity' in s), default=0)
    row['max_tilt_deg'] = math.degrees(max((abs(v) for s in samples
                                          for v in s.get('tilt', [])), default=0))
    row['clock_lag_max_s'] = {key: max((s['sim'] - s[key] for s in samples
                                      if key in s), default=None)
                              for key in ('local_clock', 'global_clock')}
    errors = [abs(s['cmd'][0] - s['actual_velocity'][0]) for s in samples
              if 'cmd' in s and 'actual_velocity' in s]
    row['mean_linear_tracking_error_mps'] = sum(errors) / len(errors) if errors else None
    recoveries = 0
    goal_max = 0
    for event in (s for s in trace if s['task'] == row['task']):
        if event.get('topic') == '/manual_nav_target':
            recoveries += goal_max
            goal_max = 0
        if event['type'] == 'sample':
            goal_max = max(goal_max, event.get('feedback', {}).get('recoveries', 0))
    row['recoveries_sum'] = recoveries + goal_max
report = {'tasks': rows, 'wall_total_s': sum(r['seconds'] for r in rows),
          'sim_total_s': sum(r['sim_seconds'] for r in rows),
          'completed': sum(r['status'] == 'success' for r in rows)}
(folder / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(report, ensure_ascii=False, indent=2))
