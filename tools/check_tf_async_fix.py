"""Run the same C++ contention probe against installed and repaired TF libraries."""
import json
import os
import subprocess
import time
from pathlib import Path

root = Path('/home/polarbear/ws_aic/install/aic_tf2_fix/lib')
exe = root/'aic_tf2_fix/tf_async_stress'
library = root/'libaic_tf2_fix.so'
reports = []
for label in ['installed', 'fixed-1', 'fixed-2', 'fixed-3']:
    env = dict(os.environ)
    env.pop('LD_PRELOAD', None)
    if label != 'installed':
        env['LD_PRELOAD'] = str(library)
    start = time.monotonic()
    try:
        result = subprocess.run([str(exe)], env=env, capture_output=True, text=True, timeout=15)
        row = dict(label=label, seconds=time.monotonic()-start, timeout=False,
                   returncode=result.returncode, output=result.stdout[-2000:], stderr=result.stderr[-2000:])
    except subprocess.TimeoutExpired as exc:
        row = dict(label=label, seconds=time.monotonic()-start, timeout=True,
                   output=(exc.stdout or b'').decode(errors='replace')[-2000:])
    reports.append(row)
    print(json.dumps(row), flush=True)
out = Path('/mnt/e/workspace/AIC/log/tf-root-20260930')
out.mkdir(parents=True, exist_ok=True)
(out/'stress-check.json').write_text(json.dumps(reports, indent=2)+'\n')
assert reports[0]['timeout'], 'Installed-library deadlock was not reproduced'
assert all(not r['timeout'] and r['returncode'] == 0 for r in reports[1:]), 'Repair failed contention probe'
