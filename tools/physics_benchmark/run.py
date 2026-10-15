#!/usr/bin/env python3
"""Repeat isolated Gazebo motor/physics comparisons; never edit production settings."""
import argparse
import hashlib
import fcntl
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--windows-output', type=Path, help='Copy final report/figures to a Windows-mounted directory such as /mnt/c/Users/.../Desktop/results')
    parser.add_argument('--ode-values', type=int, nargs='+', default=[100, 200])
    parser.add_argument('--pace-realtime', action=argparse.BooleanOptionalAction, default=True, help='Set update rate to 1/step to keep nominal real-time factor 1 across step sizes')
    parser.add_argument('--suite', choices=['ode', 'numerical'], default='ode')
    parser.add_argument('--variants', nargs='+', choices=['baseline', 'step-half', 'step-quarter', 'step-double', 'step-quadruple', 'world', 'finite'], default=['baseline', 'step-half', 'step-quarter', 'world', 'finite'])
    parser.add_argument('--repeats', type=int, default=2, help='Cold starts per ODE value; alternate forward/reverse order')
    parser.add_argument('--speed', type=float, default=3.0)
    parser.add_argument('--omega', type=float, default=2.0)
    parser.add_argument('--hold', type=float, default=3.2)
    parser.add_argument('--brake', type=float, default=2.2)
    parser.add_argument('--expected-torque', type=float, default=1000.0)
    parser.add_argument('--expected-mu', type=float, default=1.0)
    parser.add_argument('--nav-params', type=Path, default=ROOT / 'scenarios/short_routes_20261007/nav2.yaml')
    parser.add_argument('--tasks', type=Path, default=ROOT / 'scenarios/short_routes_20261007/tasks.yaml')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--prepare-only', action='store_true')
    mode.add_argument('--analyze-only', action='store_true')
    mode.add_argument('--resume', action='store_true', help='Continue an existing protocol, preserving completed and failed boot records')
    args = parser.parse_args()
    if not args.analyze_only:
        if args.repeats < 1 or any(v <= 0 for v in args.ode_values):
            parser.error('ODE values and repeats must be positive')
        if len(set(args.ode_values)) != len(args.ode_values):
            parser.error('Use --repeats rather than duplicate --ode-values')
        if not all(math.isfinite(v) for v in [args.speed, args.omega, args.hold, args.brake, args.expected_torque, args.expected_mu]):
            parser.error('Parameters must be finite')
        if not (0 < args.speed <= 3.5 and abs(args.omega) <= 4):
            parser.error('This field/probe supports 0 < speed <= 3.5 and |omega| <= 4')
        if not (max(args.speed / 3.5, abs(args.omega) / 10) + .6 <= args.hold <= 4):
            parser.error('Hold must leave 0.6 s after acceleration and be <= 4 s')
        if args.brake < 1.5 or args.expected_torque <= 0 or args.expected_mu <= 0:
            parser.error('Brake observation must be >=1.5 s; expected torque/mu must be positive')
    args.output = args.output.expanduser().resolve()
    return args


def make_map(out, world):
    """Rasterize static boxes at the same coordinates as the empty-field fixture."""
    import yaml
    resolution = .05
    width, height = 1400, 320
    data = bytearray([254]) * (width * height)
    for model in world.findall('model'):
        if model.findtext('static', '').lower() not in ('true', '1'):
            continue
        size = model.findtext('link/collision/geometry/box/size')
        if size is None:
            continue
        x, y, *_ = map(float, model.findtext('pose', '0 0 0 0 0 0').split())
        dx, dy, _ = map(float, size.split())
        x0 = max(0, int(math.floor((x - dx / 2 + 35) / resolution)))
        x1 = min(width, int(math.ceil((x + dx / 2 + 35) / resolution)))
        y0 = max(0, int(math.floor((y - dy / 2 + 8) / resolution)))
        y1 = min(height, int(math.ceil((y + dy / 2 + 8) / resolution)))
        for iy in range(y0, y1):
            offset = (height - 1 - iy) * width
            data[offset + x0:offset + x1] = bytes(x1 - x0)
    (out / 'map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode() + data)
    (out / 'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=resolution, origin=[-35., -8., 0.], negate=0, occupied_thresh=.65, free_thresh=.196)))


def prepare(args):
    import yaml
    out = args.output
    if out.exists():
        raise RuntimeError('Output already exists; choose a fresh directory. Use --analyze-only to redraw.')
    out.mkdir(parents=True)
    definitions = {
        'baseline': dict(ode=200, solver='quick', step=.001, finite=False),
        'step-half': dict(ode=200, solver='quick', step=.0005, finite=False),
        'step-quarter': dict(ode=200, solver='quick', step=.00025, finite=False),
        'step-double': dict(ode=200, solver='quick', step=.002, finite=False),
        'step-quadruple': dict(ode=200, solver='quick', step=.004, finite=False),
        'world': dict(ode=200, solver='world', step=.001, finite=False),
        'finite': dict(ode=200, solver='quick', step=.001, finite=True),
    }
    values = args.variants if args.suite == 'numerical' else args.ode_values
    if len(set(values)) != len(values):
        raise ValueError('Duplicate variants; use --repeats')
    profiles = definitions if args.suite == 'numerical' else {str(v): dict(ode=v, solver='quick', step=.001, finite=False) for v in values}
    for config in profiles.values():
        config['update_rate'] = 1. / config['step'] if args.pace_realtime else 1000.
    order = []
    for repeat in range(args.repeats):
        order.extend(values if repeat % 2 == 0 else reversed(values))
    protocol = dict(paced_realtime=args.pace_realtime, suite=args.suite, profiles=profiles, order=order, target_v=args.speed, target_w=args.omega,
                    hold_sim_seconds=args.hold, brake_observation_seconds=args.brake,
                    expected_torque=args.expected_torque, expected_mu=args.expected_mu,
                    trials_per_boot=['straight', 'arc-w2'],
                    method='Counterbalanced cold starts; command through active velocity smoother; no Nav2 path action')
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2))
    nav = yaml.safe_load(args.nav_params.read_text())
    follow = nav['controller_server']['ros__parameters']['FollowPath']
    follow['desired_linear_vel'] = args.speed
    smoother = nav['velocity_smoother']['ros__parameters']
    smoother['max_velocity'] = [args.speed, 0., 4.]
    smoother['max_accel'] = [3.5, 0., 10.]
    smoother['max_decel'] = [-5., 0., -10.]
    nav['planner_server']['ros__parameters']['GridBased']['nominal_speed'] = args.speed
    tasks = yaml.safe_load(args.tasks.read_text())
    # Motion commands bypass task navigation; task inputs are still archived for provenance.
    normalized = []
    for i, key in enumerate(order, 1):
        config = profiles[str(key)]
        limit = config['ode']
        folder = out / boot_name(i, key, args.suite)
        folder.mkdir()
        (folder / 'nav2.yaml').write_text(yaml.safe_dump(nav, sort_keys=False))
        (folder / 'tasks.yaml').write_text(yaml.safe_dump(tasks, sort_keys=False))
        tree = ET.parse(HERE / 'empty-field.world')
        world = tree.find('world')
        world.find('physics/ode/constraints/contact_max_correcting_vel').text = str(limit)
        world.find('physics/ode/solver/type').text = config['solver']
        world.find('physics/max_step_size').text = str(config['step'])
        world.find('physics/real_time_update_rate').text = str(config['update_rate'])
        plugin = ET.SubElement(world, 'plugin', name='speed_audit_native', filename=str(out / 'libnative_probe.so'))
        ET.SubElement(plugin, 'output_file').text = str(folder / 'native.jsonl')
        ET.SubElement(plugin, 'finite_rotation').text = str(config['finite']).lower()
        tree.write(folder / 'field.world', encoding='utf-8', xml_declaration=True)
        world.find('physics/ode/constraints/contact_max_correcting_vel').text = 'NORMALIZED'
        world.find('physics/ode/solver/type').text = 'NORMALIZED'
        world.find('physics/max_step_size').text = 'NORMALIZED'
        world.find('physics/real_time_update_rate').text = 'NORMALIZED'
        plugin.find('finite_rotation').text = 'NORMALIZED'
        plugin.find('output_file').text = 'NORMALIZED'
        normalized.append(ET.tostring(tree.getroot()))
    if len(set(normalized)) != 1:
        raise RuntimeError('Worlds differ beyond intended suite parameters/output path')
    make_map(out, ET.parse(HERE / 'empty-field.world').find('world'))
    sources = list(HERE.glob('*.py')) + [HERE / 'native_probe.cpp', HERE / 'empty-field.world', args.nav_params, args.tasks]
    (out / 'input-provenance.json').write_text(json.dumps({str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}, indent=2))
    print(f'Prepared {len(order)} cold starts / {2 * len(order)} trials in {out}', flush=True)
    return order


def boot_name(index, key, suite):
    return f'boot-{index}-{key}' if suite == 'numerical' else f'boot-{index}-ode-{key}'


def analyze(args):
    protocol = json.loads((args.output / 'protocol.json').read_text())
    script = 'analyze_numerical.py' if protocol.get('suite') == 'numerical' else 'analyze.py'
    subprocess.run([sys.executable, str(HERE / script), '--output', str(args.output)], check=True)
    if args.windows_output:
        export_results(args)



def export_results(args):
    destination = args.windows_output.expanduser().resolve()
    if destination == args.output:
        raise ValueError('Export directory must differ from raw-data output')
    destination.mkdir(parents=True, exist_ok=True)
    copied = {}
    names = ['REPORT.md', 'INTERPRETATION.md', 'response-comparison.png',
             'geometry-comparison.png', 'trajectories.png', 'individual-metrics.csv',
             'metrics.json', 'protocol.json', 'verification.json', 'failures.json',
             'native-runtime.json']
    for name in names:
        source = args.output / name
        if not source.exists():
            continue
        target = destination / name
        shutil.copy2(source, target)
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        if digest != hashlib.sha256(target.read_bytes()).hexdigest():
            raise RuntimeError(f'Export checksum mismatch: {target}')
        copied[name] = digest
    shutil.copy2(HERE / 'README.md', destination / '脚本使用说明.md')
    (destination / 'export-checksums.json').write_text(json.dumps(copied, indent=2))
    print(f'EXPORTED {len(copied)} verified report files to {destination}', flush=True)


def active_stack():
    state = ROOT / 'log/run/processes.json'
    if state.exists() and json.loads(state.read_text()):
        return True
    return subprocess.run(['pgrep', '-x', 'gzserver'], stdout=subprocess.DEVNULL).returncode == 0


def archive_interrupted_attempt(folder):
    """Preserve partial observations before a resumed boot starts logging again."""
    protected = {'field.world', 'nav2.yaml', 'tasks.yaml', 'interrupted-attempts'}
    existing = [path for path in folder.iterdir() if path.name not in protected]
    if not existing:
        return
    archive = folder / 'interrupted-attempts' / str(time.time_ns())
    archive.mkdir(parents=True)
    for path in existing:
        shutil.move(str(path), str(archive / path.name))
    (archive / 'archive-reason.json').write_text(json.dumps(
        dict(reason='Incomplete boot preserved before resume', original_folder=str(folder)), indent=2))
    print(f'ARCHIVED incomplete observations to {archive}', flush=True)

def run_trials(args, order):
    if active_stack():
        raise RuntimeError('A simulation is active. Stop it explicitly before running this isolated test.')
    if not args.resume:
        flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gazebo'], text=True))
        subprocess.run(['g++', '-shared', '-fPIC', '-std=c++17', str(HERE / 'native_probe.cpp'), '-o', str(args.output / 'libnative_probe.so'), *flags], check=True)
    elif not (args.output / 'libnative_probe.so').exists():
        raise RuntimeError('Cannot resume without the original native probe binary')
    env = os.environ.copy()
    env.update(PYTHONNOUSERSITE='1', AIC_MAP=str(args.output / 'map.yaml'))
    env.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', str(ROOT / 'tools/fastdds_wsl_udp.xml'))

    def launch(action, name):
        with (args.output / name).open('w') as log:
            return subprocess.run(['bash', str(ROOT / 'ros_competition_start.sh'), action, '--headless'], cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=240)

    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'Interrupted by signal {signum}')

    previous = signal.signal(signal.SIGTERM, interrupted)
    try:
        profiles = json.loads((args.output / 'protocol.json').read_text())['profiles']
        for i, key in enumerate(order, 1):
            config = profiles[str(key)]
            limit = config['ode']
            name = boot_name(i, key, args.suite)
            folder = args.output / name
            env.update(AIC_WORLD=str(folder / 'field.world'), AIC_NAV_PARAMS=str(folder / 'nav2.yaml'), AIC_TASK_CONFIG=str(folder / 'tasks.yaml'))
            prior_records = folder / 'trials.json'
            if args.resume and ((folder / 'error.json').exists() or (folder / 'runner-error.json').exists() or (prior_records.exists() and len(json.loads(prior_records.read_text())) == 2)):
                print(f'PRESERVED {name}', flush=True)
                continue
            if args.resume:
                archive_interrupted_attempt(folder)
            try:
                print(f'BEGIN {name}', flush=True)
                if launch('start', name + '-start.log').returncode:
                    raise RuntimeError(f'Start failed; inspect {name}-start.log')
                if subprocess.run(['pgrep', '-x', 'gzclient'], stdout=subprocess.DEVNULL).returncode == 0:
                    raise RuntimeError('Expected a headless stack')
                native = folder / 'native.jsonl'
                deadline = time.monotonic() + 15
                while (not native.exists() or not native.stat().st_size) and time.monotonic() < deadline:
                    time.sleep(.2)
                actual = json.loads(native.open().readline())
                if (actual['actual_contact_max_correcting_vel'] != limit
                        or actual['solver'] != config['solver']
                        or abs(actual['max_step_size'] - config['step']) > 1e-10
                        or actual['left_finite_mode'] != int(config['finite'])
                        or actual['right_finite_mode'] != int(config['finite'])
                        or ('update_rate' in config and abs(actual['real_time_update_rate'] - config['update_rate']) > 1e-6)):
                    raise RuntimeError(f'Runtime physics mismatch: {actual}')
                print(f'RUNTIME {actual}', flush=True)
                with (args.output / (name + '-probe.log')).open('w') as log:
                    subprocess.run([sys.executable, str(HERE / 'probe.py'), '--folder', str(folder), '--speed', str(args.speed), '--omega', str(args.omega), '--hold', str(args.hold), '--brake', str(args.brake), '--expected-torque', str(args.expected_torque), '--expected-mu', str(args.expected_mu)], env=env, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=180, check=True)
                (folder / 'runner-status.json').write_text(json.dumps(dict(status='completed'), indent=2))
            except Exception as exc:
                (folder / 'runner-error.json').write_text(json.dumps(dict(error=repr(exc)), indent=2))
                print(f'FAILED {name}: {exc}; continuing after shutdown', flush=True)
            finally:
                if not (folder / 'stack-logs').exists():
                    shutil.copytree(ROOT / 'log/run', folder / 'stack-logs')
                launch('stop', name + '-stop.log').check_returncode()
            print(f'END {name}', flush=True)

    finally:
        launch('stop', 'all-final-stop.log').check_returncode()
        signal.signal(signal.SIGTERM, previous)
        if active_stack():
            raise RuntimeError('Stack still active after shutdown; inspect all-final-stop.log')


def main():
    args = parse_args()
    if sys.platform != 'linux' or not Path('/opt/ros/humble/setup.bash').exists():
        raise RuntimeError('Run in WSL bash with ROS 2 Humble sourced')
    if args.analyze_only:
        analyze(args)
        return
    if not args.prepare_only and active_stack():
        raise RuntimeError('A stack is already active; no processes have been stopped or changed')
    if args.prepare_only:
        prepare(args)
        return
    # Serialize this tool before configuration/build/start; do not race another runner.
    lock_path = ROOT / 'log/physics-benchmark.lock'
    lock_path.parent.mkdir(exist_ok=True)
    with lock_path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('Another physics benchmark owns this workspace') from exc
        if args.resume:
            protocol = json.loads((args.output / 'protocol.json').read_text())
            args.suite = protocol.get('suite', 'ode')
            args.speed, args.omega = protocol['target_v'], protocol['target_w']
            args.hold, args.brake = protocol['hold_sim_seconds'], protocol['brake_observation_seconds']
            args.expected_torque = protocol.get('expected_torque', 1000.)
            args.expected_mu = protocol.get('expected_mu', 1.)
            order = protocol['order']
            pending = False
            for i, key in enumerate(order, 1):
                folder = args.output / boot_name(i, key, args.suite)
                records = folder / 'trials.json'
                terminal = (folder / 'error.json').exists() or (folder / 'runner-error.json').exists() or (records.exists() and len(json.loads(records.read_text())) == 2)
                pending = pending or not terminal
            if pending:
                provenance = json.loads((args.output / 'input-provenance.json').read_text())
                if provenance[str(HERE / 'native_probe.cpp')] != hashlib.sha256((HERE / 'native_probe.cpp').read_bytes()).hexdigest():
                    raise RuntimeError('Native probe changed since the original run; use a fresh output')
            else:
                print('ALL BOOTS PRESERVED; redrawing existing data', flush=True)
                analyze(args)
                return
        else:
            order = prepare(args)
        run_trials(args, order)
        analyze(args)


if __name__ == '__main__':
    main()