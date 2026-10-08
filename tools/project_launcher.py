"""Persistent competition launcher; only owns process groups it starts."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'log' / 'run'
STATE = RUN / 'processes.json'


def alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def load():
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def group_alive(pid):
    try:
        os.killpg(pid, 0)
        return True
    except ProcessLookupError:
        return False


def save(state):
    STATE.write_text(json.dumps(state, indent=2))


def stop():
    state = load()
    for sig, delay in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
        for pid in reversed(list(state.values())):
            if group_alive(pid):
                try:
                    os.killpg(pid, sig)
                except ProcessLookupError:
                    pass
        if any(group_alive(pid) for pid in state.values()):
            time.sleep(delay)
    save({})


def wait(check, label, state, limit=90):
    deadline = time.monotonic() + limit
    while time.monotonic() < deadline:
        if not all(alive(pid) for pid in state.values()):
            raise RuntimeError(f'Process exited while waiting for {label}; see {RUN}')
        if check():
            print(f'[ready] {label}', flush=True)
            return
        time.sleep(2)
    raise RuntimeError(f'Timed out waiting for {label}; see {RUN}')


def ros_check(command, text):
    if command[:2] in (['topic', 'info'], ['node', 'list'], ['lifecycle', 'get']):
        command = [*command, '--no-daemon']
    try:
        result = subprocess.run(['ros2', *command], capture_output=True, text=True, timeout=8)
        return result.returncode == 0 and text in result.stdout
    except subprocess.TimeoutExpired:
        return False


def llama_ok():
    try:
        with urllib.request.urlopen('http://127.0.0.1:8081/health', timeout=2) as response:
            return json.load(response).get('status') == 'ok'
    except Exception:
        return False


def controllers_ready():
    try:
        result = subprocess.run(['ros2', 'control', 'list_controllers'], capture_output=True, text=True, timeout=8)
        return result.returncode == 0 and all(
            any(name in line and 'active' in line and 'inactive' not in line for line in result.stdout.splitlines())
            for name in ('joint_state_broadcaster', 'arm_controller', 'gripper_controller'))
    except subprocess.TimeoutExpired:
        return False


def runtime_file(variable, relative):
    if os.environ.get(variable):
        candidates = [Path(os.environ[variable])]
    else:
        candidates = [base / relative for base in (ROOT, Path.home() / 'ws_aic', Path.home() / 'AIC')]
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    raise RuntimeError(f'Set {variable}; missing {relative}')


def start(headless):
    if any(group_alive(pid) for pid in load().values()):
        raise RuntimeError('Project already running; use status or restart')
    if subprocess.run(['pgrep', '-x', 'gzserver'], stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError('Another Gazebo simulation is running; stop it first')
    binary = runtime_file('LLAMA_BIN', Path('runtime/llama.cpp/build/bin/llama-server'))
    model = runtime_file('MODEL', Path('runtime/models/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf'))
    state = {}

    def spawn(name, command):
        env = os.environ.copy()
        if name == '03_nav2' and env.get('AIC_TF2_FIX_LIB'):
            env['LD_PRELOAD'] = env['AIC_TF2_FIX_LIB'] + (' ' + env['LD_PRELOAD'] if env.get('LD_PRELOAD') else '')
        with (RUN / f'{name}.log').open('w') as output:
            child = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        state[name] = child.pid
        save(state)
        print(f'[start] {name}: {child.pid}', flush=True)

    try:
        world_file = os.environ.get('AIC_WORLD')
        task_config = os.environ.get('AIC_TASK_CONFIG')
        node_arguments = ['--ros-args', '--params-file', task_config] if task_config else []
        spawn('01_gazebo', ['ros2', 'launch', 'mybot', 'gazebo_world.launch.py', f'gui:={str(not headless).lower()}'] + ([f'world:={world_file}'] if world_file else []))
        wait(lambda: ros_check(['topic', 'info', '/scan'], 'Publisher count: 1'), '/scan', state, 180)
        wait(controllers_ready, 'controllers', state)
        spawn('02_moveit', ['ros2', 'launch', 'mybot', 'my_moveit_rviz.launch.py', 'rviz:=false'])
        from ament_index_python.packages import get_package_share_directory
        nav = Path(get_package_share_directory('bot_navigation'))
        map_file = os.environ.get('AIC_MAP', str(nav / 'maps/map.yaml'))
        nav_params = os.environ.get('AIC_NAV_PARAMS', str(nav / 'param/originbot_nav2.yaml'))
        spawn('03_nav2', ['ros2', 'launch', 'nav2_bringup', 'bringup_launch.py', 'use_sim_time:=true', f'map:={map_file}', f'params_file:={nav_params}'])
        wait(lambda: ros_check(['lifecycle', 'get', '/bt_navigator'], 'active'), 'Nav2', state)
        spawn('04_llama', [binary, '-m', model, '-c', '2048', '--threads', '8', '--port', '8081'])
        wait(llama_ok, 'llama /health', state)
        for name, package, executable in [('05_parser', 'llama_command_parser', 'command_parser'), ('06_nav', 'nav_simple', 'simple_navigator'), ('07_arm', 'arm_action_integration', 'arm_grab_place_node'), ('08_detector', 'package_detector', 'color_detector_node'), ('09_main', 'main_controller', 'main_controller_node')]:
            spawn(name, ['ros2', 'run', package, executable, *node_arguments] if name in ('06_nav', '07_arm', '09_main') else ['ros2', 'run', package, executable])
        spawn('10_rosbridge', ['ros2', 'launch', str(ROOT / 'tools' / 'rosbridge_safe.launch.py')])
        wait(lambda: ros_check(['topic', 'info', '/command'], 'Subscription count: 1'), '/command parser', state)
        wait(lambda: ros_check(['topic', 'info', '/nav_done_cargo'], 'Subscription count: 1'), 'arm subscriber', state)
        wait(lambda: ros_check(['node', 'list'], '/rosbridge_websocket'), 'rosbridge', state)
        print('Ready. Publish natural language on /command. Logs: ' + str(RUN))
    except BaseException:
        stop()
        raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', nargs='?', default='start', choices=['start', 'restart', 'stop', 'status', 'logs'])
    parser.add_argument('module', nargs='?')
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    RUN.mkdir(parents=True, exist_ok=True)
    if args.command == 'stop':
        stop()
    elif args.command == 'status':
        if not load():
            print('Project stopped')
        for name, pid in load().items():
            print(name, pid, 'running' if group_alive(pid) else 'stopped')
    elif args.command == 'logs':
        files = [RUN / f'{args.module}.log'] if args.module else sorted(RUN.glob('*.log'))
        for path in files:
            print(f'=== {path.name} ===')
            print('\n'.join(path.read_text(errors='replace').splitlines()[-30:]))
    else:
        if args.command == 'restart':
            stop()
        start(args.headless)


if __name__ == '__main__':
    main()
