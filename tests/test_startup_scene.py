"""Exercise the shell entry without starting ROS or modifying an install."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class StartupSceneTests(unittest.TestCase):
    def run_entry(self, overrides=None):
        with tempfile.TemporaryDirectory() as directory:
            temporary = Path(directory)
            shim = temporary / 'python3'
            shim.write_text('''#!/usr/bin/python3
import json, os, sys
if sys.argv[1] == '-c':
    print(os.environ['PATH'] if 'os.environ["PATH"]' in sys.argv[2] else '0')
else:
    print(json.dumps(dict(scene={name: os.environ.get(name) for name in
        ('AIC_WORLD', 'AIC_MAP', 'AIC_TASK_CONFIG', 'AIC_NAV_PARAMS')}, args=sys.argv[1:])))
''')
            shim.chmod(0o755)
            environment = os.environ.copy()
            for name in ('AIC_WORLD', 'AIC_MAP', 'AIC_TASK_CONFIG', 'AIC_NAV_PARAMS', 'AIC_SCENE'):
                environment.pop(name, None)
            environment.update(overrides or {})
            environment['PATH'] = f'{temporary}:{environment["PATH"]}'
            bash_env = temporary / 'bash_env'
            bash_env.write_text('source() { :; }\ncolcon() { :; }\n')
            environment['BASH_ENV'] = str(bash_env)
            result = subprocess.run(['bash', str(ROOT / 'ros_competition_start.sh'),
                                     'benchmark', '--headless', '--use-running'],
                                    env=environment, capture_output=True, text=True, check=True)
            return json.loads(result.stdout.splitlines()[-1])

    def test_default_entry_uses_one_current_scene_and_forwards_options(self):
        result = self.run_entry()
        scene = ROOT / 'scenarios/short_routes_20261007'
        self.assertEqual(result['scene'], dict(
            AIC_WORLD=str(scene / 'office_test.world'), AIC_MAP=str(scene / 'mapn3.yaml'),
            AIC_TASK_CONFIG=str(scene / 'tasks.yaml'), AIC_NAV_PARAMS=str(scene / 'nav2.yaml')))
        self.assertEqual(result['args'][-2:], ['--headless', '--use-running'])

    def test_explicit_scene_files_keep_priority(self):
        scene = ROOT / 'scenarios/usb_office_20261007'
        overrides = dict(AIC_WORLD=str(scene / 'office_test.world'),
                         AIC_MAP=str(scene / 'mapn3.yaml'),
                         AIC_TASK_CONFIG=str(scene / 'tasks.yaml'),
                         AIC_NAV_PARAMS=str(ROOT / 'src/yzbot/bot_navigation/param/originbot_nav2.yaml'))
        self.assertEqual(self.run_entry(overrides)['scene'], overrides)


class StagedRuntimeTests(unittest.TestCase):
    def run_staged_entry(self, overrides=None):
        with tempfile.TemporaryDirectory() as directory:
            temporary = Path(directory)
            source = temporary / '桌面 AIC'
            source.mkdir()
            stage = temporary / 'stage'
            # Isolate the wrapper's fixed /tmp destination from a real running stack.
            script = (ROOT / 'test_map.sh').read_text().replace(
                'AIC_TEST_WORKSPACE="/tmp/aic-map-$AIC_SCENE_NAME"',
                f'AIC_TEST_WORKSPACE="{stage}"')
            (source / 'test_map.sh').write_text(script)
            for name in ('src', 'tools', 'scenarios/short_routes_20261007'):
                (source / name).mkdir(parents=True)
            assets = dict(
                LLAMA_BIN='runtime/llama.cpp/build/bin/llama-server',
                MODEL='runtime/models/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf')
            for relative in assets.values():
                path = source / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.touch()
            (source / 'ros_competition_start.sh').write_text('''#!/usr/bin/env bash
/usr/bin/python3 - "$@" <<'PY'
import json, os, sys
from pathlib import Path
print(json.dumps(dict(runtime={key: os.environ.get(key) for key in
    ('LLAMA_BIN', 'MODEL', 'AIC_ROSBRIDGE_PORT')}, args=sys.argv[1:], staged_runtime=Path('runtime').exists())))
PY
''')
            environment = os.environ.copy()
            for name in ('LLAMA_BIN', 'MODEL', 'AIC_ROSBRIDGE_PORT', 'BASH_ENV'):
                environment.pop(name, None)
            environment.update(overrides or {})
            result = subprocess.run(
                ['bash', str(source / 'test_map.sh'), 'short_routes_20261007',
                 'start', '--headless'], cwd=stage.parent,
                env=environment, capture_output=True, text=True, check=True)
            return json.loads(result.stdout), {
                **{key: str(source / relative) for key, relative in assets.items()},
                'AIC_ROSBRIDGE_PORT': '8765'}

    def test_new_shell_resolves_source_assets_and_forwards_options(self):
        result, expected = self.run_staged_entry()
        self.assertEqual(result['runtime'], expected)
        self.assertEqual(result['args'], ['start', '--headless'])
        self.assertFalse(result['staged_runtime'])

    def test_explicit_runtime_paths_keep_priority(self):
        overrides = dict(LLAMA_BIN='/custom runtime/llama-server', MODEL='/custom model.gguf',
                         AIC_ROSBRIDGE_PORT='9876')
        result, _ = self.run_staged_entry(overrides)
        self.assertEqual(result['runtime'], overrides)


if __name__ == '__main__':
    unittest.main()
