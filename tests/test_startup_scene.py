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


if __name__ == '__main__':
    unittest.main()
