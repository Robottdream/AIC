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
            virtual_bin = temporary / 'venv/bin'
            virtual_bin.mkdir(parents=True)
            # A poisoned PATH must never select this interpreter, even before
            # the shell sources ROS. Do not install packages into that venv.
            bad_python = virtual_bin / 'python3'
            bad_python.write_text('#!/bin/sh\necho WRONG_VIRTUAL_PYTHON >&2\nexit 87\n')
            bad_python.chmod(0o755)
            shim = temporary / 'capture.py'
            shim.write_text('''#!/usr/bin/python3
import json, os, shutil, sys
print(json.dumps(dict(scene={name: os.environ.get(name) for name in
    ('AIC_WORLD', 'AIC_MAP', 'AIC_TASK_CONFIG', 'AIC_NAV_PARAMS')}, args=sys.argv[2:],
    python=shutil.which('python3'), virtual_env=os.environ.get('VIRTUAL_ENV'),
    pythonhome=os.environ.get('PYTHONHOME'), path=os.environ['PATH'])))
''')
            # Mock only ROS setup and the final exec. Bootstrap Python/PATH
            # sanitization is the real shell code under test.
            ament = temporary / 'ament_index_python'
            ament.mkdir();(ament/'__init__.py').write_text('')
            (ament/'packages.py').write_text('import os\ndef get_package_share_directory(name):\n    return os.environ["AIC_TEST_SHARE"]\n')
            (temporary/'package.xml').write_text('<package><version>0</version></package>')
            environment = os.environ.copy()
            for name in ('AIC_WORLD', 'AIC_MAP', 'AIC_TASK_CONFIG', 'AIC_NAV_PARAMS', 'AIC_SCENE'):
                environment.pop(name, None)
            environment.update(overrides or {})
            environment.update(PATH=f'{virtual_bin}:/mnt/c/fake-sdk:{environment["PATH"]}',
                               VIRTUAL_ENV=str(virtual_bin.parent), PYTHONHOME='/invalid-python-home',
                               PYTHONPATH=str(temporary), AIC_TEST_SHARE=str(temporary),
                               AIC_TEST_CAPTURE=str(shim))
            bash_env = temporary / 'bash_env'
            bash_env.write_text('source() { :; }\ncolcon() { :; }\n'
                                'exec() { /usr/bin/python3 "$AIC_TEST_CAPTURE" "$@"; }\n')
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

    def test_poisoned_virtualenv_does_not_reach_ros_children(self):
        result = self.run_entry()
        self.assertEqual(result['python'], '/usr/bin/python3')
        self.assertIsNone(result['virtual_env'])
        self.assertIsNone(result['pythonhome'])
        self.assertNotIn('/mnt/c/fake-sdk', result['path'])

    def test_explicit_scene_files_keep_priority(self):
        scene = ROOT / 'scenarios/usb_office_20261007'
        overrides = dict(AIC_WORLD=str(scene / 'office_test.world'),
                         AIC_MAP=str(scene / 'mapn3.yaml'),
                         AIC_TASK_CONFIG=str(scene / 'tasks.yaml'),
                         AIC_NAV_PARAMS=str(ROOT / 'src/yzbot/bot_navigation/param/originbot_nav2.yaml'))
        self.assertEqual(self.run_entry(overrides)['scene'], overrides)


if __name__ == '__main__':
    unittest.main()
