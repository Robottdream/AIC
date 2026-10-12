"""Cargo dimensions must determine inertia, rather than SDFormat defaults."""
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('design_short_map', ROOT/'tools/design_short_map.py')
design = importlib.util.module_from_spec(spec)
spec.loader.exec_module(design)


class CargoInertiaTests(unittest.TestCase):
    def test_active_scene_all_ten_cubes_have_physical_inertia(self):
        world = ET.parse(ROOT/'scenarios/short_routes_20261007/office_test.world').find('world')
        for color in ('red', 'blue'):
            for number in range(1, 6):
                with self.subTest(color=color, number=number):
                    link = world.find(f"model[@name='{color}_cube_{number}']/link")
                    self.assertEqual(float(link.findtext('inertial/mass')), .1)
                    self.assertEqual(link.findtext('collision/geometry/box/size'), '0.03 0.03 0.03')
                    for axis in ('ixx', 'iyy', 'izz'):
                        self.assertAlmostEqual(float(link.findtext('inertial/inertia/'+axis)), .000015, places=12)
                    for axis in ('ixy', 'ixz', 'iyz'):
                        self.assertEqual(float(link.findtext('inertial/inertia/'+axis)), 0.)

    def test_generator_replaces_defaults_and_handles_rectangular_cargo(self):
        model = ET.fromstring('''<model><link><inertial><mass>12</mass>
          <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>1</ixy></inertia>
          </inertial><gravity>0</gravity><collision><geometry><box><size>2 3 4</size>
          </box></geometry></collision></link></model>''')
        design.set_cargo_box_inertia(model)
        design.set_cargo_box_inertia(model)
        inertia = model.find('link/inertial/inertia')
        self.assertEqual({child.tag: float(child.text) for child in inertia},
                         dict(ixx=25., iyy=20., izz=13., ixy=0., ixz=0., iyz=0.))
        self.assertEqual(len(model.findall('link/inertial/inertia')), 1)
        self.assertEqual(model.findtext('link/gravity'), '0')


if __name__ == '__main__':
    unittest.main()