"""Failed navigation candidates must not become delivered cargo in the report."""
import ast
from pathlib import Path
import re
import math
import unittest


class BenchmarkCargoTests(unittest.TestCase):
    def test_wall_corner_overlap_detected_at_reported_collision(self):
        path=Path(__file__).resolve().parents[1]/'tools/benchmark_delivery.py'
        tree=ast.parse(path.read_text())
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef)
                      and n.name=='robot_overlaps_static')
        scope={'math':math}
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(path),'exec'),scope)
        overlaps=scope['robot_overlaps_static']
        wall=dict(x=-4.905,y=-2.033,yaw=math.pi/2,length=2.75,width=.15)
        self.assertTrue(overlaps(dict(x=-4.5034126,y=-.7881663,yaw=-2.695083),wall))
        self.assertFalse(overlaps(dict(x=-4.5610683,y=.0726626,yaw=-2.347),wall))
        box=dict(x=1.,y=2.,yaw=math.pi/4,length=.5,width=.5)
        self.assertTrue(overlaps(dict(x=1.4,y=2.,yaw=0.),box))
        self.assertFalse(overlaps(dict(x=1.8,y=2.,yaw=0.),box))

    def test_failed_candidate_does_not_shift_placement_indices(self):
        path=Path(__file__).resolve().parents[1]/'tools/benchmark_delivery.py'
        tree=ast.parse(path.read_text())
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef)
                      and n.name=='grasped_cube_names')
        scope={'re':re}
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(path),'exec'),scope)
        log=('选定可达物块：blue_cube_5，接近方向1\n'
             '抓取点不可达（failed: aborted）\n'
             '选定可达物块：blue_cube_4，接近方向3\n'
             '物块标记为已抓取：blue_cube_4\n')
        self.assertEqual(scope['grasped_cube_names'](log),['blue_cube_4'])


if __name__=='__main__':
    unittest.main()
