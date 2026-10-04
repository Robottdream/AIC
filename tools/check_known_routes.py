import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT = Path('/mnt/c/Users/21920/.codex/worktrees/5a40/AIC')
spec = importlib.util.spec_from_file_location('known', ROOT/'src/yzbot/bot_navigation/scripts/known_obstacle_forecaster.py')
known = importlib.util.module_from_spec(spec); spec.loader.exec_module(known)
world = ET.parse(ROOT/'src/yzbot/mybot_description/worlds/room.world')
for name, route in known.ROUTES.items():
    model = world.find(".//model[@name='"+name+"']")
    plugin = model.find('plugin')
    assert tuple(float(plugin.findtext('start_'+axis)) for axis in ('x', 'y')) == route['start']
    assert tuple(float(plugin.findtext('end_'+axis)) for axis in ('x', 'y')) == route['end']
    assert float(plugin.findtext('speed')) == route['speed']
    assert model.findtext('link/collision/geometry/box/size').split() == ['0.75']*3
route = known.ROUTES['obstacle3']
turn = known.forecast_route(route, (-2.5, -1.85), (0, -.35), 1)
assert turn[-1]['center'][1] > turn[0]['center'][1] + 1, 'Future fails to turn at endpoint'
assert min(p['center'][1] for p in turn) > -2.0
assert all(abs(p['center'][0]+2.5) < 1e-9 for p in turn)
route = known.ROUTES['obstacle2']
ux, uy = (route['end'][i]-route['start'][i] for i in (0, 1))
length = math.hypot(ux, uy); unit = (ux/length, uy/length)
diagonal = known.forecast_route(route, (-7., -3.75), tuple(v*.35 for v in unit), 1)
assert math.dist(diagonal[-1]['center'], (-7+unit[0]*1.4, -3.75+unit[1]*1.4)) < .02
assert len(turn) == 41 and turn[-1]['t'] == 4.0
print('PASS: embedded routes match world, endpoint reversal, diagonal route, 4s timed samples')
