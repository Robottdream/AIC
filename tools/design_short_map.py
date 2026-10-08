#!/usr/bin/env python3
"""Generate a dispersed cargo layout on the imported office walls."""
import copy
import hashlib
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw
import yaml
from scipy.ndimage import distance_transform_edt, label
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'scenarios/short_routes_20261007'
RED = [(-12.0, 2.5), (-12.0, -6.5), (-7.0, -12.0), (4.0, -11.0), (11.5, 3.0)]
BLUE = [(-9.5, 0.0), (-8.5, -4.5), (-3.5, -12.7), (11.0, -9.5), (8.5, -2.5)]
CARGO_MIN_EDGE_GAP = 2.0
CARGO_DESIGN_CENTER_GAP = 3.5
ZONES = {'A': (-7.4, -5.4), 'B': (7.5, -5.4), 'C': (-2.25, 3.1)}
ZONE_SIZE = (2.0, 1.6)
OBSTACLES = [
    dict(name='design_obstacle_left', start=(-3.65, 1.0), end=(-1.4, 1.0), size=0.5, height=0.5, shape='box', speed=0.35),
    dict(name='design_obstacle_right', start=(-5.6, -7.06), end=(-1.45, -7.06), size=0.5, height=0.5, shape='box', speed=0.35, min_wall_clearance=0.4),
]
RESOLUTION = 0.05
ORIGIN = (-16.0, -16.0)
SIZE = (640, 440)


def pose(text):
    values = list(map(float, (text or '0 0 0 0 0 0').split()))
    return values[0], values[1], values[5]


def compose(parent, child):
    x, y, yaw = parent
    dx, dy, angle = child
    return (x + dx*math.cos(yaw)-dy*math.sin(yaw),
            y + dx*math.sin(yaw)+dy*math.cos(yaw), yaw+angle)


def walls(office):
    base = pose(office.findtext('pose'))
    for link in office.findall('link'):
        center = compose(base, pose(link.findtext('pose')))
        for collision in link.findall('collision'):
            size = collision.findtext('geometry/box/size')
            if size:
                x, y, yaw = compose(center, pose(collision.findtext('pose')))
                length, width, height = map(float, size.split())
                yield dict(link=link.get('name'), x=x, y=y, yaw=yaw, length=length, width=width, height=height)


def corners(wall):
    x, y, yaw = wall['x'], wall['y'], wall['yaw']
    return [(x+dx*math.cos(yaw)-dy*math.sin(yaw),
             y+dx*math.sin(yaw)+dy*math.cos(yaw))
            for dx, dy in [(-wall['length']/2, -wall['width']/2),
                           (wall['length']/2, -wall['width']/2),
                           (wall['length']/2, wall['width']/2),
                           (-wall['length']/2, wall['width']/2)]]


def pixel(point):
    return ((point[0]-ORIGIN[0])/RESOLUTION,
            SIZE[1]-1-(point[1]-ORIGIN[1])/RESOLUTION)


def rectangle_distance(first, second):
    """Minimum distance between two convex polygons, including overlap."""
    separated = False
    for polygon in (first, second):
        for a, b in zip(polygon, polygon[1:] + polygon[:1]):
            axis = (a[1]-b[1], b[0]-a[0])
            p = [x*axis[0]+y*axis[1] for x, y in first]
            q = [x*axis[0]+y*axis[1] for x, y in second]
            separated |= max(p) < min(q) or max(q) < min(p)
    if not separated:
        return 0.0
    distances = []
    for points, edges in ((first, second), (second, first)):
        for x, y in points:
            for a, b in zip(edges, edges[1:] + edges[:1]):
                dx, dy = b[0]-a[0], b[1]-a[1]
                t = max(0., min(1., ((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy)))
                distances.append(math.hypot(x-a[0]-t*dx, y-a[1]-t*dy))
    return min(distances)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    stable = ET.parse(ROOT/'src/yzbot/mybot_description/worlds/room.world').getroot().find('world')
    imported = ET.parse(ROOT/'scenarios/usb_office_20261007/original/offic_room.world').getroot().find('world')
    root = ET.Element('sdf', version='1.4')
    world = ET.SubElement(root, 'world', name='default')
    for tag in ['light', 'physics']:
        world.append(copy.deepcopy(stable.find(tag)))
    world.append(copy.deepcopy(stable.find("model[@name='ground_plane']")))
    office = copy.deepcopy(imported.find("model[@name='officeroom']"))
    world.append(office)
    static_obstacles = copy.deepcopy(imported.find("model[@name='Untitled']"))
    if static_obstacles is None:
        raise RuntimeError('Original 13-box obstacle model missing')
    static_obstacles.set('name', 'imported_static_obstacles')
    # Original SDF declared a dynamic model. Freeze its links to retain the
    # requested obstacle positions rather than allowing them to settle or drift.
    static_obstacles.find('static').text = '1'
    alignment_file = OUT/'static_obstacles_from_map.json'
    if not alignment_file.exists():
        raise RuntimeError('Run tools/align_static_obstacles.py first')
    alignment = json.loads(alignment_file.read_text())
    source_pgm = ROOT/'scenarios/usb_office_20261007/original/mapn3.pgm'
    if hashlib.sha256(source_pgm.read_bytes()).hexdigest() != alignment['source_sha256']:
        raise RuntimeError('Original PGM changed; rerun tools/align_static_obstacles.py')
    static_obstacles.find('pose').text = '0 0 0.5 0 0 0'
    recovered = {box['link']: box for box in alignment['boxes']}
    for link in static_obstacles.findall('link'):
        box = recovered[link.get('name')]
        link.find('pose').text = f"{box['x']} {box['y']} 0 0 0 {box['yaw']}"
        for visual in link.findall('visual'):
            material = visual.find('material')
            if material is None:
                material = ET.SubElement(visual, 'material')
            material.clear()
            ET.SubElement(material, 'ambient').text = '1 0.45 0.05 1'
            ET.SubElement(material, 'diffuse').text = '1 0.45 0.05 1'
    world.append(static_obstacles)
    for color, positions in [('red', RED), ('blue', BLUE)]:
        for index, (x, y) in enumerate(positions, 1):
            model = copy.deepcopy(stable.find(f"model[@name='{color}_cube_{index}']"))
            model.find('pose').text = f'{x} {y} 0.75 0 0 0'
            world.append(model)
    for key, (x, y) in ZONES.items():
        model = copy.deepcopy(stable.find(f"model[@name='zone_{key.lower()}']"))
        model.find('pose').text = f'{x} {y} 0.01 0 0 0'
        model.find("link[@name='base']/visual/geometry/box/size").text = f'{ZONE_SIZE[0]} {ZONE_SIZE[1]} 0.02'
        world.append(model)
    for obstacle in OBSTACLES:
        model = copy.deepcopy(stable.find("model[@name='obstacle2']"))
        model.set('name', obstacle['name'])
        x, y = obstacle['start']
        model.find('pose').text = f'{x} {y} {obstacle["height"]/2} 0 0 0'
        link = model.find('link')
        for geometry in link.findall('collision/geometry') + link.findall('visual/geometry'):
            geometry.clear()
            box = ET.SubElement(geometry, 'box')
            ET.SubElement(box, 'size').text = f'{obstacle["size"]} {obstacle["size"]} {obstacle["height"]}'
        inertia = link.find('inertial/inertia')
        mass = float(link.findtext('inertial/mass'))
        inertia.find('ixx').text = inertia.find('iyy').text = str(mass*(obstacle['size']**2+obstacle['height']**2)/12)
        inertia.find('izz').text = str(mass*obstacle['size']**2/6)
        material = link.find('visual/material')
        material.clear()
        ET.SubElement(material, 'ambient').text = '1 0.45 0.05 1'
        ET.SubElement(material, 'diffuse').text = '1 0.45 0.05 1'
        plugin = model.find('plugin')
        plugin.set('name', 'move_' + obstacle['name'])
        for key, value in [('start_x', x), ('start_y', y),
                           ('end_x', obstacle['end'][0]), ('end_y', obstacle['end'][1]),
                           ('speed', obstacle['speed'])]:
            plugin.find(key).text = str(value)
        world.append(model)
    for plugin in stable.findall('plugin'):
        world.append(copy.deepcopy(plugin))
    gui = ET.SubElement(world, 'gui', fullscreen='0')
    camera = ET.SubElement(gui, 'camera', name='user_camera')
    ET.SubElement(camera, 'pose').text = '9 -15 19 0 0.75 2.15'
    ET.SubElement(camera, 'view_controller').text = 'orbit'
    tree = ET.ElementTree(root)
    ET.indent(tree, space='  ')

    wall_boxes = list(walls(office))
    static_boxes = list(walls(static_obstacles))
    if len(static_boxes) != 13:
        raise RuntimeError(f'Expected 13 imported boxes, got {len(static_boxes)}')
    im = Image.new('L', SIZE, 205)
    draw = ImageDraw.Draw(im)
    # Interior bounds from the imported office's outer walls.
    draw.rectangle([pixel((-15.55, 5.0)), pixel((14.2, -14.85))], fill=254)
    for solid in wall_boxes + static_boxes:
        draw.polygon([pixel(p) for p in corners(solid)], fill=0)
    map_config = dict(
        image='mapn3.pgm', mode='trinary', resolution=RESOLUTION,
        origin=[*ORIGIN, 0.0], negate=0, occupied_thresh=0.65, free_thresh=0.25)

    parking = [(x-0.55, y-(0.16 if key == 'A' else 0.1 if key == 'B' else 0.0))
               for key, (x, y) in ZONES.items()]
    flatten = lambda pairs: [float(value) for pair in pairs for value in pair]
    config = {
        'main_controller_node': {'ros__parameters': dict(red_block_positions=flatten(RED),
            blue_block_positions=flatten(BLUE), area_parking_positions=flatten(parking), stable_layout_hints=False)},
        'simple_nav2_navigator': {'ros__parameters': dict(area_parking_positions=flatten(parking), crossing_max_wait=12.0,
            obstacle_names=[o['name'] for o in OBSTACLES],
            obstacle_tracks=flatten([p for o in OBSTACLES for p in (o['start'], o['end'])]),
            obstacle_sizes=[o['size'] for o in OBSTACLES], obstacle_speeds=[o['speed'] for o in OBSTACLES])},
        'arm_grab_place_node': {'ros__parameters': dict(zone_centers=flatten(ZONES.values()), zone_size=list(ZONE_SIZE))},
    }
    # This map has a diagonal crossing; leave the stable map's Nav2 file intact.
    navigation = yaml.safe_load((ROOT/'src/yzbot/bot_navigation/param/originbot_nav2.yaml').read_text())
    follow = navigation['controller_server']['ros__parameters']['FollowPath']
    follow.update(max_lookahead_dist=1.5, cost_scaling_dist=0.9,
                  max_allowed_time_to_collision_up_to_carrot=2.0)
    for name, frequency in [('local_costmap', 20.0), ('global_costmap', 5.0)]:
        params = navigation[name][name]['ros__parameters']
        params['update_frequency'] = frequency
        params['inflation_layer']['inflation_radius'] = 1.0

    free = np.array(im) == 254
    clearance = distance_transform_edt(free)*RESOLUTION
    components, _ = label(clearance >= 0.48)
    def cell(point):
        x, y = pixel(point)
        return int(round(y)), int(round(x))
    home_component = int(components[cell((0, 0))])
    if not home_component:
        raise RuntimeError('Start is not clear of walls/static obstacles')
    cargo_rectangles = {}
    for color, positions in [('red', RED), ('blue', BLUE)]:
        for i, (x, y) in enumerate(positions, 1):
            name = f'{color}_cube_{i}'
            model = world.find(f"model[@name='{name}']")
            dimensions = model.findtext('link/collision/geometry/box/size')
            if dimensions is None:
                raise RuntimeError(f'Cargo box geometry missing: {name}')
            length, width, _ = map(float, dimensions.split())
            cargo_rectangles[name] = dict(x=x, y=y, yaw=0., length=length, width=width)
            if not free[cell((x, y))]:
                raise RuntimeError(f'Cargo intersects a wall/static obstacle or unknown cell: {name}')
    cargo_pair_distances = []
    for (first, a), (second, b) in itertools.combinations(cargo_rectangles.items(), 2):
        center_gap = math.hypot(a['x']-b['x'], a['y']-b['y'])
        edge_gap = rectangle_distance(corners(a), corners(b))
        if edge_gap <= CARGO_MIN_EDGE_GAP or center_gap < CARGO_DESIGN_CENTER_GAP:
            raise RuntimeError(f'Cargo spacing too small: {first}/{second}, center={center_gap:.3f}m, edge={edge_gap:.3f}m')
        cargo_pair_distances.append(dict(first=first, second=second, center_m=center_gap, edge_m=edge_gap))
    grasp_checks = {}
    for color, positions in [('red', RED), ('blue', BLUE)]:
        for i, (x, y) in enumerate(positions, 1):
            candidates = [(x-.55,y), (x,y-.55), (x+.55,y), (x,y+.55)]
            reachable = [p for p in candidates if components[cell(p)] == home_component]
            if not reachable:
                raise RuntimeError(f'No reachable grasp pose for {color}_cube_{i}')
            grasp_checks[f'{color}_cube_{i}'] = reachable
    for parking_pose in parking:
        if components[cell(parking_pose)] != home_component:
            raise RuntimeError(f'Unreachable area parking: {parking_pose}')
    for name, (x, y) in ZONES.items():
        for px in np.linspace(x-ZONE_SIZE[0]/2, x+ZONE_SIZE[0]/2, 41):
            for py in np.linspace(y-ZONE_SIZE[1]/2, y+ZONE_SIZE[1]/2, 33):
                if not free[cell((px,py))]:
                    raise RuntimeError(f'Area {name} intersects a wall/static obstacle or unknown cell')
    grasp_track_distances = {}
    track_wall_clearances = {}
    track_static_clearances = {}
    for obstacle in OBSTACLES:
        sx, sy = obstacle['start']; ex, ey = obstacle['end']
        length_squared = (ex-sx)**2 + (ey-sy)**2
        distances = []
        for candidates in grasp_checks.values():
            for x, y in candidates:
                fraction = max(0., min(1., ((x-sx)*(ex-sx)+(y-sy)*(ey-sy))/length_squared))
                distances.append(math.hypot(x-sx-fraction*(ex-sx), y-sy-fraction*(ey-sy)))
        minimum = min(distances)
        # Square circumscribed radius + Nav2 footprint radius + 0.19 m margin.
        # Keep every usable grasp station outside the whole swept corridor.
        if minimum < obstacle['size']/math.sqrt(2) + 0.405 + 0.19:
            raise RuntimeError(f'Obstacle track too close to grasp parking: {obstacle["name"]} {minimum:.3f}m')
        grasp_track_distances[obstacle['name']] = minimum
        # A translating box sweeps the convex hull of its two endpoint boxes.
        # Check the continuous volume so narrow gaps cannot slip between samples.
        endpoints = np.array([point for x, y in (obstacle['start'], obstacle['end'])
            for point in corners(dict(x=x, y=y, yaw=0., length=obstacle['size'], width=obstacle['size']))])
        swept = [tuple(endpoints[i]) for i in ConvexHull(endpoints).vertices]
        wall_clearances = [rectangle_distance(swept, corners(wall)) for wall in wall_boxes]
        # The requested lower openings permit 0.4 m surface clearance;
        # the wider upper opening retains its existing 0.6 m requirement.
        if min(wall_clearances) < obstacle.get('min_wall_clearance', .6):
            raise RuntimeError(f'Obstacle swept volume too close to wall: {obstacle["name"]}')
        track_wall_clearances[obstacle['name']] = min(wall_clearances)
        static_clearances = [rectangle_distance(swept, corners(box)) for box in static_boxes]
        if min(static_clearances) < .4:
            raise RuntimeError(f'Obstacle swept volume too close to static box: {obstacle["name"]}, clearance={min(static_clearances):.3f}m')
        track_static_clearances[obstacle['name']] = min(static_clearances)
    metadata = dict(rule_source='比赛规则.docx embedded images 1-4',
        cargo=dict(red=RED, blue=BLUE), zones=ZONES, zone_size=ZONE_SIZE, parking=parking,
        cargo_pair_distances=cargo_pair_distances,
        min_cargo_center_distance_m=min(p['center_m'] for p in cargo_pair_distances),
        min_cargo_edge_gap_m=min(p['edge_m'] for p in cargo_pair_distances),
        cargo_min_edge_gap_requirement_m=CARGO_MIN_EDGE_GAP,
        obstacles=OBSTACLES, wall_boxes=wall_boxes, static_boxes=static_boxes,
        static_obstacle_geometry_preserved_from_local_usb_original=True, static_obstacle_count=len(static_boxes), static_obstacle_color="orange",
        static_obstacle_poses_from_original_pgm=True, static_image_alignment=alignment,
        reachable_grasp_poses=grasp_checks,
        static_clearance_gate_m=0.48, walls_preserved_from_local_usb_original=True,
        min_grasp_track_center_distance_m=grasp_track_distances,
        min_obstacle_surface_to_wall_m=track_wall_clearances,
        cargo_physics='unchanged current stable definitions',
        min_obstacle_surface_to_static_box_m=track_static_clearances,
        imported_static_obstacle_physics='static=1; original visual/collision geometry retained; poses aligned to original PGM')
    # Publish the files only after the whole layout has passed validation.
    tree.write(OUT/'office_test.world', encoding='utf-8', xml_declaration=True)
    im.save(OUT/'mapn3.pgm')
    (OUT/'mapn3.yaml').write_text(yaml.safe_dump(map_config, sort_keys=False))
    (OUT/'tasks.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    (OUT/'nav2.yaml').write_text(yaml.safe_dump(navigation, sort_keys=False))
    (OUT/'layout.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(f'Generated {OUT}; {len(wall_boxes)} walls, {len(static_boxes)} static boxes, all ten cargo and three areas reachable.')


if __name__ == '__main__':
    main()
