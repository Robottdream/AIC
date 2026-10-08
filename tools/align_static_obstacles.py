#!/usr/bin/env python3
"""Align imported PGM walls to local SDF, then recover its 13 obstacle poses."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import yaml
from scipy.ndimage import distance_transform_edt, map_coordinates, label
from scipy.optimize import differential_evolution, linear_sum_assignment
from scipy.spatial import ConvexHull
from design_short_map import walls

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'scenarios/usb_office_20261007/original'
OUT = ROOT / 'scenarios/short_routes_20261007/static_obstacles_from_map.json'


def main():
    pixels = np.array(Image.open(SOURCE/'mapn3.pgm'))
    config = yaml.safe_load((SOURCE/'mapn3.yaml').read_text())
    resolution = config['resolution']
    ox, oy, origin_yaw = config['origin']
    if origin_yaw != 0:
        raise ValueError('Imported PGM must use an axis-aligned origin')
    height, _ = pixels.shape
    world = ET.parse(SOURCE/'offic_room.world').getroot().find('world')
    wall_boxes = list(walls(world.find("model[@name='officeroom']")))
    original_boxes = list(walls(world.find("model[@name='Untitled']")))
    points = np.array([(box['x']+along*math.cos(box['yaw']), box['y']+along*math.sin(box['yaw']))
        for box in wall_boxes
        for along in np.arange(-box['length']/2+.1, box['length']/2, .1)])
    distance = distance_transform_edt(pixels != 0)*resolution

    def error(transform):
        dx, dy, yaw = transform
        c, s = math.cos(yaw), math.sin(yaw)
        x = c*points[:,0]-s*points[:,1]+dx
        y = s*points[:,0]+c*points[:,1]+dy
        distances = map_coordinates(distance, [height-.5-(y-oy)/resolution, (x-ox)/resolution-.5],
            order=1, mode='constant', cval=2.)
        # Ignore the worst 15%: scanned gaps, endpoints and occlusions differ.
        return np.mean(np.sort(np.minimum(distances, 1.)**2)[:int(len(distances)*.85)])

    fit = differential_evolution(error, [(1.2,2.),(-.4,.4),(-.01,.01)], seed=2,
        tol=1e-8, maxiter=120)
    rms = math.sqrt(fit.fun)
    if rms > .05:
        raise RuntimeError(f'Wall registration residual too large: {rms:.3f}m')
    dx, dy, rotation = fit.x
    c, s = math.cos(rotation), math.sin(rotation)
    components, count = label(pixels == 205)
    detections = []
    for component in range(1, count+1):
        rows, cols = np.where(components == component)
        if not (150 <= len(rows) <= 500 and np.ptp(cols) < 30 and np.ptp(rows) < 30):
            continue
        image_points = np.c_[cols+.5, -rows-.5]
        hull = image_points[ConvexHull(image_points).vertices]
        best = None
        for edge in np.roll(hull, -1, axis=0)-hull:
            angle = math.atan2(edge[1], edge[0])
            ca, sa = math.cos(angle), math.sin(angle)
            area = np.prod(np.ptp(hull @ np.array([[ca,-sa],[sa,ca]]), axis=0))
            if best is None or area < best[0]:
                best = (area, angle)
        image_angle = (best[1]+math.pi/4)%(math.pi/2)-math.pi/4
        # Use the observed bounding-box center; gray areas can be partially
        # occluded by a wall, so their center of mass is biased.
        x = ox+(cols.min()+cols.max()+1)/2*resolution
        y = oy+(height-(rows.min()+rows.max()+1)/2)*resolution
        qx, qy = x-dx, y-dy
        detections.append(dict(source_center=[float(x),float(y)],
            source_bbox=[int(cols.min()),int(rows.min()),int(cols.max()),int(rows.max())],
            x=float(c*qx+s*qy), y=float(-s*qx+c*qy), yaw=float(image_angle-rotation)))
    if len(detections) != 13:
        raise RuntimeError(f'Expected 13 box interiors in image, found {len(detections)}')
    # Original model coordinates are only used to name the matching detections.
    distances = np.array([[math.hypot(box['x']-d['source_center'][0], box['y']-d['source_center'][1])
        for d in detections] for box in original_boxes])
    rows, cols = linear_sum_assignment(distances)
    recovered = []
    for row, col in zip(rows, cols):
        if distances[row,col] > .8:
            raise RuntimeError('Image detection does not match an original box')
        box = original_boxes[row]
        recovered.append(dict(link=box['link'], **detections[col], length=box['length'],
            width=box['width'], height=box['height']))
    data = dict(source_pgm='scenarios/usb_office_20261007/original/mapn3.pgm',
        source_sha256=hashlib.sha256((SOURCE/'mapn3.pgm').read_bytes()).hexdigest(),
        local_sdf_to_image_transform=[float(dx),float(dy),float(rotation)],
        wall_alignment_trimmed_rms_m=rms, count=len(recovered), boxes=recovered,
        note='PGM positions/orientations recovered at 0.05m resolution; box dimensions remain 1m.')
    OUT.write_text(json.dumps(data, indent=2)+'\n')
    print(f'Extracted {len(recovered)} boxes; wall alignment RMS {rms:.4f}m; {OUT}')


if __name__ == '__main__':
    main()
