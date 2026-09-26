#!/usr/bin/env python3
"""Overlay Gazebo's static office wall collisions onto the navigation map.

Run this after changing room.world; map.pgm remains the original reference map.
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw
import yaml

HERE = Path(__file__).resolve().parent
WORLD = HERE.parents[1] / "mybot_description/worlds/room.world"
MAP_YAML = HERE / "map.yaml"
OUTPUT = HERE / "map_corrected.png"


def pose(text):
    values = [float(value) for value in (text or "0 0 0 0 0 0").split()]
    return values[0], values[1], values[5]


def compose(parent, child):
    x, y, yaw = parent
    dx, dy, dyaw = child
    return (
        x + dx * math.cos(yaw) - dy * math.sin(yaw),
        y + dx * math.sin(yaw) + dy * math.cos(yaw),
        yaw + dyaw,
    )


def main():
    config = yaml.safe_load(MAP_YAML.read_text())
    source = HERE / "map.pgm"
    image = Image.open(source).convert("L")
    draw = ImageDraw.Draw(image)
    origin_x, origin_y, _ = config["origin"]
    resolution = float(config["resolution"])
    world = ET.parse(WORLD).getroot()
    office = next(model for model in world.findall(".//world/model")
                  if model.get("name") == "PAL_office")
    model_pose = pose(office.findtext("pose"))
    wall_count = 0

    for link in office.findall("link"):
        link_pose = compose(model_pose, pose(link.findtext("pose")))
        for collision in link.findall("collision"):
            size = collision.findtext("geometry/box/size")
            if size is None:
                continue
            length, width, _ = map(float, size.split())
            center_x, center_y, yaw = compose(link_pose, pose(collision.findtext("pose")))
            corners = []
            for dx, dy in ((-length / 2, -width / 2),
                           (length / 2, -width / 2),
                           (length / 2, width / 2),
                           (-length / 2, width / 2)):
                x = center_x + dx * math.cos(yaw) - dy * math.sin(yaw)
                y = center_y + dx * math.sin(yaw) + dy * math.cos(yaw)
                corners.append(((x - origin_x) / resolution,
                                image.height - 1 - (y - origin_y) / resolution))
            draw.polygon(corners, fill=0)
            wall_count += 1

    image.save(OUTPUT)
    print(f"Wrote {OUTPUT} with {wall_count} Gazebo wall collision boxes")


if __name__ == "__main__":
    main()
