#!/usr/bin/env python3
"""Plot a generated scenario and optionally the measured robot trajectory."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import math

parser = argparse.ArgumentParser()
parser.add_argument('scene', type=Path)
parser.add_argument('--motion', type=Path)
args = parser.parse_args()
data = json.loads((args.scene/'layout.json').read_text())
fig, ax = plt.subplots(figsize=(11, 8), constrained_layout=True)
for wall in data['wall_boxes']:
    x,y,yaw=wall['x'],wall['y'],wall['yaw']
    pts=[(x+dx*math.cos(yaw)-dy*math.sin(yaw),y+dx*math.sin(yaw)+dy*math.cos(yaw))
         for dx,dy in [(-wall['length']/2,-wall['width']/2),(wall['length']/2,-wall['width']/2),
                       (wall['length']/2,wall['width']/2),(-wall['length']/2,wall['width']/2)]]
    ax.add_patch(Polygon(pts, color='#53606c'))
for box in data.get('static_boxes', []):
    x,y,yaw=box['x'],box['y'],box['yaw']
    pts=[(x+dx*math.cos(yaw)-dy*math.sin(yaw),y+dx*math.sin(yaw)+dy*math.cos(yaw))
         for dx,dy in [(-box['length']/2,-box['width']/2),(box['length']/2,-box['width']/2),
                       (box['length']/2,box['width']/2),(-box['length']/2,box['width']/2)]]
    ax.add_patch(Polygon(pts, facecolor='#ff730d', edgecolor='#26323d', linewidth=1.2, zorder=4))
    ax.text(x,y,box['link'].replace('link_', 'S'),ha='center',va='center',fontsize=7,zorder=5)
for color,points in data['cargo'].items():
    for i,(x,y) in enumerate(points,1):
        ax.scatter([x],[y],color=color,s=65,zorder=5)
        ax.annotate(f'{color[0].upper()}{i}',(x,y),xytext=(7,-9),textcoords='offset points',fontsize=10)
for name,(x,y) in data['zones'].items():
    width,height=data.get('zone_size',[1.,.5])
    ax.add_patch(Rectangle((x-width/2,y-height/2),width,height,color={'A':'#50ac5b','B':'#e4b943','C':'#df995c'}[name],zorder=4))
    ax.text(x,y+height/2+.25,f'{name} ({x:.1f}, {y:.1f})',ha='center',weight='bold',fontsize=10)
for i,obstacle in enumerate(data['obstacles'],1):
    sx,sy=obstacle['start'];ex,ey=obstacle['end']
    if obstacle.get('shape') == 'box':
        side = obstacle['size']
        ax.add_patch(Rectangle((sx-side/2, sy-side/2), side, side, color='#ff730d', zorder=5))
    ax.plot([sx,ex],[sy,ey],'--',color='#af5abe',linewidth=3,zorder=3)
    ax.annotate('',xy=(ex,ey),xytext=(sx,sy),arrowprops=dict(arrowstyle='<->',color='#af5abe'))
    ax.text((sx+ex)/2+.5,(sy+ey)/2,f'M{i}: {obstacle["speed"]:.2f} m/s',fontsize=9,color='#8a3298',bbox=dict(facecolor='white',edgecolor='none',alpha=.8))
ax.scatter([0],[0],marker='*',s=150,color='#25384c',zorder=6,label='Robot start')
if args.motion:
    motion=json.loads(args.motion.read_text())
    ax.plot([s['x'] for s in motion],[s['y'] for s in motion],color='#009b9f',alpha=.65,linewidth=1.5,label='Measured path',zorder=2)
ax.set_xlim(-15.9,14.6);ax.set_ylim(-15.3,5.4);ax.set_aspect('equal')
ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
ax.set_title('Dispersed cargo and three areas on the office map')
ax.grid(alpha=.15);ax.legend(loc='lower left')
output=args.scene/('layout_measured.png' if args.motion else 'layout.png')
fig.savefig(output,dpi=160)
print(output)
