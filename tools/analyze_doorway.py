import json
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt

out=Path('/home/polarbear/aic_doorway_evidence_20261003')
data=json.loads((out/'before.json').read_text())
for side,scan in data['scans'].items():
    pts=np.array(scan['points'])
    near=pts[np.linalg.norm(pts,axis=1)<.42]
    print(side,'near center',near.mean(axis=0).tolist(),'bounds',near.min(axis=0).tolist(),near.max(axis=0).tolist())
grid=np.load(out/'global-costmap.npy');info=data['global']
distance=distance_transform_edt(grid<100)*info['resolution']
for name,path in data['paths'].items():
    pts=np.array(path['points']);origin=np.array(info['origin'])
    xy=((pts-origin)/info['resolution']).astype(int)
    valid=(xy[:,0]>=0)&(xy[:,0]<info['width'])&(xy[:,1]>=0)&(xy[:,1]<info['height'])
    xy=xy[valid];d=distance[xy[:,1],xy[:,0]]
    nearest=np.argmin(d)
    print(name,'length',np.linalg.norm(np.diff(pts,axis=0),axis=1).sum(),'wall distance min',d.min(),'at',pts[nearest].tolist(),'current first',pts[:4].tolist())
print('retreat firstlast',data['paths']['retreat']['points'][::8])
