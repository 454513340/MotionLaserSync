#!/usr/bin/env python3
"""Versioned geometric proximity labels for M3/M4 V2 (offline ONLY).
0: original sample absent; 1: high-confidence CAD-near valid sample;
2: CAD-far return (NOT claimed to be known fixture); 3: ambiguous valid sample.
Uses dense, seeded triangle-surface sampling + KD-tree; approximate distance
and must NEVER be used as metric geometry ground truth or label no-return cause.
"""
import argparse, json, pathlib, struct
import numpy as np
import trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

def read_m4(path):
    with path.open('rb') as f:
        if f.read(4)!=b'M4S1': raise ValueError('not an M4S1 file')
        n,=struct.unpack('<Q',f.read(8))
        for _ in range(n):
            ordinal,t=struct.unpack('<Qd',f.read(16))
            transform=np.frombuffer(f.read(128),dtype='<f8').reshape(4,4).copy()
            count,=struct.unpack('<I',f.read(4))
            dtype=np.dtype([('index','<u4'),('x','<f8'),('z','<f8')])
            points=np.frombuffer(f.read(count*20),dtype=dtype).copy()
            yield ordinal, t,transform,points
        if f.read(): raise ValueError('trailing M4S1 bytes')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--obj',type=pathlib.Path,required=True)
    p.add_argument('--pose',type=float,nargs=6,required=True)
    p.add_argument('--m4',nargs='+',type=pathlib.Path,required=True)
    p.add_argument('--cad-near',type=float,default=.50)
    p.add_argument('--cad-far',type=float,default=1.00)
    p.add_argument('--surface-samples',type=int,default=600000)
    a=p.parse_args()
    assert 0<a.cad_near<a.cad_far and 1000<=a.surface_samples<=3000000
    mesh=trimesh.load(str(a.obj),force='mesh',process=False)
    np.random.seed(20261017)
    samples,_=trimesh.sample.sample_surface(mesh,a.surface_samples)
    tree=cKDTree(samples)
    offset=np.array(a.pose[:3],float)
    invR=Rotation.from_euler('xyz',a.pose[3:],degrees=True).as_matrix().T
    for path in a.m4:
        frames=list(read_m4(path))
        output=path.with_suffix('.m4l2')
        counts=np.zeros(4,dtype=np.int64)
        all_d=[]
        with output.open('wb') as f:
            f.write(b'M4L2'+struct.pack('<Q',len(frames)))
            for ordinal,_,T,points in frames:
                label=np.zeros(1280,dtype=np.uint8)
                if len(points):
                    v=np.column_stack((points['x'],np.zeros(len(points)),points['z']))
                    machine=v@T[:3,:3].T+T[:3,3]
                    cad=(machine-offset)@invR.T
                    distances=tree.query(cad,k=1,workers=-1)[0]
                    all_d.append(distances)
                    label[points['index']]=np.select([distances<=a.cad_near,distances>=a.cad_far],[1,2],default=3).astype(np.uint8)
                counts+=np.bincount(label,minlength=4)
                f.write(struct.pack('<Q',ordinal))
                f.write(label.tobytes())
        distances=np.concatenate(all_d) if all_d else np.zeros(0)
        report={'source':str(path),'mask':str(output),'class_counts':dict(zip(['NoReturn','CAD_near','CAD_far','Ambiguous'],map(int,counts))),
            'cad_near_mm':a.cad_near,'cad_far_mm':a.cad_far,'samples':a.surface_samples,
            'distance_p50_mm':float(np.median(distances)) if len(distances) else None,
            'distance_p95_mm':float(np.percentile(distances,95)) if len(distances) else None,
            'label_semantics':'CAD-near, CAD-far, or ambiguous by approximate mesh-sample NN; far is not confirmed fixture',
            'pose_xyz_euler_xyz_degrees':a.pose}
        output.with_suffix('.m4l2.json').write_text(json.dumps(report,indent=2),encoding='utf8')
        print(path.name,'frames',len(frames),'classCounts',counts.tolist())
if __name__=='__main__': main()
