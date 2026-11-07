#!/usr/bin/env python3
"""M4L2 labels for full scan sessions: batch KDTree against immutable CAD samples.
Identical semantics to m34_make_labels.py but avoids per-profile KDTree worker startup.
Not a ground-truth classification of background or multireflections.
"""
import argparse,json,pathlib,struct
import numpy as np,trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from m34_make_labels import read_m4

def convert(path,tree,offset,invR,near,far,batch_size):
    dest=path.with_suffix('.m4l2');stats=np.zeros(4,dtype=np.int64);count=0
    with dest.open('wb') as f:
        f.write(b'M4L2'+struct.pack('<Q',0))
        buf=[]
        def flush():
            nonlocal stats
            if not buf:return
            points=[];sizes=[]
            for ordinal,T,p in buf:
                v=np.column_stack((p['x'],np.zeros(len(p)),p['z']))
                xyz=v@T[:3,:3].T+T[:3,3]
                points.append((xyz-offset)@invR.T)
                sizes.append(len(p))
            batch=np.vstack(points) if sum(sizes) else np.zeros((0,3))
            distances=tree.query(batch,workers=4)[0] if len(batch) else np.zeros(0)
            k=0
            for (ordinal,T,p),n in zip(buf,sizes):
                labels=np.zeros(1280,dtype=np.uint8)
                if n:
                    ds=distances[k:k+n];k+=n
                    labels[p['index']]=np.select([ds<=near,ds>=far],[1,2],default=3).astype(np.uint8)
                f.write(struct.pack('<Q',ordinal));f.write(labels.tobytes());stats+=np.bincount(labels,minlength=4)
            buf.clear()
        for ordinal,_,T,p in read_m4(path):
            buf.append((ordinal,T,p));count+=1
            if len(buf)>=batch_size:flush()
        flush()
        f.seek(4);f.write(struct.pack('<Q',count))
    return {'file':path.name,'frames':count,'class_counts':dict(zip(['NoReturn','CAD_near','CAD_far','Ambiguous'],map(int,stats))),'output':dest.name,'cad_near_mm':near,'cad_far_mm':far,'approximation':'Seeded CAD surface KDTree; not physical material labels'}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--obj',type=pathlib.Path,required=True)
    ap.add_argument('--pose',nargs=6,type=float,required=True)
    ap.add_argument('--m4',nargs='+',type=pathlib.Path,required=True)
    ap.add_argument('--surface-samples',type=int,default=600000)
    ap.add_argument('--cad-near',type=float,default=.5)
    ap.add_argument('--cad-far',type=float,default=1.)
    ap.add_argument('--batch-size',type=int,default=96)
    a=ap.parse_args()
    if not 0<a.cad_near<a.cad_far or a.batch_size<1:raise ValueError('bad thresholds/batch')
    mesh=trimesh.load(str(a.obj),force='mesh',process=False)
    np.random.seed(20261017)
    samples,_=trimesh.sample.sample_surface(mesh,a.surface_samples)
    tree=cKDTree(samples);offset=np.array(a.pose[:3]);invR=Rotation.from_euler('xyz',a.pose[3:],degrees=True).as_matrix().T
    for path in a.m4:
        d=convert(path,tree,offset,invR,a.cad_near,a.cad_far,a.batch_size)
        path.with_suffix('.m4l2.json').write_text(json.dumps(d,indent=2),encoding='utf8')
        print(json.dumps(d,ensure_ascii=False),flush=True)
if __name__=='__main__':main()
