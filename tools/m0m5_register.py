#!/usr/bin/env python3
"""M0/M1 robust CAD-to-MCS initial registration from real M5S1 points.
Requires explicit approximate 6DOF initial pose (NO old pose auto-reuse).
Uses CAD surface sample NN only as a COARSE registration surrogate;
not metrology-accurate B-Rep distance or physically calibrated laser model.
Optimize on independent named scans, report held-out scans without refitting.
"""
import argparse,hashlib,json,pathlib
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import trimesh
DT=np.dtype([('frame','<u4'),('x','<f8'),('y','<f8'),('z','<f8')])
def read(path):
  with path.open('rb') as f:
    h=f.read(20)
  if h[:4]!=b'M5S1':raise ValueError('not a validated M5S1 point archive')
  n=int.from_bytes(h[12:20],'little')
  if path.stat().st_size!=20+28*n:raise ValueError('M5S1 body size mismatch')
  a=np.memmap(path,dtype=DT,mode='r',offset=20,shape=(n,))
  return np.column_stack((a['x'],a['y'],a['z']))
def cad_coordinates(x,q):
  R=Rotation.from_euler('xyz',q[3:],degrees=True).as_matrix()
  return (x-q[:3])@R

def main():
  parser=argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--obj',type=pathlib.Path,required=True)
  parser.add_argument('--train',type=pathlib.Path,nargs='+',required=True)
  parser.add_argument('--holdout',type=pathlib.Path,nargs='+',required=True)
  parser.add_argument('--init',nargs=6,type=float,required=True,metavar=('x','y','z','rx','ry','rz'))
  parser.add_argument('--output',type=pathlib.Path,required=True)
  parser.add_argument('--npoints',type=int,default=6500)
  a=parser.parse_args();r=np.random.default_rng(20261017)
  mesh=trimesh.load(a.obj,process=False)
  if not isinstance(mesh,trimesh.Trimesh):raise ValueError('OBJ is not triangular CAD mesh')
  sampled,_=trimesh.sample.sample_surface(mesh,190000,seed=12)
  surface=cKDTree(np.vstack((sampled,mesh.vertices)))
  q0=np.array(a.init)
  def prepare(f):
    xyz=read(f)
    # Select plausible CAD-body region in the *user supplied initial* position,
    # never silently include the 06 support/fixture. Extra margin 2 mm.
    C=cad_coordinates(xyz,q0)
    inside=np.all((C>=mesh.bounds[0]-2)&(C<=mesh.bounds[1]+2),axis=1)
    xyz=xyz[inside];C=C[inside]
    # Background can overlap the CAD's axis-aligned bounding BOX. Reject
    # it by coarse proximity to the *initial* CAD surface; never let fixture
    # points pull the new part placement. Any true part points far from the
    # initial model will be excluded: initialization must be reasonably close.
    coarse=surface.query(C,workers=3)[0]
    keep=coarse<0.9
    xyz=xyz[keep];C=C[keep]
    if len(xyz)<1000:raise ValueError(f'Not enough points near CAD for {f}; supply better initialization')
    # Avoid dense roof points completely overwhelming hole samples.
    deep=np.flatnonzero(C[:,2]<mesh.bounds[1,2]-0.65)
    roof=np.flatnonzero(C[:,2]>=mesh.bounds[1,2]-0.65)
    nroof=min(len(roof),int(a.npoints*.75));ndeep=min(len(deep),a.npoints-nroof)
    take=np.concatenate([r.choice(roof,nroof,replace=False),r.choice(deep,ndeep,replace=False)])
    return xyz[take]
  train=[prepare(f) for f in a.train];test=[prepare(f) for f in a.holdout]
  training=np.vstack(train)
  weights=np.concatenate([np.full(len(s),1./(len(train)*len(s))) for s in train])
  def distance(q,pts):
    return surface.query(cad_coordinates(pts,q),workers=3)[0]
  def loss(q):
    d=distance(q,training)
    return float(np.dot(weights,np.log1p((d/.25)**2)))
  bounds=[(q0[0]-3,q0[0]+3),(q0[1]-3,q0[1]+3),(q0[2]-3,q0[2]+3),
          (q0[3]-1.5,q0[3]+1.5),(q0[4]-1.5,q0[4]+1.5),(q0[5]-3,q0[5]+3)]
  result=minimize(loss,q0,method='Powell',bounds=bounds,options={'maxiter':30,'maxfev':520,'ftol':1e-6,'xtol':1e-5})
  q=result.x
  def metrics(xs,qmetric):
    out=[]
    for p in xs:
      d=distance(qmetric,p)
      out.append(dict(count=len(d),median_mm=float(np.median(d)),p90_mm=float(np.percentile(d,90)),p99_mm=float(np.percentile(d,99)),fraction_below_0_3_mm=float(np.mean(d<.3))))
    return out
  rep=dict(method='robust approximate surface-NN, NOT certified registration',cad_mesh_sha256=hashlib.sha256(a.obj.read_bytes()).hexdigest(),initial_pose_xyz_rxryrz=q0.tolist(),estimated_pose_xyz_rxryrz=q.tolist(),training_files=[str(f) for f in a.train],holdout_files=[str(f) for f in a.holdout],train=metrics(train,q),holdout=metrics(test,q),train_initial=metrics(train,q0),holdout_initial=metrics(test,q0),loss_initial=loss(q0),loss_fitted=loss(q),optimization_success=bool(result.success),optimization_message=str(result.message),n_function_evaluations=int(result.nfev),model_surface_samples=len(sampled))
  rep['heldout_improved'] = all(x['median_mm'] <= v['median_mm']+0.005
      for x,v in zip(rep['holdout'],rep['holdout_initial']))
  rep['safe_to_propose_fitted_pose'] = (rep['heldout_improved'] and
      rep['loss_fitted'] < rep['loss_initial'] and result.success)
  a.output.write_text(json.dumps(rep,indent=2,ensure_ascii=False),encoding='utf8')
  print('estimated',np.round(q,7),'train median',[round(x['median_mm'],4) for x in rep['train']],'heldout median',[round(x['median_mm'],4) for x in rep['holdout']],'nfev',result.nfev,'status',result.success)
if __name__=='__main__':main()
