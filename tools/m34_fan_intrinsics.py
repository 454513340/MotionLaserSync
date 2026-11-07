#!/usr/bin/env python3
"""Infer the virtual ray fan's index->Sensor-X affine correction from ORIGINAL
valid SENSOR-X (no CAD truth, no fixture labels). Train only user-designated sessions; evaluate
new held-out session but DO NOT optimize from holdouts. This does not recalibrate the device.
"""
import argparse,json,pathlib
import numpy as np
from scipy.optimize import least_squares
from m34_make_labels import read_m4

def extract(path):
    base=[];y=[];zs=[]
    for _,_,_,p in read_m4(path):
        zz=p['z'];ii=p['index'];width=np.where(zz<=95.,58+(50-58)*(zz-70)/25.,50+(42-50)*(zz-95)/25.)
        base.append(-((ii+.5)/1280.-.5)*width)
        y.append(p['x']);zs.append(zz)
    return np.concatenate(base),np.concatenate(y),np.concatenate(zs)

def quality(x,y,s,b):
    e=y-(s*x+b)
    return {'n':len(e),'mae_mm':float(np.mean(np.abs(e))),
           'median_error_mm':float(np.median(e)),'p95_abs_error_mm':float(np.percentile(np.abs(e),95)),
           'rmse_mm':float(np.sqrt(np.mean(e*e)))}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--train',nargs='+',type=pathlib.Path,required=True)
    ap.add_argument('--holdout',nargs='*',type=pathlib.Path,default=[])
    ap.add_argument('--output',type=pathlib.Path,required=True)
    args=ap.parse_args()
    if len(args.train)<2 or set(args.train)&set(args.holdout):raise ValueError('Require >=2 train sessions and strictly disjoint holdouts')
    data={p.stem:extract(p) for p in [*args.train,*args.holdout]}
    X=np.concatenate([data[p.stem][0] for p in args.train]);Y=np.concatenate([data[p.stem][1] for p in args.train])
    r=least_squares(lambda t:Y-(t[0]*X+t[1]),x0=[1,0],loss='soft_l1',f_scale=.20,
                    bounds=([.95,-.60],[1.05,.60]),max_nfev=150)
    s,b=map(float,r.x)
    report={'model':'V2 virtual indexed fan ray correction, Sensor-X measured in original SCAN',
            'trained_only_on':[p.name for p in args.train],
            'heldout_only':[p.name for p in args.holdout],
            'fan_scale':s,'fan_offset_mm':b,
            'trained_z_minmax':[float(np.min(np.concatenate([data[p.stem][2] for p in args.train]))),float(np.max(np.concatenate([data[p.stem][2] for p in args.train])))],
            'interpretation':'Empirical projection of index to already-calibrated Sensor-X, not true laser emitter coordinate.',
            'validation':{}}
    for p in [*args.train,*args.holdout]:
        x,y,z=data[p.stem]
        report['validation'][p.stem]={'legacy':quality(x,y,1.,0.),'corrected':quality(x,y,s,b),
                                      'raw_sensor_z_mm':[float(z.min()),float(z.max())]}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf8')
    print('Fan projection Sensor-X: scale',s,'offset',b)
    for k,v in report['validation'].items():print(k,'MAE',round(v['legacy']['mae_mm'],4),'->',round(v['corrected']['mae_mm'],4))
if __name__=='__main__':main()
