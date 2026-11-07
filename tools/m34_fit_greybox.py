#!/usr/bin/env python3
"""Train V3 grey-box effective stripe model on 02/04/05/06; 03 is test-only.

Offline research prototype. Not factory parameter inversion and not verified
physical camera images. Four training scans are the only optimization sources.
"""
from __future__ import annotations
import argparse,json,time,hashlib,sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import minimize
from scipy.special import logsumexp
from dataclasses import asdict
from m34_greybox_imager import GreyboxParams,prepare,predict,score,render_virtual_detector

KEYS=['gain','recessed_reflectance','specular_ratio','specular_roughness_rad',
      'footprint_sigma_cols','min_component_area','detection_softness']
BOUNDS=[(np.log(.6),np.log(25)), (np.log(.08),np.log(1.5)),
        (np.log(.05),np.log(15.)),(.18,1.4),(.30,1.8),(.25,5),(.15,1.8)]
INIT=np.array([np.log(4.),np.log(.7),np.log(2.),.55,.8,1.6,.50])

def param_vector(x):
    return GreyboxParams(gain=float(np.exp(x[0])),recessed_reflectance=float(np.exp(x[1])),
      specular_ratio=float(np.exp(x[2])),specular_roughness_rad=float(x[3]),
      footprint_sigma_cols=float(x[4]),min_component_area=float(x[5]),
      detection_softness=float(x[6]))

def valid_logloss(data,output,hole=False):
    use=data['hole_domain'] if hole else data['work_domain']
    y=data['real'][use].astype(float)
    p=np.clip(output['probability'][use],1e-8,1-1e-8)
    return float(np.mean(-y*np.log(p)-(1-y)*np.log1p(-p)))

def robust_depth_loss(data,pred,hole=False):
    # Jointly fit final output position and return-validity. Clip gross CAD
    # surface switching at 4 mm so an unknown secondary reflection cannot
    # dominate all (otherwise correct) correspondences.
    domain=data['hole_domain'] if hole else data['work_domain']
    sel=domain&data['real']&pred['active']&np.isfinite(pred['predicted_z'])
    if not np.any(sel):return 0.
    dz=np.minimum(np.abs(data['real_z'][sel]-pred['predicted_z'][sel]),4.)/.25
    huber=np.where(dz<=1.,.5*dz**2,dz-.5)
    return float(np.mean(huber))

def training_loss(datasets,vector,depth_weight=.02):
    p=param_vector(vector)
    vals=[]
    for d in datasets.values():
        pred=predict(d,p)
        vals.append(valid_logloss(d,pred,False) + 1.7*valid_logloss(d,pred,True)
                    + depth_weight*(robust_depth_loss(d,pred,False)+2*robust_depth_loss(d,pred,True)))
    # Regularizers: these are NOT independent physical priors. They prevent
    # equivalent intensity/threshold parametrizations from moving without data.
    reg=.004*(vector[0]-INIT[0])**2 +.002*(vector[1]-INIT[1])**2
    reg+=.002*(vector[2]-INIT[2])**2+.005*(vector[3]-INIT[3])**2
    reg+=.002*(vector[4]-INIT[4])**2+.002*(vector[5]-INIT[5])**2
    return float(np.mean(vals)+reg)

def baseline(data):
    v=data['valid_geom'];d=data['work_domain'];h=data['hole_domain'];real=data['real']
    def calc(sel):
        y=real[sel];b=v[sel];tp=(b&y).sum();fp=(b&~y).sum();fn=(~b&y).sum();tn=(~b&~y).sum()
        return {'n':int(sel.sum()),'tp':int(tp),'fp':int(fp),'fn':int(fn),'tn':int(tn),'f1':float(2*tp/max(1,2*tp+fp+fn))}
    return {'workpiece':calc(d),'holes':calc(h)}

def run(args):
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    train={k:prepare(Path(args.features)/f'{k}.csv') for k in ('02','04','05','06')}
    print('Training profiles', {k:len(v['label']) for k,v in train.items()},flush=True)
    calls=0;t0=time.time();best=[float('inf'),None]
    def fun(x):
        nonlocal calls
        v=training_loss(train,x,args.depth_weight);calls+=1
        if v<best[0]:
            best[:]=[v,x.copy()]
            if calls%8==0 or calls==1:
                print('iter',calls,'best training loss',round(v,5),'seconds',round(time.time()-t0,1),flush=True)
        return v
    # Explicit bounded derivative-free optimization. All candidate models are
    # scored in four sessions equally; no evaluation of 03 during parameter search.
    starts=[INIT.copy()]
    if args.restarts>=2:
        v=INIT.copy();v[2]=np.log(6);v[3]=.33;v[5]=2.;starts.append(v)
    if args.restarts>=3:
        v=INIT.copy();v[1]=np.log(.25);v[2]=np.log(8);starts.append(v)
    results=[]
    for i,x in enumerate(starts):
        res=minimize(fun,x,method='Powell',bounds=BOUNDS, options={
                'maxfev':args.maxfev,'maxiter':70,'xtol':.08,'ftol':.007})
        results.append({'restart':i,'success':bool(res.success),'nit':int(res.nit),'nfev':int(res.nfev),
                        'loss':float(res.fun),'status':str(res.message)})
    chosen=param_vector(best[1]);loss=best[0]
    samples={}
    for k,d in train.items():
        pp=predict(d,chosen)
        samples[k]={'previous_geometry':baseline(d),'v3_workpiece':score(d,pp),'v3_holes':score(d,pp,True)}
    # Freeze and serialize BEFORE accessing holdout. Do not tune later with 03.
    config={'type':'M34_V3_GREYBOX_IMAGING_RESEARCH',
       'train_sessions':list(train),'reserved_validation':'03','trained_by_03':False,
       'source_code':'tools/m34_greybox_imager.py',
       'parameters':asdict(chosen),'train_objective':loss,'joint_depth_weight':args.depth_weight,
       'warning':'effective sensor-model surrogate: no camera raw images or four-reflection data; not a physical calibration or actual firmware implementation',
       'gates':{'ready_for_m6':False,'physical_optics_certified':False,'image_psf_certified':False},
       'data_sha256':{k:hashlib.sha256((Path(args.features)/f'{k}.csv').read_bytes()).hexdigest() for k in train}}
    with (out/'v3_frozen_params.json').open('w',encoding='utf8') as f:json.dump(config,f,ensure_ascii=False,indent=2)
    d=prepare(Path(args.features)/'03.csv')
    p=predict(d,chosen)
    hold={'geometry':baseline(d),'v3_workpiece':score(d,p),'v3_holes':score(d,p,True)}
    result={'frozen_model':config,'train':samples,'test03':hold,'optimize':results,
            'evaluations':calls,'elapsed_seconds':time.time()-t0,
            'interpretation':'03 was not used to fit these params; however it was inspected in earlier project versions, so not a historically pristine blind holdout'}
    with (out/'v3_results.json').open('w',encoding='utf8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    pd.DataFrame([{'session':k,'part':part,**sc} for k,dct in samples.items() for part,sc in [('workpiece',dct['v3_workpiece']),('holes',dct['v3_holes'])]]+
                 [{'session':'03','part':'workpiece',**hold['v3_workpiece']},{'session':'03','part':'holes',**hold['v3_holes']}]).to_csv(out/'v3_metrics.csv',index=False)
    print('Frozen train loss',loss,'params',asdict(chosen),flush=True)
    for k,item in samples.items():print('TRAIN',k,'work F1',round(item['v3_workpiece']['f1'],4),'hole F1',round(item['v3_holes']['f1'],4))
    print('TEST 03',hold,flush=True)
    # Synthetic rendered signal is not the device raw image. Single profile demonstrator.
    if args.render:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        ar=np.flatnonzero(d['hole'].any(axis=1));frame=int(ar[0]) if len(ar) else 0
        signal,zs,cols=render_virtual_detector(d,chosen,frame,200,1080,rows=320)
        plt.figure(figsize=(14,4));plt.imshow(signal,origin='lower',aspect='auto',extent=(200,1080,zs[0],zs[-1]),cmap='gray')
        plt.xlabel('virtual profile column (index)');plt.ylabel('effective detector-depth [mm]')
        plt.title('SIMULATED proxy stripe image — NOT scanCONTROL camera image')
        plt.tight_layout();plt.savefig(out/'virtual_detector_demo.png',dpi=155);plt.close()
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--features',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--maxfev',type=int,default=145)
    parser.add_argument('--restarts',type=int,default=1)
    parser.add_argument('--render',action='store_true')
    parser.add_argument('--depth-weight',type=float,default=.02)
    run(parser.parse_args())
