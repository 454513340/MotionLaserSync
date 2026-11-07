#!/usr/bin/env python3
"""Restricted ablation — uses training sessions only to refit effective gain."""
from pathlib import Path
from dataclasses import replace
import json,argparse,csv
import numpy as np
from scipy.optimize import minimize_scalar
from m34_greybox_imager import prepare,GreyboxParams,predict,score
from m34_fit_greybox import valid_logloss,robust_depth_loss

ap=argparse.ArgumentParser();ap.add_argument('--features',type=Path,required=True);ap.add_argument('--params',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
a=ap.parse_args();p=GreyboxParams(**json.loads(a.params.read_text())['parameters'])
tr={k:prepare(a.features/(k+'.csv')) for k in ('02','04','05','06')};test=prepare(a.features/'03.csv')
mods={'full_v3':p,'no_specular_lobe':replace(p,specular_ratio=0.),
 'no_recessed_albedo':replace(p,recessed_reflectance=1.),
 'narrow_single_beam':replace(p,footprint_sigma_cols=.3)}
rows=[]
for name,params in mods.items():
    if name!='full_v3':
        def loss(g):
            pp=replace(params,gain=float(np.exp(g)))
            return float(np.mean([valid_logloss(d,predict(d,pp),False)+1.7*valid_logloss(d,predict(d,pp),True) for d in tr.values()]))
        result=minimize_scalar(loss,method='bounded',bounds=(np.log(.6),np.log(25)),options={'xatol':.07,'maxiter':18})
        params=replace(params,gain=float(np.exp(result.x)))
    for sid,d in list(tr.items())+[('03',test)]:
        pred=predict(d,params)
        for tag,hole in [('workpiece',False),('holes',True)]:
            metrics=score(d,pred,hole)
            rows.append({'ablation':name,'session':sid,'roi':tag,'gain_retuned':params.gain,
                         **metrics})
    print(name,'gain=',params.gain,'05_holes_f1=',[x['f1'] for x in rows if x['ablation']==name and x['session']=='05' and x['roi']=='holes'][0],flush=True)
a.output.parent.mkdir(parents=True,exist_ok=True)
with a.output.open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
