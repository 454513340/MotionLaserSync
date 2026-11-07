#!/usr/bin/env python3
"""Optional NON-DEPLOYED specular-vector feature ablation on a fixed M34 geometry.
Compared on previously inspected 05/06; useful for failure analysis, NOT blind certification.
"""
import argparse,json,pathlib,numpy as np,pandas as pd
from scipy.optimize import minimize
from scipy.special import expit
from m34_fit_v2 import make_design,read_features,PRIORS,metrics

def train(d,extra):
    d=d.loc[d.fit].copy();X=make_design(d)
    if extra:X=np.column_stack((X,1-np.clip(np.nan_to_num(d.specular_alignment.to_numpy(),nan=0.),0.,1.)))
    prior=np.r_[PRIORS,(-1.,) if extra else ()];bounds=[(-6,9)]+[(-16,0)]*(len(prior)-1)
    y=(d.label.to_numpy()==1).astype(float);wt=np.where(d.hole.to_numpy()==1,3.,1.)
    def fn(w):
        z=np.clip(X@w,-40,40);p=expit(z);den=wt.sum();dw=w-prior
        return (float(np.dot(wt,np.logaddexp(0,z)-y*z)/den+.0007*(dw@dw)),
          X.T@(wt*(p-y))/den+.0014*dw)
    return minimize(fn,prior,method='L-BFGS-B',jac=True,bounds=bounds).x

def summary(d,w,extra):
    domain=d.loc[d.domain].copy();X=make_design(domain)
    if extra:X=np.column_stack((X,1-np.clip(np.nan_to_num(domain.specular_alignment.to_numpy(),nan=0.),0.,1.)))
    v=(domain.hit_workpiece.to_numpy()==1)&(domain.geometry_valid.to_numpy()==1)
    p=np.zeros(len(domain));p[v]=expit(X[v]@w)
    y=domain.label.to_numpy()==1;out={}
    for s in [2,3]:
        si=domain.session.to_numpy()==s;h=domain.hole.to_numpy()==1
        out[str(s)]={r:metrics(y[si&selected],p[si&selected]) for r,selected in [('workpiece',np.ones(len(domain),bool)),('hole',h)]}
    return out

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--features',type=pathlib.Path,required=True)
    ap.add_argument('--output',type=pathlib.Path,required=True)
    args=ap.parse_args();d=read_features(args.features)
    trainSet=d[d.session.isin([0,1])];hold=d[d.session.isin([2,3])]
    if len(trainSet)==0 or len(hold)==0:raise ValueError('Expected two train + two validation sessions in features')
    report={'notice':'Exploratory, tested after 05/06 were inspected; not independent blind validation'}
    for extra in [False,True]:
        w=train(trainSet,extra)
        report['geometric_risk_plus_specular' if extra else 'geometric_risk_only']={
            'weights':w.tolist(),'holdout':summary(hold,w,extra)}
    args.output.write_text(json.dumps(report,indent=2),encoding='utf8')
    print('Specular proxy ablation saved:',args.output)
if __name__=='__main__':main()
