#!/usr/bin/env python3
"""M3/M4 V2 reproducible research calibration: geometry grid + constrained
probability-of-return model. No hardware, no claims of camera physical extrinsics.
Train session set ONLY selects model; holdouts remain quarantined until selection.
Visibility and incidence features are provided by actual C++ VirtualLaser2950.
"""
import argparse,hashlib,json,pathlib,subprocess,sys
import numpy as np
import pandas as pd
from scipy.optimize import minimize
from scipy.special import expit

FEATURES=['grazing_laser','grazing_receiver','depth_risk','field_risk','boundary_risk','hole_risk']
PRIORS=np.array([4.,-3.,-3.,-.25,-.2,-.7,-1.],float)
BOUNDS=[(-6,9)]+[(-16,0)]*6

def make_design(table):
    def clean(x): return np.nan_to_num(np.asarray(x,dtype=float),nan=0,posinf=0,neginf=0)
    ca=np.clip(clean(table.cos_laser),0,1)
    cr=np.clip(clean(table.cos_receiver),0,1)
    z=np.clip(clean(table.depth_risk),0,3)
    field=np.clip(clean(table.field_risk),0,3)
    bd=np.clip(clean(table.boundary_risk),0,1)
    hole=np.asarray(table.hole,dtype=float)
    return np.column_stack([np.ones(len(table)),1-ca,1-cr,z,field,bd,hole])

def read_features(file):
    d=pd.read_csv(file)
    required={'session','frame','index','label','hole','geometry_valid','hit_workpiece','hit_support','state',
              'real_z','pred_z','cos_laser','cos_receiver','depth_risk','field_risk','boundary_risk'}
    if not required.issubset(set(d)):raise ValueError('Missing columns: '+str(required-set(d)))
    d['domain']=((d.hit_workpiece.eq(1)|d.label.eq(1))&~d.label.isin([2,3]))
    d['fit']=((d.geometry_valid.eq(1)&d.hit_workpiece.eq(1))&d.label.isin([0,1]))
    d['real_workpiece']=d.label.eq(1)
    return d

def fit_weights(d):
    sel=d['fit'].to_numpy()
    data=d.loc[sel]
    if len(data)<100 or data.real_workpiece.nunique()<2:raise ValueError('Insufficient class diversity in training')
    X=make_design(data)
    y=data.real_workpiece.astype(float).to_numpy()
    sample_weight=np.where(data.hole.to_numpy()>0,3.,1.)
    den=sample_weight.sum()
    reg=.0007
    def objective(w):
        logits=np.clip(X@w,-40,40)
        probs=expit(logits)
        loss=(sample_weight*np.logaddexp(0,logits)-sample_weight*y*logits).sum()/den
        grad=X.T@(sample_weight*(probs-y))/den
        # Independent per-parameter weak Gaussian priors / Tikhonov regularization.
        dx=w-PRIORS
        loss+=reg*(dx@dx)
        grad+=2*reg*dx
        return float(loss),grad
    res=minimize(objective,PRIORS,method='L-BFGS-B',jac=True,bounds=BOUNDS,options={'maxiter':220,'ftol':1e-10})
    return res.x,{'success':bool(res.success),'iterations':int(res.nit),'train_weighted_log_loss_regularized':float(res.fun),
                  'labelled_fit_samples':int(len(data)),'positive_fit_samples':int(y.sum()),
                  'weights':dict(zip(['bias']+FEATURES,map(float,res.x))),'optimizer_message':str(res.message)}

def metrics(y,p,cut=.5):
    y=np.asarray(y,bool);p=np.clip(np.asarray(p,float),1e-9,1-1e-9);pred=p>=cut
    tp=int((pred&y).sum()); fp=int((pred&~y).sum());fn=int((~pred&y).sum());tn=int((~pred&~y).sum())
    denom=2*tp+fp+fn
    return {'total':len(y),'TP':tp,'FP':fp,'FN':fn,'TN':tn,
            'precision':tp/(tp+fp) if tp+fp else 0.,'recall':tp/(tp+fn) if tp+fn else 0.,
            'f1':2*tp/denom if denom else 0.,'brier':float(np.mean((y-p)**2)) if len(y) else None,
            'logloss':float(-np.mean(y*np.log(p)+(1-y)*np.log1p(-p))) if len(y) else None}

def evaluate(d,w):
    idx=d.domain.to_numpy()
    d=d.loc[idx].copy()
    if not len(d):return {'error':'Empty CAD-relative evaluation domain'}
    X=make_design(d)
    predGeom=((d.geometry_valid.to_numpy()==1)&(d.hit_workpiece.to_numpy()==1))
    p=np.zeros(len(d),float)
    p[predGeom]=expit(X[predGeom]@w)
    old=predGeom.astype(float)
    y=d.real_workpiece.to_numpy()
    result={'cad_domain_samples':int(len(d)),'real_workpiece':int(y.sum()),
            'geom_only':metrics(y,old),'v2':metrics(y,p),
            'structural_false_negative':int(((~predGeom)&y).sum()),
            'outside_cad_excluded':int((~idx).sum()),
            'real_cad_far_excluded':None} # per-split exclusion count is populated by caller
    result['by_session']={}
    for si,group in d.groupby('session',sort=True):
        j=group.index
        loc=d.index.get_indexer(j)
        maskHole=group.hole.to_numpy()==1
        def section(sel):return {'geom_only':metrics(y[loc][sel],old[loc][sel]),'v2':metrics(y[loc][sel],p[loc][sel])}
        bothValid=maskHole & group.real_workpiece.to_numpy() & (group.geometry_valid.to_numpy()==1)
        zerr=(group.real_z.to_numpy()-group.pred_z.to_numpy())
        zvalid=zerr[np.isfinite(zerr)&bothValid]
        result['by_session'][str(int(si))]={**section(np.ones(len(group),bool)),
            'holes':section(maskHole),'hole_valid_paired':int(len(zvalid)),
            'hole_z_mae_mm':float(np.mean(np.abs(zvalid))) if len(zvalid) else None,
            'real_cad_near':int(group.real_workpiece.sum())}
    return result

def run_exe(exe,obj,pose,train,output,y,z,fanScale=1.,fanOffset=0.):
    command=[str(exe),'--obj',str(obj),'--pose',*map(str,pose),
             '--camera-x','0','--camera-y',str(y),'--camera-z',str(z),
             '--fan-scale',str(fanScale),'--fan-offset',str(fanOffset)]
    for src in train:command.extend(['--source',str(src)])
    command.extend(['--output',str(output)])
    subprocess.run(command,check=True)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--exe',type=pathlib.Path,required=True)
    ap.add_argument('--obj',type=pathlib.Path,required=True)
    ap.add_argument('--pose',nargs=6,type=float,required=True)
    ap.add_argument('--train',nargs='+',type=pathlib.Path,required=True)
    ap.add_argument('--holdout',nargs='+',type=pathlib.Path,required=True)
    ap.add_argument('--output',type=pathlib.Path,required=True)
    ap.add_argument('--fan-json',type=pathlib.Path,required=True,help='02/04-only empirical virtual index->X calibration JSON')
    ap.add_argument('--acquisition-config',type=pathlib.Path,help='Recorded acquisition metadata, not auto-estimated exposure parameters')
    ap.add_argument('--keep-features',action='store_true')
    ap.add_argument('--refine-geometry',action='store_true',help='Bounded Powell refinement on 02/04 ONLY')
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    if len(a.train)<2 or set(a.train)&set(a.holdout):raise ValueError('Need >=2 training sessions and disjoint holdouts')
    fan=json.loads(a.fan_json.read_text())
    fanScale,fanOffset=float(fan['fan_scale']),float(fan['fan_offset_mm'])
    # A pre-declared, coarse, constrained *equivalent* receiver search. These are
    # NOT physically identifiable camera coordinates from scan profiles alone.
    candidates=[(75.,251.25),(90.,251.25),(105.,251.25),(90.,231.25),(90.,271.25)]
    fit_results=[];best=None
    for i,(y,z) in enumerate(candidates):
        feature=a.output/f'candidate_{i:02d}_train.csv'
        print('M3 V2 candidate',i,'receiver Y,Z=',y,z,flush=True)
        run_exe(a.exe,a.obj,a.pose,a.train,feature,y,z,fanScale,fanOffset)
        d=read_features(feature)
        w,opt=fit_weights(d)
        evaluation=evaluate(d,w)
        # Comparable across candidates: weighted log-likelihood plus penalized
        # true CAD returns missed by any laser-camera first-hit geometry.
        missed=evaluation['structural_false_negative']
        positive=evaluation['real_workpiece']
        penalty=2*missed/max(1,positive)
        # weak physical prior guards against optimizing arbitrary receiver pose
        prior=.00003*((y-75.)**2+(z-251.25)**2)
        score=opt['train_weighted_log_loss_regularized']+penalty+prior
        line={'receiver_Y_mm':y,'receiver_Z_mm':z,'train_score':float(score),
              'missed_workpiece_penalty':float(penalty),'geometry_prior_penalty':float(prior),
              'fit':opt,'train':evaluation}
        fit_results.append(line)
        print('  train score=',round(score,6),'F1 new=',round(evaluation['v2']['f1'],4),
              'baseline=',round(evaluation['geom_only']['f1'],4),'structFN=',missed,flush=True)
        if best is None or score<best[0]:best=(score,y,z,w,i)
        if not a.keep_features:feature.unlink()
    _,by,bz,bw,bi=best
    refined=[]
    if a.refine_geometry:
        # Powell uses no differentiability assumption for visibility changes.
        # Objective can have discontinuities at CAD triangle/occlusion edges.
        cache={}
        def goal(x):
            y,z=float(x[0]),float(x[1])
            key=(round(y,3),round(z,3))
            if key in cache:return cache[key][0]
            name=a.output/f'refine_{len(cache):02d}.csv'
            run_exe(a.exe,a.obj,a.pose,a.train,name,y,z,fanScale,fanOffset)
            dt=read_features(name)
            wt,op=fit_weights(dt)
            ev=evaluate(dt,wt)
            penalty=2*ev['structural_false_negative']/max(1,ev['real_workpiece'])
            prior=.00003*((y-75.)**2+(z-251.25)**2)
            total=float(op['train_weighted_log_loss_regularized']+penalty+prior)
            cache[key]=(total,wt)
            refined.append({'Y':y,'Z':z,'train_score':total,'structural_FN':ev['structural_false_negative']})
            if not a.keep_features:name.unlink()
            print('REFINE',y,z,'score',total,flush=True)
            return total
        res=minimize(goal,np.array([by,bz]),method='Powell',bounds=[(65,110),(231,271)],
                     options={'maxfev':28,'xtol':.8,'ftol':.003})
        if float(res.fun)<best[0]:
            _,by,bz=best[:3]
            by,bz=map(float,res.x)
            goal([by,bz])
            bw=cache[(round(by,3),round(bz,3))][1]
            bi='powell_refined'
            best=(float(res.fun),by,bz,bw,bi)
            print('SELECTED REFINED',by,bz,'score',res.fun,flush=True)
    # Portable, read-only C++ inference parameters with explicit research tag.
    parameter_path=a.output/'M34_V2_RESEARCH_NOT_CERTIFIED.m34cfg'
    parameter_path.write_text('M34P2\n'+f'0 {by:.12g} {bz:.12g}\n'+
          ' '.join(f'{x:.12g}' for x in bw)+'\n'+
          f'{fanScale:.12g} {fanOffset:.12g}\n',encoding='utf8')
    # Cluster bootstrap at PROFILE level (not pretending 1280 samples are
    # independent). Estimates *within-training-session* numerical stability.
    selected_features=a.output/'chosen_train_for_uncertainty.csv'
    run_exe(a.exe,a.obj,a.pose,a.train,selected_features,by,bz,fanScale,fanOffset)
    trainframe=read_features(selected_features)
    result_training=evaluate(trainframe,bw)
    rng=np.random.default_rng(20261017)
    groups=[np.asarray(v,dtype=int) for v in trainframe.groupby(['session','frame'],sort=True).indices.values()]
    groups_by_session=[]
    for _,g in trainframe.groupby('session',sort=True):
        indices=list(g.groupby('frame').indices.values())
        groups_by_session.append([g.index.to_numpy()[np.asarray(x,dtype=int)] for x in indices])
    boots=[]
    for k in range(20):
        selected=[]
        for g in groups_by_session:
            chosen=rng.integers(0,len(g),size=len(g))
            selected.extend(g[idx] for idx in chosen)
        ds=trainframe.iloc[np.concatenate(selected)]
        try:
            wboot,bst=fit_weights(ds)
            if bst['success']:boots.append(wboot)
        except ValueError:
            continue
    conf={}
    if boots:
        quant=np.quantile(np.asarray(boots),[.025,.5,.975],axis=0)
        for j,name in enumerate(['bias']+FEATURES):conf[name]=[float(x) for x in quant[:,j]]
    if not a.keep_features:selected_features.unlink()
    result={'version':'M3M4_V2_FOUR_TRAIN_ONE_HOLDOUT_2026_10','selected_by_training_only':True,
        'best_candidate_index':bi,'receiver_equivalent_mm':{'x':0.,'y':by,'z':bz},
        'empirical_fan_x_calibration':{'scale':fanScale,'offset_mm':fanOffset,'source':str(a.fan_json)},
        'trained_return_model':dict(zip(['bias']+FEATURES,map(float,bw))),
        'model_semantics':'hard first-hit illumination+receiver-occlusion; conditional logistic probability on CAD target',
        'gates':{'physical_calibration_certified':False,'ready_for_m6':False,
                 'exposure_and_reflectivity_calibrated':False},
        'training_candidates':fit_results,'powell_refinement_train_only':refined,
        'train_sessions':[x.name for x in a.train],
        'selected_model_training_evaluation':result_training,
        'acquisition_config':json.loads(a.acquisition_config.read_text(encoding='utf8')) if a.acquisition_config else None,
        'holdout_sessions':[x.name for x in a.holdout],
        'evaluation_note':'CAD-far samples excluded from object F1, NOT labelled as confirmed fixture; no-return cause unknown; missing CAD hit counted',
        'pose_xyz_rxryrz':a.pose,
        'cad_obj_sha256':hashlib.sha256(a.obj.read_bytes()).hexdigest(),
        'portable_cpp_parameter_file':parameter_path.name,
        'profile_bootstrap_fit_count':len(boots),'profile_bootstrap_coefficients_q025_q50_q975':conf,
        'uncertainty_warning':'Profile bootstrap covers observed training sessions only, not physical or cross-material uncertainties'}
    f=a.output/'chosen_holdout.csv'
    run_exe(a.exe,a.obj,a.pose,a.holdout,f,by,bz,fanScale,fanOffset)
    hold=read_features(f)
    result['holdout']=evaluate(hold,bw)
    # Execute the newly introduced portable C++ inference on the selected
    # parameter file and compare probability outputs against Python exactly.
    check=a.output/'cpp_inference_crosscheck.csv'
    cross=[str(a.exe),'--obj',str(a.obj),'--pose',*map(str,a.pose),
           '--camera-x','0','--camera-y',str(by),'--camera-z',str(bz),
           '--fan-scale',str(fanScale),'--fan-offset',str(fanOffset)]
    for fsrc in a.holdout:cross.extend(['--source',str(fsrc)])
    cross.extend(['--params',str(parameter_path),'--output',str(check)])
    subprocess.run(cross,check=True)
    cpp=pd.read_csv(check)
    work=(cpp.geometry_valid.eq(1)&cpp.hit_workpiece.eq(1)).to_numpy()
    xp=make_design(cpp)
    predicted=np.zeros(len(cpp),float);predicted[work]=expit(xp[work]@bw)
    maxerr=float(np.max(np.abs(np.nan_to_num(cpp.v2_probability.to_numpy(),nan=0)-predicted)))
    result['cpp_python_probability_max_abs_error']=maxerr
    result['cpp_python_probability_agreement_pass']=bool(maxerr<5e-6)
    if not result['cpp_python_probability_agreement_pass']:
        raise AssertionError(f'C++ vs Python probability mismatch: {maxerr}')
    if not a.keep_features:check.unlink()
    result['holdout']['excluded_CAD_far_return_samples']=int(hold.label.eq(2).sum())
    result['holdout']['excluded_ambiguous_return_samples']=int(hold.label.eq(3).sum())
    # Baseline same evaluation domain but V1's deterministic geometric 0/1 return.
    b=a.output/'old_baseline_holdout.csv'
    run_exe(a.exe,a.obj,a.pose,a.holdout,b,90.,251.25,1.,0.)
    prior=read_features(b)
    result['old_model_baseline_holdout']=evaluate(prior,np.r_[100.,np.zeros(6)])
    result['old_model_baseline_holdout']['excluded_CAD_far_return_samples']=int(prior.label.eq(2).sum())
    result['old_model_baseline_holdout']['excluded_ambiguous_return_samples']=int(prior.label.eq(3).sum())
    # Model-improvement cannot be called a clean blind holdout once inspected
    # during prior rounds; record leakage honestly.
    result['holdout_blind_protocol']='03 session quarantined: metadata/CRC inspected for import repair; no 03 return/geometry metrics consulted for selection; model and frozen parameters selected exclusively from 02/04/05/06.'
    if not a.keep_features:f.unlink();b.unlink()
    (a.output/'M3_M4_V2_fit_report.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf8')
    print('CHOSEN',bi,by,bz,'holdout F1 geometry=',result['holdout']['geom_only']['f1'],
        'conditional probability=',result['holdout']['v2']['f1'],flush=True)
if __name__=='__main__':main()
