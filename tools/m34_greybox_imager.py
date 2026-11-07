#!/usr/bin/env python3
"""Research M3/M4 V3 grey-box stripe/selection surrogate (offline only).

This is NOT reverse-engineered Micro-Epsilon firmware. The actual sensor images,
internal camera intrinsics, radiometric gain, noise and 4-reflection data were not
archived in SCAN v3, so intensity and width parameters are EFFECTIVE, bounded,
not independently physically calibrated. Uses CAD ray features from the unchanged
C++ first-hit / receiver-visibility backend. A finite lateral footprint introduces
multi-peak competing STRIPE HYPOTHESES near CAD depth discontinuities; it does
NOT synthesize verified multi-bounce reflections.

Layers: CAD ray hits -> constrained diffuse+specular scattering -> finite laser /
camera PSF along synthetic detector depth -> background/relative threshold ->
peak-area competition ("largest area" proxy) -> subpixel moment -> calibrated Z.
Everything is deterministic, with uncertainty only in the observation likelihood.
"""
from __future__ import annotations
from dataclasses import dataclass,asdict
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.special import erf,expit

PROFILE_WIDTH=1280
TRAIN_SESSIONS=('02','04','05','06')
EPS=1e-9

@dataclass(frozen=True)
class GreyboxParams:
    gain:float=4.0               # normalized effective photon/threshold G*E*rho_top (not physical)
    recessed_reflectance:float=.60 # albedo ratio interior/top, no real reflectance calibration
    specular_ratio:float=1.7     # strength relative to diffuse channel
    specular_roughness_rad:float=.5 # angular lobe width, effective only
    footprint_sigma_cols:float=.8 # lateral laser line / image coupling in index columns
    min_component_area:float=1.5 # normalized above-threshold area (not manufacturer physical units)
    detection_softness:float=.45 # fitted effective uncertainty (not readout noise variance)
    laser_cos_power:float=1.0   # fixed physical-inspired incidence term
    receiver_cos_power:float=1.0
    detector_sigma_z_mm:float=.25 # virtual detector depth kernel, SCALE NOT CALIBRATED
    dynamic_threshold_fraction:float=.125 # setting from screenshot; interpretation assumed
    absolute_floor:float=1.0 # fixes non-identifiable overall intensity gauge
    background_level:float=0.0 # nominal after background suppression, cannot infer from profiles
    def check(self):
        assert .02<=self.gain<=50 and .03<=self.recessed_reflectance<=2
        assert 0<=self.specular_ratio<=30 and .08<=self.specular_roughness_rad<=2.5
        assert .15<=self.footprint_sigma_cols<=3 and .05<=self.min_component_area<=20
        assert .05<=self.detection_softness<=4 and .01<=self.detector_sigma_z_mm<=2
        assert 0<=self.dynamic_threshold_fraction<1


def prepare(path:Path)->dict:
    d=pd.read_csv(path)
    if len(d)%PROFILE_WIDTH:raise ValueError('Dataset is not whole profiles')
    if not np.array_equal(d['index'].to_numpy().reshape(-1,PROFILE_WIDTH)[0],np.arange(PROFILE_WIDTH)):
        raise ValueError('Profile ordering / index mismatch')
    shape=(-1,PROFILE_WIDTH)
    a={key:np.asarray(d[key]).reshape(shape) for key in [
        'label','state','geometry_valid','hit_workpiece','hole','pred_z','real_z',
        'cos_laser','cos_receiver','specular_alignment','hit_cad_z']}
    a['work_domain']=(a['hit_workpiece']==1)|(a['label']==1)
    a['work_domain'] &= (a['label']<2)
    a['hole_domain']=a['work_domain']&(a['hole']==1)
    a['real']=(a['label']==1)
    a['valid_geom']=(a['geometry_valid']==1)&(a['hit_workpiece']==1)
    a['specular_alignment']=np.nan_to_num(a['specular_alignment'],nan=0)
    a['cos_laser']=np.nan_to_num(a['cos_laser'],nan=0)
    a['cos_receiver']=np.nan_to_num(a['cos_receiver'],nan=0)
    a['pred_z']=np.where(np.isfinite(a['pred_z']),a['pred_z'],0)
    a['recessed']=(a['hole']==1)&(a['hit_cad_z']<9.25)
    a['nprofiles']=a['label'].shape[0]
    return a


def shifted(arr:np.ndarray,offset:int,fill=0):
    # Returns source i+offset in target column i, never crosses profiles.
    if offset==0:return arr
    x=np.empty_like(arr)
    if offset>0:
        x[:,:-offset]=arr[:,offset:]
        x[:,-offset:]=fill
    else:
        n=-offset
        x[:,n:]=arr[:,:-n]
        x[:,:n]=fill
    return x


def gaussian_component_area(peak:np.ndarray,width:np.ndarray,threshold:np.ndarray):
    """Integrated (intensity-threshold)+ for Gaussian peak: area, width proxy.
       Exact for an isolated one-dimensional Gaussian, approximate with overlap.
    """
    ratio=np.maximum(peak,0)/(np.maximum(threshold,EPS))
    good=ratio>1
    t=np.sqrt(np.maximum(np.log(np.maximum(ratio,1)),0))
    area=peak*width*np.sqrt(2*np.pi)*erf(t)-2*width*threshold*np.sqrt(2)*t
    return np.where(good,np.maximum(area,0),0)


def predict(a:dict,pars:GreyboxParams,detail:bool=False):
    """Finite footprint creates up to 3 CAD-direct stripe hypotheses / column.
    Each hypothesis has CAD Z; Gaussian detector lobe; local threshold selects
    the largest thresholded area; centroid provides an approximate subpixel Z.
    """
    pars.check()
    ok=a['valid_geom']
    cl=np.clip(a['cos_laser'],0,1)
    cr=np.clip(a['cos_receiver'],0,1)
    spec=np.clip(a['specular_alignment'],0,1)
    angular=np.arccos(spec)
    # Effective BRDF mixture; no claim of known BRDF for 6061/alumina.
    lobe=np.exp(-.5*(angular/pars.specular_roughness_rad)**2)
    brdf=(1+pars.specular_ratio*lobe)*np.maximum(cl,.005)**pars.laser_cos_power * np.maximum(cr,.005)**pars.receiver_cos_power
    rho=np.where(a['recessed'],pars.recessed_reflectance,1.)
    src=pars.gain*rho*brdf*ok
    # Lateral spot footprint: neighboring CAD first-hits can form separate
    # depth peaks at hole/step edges. No unverified second-bounce geometry.
    offsets=(-1,0,1)
    weights=np.asarray([np.exp(-.5*(k/pars.footprint_sigma_cols)**2) for k in offsets]);weights/=weights.sum()
    amp=np.stack([shifted(src,k)*w for k,w in zip(offsets,weights)],axis=-1)
    zz=np.stack([shifted(a['pred_z'],k) for k in offsets],axis=-1)
    cosl=np.stack([shifted(cl,k) for k in offsets],axis=-1)
    present=np.stack([shifted(ok,k) for k in offsets],axis=-1)
    # Effective broadened spot in the synthetic depth coordinate; increases
    # for grazing geometry (no physical sensor-pixel calibration available).
    widths=pars.detector_sigma_z_mm*(1+.7*(1-cosl))
    widths=np.maximum(widths,.01)
    # Approximate intensity at each candidate center after Gaussian PSF overlap.
    delta=zz[:,:,:,None]-zz[:,:,None,:]
    sig=widths[:,:,None,:]
    mixed=np.sum(amp[:,:,None,:]*np.exp(-.5*(delta/np.maximum(sig,EPS))**2),axis=-1)
    mixed=np.where(present,mixed,0)
    # Per-column threshold: dynamic fraction of strongest lobe plus an unknown
    # effective electronics floor. min_area & gain are gauge-fixed/normalized.
    peak=np.max(mixed,axis=-1)
    threshold=np.maximum(pars.absolute_floor,pars.dynamic_threshold_fraction*peak)
    q=gaussian_component_area(mixed,widths,threshold[:,:,None]);q=np.where(present,q,0)
    pick=np.argmax(q,axis=-1)
    qbest=np.take_along_axis(q,pick[:,:,None],axis=-1)[...,0]
    zbest=np.take_along_axis(zz,pick[:,:,None],axis=-1)[...,0]
    # Moment of overlapped lobes around the winning peak, subpixel proxy.
    near=np.exp(-.5*((zz-zbest[:,:,None])/np.maximum(widths,EPS))**2)*amp
    total=near.sum(axis=-1)
    zcentroid=np.where(total>EPS,(near*zz).sum(axis=-1)/np.maximum(total,EPS),np.nan)
    active=np.any(present,axis=-1)
    p=expit((qbest-pars.min_component_area)/pars.detection_softness)
    p=np.where(active,p,0.)
    out={'probability':p,'predicted_z':zcentroid,'active':active,'area':qbest,
         'valid':(p>=.5)&active}
    if detail:
        out.update(candidate_amplitude=amp,candidate_depth=zz,candidate_areas=q,candidate_peak=mixed,candidate_widths=widths,
                   selected_candidate=pick,threshold=threshold,beam_source_intensity=src)
    return out


def score(data:dict,pred:dict,require_hole=False):
    mask=data['hole_domain'] if require_hole else data['work_domain']
    y=data['real'][mask]
    p=np.clip(pred['probability'][mask],1.e-8,1-1.e-8)
    b=p>=.5
    tp=int((b & y).sum());fp=int((b & ~y).sum())
    fn=int((~b & y).sum());tn=int((~b & ~y).sum())
    zerr=np.abs(data['real_z'][mask] - pred['predicted_z'][mask]);zmask=b&y&np.isfinite(zerr)
    return {'n':int(len(y)),'positives':int(y.sum()),'TP':tp,'FP':fp,'FN':fn,'TN':tn,
            'f1':2*tp/max(1,2*tp+fp+fn),'precision':tp/max(1,tp+fp),'recall':tp/max(1,tp+fn),
            'brier':float(np.mean((y-p)**2)),
            'logloss':float(np.mean(np.logaddexp(0,np.log(p/(1-p)))-y*np.log(p/(1-p)))) if len(y) else None,
            'paired_z_mae_mm':float(zerr[zmask].mean()) if np.any(zmask) else None}


def render_virtual_detector(a:dict,p:GreyboxParams,frame:int,lo:int=0,hi:int=1280,
                            z_min=69.,z_max=121.,rows=520):
    """Synthetic grayscale strip (index column vs effective depth), diagnostic.
    This is NOT a reproduced scanCONTROL CMOS image; no image data is in SCAN.
    """
    pred=predict(a,p,detail=True)
    zs=np.linspace(z_min,z_max,rows)
    cols=np.arange(lo,hi)
    centers=pred['candidate_depth'][frame,lo:hi]
    weights=pred['candidate_amplitude'][frame,lo:hi]
    sigmas=pred['candidate_widths'][frame,lo:hi]
    # Avoid an actual full-SDK Bayer/raw sensor matrix; synthetic 1D columns.
    signal=np.sum(weights[None,:,:]*np.exp(-.5*((zs[:,None,None]-centers[None,:,:])/sigmas[None,:,:])**2),axis=2)
    signal=np.minimum(signal+p.background_level,1023.)
    return signal,zs,cols


def extract_profile_from_synthetic_image(image:np.ndarray,z_axis_mm:np.ndarray,
                                          threshold_fraction:float=.125,
                                          absolute_floor:float=1.,
                                          min_component_width_px:int=2,
                                          max_component_width_px:int=256,
                                          min_component_area:float=0.,
                                          reflection_selection:str='largest_area',
                                          filter_sigma_px:float=0.,
                                          background:float=0.,
                                          sample_count:int=1280,
                                          index_offset:int=0,
                                          fan_scale:float=1.,
                                          fan_offset_mm:float=0.):
    """Explicit per-column image -> reflection component -> subpixel profile.

    Diagnostic surrogate, not micro-epsilon SDK/firmware. Image rows denote an
    arbitrary equivalent *detector-depth* coordinate (not actual CMOS pixels).
    Relative threshold interpretation is an unverified hypothesis. Emits a list
    of reflection components and chosen Z per index. No hardware operations.
    """
    from scipy.ndimage import gaussian_filter1d
    pixels=np.asarray(image,float)
    z=np.asarray(z_axis_mm,float)
    if pixels.ndim!=2 or len(z)!=pixels.shape[0] or not np.all(np.diff(z)>0):
        raise ValueError('image must be [depth rows, index columns] and Z ascending')
    if reflection_selection not in {'largest_area','highest_intensity','first','last','only_single'}:
        raise ValueError('Unknown reflection choice')
    if not (0<=threshold_fraction<1 and absolute_floor>=0):raise ValueError('Threshold invalid')
    signal=np.maximum(pixels-background,0)
    dz=float(np.median(np.diff(z)))
    if filter_sigma_px>0:signal=gaussian_filter1d(signal,filter_sigma_px,axis=0,mode='nearest')
    depth_cols=signal.shape[1]
    selected=np.full(depth_cols,np.nan)
    retained=np.zeros(depth_cols,dtype=bool)
    all_reflections=[]
    for i in range(depth_cols):
        col=signal[:,i]
        thr=max(absolute_floor,threshold_fraction*float(col.max()))
        mask=col>thr
        # 1D connected components of thresholded bright stripe along detector Z.
        starts=np.where(mask & ~np.r_[False,mask[:-1]])[0]
        ends=np.where(mask & ~np.r_[mask[1:],False])[0]+1
        peaks=[]
        for s,e in zip(starts,ends):
            if not min_component_width_px<=e-s<=max_component_width_px:continue
            area=float(np.sum(col[s:e]-thr)*dz)
            if area<min_component_area:continue
            intens=col[s:e]-thr
            center=float(np.dot(z[s:e],intens)/max(np.sum(intens),EPS))
            peaks.append({'z':center,'area':area,'peak':float(col[s:e].max()),'width_pixels':int(e-s),'begin_row':int(s),'end_row':int(e)})
        all_reflections.append(peaks)
        if not peaks:continue
        if reflection_selection=='only_single' and len(peaks)!=1:continue
        if reflection_selection=='largest_area':choice=max(peaks,key=lambda v:v['area'])
        elif reflection_selection=='highest_intensity':choice=max(peaks,key=lambda v:v['peak'])
        elif reflection_selection=='last':choice=max(peaks,key=lambda v:v['z'])
        else:choice=min(peaks,key=lambda v:v['z'])
        selected[i]=choice['z'];retained[i]=True
    # Output the same type of final Sensor-X/Z profile as an LLT profile.
    # Virtual calibration is deliberately separate from the latent CMOS scale.
    outx=np.full(depth_cols,np.nan)
    if np.any(retained):
        iz=np.where(retained)[0]
        zv=selected[iz]
        widths=np.interp(zv,[70.,95.,120.],[58.,50.,42.])
        u=(iz+index_offset+.5)/float(sample_count)-.5
        outx[iz]=-u*widths*fan_scale+fan_offset_mm
    return {'valid':retained,'sensor_x':outx,'sensor_z':selected,'all_reflections':all_reflections}
