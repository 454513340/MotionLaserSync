#!/usr/bin/env python3
"""Run the vetted OFFLINE M0-M5 pipeline on four new H1 scan sessions.
1) validated STEP SHA + SCAN CRC + per-exposure XYZCB transforms;
2) independent-session CAD pose registration (requires coarse starting pose);
3) M5 per-scan/pooled true-return coverage and surface cells;
4) M3/M4 multi-view Y-baseline candidate fit with held-out sessions.
No GUI, controller, sensor or M6 hardware commands.
"""
import argparse,json,pathlib,subprocess,sys
P=pathlib.Path(__file__).resolve().parents[1]
def run(args):
 print('RUN:', ' '.join(map(str,args)),flush=True)
 subprocess.run(list(map(str,args)),check=True)
def main():
 ap=argparse.ArgumentParser(description=__doc__)
 ap.add_argument('--scan',nargs=4,type=pathlib.Path,required=True,help='4 files in order: train train holdout holdout')
 ap.add_argument('--step',type=pathlib.Path,required=True)
 ap.add_argument('--init',nargs=6,required=True,type=float,metavar=('X','Y','Z','RX','RY','RZ'))
 ap.add_argument('--out',type=pathlib.Path,required=True)
 ap.add_argument('--build',type=pathlib.Path,default=P/'build-cli')
 ap.add_argument('--stride',type=int,default=1)
 a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
 mesh=P/'examples/digital_twin/6061_sensor_connector_NEW_2026.obj'
 if not mesh.is_file():raise FileNotFoundError(mesh)
 cli=lambda tool:a.build/tool
 for exe in ('twin_m0m5_multiview_cli','twin_m0m5_optical_cli'):
  if not cli(exe).is_file():raise FileNotFoundError('Build with DIGITAL_TWIN_CLI_ONLY=ON before running: '+str(cli(exe)))
 run([sys.executable,P/'tools/m0m5_prepare_scans.py','--step',a.step,'--scan',*a.scan,'--output',a.out,'--stride',a.stride])
 m5=[a.out/(p.stem+'.m5s1') for p in a.scan]
 m4=[a.out/(p.stem+'.m4s1') for p in a.scan]
 registration=a.out/'M1_registration.json'
 run([sys.executable,P/'tools/m0m5_register.py','--obj',mesh,'--train',*m5[:2],'--holdout',*m5[2:],'--init',*a.init,'--output',registration])
 result=json.loads(registration.read_text())
 pose=result['estimated_pose_xyz_rxryrz'] if result['safe_to_propose_fitted_pose'] else a.init
 if not result['safe_to_propose_fitted_pose']:
  print('WARNING: fit failed holdout checks; retain initial pose instead.',flush=True)
 for association in ([.65,.30,.15] if a.stride==1 else [.65]):
  name=('M5_coverage_'+str(association).replace('.','p'))
  run([cli('twin_m0m5_multiview_cli'),'--obj',mesh,'--pose',*pose,'--sources',*m5,
       '--distance',association,'--frames',2,'--output',a.out/name])
 run([cli('twin_m0m5_optical_cli'),'--obj',mesh,'--pose',*pose,'--train',*m4[:2],
      '--holdout',*m4[2:],'--output',a.out/'M3_M4_optical_diagnostic.json'])
 best=a.out/'M5_coverage_0p65_report.json'
 cov=json.loads(best.read_text())
 if abs(cov['target_area_mm2']-cov['observed_area_pooled_mm2']-cov['no_hits_area_mm2']-cov['sparse_area_mm2'])>1e-4:raise ValueError('M5 area conservation failed')
 output={'steps':'M0-M5, offline only','step':str(a.step),'fitted_pose_used':list(pose),
         'registration_heldout_supported':bool(result['safe_to_propose_fitted_pose']),
         'coverage_0_65_ratio':cov['pooled_coverage'],
         'calibration_claim':'NOT physically certified; multi-view holdout diagnostic only'}
 (a.out/'M0_M5_run_summary.json').write_text(json.dumps(output,indent=2),encoding='utf8')
 print('M0-M5 complete; output:',a.out, 'coverage',round(cov['pooled_coverage']*100,3),'%')
if __name__=='__main__':main()
