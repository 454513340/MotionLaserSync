#!/usr/bin/env python3
from pathlib import Path
import csv, json, re, sys
root=Path(__file__).resolve().parents[1]

def text(rel): return (root/rel).read_text(encoding='utf-8-sig')
checks=[]
def check(name, ok):
    checks.append((name,bool(ok)))
    print(('PASS ' if ok else 'FAIL ')+name)

hdr=text('include/safety/MachineSafetyModel.h')
safety=text('src/safety/MachineSafetyModel.cpp')
app=text('src/app/MainWindow.cpp')
ctlh=text('include/core/ZmcController.h')
adapter=text('src/core/ZmcAxisCompensation.cpp')
check('MachineSafety transition API supports full-path static clearance', 'checkStaticClearanceThroughout = false' in hdr)
check('synchronous transition checks clearance at every sample when requested', 'checkStaticClearanceThroughout || (i == 0 || i == segments)' in safety)
check('both synchronous and independent loops use the full-clearance gate', safety.count('checkStaticClearanceThroughout || (i == 0 || i == segments)') == 2)
check('MainWindow couples compensated transition to full-clearance mode', 'checkTransition(from, to, compensationOn)' in app)
check('independent safety predictor requests actual compensated speed parameters', 'effectiveIndependentAxisParams5D(' in app and 'safetyAxisParams' in app)
check('independent safety predictor uses adjusted parameters', 'current, target, safetyAxisParams, scale, compensationOn' in app)
check('controller exports compensation safety envelope', 'axisCompensationSafetyEnvelopeMm' in ctlh and 'axisCompensationSafetyEnvelopeMm' in adapter)
check('MachineSafety fails closed when configured clearance does not dominate compensation envelope', 'clearanceMm > envelopeMm + 1.0e-6' in app)
check('controller speed adapter applies precision-speed cap before safety prediction', 'capped=(std::min)(requestedEffective,precision)' in adapter)

maxcorr={}
for axis in 'XYZ':
    f=root/f'calibration/runtime_xyz/{axis}/{axis}_compensation_candidate.csv'
    rows=list(csv.DictReader(f.open(encoding='utf-8-sig')))
    maxcorr[axis]=max(abs(float(r['required_dpos_mm'])-float(r['desired_mpos_mm'])) for r in rows)
envelope=sum(maxcorr.values())
cfg=json.loads((root/'models/machine5x/machine_config.json').read_text(encoding='utf-8'))
clearance=float(cfg['safety']['minimum_static_clearance_mm'])
check('supplied XYZ compensation envelope is finite and positive', 0 < envelope < 1)
check('configured static clearance dominates supplied XYZ compensation envelope', clearance > envelope)
print(f'INFO envelope_mm={envelope:.9f} X={maxcorr["X"]:.9f} Y={maxcorr["Y"]:.9f} Z={maxcorr["Z"]:.9f} clearance_mm={clearance:.6f}')

# Preserve the historical scope truth: FCL is not a global motion facade.
onmerge=re.search(r'void MainWindow::onStartMerge\(\)\s*\{(.*?)\n\}', app, re.S)
check('audit does not falsely globalize FCL: generic merge remains outside MachineSafety', bool(onmerge and 'MachineSafety' not in onmerge.group(1) and 'checkSphereStudy' not in onmerge.group(1)))

failed=[n for n,o in checks if not o]
print(f'{len(checks)-len(failed)}/{len(checks)} checks passed')
sys.exit(1 if failed else 0)
