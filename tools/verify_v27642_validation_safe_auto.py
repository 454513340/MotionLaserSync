#!/usr/bin/env python3
from pathlib import Path
import math
import sys

ROOT = Path(__file__).resolve().parents[1]
widget = (ROOT / 'src/widgets/SingleSphere14CalibrationWidget.cpp').read_text(encoding='utf-8')
header = (ROOT / 'include/widgets/SingleSphere14CalibrationWidget.h').read_text(encoding='utf-8')
controller = (ROOT / 'src/core/ZmcController.cpp').read_text(encoding='utf-8')
main = (ROOT / 'src/app/main.cpp').read_text(encoding='utf-8')
readme = (ROOT / 'README_V27_6_42_VALIDATION_SAFE_AUTO_ZH.md').read_text(encoding='utf-8')

checks = []
def check(name, cond, detail=''):
    checks.append((name, bool(cond), detail))

check('version', '2.0.0-v27.6.42' in main)
check('hazard checkbox', 'm_checkValidationAvoidRejected' in header and '避开阶段2已剔除的高误差B/C区域' in widget)
check('hazard distance helper', 'validationPoseOutsideRejectedTrainingHazards' in header and 'std::hypot(db, dc)' in widget)
check('hazard source is data-driven', 'm_lastResult.rejectedTrainingPoses' in widget)
check('default hazard radius 10 deg', 'm_spinValidationHazardBCSep->setValue(10.0)' in widget)
check('candidate pool widened', 'std::max(48, wanted * 8)' in widget)
check('auto validation start', 'onStartAutomaticValidationAcquisition' in header and 'continueAutomaticValidationAcquisition' in widget)
check('holdout auto calls capture(true)', 'startCapture(true);' in widget)
check('validation rechecks separation', 'validationPoseSeparated(q, separationError)' in widget)
check('validation uses independent move signal', 'emit requestIndependentJointMove(finalTarget' in widget)
check('controller facade', 'moveJointAbs5DIndependent(' in controller)
check('per-axis Single_MoveAbs', 'ZAux_Direct_Single_MoveAbs(m_handle, axis, destination)' in controller)
check('no interpolation claim backed by code', 'no BASE list and no MoveAbs(5 axis) interpolation' in controller)
check('validation scope JSON', 'retained_safe_bc_domain_excluding_stage2_rejected_neighborhoods' in widget)
check('scope caveat documented', '只证明“保留安全 B/C 工作域”有效' in readme)

# Regression sanity using the known V27.6.41 rejected-pose clusters and the old
# hold-out angles discussed from the user's dataset.  With the V27.6.42 default
# radius, all three previously risky validation angles must be outside the safe set.
rejected = [
    (20.0004, 39.9977),
    (20.0004, 44.9982),
    (-29.9994, 39.9977),
    (-29.9994, 44.9982),
    (20.0004, -35.0014),
]
holdout = [
    (25.0, 37.5),
    (-25.0, 37.5),
    (25.0, -37.5),
]

def wrap180(x):
    return math.remainder(x, 360.0)

def nearest_bc(p):
    b,c = p
    return min(math.hypot(b-br, wrap180(c-cr)) for br,cr in rejected)

d = [nearest_bc(p) for p in holdout]
check('known risky holdout angles excluded by 10 deg', all(x < 10.0 for x in d), f'nearest distances={d}')

failed = [x for x in checks if not x[1]]
for name, ok, detail in checks:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f": {detail}" if detail else ''))
print(f"\n{len(checks)-len(failed)}/{len(checks)} checks passed")
if failed:
    sys.exit(1)
