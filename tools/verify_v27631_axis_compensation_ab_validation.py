#!/usr/bin/env python3
from pathlib import Path
import re, sys
root = Path(__file__).resolve().parents[1]
cpp = (root/'src/app/MainWindow.cpp').read_text(encoding='utf-8')
hdr = (root/'include/app/MainWindow.h').read_text(encoding='utf-8')
checks = {
    'version title': 'MotionLaserSync V27.6.31' in cpp,
    'A/B group': '候选补偿表 A/B 验证' in cpp,
    'candidate load': 'loadAxisCompensationValidationTable' in cpp and 'desired_mpos_mm' in cpp and 'required_dpos_mm' in cpp,
    'linear interpolation': 'interpolateAxisCompensationCommand' in cpp and '(targetMposMm - x0) / (x1 - x0)' in cpp,
    'independent midpoint targets': '0.5 * (reference[i].desiredMposMm + reference[i + 1].desiredMposMm)' in cpp,
    'raw command': 'double command = target;' in cpp,
    'compensated mode': 'QStringLiteral("COMPENSATED")' in cpp,
    'no trim declaration': 'mpos_trim_used' in cpp and 'false' in cpp,
    'global compensation stays off': 'runtime_global_compensation_enabled' in cpp,
    'direction preload': 'direction > 0 ? command - preload : command + preload' in cpp,
    'settle measurement': 'QThread::msleep' in cpp and 'endSettled' in cpp,
    'summary metrics': all(x in cpp for x in ['p95AbsMm','maxAbsMm','meanErrorMm','improvement']),
    'output files': all(x in cpp for x in ['axis_compensation_ab_validation.csv','axis_compensation_ab_summary.csv','axis_compensation_ab_validation_manifest.json']),
    'cancellation member': 'm_axisCompValCancel' in hdr and 'm_axisCompValRunning' in hdr,
}
failed = [name for name, ok in checks.items() if not ok]
for name, ok in checks.items():
    print(('PASS' if ok else 'FAIL'), name)
if failed:
    print('failed:', ', '.join(failed), file=sys.stderr)
    sys.exit(1)
print('V27.6.31 A/B validation source contract passed')
