#!/usr/bin/env python3
from pathlib import Path
import hashlib, sys

ROOT = Path(__file__).resolve().parents[1]

def text(rel):
    return (ROOT/rel).read_text(encoding='utf-8', errors='replace')

def need(rel, *patterns):
    s=text(rel)
    for p in patterns:
        if p not in s:
            raise AssertionError(f'{rel}: missing {p!r}')

def main():
    expected='263d1f2a560120b006d25262e79fc364ebf35b4e6f2b3bf986f56013166b105a'
    for rel in ['ECI3808_V32_9_SINGLE_OP14_PWM_FULL_MPOS_StopFix_RAMCompact.bas','controller/ECI3808_5Axis_Controller.bas']:
        got=hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()
        if got != expected:
            raise AssertionError(f'{rel}: commissioned controller BASIC changed: {got}')
    need('src/app/main.cpp','2.0.0-v27.6.47')
    need('src/app/MainWindow.cpp','MotionLaserSync V27.6.47','Multi-Speed Repeated Hardware Temporal Calibration')
    need('src/widgets/HardwareSyncCalibrationWidget.cpp',
         '生成多速度实验模板',
         'm_templateMinSpeed->setValue(1.0)',
         'm_templateMaxSpeed->setValue(5.0)',
         'm_templateSpeedStep->setValue(0.5)',
         'm_templateCalibrationRepeats->setValue(3)',
         'm_templateValidationRepeats->setValue(3)',
         'X主扫掠',
         'for(int rep=0;rep<reps;++rep)',
         '各速度重复性统计',
         'τ均值 µs',
         'τ标准差 µs',
         '[硬件同步标定][速度重复性]',
         'const auto aligned=m_sync->getHardwareAlignedPoints();',
         'const auto poses=m_sync->getHardwarePoseSamples();')
    need('include/widgets/HardwareSyncCalibrationWidget.h',
         'm_templateMinSpeed', 'm_templateMaxSpeed', 'm_templateSpeedStep',
         'm_templateCalibrationRepeats', 'm_speedSummaryTable')
    print('PASS V27.6.47 multi-speed repeated temporal experiment source contract')
    print('Default calibration experiment: 1.0..5.0 mm/s, 0.5 step, 3 +/- pairs per speed')
    print('Default hold-out: XY/XZ/XYZ, 3 +/- pairs per trajectory')
    print('Controller BASIC hash unchanged:', expected)
    return 0

if __name__=='__main__':
    try:
        raise SystemExit(main())
    except AssertionError as e:
        print('FAIL V27.6.47 multi-speed repeated temporal experiment source contract:', e, file=sys.stderr)
        raise SystemExit(1)
