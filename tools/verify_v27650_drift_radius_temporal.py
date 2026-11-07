#!/usr/bin/env python3
from pathlib import Path
import hashlib, re, sys
ROOT=Path(__file__).resolve().parents[1]

def text(rel): return (ROOT/rel).read_text(encoding='utf-8',errors='replace')
def need(rel,*patterns):
    s=text(rel)
    for p in patterns:
        if p not in s: raise AssertionError(f'{rel}: missing {p!r}')

def main():
    expected='263d1f2a560120b006d25262e79fc364ebf35b4e6f2b3bf986f56013166b105a'
    for rel in ['ECI3808_V32_9_SINGLE_OP14_PWM_FULL_MPOS_StopFix_RAMCompact.bas','controller/ECI3808_5Axis_Controller.bas']:
        got=hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()
        if got!=expected: raise AssertionError(f'{rel}: commissioned BASIC changed: {got}')
    app=text('src/app/main.cpp')
    if '2.0.0-v27.6.50' not in app and '2.0.0-v27.6.51' not in app:
        raise AssertionError('src/app/main.cpp: missing compatible V27.6.50+ application version')
    mw=text('src/app/MainWindow.cpp')
    if 'MotionLaserSync V27.6.50' not in mw and 'MotionLaserSync V27.6.51' not in mw:
        raise AssertionError('src/app/MainWindow.cpp: missing compatible V27.6.50+ title')
    need('include/calibration/HardwareTemporalCalibration.h',
         'struct FreeSphereFit','refineFreeSphere','captureMidPcUs','driftModelUsed','centerDriftMmPerSec',
         'meanEffectiveRadiusMm','maxPairRadiusDifferenceMm')
    need('src/calibration/HardwareTemporalCalibration.cpp',
         'fitCenterDriftAndDelay','c_i = c0 + g*(t_i-t0) - tau*v_i','FreeSphereFit HardwareTemporalCalibration::refineFreeSphere',
         'common-deformation gate')
    hw=text('src/widgets/HardwareSyncCalibrationWidget.cpp')
    for p in [
        'm_templateMinSpeed->setValue(0.5)',
        '0.5~5.0 mm/s',
        'std::vector<double> speedOrder',
        '低/高速度交错',
        'Alternate pair acquisition order (+,-), (-,+), (+,-)',
        'refineFreeSphere(',
        '动态自由有效半径球拟合失败',
        '有效R相对物理标称R偏差=',
        'captureMidPcUs',
        'refreshPathTable()',
        'refreshCaptureTable()',
        'refreshSpeedSummaryTable()']:
        if p not in hw: raise AssertionError(f'HardwareSyncCalibrationWidget.cpp missing {p!r}')
    # Regression guard for the exact V49 linker failure reported by MSVC.
    for name in ['refreshPathTable','refreshCaptureTable','refreshSpeedSummaryTable']:
        pat=rf'void\s+HardwareSyncCalibrationWidget::{name}\s*\('
        if len(re.findall(pat,hw)) != 1:
            raise AssertionError(f'{name}: expected exactly one out-of-class definition')
    # Reference sphere remains a physical fixed-radius anchor; dynamic scans use free effective radius.
    if 'refineKnownSphereCenter(in,r.centerMachineMm,opt.expectedRadiusMm' not in hw:
        raise AssertionError('reference sphere no longer uses fixed physical radius')
    if 'const auto dynamicFit=HardwareTemporalCalibration::refineFreeSphere' not in hw:
        raise AssertionError('dynamic scan does not use free effective radius')
    # Keep V49 resumable safety behavior and strict closure.
    need('src/widgets/HardwareSyncCalibrationWidget.cpp','开始/继续标定路径采集','session_manifest_v49.json')
    hw2=text('src/widgets/HardwareSyncCalibrationWidget.cpp')
    if 'HTRUN50-' not in hw2 and 'HTRUN51-' not in hw2:
        raise AssertionError('resumable HTRUN50/51 session prefix missing')
    need('src/core/SyncManager.cpp','停止收尾未达到完整闭环','本次数据禁止判为有效')
    print('PASS V27.6.50 drift-aware/free-effective-radius temporal source contract')
    print('MSVC unresolved refresh* definitions restored')
    print('Default experiment includes 0.5..5.0 mm/s and drift-decorrelating ordering')
    print('Controller BASIC hash unchanged:', expected)
    return 0

if __name__=='__main__':
    try: raise SystemExit(main())
    except AssertionError as e:
        print('FAIL V27.6.50 source contract:',e,file=sys.stderr)
        raise SystemExit(1)
