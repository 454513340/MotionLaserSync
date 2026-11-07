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

    need('src/app/main.cpp','2.0.0-v27.6.51')
    need('src/app/MainWindow.cpp',
         'MotionLaserSync V27.6.51',
         '硬件同步时域补偿',
         '启用正式硬件同步时域补偿',
         'refreshHardwareTemporalRuntimeUi()',
         'onHardwareTemporalRuntimeToggled',
         'clearPointCloudForTemporalModeChange',
         'hardwareTemporalRuntimeCorrectionChanged',
         'q(t+τ)')
    need('include/app/MainWindow.h',
         'm_checkHardwareTemporalRuntime',
         'm_lblHardwareTemporalValue',
         'm_lblHardwareTemporalConfig')

    sync=text('src/core/SyncManager.cpp')
    need('src/core/SyncManager.cpp',
         'V27.6.51: keep the loaded/calibrated delay even when runtime correction',
         'Do not erase tau / sigma / calibration ID')
    # Guard against regression to V50 behavior where OFF erased the calibration payload.
    disable=re.search(r'void\s+SyncManager::disableHardwareTemporalCorrection\s*\(\)\s*\{(.*?)\n\}',sync,re.S)
    if not disable: raise AssertionError('disableHardwareTemporalCorrection definition missing')
    body=disable.group(1)
    if 'delaySec = 0.0' in body or 'delayStdSec = 0.0' in body:
        raise AssertionError('runtime OFF still erases loaded temporal calibration')

    hw=text('src/widgets/HardwareSyncCalibrationWidget.cpp')
    for p in [
        'setRuntimeCorrectionEnabled',
        '验证PASS结果已自动保存为活动标定（默认运行时OFF）',
        'saveJson(activePath(),false',
        'always stage a validated active calibration into SyncManager',
        '已自动加载活动硬件时域标定',
        '运行时补偿：DISABLED / RAW；活动τ仍已加载=',
        'root["softwareVersion"]="V27.6.51"',
        'restoring a completed V49/V50/V51 Session',
        '已恢复的验证PASS Session已自动保存为活动标定']:
        if p not in hw: raise AssertionError(f'HardwareSyncCalibrationWidget.cpp missing {p!r}')
    need('include/widgets/HardwareSyncCalibrationWidget.h','activeCalibrationChanged','setRuntimeCorrectionEnabled')
    need('src/widgets/CalibrationWidget.cpp','hardwareTemporalRuntimeCorrectionChanged','activeCalibrationChanged')

    need('include/widgets/PointCloudWidget.h','setTemporalCorrectionStatus','m_temporalCorrectionLabel')
    need('src/widgets/PointCloudWidget.cpp',
         'Temporal Runtime: ON',
         'Temporal Runtime: OFF / RAW',
         'Temporal Runtime: N/A / 软件同步',
         '三维点云将使用q(t+τ)重建')

    print('PASS V27.6.51 temporal runtime-control source contract')
    print('Active calibration auto-load/persistence + Parameters ON/OFF + 3-D provenance present')
    print('Controller BASIC hash unchanged:', expected)
    return 0

if __name__=='__main__':
    try: raise SystemExit(main())
    except AssertionError as e:
        print('FAIL V27.6.51 source contract:',e,file=sys.stderr)
        raise SystemExit(1)
