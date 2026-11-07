#!/usr/bin/env python3
from pathlib import Path
import hashlib, sys

ROOT=Path(__file__).resolve().parents[1]

def text(rel):
    return (ROOT/rel).read_text(encoding='utf-8',errors='replace')

def need(rel,*patterns):
    s=text(rel)
    for p in patterns:
        if p not in s:
            raise AssertionError(f'{rel}: missing {p!r}')

def main():
    expected='263d1f2a560120b006d25262e79fc364ebf35b4e6f2b3bf986f56013166b105a'
    for rel in ['ECI3808_V32_9_SINGLE_OP14_PWM_FULL_MPOS_StopFix_RAMCompact.bas','controller/ECI3808_5Axis_Controller.bas']:
        got=hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()
        if got!=expected: raise AssertionError(f'{rel}: commissioned BASIC changed: {got}')
    need('src/app/main.cpp','2.0.0-v27.6.49')
    need('src/app/MainWindow.cpp','MotionLaserSync V27.6.49','Resumable Checkpointed Hardware Temporal Calibration')
    need('include/widgets/HardwareSyncCalibrationWidget.h',
         'attemptCount','failureCount','consecutiveFailures','lastError','restoredFromDisk',
         'restoreCaptureSession','newCaptureSession','saveSessionCheckpoint','recoverLegacyV48Session','m_autoRetryCount')
    hw=text('src/widgets/HardwareSyncCalibrationWidget.cpp')
    for p in [
        '开始/继续标定路径采集','开始/继续验证路径采集','恢复/继续已有Session','新建/重置本轮Session',
        'session_manifest_v49.json','HTRUN49-','HTCAL49-','失败自动重试',
        '本Session已完成的数据和RAW归档全部保留','将从首个未完成路径继续',
        'recoverLegacyV48Session','完成/V48磁盘恢复','从旧RAW恢复扫描轨迹中点',
        'inferPlanCenterFromArchivedScan','retryablePlannedScanFailure',
        'p.completed=true;','saveSessionCheckpoint(&checkpointError)']:
        if p not in hw: raise AssertionError(f'HardwareSyncCalibrationWidget.cpp missing {p!r}')
    # Regression guard: starting/resuming calibration must not clear accepted observations.
    start=hw.index('void HardwareSyncCalibrationWidget::startCalibrationAcquisition()')
    stop=hw.index('void HardwareSyncCalibrationWidget::startValidationAcquisition()',start)
    body=hw[start:stop]
    for forbidden in ['m_calibrationObservations.clear()','p.completed=false;p.accepted=false;p.status=tr("待执行")']:
        if forbidden in body: raise AssertionError(f'resume path still resets progress: {forbidden}')
    # Strict closure remains; V49 must not “fix” a mismatch by silently accepting it.
    need('src/core/SyncManager.cpp','停止收尾未达到完整闭环','本次数据禁止判为有效')
    print('PASS V27.6.49 resumable/checkpointed temporal workflow source contract')
    print('Resume is default; explicit new/reset is required to discard UI progress')
    print('Legacy V48 .scan + sidecar recovery is present')
    print('Controller BASIC hash unchanged:',expected)
    return 0

if __name__=='__main__':
    try: raise SystemExit(main())
    except AssertionError as e:
        print('FAIL V27.6.49 resumable temporal source contract:',e,file=sys.stderr)
        raise SystemExit(1)
