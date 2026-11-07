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

    need('src/app/main.cpp','2.0.0-v27.6.48')
    need('src/app/MainWindow.cpp',
         'MotionLaserSync V27.6.48',
         'setHardwareAlignedDataEmissionEnabled(false)',
         'setHardwareRawProfileCaptureEnabled(true)',
         'setHardwareRawProfileCaptureEnabled(false)')
    need('include/core/SyncManager.h',
         'setRawProfileCaptureEnabled',
         'takeCompletedRawCapture',
         'm_rawCapturedProfiles',
         'takeHardwareRawCaptureData')
    need('src/core/SyncManager.cpp',
         'const bool rawProfileMode = m_rawProfileCaptureEnabled.load()',
         'newRawProfiles.push_back(std::move(pending.profile))',
         'm_rawCapturedProfiles.push_back(std::move(profile))',
         'takeCompletedRawCapture',
         'QVector<LaserProfile>().swap(m_rawCapturedProfiles)')
    need('src/widgets/HardwareSyncCalibrationWidget.cpp',
         'E:/Projects/CData',
         '每条扫描自动保存完整RAW .scan',
         'QtConcurrent::run',
         'takeHardwareRawCaptureData',
         'CompactScanIO::fromProfilesAndPoses',
         'RANSAC/验证最大分析点数',
         'm_maxAnalysisPoints->setValue(250000)',
         '下一条路径会等待归档、分析和内存释放',
         'RAW有效点=',
         '生成多速度实验模板',
         'm_templateCalibrationRepeats->setValue(3)')
    need('src/core/PointCloudBuilder.cpp',
         'QVector<QVector3D>().swap(m_points)',
         'QVector<AlignedPoint>().swap(m_alignmentLog)')

    # Critical regression guard: the temporal widget must not pull/retain the
    # full expanded aligned cloud after a planned scan.
    hw=text('src/widgets/HardwareSyncCalibrationWidget.cpp')
    if 'getHardwareAlignedPoints()' in hw:
        raise AssertionError('temporal widget still copies the full expanded aligned cloud')

    print('PASS V27.6.48 memory-safe streamed temporal calibration source contract')
    print('RAW archive root default: E:/Projects/CData')
    print('Automatic temporal scans keep decoded profiles + REGIST poses, not expanded AlignedPoint clouds')
    print('Controller BASIC hash unchanged:', expected)
    return 0

if __name__=='__main__':
    try:
        raise SystemExit(main())
    except AssertionError as e:
        print('FAIL V27.6.48 memory-safe temporal source contract:', e, file=sys.stderr)
        raise SystemExit(1)
