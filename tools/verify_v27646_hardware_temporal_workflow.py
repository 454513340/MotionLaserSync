#!/usr/bin/env python3
from pathlib import Path
import hashlib
import sys

ROOT = Path(__file__).resolve().parents[1]


def text(rel):
    return (ROOT / rel).read_text(encoding='utf-8', errors='replace')


def need(rel, *patterns):
    s = text(rel)
    for p in patterns:
        if p not in s:
            raise AssertionError(f'{rel}: missing required text: {p!r}')


def main():
    expected = '263d1f2a560120b006d25262e79fc364ebf35b4e6f2b3bf986f56013166b105a'
    for rel in ['ECI3808_V32_9_SINGLE_OP14_PWM_FULL_MPOS_StopFix_RAMCompact.bas',
                'controller/ECI3808_5Axis_Controller.bas']:
        got = hashlib.sha256((ROOT / rel).read_bytes()).hexdigest()
        if got != expected:
            raise AssertionError(f'{rel}: controller BASIC changed: {got}')
        need(rel, 'REGIST(104', 'PWM_FREQ(0)=200', 'OP(14,ON)')

    need('src/app/main.cpp', '2.0.0-v27.6.47')
    need('src/app/MainWindow.cpp',
         'MotionLaserSync V27.6.47',
         'hardwareTemporalPlannedScanRequested',
         'beginHardwareTemporalPlannedScan',
         'prepareHardwareTemporalScanAfterPreposition',
         'startHardwareTemporalRawCapture',
         'hwTableCommissioningPreflight',
         'HardwareTriggerType::EdgeRising',
         'SyncMode::Hardware',
         'prepareMergePath',
         'PathStartReference::MeasuredMpos',
         'pollHardwareTemporalSyncReady',
         'syncEstablished',
         'm_hardwareTemporalMotionStarted',
         'm_syncManager->stop()',
         'notifyHardwareTemporalScanFinished',
         'abortHardwareTemporalPlannedScan',
         'if (m_hardwareTemporalScanActive)')

    need('src/widgets/HardwareSyncCalibrationWidget.cpp',
         '1. 标准球参考：导入机床坐标系球面点云',
         '导入球面点云',
         'RANSAC拟合球心',
         '示教当前XYZCB为扫描轨迹中点',
         'X权重', 'Y权重', 'Z权重',
         '速度 mm/s', '距离 mm',
         '根据配置生成路径',
         '开始标定路径采集',
         '所有标定路径采集完成。现在点击“求解 τ”',
         '求解 τ',
         '开始验证路径采集',
         '评估验证结果',
         '验证通过后启用补偿',
         'active_hardware_temporal.json',
         'solveTrajectoryPairs',
         'estimateLinearMotionVector',
         'reconstructCorrectedValidationCloud')

    need('include/calibration/HardwareTemporalCalibration.h',
         'TemporalTrajectoryObservation',
         'TemporalTrajectoryCalibrationResult',
         'TemporalValidationResult',
         '(c+ - c-) = -delay * (v+ - v-)')
    need('src/calibration/HardwareTemporalCalibration.cpp',
         'solveTrajectoryPairs',
         'pf.pairDelaySec=-dot/dv2',
         'out.delaySec=-numer/denom',
         'evaluateValidationPairs')

    need('src/widgets/CalibrationWidget.cpp',
         'hardwareTemporalPlannedScanRequested',
         'hardwareTemporalReferenceCloudViewRequested',
         'notifyHardwareTemporalScanFinished')

    need('src/runtime/HardwareTemporalCorrection.cpp',
         'HardwareTemporalInterpolated',
         'next.ordinal != current.ordinal + 1')
    need('src/core/SyncManager.cpp',
         'm_temporalCorrectionEnabled',
         'REGIST(104)')

    print('PASS V27.6.47 staged hardware temporal calibration source contract')
    print('Controller BASIC hash unchanged:', expected)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print('FAIL V27.6.47 staged hardware temporal calibration source contract:', exc, file=sys.stderr)
        raise SystemExit(1)
