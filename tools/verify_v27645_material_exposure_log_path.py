#!/usr/bin/env python3
from pathlib import Path
import hashlib
import sys

ROOT = Path(__file__).resolve().parents[1]

def txt(rel):
    return (ROOT / rel).read_text(encoding='utf-8', errors='replace')

def need(rel, *patterns):
    s = txt(rel)
    for p in patterns:
        if p not in s:
            raise AssertionError(f'{rel}: missing required text: {p!r}')

def forbid(rel, *patterns):
    s = txt(rel)
    for p in patterns:
        if p in s:
            raise AssertionError(f'{rel}: forbidden text still present: {p!r}')

def main():
    expected = '263d1f2a560120b006d25262e79fc364ebf35b4e6f2b3bf986f56013166b105a'
    for rel in ['ECI3808_V32_9_SINGLE_OP14_PWM_FULL_MPOS_StopFix_RAMCompact.bas',
                'controller/ECI3808_5Axis_Controller.bas']:
        got = hashlib.sha256((ROOT / rel).read_bytes()).hexdigest()
        if got != expected:
            raise AssertionError(f'{rel}: controller BASIC changed: {got}')

    need('src/app/main.cpp', '2.0.0-v27.6.47')
    need('src/app/MainWindow.cpp',
         'MotionLaserSync V27.6.47',
         'QStandardPaths::AppDataLocation/calibration/material_exposure_library.json',
         '[材料曝光方案] 材料曝光库已加载',
         '[材料曝光方案] 材料曝光库刷新失败',
         '[材料曝光方案] 已切换为手动Exposure/Idle',
         '[材料曝光方案] 方案不存在',
         '[材料曝光方案] 当前方案无法用于当前同步模式',
         '[材料曝光方案] 已应用',
         '[材料曝光方案] 曝光分析推荐值已写入材料库',
         'LOG_INFO(LogCategory::Sensor',
         'LOG_WARNING(LogCategory::Sensor',
         'LOG_ERROR(LogCategory::Sensor')
    forbid('src/app/MainWindow.cpp', 'm_lblMaterialExposureStatus')
    forbid('include/app/MainWindow.h', 'm_lblMaterialExposureStatus')

    need('src/calibration/MaterialExposureLibrary.cpp',
         'QStandardPaths::AppDataLocation',
         'calibration/material_exposure_library.json',
         'QStandardPaths::AppConfigLocation',
         'loadedLegacy',
         'V27.6.47')

    # The material plan no longer uses transient status-bar feedback either.
    main = txt('src/app/MainWindow.cpp')
    if '材料曝光方案已导入：Exp=' in main:
        raise AssertionError('legacy material-exposure statusBar message still present')

    print('PASS V27.6.47 material exposure log/path compatibility contract')
    print('Expected Windows path: %APPDATA%/YourCompany/MotionLaserSync/calibration/material_exposure_library.json')
    print('Controller BASIC hash unchanged:', expected)
    return 0

if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print('FAIL V27.6.47 material exposure log/path compatibility contract:', exc, file=sys.stderr)
        raise SystemExit(1)
