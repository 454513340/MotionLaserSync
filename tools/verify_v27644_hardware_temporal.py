#!/usr/bin/env python3
"""Compatibility contract for the V27.6.44 temporal runtime retained by V27.6.46.

V27.6.46 deliberately replaces the old manual/per-axis calibration UI with the
staged trajectory workflow. V27.6.52 intentionally upgrades the commissioned
SINGLE-OP14 controller program to V32.10 runtime-PWM while retaining the same
physical trigger/latch chain, archive-v5 fields, runtime interpolation and legacy numerical API.
"""
from pathlib import Path
import hashlib, sys

root=Path(__file__).resolve().parents[1]

def text(rel): return (root/rel).read_text(encoding='utf-8',errors='replace')
def need(rel, *patterns):
    s=text(rel)
    for p in patterns:
        if p not in s:
            raise AssertionError(f'{rel}: missing {p!r}')

for rel in ['controller/ECI3808_V32_13_PWM_HANDSHAKE_FIVE_AXIS_HOME.bas','controller/ECI3808_5Axis_Controller.bas']:
    need(rel,'ECI3808 V32.13','REGIST(104','PWM_FREQ(0)=gf','TABLE(355)=DEFAULT_FREQ',
         'TABLE(356)=DEFAULT_FREQ','TABLE(357)=0','ApplyRuntimeConfig()','OP(14,ON)')

need('src/app/main.cpp','2.0.0-v27.6.55')
need('src/app/MainWindow.cpp','MotionLaserSync V27.6.55','setSyncManager(m_syncManager.get())')
need('src/widgets/CalibrationWidget.cpp','硬件同步标定','HardwareSyncCalibrationWidget')
need('include/calibration/HardwareTemporalCalibration.h','delay = t_sensor_effective - t_eci_latch','HardwareTemporalObservation')
need('src/calibration/HardwareTemporalCalibration.cpp','out.delaySec = -slope','distinctAbsSpeedLevels')
need('src/runtime/HardwareTemporalCorrection.cpp','HardwareTemporalInterpolated','current.ordinal < 1','next.ordinal != current.ordinal + 1')
need('src/core/SyncManager.cpp','m_temporalCorrectionEnabled','temporalBoundarySkipped','processProfiles(true)','REGIST(104)')
need('src/widgets/HardwareSyncCalibrationWidget.cpp','验证通过后启用补偿','active_hardware_temporal.json','solveTrajectoryPairs')
need('src/runtime/ScanArchive5D.cpp','w.u(5,4)','payloadVersion!=4 && payloadVersion!=5')
need('src/core/HardwareSyncReport.cpp','V27.6.55 硬件同步时域标定/补偿','t_sensor,effective - t_ECI,latch')
print('PASS V27.6.55 compatibility: V27.6.44 temporal runtime/archive retained with V32.13 runtime PWM')
