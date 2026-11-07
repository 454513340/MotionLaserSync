#!/usr/bin/env python3
"""Source contract for V27.6.52 runtime hardware-sync rate and power-session XYZ home state."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]

def text(rel):
    return (root / rel).read_text(encoding='utf-8', errors='replace')

def need(rel, *patterns):
    s = text(rel)
    for p in patterns:
        if p not in s:
            raise AssertionError(f"{rel}: missing {p!r}")

acq = text('include/runtime/Acquisition5D.h')
for p in [
    'defaultTriggerFrequencyHz=200',
    'requestedFrequencyHzOffset=25',
    'appliedFrequencyHzOffset=26',
    'linearHomeMaskOffset=27',
    'runtimeCapabilityMagic=321052',
]:
    if p not in acq:
        raise AssertionError(f'Acquisition5D.h missing {p!r}')

for rel in ['controller/ECI3808_5Axis_Controller.bas',
            'ECI3808_V32_9_SINGLE_OP14_PWM_FULL_MPOS_StopFix_RAMCompact.bas']:
    need(rel,
         'ECI3808 V32.10',
         'TABLE(355)=DEFAULT_FREQ',
         'TABLE(356)=DEFAULT_FREQ',
         'TABLE(357)=0',
         'PWM_FREQ(0)=DEFAULT_FREQ',
         'ApplyRuntimeConfig()',
         'PWM_FREQ(0)=reqf')
    if 'PWM_FREQ(0)=200' in text(rel):
        raise AssertionError(f'{rel}: still hardcodes active PWM_FREQ(0)=200')

need('src/app/MainWindow.cpp',
     'm_spinHardwareSyncFrequency',
     'updateHardwareSyncFrequencyLimit()',
     'LaserSensor::getMaxFrequencyForIndex(fieldIndex)',
     'setHardwareTriggerFrequencyHz(hardwareSyncFrequencyHz())')
need('src/core/SyncManager.cpp',
     'LaserSensor::getMaxFrequencyForIndex(m_sensorParams.measuringFieldIndex)',
     '1.0e6 / static_cast<double>(m_hwTriggerFrequencyHz)',
     'm_zmc->setHardwareTriggerFrequencyHz(m_hwTriggerFrequencyHz')
need('src/core/ZmcController.cpp',
     'restoreLinearReferenceStateFromController',
     'updateControllerLinearHomeMaskBit(axis, false',
     'updateControllerLinearHomeMaskBit(axis, true',
     'homeStatus == 1u')

# The old 5 ms TABLE header field remains intentionally as a legacy ABI marker,
# but active C++ code must not address a Table5D::periodMs runtime constant.
for base in ['include', 'src']:
    for p in (root / base).rglob('*'):
        if p.suffix.lower() not in {'.h', '.hpp', '.cpp', '.cc', '.cxx'}:
            continue
        s = p.read_text(encoding='utf-8', errors='replace')
        if 'Table5D::periodMs' in s or 'm_hwTriggerPeriodMs' in s or 'setHardwareTriggerPeriodMs' in s:
            raise AssertionError(f'{p.relative_to(root)}: stale fixed-period API/reference')

print('PASS V27.6.52 runtime PWM + controller power-session XYZ home-state source contract')
