#!/usr/bin/env python3
"""Source contract for V27.6.53 runtime PWM + controller power-session XYZ/C/B Home state."""
from pathlib import Path

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
    'machineHomeMaskOffset=27',
    'runtimeCapabilityMagic=321053',
]:
    if p not in acq:
        raise AssertionError(f'Acquisition5D.h missing {p!r}')

for rel in ['controller/ECI3808_5Axis_Controller.bas',
            'controller/ECI3808_V32_11_RUNTIME_PWM_FIVE_AXIS_HOME_STATE.bas']:
    need(rel,
         'ECI3808 V32.11',
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
     'homeRotaryAxisToZero(axisId, motion, &err)',
     'areAllAxesHomed()',
     'isAxisHomed(AxisC)',
     'isAxisHomed(AxisB)',
     'setHardwareTriggerFrequencyHz(hardwareSyncFrequencyHz())')
need('src/core/SyncManager.cpp',
     'LaserSensor::getMaxFrequencyForIndex(m_sensorParams.measuringFieldIndex)',
     '1.0e6 / static_cast<double>(m_hwTriggerFrequencyHz)',
     'm_zmc->setHardwareTriggerFrequencyHz(m_hwTriggerFrequencyHz')
need('src/core/ZmcController.cpp',
     'restoreMachineHomeStateFromController',
     'updateControllerHomeMaskBit(axis, false',
     'updateControllerHomeMaskBit(axis, true',
     'homeRotaryAxisToZero',
     'm_rotaryHomeCompleted[AxisC]',
     'm_rotaryHomeCompleted[AxisB]',
     'mask > 31',
     'homeMask != 31',
     'homeMask!=31',
     'homeStatus == 1u')

# Active C++ must derive the period from frequency, not use the old 5 ms API.
for base in ['include', 'src']:
    for p in (root / base).rglob('*'):
        if p.suffix.lower() not in {'.h', '.hpp', '.cpp', '.cc', '.cxx'}:
            continue
        s = p.read_text(encoding='utf-8', errors='replace')
        if 'Table5D::periodMs' in s or 'm_hwTriggerPeriodMs' in s or 'setHardwareTriggerPeriodMs' in s:
            raise AssertionError(f'{p.relative_to(root)}: stale fixed-period API/reference')

print('PASS V27.6.53 runtime PWM + controller power-session XYZ/C/B Home-state source contract')
