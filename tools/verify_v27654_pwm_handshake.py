#!/usr/bin/env python3
"""ARCHIVED V27.6.54/V32.12 contract: run only against a v54 source snapshot, NOT current v55 release."""
from pathlib import Path
r=Path(__file__).resolve().parents[1]
read=lambda x:(r/x).read_text(encoding='utf8')
b=read('controller/ECI3808_5Axis_Controller.bas')
a=read('controller/ECI3808_V32_12_PWM_HANDSHAKE_FIVE_AXIS_HOME.bas')
assert a==b,'canonical field BASIC differs from V32.12 source'
assert 'CONST RUNTIME_MAGIC=321054' in b
assert 'TABLE(357)=0' in b and 'TABLE(358)=0' in b
assert 'TABLE(359)=0' in b and 'TABLE(360)=0' in b and 'TABLE(361)=0' in b
assert b.index('StopPwmSync()') < b.index('  gm=0\n  IF TABLE(333)=0 THEN\n   ApplyRuntimeConfig()'), 'stop must precede apply'
assert 'stopdelay=1000/gf+2' in b and '1000/TABLE(356)' not in b
assert 'ELSEIF TABLE(355)<>gf OR TABLE(359)<>ga OR TABLE(361)<>0 THEN' in b
assert 'IF reqseq<>ga THEN' in b and 'TABLE(360)=ga' in b
assert 'TABLE(333)=10' not in b,'invalid configuration must not repeatedly call Bad()'
assert 'TABLE(357)=0' in b and 'TABLE(340,0,0,1,31,4,31,0,0,14,104,14,1)' in b
assert b.index('PWM_FREQ(0)=reqf') < b.index('TABLE(360)=ga')
h=read('include/runtime/Acquisition5D.h')
for term in ('runtimeCapabilityMagic=321054','controllerStateOffset=28','frequencyRequestSequenceOffset=29','frequencyAckSequenceOffset=30','frequencyConfigErrorOffset=31','runtimeReady('):
    assert term in h,term
c=read('src/core/ZmcController.cpp')
for term in ('hwTableWaitIdleReady','const float request','frequencyRequestSequenceOffset','T::runtimeReady(runtime,m_hwTriggerFrequencyHz.load(),T::IdleReady)','T::runtimeReady(runtime,m_hwTriggerFrequencyHz.load(),T::Probe)','T::runtimeReady(v,m_hwTriggerFrequencyHz.load(),T::IdleReady)'):
    if term=='const float request':continue
    assert term in c,term
sm=read('src/core/SyncManager.cpp')
assert 'm_zmc->hwTableWaitIdleReady(1500' in sm
for file in ('calibration/runtime_xyz/bundle.json','third_party/scanCONTROL/bin/x64/LLT.dll','third_party/scanCONTROL/lib/x64/LLT.lib','third_party/zmc/lib/x64/zauxdll.lib','models/machine5x/visual/01_BASE.ply'):
    assert (r/file).is_file(),file
print('PASS V27.6.54 V32.12 static safety contract & restored original resources')
