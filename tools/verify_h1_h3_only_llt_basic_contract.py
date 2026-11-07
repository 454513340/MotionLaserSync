#!/usr/bin/env python3
"""Static smoke contract for the H1/H3-only LLT + controller BASIC project.

Not a ZDevelop compiler or a Windows GUI hardware-in-the-loop test.
This script intentionally rejects stale H2 and pre-removal protocol artifacts.
"""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]

def check(ok, message):
    if not ok:
        raise AssertionError(message)


def source(path):
    return (root / path).read_text(encoding='utf-8')

laser = source('src/core/LaserSensor.cpp')
sync = source('src/core/SyncManager.cpp')
controller = source('src/core/ZmcController.cpp')
header = source('include/runtime/Acquisition5D.h')
ui = source('src/app/MainWindow.cpp')
mock = source('tests/MockZaux.cpp')
basic_path = root / 'controller/ECI3808_V32_16_H1_H3_ONLY.bas'
check(basic_path.is_file(), 'H1/H3-only BASIC program is missing')
check([p.name for p in basic_path.parent.glob('*.bas')] == [basic_path.name],
      'More than one BASIC deployment entrypoint is present')
basic = basic_path.read_text(encoding='utf-8')

sdk_h = (root / 'third_party/scanCONTROL/include/InterfaceLLT_2.h').read_text(encoding='latin1')
sdk_cpp = (root / 'third_party/scanCONTROL/source/InterfaceLLT_2.cpp').read_text(encoding='latin1')
check(bool(re.search(r'void\s+Timestamp2CmmTriggerAndInCounter\s*\(', sdk_h)),
      'Unexpected Micro-Epsilon CMM timestamp interface return type')
check(bool(re.search(r'void\s+CInterfaceLLT::Timestamp2CmmTriggerAndInCounter\s*\(', sdk_cpp)),
      'Unexpected bundled LLT C++ CMM timestamp wrapper')
check(bool(re.search(r'cmmTimestampDecoded\s*=\s*true\s*;', laser)) and bool(re.search(r'profile\.cmmValid\s*=\s*cmmTimestampDecoded\s*;', laser)),
      'H3 CMM timestamp decoding has been removed')
check('static_assert(std::is_same_v<' in laser, 'CMM timestamp SDK ABI guard missing')

check('runtimeCapabilityMagic=321058' in header, 'Host V32.16 capability marker missing')
check('TABLE(352,0,0,321058' in basic, 'BASIC V32.16 capability marker missing')
check('table[354]=321058' in mock and 'table[362]=1316' in mock,
      'Mock protocol header is incomplete')
check('stopTailMarker=1316' in header and 'TABLE(362,1316' in basic,
      'STOP-DRAIN compatibility marker changed unexpectedly')
check('EciPwmShared = 1, SensorInternalCmm = 3' in header,
      'H1/H3 topology identities changed')
check('(mode==1 && edge==h1R0Falling)' in header and '(mode==3 &&' in header,
      'H1/H3 ACK validation missing')
check('validTopologyEdge(' in controller, 'Host controller config does not validate topology')
check('validTopologyEdge(' in sync, 'Host capture start does not fail closed for topology 2')
check('ELSEIF (rm<>1 AND rm<>3) OR rm<>INT(rm) THEN' in basic and 'TABLE(361)=14' in basic,
      'BASIC must reject topology ID 2 with config error 14')
check('(rm=1 AND re<>104) OR (rm=3 AND re<>114 AND re<>115)' in basic,
      'BASIC H1/H3 input edge checks missing')
check('REGIST(he,821+129*j,129) AXIS(j)' in basic and 'FOR j=1 TO 5' in basic,
      'Five-axis hardware-latched FIFO path missing')
check('REG_INPUTS=16,16,16,16,16' in basic, 'R0/R1 inputs mapping missing')
check('IF hm=3 AND TABLE(333)=0 THEN' in basic, 'H3 stop quiet-window missing')
check('IF en=2 AND hm=1 AND gq>1 THEN FL(5)' in basic, 'H1 one-shot count safety missing')
check('hwSensorColdPrimeStart' in sync, 'H1 cold-prime missing')
check('startCmmGated' in sync and 'stopCmmGated' in sync,
      'H3 gated CMM start/stop missing')
check('StartTransmissionAndCmmTrigger' in laser and 'StopTransmissionAndCmmTrigger' in laser,
      'H3 CMM SDK gated transmission functions missing')
check('MULTI_RS422_CMM' in laser, 'H3 CMM RS422 mode setup missing')
check('m_comboHardwareTopology->addItem(tr("H1 ' in ui and
      'm_comboHardwareTopology->addItem(tr("H3 ' in ui,
      'Only H1 and H3 should be visible in the hardware selector')

retired_symbols = (
    'EciPwmSensorCmm', 'stopExternalCmmGated', 'verifyExternalTriggerAfterCmm',
    'setCmmOutputActive', 'CmmWarmupDiagnostics', 'kH2StagedDiagnosticOnly',
    '[H2-DIAG]', 'H1/H2/H3',
)
for path in [root / 'include/runtime/Acquisition5D.h',
             root / 'include/core/LaserSensor.h', root / 'src/core/LaserSensor.cpp',
             root / 'src/core/SyncManager.cpp', root / 'src/core/ZmcController.cpp',
             root / 'src/app/MainWindow.cpp', basic_path]:
    contents = path.read_text(encoding='utf-8')
    for retired in retired_symbols:
        check(retired not in contents, f'Retired H2 implementation still present in {path.name}: {retired}')

for item in ['CMakeLists.txt', 'calibration/runtime_xyz/bundle.json',
             'third_party/scanCONTROL/include/InterfaceLLT_2.h',
             'third_party/scanCONTROL/source/InterfaceLLT_2.cpp',
             'third_party/zmc/include/zauxdll2.h']:
    check((root/item).is_file(), f'Bundled build/runtime resource missing: {item}')

print('PASS: LLT SDK signature and H3 CMM; H1/H3 UI+host+BASIC handshake; mode 2 retired; 321058/1316; vendor SDK headers')
