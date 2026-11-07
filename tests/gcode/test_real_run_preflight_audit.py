"""Non-actuating regression for the V29 G-code readiness inspector."""
import hashlib
import importlib.util
import json
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[2]
script = root / 'tools' / 'audit_multi_pose_nc_for_real_run.py'
spec = importlib.util.spec_from_file_location('nc_audit', script)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

nc = ('''\
; OfflineOnly: true
; RegistrationVerified: false
; SimulationVerified: false
; InitialJointAnchored: false
; HardwareTopology: H3
; CmmR1Edge: 114
; PartPoseId: UNVERIFIED
; CalibrationId: UNVERIFIED
G21
G90
G93
G00 X1 Y2 Z3 C0 B0
M50
G05 S1 F100
X0 Y2 Z3 C0 B0 F600
M51
M60 L"P01"
M61
M63
M30
''').encode('utf-8')
with tempfile.TemporaryDirectory() as temp:
    path = Path(temp) / 'synthetic.nc'; path.write_bytes(nc)
    manifest = Path(temp) / 'manifest.json'; manifest.write_text(json.dumps({
        'hardwareAuthorized': False,
        'registrationVerified': False,
        'opticsAcrossAnglesVerified': False,
        'continuousNcSession': {
            'sha256': hashlib.sha256(nc).hexdigest(),
            'firstG00StartUnverified': True,
        }
    }))
    result = module.audit(path, manifest)
    assert not result['hardwareExecutionAuthorized']
    assert result['manifest']['ncSha256Matches']
    assert result['firstG00'] == [1.,2.,3.,0.,0.]
    assert result['commandCounts']['M50'] == 1
    assert result['commandCounts']['M51'] == 1
    assert len(result['blockingReasons']) > 5
    manifest.write_text(json.dumps({
        'continuousNcSession': {'sha256': '0'*64}}))
    assert any('SHA256' in line for line in module.audit(path, manifest)['blockingReasons'])

executor = (root / 'src/gcode/GCodeExecutor.cpp').read_text(encoding='utf-8')
start = executor.find('GCodeExecutor::RealRunPreflightReport GCodeExecutor::inspectRealRunReadOnly(')
end = executor.find('// ==================== 执行控制 ====================', start)
assert start != -1 and end > start
inspect = executor[start:end]
for unsafe in ('prepareMergePath(', 'pumpMergeBuffer(', 'startSyncCapture(',
               'moveAbs(', 'moveRel(', 'SyncManager::start(', 'startMotion('):
    assert unsafe not in inspect, unsafe
start_idx = executor.find('bool GCodeExecutor::start(QString* err)')
start_body = executor[start_idx:executor.find('bool GCodeExecutor::resume(',start_idx)]
assert 'if(m_program.offlineOnly)' in start_body
assert 'if(!m_siteHardwareNcVerifier)' in start_body
assert 'm_siteEntryMoveVerifier(measured,firstTarget,digest' in start_body
widget = (root/'src/widgets/GCodeWidget.cpp').read_text(encoding='utf-8')
assert 'm_btnRealPreflight' in widget
assert 'onRealRunPreflightClicked()' in widget
print('read-only NC audit, negative SHA test, preserved machine gates, and UI integration: PASS')
