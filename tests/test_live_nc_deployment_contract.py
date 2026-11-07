"""Source-level contract for the transition from multi-pose offline planning.

Does not enable real motor/laser movement. Run with Python 3 on Windows/Linux.
"""
from pathlib import Path
r = Path(__file__).resolve().parents[1]
adapter = (r/'src/twin/MultiPoseExperimentQtAdapter.cpp').read_text(encoding='utf-8')
executor = (r/'src/gcode/GCodeExecutor.cpp').read_text(encoding='utf-8')
widget = (r/'src/widgets/GCodeWidget.cpp').read_text(encoding='utf-8')
assert 'for(int topo:{1,3}){' in adapter, 'Both H1 and H3 connected NC must be exported'
assert 'executeConnectedNc(r,topo,h3Edge,selectedText)' in adapter
assert 'executeConnectedNc(r,topo,h3Edge,&saved)' in adapter, 'Read-back NC must be simulated again'
assert 'MP_ALL_POSES_CONNECTED_OFFLINE_ONLY_H%1.nc' in adapter
assert 'root.insert("continuousNcSessions",connectedSessions)' in adapter
assert 'm_program.offlineOnly' in executor, 'Offline NC candidate must be gated'
assert 'const bool validCandidate = needsCapture' in executor, 'Only explicit hardware NC may request release'
assert 'diskProgram.blocks.size()!=m_program.blocks.size()' in executor, 'Disk and memory programs must match'
assert 'if(!m_siteHardwareNcVerifier)' in executor, 'No unsigned hardware execution'
assert 'if(m_program.offlineOnly)' in executor, 'OfflineOnly must stay fail-closed'
assert 'hwTableCommissioningPreflight(' in executor, 'Controller commissioning gate retained'
assert 'm_syncMgr->setTriggerType(HardwareTriggerType::EdgeRising)' in executor
assert 'm_program.hardwareTriggerType!=QStringLiteral("EdgeRising")' in executor
assert 'm_siteEntryMoveVerifier(measured,firstTarget,digest' in executor, 'Live first G00 trajectory must be approved when motion is needed' 
assert 'current=\u2014' not in executor
assert 'r.requiredActions' in executor
assert 'SiteHardwareNcVerifier' in (r/'include/gcode/GCodeExecutor.h').read_text(encoding='utf-8')
assert 'obj.insert("requiredActions"' in widget
print('PASS: dual NC export and SHA traceability; commissioned hardware path exists, unsigned or unanchored NC still rejected.')
