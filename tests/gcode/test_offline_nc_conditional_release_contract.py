"""Static integration regression (not a controller/PLC/Windows runtime test)."""
from pathlib import Path
p=Path(__file__).resolve().parents[2]
exe=(p/'src/gcode/GCodeExecutor.cpp').read_text(encoding='utf-8')
ui=(p/'src/widgets/GCodeWidget.cpp').read_text(encoding='utf-8')
app=(p/'src/app/MainWindow.cpp').read_text(encoding='utf-8')
h=(p/'include/app/MainWindow.h').read_text(encoding='utf-8')
widget_h=(p/'include/widgets/GCodeWidget.h').read_text(encoding='utf-8')
assert 'const bool validCandidate = needsCapture' in exe
assert 'm_program.programFrame==QStringLiteral("MACHINE_JOINTS_XYZCB")' in exe
assert 'if(!m_siteHardwareNcVerifier)' in exe
assert 'm_siteHardwareNcVerifier(m_program,digest,&approvalError)' in exe
assert 'm_siteEntryMoveVerifier(measured,firstTarget,digest,&entryError)' in exe
assert 'diskProgram.blocks[i].rawLine!=m_program.blocks[i].rawLine' in exe
assert 'diskProgram.motionParams.accel!=m_program.motionParams.accel' in exe
assert 'const bool commissionedCandidate = m_program.offlineOnly' in ui
assert 'm_btnStart->setEnabled(canStart)' in ui
assert 'm_btnStep->setEnabled(canStep)' in ui
assert 'emit requestHardwareNcProbe(m_program,m_preflightNcText)' in ui
assert 'void requestHardwareNcProbe(const GCodeProgram& nc,const QString& originalNcText)' in widget_h
assert 'void MainWindow::onGCodeHardwareNcProbeRequested(const GCodeProgram& nc,const QString& originalNcText)' in app
assert 'setSiteHardwareNcVerifier(' in app
assert 'if(!m_siteSafetyReader)return reject(' in app
assert 'm_geometryNcProof.passed || !m_hardwareNcProof.passed' in app
assert 'm_geometryNcProof.normalizedNcDigest!=key' in app
assert 'm_hardwareNcProof.normalizedNcDigest!=key' in app
assert 'current[i]!=m_geometryNcProof.model[i]' in app
assert 'current[i]!=m_hardwareNcProof.model[i]' in app
assert 'now-m_geometryNcProof.finishedAt>std::chrono::minutes(20)' in app
assert 'now-m_hardwareNcProof.finishedAt>std::chrono::minutes(5)' in app
assert 'm_controllerSensorConnectionGeneration' in app
assert 'm_hardwareNcProof.topology!=nc.hardwareTopologyId' in app
assert 'm_hardwareNcProof.frequency!=nc.hardwareFrequencyHz' in app
assert 'm_hardwareNcProof.edge!=(nc.hardwareTopologyId==1?104:nc.cmmR1Edge)' in app
assert 'currentReport!=m_approvedGeometryReport' in app
assert 'safety.realOpticalCoverageQualified' in app
assert 'safety.actualFixtureMatchesFclModel' in app
assert 'safety.independentlyWiredProtectiveStop' in app
assert 'safety.sampleAgeMs>250' in app
assert 'using SiteSafetyReader = std::function<bool(SiteSafetyInputs&,QString*)>' in h
assert 'SiteSafetyReader m_siteSafetyReader' in h
assert 'm_hardwareNcProof.passed=verdict.sessionValid && reportSaved' in app
for func in ['onControllerConnect','onControllerDisconnect','onSensorConnect','onSensorDisconnect']:
    segment=app.split(f'void MainWindow::{func}()',1)[1][:220]
    assert '++m_controllerSensorConnectionGeneration' in segment
assert 'return true;' in app.split('setSiteHardwareNcVerifier(',1)[1].split('});',1)[0]
print('PASS: offline candidate is conditionally releasable only with same NC/model, live probe, FCL/optics, fresh PLC/fixture/real optical evidence; still fails closed absent site provider')
