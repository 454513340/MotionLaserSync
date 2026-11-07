"""Static assertions for V30 integrated 2026-10-24 patch. No Qt GUI or hardware in this environment."""
from pathlib import Path
r=Path(__file__).resolve().parents[2]
s=lambda p:(r/p).read_text(encoding='utf-8')
app=s('src/app/MainWindow.cpp')
ui=s('src/widgets/GCodeWidget.cpp')
exe=s('src/gcode/GCodeExecutor.cpp')
win=s('src/widgets/MultiPoseExperimentWidget.cpp')
policy=s('include/gcode/HardwareNcProbeLogic.h')
assert 'requestHardwareNcProbe(m_program,m_preflightNcText)' in ui
assert 'emit requestStageHardwareSettings(m_program)' in ui
assert 'void MainWindow::stageGCodeHardwareSettings(const GCodeProgram& nc)' in app
stage=app.split('void MainWindow::stageGCodeHardwareSettings(const GCodeProgram& nc)',1)[1].split('void MainWindow::onGCodeFullNcGeometryAuditRequested',1)[0]
assert 'nc.hardwareTopologyId' in stage and 'nc.hardwareFrequencyHz' in stage
assert 'm_comboHardwareTopology' not in stage
assert 'm_syncManager->start(&error)' in app and 'm_syncManager->stop();' in app
assert 'getHardwareAlignStats()' in app and 'getHardwareProfileRecords()' in app
assert 'measuredMedianPeriodMs' in app and 'std::abs(e.measuredMedianPeriodMs' in policy
assert 'm_syncManager->setHardwareAlignedDataEmissionEnabled(false)' in app
assert 'hwTableCommissioningPreflight' in app
assert 'm_machineSafety->isReady()' in app
assert 'simulateMultiPoseNc(program,200000)' in app
assert 'safety->checkTransition' in app
assert 'grid->ingest(sim)' in app and 'laser.simulate(opticalPose)' in app
assert 'snapshotOpticalForExactNc' in win and 'm_scene.hasWorkpiece()' in win
assert 'm_syncMgr->currentKinematicModel(&active)' in exe
assert 'mls::identity::physical17Sha256(active)!=m_program.physical17Sha256' not in exe
assert 'classifyHardwareSettings' not in exe
assert 'if(m_program.offlineOnly)' in exe and 'if(!m_siteHardwareNcVerifier)' in exe
assert 'const bool validCandidate = needsCapture' in exe
assert 'm_siteHardwareNcVerifier(m_program,digest,&approvalError)' in exe
assert 'G91_IncrementalMode' in exe and 'G92_SetPosition' in exe
assert 'm_siteEntryMoveVerifier(liveNow,actualTarget' in exe
assert 'setSiteEntryMoveVerifier' in app
print('NC-aware hardware probe, sensor rate evidence, FCL/optical replay, active calibration and guarded first G00: PASS')
