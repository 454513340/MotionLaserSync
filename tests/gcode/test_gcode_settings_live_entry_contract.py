"""Source/portable tests; never connects to controller, laser, PWM or real axes."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
logic = root / 'tests/gcode/gcode_preflight_logic_tests.cpp'
with tempfile.TemporaryDirectory(prefix='gcode-preflight-') as wd:
    exe = Path(wd) / 'settings_contract'
    subprocess.run(['g++', '-std=c++17','-O2', '-Wall','-Wextra',
                    '-I', str(root / 'include'), str(logic), '-o', str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

main = (root/'src/app/MainWindow.cpp').read_text(encoding='utf8')
gui = (root/'src/widgets/GCodeWidget.cpp').read_text(encoding='utf8')
execsrc = (root/'src/gcode/GCodeExecutor.cpp').read_text(encoding='utf8')
assert 'm_gcodeWidget->setHardwareSyncUiIntent(topology,frequency,edge,manualHardware)' in main
assert 'const SyncMode mode = SyncMode::Software;' not in main[main.index('void MainWindow::onGCodeRequestSensorParams()'):]
assert 'inspectRealRunReadOnly(m_program,text,m_lastFilePath)' in gui
assert 'emit requestHardwareNcProbe(m_program,m_preflightNcText)' in gui
assert 'classifyHardwareSettings(' not in execsrc
assert 'requestStageHardwareSettings(m_program)' in gui
assert 'stageGCodeHardwareSettings(const GCodeProgram& nc)' in main
assert 'setHardwareTriggerFrequencyHz(frequency)' in main
assert 'setTriggerType(' not in main[main.index('void MainWindow::stageGCodeHardwareSettings('):main.index('void MainWindow::onGCodeFullNcGeometryAuditRequested(')]
assert 'm_siteEntryMoveVerifier(measured,firstTarget,digest' in execsrc
assert 'if(m_program.offlineOnly)' in execsrc
assert 'if(!m_siteHardwareNcVerifier)' in execsrc
assert 'hwTableCommissioningPreflight(' in execsrc
print('PASS: NC controls H1/H3/Hz, cache not an equipment proof, real gates retained')
