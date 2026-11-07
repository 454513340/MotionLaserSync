#!/usr/bin/env python3
"""Structural assertions for source wiring; not a substitute for Qt/LLT hardware tests."""
from pathlib import Path
import json
import re
ROOT=Path(__file__).resolve().parents[1]
window=(ROOT/'src/app/MainWindow.cpp').read_text(encoding='utf8')
exp=(ROOT/'src/app/MainWindowSmall7Experiment.cpp').read_text(encoding='utf8')
core=(ROOT/'src/calibration/Small7HeightPlan.cpp').read_text(encoding='utf8')
header=(ROOT/'include/app/MainWindow.h').read_text(encoding='utf8')
laser=(ROOT/'src/core/LaserSensor.cpp').read_text(encoding='utf8')
start=window.split('void MainWindow::onStartProfilePreview()',1)[1].split('void MainWindow::onStartSingleSphereCalibrationPreview()',1)[0]
stop=window.split('void MainWindow::onStopProfilePreview()',1)[1].split('void MainWindow::onPreviewModeChanged()',1)[0]
mirror=start.split('if (m_syncManager->isRunning()) {',1)[1].split('if (m_laser->isVideoRunning()) {',1)[0]
# Mirror block includes nested video conflict branch; verify full early-return before software configuration.
mirror=start[:start.index('    if (m_laser->isProfileRunning())')]
assert 'm_profilePreviewMirrorsCapture = true' in mirror
assert 'return;' in mirror
assert 'm_laser->configure' not in mirror and 'm_laser->stopContinuous' not in mirror
assert 'm_laser->stopContinuous' in stop and stop.index('m_profilePreviewMirrorsCapture') < stop.index('m_laser->stopContinuous')
assert '独立预览不会擅自停流' in start
assert 'm_profilePreviewMirrorSinceMs=QDateTime::currentMSecsSinceEpoch();' in window
assert 'm_profilePreviewMirrorBaselineOrdinal' in window
assert 'm_profilePreviewMirrorFirstFrameSeen' in header and 'm_profilePreviewMirrorFirstFrameSeen=true' in window
assert 'const bool canMirror=' in window and '|| canMirror' in window
assert 'if(m_profilePreviewMirrorsCapture)onStopProfilePreview();' in window
assert '不会自动改触发设置' in window
assert '非硬件同步测量' in start
assert 'm_comboSmall7Field=new QComboBox' in exp
assert 'm_comboSmall7Field->setCurrentIndex(2)' in exp
assert 'field<128' in exp
assert 'm_spinSmall7Rate->setMaximum(maximum)' in exp
assert 'w.measuringFieldIndex != s.measuringFieldIndex' in core
assert 'referenceMaxProfileHz(s.measuringFieldIndex)' in core
assert 'm_small7Optics.measuringFieldIndex!=selectedField' in exp
assert 'params.measuringFieldIndex=m_small7Settings.measuringFieldIndex' in exp
assert 'liveField!=params.measuringFieldIndex' in exp
assert 'session["measuringFieldIndex"]=m_small7Settings.measuringFieldIndex' in exp
assert 'summary["measuringFieldIndex"]=m_small7Settings.measuringFieldIndex' in exp
# All 128 ROI reference frequencies MUST agree with actual LaserSensor device table.
def reference_table(source, variable):
    match=re.search(r'\b'+variable+r'\[16\]\[8\]\s*=\s*\{(.*?)\n\s*\};',source,re.S)
    assert match, f'Frequency table not found: {variable}'
    body=re.sub(r'//[^\n]*','',match.group(1))
    rows=re.findall(r'\{([^{}]*)\}',body)
    result=[[int(x) for x in re.findall(r'\d+',row)] for row in rows]
    assert len(result)==16 and all(len(row)==8 for row in result)
    assert min(x for row in result for x in row)>=25
    return result
assert reference_table(laser,'freqTable')==reference_table(core,'table'), 'Planner and hardware frequency limits DIVERGE'
for name,expected_index in [('large',0),('standard',2),('small',7)]:
    path=ROOT/f'config/experiment3_{name}_field_{expected_index}_verified_roi.example.json'
    data=json.loads(path.read_text(encoding='utf8'))
    assert data['measuringFieldIndex']==expected_index
    assert data['verified'] is False, f'Unmeasured optical boundary accidentally enabled: {path}'
print('PASS: 28 hardware preview / multi-field structure, 128-frequency cross-check, and unverified-config assertions')
