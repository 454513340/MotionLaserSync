#!/usr/bin/env python3
"""Source-level guard for the V32.16 H3 signed-temporal extension.

This checks integration intent; it is NOT a substitute for a Windows/Qt build
or ECI3808 + scanCONTROL real-hardware acceptance testing.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

def get(name):
    return (ROOT / name).read_text(encoding="utf-8")

def require(filename, *expected):
    s = get(filename)
    for item in expected:
        assert item in s, f"{filename}: missing {item}"

ui = get("src/widgets/HardwareSyncCalibrationWidget.cpp")
app = get("src/app/MainWindow.cpp")
fn = app[app.index("bool MainWindow::startHardwareTemporalRawCapture("):
         app.index("void MainWindow::pollHardwareTemporalSyncReady(")]
assert "m_calibrationWidget->hardwareTemporalCalibrationTopologyId()" in fn
assert "m_calibrationWidget->hardwareTemporalExpectedR1Edge()" in fn
assert "m_syncManager->setHardwareTopology(" in fn
assert "setHardwareTopology(mls::runtime::HardwareSyncTopology::EciPwmShared" not in fn
assert "H2" not in ui[ui.index("HardwareTemporalCorrectionSettings HardwareSyncCalibrationWidget::correctionSettings"):
                           ui.index("QString HardwareSyncCalibrationWidget::activePath")]
require("src/widgets/HardwareSyncCalibrationWidget.cpp",
        '"calibration/active_hardware_temporal_H1.json"',
        '"calibration/active_hardware_temporal_H3.json"',
        'm_sessionLatchEdge=completedEdge;',
        'root["topologyId"]=m_selectedTopologyId',
        'root["latchEdge"]=m_sessionLatchEdge',
        'root["schemaVersion"]=4',
        'c.allowSignedDelay=(m_selectedTopologyId==3)',
        'const bool forward=candidateDelay>=0.0;',
        'm_selectedTopologyId==3 ? 1 : 0',
        'm_selectedTopologyId==1 ? 104 : m_sessionLatchEdge',
        'calibrationTopology==3')
require("src/runtime/HardwareTemporalCorrection.cpp",
        'std::abs(config.delaySec) >= config.triggerPeriodSec',
        'current.ordinal >= 2',
        'neighbor.ordinal + 1 == current.ordinal',
        'current.ordinal + 1')
require("src/runtime/Acquisition5D.cpp",
        'const bool backward = f.temporalOffsetSec < 0.0',
        'f.temporalNextOrdinal+1 == f.ordinal')
require("src/core/SyncManager.cpp",
        'm_lastCaptureCmmR1Edge =',
        'c.topology != m_hardwareTopology',
        'c.latchEdge != currentEdge',
        'm_temporalDelaySec >= 0.0 ? ordinal + 1 : ordinal - 1')
require("src/calibration/HardwareTemporalCalibration.cpp", 'c.allowSignedDelay', 'std::abs(r.delaySec)')
require("tests/h3_signed_temporal_tests.cpp", 'H3 accepts measured negative tau', 'completeFrame accepts H3 negative provenance')
old_bas = get("controller/ECI3808_V32_16_H1_H3_ONLY.bas")
assert "321058" in old_bas
print("PASS: H3 signed-calibration wiring, isolated storage and H1/H3 guards are present")
