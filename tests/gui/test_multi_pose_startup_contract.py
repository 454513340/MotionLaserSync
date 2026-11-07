"""Source regression guard for the multi-pose startup crash and launcher placement.

This does NOT substitute for running the Windows Qt --smoke-ui test.  It prevents
reintroducing the premature addOrientation -> invalidate null pointer path and
eager creation of the experimental tab during MainWindow construction.
"""
from pathlib import Path

root = Path(__file__).resolve().parents[2]
mp = (root / "src/widgets/MultiPoseExperimentWidget.cpp").read_text(encoding="utf-8")
main = (root / "src/app/MainWindow.cpp").read_text(encoding="utf-8")
path = (root / "src/widgets/PathPlanningWidget.cpp").read_text(encoding="utf-8")
app = (root / "src/app/main.cpp").read_text(encoding="utf-8")

# Both export controls and status label are constructed before initial rows,
# and invalidation is additionally null-safe if a future refactor reorders UI.
assert mp.index('m_btnExport=new QPushButton') < mp.index('restoreStandardMatrix();')
assert mp.index('m_summary=new QLabel') < mp.index('restoreStandardMatrix();')
assert 'if(m_btnExport)m_btnExport->setEnabled(false);' in mp
assert 'if(m_btnPreviewNc)m_btnPreviewNc->setEnabled(false);' in mp
assert 'if(m_summary)m_summary->setText(' in mp
assert 'if(m_log)m_log->appendPlainText(' in mp

# UI isn't built as a top-level tab; it is created only after clicking the
# new button on Path Planning, directly above the original M0-M8 entry.
assert 'new MultiPoseExperimentWidget' not in main
assert 'addTab(mpScrollArea' not in main
assert path.index('mainLayout->addWidget(m_btnMultiPoseExperiment);') < path.index('mainLayout->addWidget(m_btnDigitalTwin);')
assert 'connect(m_btnMultiPoseExperiment, &QPushButton::clicked' in path
assert 'new MultiPoseExperimentWidget(scroll)' in path
assert 'if (m_multiPoseExperimentDialog)' in path
assert 'm_multiPoseExperimentDialog = window;' in path
assert '--smoke-ui' in app
print('PASS: multi-pose startup source guard, lazy launcher and placement')
