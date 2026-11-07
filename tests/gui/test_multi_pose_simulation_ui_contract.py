#!/usr/bin/env python3
"""Textual interface guard, not a substitute for running the Windows GUI."""
from pathlib import Path
root = Path(__file__).resolve().parents[2]
widget=(root/'src/widgets/MultiPoseExperimentWidget.cpp').read_text(encoding='utf-8')
viewer=(root/'src/widgets/PointCloudWidget.cpp').read_text(encoding='utf-8')
core=(root/'src/twin/MultiPoseExperimentQtAdapter.cpp').read_text(encoding='utf-8')
assert 'm_btnPlay=new QPushButton' in widget
assert 'm_btnPause=new QPushButton' in widget
assert 'm_btnStep=new QPushButton' in widget
assert 'm_btnCompare=new QPushButton' in widget
assert 'm_playTimer=new QTimer(this)' in widget
assert widget.index('m_btnPlay=new QPushButton') < widget.index('restoreStandardMatrix();'), 'Startup risk: control is used before construction'
assert 'setTwinMultiPosePlaybackFrame(' in widget
assert 'm_view->setTwinMultiPosePlaybackActive(false,clearCloud)' in widget
assert 'updateMachineActorTransforms();' in viewer and 'm_multiPosePlaybackActive' in viewer
assert 'MULTIPOSE_PLAYBACK' in viewer and 'MULTIPOSE_COMPARISON' in viewer
assert 'PoseTiming::Simulated' in core
assert 'compareMultiPoseClouds' in core
assert 'QSaveFile ply(plyPath)' in core
assert 'ALL_POSES_SIMULATION_ONLY_UNFUSED.ply' in core
assert 'H1' in core and 'H3' in core
print('MultiPose GUI static-contract checks passed (NOT a Windows Qt GUI runtime test)')
