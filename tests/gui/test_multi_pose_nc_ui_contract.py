#!/usr/bin/env python3
"""Text-only GUI integration guard: no Qt/VTK Windows runtime is claimed."""
from pathlib import Path
r=Path(__file__).resolve().parents[2]
w=(r/'src/widgets/MultiPoseExperimentWidget.cpp').read_text(encoding='utf-8')
a=(r/'src/twin/MultiPoseExperimentQtAdapter.cpp').read_text(encoding='utf-8')
p=(r/'src/twin/MultiPosePlayback.cpp').read_text(encoding='utf-8')
s=(r/'src/twin/MultiPoseNcSimulation.cpp').read_text(encoding='utf-8')
assert w.index('m_btnImportNc=new QPushButton') < w.index('restoreStandardMatrix();'), 'startup-order regression'
for token in ['rebuildNcReplays(', 'importNcForPose(', 'm_activeNcReplays',
              'm_overriddenNcByPose', 'm_btnPlay=new QPushButton(tr("按已解析 G代码播放五轴+传感器")']:
    assert token in w, token
assert 'makeMultiPosePlaybackTimeline(d->plan)' not in w
assert 'makeNcDrivenPlaybackTimeline(executions)' in w
assert 'sourceNc.profiles[frame]' in a
assert 'view.exposures[frame]' not in a
assert 'executeNcForPose(r,i,topo,h3Edge,&saved)' in a
assert 'ncSourceSha256' in a and 'G代码与规划不同，不生成虚拟点云' in a
assert 'sampleMultiPoseNc(runs[seg->poseIndex]' in p
for token in ['parseNcBlock(line,modal,block,parserError)', '60.0/(*block.feed)',
              'M50', 'M51', 'G43.4 RTCP is invalid', 'initialG00Unverified',
              'verifyMultiPoseNcAgainstPlan']:
    assert token in (s+(r/'include/twin/MultiPoseNcSimulation.h').read_text(encoding='utf-8')), token
print('NC-driven GUI integration static checks PASS (Windows GUI not executed)')

assert 'standardBCExperimentMatrix()' in w
assert 'm_leadIn' in w and 'm_leadOut' in w
assert 'AutoMachineX' in w
assert 'm_poses->rowCount()==3' not in w
assert 'const auto report=m_report,scene=m_scene' not in w
