#!/usr/bin/env python3
"""Static source contract, NOT a Windows Qt/VTK runtime proof."""
from pathlib import Path
root=Path(__file__).resolve().parents[2]
ui=(root/'src/widgets/MultiPoseExperimentWidget.cpp').read_text(encoding='utf-8')
adapter=(root/'src/twin/MultiPoseExperimentQtAdapter.cpp').read_text(encoding='utf-8')
planning=(root/'src/twin/MultiPoseExperiment.cpp').read_text(encoding='utf-8')
playback=(root/'src/twin/MultiPosePlayback.cpp').read_text(encoding='utf-8')
expected_ui=[
    'executeConnectedNc(',
    'makeConnectedNcPlaybackTimeline(',
    'sampleCurrentNcPlayback(',
    'm_connectedNcUsable',
    'm_connectedNcReplay.vertices',
    '不能在无转场指令时瞬间跳到下一起点',
    '不能在独立姿态的两份 NC 之间单步瞬移',
    'MP_ALL_POSES_CONNECTED_OFFLINE_ONLY',
]
for token in expected_ui[:-1]:assert token in ui,token
assert expected_ui[-1] in adapter
for token in ['planConnectedExperimentSession(',
              'if(plan.views.size()>1&&!collision)',
              'M6SegmentRole::Transfer',
              'const auto why=badPath(joints,options,collision)',
              'duration=std::max(duration',
              'out.poseStartSeconds.push_back(joinedAt)']:
    assert token in planning,token
for token in ['sampleConnectedNcPlayback(',
              'sampleMultiPoseNc(nc,at)',
              'out.betweenPoses=movingBetween',
              'out.scanning=sampled.scanning&&!movingBetween']:
    assert token in playback,token
for token in ['executeConnectedNc(',
              'simulateMultiPoseNc(result.text,r.options.maximumProfiles)',
              'verifyMultiPoseNcAgainstPlan(nc,r.connectedSession.merged)',
              'verifiedLaserOffConnections']:
    assert token in adapter,token
assert 'const auto report=m_report,scene=m_scene' not in ui
print('No-teleport connected NC Qt source contract PASS (NOT GUI runtime tested)')
