#!/usr/bin/env python3
"""Source-only guard for BC37 GUI, not a Qt/VTK runtime smoke test."""
from pathlib import Path
root=Path(__file__).resolve().parents[2]
source=(root/'src/widgets/MultiPoseExperimentWidget.cpp').read_text(encoding='utf-8')
core=(root/'src/twin/MultiPoseExperiment.cpp').read_text(encoding='utf-8')
assert 'restoreStandardMatrix();' in source
assert '恢复 B/C 37组' in source
assert '已装入37组' in source
assert '已载入标准37组' in source
assert '恢复标准 B/C 45组' not in source
assert 'for(int B=-10;B<=10;B+=4)' in core
assert 'for(int C=-10;C<=10;C+=4)' in core
assert 'poses.push_back({"P37",0.0,0.0});' in core
assert source.index('m_btnImportNc=new QPushButton') < source.index('restoreStandardMatrix();')
print('BC37 UI source contract PASS (not Windows GUI runtime)')
