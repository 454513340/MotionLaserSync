"""Static integration check for imported candidate provenance and hard gates.
Qt/GUI and a physical machine are NOT present in this Linux test environment.
"""
from pathlib import Path
root=Path(__file__).resolve().parents[2]
def source(path):return (root/path).read_text(encoding='utf-8')
identity=source('include/core/Physical17Identity.h')
assert 'QCryptographicHash::Sha256' in identity
assert 'MLS-PHYSICAL17-IEEE754-BE-V1' in identity
assert 'MLS-PART-POSE-CANDIDATE-V1' in identity
assert 'POSE_CANDIDATE_SHA256_' in identity
assert 'CAL17_IMPORTED_SHA256_' in identity
assert 'std::isfinite(value)' in identity
adapter=source('src/twin/MultiPoseExperimentQtAdapter.cpp')
assert 'd->physical17Sha256=mls::identity::physical17Sha256(p.value);' in adapter
assert 'd->partPoseId=mls::identity::candidatePoseId' in adapter
assert 'r.realScanSha256' in adapter
assert adapter.count('opt.physical17Sha256=r.physical17Sha256.toStdString();') == 2
assert 'opts.physical17Sha256=r.physical17Sha256.toStdString();' in adapter
assert 'independentSixDofRegistrationAccepted' in adapter
preview=source('src/twin/TwinM6NcPreview.cpp')
assert 'Physical17Sha256:' in preview
assert 'OfflineOnly: true' in preview
assert 'RegistrationVerified: false' in preview
parser=source('src/gcode/GCodeParser.cpp')
assert 'key=="physical17sha256"' in parser
assert 'program.physical17Sha256=hash' in parser
assert 'program.filePath=QFileInfo(path).absoluteFilePath()' in parser
sync=source('src/core/SyncManager.cpp')
assert 'm_activePhysical17=p;' in sync
assert 'm_activePhysical17.fill(0.0);' in sync
exe=source('src/gcode/GCodeExecutor.cpp')
assert 'm_syncMgr->currentKinematicModel(&active)' in exe
assert 'mls::identity::physical17Sha256(active)!=m_program.physical17Sha256' not in exe
assert 'm_syncMgr->currentKinematicModel(&active)' in exe
assert 'if(m_program.offlineOnly)' in exe
assert 'if(!m_siteHardwareNcVerifier)' in exe
assert 'if(!m_siteEntryMoveVerifier)' in exe
assert 'm_siteHardwareNcVerifier(m_program,digest' in exe
widget=source('src/widgets/MultiPoseExperimentWidget.cpp')
assert '导入数据身份已建立' in widget
assert '已生成 H1/H3' in widget
print('Imported model provenance retained, active numerical model used, no hash enforcement, real release fail-closed: PASS')
