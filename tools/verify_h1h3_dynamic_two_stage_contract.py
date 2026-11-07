#!/usr/bin/env python3
"""Static integration contract (NOT a replacement for Windows full build)."""
from pathlib import Path
root = Path(__file__).resolve().parents[1]
cmake = (root / 'CMakeLists.txt').read_text(encoding='utf-8')
impl = (root / 'src/calibration/SphereTwoStageFit.cpp').read_text(encoding='utf-8')
hdr = (root / 'include/calibration/SphereTwoStageFit.h').read_text(encoding='utf-8')
widget = (root / 'src/widgets/HardwareSyncCalibrationWidget.cpp').read_text(encoding='utf-8')
uihdr = (root / 'include/widgets/HardwareSyncCalibrationWidget.h').read_text(encoding='utf-8')
gate = (root / 'include/calibration/SphereArcQualityGate.h').read_text(encoding='utf-8')
assert cmake.count('"${CMAKE_CURRENT_SOURCE_DIR}/src/calibration/SphereTwoStageFit.cpp"') == 2, 'missing explicit target_sources plus EXISTS guard'
assert 'target_sources(MotionLaserSync PRIVATE\n    "${CMAKE_CURRENT_SOURCE_DIR}/src/calibration/SphereTwoStageFit.cpp"' in cmake
for symbol in ('fitImportedSphere', 'comparisonText'):
    assert 'SphereTwoStageFit::'+symbol in impl, symbol+' definition not found'
    assert symbol+'(' in hdr, symbol+' declaration not found'
assert widget.count('const auto legacyFit=SphereDistanceAngleStudy::fitMachineSphereRansac(cloud,opt);') == 1
assert 'qualifySphereArc(arc,quality,expectedRadius)' in widget
assert 'selectedCloud.push_back(cloud[idx]);' in widget
assert 'selectedRefs.push_back(sampleRefs[idx]);' in widget
assert 'fit=SphereDistanceAngleStudy::fitMachineSphereRansac(selectedCloud,refined);' in widget
assert 'inlierRefs.push_back(selectedRefs[sampleIdx]);' in widget
assert 'rawMatched.push_back(in[sampleNo]);' in widget
assert 'corr.push_back({mapped[0],mapped[1],mapped[2]});' in widget
assert widget.index('rawMatched.push_back(in[sampleNo]);') < widget.index('corr.push_back({mapped[0],mapped[1],mapped[2]});')
assert 'radialRmsLocal(rawMatched,referenceCenter' in widget
assert 'vc.inliers=rawMatched.size();' in widget
assert 'kDynamicSphereFitContract' in widget
assert 'dynamicSphereFitContract' in widget
assert 'SphereArcQualityGate.h' in hdr and 'SphereArcQualityGate.h' in widget
assert 'arcQualifiedProfiles' in uihdr
assert 'projectSphereCenterToSensorArcXZ' in gate
assert (root/'controller/ECI3808_V32_16_H1_H3_ONLY.bas').exists()
print('PASS H1/H3 dynamic sphere two-stage, matched holdout, contract, explicit linker sources')
