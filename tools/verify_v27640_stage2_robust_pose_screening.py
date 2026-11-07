from pathlib import Path
root=Path(__file__).resolve().parents[1]
header=(root/'include/calibration/SingleSphere17Calibration.h').read_text(encoding='utf-8')
core=(root/'src/calibration/SingleSphere17Calibration.cpp').read_text(encoding='utf-8')
widget=(root/'src/widgets/SingleSphere14CalibrationWidget.cpp').read_text(encoding='utf-8')
cli=(root/'tools/single_sphere17_cli.cpp').read_text(encoding='utf-8')
tests=(root/'tests/v20_2_single_sphere17_tests.cpp').read_text(encoding='utf-8')
# Historical V27.6.40 compatibility contract: V27.6.41 intentionally changes
# the exact thresholds/round count, so only the retained architectural promises
# are checked here.  V27.6.41 has its own stricter source contract.
checks={
 'screening_api': 'Physical17PoseScreeningReport screenPoseOutliers' in header and 'SingleSphere17Calibrator::screenPoseOutliers' in core,
 'mad_statistics': '1.4826 * report.madRmsMm' in core and 'medianRmsMm' in header,
 'coverage_guard': 'minRotarySpanFraction' in core and 'minPositiveNegativePoseCount' in core,
 'gui_rebootstrap': 'const NominalHandEyeSphereResult filteredBootstrap = solveBootstrap(proposedBootstrapData);' in widget,
 'gui_rank_guard': 'expectedTargetRank' in widget and 'expectedFullStateRank' in widget,
 'json_provenance': 'pose_screening' in core and 'rejectedTrainingPoses' in core,
 'cli_default': '--no-pose-screening' in cli and 'screenPoseOutliers' in cli,
 'regression_test': 'testRobustPoseScreeningRejectsBadPoseGroups' in tests,
}
failed=[k for k,v in checks.items() if not v]
for k,v in checks.items(): print(f'{k}: {"PASS" if v else "FAIL"}')
if failed:
    raise SystemExit('FAILED: '+', '.join(failed))
print(f'V27.6.40 compatibility contract: {len(checks)}/{len(checks)} PASS')
