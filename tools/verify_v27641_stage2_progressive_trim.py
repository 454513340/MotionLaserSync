from pathlib import Path
root=Path(__file__).resolve().parents[1]
header=(root/'include/calibration/SingleSphere17Calibration.h').read_text(encoding='utf-8')
core=(root/'src/calibration/SingleSphere17Calibration.cpp').read_text(encoding='utf-8')
widget=(root/'src/widgets/SingleSphere14CalibrationWidget.cpp').read_text(encoding='utf-8')
cli=(root/'tools/single_sphere17_cli.cpp').read_text(encoding='utf-8')
main=(root/'src/app/main.cpp').read_text(encoding='utf-8')
window=(root/'src/app/MainWindow.cpp').read_text(encoding='utf-8')
tests=(root/'tests/v20_2_single_sphere17_tests.cpp').read_text(encoding='utf-8')
checks={
 'version': '2.0.0-v27.6.41' in main and 'MotionLaserSync V27.6.41' in window,
 'stricter_gate': 'std::min(rmsRobustGate, rmsRelativeGate)' in core and 'madSigma = 1.75' in header,
 'progressive_options': 'progressiveTrimFraction = 0.12' in header and 'targetTrainingRmsMm = 0.045' in header,
 'progressive_tail': 'progressiveCandidateCount' in core and 'desired = std::min' in core,
 'xyzbc_coverage': 'minLinearSpanFraction' in header and 'originalXSpan' in core and 'minRotarySpanFraction' in core,
 'four_rounds': 'constexpr int kMaxScreeningRounds = 4' in widget,
 'cumulative_cap': 'kMaxCumulativeRejectedFraction = 0.40' in widget,
 'trial_refit': 'trialOptions.jointIterations = 25' in widget and 'trialCalibrator.solve' in widget,
 'penalized_objective': 'kSelectionPenaltyMm = 0.020' in widget and 'selectionObjectiveAfter' in widget,
 'adaptive_batch': '2*report.rejectedPoseCount' in widget and 'limitRejectedPoseCount' in widget,
 'rank_condition_guard': 'relativeConditionLimit' in widget and 'expectedTargetRank' in widget,
 'json_version': '20.2-single-sphere17-rank14-v27.6.41' in core,
 'cli_progressive': 'maxRounds=4' in cli and 'Accepted progressive trim' in cli,
 'regression_test': 'testProgressivePoseScreeningBreaksOutlierMasking' in tests,
}
failed=[k for k,v in checks.items() if not v]
for k,v in checks.items(): print(f'{k}: {"PASS" if v else "FAIL"}')
if failed:
    raise SystemExit('FAILED: '+', '.join(failed))
print(f'V27.6.41 source contract: {len(checks)}/{len(checks)} PASS')
