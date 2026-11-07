#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
widget = (ROOT / 'src/widgets/SingleSphere14CalibrationWidget.cpp').read_text(encoding='utf-8')
header = (ROOT / 'include/widgets/SingleSphere14CalibrationWidget.h').read_text(encoding='utf-8')
main = (ROOT / 'src/app/main.cpp').read_text(encoding='utf-8')
window = (ROOT / 'src/app/MainWindow.cpp').read_text(encoding='utf-8')
readme = (ROOT / 'README_V27_6_42B_VALIDATION_AFTER_CSV_FIX_ZH.md').read_text(encoding='utf-8')

checks=[]
def ck(name, cond, detail=''):
    checks.append((name, bool(cond), detail))

ck('version', '2.0.0-v27.6.42B' in main and 'MotionLaserSync V27.6.42B' in window)
ck('new bind helper declared', 'tryBindCurrentSolvedResultForCalibrationMotion' in header)
ck('new bind helper defined', 'bool SingleSphere14CalibrationWidget::tryBindCurrentSolvedResultForCalibrationMotion' in widget)
ck('validation plan button no liveMotionSession gate',
   'm_btnGenerateValidation->setEnabled(m_hasResult && idleUi && hasStage2Training);' in widget
   and 'm_btnGenerateValidation->setEnabled(liveMotionSession && m_hasResult && idleUi)' not in widget)
ck('validation generation supports offline start',
   'const bool haveLiveStartPose = m_referenceTaught && controllerOnline && sensorOnline' in widget
   and 'seed.hasTaughtJoint = haveLiveStartPose;' in widget
   and 'preflightAndFilterGeneratedPlan(validationStart,haveLiveStartPose' in widget)
ck('offline validation uses finite Machine5X bounds',
   '独立验证规划限位' in widget and '不能离线生成验证姿态' in widget)
ck('auto validation attempts safe rebind',
   'if (!tryBindCurrentSolvedResultForCalibrationMotion(bindError))' in widget
   and '不能重新绑定验证运动会话' in widget)
ck('auto validation button can enable before live bind',
   'const bool canRebindValidationMotion = !m_referenceTaught && controllerOnline && sensorOnline' in widget
   and '(liveAcquisitionSession || canRebindValidationMotion)' in widget)
ck('CSV import clears current persistent authorization',
   'm_persistentCalibrationLoaded=false;' in widget
   and 'm_persistedValidationSummary=ValidationSummary{};' in widget)
ck('old validation is not restored by calibration-only rebind',
   '未恢复任何旧hold-out通过结论' in widget)
ck('Stage2 self-bootstrap gauge checked against initialization anchor',
   'm_lastResult.initializationSource.rfind("stage2_", 0) == 0' in widget
   and 'm_lastResult.initializationParameters' in widget)
ck('holdout requires exact training pose-source convention',
   'currentMask != m_referenceFeedbackMask' in widget
   and 'Hold-out validation must use the SAME pose-source convention' in widget)
ck('production authorization still requires validation',
   'm_validationSummary.passed' in widget and 'scanCalibration' in widget)
ck('workflow documents offline plan then live rebind',
   '直接点击“生成独立验证姿态”' in widget and 'V27.6.42B允许在未绑定实机运动会话时做只读hold-out规划' in widget)
ck('readme covers regression', '旧验证结论不会污染新模型' in readme)

failed=[x for x in checks if not x[1]]
for name, ok, detail in checks:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f": {detail}" if detail else ''))
print(f"\n{len(checks)-len(failed)}/{len(checks)} checks passed")
if failed:
    sys.exit(1)
