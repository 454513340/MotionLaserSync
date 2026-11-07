#!/usr/bin/env python3
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
ss = (root/'src/widgets/SingleSphere14CalibrationWidget.cpp').read_text(encoding='utf-8')
ssh = (root/'include/widgets/SingleSphere14CalibrationWidget.h').read_text(encoding='utf-8')
cal = (root/'src/widgets/CalibrationWidget.cpp').read_text(encoding='utf-8')
calh = (root/'include/widgets/CalibrationWidget.h').read_text(encoding='utf-8')
mw = (root/'src/app/MainWindow.cpp').read_text(encoding='utf-8')

checks = []
def check(name, cond):
    checks.append((name, bool(cond)))

check('global-vs-pose preflight enum exists', 'enum class Stage2MovePreflightFailure' in ssh)
check('explicit live motion readiness helper exists', 'checkStage2LiveMotionReadiness' in ss)
check('XYZ DATUM readiness is checked', 'areLinearMposReferenced()' in ss)
check('B/C absolute readiness is checked', 'isRotaryAbsoluteReady(AxisC)' in ss and 'isRotaryAbsoluteReady(AxisB)' in ss)
check('B/C manual servo readiness is checked', 'isRotaryServoEnabled(AxisC)' in ss and 'isRotaryServoEnabled(AxisB)' in ss)
check('auto acquisition checks readiness before starting', 'QString readinessError;' in ss and '阶段2自动采集尚未启动，计划姿态不会被标记为失败或跳过' in ss)
check('generic mass-skip overwrite removed', 'markPlanStatus(index, tr("已跳过：执行前门禁失败"))' not in ss)
check('global failures pause and preserve pending rows', '当前姿态及剩余姿态均保留为待处理' in ss)
check('pose-specific safety statuses remain detailed', '已跳过：目标安全门禁失败' in ss and '已跳过：路径碰撞门禁失败' in ss)
check('automatic fit does not steal profile tab', 'if (!m_autoStage2Acquisition) emit requestProfileView();' in ss)
check('Stage2 tab entry does not force 2D profile view', 'if (index == m_handEyeTabIndex)' in cal and 'const bool sphereStage' not in cal)
check('Stage2 motion readiness UI exists', '真实运动准备（执行前门禁）' in ss and 'm_lblStage2MotionReadiness' in ssh)
check('quick navigation signal is forwarded', 'requestMotionControlView' in ss and 'singleSphereMotionControlViewRequested' in calh and 'singleSphereMotionControlViewRequested' in mw)
check('Stage2 MainWindow move paths additionally gate XYZ reference', mw.count('XYZ光栅MPOS尚未通过正规DATUM回零建立机床基准') >= 2)
# Preserve V27.6.36 safety fix: collision prediction uses effective compensated speeds.
check('independent safety uses effective compensated speed parameters', 'effectiveIndependentAxisParams5D' in mw and 'checkIndependentTransition' in mw)
check('compensation safety envelope is retained', 'axisCompensationSafetyEnvelopeMm' in mw)

failed = [name for name, ok in checks if not ok]
for name, ok in checks:
    print(('PASS' if ok else 'FAIL') + ': ' + name)
print(f'RESULT: {len(checks)-len(failed)}/{len(checks)} checks passed')
if failed:
    sys.exit(1)
