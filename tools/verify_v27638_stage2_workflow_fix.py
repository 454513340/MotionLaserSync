from pathlib import Path
import sys
root = Path(__file__).resolve().parents[1]
widget = (root/'src/widgets/SingleSphere14CalibrationWidget.cpp').read_text(encoding='utf-8')
wh = (root/'include/widgets/SingleSphere14CalibrationWidget.h').read_text(encoding='utf-8')
ch = (root/'include/widgets/CalibrationWidget.h').read_text(encoding='utf-8')
main = (root/'src/app/MainWindow.cpp').read_text(encoding='utf-8')
ctrl = (root/'src/core/ZmcController.cpp').read_text(encoding='utf-8')
checks = []
def ck(name, ok):
    checks.append((name, bool(ok)))

ck('Stage2 exposes separate XYZ speed scale', 'm_spinStage2XyzSpeedScale' in widget)
ck('Stage2 exposes separate BC speed scale', 'm_spinStage2BcSpeedScale' in widget)
ck('Independent move signal carries XYZ and BC scales', 'double xyzSpeedScale' in wh and 'double bcSpeedScale' in wh and 'double xyzSpeedScale' in ch and 'double bcSpeedScale' in ch)
ck('MainWindow scales a local axis-param copy', 'auto axisParams = currentAxisParams5D();' in main and 'axisParams[axis].speed = static_cast<float>(axisParams[axis].speed * xyzScale)' in main and 'axisParams[axis].speed = static_cast<float>(axisParams[axis].speed * bcScale)' in main)
ck('Single profile gate compares observable |Y_S| magnitude', 'std::abs(std::abs(measuredCenter.y) - std::abs(targetCenter.y))' in widget)
ck('Old signed-Y capture gate removed', 'measuredCenter.y = std::copysign(std::abs(measuredCenter.y)' not in widget)
ck('Move-failed rows are excluded from auto pending selection', 'st.contains(tr("移动失败"))' in widget)
ck('Failed auto move is not retried in same run', '本轮不再自动重试' in widget)
ck('Manual move completion refreshes capture readiness', widget.count('updateCalibrationReadiness();') >= 5)
ck('XYZ speed-only full write does not revoke DATUM', 'axis>=AxisX && axis<=AxisZ && unitsChanged' in ctrl)
ck('BC speed-only full write does not revoke absolute-ready', 'const bool rotaryFrameChanged' in ctrl and 'if (rotaryFrameChanged)' in ctrl)
ck('SetUnits skipped for speed-only write', 'if (unitsChanged) {\n        ret = ZAux_Direct_SetUnits' in ctrl)
ck('Solver retains 8-pose hard floor', 'poseCount < 8' in widget or 'poseCount<8' in widget)

failed = [n for n, ok in checks if not ok]
for n, ok in checks:
    print(('PASS ' if ok else 'FAIL ') + n)
print(f'{len(checks)-len(failed)}/{len(checks)} passed')
if failed:
    sys.exit(1)
