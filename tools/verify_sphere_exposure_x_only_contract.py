#!/usr/bin/env python3
"""Structural regression of the moving-X whole-sphere exposure workflow.

It does NOT replace a Windows/Qt build, controller mock, or hardware acceptance.
"""
from pathlib import Path
p=Path(__file__).resolve().parents[1]
sweep=(p/'src/app/MainWindowSphereExposure.cpp').read_text(encoding='utf8')
controller=(p/'src/core/ZmcController.cpp').read_text(encoding='utf8')
ctlheader=(p/'include/core/ZmcController.h').read_text(encoding='utf8')
ui=(p/'src/app/MainWindow.cpp').read_text(encoding='utf8')
view=(p/'src/widgets/PointCloudWidget.cpp').read_text(encoding='utf8')
checks={
    'X-only move in sweep': 'controller->moveLinearMposAbs(AxisX' in sweep,
    'No 5-axis move in sweep': 'controller->moveJointAbs5D(' not in sweep,
    'X-only normal stop': sweep.count('requestStopAxis(AxisX,false)') == 2,
    'No joint stop for sphere sweep': 'requestStopJointMove5D' not in sweep,
    'Measured-MPOS entry declared': 'moveLinearMposAbs(int axis' in ctlheader,
    'Measured-MPOS coordinate mapping': 'command=static_cast<double>(d)+(static_cast<double>(pos)-static_cast<double>(m));' in controller,
    'Controller issues Single_MoveAbs': 'ZAux_Direct_Single_MoveAbs(m_handle, axis, controllerTarget)' in controller,
    'Home-reference safety gate': 'isLinearMposReferenced(axis)' in controller,
    'Coordinate limits checked': 'validateLinearCommand(axis,pos,controllerTarget,err)' in controller,
    'Non-X guard records actual deltas': 'lockedAxisDeltaText' in sweep and '完成X移动后非X轴位置保护触发' in sweep,
    'MachineSafety transitions checked': 'checkSphereStudyTransitionSafety(current.mpos,target' in sweep,
    'Plan preview table': 'm_tableSphereExpPlan = new QTableWidget' in ui,
    'Pre-run plan visualization': 'm_pointCloudView->setSphereExposureSweepPath(trace)' in sweep,
    'No movement during planning': 'void MainWindow::onPlanSphereExposureSweep()' in sweep.split('void MainWindow::onStartSphereExposureSweep()')[0],
    'Start remains gated until planned': 'm_btnSphereExpStart->setEnabled(false); // cannot scan before' in ui,
}
for name,ok in checks.items():
    print(('PASS' if ok else 'FAIL')+' '+name)
if not all(checks.values()): raise SystemExit(1)
print(f'{len(checks)}/{len(checks)} static safety contracts passed; Windows Qt and hardware tests remain necessary.')
