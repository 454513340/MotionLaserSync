#!/usr/bin/env python3
"""V30 GUI/source contract checks; not a substitute for Qt/MSVC or hardware tests."""
from pathlib import Path

R=Path(__file__).resolve().parents[2]
gui=(R/'src/widgets/GCodeWidget.cpp').read_text(encoding='utf-8')
guih=(R/'include/widgets/GCodeWidget.h').read_text(encoding='utf-8')
main=(R/'src/app/MainWindow.cpp').read_text(encoding='utf-8')
mainh=(R/'include/app/MainWindow.h').read_text(encoding='utf-8')
executor=(R/'src/gcode/GCodeExecutor.cpp').read_text(encoding='utf-8')
exec_h=(R/'include/gcode/GCodeExecutor.h').read_text(encoding='utf-8')

checks = {
    'single preflight toolbar entry': 'toolbarLayout->addWidget(m_btnRealPreflight);' in gui,
    'active button is not misleadingly labelled read-only':
      '实机状态诊断（含硬件试采）' in gui and '实机状态诊断（只读）' not in gui,
    'old standalone buttons are removed': not any(x in gui+guih for x in
      ['m_btnOfflineNcCheck','m_btnHardwareNcProbe','m_btnHardwareNcProbeStop',
       'm_btnFullGeometryAudit','onOfflineNcCheckClicked','onHardwareNcProbeClicked',
       'onFullNcGeometryAuditClicked']),
    'same preflight function invokes G parser':
      'void GCodeWidget::onRealRunPreflightClicked()' in gui and
      'parseAndValidate();' in gui.split('void GCodeWidget::onRealRunPreflightClicked()',1)[1].split('void GCodeWidget::showHardwareNcProbeResult',1)[0],
    'same preflight function invokes offline NC replay':
      'simulateMultiPoseNc(source)' in gui,
    'same preflight starts real sync trial after confirmation':
      gui.index('simulateMultiPoseNc(source)') < gui.index('emit requestHardwareNcProbe(m_program,m_preflightNcText)'),
    'trial completion starts real geometry audit':
      gui.index('void GCodeWidget::showHardwareNcProbeResult') <
      gui.index('emit requestFullNcGeometryAudit(m_preflightNcText)') <
      gui.index('void GCodeWidget::showFullNcGeometryAuditResult'),
    'site readback and fresh start gate reused at the end':
      'inspectSiteExecutionQualificationReadOnly' in gui and
      'm_siteHardwareNcVerifier(program,rawDiskDigest,failure)' in exec_h,
    'on beginning a new diagnosis old PASS is invalidated':
      'emit requestInvalidateNcPreflightEvidence();' in gui and
      'm_geometryNcProof.passed=false;' in main and
      'm_hardwareNcProof.passed=false;' in main,
    'start button for an OfflineOnly NC needs current integrated pass':
      '(commissionedCandidate && m_preflightQualified)' in gui,
    'still refuses absent independent on-machine safety reader':
      'if(!m_siteSafetyReader)' in main and 'm_siteSafetyReader(safety,&safetyError)' in main,
    'read-only stage no longer directs user to separate button':
      '专用试采按钮' not in executor and '综合诊断自动按NC试采' in executor,
    'preflight cancel safely requests probe stop':
      'emit requestStopHardwareNcProbe();' in gui and
      'onGCodeHardwareNcProbeStopRequested' in main,
    'preflight cancel safely requests geometry cancellation':
      'emit requestCancelFullNcGeometryAudit();' in gui and
      'm_gcodeGeometryCancel->store(true)' in main,
    'no automatic NC axis execution in diagnostics':
      not any(x in gui.split('void GCodeWidget::onRealRunPreflightClicked()',1)[1].split('void GCodeWidget::onStartClicked()',1)[0]
        for x in ['m_executor->start(', 'emit requestStartNcExecution(', 'm_zmc->move']),
    'V30 identity in project, version and title':
      'project(MotionLaserSync_V30 ' in (R/'CMakeLists.txt').read_text() and
      'OUTPUT_NAME "MotionLaserSync_V30"' in (R/'CMakeLists.txt').read_text() and
      'setApplicationVersion("V30.0.0")' in (R/'src/app/main.cpp').read_text() and
      'MotionLaserSync V30 (' in main,
    'ASCII-only file and folder names in full project':
      all(p.name.isascii() for p in R.rglob('*')),
}

for name,result in checks.items():
    print(('PASS' if result else 'FAIL') + ': '+name)
assert all(checks.values()),'V30 integrated GUI contract failed'
print(f'Passed {len(checks)} V30 source-contract checks (GUI not compiled here)')
