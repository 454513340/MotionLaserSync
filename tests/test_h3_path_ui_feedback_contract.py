"""Portable source-contract checks for the H3 path/session diagnostics.

No Qt SDK or controller hardware is required. Run:
  python tests/test_h3_path_ui_feedback_contract.py
These do not replace Windows Qt GUI integration testing.
"""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
widget = (root / 'src/widgets/HardwareSyncCalibrationWidget.cpp').read_text(encoding='utf-8')
container = (root / 'src/widgets/CalibrationWidget.cpp').read_text(encoding='utf-8')

def function_body(function_name, next_function):
    start = widget.index('void HardwareSyncCalibrationWidget::' + function_name + '(')
    end = widget.index('void HardwareSyncCalibrationWidget::' + next_function + '(', start + 1)
    return widget[start:end]

checks = {
    'buttons use readable 2-column grid': 'planActions->addWidget(m_generateButton,2,0,1,2)' in widget
      and 'ah->addWidget(m_newSessionButton,1,1)' in widget and 'm_status->setWordWrap(true)' in widget,
    'template inputs arranged in narrow-friendly four-column grid':
      'templateLine->addWidget(m_templateMinSpeed,0,1)' in widget
      and 'templateLine->addWidget(m_templateMaxSpeed,0,3)' in widget,
    'generate log on any idle click': '[PATH-REQUEST]' in function_body('generatePaths', 'startCalibrationAcquisition')
      and '[PATH-REJECT]' in function_body('generatePaths', 'startCalibrationAcquisition')
      and '[PATH-OK]' in function_body('generatePaths', 'startCalibrationAcquisition'),
    'old observations block paths as well as session ID':
      '!m_calibrationObservations.empty()||!m_validationCaptures.empty()' in function_body('generatePaths', 'startCalibrationAcquisition'),
    'path reject displays modal + inline feedback':
      'QMessageBox::warning(this,tr("无法生成扫描路径")' in function_body('generatePaths', 'startCalibrationAcquisition')
      and 'm_pathGenerationFeedback->setText(message)' in function_body('generatePaths', 'startCalibrationAcquisition'),
    'system log writes rejects as WARN': '[PATH-REJECT]' in container
      and 'LogManager::instance().warning(LogCategory::Calibration, message);' in container,
    'session reset is logged and clears pending plan ID':
      '[SESSION-RESET-OK]' in function_body('newCaptureSession', 'startCaptureProcessing')
      and 'm_plannedScans.clear();m_nextPlanId=1;m_nextPairId=1;' in function_body('newCaptureSession', 'startCaptureProcessing'),
    'center rejection is logged': '[CENTER-REJECT]' in function_body('capturePlanCenter', 'addPlanRow'),
    'reference results and path table clearly distinguished':
      '已完成扫描的球心拟合结果' in widget and '可执行路径队列：' in widget,
    'RAW archive described as .scan not planned route':
      'RAW归档=完成一次硬件扫描后保存的原始Profile' in widget,
    'no BASIC protocol or H2 operation changed':
      (root / 'controller/ECI3808_V32_16_H1_H3_ONLY.bas').exists(),
}
for name, ok in checks.items():
    print(('PASS' if ok else 'FAIL') + ' | ' + name)
if not all(checks.values()):
    raise SystemExit(1)
print(f'PASS | {len(checks)}/{len(checks)} portable UI/source checks')
