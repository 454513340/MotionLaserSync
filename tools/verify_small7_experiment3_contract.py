#!/usr/bin/env python3
"""Portable structural regression; not a Qt/vendor SDK or hardware test."""
from pathlib import Path
import re
ROOT=Path(__file__).resolve().parents[1]
def src(path):
    p=ROOT/path
    assert p.exists(), f"missing: {path}"
    return p.read_text(encoding="utf-8")
app=src('src/app/MainWindowSmall7Experiment.cpp')
header=src('include/app/MainWindow.h')
portable=src('src/calibration/Small7HeightPlan.cpp')
controller=src('src/core/ZmcController.cpp')
exposure=src('src/app/MainWindowSphereExposure.cpp')
window=src('src/app/MainWindow.cpp')
cmake=src('CMakeLists.txt')
checks={
    'New experiment module linked': 'src/app/MainWindowSmall7Experiment.cpp' in cmake,
    'Portable planner linked to full Windows GUI': 'set(ALGORITHM_SOURCES\n    src/algorithms/CalibrationAlgorithms.cpp\n    src/calibration/SingleSphere14Calibration.cpp\n    src/calibration/SingleSphere17Calibration.cpp\n    src/calibration/Small7HeightPlan.cpp' in cmake,
    'Windows UI includes experiment 3 panel': 'layout->addWidget(createSmall7ExperimentPanel' in window,
    'Dedicated machine Z inputs': all(x in app for x in ['m_spinSmall7ZStart','m_spinSmall7ZEnd','m_spinSmall7ZInterval']),
    'No editable sensor Z lower or upper in UI': not re.search(r'Z(下限|上限)\s*[,)]',app),
    'Verified per-field ROI file required': 'readOptics(' in app and 'verified' in app,
    'Selected field must match verified optical field': 'w.measuringFieldIndex != s.measuringFieldIndex' in portable,
    'Runtime readback checks selected field': 'liveField!=params.measuringFieldIndex' in app,
    'Runtime field/exposure/idle readback': 'readTimingAndMeasuringField' in app,
    'Experiment3 GUI has field selector': 'm_comboSmall7Field=new QComboBox' in app,
    'Large, Standard, Small and additional indexes': all(x in app for x in ['{0,2,7}','field<128','s7::referenceMaxProfileHz(field)']),
    'Per-field optical JSON mismatch is rejected': 'm_small7Optics.measuringFieldIndex!=selectedField' in app,
    'Per-field profile-rate maximum updates': 'm_spinSmall7Rate->setMaximum(maximum)' in app,
    'Per-field reference bound validated in portable core': 'referenceMaxProfileHz(s.measuringFieldIndex)' in portable,
    'Live rate verified against chosen field': 'LaserSensor::getMaxFrequencyForIndex(m_small7Settings.measuringFieldIndex)' in app,
    'Session persists actual selected field': 'session["measuringFieldIndex"]=m_small7Settings.measuringFieldIndex' in app,
    'Capture config uses selected field': 'params.measuringFieldIndex=m_small7Settings.measuringFieldIndex' in app,
    'Capture records actual field': 'summary["measuringFieldIndex"]=m_small7Settings.measuringFieldIndex' in app,
    'Hardware acquisition UI attaches read only': 'm_profilePreviewMirrorsCapture = true' in window,
    'Hardware mirror does not reconfigure sensor': 'if (m_syncManager->isRunning()) {' in window and window.index('m_profilePreviewMirrorsCapture = true') < window.index('SensorTriggerMode::SoftwareInternal, params', window.index('void MainWindow::onStartProfilePreview')), 
    'Hardware mirror does not stop formal stream': window.index('if (m_profilePreviewMirrorsCapture) {', window.index('void MainWindow::onStopProfilePreview')) < window.index('m_laser->stopContinuous();', window.index('void MainWindow::onStopProfilePreview')),
    'Hardware preview UI enabled during running capture': 'const bool canMirror=' in window,
    'Hardware mirror requires fresh callback': 'profile.callbackOrdinal!=m_profilePreviewMirrorBaselineOrdinal' in window,
    'No trigger fallback while waiting for hardware frames': '不会自动改触发设置' in window,
    'Independent preview labeled as software clock': '非硬件同步测量' in window,
    'Stopping capture detaches mirror': 'if(m_profilePreviewMirrorsCapture)onStopProfilePreview();' in window,

    'Flip configuration matches calibrated model': 'm_sphereStudyPlanningFlipX' in app and 'm_sphereStudyPlanningFlipZ' in app,
    'Sensor flip read back before execute': 'readAdvancedParams(adv' in app,
    'Physical17 used for each Z': 'machinePointInSensor(' in portable,
    'MachineSafety transition preflight': 'checkSphereStudyTransitionSafety' in app,
    'Controller and XYZ homing checked': 'areLinearMposReferenced()' in app,
    'Non-X/Y/C/B drift gated': 'fixedYCB(' in app,
    'Position speed 1..5 selectable': 'for(int speed=1;speed<=5;++speed)' in app,
    'Scan speed derived from configured profile spacing and quantized rate':
        'calculateScanTiming(conf.profileRateHz,conf.requestedLineSpacingMm' in app and
        'conf.scanSpeedMmS=timing.scanSpeedMmS' in app and
        'm_small7Settings.scanSpeedMmS' in app and
        'desiredLineSpacingMm*out.effectiveProfileHz' in portable,
    'Frame spacing has quantized profile timing': 'std::ceil(100000.0/requestedHz)' in portable,
    'Independent experiment3 exposure start/end/step controls':
        all(x in app for x in ['m_spinSmall7ExpStart=new QSpinBox',
          'm_spinSmall7ExpEnd=new QSpinBox','m_spinSmall7ExpStep=new QSpinBox',
          'expStart=m_spinSmall7ExpStart->value()',
          'expEnd=m_spinSmall7ExpEnd->value()',
          'expStep=m_spinSmall7ExpStep->value()']),
    'Experiment3 exposes profile count, Video count and output directory':
        all(x in app for x in ['m_spinSmall7ExposureFrames=new QSpinBox',
          'm_spinSmall7VideoFrames=new QSpinBox',
          'm_checkSmall7SaveVideo=new QCheckBox',
          'm_editSmall7OutputDir=new QLineEdit',
          'm_spinSmall7StaticDiscard=new QSpinBox']),
    'Conservative theoretical arc exists at every Z, not an impossible full 360 circle':
        'maxCircleFitArcInsideOptics(peak.x,peak.z,radius,w)' in portable and
        'findOpticalZIntervals' in app and 'entireZAllowed' in app,
    'Scanned-X peak actually fitted from measured profile radii':
        'm_small7ScanProfiles.push_back(' in app and
        'findScannedMaximumCircle(labelled' in app and
        'm_small7ProbePositions[0]' in app,
    'Scan position interpolation requires bracketing controller MPOS telemetry':
        'm_small7AxisSamples.push_back(' in app and 'width>400000' in app and
        'GlobalTimeSync::instance().timestampUs()' in app,
    'Scan inter-profile callback spacing checked against maximum':
        'm_small7MaxObservedFrameSpacingMm>m_small7Settings.maximumFrameSpacingMm' in app,
    'Frame spacing verified using all raw scan frames (including invalid fits)':
        'm_small7ScanAllFramePcUs.push_back(profile.tPcUs)' in app and
        'for(const auto pcUs:m_small7ScanAllFramePcUs)' in app,
    'Seven static positions refine scanned maximum and verify actual diameter':
        'm_small7ChosenPeakX=m_small7ProbePositions' in app and
        'm_small7DiameterToleranceMm' in app,
    'Exposure sweep collects even invalid-fit frames and preserves index drop events':
        'm_small7ReceivedFrames>=m_small7RequiredFrames+m_small7StaticDiscardFrames' in app and
        'fitValidFractionAfterWarmup' in app and
        'if(m_small7Phase!=Small7Phase::ExposureCapture)' in app and
        'estimatedMissingProfileIndices' in app,
    'Optional sequential Video PNG capture after Profile group':
        'small7StartVideoGroup()' in app and 'configureVideo(params' in app and
        'frame.save(fileName,"PNG")' in app and 'small7StopVideo()' in app,
    'Scan warmup frames before X motion': 'Small7Phase::ScanWarmup' in app and 'm_small7ReceivedFrames>=4' in app,
    'Only motion-time scan frames count': 'm_small7ScanMotionGoodFrames<15' in app and 'm_small7WorkerActive' in app,
    'Stream-loss motion watchdog': 'small7SensorWatchdog' in app and 'liveScan?1500:3000' in app,
    '7 point static search': 'm_small7ProbeIndex<7' in app and 'sevenPointX' in portable,
    'Exposure sequence stationary and peak X fixed': 'Small7Phase::ExposureCapture' in app and '!state.allIdle' in app and 'std::abs(state.mpos.x-m_small7ChosenPeakX)>0.005' in app,
    'Per-group RAW preserved': '"S7P1"' in app and 'm_small7Raw' in app,
    'Frame continuity fail closed': 'm_small7FrameGap=true' in app,
    'Static sensor transient discarded': 'm_small7StaticDiscardFrames' in app,
    'Stop both X and Z axes': 'requestStopAxis(AxisX,false)' in app and 'requestStopAxis(AxisZ,false)' in app,
    'Controller disconnect deferred': 'if(m_small7Active){' in window,
    'Axis feedback progress': 'small7PositionHeartbeat' in app and 'readAxisUiPosition' in app,
    'Timeout uses movement distance': 'motionTimeoutMs(moveLengthMm' in controller,
    'Timeout requests stop': 'requestStopAxis(axis,false); // a timeout' in controller,
    'Compensated speed not silently lowered': '不允许无提示降速' in controller,
    'Legacy full-field positioning speed independent': 'm_comboSphereExpPositionSpeed' in exposure,
    'Old-new sweep interlock': 'm_sphereExpActive || m_small7Active' in exposure,
    'Full source includes safety guidance': (ROOT/'README_SMALL7_EXPERIMENT3_FIXED_ZH.md').exists(),
}
failed=[]
for title,passed in checks.items():
    print(('PASS' if passed else 'FAIL')+' '+title)
    if not passed:failed.append(title)
print(f'{len(checks)-len(failed)}/{len(checks)} static checks pass')
if failed:raise SystemExit(1)
