# V20 changes: integrated single-sphere 14-D calibration

Base: original uploaded `5Axis_project_v19`.

## Added calibration core

- `include/calibration/SingleSphere14Calibration.h`
- `include/calibration/SingleSphere14QtAdapter.h`
- `src/calibration/SingleSphere14Calibration.cpp`
- `tools/single_sphere14_cli.cpp`
- `tools/simulate_single_sphere14.py`
- `tests/v20_single_sphere14_tests.cpp`

The numerical model estimates the 14 independent parameters observable from one
unknown fixed 19.05 mm sphere with an eye-in-hand line laser. It explicitly does
not claim a unique physical 17-parameter PIGE/hand-eye decomposition.

## Integrated into the original GUI

- Added `SingleSphere14CalibrationWidget` as a fourth calibration workflow tab.
- Preview requests reuse V19 `MainWindow::onStartProfilePreview/onStopProfilePreview`.
- New static capture listens to the real `LaserSensor::profileReceived` signal.
- Capture reads `JointState5D` before and after fresh profiles and rejects motion drift.
- Pose moves are routed through V19 `JointTrajectoryPlanner5Axis` and
  `ZmcController::moveJointAbs5D`, including the existing B/C absolute-ready,
  enable and controller soft-limit checks.
- Added ROI sphere fitting, fitted-circle overlay, reference teaching,
  D-optimal pose generation, bounded XYZ recentering, observability display,
  14-D solve, JSON save/load and observation CSV import/export.

## Modified original files

- `CMakeLists.txt`
- `include/widgets/CalibrationWidget.h`
- `src/widgets/CalibrationWidget.cpp`
- `include/widgets/Profile2DWidget.h`
- `src/widgets/Profile2DWidget.cpp`
- `src/app/MainWindow.cpp`

## Safety / scope

- Machine pose execution remains operator-confirmed. No collision, fixture or
  cable-twist model has been added.
- The calibration workflow is static move-stop-capture; it does not use the old
  V19 XYZ-only TABLE synchronization protocol.
- The V19 dynamic scan point-cloud path is still XYZ-only and does not yet apply
  the 14-D model to moving B/C scans.
