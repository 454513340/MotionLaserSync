# V20 integrated single-sphere calibration test results

## Portable build/tests

The integrated tree was configured with `-DV19_TESTS_ONLY=ON` and all tests passed:

- `p0_portable_unit` PASS
- `p0_real_core_mock_transport` PASS
- `v19_scheduler` PASS
- `v20_single_sphere14` PASS
- `v19_sync_report` PASS
- `v19_source_contract` PASS
- `v20_gui_source_contract` PASS

Result: **7/7 PASS**.

The V20 numerical test covers sphere-section fitting, D-optimal planning,
14/14 observability and synthetic recovery.

## V19 source/package contract

`python tools/verify_v19.py --structure-only`:

**13/13 PASS**.

This confirms that the protected V19 BASIC/TABLE protocol, audited ZAux boundary,
scheduler/report sources and original stop-entry contracts were not accidentally
changed by the calibration integration.

## V20 GUI integration source contract

`python tools/verify_v20_single_sphere_gui_integration.py`:

**12/12 PASS**.

Checks include:

- original CalibrationWidget contains the new tab;
- real `JointState5D` is read for static capture;
- capture collects fresh profiles;
- sphere section fitting is called;
- 14-D observability is checked;
- bounded XYZ recentering is wired;
- preview is routed through the original MainWindow preview path;
- movement is routed through the original five-axis planner and controller move;
- the new GUI source is in the normal application target.

## Full GUI build limitation of this environment

A normal GUI CMake configure was attempted. It stops at `find_package(Qt6 ...)`
because this Linux execution environment does not contain Qt6. Therefore the
full Windows target with Qt6 + VTK + ZMotion SDK + Micro-Epsilon LLT SDK could
not be linked here.

The intended final compile check must be run in the original Windows development
environment. No claim is made that hardware execution was tested in this container.

## V20.1 RANSAC / full-field regression

- Portable `v20_single_sphere14_tests`: PASS after replacing direct ROI circle fit with deterministic RANSAC + geometric refinement.
- Added synthetic regression containing a correct sphere arc, a long fixture/background line and isolated boundary returns. RANSAC recovers the sphere center and exports only sphere-arc inliers.
- GUI/source integration contract: 18/18 PASS, including RANSAC core, RANSAC-only observations, calibration-specific preview entry, temporary full measuring field, and software/internal trigger path.
- Single-sphere calibration preview now forces `Large / ROI1_PRESET=0` without saving to user mode.
