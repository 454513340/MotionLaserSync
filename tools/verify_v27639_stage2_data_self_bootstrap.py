#!/usr/bin/env python3
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
widget = (root / 'src/widgets/SingleSphere14CalibrationWidget.cpp').read_text(encoding='utf-8')
core_h = (root / 'include/calibration/SingleSphere17Calibration.h').read_text(encoding='utf-8')
core_cpp = (root / 'src/calibration/SingleSphere17Calibration.cpp').read_text(encoding='utf-8')
cli = (root / 'tools/single_sphere17_cli.cpp').read_text(encoding='utf-8')
tests = (root / 'tests/v20_2_single_sphere17_tests.cpp').read_text(encoding='utf-8')


def section(text: str, start_sig: str, end_sig=None) -> str:
    start = text.find(start_sig)
    if start < 0:
        return ''
    if end_sig:
        end = text.find(end_sig, start + len(start_sig))
        if end < 0:
            end = len(text)
    else:
        end = len(text)
    return text[start:end]


checks = []
def ck(name: str, ok: bool):
    checks.append((name, bool(ok)))
    print(('PASS ' if ok else 'FAIL ') + name)

solve = section(widget, 'void SingleSphere14CalibrationWidget::onSolve()',
                'void SingleSphere14CalibrationWidget::onCancelStage2Solve()')
readiness = section(widget, 'void SingleSphere14CalibrationWidget::updateCalibrationReadiness()',
                    'void SingleSphere14CalibrationWidget::updateWorkflowGuide()')
import_csv = section(widget, 'void SingleSphere14CalibrationWidget::onImportSessionCsv()',
                     'void SingleSphere14CalibrationWidget::markPlanStatus(')

ck('Core exposes Stage2-data nominal hand-eye bootstrap API',
   'NominalHandEyeSphereResult solveNominalHandEyeBootstrap() const;' in core_h and
   'SingleSphere17Calibrator::solveNominalHandEyeBootstrap() const' in core_cpp)
ck('Stage2 final solve no longer hard-requires historical Stage1',
   'if (!m_hasBootstrap || !m_bootstrapResult.success)' not in solve)
ck('Stage2 final solve bootstraps from current representative training observations',
   ('task.bootstrap = bootstrapCalibrator.solveNominalHandEyeBootstrap();' in solve or
    'task.bootstrap = solveBootstrap(bootstrapData);' in solve) and
   'auto bootstrapObservations = reducedStage2Observations(1, 180);' in solve)
ck('Physical17 seed is the data-driven bootstrap hand-eye and sphere center',
   (('const auto initial = task.bootstrap.parameters;' in solve and
     'const Vec3d initialCenter = task.bootstrap.sphereCenterMachineMm;' in solve and
     'reducedCalibrator.solve(initial, &initialCenter' in solve) or
    ('solveRepresentative(' in solve and 'bootstrap.parameters' in solve and
     '&bootstrap.sphereCenterMachineMm' in solve)))
ck('Full-data refinement continues from representative Physical17 result',
   'fullCalibrator.solve(result.parameters, &result.sphereCenterMachineMm' in solve)
ck('Physical17 gauge-fixed observable state excludes hand-eye translation',
   'Physical17Parameters::HandEyeTx' not in section(core_cpp, 'std::vector<int> gaugeFixedObservableAndCenterIndices()', 'std::vector<int> fullIndices()'))
ck('Deterministic Physical17 starts do not perturb hand-eye translation',
   'i >= Physical17Parameters::HandEyeTx' in core_cpp and
   'i <= Physical17Parameters::HandEyeTz' in core_cpp and
   'amplitude = 0.0; // exact translation gauge anchor' in core_cpp)
ck('Final result stores Stage2 bootstrap provenance',
   'result.hasInitializationBootstrap = true;' in solve and
   'stage2_training_data_nominal_machine_handeye' in solve and
   'initialization_bootstrap' in core_cpp)
ck('Direct Stage2-only CSV can unlock final solve without Stage1 JSON',
   'canSelfBootstrapSolve' in readiness and
   '不需要另行加载阶段1 JSON' in readiness)
ck('Legacy mixed CSV limitation remains explicit',
   '旧V27.6.10A阶段1+阶段2混合CSV' in import_csv and
   '可靠剔除阶段1 profiles' in import_csv)
ck('CLI defaults to data-driven bootstrap',
   'bool autoHandEyeBootstrap = true;' in cli and
   'bootstrap = calibrator.solveNominalHandEyeBootstrap();' in cli and
   'initial = bootstrap.parameters;' in cli)
ck('CLI rejects ambiguous explicit auto-bootstrap plus manual seed',
   'Manual seed options cannot be combined with --auto-handeye-bootstrap' in cli)
ck('Numerical regression covers fixed bootstrap translation anchor',
   'testStage2DataSelfBootstrapAnchorsTranslation' in tests and
   'self-bootstrap Tx must remain the exact gauge anchor' in tests and
   'initialization_bootstrap' in tests)

failed = [name for name, ok in checks if not ok]
print(f'\n{len(checks) - len(failed)}/{len(checks)} checks passed')
if failed:
    sys.exit(1)
