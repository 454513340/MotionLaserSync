#!/usr/bin/env python3
"""Source-contract verifier for MotionLaserSync V27.6.43.

This intentionally does not replace the Windows x64 GUI build. It checks that
all exposure-analysis/material-library integration pieces expected by the
release are present and that the bundled Natural_Sunny seed is internally
consistent.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def require_file(rel: str) -> Path:
    p = ROOT / rel
    if not p.is_file():
        raise AssertionError(f"missing release file: {rel}")
    return p


def require_text(rel: str, *needles: str) -> None:
    text = require_file(rel).read_text(encoding="utf-8")
    for needle in needles:
        if needle not in text:
            raise AssertionError(f"{rel}: missing required text: {needle}")


def main() -> int:
    required_files = [
        "include/calibration/ExposureAnalysisEngine.h",
        "src/calibration/ExposureAnalysisEngine.cpp",
        "include/calibration/ExposureTimingPolicy.h",
        "src/calibration/ExposureTimingPolicy.cpp",
        "include/calibration/MaterialExposureLibrary.h",
        "src/calibration/MaterialExposureLibrary.cpp",
        "tools/exposure_analysis_cli.cpp",
        "config/material_exposure_defaults.json",
        "README_V27_6_43_EXPOSURE_ANALYSIS_ZH.md",
        "VALIDATION_V27_6_43.txt",
    ]
    for rel in required_files:
        require_file(rel)

    require_text(
        "CMakeLists.txt",
        "EXPOSURE_ANALYSIS_CLI_ONLY",
        "src/calibration/ExposureAnalysisEngine.cpp",
        "src/calibration/ExposureTimingPolicy.cpp",
        "src/calibration/MaterialExposureLibrary.cpp",
    )
    require_text("src/app/main.cpp", 'setApplicationVersion("2.0.0-v27.6.43")')
    require_text(
        "src/app/MainWindow.cpp",
        "MotionLaserSync V27.6.43",
        "runExposureAnalysisAndImport",
        "exposure_analysis.csv",
        "exposure_analysis.json",
        "m_btnAnalyzeExpTestDir",
        "m_comboMaterialExposureProfile",
        "V27.6.43-center-segment-knee-v1",
    )
    require_text(
        "src/calibration/ExposureAnalysisEngine.cpp",
        "continuous valid surface segment",
        "saturationPlateauFractionThreshold",
        'V27.6.43-center-segment-knee-v1',
    )
    require_text(
        "src/calibration/ExposureTimingPolicy.cpp",
        "triggerPeriod10us - 1",
        "recommendedExposure10us >= triggerPeriod10us - 1",
    )
    require_text(
        "src/calibration/MaterialExposureLibrary.cpp",
        "QStandardPaths::AppConfigLocation",
        "QSaveFile",
        "sourcePoseXYZBC",
        "referenceProfileRateHz",
    )

    seed = json.loads(require_file("config/material_exposure_defaults.json").read_text(encoding="utf-8"))
    assert seed["schema"] == "MotionLaserSync.MaterialExposureLibrary.v1"
    assert seed["applicationVersion"] == "V27.6.43"
    assert len(seed["entries"]) >= 1
    alumina = seed["entries"][0]
    assert alumina["materialFileSafe"] == "Alumina_ceramic_ball_1"
    assert alumina["lightingCode"] == "Natural_Sunny"
    assert alumina["recommendedExposure_10us"] == 30
    assert alumina["referenceIdle_10us"] == 3970
    assert alumina["referenceProfileRateHz"] == 25
    assert alumina["saturationOnset_10us"] == 35
    assert alumina["safeCandidateBand_10us"] == [25, 30]
    assert abs(alumina["inferredSaturationCeiling"] - 875.5) < 1e-9
    assert alumina["sourcePoseXYZBC"]["valid"] is True

    evidence = json.loads(
        require_file(
            "validation/v27_6_43_exposure_analysis/Natural_Sunny_Alumina_exposure_analysis.json"
        ).read_text(encoding="utf-8")
    )
    assert evidence["profileFilesAnalyzed"] == 130
    assert evidence["recommendedExposure_10us"] == 30
    assert evidence["safeCandidateBand_10us"] == [25, 30]
    assert evidence["saturationOnset_10us"] == 35
    assert evidence["confidence"] == "high"

    print("V27.6.43 source-contract verification: PASS")
    print("Bundled Natural_Sunny recommendation: Exp=30, ref Idle=3970 @25Hz")
    print("Hardware 5ms policy expected: Exp=30, Idle=469 (499 < 500 units)")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, KeyError, json.JSONDecodeError) as exc:
        print(f"V27.6.43 source-contract verification: FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
