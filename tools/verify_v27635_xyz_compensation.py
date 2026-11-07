#!/usr/bin/env python3
"""V27.6.35 source/data integration contract. Not a compiler or hardware test."""
from pathlib import Path
import csv
import hashlib
import json
import re
import sys

ROOT = Path(__file__).resolve().parents[1]

def text(name: str) -> str:
    return (ROOT / name).read_text(encoding="utf-8")

def function_body(source: str, name: str) -> str:
    """Source-contract window up to the next out-of-class method definition."""
    match = re.search(r"^[^\n;]*\b" + re.escape(name) + r"\s*\(", source, re.M)
    if not match:
        raise AssertionError(f"Missing function: {name}")
    following = re.search(r"^\w[^\n;]*\b\w+::\w+\s*\(", source[match.end():], re.M)
    return source[match.start():match.end() + following.start()] if following else source[match.start():]

def main() -> int:
    checks: list[tuple[str, bool]] = []
    def check(name: str, condition: bool) -> None:
        checks.append((name, bool(condition)))
    app = text("src/app/MainWindow.cpp")
    core = text("src/core/ZmcController.cpp")
    adapter = text("src/core/ZmcAxisCompensation.cpp")
    engine = text("src/runtime/AxisCompensation.cpp")
    ui = text("src/app/MainWindowAxisCompensation.cpp")
    gcode = text("src/gcode/GCodeExecutor.cpp")
    header = text("include/core/ZmcController.h")
    check("new GUI release title", "MotionLaserSync V27.6.35" in app)
    check("default runtime OFF", "m_axisCompEnabled{false}" in header)
    check("independent runtime UI panel connected", "createRuntimeAxisCompensationPanel(widget)" in app)
    check("bundle hashes checked before install", "QCryptographicHash::Sha256" in ui and "manifest_sha256" in ui)
    check("complete Stage2 workflow switch exclusion", "runtimeCompensationChangeBlocked()" in ui)
    check("no feedback or coordinate-reset writes in compensation adapter",
          not re.search(r"ZAux_Direct_Set(?:Mpos|Dpos)\s*\(", adapter + engine))
    check("finite step jog uses physical relative entry", "return moveRel(" in function_body(core,"ZmcController::jog"))
    check("relative target starts at actual MPOS",
          "physicalTarget=(compensate?double(m):double(d))+double(delta)" in core)
    check("raw command-space APIs exported", all(x in header for x in ("moveAbsRawDpos", "moveRelRawDpos")))
    for name in ("startAutomaticAxisMoveDiagnostic", "startAxisCompensationSpeedQualification", "startAxisCompensationCalibration", "startAxisCompensationValidation"):
        body = function_body(app, "MainWindow::" + name)
        check(name + " rejects enabled runtime and uses explicit RAW",
              "axisCompensationEnabled()" in body and "moveAbsRawDpos(" in body
              and not re.search(r"controller->move(?:Abs|Rel)\s*\(", body))
    check("A/B uses same pure interpolation", "mls::axiscomp::interpolate(" in function_body(app,"MainWindow::interpolateAxisCompensationCommand"))
    check("separate XY/Z validation strategies retained", all(x in app for x in ("AxisCompValMonotonicSweep", "AxisCompValPointToPoint", "resolvedAxisCompensationValidationMethod")))
    check("flattened per-axis manifest supported", '"_axis_compensation_manifest.json"' in app)
    check("trajectory transform occurs in prepare, never twice in pump",
          "mls::axiscomp::mapPath(" in function_body(core,"ZmcController::prepareMergePath")
          and "mls::axiscomp::mapPath(" not in function_body(core,"ZmcController::pumpMergeBuffer"))
    check("RTCP explicit measured start", "ZmcController::PathStartReference::MeasuredMpos" in gcode)
    check("ordinary G-code nominal start adapter", "m_zmc->readMotionStartPose5D(" in gcode)
    check("all legacy arc entries route compensated arcs", sum("compensatedArc(" in function_body(core,"ZmcController::"+n) for n in ("moveArc2Abs","moveArc2Rel","moveArcCenterAbs")) == 3)
    check("no extrapolation / silent raw fallback in pure engine",
          "extrapolation is forbidden" in engine and "branch change would reverse" in engine)
    # Motion SDK routing audit: changing this whitelist requires a new review.
    sdk = re.compile(r"\bZAux_Direct_(?:Single_MoveAbs|Single_Move|Single_Vmove|MoveAbsSp|MoveAbs|MoveCirc2Abs|MoveCirc2|SetMpos|SetDpos)\s*\(")
    unexpected = []
    for file in (ROOT / "src").rglob("*.cpp"):
        if sdk.search(file.read_text(encoding="utf-8")) and file.relative_to(ROOT).as_posix() != "src/core/ZmcController.cpp":
            unexpected.append(str(file.relative_to(ROOT)))
    check("all SDK positioning remains in reviewed controller wrapper", not unexpected)

    bundle_root = ROOT / "calibration/runtime_xyz"
    bundle = json.loads((bundle_root / "bundle.json").read_text())
    check("bundle default OFF", bundle["default_enabled"] is False)
    total = 0
    for axis, expected in zip("XYZ", (519,327,173)):
        entry = bundle["axes"][axis]
        for kind in ("table", "manifest"):
            file = bundle_root / entry[kind]
            check(axis+" "+kind+" exact SHA256", hashlib.sha256(file.read_bytes()).hexdigest() == entry[kind+"_sha256"])
        meta = json.loads((bundle_root / entry["manifest"]).read_text())
        check(axis+" source approval status preserved", meta["status"] == "CANDIDATE_UNVALIDATED" and not meta["runtime_compensation_enabled"])
        check(axis+" minimum acquisition settle metadata", meta["settle_ms"] == 2000)
        rows = list(csv.DictReader((bundle_root / entry["table"]).open(encoding="utf-8-sig")))
        total += len(rows)
        for direction in "+-":
            branch = [x for x in rows if x["direction"] == direction]
            check(axis+direction+" node count", len(branch) == expected)
        check(axis+" axis identity", all(x["axis"] == axis for x in rows))
        method = meta.get("motion_strategy", "")
        check(axis+" correct acquisition family", "monotonic" in method if axis != "Z" else meta["version"] == 1 and "monotonic" not in method)
    check("2038 supplied nodes", total == 2038)
    for name, passed in checks:
        print(("PASS " if passed else "FAIL ")+name)
    print(f"{sum(p for _,p in checks)}/{len(checks)} source/data checks passed")
    return int(not all(p for _,p in checks))

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, AssertionError) as exc:
        print(f"FAIL source/data contract: {exc}", file=sys.stderr)
        raise SystemExit(1)
