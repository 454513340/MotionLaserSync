#!/usr/bin/env python3
"""Inspect the imported-source identities embedded in a V29 MultiPose NC.

This tool reads files only. It cannot certify calibration, registration,
controller hardware, physical collision clearance, or release a machine job.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import struct
from pathlib import Path

PHYS_PREFIX = b"MLS-PHYSICAL17-IEEE754-BE-V1"
POSE_PREFIX = b"MLS-PART-POSE-CANDIDATE-V1"


def normalized_doubles(values):
    return b"".join(struct.pack(">d", 0.0 if float(x) == 0.0 else float(x))
                    for x in values)


def scan_headers(text: str):
    entries = {}
    for line in text.splitlines():
        text = line.strip()
        if not text.startswith(";"):
            continue
        value = text[1:].strip().partition(":")
        if value[1]:
            entries[value[0].strip().lower()] = value[2].strip()
    return entries


def sha256_file(path: Path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for data in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(data)
    return digest.hexdigest()


def audit(nc_path: Path, manifest_path: Path):
    nc_path = nc_path.resolve()
    manifest_path = manifest_path.resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    nc_bytes = nc_path.read_bytes()
    headers = scan_headers(nc_bytes.decode("utf-8-sig"))
    checks, errors = {}, []

    def checked(name, ok, reason):
        checks[name] = bool(ok)
        if not ok:
            errors.append(reason)

    for manifest_key, nc_key in (("partPoseId", "partposeid"),
                                  ("calibrationId", "calibrationid"),
                                  ("physical17Sha256", "physical17sha256")):
        checked(manifest_key, bool(manifest.get(manifest_key)) and
                manifest.get(manifest_key) == headers.get(nc_key),
                f"NC {nc_key} 不匹配清单中的 {manifest_key}")

    try:
        values = manifest["importedPhysical17Parameters"]
        assert len(values) == 17 and all(math.isfinite(float(v)) for v in values)
        digest = hashlib.sha256(PHYS_PREFIX + normalized_doubles(values)).hexdigest()
        checked("physical17NumericHash", digest == manifest.get("physical17Sha256"),
                "清单里的Physical17数值与其SHA256不匹配")
        checked("calibrationIdDerived", "CAL17_IMPORTED_SHA256_" + digest ==
                manifest.get("calibrationId"), "CalibrationId不是导入的17参数数值身份")
    except (KeyError, AssertionError, ValueError, TypeError, OverflowError):
        checked("physical17NumericHash", False, "清单缺少完整有效的17参数快照")

    try:
        matrix = manifest["cadToMachine"]
        cad_digest = manifest["cadSha256"]
        assert len(matrix) == 16 and len(cad_digest) == 64
        digest = hashlib.sha256(POSE_PREFIX + cad_digest.encode("ascii") +
                                normalized_doubles(matrix)).hexdigest()
        checked("poseIdDerived", "POSE_CANDIDATE_SHA256_"+digest ==
                manifest.get("partPoseId"), "PartPoseId与STEP摘要/CAD位姿矩阵不符")
    except (KeyError, AssertionError, ValueError, TypeError, OverflowError):
        checked("poseIdDerived", False, "清单缺少有效STEP摘要或16元素位姿矩阵")

    expected_nc = set()
    for item in manifest.get("poses", []):
        expected_nc.update((f.get("name"), f.get("sha256"))
                           for f in item.get("ncFiles", []))
    for item in manifest.get("continuousNcSessions", []):
        expected_nc.add((item.get("filename"), item.get("sha256")))
    actual_digest = hashlib.sha256(nc_bytes).hexdigest()
    checked("ncFileHash", (nc_path.name, actual_digest) in expected_nc,
            "当前NC文件未列在清单中，或内容SHA256与清单不符")

    for source_key, source_hash_key in (("sourceCad", "cadSha256"),
                                         ("sourceScan", "realScanSha256")):
        source = Path(manifest.get(source_key, ""))
        # Old manifest paths may originate on another Windows machine.
        if source.is_file():
            checked(f"{source_key}FileHash", sha256_file(source) == manifest.get(source_hash_key),
                    f"本机{source_key}文件与清单摘要不一致")
        else:
            checks[f"{source_key}FileHash"] = None

    # Invariant independent of hashes: no generated candidate becomes a permit.
    checks["independentSiteAcceptance"] = False
    return {
        "schema": "MLS_MULTIPOSE_PROVENANCE_AUDIT_V1",
        "ncPath": str(nc_path),
        "ncSha256": actual_digest,
        "partPoseId": headers.get("partposeid", ""),
        "calibrationId": headers.get("calibrationid", ""),
        "checks": checks,
        "errors": errors,
        "importedSourceIdentityMatches": not errors,
        "safeForHardwareExecution": False,
        "notes": "Source identity checks are not independent registration, FCL, synchronization, or live machine approval.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nc", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit(args.nc, args.manifest)
    message = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(message + "\n", encoding="utf-8")
    print(message)
    return 0 if report["importedSourceIdentityMatches"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
