#!/usr/bin/env python3
"""Package separately acquired XYZ candidates without changing measured bytes.

Usage: python tools/make_xyz_compensation_bundle.py --source C:/calibration --output C:/xyz_bundle
The source may be the flattened contents of calibration.rar, or X/Y/Z subfolders.
Output must not exist; no approval is inferred and runtime remains OFF.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile


def locate(root: Path, axis: str, kind: str) -> Path:
    if kind == "manifest":
        choices = [root/f"{axis}_axis_compensation_manifest.json",
                   root/axis/"axis_compensation_manifest.json"]
    else:
        name = f"{axis}_compensation_candidate.csv" if kind == "table" else f"{axis}_raw_axis_compensation_samples.csv"
        choices = [root/name, root/axis/name]
    found = [file for file in choices if file.is_file()]
    if len(found) != 1:
        raise ValueError(f"{axis} {kind}: expected exactly one source file, found {len(found)}")
    return found[0]


def validate(table: Path, manifest: Path, axis: str) -> None:
    meta = json.loads(manifest.read_text(encoding="utf-8-sig"))
    if meta.get("format") != "MotionLaserSync.AxisPositionCompensationCandidate" or meta.get("mapping") != "desired_mpos_mm -> required_dpos_mm":
        raise ValueError(f"{axis}: unsupported manifest or mapping direction")
    strategy = (meta.get("motion_strategy", "")+" "+meta.get("sampling_strategy", "")).lower()
    if "monotonic" not in strategy and "direction-prime-once" not in strategy and meta.get("version") != 1:
        raise ValueError(f"{axis}: unknown acquisition strategy; do not guess")
    branches: dict[str,list[tuple[float,float]]] = {"+":[], "-":[]}
    with table.open(encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            if row["axis"] != axis or row["direction"] not in branches:
                raise ValueError(f"{axis}: mixed axis/direction")
            x, y = float(row["desired_mpos_mm"]),float(row["required_dpos_mm"])
            if not math.isfinite(x) or not math.isfinite(y) or abs(y-x)>0.1:
                raise ValueError(f"{axis}: nonfinite coordinate or correction over 0.1 mm")
            correction = row.get("correction_required_dpos_minus_desired_mpos_mm")
            if correction is not None and (not math.isfinite(float(correction)) or abs(y-x-float(correction))>5e-8):
                raise ValueError(f"{axis}: inconsistent correction column")
            branches[row["direction"]].append((x,y))
    for direction, name in (("+","positive"),("-","negative")):
        values=sorted(branches[direction])
        if not 2<=len(values)<=100000 or len(values)!=meta["axes"][axis][name]["node_count"]:
            raise ValueError(f"{axis}{direction}: invalid/inconsistent node count")
        if any(x[0]>=y[0] or x[1]>=y[1] for x,y in zip(values, values[1:])):
            raise ValueError(f"{axis}{direction}: mapping must be strictly monotonic")
    if not 0<=meta.get("settle_ms",-1)<=60000 or meta.get("direction_preload_mm",0)<=0:
        raise ValueError(f"{axis}: invalid settle/preload metadata")


def package(source: Path, output: Path) -> None:
    source=source.resolve();output=output.resolve()
    if output.exists():
        raise ValueError("Output already exists; use a new directory (no silent replacement).")
    files={a:{kind:locate(source,a,kind) for kind in ("table","manifest","raw")} for a in "XYZ"}
    for a in "XYZ":
        validate(files[a]["table"],files[a]["manifest"],a)
    output.parent.mkdir(parents=True,exist_ok=True)
    stage=Path(tempfile.mkdtemp(prefix=".xyz_bundle_",dir=str(output.parent)))
    try:
        bundle={"format":"MotionLaserSync.XYZRuntimeCompensationBundle", "version":1,
                "mapping":"desired_mpos_mm -> required_dpos_mm", "default_enabled":False,
                "source_package":str(source),
                "validation_evidence":"Packaging validates format/integrity only. Per-axis validation records and production-condition approval remain separate.",
                "axes":{}}
        for axis in "XYZ":
            dest=stage/axis;dest.mkdir();entry={}
            for kind,src in files[axis].items():
                name="axis_compensation_manifest.json" if kind=="manifest" else src.name
                file=dest/name;shutil.copy2(src,file)
                if kind in ("table","manifest"):
                    entry[kind]=f"{axis}/{name}"
                    entry[kind+"_sha256"]=hashlib.sha256(file.read_bytes()).hexdigest()
            bundle["axes"][axis]=entry
        (stage/"bundle.json").write_text(json.dumps(bundle,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
        stage.rename(output)
    finally:
        if stage.exists():shutil.rmtree(stage)
    print(f"Created {output / 'bundle.json'}; original bytes/status preserved; runtime OFF.")


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",required=True,type=Path)
    parser.add_argument("--output",required=True,type=Path)
    args=parser.parse_args()
    try:
        package(args.source,args.output)
    except (OSError,ValueError,KeyError,csv.Error) as error:
        parser.exit(1,f"Bundle creation failed: {error}\n")

if __name__=="__main__":main()
