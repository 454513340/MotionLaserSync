#!/usr/bin/env bash
# Reproduce M3/M4 OFFLINE research calibration. No hardware commands.
# Usage: bash tools/run_2950_fourtrain_03holdout.sh <directory-of-five-original-SCANs> <output-directory>
set -euo pipefail
if (( $# != 2 )); then echo "Usage: $0 SCAN_DIR OUTPUT_DIR" >&2; exit 2; fi
SRC=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
RAW=$(cd "$1" && pwd)
OUT=$2
mkdir -p "$OUT" "$OUT/sample" "$OUT/full03" "$OUT/model"
OUT=$(cd "$OUT" && pwd)
STEP="$SRC/examples/digital_twin/6061_sensor_connector_NEW_2026.step"
OBJ="$SRC/examples/digital_twin/6061_sensor_connector_NEW_2026.obj"
SETTINGS="$SRC/examples/digital_twin/2950_50BL_acquisition_settings.json"
POSE=(6.875014828 8.951331293 -235.26297484 -0.025937784 0.228699434 -89.416079059)
TRAINS=();ALL=()
for id in 02 04 05 06 03; do
  f="$RAW/6061_Alumina_Scan_H1_New_${id}.scan"
  [[ -r "$f" ]] || { echo "Missing SCAN: $f" >&2; exit 3; }
  ALL+=("$f")
  [[ "$id" == 03 ]] || TRAINS+=("$OUT/sample/6061_Alumina_Scan_H1_New_${id}.m4s1")
done
HOLD="$OUT/sample/6061_Alumina_Scan_H1_New_03.m4s1"
cmake -S "$SRC" -B "$OUT/build" -DDIGITAL_TWIN_CLI_ONLY=ON -DTWIN_ENABLE_OCCT=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build "$OUT/build" -j4
ctest --test-dir "$OUT/build" --output-on-failure
python "$SRC/tools/m0m5_prepare_scans.py" --scan "${ALL[@]}" --step "$STEP" --output "$OUT/sample" --stride 8
LABEL_ARGS=()
for f in "${TRAINS[@]}" "$HOLD"; do LABEL_ARGS+=("$f"); done
python "$SRC/tools/m34_make_labels_bulk.py" --obj "$OBJ" --pose "${POSE[@]}" --m4 "${LABEL_ARGS[@]}"
python "$SRC/tools/m34_fan_intrinsics.py" --train "${TRAINS[@]}" --output "$OUT/model/fan_four_sessions_only.json"
python "$SRC/tools/m34_fit_v2.py" --exe "$OUT/build/twin_m34_v2_features_cli" --obj "$OBJ" \
  --pose "${POSE[@]}" --train "${TRAINS[@]}" --holdout "$HOLD" \
  --fan-json "$OUT/model/fan_four_sessions_only.json" --acquisition-config "$SETTINGS" \
  --output "$OUT/model" --refine-geometry
# All profile validation only after previous step finalized every parameter.
python "$SRC/tools/m0m5_prepare_scans.py" --scan "$RAW/6061_Alumina_Scan_H1_New_03.scan" \
  --step "$STEP" --output "$OUT/full03" --stride 8 --m4-profile-step 1
python "$SRC/tools/m34_make_labels_bulk.py" --obj "$OBJ" --pose "${POSE[@]}" \
  --m4 "$OUT/full03/6061_Alumina_Scan_H1_New_03.m4s1"
MODEL="$OUT/model/M34_V2_RESEARCH_NOT_CERTIFIED.m34cfg"
read -r CX CY CZ FS FO < <(python - "$MODEL" <<'PY'
import sys
lines=open(sys.argv[1]).read().splitlines();x,y,z=lines[1].split();s,o=lines[3].split();print(x,y,z,s,o)
PY
)
"$OUT/build/twin_m34_v2_features_cli" --obj "$OBJ" --pose "${POSE[@]}" \
 --camera-x "$CX" --camera-y "$CY" --camera-z "$CZ" --fan-scale "$FS" --fan-offset "$FO" \
 --source "$OUT/full03/6061_Alumina_Scan_H1_New_03.m4s1" --params "$MODEL" \
 --summary-output "$OUT/full03/03_full_profile_validation.csv"
echo "Completed. Training config at $MODEL and true 4324-profile validation at $OUT/full03/03_full_profile_validation.csv"
