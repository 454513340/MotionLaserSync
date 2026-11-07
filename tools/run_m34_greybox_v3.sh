#!/usr/bin/env bash
# OFFLINE research grey-box imaging prototype. Never sends machine commands.
# Usage: bash tools/run_m34_greybox_v3.sh DIRECTORY_WITH_02_03_04_05_06_SCAN OUT_DIR
set -euo pipefail
if (( $# != 2 )); then echo "Usage: $0 SCAN_DIR OUTPUT_DIR" >&2; exit 2; fi
SRC="$(cd "$(dirname "$0")/.." && pwd)"
RAW="$(cd "$1" && pwd)"
mkdir -p "$2"
OUT="$(cd "$2" && pwd)"
OBJ="$SRC/examples/digital_twin/6061_sensor_connector_NEW_2026.obj"
STEP="$SRC/examples/digital_twin/6061_sensor_connector_NEW_2026.step"
POSE=(6.875014828 8.951331293 -235.26297484 -0.025937784 0.228699434 -89.416079059)
CX=0
CY=85.2034982349
CZ=247.676619523
FAN_SCALE=.991235470598
FAN_OFFSET=-.135174605085
mkdir -p "$OUT/sample" "$OUT/features" "$OUT/model"
cmake -S "$SRC" -B "$OUT/build" -DDIGITAL_TWIN_CLI_ONLY=ON -DTWIN_ENABLE_OCCT=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build "$OUT/build" --target twin_m34_v2_features_cli -j 4
SCANS=()
for id in 02 04 05 06 03; do
 f="$RAW/6061_Alumina_Scan_H1_New_${id}.scan"
 [[ -f "$f" ]] || { echo "Missing $f" >&2; exit 3; }
 SCANS+=("$f")
done
python "$SRC/tools/m0m5_prepare_scans.py" --scan "${SCANS[@]}" --step "$STEP" --output "$OUT/sample" --stride 8
SOURCES=()
for id in 02 04 05 06 03; do SOURCES+=("$OUT/sample/6061_Alumina_Scan_H1_New_${id}.m4s1"); done
python "$SRC/tools/m34_make_labels_bulk.py" --obj "$OBJ" --pose "${POSE[@]}" --m4 "${SOURCES[@]}"
for id in 02 04 05 06 03; do
 "$OUT/build/twin_m34_v2_features_cli" --obj "$OBJ" --pose "${POSE[@]}" \
  --camera-x "$CX" --camera-y "$CY" --camera-z "$CZ" \
  --fan-scale "$FAN_SCALE" --fan-offset "$FAN_OFFSET" \
  --source "$OUT/sample/6061_Alumina_Scan_H1_New_${id}.m4s1" \
  --output "$OUT/features/${id}.csv"
done
OPENBLAS_NUM_THREADS=1 PYTHONPATH="$SRC/tools" python "$SRC/tools/m34_fit_greybox.py" \
 --features "$OUT/features" --output "$OUT/model" --maxfev 90 --restarts 1 --depth-weight .02
OPENBLAS_NUM_THREADS=1 PYTHONPATH="$SRC/tools" python "$SRC/tools/m34_v3_ablate.py" \
 --features "$OUT/features" --params "$OUT/model/v3_frozen_params.json" --output "$OUT/model/v3_ablation.csv"
PYTHONPATH="$SRC/tools" python -m unittest discover -s "$SRC/tests/twin" -p 'test_m34_greybox_imager.py' -v
printf '\nFinished. All parameters still OFFLINE RESEARCH and M6 disabled.\n'
printf 'Model: %s\n' "$OUT/model/v3_frozen_params.json"
printf 'Metrics: %s\n' "$OUT/model/v3_metrics.csv"
