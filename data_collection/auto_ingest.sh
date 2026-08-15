#!/usr/bin/env bash
# auto_ingest.sh — one-command pipeline run after a new drag-demo collection batch.
#
# Re-runs SOP verifier -> build initial-batch manifest -> loader smoke test
# against ~/workspace/raw_data, then prints how many episodes count toward the
# 100-demo target at the accepted 0.10 rad stillness threshold.
#
# Usage:
#   ~/workspace/arx_wrapper/data_collection/auto_ingest.sh
#
# Optional overrides via env vars:
#   RAW_DATA_ROOT      default: ~/workspace/raw_data
#   REPORT_DIR         default: ~/workspace/.verifier_reports
#   PYTHON             default: ~/miniconda3/envs/robo_ctrl/bin/python
#   STILLNESS_RAD      default: 0.10
#
# Exit code: 0 on full success; 1 if the loader smoke test fails (i.e. one of
# the counted episodes won't open cleanly — investigate before re-running).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
PYTHON="${PYTHON:-$HOME/miniconda3/envs/robo_ctrl/bin/python}"
RAW_DATA_ROOT="${RAW_DATA_ROOT:-${ARX_DATA_ROOT:-$HOME/workspace/raw_data}}"
REPORT_DIR="${REPORT_DIR:-$HOME/workspace/.verifier_reports}"
STILLNESS_RAD="${STILLNESS_RAD:-0.10}"

VERIFIER="$REPO_ROOT/data_collection/sop_episode_verifier.py"
BUILDER="$REPO_ROOT/data_collection/build_initial_batch_manifest.py"
LOADER_SMOKE="$REPO_ROOT/data_collection/load_initial_batch.py"

REPORT_JSON="$REPORT_DIR/raw_data_full_report.json"
MANIFEST_JSON="$REPORT_DIR/initial_batch_manifest.json"

mkdir -p "$REPORT_DIR"

echo "==> [1/3] SOP verifier  (threshold ${STILLNESS_RAD} rad)"
# Exit 1 from the verifier just means some episodes failed SOP gates — that's
# the whole point of running it. Capture the code for the final summary but
# do not abort the pipeline on it.
set +e
"$PYTHON" "$VERIFIER" \
    --root "$RAW_DATA_ROOT" \
    --report "$REPORT_JSON" \
    --stillness-threshold-rad "$STILLNESS_RAD" \
    --quiet | tail -40
VERIFIER_EXIT=${PIPESTATUS[0]}
set -e
echo "(verifier exit=$VERIFIER_EXIT — exit 1 just means >=1 episode failed an SOP gate, not a script error.)"
echo

echo "==> [2/3] build manifest"
"$PYTHON" "$BUILDER" \
    --report "$REPORT_JSON" \
    --out "$MANIFEST_JSON"
echo

echo "==> [3/3] loader smoke test"
"$PYTHON" "$LOADER_SMOKE" \
    --manifest "$MANIFEST_JSON" \
    --include-eval

echo
echo "==> ingest done."
"$PYTHON" - "$MANIFEST_JSON" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
print(f"counted_toward_target = {m['n_counted_toward_target']} / "
      f"{m['demo_target']}  (delta {m['delta_to_target']})")
print(f"eval_tier_strict       = {m['n_eval_tier_strict']}  (held-out)")
print(f"by_dataset_counted     = {m['by_dataset_counted']}")
PY
