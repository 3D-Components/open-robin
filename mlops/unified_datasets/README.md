# Unified Datasets

This folder contains builders and configs for datasets that intentionally combine multiple source datasets onto one modeling contract.

## Current unified dataset

`build_unified_legacy_arc0_plus_fragmented_dataset.py` creates:

- `data/unified_legacy_arc0_plus_fragmented/gold/train_dataset.csv`

It merges:

- legacy `robin_data_01` gold rows, mapped onto the `(wire feed, travel speed, arc length correction)` contract by setting `arc_length_correction_mm_model_input = 0.0`
- fragmented corrected-input selected-gold rows

Build it with:

```bash
python mlops/unified_datasets/build_unified_legacy_arc0_plus_fragmented_dataset.py
```

Evaluate it with the shared Torch leave-one-out evaluator:

```bash
python mlops/fragmented_datasets/evaluate_torch_selected_gold.py \
  --dataset-csv data/unified_legacy_arc0_plus_fragmented/gold/train_dataset.csv \
  --outdir data/comparisons/unified_legacy_arc0_plus_fragmented_torch_selected_gold \
  --model-dir data/models/unified_legacy_arc0_plus_fragmented/selected_gold_torch_mlp \
  --dataset-label "unified legacy arc0 plus fragmented selected gold"
```

Important caveat:

- the legacy rows use measured fallback wire-feed speed and assumed `arc_length_correction_mm_model_input = 0.0`

So this unified dataset is best understood as a pragmatic retrospective training table, not a pure prescriptive DOE table.

## Experimental chunked progression dataset

`chunked_unified_legacy_arc0_plus_fragmented_progression_experiment.py` creates an experimental chunked variant:

- `data/unified_legacy_arc0_plus_fragmented_chunk4_progression/gold/train_dataset.csv`

Each original unified row is split into 4 stable-region chunks:

- `0.10` to `0.30` with `chunk_center_progression_model_input = 0.20`
- `0.30` to `0.50` with `chunk_center_progression_model_input = 0.40`
- `0.50` to `0.70` with `chunk_center_progression_model_input = 0.60`
- `0.70` to `0.90` with `chunk_center_progression_model_input = 0.80`

It also runs a grouped Torch evaluation at:

- `data/comparisons/unified_legacy_arc0_plus_fragmented_chunk4_progression_torch_grouped`

Run it with:

```bash
python mlops/unified_datasets/chunked_unified_legacy_arc0_plus_fragmented_progression_experiment.py
```
