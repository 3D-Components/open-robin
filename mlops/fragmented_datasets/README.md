# Fragmented DOE Dataset

This folder contains the corrected-input fragmented weld dataset and the tooling to turn it into a model-ready dataset without touching the legacy `robin_data_01` output.

## Source data

- `robin_data_03` to `robin_data_07`: fragmented corrected-input MCAP bags
- `DOE_LHS_48.json`: canonical intended 48-point DOE for this fragmented run set
- canonical mapping outputs in `data/fragmented_doe_corrected/metadata/`
  - `ros_doe_mapping_summary.csv`: attempt-to-DOE manifest used as the canonical mapping input
  - `ros_doe_mapping_report.md`: notes about the mapping assumptions and bag quality
  - `ros_bag_inventory.csv`: per-bag source inventory

The resulting selected dataset currently gives you 22 unique DOE recipes, with 21 trainable rows (`quality_flag == ok`) and one explicit rejected placeholder for `LHS48-22`, which only contains progression messages.

To rebuild the mapping manifest from the canonical DOE:

```bash
python mlops/fragmented_datasets/map_ros_doe_dataset.py \
  --data-dir mlops/fragmented_datasets \
  --doe-json mlops/fragmented_datasets/DOE_LHS_48.json \
  --outdir data/fragmented_doe_corrected/metadata
```

## Build the cleaned dataset

```bash
python mlops/fragmented_datasets/build_fragmented_dataset.py \
  --config mlops/fragmented_datasets/configs/pipeline.yaml
```

This writes a separate dataset tree under `data/fragmented_doe_corrected/`:

- `metadata/`: canonical mapping manifest, mapping report, and source inventory
- `bronze/topic_exports/`: decoded per-topic CSVs for each bag
- `silver/bead_level/attempt_level_dataset.csv`: one cleaned row per mapped attempt
- `gold/train_dataset.csv`: one selected row per unique DOE recipe
- `gold/selection_report.csv`: candidate ranking within each DOE recipe
- `gold/qc_report.json`: overall counts and per-attempt filtering details
- `schemas/train_dataset_schema.csv`: column-level schema for the selected gold dataset
- `notebooks/explore_fragmented_corrected_dataset.ipynb`: exploration notebook centered on the retained Torch evaluation

## Canonical Torch Evaluation

```bash
python mlops/fragmented_datasets/evaluate_torch_selected_gold.py
```

This is the single retained model benchmark for the fragmented corrected-input dataset:

- dataset view: selected gold rows only
- usable rows: `21`
- validation: leave-one-out
- model: Torch MLP, hidden layers `[64, 32]`
- optimizer: `LBFGS`
- weight decay: `1e-4`
- max iterations per fold: `215`
- seed: `6`
- variance-weighted `R2`: `0.8681`

The feature contract is:

- `wire_feed_speed_mpm_model_input`
- `travel_speed_mps_model_input`
- `arc_length_correction_mm_model_input`

Outputs are written under `data/comparisons/fragmented_corrected_torch_selected_gold/`:

- `torch_selected_gold_metrics.json`
- `torch_selected_gold_predictions.csv`
- `README.md`

The saved refit model artifact is written under `data/models/fragmented_doe_corrected/selected_gold_torch_mlp/`:

- `torch_selected_gold_model.pt`
- `torch_selected_gold_feature_scaler.joblib`

The saved model artifact is a refit on all `21` usable selected-gold rows. The retained `0.8681` score remains the leave-one-out benchmark, not the in-sample refit score.

## Explore Notebook

To regenerate the notebook:

```bash
python mlops/fragmented_datasets/notebooks/build_explore_fragmented_corrected_notebook.py
```
