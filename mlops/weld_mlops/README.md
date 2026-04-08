# Weld Bead MLOps Starter — final version with wire-speed fallback

This package defines a reproducible MLOps flow for turning a ROS 2 MCAP bag into a bead-level CSV for training a PyTorch MLP.

## Objective

Predict:
- `height_mm_target`
- `width_mm_target`

From process inputs:
- current
- wire feed speed
- travel speed
- stickout

## Final modeling stance

### Long-term production contract
Use **target/instructed** process parameters as the primary production inputs because those are the variables known at planning time.

### This bag's exception
For this specific bag, `target_wire_speed` and `target_stickout` were not populated in `ActiveBead`. To keep the prototype usable without corrupting the data contract, this package adds controlled fallbacks:

- preserve `target_wire_feed_speed_mpm` exactly as recorded
- compute stable-window measured wire feed speed per bead
- derive `wire_speed_mpm_model_input`
- set `wire_speed_source` to either `target` or `measured_bead_fallback`
- set `wire_speed_is_fallback` to `true` or `false`
- preserve `target_stickout_mm` exactly as recorded
- optionally backfill per-bead stickout from the configured DOE CSV
- derive `stickout_mm_model_input`
- set `stickout_source` to either `target` or `doe_bead_fallback`
- set `stickout_is_fallback` to `true` or `false`

That means the exported dataset is suitable for a **hybrid prototype model** today while remaining forward-compatible with future bags that contain real target wire speed.

## Recommended exported features

### Core recipe features
- `target_current_A`
- `target_travel_speed_mps`
- `wire_speed_mpm_model_input`
- `stickout_mm_model_input`

### Provenance columns that must stay with the data
- `target_wire_feed_speed_mpm`
- `target_wire_feed_speed_available`
- `wire_speed_source`
- `wire_speed_is_fallback`
- `target_stickout_mm`
- `target_stickout_available`
- `stickout_source`
- `stickout_is_fallback`

### Diagnostic measured columns
- `measured_current_A_mean`
- `measured_current_A_median`
- `measured_wire_feed_speed_mpm_mean`
- `measured_wire_feed_speed_mpm_median`
- `measured_voltage_V_mean`
- `measured_power_W_mean`

### Labels
- `height_mm_target`
- `width_mm_target`

## Why one target can map to many geometries

That is expected. A single recipe can produce a distribution of outcomes because of:
- sensor noise
- process disturbance
- thermal history
- bead position and order
- robot execution variation
- material variation

Do **not** use one raw time sample as one training row. Create **one row per bead** and aggregate raw signals robustly over the stable bead window.

## Stable-window rule

Default stable progression window:
- exclude first 10 percent of progression
- exclude last 10 percent of progression

Recommended bead-level aggregation:
- label targets = median height and median width in the stable window
- fallback wire speed = stable-window measured mean or median per bead
- remove sample outliers with per-bead IQR filtering (configurable)

## Outlier filtering

The extractor applies two layers before bead aggregation:
1. hard physical bounds from `validation` (e.g. height, width, current, wire-feed speed)
2. optional per-bead IQR filter on stable-window samples

Control this in `configs/pipeline.yaml` under `outlier_policy`.

## Directory layout

```text
project/
  data/
    raw/
      2026-03-16-12-13-32_0.mcap
      metadata.yaml
    bronze/
      topic_exports/
    silver/
      bead_level/
    gold/
      train_dataset.csv
  configs/
    pipeline.yaml
  schemas/
    train_dataset_schema.csv
  docs/
    mlops_decisions.md
  scripts/
    extract_rosbag_to_csv.py
```

## Pipeline layers

### Raw
Immutable MCAP and metadata files.

### Bronze
Topic-level decoded tables, one file per topic:
- weld dimensions
- welder data
- progression
- active bead
- welding state

### Silver
Cleaned and aligned bead-level tables:
- progression filtered
- bead boundaries identified
- geometry samples cleaned
- measured signals aggregated per bead
- wire-speed fallback derived if needed

### Gold
Model-ready CSV with one row per bead and fixed schema.

## Training pipeline (PyTorch + scaling)

The package includes a Torch training script that:
- filters to `quality_flag == ok`
- selects configured feature/target columns
- scales input features with `StandardScaler` fit on the training split
- trains a multi-output MLP regressor for height and width
- writes model, scaler, and metrics artifacts

Run extraction first, then training:

```bash
poetry run python mlops/weld_mlops/scripts/extract_rosbag_to_csv.py \
  --config mlops/weld_mlops/configs/pipeline.yaml \
  --run-id robin_data_01

poetry run python mlops/weld_mlops/scripts/train_torch_model.py \
  --config mlops/weld_mlops/configs/training.yaml
```

