# MLOps decisions for this dataset

## Primary modeling stance

Use instructed/target process parameters as the long-term production input contract.

For this specific bag, `target_wire_speed` and `target_stickout` were not populated, so the pipeline supports **controlled fallbacks**:

- keep `target_wire_feed_speed_mpm` unchanged
- compute stable-window measured summaries from `/robin/data/fronius`
- populate `wire_speed_mpm_model_input` from target if valid, else from measured bead-level fallback
- record provenance in `wire_speed_source` and `wire_speed_is_fallback`
- keep `target_stickout_mm` unchanged
- populate `stickout_mm_model_input` from target if valid, else from DOE bead-level fallback
- record provenance in `stickout_source` and `stickout_is_fallback`

## Why this is acceptable

This lets you train a prototype model now while preserving the difference between:

- a true instructed wire speed
- a post-hoc measured surrogate used only because the target field is missing

## Why provenance matters

A model trained on fallback wire speed is a **hybrid prototype model**, not a pure target-driven production recipe model.
That is fine for the 3-bead example and early 20-50 bead experiments, but it should remain explicit in the dataset and model metadata.

## Recommended model versions

- **Model v1-common-subset**: `target_current_A`, `target_travel_speed_mps`
- **Model v1-hybrid**: `target_current_A`, `target_travel_speed_mps`, `wire_speed_mpm_model_input`, `stickout_mm_model_input`
- **Model v2-target-complete**: `target_current_A`, `target_wire_feed_speed_mpm`, `target_travel_speed_mps`, `target_stickout_mm` once future runs populate target wire speed and stickout reliably

## Rejection vs fallback

Missing or zero target wire speed should **not** automatically reject the bead if measured wire feed speed is present and the fallback policy is enabled.
Instead:

- keep the bead
- set `wire_speed_source = measured_bead_fallback`
- set `wire_speed_is_fallback = true`
- note the feature contract version in the exported dataset

## Stable-window aggregation

Use only the stable progression window for bead summaries, default `[0.10, 0.90]`.
This avoids startup and shutdown transients from contaminating the fallback value and the geometry labels.

Outlier handling is applied inside the stable window with:
- hard physical bounds from config validation limits
- optional per-bead IQR filtering before aggregation
