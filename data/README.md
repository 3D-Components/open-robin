# Data Layout

`data/` in this repository is intentionally small.

Most real-data assets do **not** live here:

- raw proprietary datasets
- cleaned training tables generated from those datasets
- notebook exports and investigation outputs
- bulky experiment artifacts and alternate model checkpoints

Those belong in a private/local workspace outside the repository.

## What Stays In Repo

Keep only small, durable artifacts that the repository needs for integration or
for one canonical benchmark:

- persisted runtime-facing model artifacts
- the matching feature scaler or equivalent preprocessing artifact
- compact benchmark outputs that explain the kept model

## Current In-Repo Contents

- `data/models/unified_legacy_arc0_plus_fragmented/selected_gold_torch_mlp/`
  - retained Torch checkpoint for the unified selected-gold model
  - matching `joblib`-serialized `StandardScaler`
- `data/comparisons/unified_legacy_arc0_plus_fragmented_torch_selected_gold/`
  - benchmark metrics and predictions for that retained model

## Ownership Rule

- `mlops/` contains code, configs, schemas, notebooks, and source-side
  definitions such as DOE files.
- `data/` contains only the small committed artifacts we intentionally keep.
- Generated datasets can still be written under `data/` locally while working,
  but they are not expected to be committed unless they are explicitly chosen as
  canonical lightweight artifacts.
