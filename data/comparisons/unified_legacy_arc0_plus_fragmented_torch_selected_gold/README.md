# Torch Selected-Gold MLP Evaluation

This is a Torch leave-one-out evaluation for `unified legacy arc0 plus fragmented selected gold`.

- dataset: `data/unified_legacy_arc0_plus_fragmented/gold/train_dataset.csv`
- usable rows: `51`
- validation: leave-one-out on the selected gold rows
- model: Torch MLP with hidden layers `[64, 32]`
- optimizer: `LBFGS`
- weight decay: `0.0001`
- max iterations per fold: `215`
- constant initialization seed: `6`

- variance-weighted R2: `0.8947`
- height R2: `0.9019`
- width R2: `0.8942`

Artifacts:

- `torch_selected_gold_metrics.json`
- `torch_selected_gold_predictions.csv`
- comparison dir: `data/comparisons/unified_legacy_arc0_plus_fragmented_torch_selected_gold`
- saved model: `data/models/unified_legacy_arc0_plus_fragmented/selected_gold_torch_mlp/torch_selected_gold_model.pt`
- saved scaler: `data/models/unified_legacy_arc0_plus_fragmented/selected_gold_torch_mlp/torch_selected_gold_feature_scaler.joblib`

The saved model artifact is a refit on all `51` usable selected-gold rows.
The reported R2 benchmark remains the leave-one-out score above.
