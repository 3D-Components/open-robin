# Model Evidence

This folder contains model-performance evidence for the public review.

## Open Online Model Benchmark

The open online benchmark is a Torch MLP leave-one-out evaluation on the `unified legacy arc0 plus fragmented selected gold` dataset view.

| Metric | Value |
| --- | --- |
| Validation protocol | Leave-one-out |
| Usable rows | 51 |
| Variance-weighted R2 | 0.8947 |
| Height R2 | 0.9019 |
| Width R2 | 0.8942 |
| Model | Torch MLP, hidden layers `[64, 32]` |
| Optimizer | LBFGS |
| Weight decay | 0.0001 |
| Random seed | 6 |

Evidence files:

- [open-online-model-r2-090-scatter.png](open-online-model-r2-090-scatter.png): actual-vs-predicted scatter plot generated from the benchmark predictions.
- [open-online-model-r2-090-metrics.json](open-online-model-r2-090-metrics.json): retained benchmark metrics.
- [open-online-model-r2-090-predictions.csv](open-online-model-r2-090-predictions.csv): leave-one-out predictions used to generate the plot.

Source comparison artifacts are retained in `data/comparisons/unified_legacy_arc0_plus_fragmented_torch_selected_gold/`.

## Proprietary Offline Benchmark

[proprietary-offline-model-r2-096-scatter.png](proprietary-offline-model-r2-096-scatter.png) is D3 evidence for a proprietary/offline benchmark with generalized R2 about 0.96. If referenced externally, describe it clearly as proprietary/external benchmark evidence, not as the open reusable model.
