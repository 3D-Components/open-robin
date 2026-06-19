from __future__ import annotations

import math

import pandas as pd

from mlops.fragmented_datasets import evaluate_torch_selected_gold as torch_eval


def test_evaluate_loo_returns_prediction_per_row(monkeypatch) -> None:
    model_df = pd.DataFrame(
        {
            "doe_input_id": [f"LHS48-{idx:02d}" for idx in range(1, 7)],
            "attempt_id": [f"A{idx}" for idx in range(1, 7)],
            "quality_flag": ["ok"] * 6,
            "wire_feed_speed_mpm_model_input": [8.0, 9.0, 10.0, 11.0, 12.0, 13.0],
            "travel_speed_mps_model_input": [0.010, 0.012, 0.014, 0.016, 0.018, 0.020],
            "arc_length_correction_mm_model_input": [-4.0, -2.0, 0.0, 2.0, 4.0, 6.0],
            "height_mm_target": [1.8, 2.0, 2.4, 2.9, 3.3, 3.8],
            "width_mm_target": [4.1, 4.5, 5.0, 5.6, 6.1, 6.7],
        }
    )

    monkeypatch.setitem(torch_eval.MODEL_CONFIG, "hidden_dims", [4])
    monkeypatch.setitem(torch_eval.MODEL_CONFIG, "weight_decay", 1e-4)
    monkeypatch.setitem(torch_eval.MODEL_CONFIG, "max_iter", 40)
    monkeypatch.setitem(torch_eval.MODEL_CONFIG, "random_seed", 3)

    predictions_df, metrics = torch_eval.evaluate_loo(
        model_df,
        dataset_csv=torch_eval.DEFAULT_GOLD_CSV,
        dataset_label="test dataset",
    )

    assert len(predictions_df) == len(model_df)
    assert set(predictions_df["doe_input_id"]) == set(model_df["doe_input_id"])
    assert metrics["n_rows"] == len(model_df)
    assert metrics["model"]["hidden_dims"] == [4]
    assert math.isfinite(metrics["r2_variance_weighted"])
    assert math.isfinite(metrics["targets"]["height_mm_target"]["r2"])
    assert math.isfinite(metrics["targets"]["width_mm_target"]["r2"])


def test_save_refit_artifact_writes_model_and_scaler(tmp_path, monkeypatch) -> None:
    model_df = pd.DataFrame(
        {
            "doe_input_id": [f"LHS48-{idx:02d}" for idx in range(1, 7)],
            "attempt_id": [f"A{idx}" for idx in range(1, 7)],
            "quality_flag": ["ok"] * 6,
            "wire_feed_speed_mpm_model_input": [8.0, 9.0, 10.0, 11.0, 12.0, 13.0],
            "travel_speed_mps_model_input": [0.010, 0.012, 0.014, 0.016, 0.018, 0.020],
            "arc_length_correction_mm_model_input": [-4.0, -2.0, 0.0, 2.0, 4.0, 6.0],
            "height_mm_target": [1.8, 2.0, 2.4, 2.9, 3.3, 3.8],
            "width_mm_target": [4.1, 4.5, 5.0, 5.6, 6.1, 6.7],
        }
    )

    monkeypatch.setitem(torch_eval.MODEL_CONFIG, "hidden_dims", [4])
    monkeypatch.setitem(torch_eval.MODEL_CONFIG, "weight_decay", 1e-4)
    monkeypatch.setitem(torch_eval.MODEL_CONFIG, "max_iter", 40)
    monkeypatch.setitem(torch_eval.MODEL_CONFIG, "random_seed", 3)
    model_path = tmp_path / "model.pt"
    scaler_path = tmp_path / "scaler.joblib"

    _, summary = torch_eval.evaluate_loo(
        model_df,
        dataset_csv=torch_eval.DEFAULT_GOLD_CSV,
        dataset_label="test dataset",
    )
    summary = torch_eval.save_refit_artifact(
        model_df,
        summary,
        dataset_csv=torch_eval.DEFAULT_GOLD_CSV,
        model_path=model_path,
        scaler_path=scaler_path,
    )

    assert model_path.exists()
    assert scaler_path.exists()
    assert summary["saved_model_artifact"]["model_path"] == str(model_path)
    assert summary["saved_model_artifact"]["scaler_path"] == str(scaler_path)
    assert math.isfinite(summary["saved_model_artifact"]["refit_train_metrics"]["r2_variance_weighted_train"])
