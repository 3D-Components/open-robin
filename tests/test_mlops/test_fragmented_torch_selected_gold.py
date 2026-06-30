from __future__ import annotations

import json
import math
import sys

import pandas as pd
import pytest

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


def test_load_selected_gold_filters_quality_and_requires_three_rows(tmp_path) -> None:
    csv_path = tmp_path / "selected_gold.csv"
    pd.DataFrame(
        [
            {
                "quality_flag": "ok",
                "wire_feed_speed_mpm_model_input": 8.0,
                "travel_speed_mps_model_input": 0.01,
                "arc_length_correction_mm_model_input": 0.0,
                "height_mm_target": 2.0,
                "width_mm_target": 5.0,
            },
            {
                "quality_flag": "reject_missing_geometry_samples",
                "wire_feed_speed_mpm_model_input": 9.0,
                "travel_speed_mps_model_input": 0.02,
                "arc_length_correction_mm_model_input": 1.0,
                "height_mm_target": 2.5,
                "width_mm_target": 5.5,
            },
            {
                "quality_flag": "ok",
                "wire_feed_speed_mpm_model_input": 10.0,
                "travel_speed_mps_model_input": 0.03,
                "arc_length_correction_mm_model_input": 2.0,
                "height_mm_target": float("nan"),
                "width_mm_target": 6.0,
            },
        ]
    ).to_csv(csv_path, index=False)

    with pytest.raises(ValueError, match="Need at least 3"):
        torch_eval.load_selected_gold(csv_path)


def test_write_readme_and_main_write_evaluation_outputs(
    tmp_path, monkeypatch
) -> None:
    dataset_csv = tmp_path / "dataset.csv"
    outdir = tmp_path / "out"
    model_dir = tmp_path / "model"
    df = pd.DataFrame(
        {
            "doe_input_id": ["R1", "R2", "R3"],
            "attempt_id": ["A1", "A2", "A3"],
            "quality_flag": ["ok", "ok", "ok"],
            "wire_feed_speed_mpm_model_input": [8.0, 9.0, 10.0],
            "travel_speed_mps_model_input": [0.01, 0.02, 0.03],
            "arc_length_correction_mm_model_input": [0.0, 1.0, 2.0],
            "height_mm_target": [2.0, 2.5, 3.0],
            "width_mm_target": [5.0, 5.5, 6.0],
        }
    )
    df.to_csv(dataset_csv, index=False)

    pred_df = pd.DataFrame(
        {
            "doe_input_id": ["R1"],
            "attempt_id": ["A1"],
            "height_mm_target_actual": [2.0],
            "height_mm_target_pred": [2.1],
            "height_mm_target_abs_error": [0.1],
            "width_mm_target_actual": [5.0],
            "width_mm_target_pred": [5.1],
            "width_mm_target_abs_error": [0.1],
        }
    )
    summary = {
        "dataset_csv": str(dataset_csv),
        "dataset_label": "test selected gold",
        "n_rows": 3,
        "feature_columns": torch_eval.FEATURE_COLUMNS,
        "target_columns": torch_eval.TARGET_COLUMNS,
        "model": dict(torch_eval.MODEL_CONFIG),
        "r2_variance_weighted": 0.9,
        "targets": {
            "height_mm_target": {"r2": 0.91, "rmse": 0.1, "mae": 0.1},
            "width_mm_target": {"r2": 0.92, "rmse": 0.1, "mae": 0.1},
        },
    }

    monkeypatch.setattr(torch_eval, "evaluate_loo", lambda *_args: (pred_df, summary.copy()))

    def fake_save_refit_artifact(df_arg, summary_arg, *, dataset_csv, model_path, scaler_path):
        model_path.parent.mkdir(parents=True, exist_ok=True)
        scaler_path.parent.mkdir(parents=True, exist_ok=True)
        model_path.write_text("model")
        scaler_path.write_text("scaler")
        summary_arg["saved_model_artifact"] = {
            "model_path": str(model_path),
            "scaler_path": str(scaler_path),
        }
        return summary_arg

    monkeypatch.setattr(torch_eval, "save_refit_artifact", fake_save_refit_artifact)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "evaluate_torch_selected_gold",
            "--dataset-csv",
            str(dataset_csv),
            "--outdir",
            str(outdir),
            "--model-dir",
            str(model_dir),
            "--dataset-label",
            "test selected gold",
        ],
    )

    torch_eval.main()

    assert (outdir / "torch_selected_gold_predictions.csv").exists()
    metrics = json.loads((outdir / "torch_selected_gold_metrics.json").read_text())
    assert metrics["saved_model_artifact"]["model_path"].endswith(
        "torch_selected_gold_model.pt"
    )
    readme = (outdir / "README.md").read_text()
    assert "Torch Selected-Gold MLP Evaluation" in readme
    assert "variance-weighted R2" in readme
