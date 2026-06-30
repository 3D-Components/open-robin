from __future__ import annotations

import json
import sys

import pandas as pd

from mlops.unified_datasets import (
    build_unified_legacy_arc0_plus_fragmented_dataset as unified,
)
from mlops.unified_datasets.build_unified_legacy_arc0_plus_fragmented_dataset import (
    FRAGMENTED_SOURCE_NAME,
    LEGACY_SOURCE_NAME,
    build_unified_dataset,
)


def test_build_unified_dataset_collapses_duplicate_feature_triples() -> None:
    component_df = pd.DataFrame(
        [
            {
                "source_dataset": LEGACY_SOURCE_NAME,
                "source_run_id": "legacy",
                "source_recipe_id": "B001",
                "source_record_id": "legacy::B001",
                "component_quality_flag": "ok",
                "wire_feed_speed_mpm_model_input": 10.0,
                "travel_speed_mps_model_input": 0.01,
                "arc_length_correction_mm_model_input": 0.0,
                "height_mm_target": 2.0,
                "width_mm_target": 5.0,
                "height_mm_std": 0.1,
                "width_mm_std": 0.2,
                "height_mm_p10": 1.9,
                "height_mm_p90": 2.1,
                "width_mm_p10": 4.8,
                "width_mm_p90": 5.2,
                "n_geometry_samples": 100,
                "n_welder_samples": 20,
                "wire_feature_origin": "legacy",
                "travel_feature_origin": "legacy",
                "arc_feature_origin": "assumed_zero",
                "legacy_target_current_A": 150.0,
                "legacy_stickout_mm_aux": 12.0,
                "support_count_within_source_recipe": 1,
                "unification_note": "legacy",
            },
            {
                "source_dataset": FRAGMENTED_SOURCE_NAME,
                "source_run_id": "fragmented",
                "source_recipe_id": "LHS48-01",
                "source_record_id": "fragmented::A001",
                "component_quality_flag": "ok",
                "wire_feed_speed_mpm_model_input": 10.0,
                "travel_speed_mps_model_input": 0.01,
                "arc_length_correction_mm_model_input": 0.0,
                "height_mm_target": 4.0,
                "width_mm_target": 9.0,
                "height_mm_std": 0.2,
                "width_mm_std": 0.3,
                "height_mm_p10": 3.7,
                "height_mm_p90": 4.3,
                "width_mm_p10": 8.5,
                "width_mm_p90": 9.5,
                "n_geometry_samples": 120,
                "n_welder_samples": 24,
                "wire_feature_origin": "fragmented",
                "travel_feature_origin": "fragmented",
                "arc_feature_origin": "fragmented",
                "legacy_target_current_A": float("nan"),
                "legacy_stickout_mm_aux": float("nan"),
                "support_count_within_source_recipe": 2,
                "unification_note": "fragmented",
            },
        ]
    )

    train_df, agg_df = build_unified_dataset(component_df)

    assert len(train_df) == 1
    assert len(agg_df) == 1
    row = train_df.iloc[0]
    assert row["source_row_count"] == 2
    assert row["legacy_row_count"] == 1
    assert row["fragmented_row_count"] == 1
    assert row["height_mm_target"] == 3.0
    assert row["width_mm_target"] == 7.0
    assert row["n_geometry_samples_total"] == 220


def test_load_component_rows_and_write_outputs(tmp_path, monkeypatch) -> None:
    legacy_csv = tmp_path / "legacy.csv"
    fragmented_csv = tmp_path / "fragmented.csv"
    attempt_csv = tmp_path / "attempts.csv"
    outroot = tmp_path / "unified"

    pd.DataFrame(
        [
            {
                "run_id": "legacy-run",
                "bead_id": "B001",
                "quality_flag": "ok",
                "wire_speed_mpm_model_input": 10.0,
                "target_travel_speed_mps": 0.02,
                "height_mm_target": 2.0,
                "width_mm_target": 5.0,
                "height_mm_std": 0.1,
                "width_mm_std": 0.2,
                "height_mm_p10": 1.9,
                "height_mm_p90": 2.1,
                "width_mm_p10": 4.8,
                "width_mm_p90": 5.2,
                "n_geometry_samples": 100,
                "n_welder_samples": 20,
                "wire_speed_source": "measured",
                "target_current_A": 150.0,
                "stickout_mm_model_input": 12.0,
            },
            {
                "run_id": "legacy-bad",
                "bead_id": "B002",
                "quality_flag": "reject",
                "wire_speed_mpm_model_input": 9.0,
                "target_travel_speed_mps": 0.03,
                "height_mm_target": 2.5,
                "width_mm_target": 5.5,
                "height_mm_std": 0.1,
                "width_mm_std": 0.2,
                "height_mm_p10": 2.4,
                "height_mm_p90": 2.6,
                "width_mm_p10": 5.3,
                "width_mm_p90": 5.7,
                "n_geometry_samples": 80,
                "n_welder_samples": 10,
                "wire_speed_source": "measured",
                "target_current_A": 140.0,
                "stickout_mm_model_input": 11.0,
            },
        ]
    ).to_csv(legacy_csv, index=False)

    pd.DataFrame(
        [
            {
                "run_id": "frag-run",
                "doe_input_id": "R1",
                "attempt_id": "A1",
                "quality_flag": "ok",
                "wire_feed_speed_mpm_model_input": 11.0,
                "travel_speed_mps_model_input": 0.021,
                "arc_length_correction_mm_model_input": 1.5,
                "height_mm_target": 3.0,
                "width_mm_target": 6.0,
                "height_mm_std": 0.2,
                "width_mm_std": 0.3,
                "height_mm_p10": 2.8,
                "height_mm_p90": 3.2,
                "width_mm_p10": 5.7,
                "width_mm_p90": 6.3,
                "n_geometry_samples": 120,
                "n_welder_samples": 24,
                "input_source": "doe_mapped_recipe",
            }
        ]
    ).to_csv(fragmented_csv, index=False)

    pd.DataFrame(
        [
            {"doe_input_id": "R1", "quality_flag": "ok"},
            {"doe_input_id": "R1", "quality_flag": "ok"},
            {"doe_input_id": "R2", "quality_flag": "reject"},
        ]
    ).to_csv(attempt_csv, index=False)

    monkeypatch.setattr(unified, "LEGACY_GOLD_CSV", legacy_csv)
    monkeypatch.setattr(unified, "FRAGMENTED_GOLD_CSV", fragmented_csv)
    monkeypatch.setattr(unified, "FRAGMENTED_ATTEMPT_CSV", attempt_csv)
    monkeypatch.setattr(unified, "OUTROOT", outroot)
    monkeypatch.setattr(unified, "OUT_GOLD_CSV", outroot / "gold" / "train_dataset.csv")
    monkeypatch.setattr(unified, "OUT_COMPONENT_CSV", outroot / "metadata" / "component_rows.csv")
    monkeypatch.setattr(unified, "OUT_AGG_CSV", outroot / "metadata" / "aggregation_report.csv")
    monkeypatch.setattr(unified, "OUT_SUMMARY_JSON", outroot / "metadata" / "summary.json")
    monkeypatch.setattr(unified, "OUT_README", outroot / "README.md")

    legacy_rows = unified.load_legacy_rows()
    fragmented_rows = unified.load_fragmented_rows()

    assert len(legacy_rows) == 1
    assert legacy_rows.iloc[0]["arc_length_correction_mm_model_input"] == 0.0
    assert fragmented_rows.iloc[0]["support_count_within_source_recipe"] == 2

    component_df = pd.concat([legacy_rows, fragmented_rows], ignore_index=True)
    train_df, agg_df = unified.build_unified_dataset(component_df)
    unified._ensure_dirs()
    unified.write_outputs(component_df, train_df, agg_df)

    assert unified.OUT_GOLD_CSV.exists()
    assert unified.OUT_COMPONENT_CSV.exists()
    summary = json.loads(unified.OUT_SUMMARY_JSON.read_text())
    assert summary["component_rows_total"] == 2
    assert "Unified Legacy Arc0 Plus Fragmented Dataset" in unified.OUT_README.read_text()


def test_unified_main_writes_outputs(tmp_path, monkeypatch) -> None:
    outroot = tmp_path / "main-out"
    component_df = pd.DataFrame(
        [
            {
                "source_dataset": LEGACY_SOURCE_NAME,
                "source_run_id": "legacy",
                "source_recipe_id": "B001",
                "source_record_id": "legacy::B001",
                "component_quality_flag": "ok",
                "wire_feed_speed_mpm_model_input": 10.0,
                "travel_speed_mps_model_input": 0.01,
                "arc_length_correction_mm_model_input": 0.0,
                "height_mm_target": 2.0,
                "width_mm_target": 5.0,
                "height_mm_std": 0.1,
                "width_mm_std": 0.2,
                "height_mm_p10": 1.9,
                "height_mm_p90": 2.1,
                "width_mm_p10": 4.8,
                "width_mm_p90": 5.2,
                "n_geometry_samples": 100,
                "n_welder_samples": 20,
                "wire_feature_origin": "legacy",
                "travel_feature_origin": "legacy",
                "arc_feature_origin": "assumed_zero",
                "legacy_target_current_A": 150.0,
                "legacy_stickout_mm_aux": 12.0,
                "support_count_within_source_recipe": 1,
                "unification_note": "legacy",
            }
        ]
    )

    monkeypatch.setattr(unified, "OUTROOT", outroot)
    monkeypatch.setattr(unified, "OUT_GOLD_CSV", outroot / "gold" / "train_dataset.csv")
    monkeypatch.setattr(unified, "OUT_COMPONENT_CSV", outroot / "metadata" / "component_rows.csv")
    monkeypatch.setattr(unified, "OUT_AGG_CSV", outroot / "metadata" / "aggregation_report.csv")
    monkeypatch.setattr(unified, "OUT_SUMMARY_JSON", outroot / "metadata" / "summary.json")
    monkeypatch.setattr(unified, "OUT_README", outroot / "README.md")
    monkeypatch.setattr(unified, "load_legacy_rows", lambda: component_df)
    monkeypatch.setattr(
        unified,
        "load_fragmented_rows",
        lambda: component_df.assign(source_dataset=FRAGMENTED_SOURCE_NAME),
    )
    monkeypatch.setattr(sys, "argv", ["build_unified"])

    unified.main()

    assert unified.OUT_GOLD_CSV.exists()
    summary = json.loads(unified.OUT_SUMMARY_JSON.read_text())
    assert summary["legacy_source_rows"] == 1
    assert summary["fragmented_source_rows"] == 1
