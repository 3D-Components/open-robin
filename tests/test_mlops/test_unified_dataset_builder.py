from __future__ import annotations

import pandas as pd

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
