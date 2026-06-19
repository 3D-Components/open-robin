from __future__ import annotations

from pathlib import Path

import pandas as pd

from mlops.fragmented_datasets.build_fragmented_dataset import (
    Config,
    aggregate_attempt,
    select_recipe_rows,
)


def _cfg() -> Config:
    return Config(
        dataset_id="fragmented_doe_corrected_v1",
        schema_version="v1.0",
        feature_contract_version="corrected_inputs_v1",
        source_root=Path("mlops/fragmented_datasets"),
        mapping_csv_path=Path("data/fragmented_doe_corrected/metadata/ros_doe_mapping_summary.csv"),
        bronze_dir=Path("data/fragmented_doe_corrected/bronze/topic_exports"),
        silver_csv=Path("data/fragmented_doe_corrected/silver/bead_level/attempt_level_dataset.csv"),
        gold_csv=Path("data/fragmented_doe_corrected/gold/train_dataset.csv"),
        selection_report_csv=Path("data/fragmented_doe_corrected/gold/selection_report.csv"),
        qc_report_json=Path("data/fragmented_doe_corrected/gold/qc_report.json"),
        readme_path=Path("data/fragmented_doe_corrected/README.md"),
    )


def test_select_recipe_rows_prefers_ok_and_exact_match() -> None:
    df = pd.DataFrame(
        [
            {
                "doe_input_id": "LHS48-01",
                "attempt_id": "A001",
                "quality_flag": "ok",
                "weld_speed_match": "mismatch",
                "geometry_anomaly_flag": False,
                "progression_anomaly_flag": False,
                "ros_travel_speed_cmd_mps": 0.0198,
                "doe_travel_speed_mps": 0.0297,
                "n_geometry_samples": 251,
                "n_welder_samples": 74,
                "start_time_ns": 10,
            },
            {
                "doe_input_id": "LHS48-01",
                "attempt_id": "A002",
                "quality_flag": "ok",
                "weld_speed_match": "exact",
                "geometry_anomaly_flag": False,
                "progression_anomaly_flag": False,
                "ros_travel_speed_cmd_mps": 0.0297,
                "doe_travel_speed_mps": 0.0297,
                "n_geometry_samples": 247,
                "n_welder_samples": 58,
                "start_time_ns": 5,
            },
            {
                "doe_input_id": "LHS48-01",
                "attempt_id": "A003",
                "quality_flag": "reject_missing_geometry_samples",
                "weld_speed_match": "exact",
                "geometry_anomaly_flag": False,
                "progression_anomaly_flag": False,
                "ros_travel_speed_cmd_mps": 0.0297,
                "doe_travel_speed_mps": 0.0297,
                "n_geometry_samples": 0,
                "n_welder_samples": 50,
                "start_time_ns": 1,
            },
        ]
    )

    out = select_recipe_rows(df)

    selected = out[out["selected_for_training"]].iloc[0]
    assert selected["attempt_id"] == "A002"
    assert selected["selection_reason"] == "selected_best_quality_exact_match"


def test_aggregate_attempt_uses_doe_inputs_and_rejects_missing_geometry() -> None:
    cfg = _cfg()
    attempt = pd.Series(
        {
            "attempt_id": "robin_data_07_0.mcap::A022",
            "bag_file": "robin_data_07_0.mcap",
            "attempt_idx_in_bag": 22,
            "start_ts": 100,
            "end_ts_exclusive": 200,
            "start_bead_id": "B018",
            "input_id": "LHS48-22",
            "wire_feed_speed_cmd_ros": 10.42,
            "weld_speed_cmd_ros": 0.0181,
            "arc_length_correction_cmd_ros": 6.5,
            "current_recomm_ros": 213.67,
            "voltage_recomm_ros": 25.46,
            "total_length_m": 0.1,
            "wire_feed_speed": 10.42,
            "weld_speed": 0.0269,
            "arc_length_correction_mm": 6.5,
            "weld_speed_match": "mismatch",
            "weld_speed_ratio_ros_to_doe": 0.67,
            "active_start_msgs": 1,
            "geometry_anomaly_flag": False,
            "progression_anomaly_flag": True,
            "notes": "missing geometry; missing fronius;",
        }
    )
    tables = {
        "/robin/weld_dimensions": pd.DataFrame(
            columns=["timestamp_ns", "bead_id", "progression", "height_mm", "width_mm", "toe_angle_rad"]
        ),
        "/robin/data/fronius": pd.DataFrame(
            columns=["timestamp_ns", "bead_id", "progression", "current", "voltage", "wire_feed_speed", "power", "energy"]
        ),
        "/robin/data/active_bead": pd.DataFrame(
            columns=[
                "timestamp_ns",
                "bead_id",
                "ros_travel_speed_cmd_mps",
                "ros_wire_feed_speed_cmd_mpm",
                "ros_arc_length_correction_cmd_mm",
                "ros_current_recomm_A",
                "ros_voltage_recomm_V",
                "total_length_m",
            ]
        ),
        "/robin/data/progression": pd.DataFrame(
            {
                "timestamp_ns": [110, 120, 130],
                "bead_id": ["B018", "B018", "B018"],
                "progression": [0.2, 0.4, 0.6],
                "is_welding": [True, True, True],
            }
        ),
    }

    row, _ = aggregate_attempt(attempt, tables, cfg)

    assert row["wire_feed_speed_mpm_model_input"] == 10.42
    assert row["travel_speed_mps_model_input"] == 0.0269
    assert row["arc_length_correction_mm_model_input"] == 6.5
    assert row["quality_flag"] == "reject_missing_geometry_samples"
