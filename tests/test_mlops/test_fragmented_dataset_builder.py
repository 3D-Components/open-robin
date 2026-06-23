from __future__ import annotations

import sys
import types
from pathlib import Path

import numpy as np
import pandas as pd

from mlops.fragmented_datasets import build_fragmented_dataset as builder
from mlops.fragmented_datasets.build_fragmented_dataset import (
    Config,
    aggregate_attempt,
    apply_iqr_filter,
    apply_physical_bounds,
    export_bronze_tables,
    filter_stable_window,
    infer_end_times,
    quality_flag_for_attempt,
    robust_series_stats,
    select_recipe_rows,
    selection_report_columns,
    to_float,
    to_str,
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


def test_fragmented_builder_utility_filters_and_exports(tmp_path: Path) -> None:
    cfg = _cfg()
    cfg.min_geometry_samples_per_attempt = 2
    cfg.min_welder_samples_per_attempt = 2

    assert to_float("2.5") == 2.5
    assert np.isnan(to_float("bad"))
    assert to_str(None) == ""
    assert to_str("  bead-1  ") == "bead-1"

    raw = pd.DataFrame(
        {
            "timestamp_ns": [1, 2, 3, 4],
            "progression": [0.05, 0.2, 0.8, 0.95],
            "height_mm": [0.5, 2.0, 30.0, 3.0],
        }
    )
    stable = filter_stable_window(raw, 0.1, 0.9)
    assert stable["timestamp_ns"].tolist() == [2, 3]

    bounded, removed = apply_physical_bounds(
        stable, {"height_mm": (1.0, 10.0), "missing": (0.0, 1.0)}
    )
    assert bounded["height_mm"].tolist() == [2.0]
    assert removed["height_mm"] == 1

    empty_filtered, empty_stats = apply_iqr_filter(
        pd.DataFrame(), ["height_mm"], iqr_multiplier=1.5, min_samples=3
    )
    assert empty_filtered.empty
    assert empty_stats["rows_removed_total"] == 0

    with_outlier = pd.DataFrame({"height_mm": [1.0, 1.1, 1.2, 1.3, 10.0]})
    filtered, stats = apply_iqr_filter(
        with_outlier, ["height_mm", "missing"], iqr_multiplier=1.0, min_samples=3
    )
    assert len(filtered) == 4
    assert stats["rows_removed_by_column"]["height_mm"] == 1

    stats = robust_series_stats(pd.Series([1.0, "bad", 3.0]), "height")
    assert stats["height_mean"] == 2.0
    empty = robust_series_stats(pd.Series(["bad"]), "width")
    assert np.isnan(empty["width_mean"])

    assert quality_flag_for_attempt("", 2, 2, cfg) == "reject_unmapped_recipe"
    assert quality_flag_for_attempt("R1", 0, 2, cfg) == "reject_missing_geometry_samples"
    assert quality_flag_for_attempt("R1", 1, 2, cfg) == "reject_low_geometry_samples"
    assert quality_flag_for_attempt("R1", 2, 0, cfg) == "reject_missing_welder_samples"
    assert quality_flag_for_attempt("R1", 2, 1, cfg) == "reject_low_welder_samples"
    assert quality_flag_for_attempt("R1", 2, 2, cfg) == "ok"

    attempts = pd.DataFrame(
        {
            "bag_file": ["bag.mcap", "bag.mcap"],
            "start_ts": [10, 30],
        }
    )
    inferred = infer_end_times(attempts, {"bag.mcap": 50})
    assert inferred["end_ts_exclusive"].tolist() == [30, 51]

    tables = {
        topic: pd.DataFrame(columns=columns)
        for topic, columns in builder.TOPIC_COLUMNS.items()
    }
    export_bronze_tables("bag", tables, tmp_path)
    assert (tmp_path / "bag__weld_dimensions.csv").exists()

    report_cols = selection_report_columns(
        pd.DataFrame(columns=["doe_input_id", "attempt_id", "ignored"])
    )
    assert report_cols == ["doe_input_id", "attempt_id"]


def test_read_fragmented_topics_decodes_all_known_topics(
    tmp_path: Path, monkeypatch
) -> None:
    bag_path = tmp_path / "bag.mcap"
    bag_path.touch()

    def conn(topic: str):
        return types.SimpleNamespace(topic=topic, msgtype="msg")

    messages = [
        (
            conn("/robin/weld_dimensions"),
            10,
            types.SimpleNamespace(
                bead_id=" B1 ",
                progression=0.2,
                height_mm=2.1,
                width_mm=5.2,
                toe_angle_rad=0.1,
            ),
        ),
        (
            conn("/robin/data/fronius"),
            11,
            types.SimpleNamespace(
                bead_id="B1",
                progression=0.25,
                current=150.0,
                voltage=24.0,
                wire_feed_speed=10.0,
                power=3600.0,
                energy=12.0,
            ),
        ),
        (
            conn("/robin/data/active_bead"),
            12,
            types.SimpleNamespace(
                bead_id="B1",
                weld_speed=0.02,
                wire_feed_speed=10.0,
                arc_length_correction_mm=1.5,
                current_recommvalue=151.0,
                voltage_recommvalue=24.5,
                total_length=0.4,
            ),
        ),
        (
            conn("/robin/data/progression"),
            13,
            types.SimpleNamespace(bead_id="B1", progression=0.3, is_welding=True),
        ),
        (
            conn("/ignored/topic"),
            14,
            types.SimpleNamespace(),
        ),
    ]

    class FakeReader:
        def __init__(self, _paths):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def messages(self):
            return iter(messages)

        def deserialize(self, rawdata, _msgtype):
            return rawdata

    fake_highlevel = types.SimpleNamespace(AnyReader=FakeReader)
    monkeypatch.setitem(sys.modules, "rosbags", types.SimpleNamespace())
    monkeypatch.setitem(sys.modules, "rosbags.highlevel", fake_highlevel)

    tables = builder.read_fragmented_topics(bag_path)

    assert tables["/robin/weld_dimensions"].iloc[0]["bead_id"] == "B1"
    assert tables["/robin/data/fronius"].iloc[0]["current"] == 150.0
    assert (
        tables["/robin/data/active_bead"].iloc[0][
            "ros_arc_length_correction_cmd_mm"
        ]
        == 1.5
    )
    assert bool(tables["/robin/data/progression"].iloc[0]["is_welding"]) is True


def test_fragmented_builder_main_writes_expected_outputs(
    tmp_path: Path, monkeypatch
) -> None:
    source_root = tmp_path / "source"
    source_root.mkdir()
    (source_root / "bag.mcap").touch()

    mapping_csv = tmp_path / "mapping.csv"
    pd.DataFrame(
        [
            {
                "attempt_id": "bag::A001",
                "bag_file": "bag.mcap",
                "attempt_idx_in_bag": 1,
                "start_ts": 100,
                "start_bead_id": "B1",
                "input_id": "R1",
                "wire_feed_speed_cmd_ros": 10.0,
                "weld_speed_cmd_ros": 0.02,
                "arc_length_correction_cmd_ros": 1.0,
                "current_recomm_ros": 150.0,
                "voltage_recomm_ros": 24.0,
                "total_length_m": 0.5,
                "wire_feed_speed": 10.0,
                "weld_speed": 0.02,
                "arc_length_correction_mm": 1.0,
                "weld_speed_match": "exact",
                "weld_speed_ratio_ros_to_doe": 1.0,
                "active_start_msgs": 1,
                "geometry_anomaly_flag": False,
                "progression_anomaly_flag": False,
                "notes": "",
            }
        ]
    ).to_csv(mapping_csv, index=False)

    def samples(column: str, base: float) -> list[float]:
        return [base + idx * 0.01 for idx in range(6)]

    fake_tables = {
        "/robin/weld_dimensions": pd.DataFrame(
            {
                "timestamp_ns": [110, 120, 130, 140, 150, 160],
                "bead_id": ["B1"] * 6,
                "progression": [0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
                "height_mm": samples("height_mm", 2.0),
                "width_mm": samples("width_mm", 5.0),
                "toe_angle_rad": [0.1] * 6,
            }
        ),
        "/robin/data/fronius": pd.DataFrame(
            {
                "timestamp_ns": [110, 120, 130, 140, 150, 160],
                "bead_id": ["B1"] * 6,
                "progression": [0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
                "current": [150, 151, 152, 153, 154, 155],
                "voltage": [24, 24.1, 24.2, 24.3, 24.4, 24.5],
                "wire_feed_speed": [10, 10.1, 10.2, 10.3, 10.4, 10.5],
                "power": [3600, 3610, 3620, 3630, 3640, 3650],
                "energy": [1, 2, 3, 4, 5, 6],
            }
        ),
        "/robin/data/active_bead": pd.DataFrame(
            columns=builder.TOPIC_COLUMNS["/robin/data/active_bead"]
        ),
        "/robin/data/progression": pd.DataFrame(
            {
                "timestamp_ns": [110, 120, 130],
                "bead_id": ["B1"] * 3,
                "progression": [0.2, 0.4, 0.6],
                "is_welding": [True, True, True],
            }
        ),
    }
    monkeypatch.setattr(builder, "read_fragmented_topics", lambda _path: fake_tables)

    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        f"""
dataset:
  dataset_id: fragmented_test
  schema_version: v1.0
  feature_contract_version: corrected_inputs_v1
raw:
  source_root: {source_root}
  mapping_csv_path: {mapping_csv}
outputs:
  bronze_dir: {tmp_path / "bronze"}
  silver_csv: {tmp_path / "silver" / "attempts.csv"}
  gold_csv: {tmp_path / "gold" / "train.csv"}
  selection_report_csv: {tmp_path / "gold" / "selection.csv"}
  qc_report_json: {tmp_path / "gold" / "qc.json"}
  readme_path: {tmp_path / "README.md"}
filters:
  progression_min: 0.1
  progression_max: 0.9
  min_geometry_samples_per_attempt: 2
  min_welder_samples_per_attempt: 2
validation:
  height_mm:
    min: 1.0
    max: 10.0
  width_mm:
    min: 1.0
    max: 20.0
  current_A:
    min: 100
    max: 250
  wire_feed_speed_mpm:
    min: 1
    max: 20
outlier_policy:
  enable_per_attempt_iqr_filter: false
  iqr_multiplier: 1.5
  min_samples_per_attempt: 4
"""
    )
    monkeypatch.setattr(sys, "argv", ["build_fragmented_dataset", "--config", str(config_path)])

    builder.main()

    silver = pd.read_csv(tmp_path / "silver" / "attempts.csv")
    gold = pd.read_csv(tmp_path / "gold" / "train.csv")
    assert silver.iloc[0]["quality_flag"] == "ok"
    assert gold.iloc[0]["selected_for_training"] == np.bool_(True)
    assert (tmp_path / "gold" / "qc.json").exists()
    assert "Fragmented Corrected-Input Dataset" in (tmp_path / "README.md").read_text()
