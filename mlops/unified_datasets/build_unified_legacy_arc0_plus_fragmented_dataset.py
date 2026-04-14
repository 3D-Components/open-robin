from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]

LEGACY_GOLD_CSV = ROOT / "data" / "gold" / "train_dataset.csv"
FRAGMENTED_GOLD_CSV = ROOT / "data" / "fragmented_doe_corrected" / "gold" / "train_dataset.csv"
FRAGMENTED_ATTEMPT_CSV = ROOT / "data" / "fragmented_doe_corrected" / "silver" / "bead_level" / "attempt_level_dataset.csv"

OUTROOT = ROOT / "data" / "unified_legacy_arc0_plus_fragmented"
OUT_GOLD_CSV = OUTROOT / "gold" / "train_dataset.csv"
OUT_COMPONENT_CSV = OUTROOT / "metadata" / "component_rows.csv"
OUT_AGG_CSV = OUTROOT / "metadata" / "aggregation_report.csv"
OUT_SUMMARY_JSON = OUTROOT / "metadata" / "summary.json"
OUT_README = OUTROOT / "README.md"

FEATURE_COLUMNS = [
    "wire_feed_speed_mpm_model_input",
    "travel_speed_mps_model_input",
    "arc_length_correction_mm_model_input",
]
TARGET_COLUMNS = ["height_mm_target", "width_mm_target"]

DATASET_ID = "unified_legacy_arc0_plus_fragmented_v1"
FEATURE_CONTRACT_VERSION = "wire_travel_arc_length_v1"
SCHEMA_VERSION = "v1.0"
LEGACY_SOURCE_NAME = "legacy_robin_data_01_arc0_assumed"
FRAGMENTED_SOURCE_NAME = "fragmented_doe_corrected_selected_gold"

ROUNDING = {
    "wire_feed_speed_mpm_model_input": 6,
    "travel_speed_mps_model_input": 5,
    "arc_length_correction_mm_model_input": 3,
}


def _ensure_dirs() -> None:
    (OUTROOT / "gold").mkdir(parents=True, exist_ok=True)
    (OUTROOT / "metadata").mkdir(parents=True, exist_ok=True)


def _round_features(df: pd.DataFrame) -> pd.DataFrame:
    for column, digits in ROUNDING.items():
        df[column] = df[column].astype(float).round(digits)
    return df


def load_legacy_rows() -> pd.DataFrame:
    legacy_df = pd.read_csv(LEGACY_GOLD_CSV)
    legacy_df = legacy_df[legacy_df["quality_flag"] == "ok"].copy()
    legacy_df = legacy_df.dropna(
        subset=["wire_speed_mpm_model_input", "target_travel_speed_mps", "height_mm_target", "width_mm_target"]
    ).reset_index(drop=True)

    out = pd.DataFrame(
        {
            "source_dataset": LEGACY_SOURCE_NAME,
            "source_run_id": legacy_df["run_id"].astype(str),
            "source_recipe_id": legacy_df["bead_id"].astype(str),
            "source_record_id": "legacy::" + legacy_df["bead_id"].astype(str),
            "component_quality_flag": legacy_df["quality_flag"].astype(str),
            "wire_feed_speed_mpm_model_input": legacy_df["wire_speed_mpm_model_input"].astype(float),
            "travel_speed_mps_model_input": legacy_df["target_travel_speed_mps"].astype(float),
            "arc_length_correction_mm_model_input": 0.0,
            "height_mm_target": legacy_df["height_mm_target"].astype(float),
            "width_mm_target": legacy_df["width_mm_target"].astype(float),
            "height_mm_std": legacy_df["height_mm_std"].astype(float),
            "width_mm_std": legacy_df["width_mm_std"].astype(float),
            "height_mm_p10": legacy_df["height_mm_p10"].astype(float),
            "height_mm_p90": legacy_df["height_mm_p90"].astype(float),
            "width_mm_p10": legacy_df["width_mm_p10"].astype(float),
            "width_mm_p90": legacy_df["width_mm_p90"].astype(float),
            "n_geometry_samples": legacy_df["n_geometry_samples"].astype(int),
            "n_welder_samples": legacy_df["n_welder_samples"].astype(int),
            "wire_feature_origin": legacy_df["wire_speed_source"].astype(str),
            "travel_feature_origin": "legacy_target_travel_speed",
            "arc_feature_origin": "assumed_zero_arc_length_correction",
            "legacy_target_current_A": legacy_df["target_current_A"].astype(float),
            "legacy_stickout_mm_aux": legacy_df["stickout_mm_model_input"].astype(float),
            "support_count_within_source_recipe": 1,
            "unification_note": "Legacy row mapped to unified contract with arc_length_correction_mm_model_input=0.0.",
        }
    )
    return _round_features(out)


def load_fragmented_rows() -> pd.DataFrame:
    fragmented_df = pd.read_csv(FRAGMENTED_GOLD_CSV)
    fragmented_df = fragmented_df[fragmented_df["quality_flag"] == "ok"].copy()
    fragmented_df = fragmented_df.dropna(subset=FEATURE_COLUMNS + TARGET_COLUMNS).reset_index(drop=True)

    attempt_df = pd.read_csv(FRAGMENTED_ATTEMPT_CSV)
    attempt_df = attempt_df[attempt_df["quality_flag"] == "ok"].copy()
    support_counts = attempt_df.groupby("doe_input_id").size().rename("support_count_within_source_recipe")

    out = pd.DataFrame(
        {
            "source_dataset": FRAGMENTED_SOURCE_NAME,
            "source_run_id": fragmented_df["run_id"].astype(str),
            "source_recipe_id": fragmented_df["doe_input_id"].astype(str),
            "source_record_id": "fragmented::" + fragmented_df["attempt_id"].astype(str),
            "component_quality_flag": fragmented_df["quality_flag"].astype(str),
            "wire_feed_speed_mpm_model_input": fragmented_df["wire_feed_speed_mpm_model_input"].astype(float),
            "travel_speed_mps_model_input": fragmented_df["travel_speed_mps_model_input"].astype(float),
            "arc_length_correction_mm_model_input": fragmented_df["arc_length_correction_mm_model_input"].astype(float),
            "height_mm_target": fragmented_df["height_mm_target"].astype(float),
            "width_mm_target": fragmented_df["width_mm_target"].astype(float),
            "height_mm_std": fragmented_df["height_mm_std"].astype(float),
            "width_mm_std": fragmented_df["width_mm_std"].astype(float),
            "height_mm_p10": fragmented_df["height_mm_p10"].astype(float),
            "height_mm_p90": fragmented_df["height_mm_p90"].astype(float),
            "width_mm_p10": fragmented_df["width_mm_p10"].astype(float),
            "width_mm_p90": fragmented_df["width_mm_p90"].astype(float),
            "n_geometry_samples": fragmented_df["n_geometry_samples"].astype(int),
            "n_welder_samples": fragmented_df["n_welder_samples"].astype(int),
            "wire_feature_origin": fragmented_df["input_source"].astype(str),
            "travel_feature_origin": fragmented_df["input_source"].astype(str),
            "arc_feature_origin": fragmented_df["input_source"].astype(str),
            "legacy_target_current_A": np.nan,
            "legacy_stickout_mm_aux": np.nan,
            "support_count_within_source_recipe": fragmented_df["doe_input_id"].map(support_counts).fillna(1).astype(int),
            "unification_note": "Fragmented corrected-input selected-gold row kept on the canonical unified feature contract.",
        }
    )
    return _round_features(out)


def build_unified_dataset(component_df: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    group_cols = FEATURE_COLUMNS

    agg_df = (
        component_df.groupby(group_cols, dropna=False, as_index=False)
        .agg(
            source_row_count=("source_record_id", "size"),
            legacy_row_count=("source_dataset", lambda values: int((values == LEGACY_SOURCE_NAME).sum())),
            fragmented_row_count=("source_dataset", lambda values: int((values == FRAGMENTED_SOURCE_NAME).sum())),
            source_dataset_count=("source_dataset", lambda values: int(pd.Series(values).nunique())),
            source_datasets=("source_dataset", lambda values: "|".join(sorted(pd.Series(values).unique()))),
            source_record_ids=("source_record_id", lambda values: "|".join(sorted(pd.Series(values).astype(str)))),
            source_recipe_ids=("source_recipe_id", lambda values: "|".join(sorted(pd.Series(values).astype(str).unique()))),
            height_mm_target=("height_mm_target", "mean"),
            width_mm_target=("width_mm_target", "mean"),
            height_mm_support_std_across_rows=("height_mm_target", "std"),
            width_mm_support_std_across_rows=("width_mm_target", "std"),
            height_mm_p10_across_rows=("height_mm_target", lambda values: float(pd.Series(values).quantile(0.10))),
            height_mm_p90_across_rows=("height_mm_target", lambda values: float(pd.Series(values).quantile(0.90))),
            width_mm_p10_across_rows=("width_mm_target", lambda values: float(pd.Series(values).quantile(0.10))),
            width_mm_p90_across_rows=("width_mm_target", lambda values: float(pd.Series(values).quantile(0.90))),
            mean_component_height_std=("height_mm_std", "mean"),
            mean_component_width_std=("width_mm_std", "mean"),
            n_geometry_samples_total=("n_geometry_samples", "sum"),
            n_welder_samples_total=("n_welder_samples", "sum"),
            max_support_count_within_source_recipe=("support_count_within_source_recipe", "max"),
            unification_notes=("unification_note", lambda values: "|".join(sorted(pd.Series(values).unique()))),
        )
    )

    agg_df["height_mm_support_std_across_rows"] = agg_df["height_mm_support_std_across_rows"].fillna(0.0)
    agg_df["width_mm_support_std_across_rows"] = agg_df["width_mm_support_std_across_rows"].fillna(0.0)
    agg_df = agg_df.sort_values(group_cols).reset_index(drop=True)

    train_df = agg_df.copy()
    train_df.insert(0, "schema_version", SCHEMA_VERSION)
    train_df.insert(1, "feature_contract_version", FEATURE_CONTRACT_VERSION)
    train_df.insert(2, "dataset_id", DATASET_ID)
    train_df.insert(3, "quality_flag", "ok")
    train_df.insert(4, "doe_input_id", [f"UNIFIED-{idx:03d}" for idx in range(1, len(train_df) + 1)])
    train_df.insert(
        5,
        "attempt_id",
        train_df["doe_input_id"].map(lambda value: f"{DATASET_ID}::{value}"),
    )
    return train_df, agg_df


def write_outputs(component_df: pd.DataFrame, train_df: pd.DataFrame, agg_df: pd.DataFrame) -> None:
    component_df.to_csv(OUT_COMPONENT_CSV, index=False)
    train_df.to_csv(OUT_GOLD_CSV, index=False)
    agg_df.to_csv(OUT_AGG_CSV, index=False)

    summary = {
        "dataset_id": DATASET_ID,
        "feature_contract_version": FEATURE_CONTRACT_VERSION,
        "legacy_source_rows": int((component_df["source_dataset"] == LEGACY_SOURCE_NAME).sum()),
        "fragmented_source_rows": int((component_df["source_dataset"] == FRAGMENTED_SOURCE_NAME).sum()),
        "component_rows_total": int(len(component_df)),
        "unified_rows_total": int(len(train_df)),
        "collapsed_duplicates": int(len(component_df) - len(train_df)),
        "feature_columns": FEATURE_COLUMNS,
        "target_columns": TARGET_COLUMNS,
    }
    OUT_SUMMARY_JSON.write_text(json.dumps(summary, indent=2))

    text = f"""# Unified Legacy Arc0 Plus Fragmented Dataset

This dataset merges:

- `data/gold/train_dataset.csv` from `robin_data_01`, mapped onto the unified feature contract by setting `arc_length_correction_mm_model_input = 0.0`
- `data/fragmented_doe_corrected/gold/train_dataset.csv` from the corrected fragmented DOE pipeline

Important caveat for the legacy component:

- wire feed speed comes from `wire_speed_mpm_model_input`, which is a measured fallback in the legacy dataset
- travel speed comes from `target_travel_speed_mps`
- arc length correction is assumed to be `0.0` for all legacy rows

Output files:

- `gold/train_dataset.csv`: final unified training dataset on the 3-input contract
- `metadata/component_rows.csv`: source rows before any duplicate collapse
- `metadata/aggregation_report.csv`: per-feature-triple aggregation summary
- `metadata/summary.json`: compact counts

Counts:

- legacy source rows: {summary['legacy_source_rows']}
- fragmented source rows: {summary['fragmented_source_rows']}
- total component rows: {summary['component_rows_total']}
- unified final rows: {summary['unified_rows_total']}
- collapsed duplicate rows: {summary['collapsed_duplicates']}
"""
    OUT_README.write_text(text)


def main() -> None:
    _ensure_dirs()
    component_df = pd.concat([load_legacy_rows(), load_fragmented_rows()], ignore_index=True)
    train_df, agg_df = build_unified_dataset(component_df)
    write_outputs(component_df, train_df, agg_df)

    print(f"Wrote {OUT_COMPONENT_CSV}")
    print(f"Wrote {OUT_AGG_CSV}")
    print(f"Wrote {OUT_GOLD_CSV}")
    print(f"Wrote {OUT_SUMMARY_JSON}")
    print(f"Wrote {OUT_README}")


if __name__ == "__main__":
    main()
