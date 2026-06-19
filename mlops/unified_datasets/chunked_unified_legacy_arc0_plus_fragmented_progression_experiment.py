from __future__ import annotations

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
import pandas as pd
import torch
from joblib import dump
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from sklearn.model_selection import LeaveOneGroupOut

from mlops.fragmented_datasets.build_fragmented_dataset import (
    TOPIC_COLUMNS as FRAGMENTED_TOPIC_COLUMNS,
    TOPIC_EXPORT_NAMES as FRAGMENTED_TOPIC_EXPORT_NAMES,
    apply_iqr_filter,
    apply_physical_bounds as apply_fragmented_physical_bounds,
    filter_stable_window as filter_fragmented_stable_window,
    load_config as load_fragmented_config,
)
from mlops.fragmented_datasets.evaluate_torch_selected_gold import (
    MODEL_CONFIG,
    fit_fold_lbfgs,
    fit_full_lbfgs,
)
from mlops.weld_mlops.scripts.extract_rosbag_to_csv import (
    apply_per_bead_iqr_filter,
    apply_physical_bounds as apply_legacy_physical_bounds,
    filter_stable_window as filter_legacy_stable_window,
    load_config as load_legacy_config,
    read_rosbag_topics,
)


LEGACY_GOLD_CSV = ROOT / "data" / "gold" / "train_dataset.csv"
FRAGMENTED_GOLD_CSV = ROOT / "data" / "fragmented_doe_corrected" / "gold" / "train_dataset.csv"
FRAGMENTED_BRONZE_DIR = ROOT / "data" / "fragmented_doe_corrected" / "bronze" / "topic_exports"
LEGACY_PIPELINE_CFG = ROOT / "mlops" / "weld_mlops" / "configs" / "pipeline.yaml"
FRAGMENTED_PIPELINE_CFG = ROOT / "mlops" / "fragmented_datasets" / "configs" / "pipeline.yaml"

OUTROOT = ROOT / "data" / "unified_legacy_arc0_plus_fragmented_chunk4_progression"
OUT_GOLD_CSV = OUTROOT / "gold" / "train_dataset.csv"
OUT_COMPONENT_CSV = OUTROOT / "metadata" / "component_rows.csv"
OUT_SUMMARY_JSON = OUTROOT / "metadata" / "summary.json"
OUT_README = OUTROOT / "README.md"

COMPARE_DIR = ROOT / "data" / "comparisons" / "unified_legacy_arc0_plus_fragmented_chunk4_progression_torch_grouped"
COMPARE_METRICS_JSON = COMPARE_DIR / "metrics.json"
COMPARE_CHUNK_PRED_CSV = COMPARE_DIR / "chunk_predictions.csv"
COMPARE_PARENT_PRED_CSV = COMPARE_DIR / "parent_predictions.csv"
COMPARE_README = COMPARE_DIR / "README.md"
MODEL_DIR = ROOT / "data" / "models" / "unified_legacy_arc0_plus_fragmented_chunk4_progression" / "selected_gold_torch_mlp"
MODEL_PATH = MODEL_DIR / "torch_chunked_progression_model.pt"
SCALER_PATH = MODEL_DIR / "torch_chunked_progression_feature_scaler.joblib"
MODEL_METADATA_JSON = MODEL_DIR / "torch_chunked_progression_model_metadata.json"

BASELINE_METRICS_JSON = (
    ROOT / "data" / "comparisons" / "unified_legacy_arc0_plus_fragmented_torch_selected_gold" / "torch_selected_gold_metrics.json"
)

FEATURE_COLUMNS = [
    "wire_feed_speed_mpm_model_input",
    "travel_speed_mps_model_input",
    "arc_length_correction_mm_model_input",
    "chunk_center_progression_model_input",
]
TARGET_COLUMNS = ["height_mm_target", "width_mm_target"]

DATASET_ID = "unified_legacy_arc0_plus_fragmented_chunk4_progression_v1"
FEATURE_CONTRACT_VERSION = "wire_travel_arc_length_progression_chunk4_v1"
SCHEMA_VERSION = "v1.0"
MIN_CHUNK_GEOMETRY_SAMPLES = 5
CHUNKS = [
    (1, 0.10, 0.30, 0.20),
    (2, 0.30, 0.50, 0.40),
    (3, 0.50, 0.70, 0.60),
    (4, 0.70, 0.90, 0.80),
]


def ensure_dirs() -> None:
    (OUTROOT / "gold").mkdir(parents=True, exist_ok=True)
    (OUTROOT / "metadata").mkdir(parents=True, exist_ok=True)
    COMPARE_DIR.mkdir(parents=True, exist_ok=True)
    MODEL_DIR.mkdir(parents=True, exist_ok=True)


def load_fragmented_exported_tables(bag_file: str) -> dict[str, pd.DataFrame]:
    bag_stem = Path(bag_file).stem
    tables: dict[str, pd.DataFrame] = {}
    for topic, export_name in FRAGMENTED_TOPIC_EXPORT_NAMES.items():
        path = FRAGMENTED_BRONZE_DIR / f"{bag_stem}__{export_name}.csv"
        columns = FRAGMENTED_TOPIC_COLUMNS[topic]
        if path.exists():
            df = pd.read_csv(path)
            for col in columns:
                if col not in df.columns:
                    df[col] = np.nan
            if "bead_id" in df.columns:
                df["bead_id"] = df["bead_id"].fillna("").astype(str).str.strip()
            tables[topic] = df[columns].sort_values("timestamp_ns").reset_index(drop=True)
        else:
            tables[topic] = pd.DataFrame(columns=columns)
    return tables


def select_chunk(df: pd.DataFrame, lower: float, upper: float) -> pd.DataFrame:
    if df.empty:
        return df.copy()
    progression = pd.to_numeric(df["progression"], errors="coerce")
    if upper >= 0.90:
        mask = (progression >= lower) & (progression <= upper)
    else:
        mask = (progression >= lower) & (progression < upper)
    return df[mask].copy().reset_index(drop=True)


def chunk_target_stats(df: pd.DataFrame) -> dict[str, float]:
    height = pd.to_numeric(df.get("height_mm", pd.Series(dtype=float)), errors="coerce").dropna()
    width = pd.to_numeric(df.get("width_mm", pd.Series(dtype=float)), errors="coerce").dropna()
    return {
        "height_mm_target": float(height.median()) if not height.empty else np.nan,
        "width_mm_target": float(width.median()) if not width.empty else np.nan,
        "height_mm_std": float(height.std(ddof=0)) if len(height) > 1 else np.nan,
        "width_mm_std": float(width.std(ddof=0)) if len(width) > 1 else np.nan,
        "height_mm_p10": float(height.quantile(0.10)) if not height.empty else np.nan,
        "height_mm_p90": float(height.quantile(0.90)) if not height.empty else np.nan,
        "width_mm_p10": float(width.quantile(0.10)) if not width.empty else np.nan,
        "width_mm_p90": float(width.quantile(0.90)) if not width.empty else np.nan,
    }


def build_legacy_chunk_rows() -> list[dict[str, object]]:
    cfg = load_legacy_config(LEGACY_PIPELINE_CFG)
    legacy_df = pd.read_csv(LEGACY_GOLD_CSV)
    legacy_df = legacy_df[legacy_df["quality_flag"] == "ok"].copy().reset_index(drop=True)
    topic_tables = read_rosbag_topics(cfg.raw_bag_path)

    geom_stable = filter_legacy_stable_window(
        topic_tables["/robin/weld_dimensions"], cfg.progression_min, cfg.progression_max
    )
    geom_bounded, _ = apply_legacy_physical_bounds(
        geom_stable,
        {
            "height_mm": (cfg.height_min_mm, cfg.height_max_mm),
            "width_mm": (cfg.width_min_mm, cfg.width_max_mm),
        },
    )
    if cfg.enable_per_bead_iqr_filter:
        geom_clean, _ = apply_per_bead_iqr_filter(
            geom_bounded,
            value_columns=["height_mm", "width_mm"],
            bead_col="bead_id",
            iqr_multiplier=cfg.iqr_multiplier,
            min_samples_per_bead=cfg.iqr_min_samples_per_bead,
        )
    else:
        geom_clean = geom_bounded

    rows: list[dict[str, object]] = []
    for bead_row in legacy_df.itertuples(index=False):
        bead_id = str(bead_row.bead_id)
        bead_geom = geom_clean[geom_clean["bead_id"] == bead_id].copy()
        if bead_geom.empty:
            continue
        parent_group_id = f"legacy::{bead_id}"
        for chunk_index, lower, upper, center in CHUNKS:
            chunk_geom = select_chunk(bead_geom, lower, upper)
            if len(chunk_geom) < MIN_CHUNK_GEOMETRY_SAMPLES:
                continue
            stats = chunk_target_stats(chunk_geom)
            rows.append(
                {
                    "schema_version": SCHEMA_VERSION,
                    "feature_contract_version": FEATURE_CONTRACT_VERSION,
                    "dataset_id": DATASET_ID,
                    "quality_flag": "ok",
                    "parent_group_id": parent_group_id,
                    "source_dataset": "legacy_robin_data_01_arc0_assumed",
                    "source_record_id": parent_group_id,
                    "source_recipe_id": bead_id,
                    "chunk_index": chunk_index,
                    "chunk_id": f"{parent_group_id}::chunk{chunk_index}",
                    "chunk_progression_min": lower,
                    "chunk_progression_max": upper,
                    "chunk_center_progression_model_input": center,
                    "chunk_geometry_samples": int(len(chunk_geom)),
                    "wire_feed_speed_mpm_model_input": float(bead_row.wire_speed_mpm_model_input),
                    "travel_speed_mps_model_input": float(bead_row.target_travel_speed_mps),
                    "arc_length_correction_mm_model_input": 0.0,
                    "parent_height_mm_target": float(bead_row.height_mm_target),
                    "parent_width_mm_target": float(bead_row.width_mm_target),
                    **stats,
                }
            )
    return rows


def build_fragmented_chunk_rows() -> list[dict[str, object]]:
    cfg = load_fragmented_config(FRAGMENTED_PIPELINE_CFG)
    fragmented_df = pd.read_csv(FRAGMENTED_GOLD_CSV)
    fragmented_df = fragmented_df[fragmented_df["quality_flag"] == "ok"].copy().reset_index(drop=True)

    tables_by_bag: dict[str, dict[str, pd.DataFrame]] = {}
    rows: list[dict[str, object]] = []
    for attempt_row in fragmented_df.itertuples(index=False):
        bag_file = str(attempt_row.bag_file)
        if bag_file not in tables_by_bag:
            tables_by_bag[bag_file] = load_fragmented_exported_tables(bag_file)
        tables = tables_by_bag[bag_file]

        start_ts = int(attempt_row.start_time_ns)
        end_ts_exclusive = int(attempt_row.end_time_ns) + 1
        bead_id = str(attempt_row.bead_id)

        geom_raw = tables["/robin/weld_dimensions"]
        geom_raw = geom_raw[
            (geom_raw["bead_id"] == bead_id)
            & (geom_raw["timestamp_ns"] >= start_ts)
            & (geom_raw["timestamp_ns"] < end_ts_exclusive)
        ].copy()
        geom_stable = filter_fragmented_stable_window(geom_raw, cfg.progression_min, cfg.progression_max)
        geom_bounded, _ = apply_fragmented_physical_bounds(
            geom_stable,
            {
                "height_mm": (cfg.height_min_mm, cfg.height_max_mm),
                "width_mm": (cfg.width_min_mm, cfg.width_max_mm),
            },
        )
        if cfg.enable_per_attempt_iqr_filter:
            geom_clean, _ = apply_iqr_filter(
                geom_bounded,
                value_columns=["height_mm", "width_mm"],
                iqr_multiplier=cfg.iqr_multiplier,
                min_samples=cfg.iqr_min_samples_per_attempt,
            )
        else:
            geom_clean = geom_bounded

        if geom_clean.empty:
            continue

        parent_group_id = f"fragmented::{attempt_row.attempt_id}"
        for chunk_index, lower, upper, center in CHUNKS:
            chunk_geom = select_chunk(geom_clean, lower, upper)
            if len(chunk_geom) < MIN_CHUNK_GEOMETRY_SAMPLES:
                continue
            stats = chunk_target_stats(chunk_geom)
            rows.append(
                {
                    "schema_version": SCHEMA_VERSION,
                    "feature_contract_version": FEATURE_CONTRACT_VERSION,
                    "dataset_id": DATASET_ID,
                    "quality_flag": "ok",
                    "parent_group_id": parent_group_id,
                    "source_dataset": "fragmented_doe_corrected_selected_gold",
                    "source_record_id": parent_group_id,
                    "source_recipe_id": str(attempt_row.doe_input_id),
                    "chunk_index": chunk_index,
                    "chunk_id": f"{parent_group_id}::chunk{chunk_index}",
                    "chunk_progression_min": lower,
                    "chunk_progression_max": upper,
                    "chunk_center_progression_model_input": center,
                    "chunk_geometry_samples": int(len(chunk_geom)),
                    "wire_feed_speed_mpm_model_input": float(attempt_row.wire_feed_speed_mpm_model_input),
                    "travel_speed_mps_model_input": float(attempt_row.travel_speed_mps_model_input),
                    "arc_length_correction_mm_model_input": float(attempt_row.arc_length_correction_mm_model_input),
                    "parent_height_mm_target": float(attempt_row.height_mm_target),
                    "parent_width_mm_target": float(attempt_row.width_mm_target),
                    **stats,
                }
            )
    return rows


def build_chunked_dataset() -> pd.DataFrame:
    chunk_df = pd.DataFrame(build_legacy_chunk_rows() + build_fragmented_chunk_rows())
    if chunk_df.empty:
        raise ValueError("Chunked progression dataset is empty.")
    chunk_df = chunk_df.sort_values(["source_dataset", "source_recipe_id", "chunk_index"]).reset_index(drop=True)
    chunk_df.to_csv(OUT_COMPONENT_CSV, index=False)
    chunk_df.to_csv(OUT_GOLD_CSV, index=False)
    summary = {
        "dataset_id": DATASET_ID,
        "feature_contract_version": FEATURE_CONTRACT_VERSION,
        "n_rows": int(len(chunk_df)),
        "n_parent_groups": int(chunk_df["parent_group_id"].nunique()),
        "n_legacy_rows": int((chunk_df["source_dataset"] == "legacy_robin_data_01_arc0_assumed").sum()),
        "n_fragmented_rows": int((chunk_df["source_dataset"] == "fragmented_doe_corrected_selected_gold").sum()),
        "min_chunk_geometry_samples": MIN_CHUNK_GEOMETRY_SAMPLES,
        "chunks": CHUNKS,
        "feature_columns": FEATURE_COLUMNS,
        "target_columns": TARGET_COLUMNS,
    }
    OUT_SUMMARY_JSON.write_text(json.dumps(summary, indent=2))
    OUT_README.write_text(
        "\n".join(
            [
                "# Chunked Unified Legacy Arc0 Plus Fragmented Dataset With Progression",
                "",
                "This is an experimental dataset.",
                "Each original selected-gold row is split into 4 cleaned subwindows and gets an explicit center-progression model input.",
                "",
                "- 10% to 30% with center progression `0.20`",
                "- 30% to 50% with center progression `0.40`",
                "- 50% to 70% with center progression `0.60`",
                "- 70% to 90% with center progression `0.80`",
                "",
                f"- total chunk rows: `{summary['n_rows']}`",
                f"- parent groups: `{summary['n_parent_groups']}`",
                f"- legacy chunk rows: `{summary['n_legacy_rows']}`",
                f"- fragmented chunk rows: `{summary['n_fragmented_rows']}`",
            ]
        )
    )
    return chunk_df


def evaluate_grouped_logo(chunk_df: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame, dict[str, object]]:
    model_df = chunk_df.dropna(subset=FEATURE_COLUMNS + TARGET_COLUMNS).reset_index(drop=True)
    X = model_df[FEATURE_COLUMNS].to_numpy(dtype=np.float32)
    y = model_df[TARGET_COLUMNS].to_numpy(dtype=np.float32)
    groups = model_df["parent_group_id"].astype(str).to_numpy()
    preds = np.zeros_like(y, dtype=np.float32)

    logo = LeaveOneGroupOut()
    for train_idx, test_idx in logo.split(X, y, groups):
        preds[test_idx] = fit_fold_lbfgs(
            X[train_idx],
            y[train_idx],
            X[test_idx],
            seed=int(MODEL_CONFIG["random_seed"]),
            hidden_dims=list(MODEL_CONFIG["hidden_dims"]),
            weight_decay=float(MODEL_CONFIG["weight_decay"]),
            max_iter=int(MODEL_CONFIG["max_iter"]),
        )

    pred_df = model_df[
        [
            "parent_group_id",
            "source_dataset",
            "source_recipe_id",
            "chunk_index",
            "chunk_id",
            "chunk_center_progression_model_input",
            "chunk_geometry_samples",
            "parent_height_mm_target",
            "parent_width_mm_target",
        ]
    ].copy()
    for idx, target_name in enumerate(TARGET_COLUMNS):
        pred_df[f"{target_name}_actual"] = y[:, idx]
        pred_df[f"{target_name}_pred"] = preds[:, idx]
        pred_df[f"{target_name}_abs_error"] = np.abs(preds[:, idx] - y[:, idx])

    parent_df = (
        pred_df.groupby("parent_group_id", as_index=False)
        .agg(
            source_dataset=("source_dataset", "first"),
            source_recipe_id=("source_recipe_id", "first"),
            n_chunks=("chunk_index", "size"),
            height_mm_target_parent_actual=("parent_height_mm_target", "first"),
            width_mm_target_parent_actual=("parent_width_mm_target", "first"),
            height_mm_target_parent_pred_from_chunk_median=("height_mm_target_pred", "median"),
            width_mm_target_parent_pred_from_chunk_median=("width_mm_target_pred", "median"),
            height_mm_target_parent_pred_from_chunk_mean=("height_mm_target_pred", "mean"),
            width_mm_target_parent_pred_from_chunk_mean=("width_mm_target_pred", "mean"),
        )
        .sort_values("parent_group_id")
        .reset_index(drop=True)
    )

    chunk_metrics = {
        "n_chunk_rows": int(len(pred_df)),
        "n_parent_groups": int(parent_df["parent_group_id"].nunique()),
        "r2_variance_weighted_chunk_level": float(r2_score(y, preds, multioutput="variance_weighted")),
        "height_chunk_level_r2": float(r2_score(y[:, 0], preds[:, 0])),
        "width_chunk_level_r2": float(r2_score(y[:, 1], preds[:, 1])),
        "height_chunk_level_rmse": float(np.sqrt(mean_squared_error(y[:, 0], preds[:, 0]))),
        "width_chunk_level_rmse": float(np.sqrt(mean_squared_error(y[:, 1], preds[:, 1]))),
    }

    parent_actual = parent_df[
        ["height_mm_target_parent_actual", "width_mm_target_parent_actual"]
    ].to_numpy(dtype=float)
    parent_pred_median = parent_df[
        [
            "height_mm_target_parent_pred_from_chunk_median",
            "width_mm_target_parent_pred_from_chunk_median",
        ]
    ].to_numpy(dtype=float)
    parent_pred_mean = parent_df[
        [
            "height_mm_target_parent_pred_from_chunk_mean",
            "width_mm_target_parent_pred_from_chunk_mean",
        ]
    ].to_numpy(dtype=float)

    metrics = {
        "dataset_csv": str(OUT_GOLD_CSV),
        "feature_columns": FEATURE_COLUMNS,
        "target_columns": TARGET_COLUMNS,
        "model": MODEL_CONFIG,
        "grouping": "LeaveOneGroupOut on parent_group_id",
        "chunking": CHUNKS,
        "chunk_dataset": chunk_metrics,
        "parent_reconstruction_from_chunk_median": {
            "r2_variance_weighted": float(r2_score(parent_actual, parent_pred_median, multioutput="variance_weighted")),
            "height_r2": float(r2_score(parent_actual[:, 0], parent_pred_median[:, 0])),
            "width_r2": float(r2_score(parent_actual[:, 1], parent_pred_median[:, 1])),
            "height_rmse": float(np.sqrt(mean_squared_error(parent_actual[:, 0], parent_pred_median[:, 0]))),
            "width_rmse": float(np.sqrt(mean_squared_error(parent_actual[:, 1], parent_pred_median[:, 1]))),
            "height_mae": float(mean_absolute_error(parent_actual[:, 0], parent_pred_median[:, 0])),
            "width_mae": float(mean_absolute_error(parent_actual[:, 1], parent_pred_median[:, 1])),
        },
        "parent_reconstruction_from_chunk_mean": {
            "r2_variance_weighted": float(r2_score(parent_actual, parent_pred_mean, multioutput="variance_weighted")),
            "height_r2": float(r2_score(parent_actual[:, 0], parent_pred_mean[:, 0])),
            "width_r2": float(r2_score(parent_actual[:, 1], parent_pred_mean[:, 1])),
            "height_rmse": float(np.sqrt(mean_squared_error(parent_actual[:, 0], parent_pred_mean[:, 0]))),
            "width_rmse": float(np.sqrt(mean_squared_error(parent_actual[:, 1], parent_pred_mean[:, 1]))),
            "height_mae": float(mean_absolute_error(parent_actual[:, 0], parent_pred_mean[:, 0])),
            "width_mae": float(mean_absolute_error(parent_actual[:, 1], parent_pred_mean[:, 1])),
        },
    }
    return pred_df, parent_df, metrics


def save_refit_artifact(chunk_df: pd.DataFrame, metrics: dict[str, object]) -> dict[str, object]:
    model_df = chunk_df.dropna(subset=FEATURE_COLUMNS + TARGET_COLUMNS).reset_index(drop=True)
    X = model_df[FEATURE_COLUMNS].to_numpy(dtype=np.float32)
    y = model_df[TARGET_COLUMNS].to_numpy(dtype=np.float32)

    model, scaler = fit_full_lbfgs(
        X,
        y,
        seed=int(MODEL_CONFIG["random_seed"]),
        hidden_dims=list(MODEL_CONFIG["hidden_dims"]),
        weight_decay=float(MODEL_CONFIG["weight_decay"]),
        max_iter=int(MODEL_CONFIG["max_iter"]),
    )

    X_scaled = scaler.transform(X).astype(np.float32)
    with torch.no_grad():
        pred = model(torch.from_numpy(X_scaled)).cpu().numpy()

    dump(scaler, SCALER_PATH)
    torch.save(
        {
            "schema": "unified_legacy_arc0_plus_fragmented_chunk4_progression_torch_mlp_v1",
            "state_dict": model.state_dict(),
            "feature_columns": FEATURE_COLUMNS,
            "target_columns": TARGET_COLUMNS,
            "hidden_dims": MODEL_CONFIG["hidden_dims"],
            "optimizer": MODEL_CONFIG["optimizer"],
            "weight_decay": MODEL_CONFIG["weight_decay"],
            "max_iter": MODEL_CONFIG["max_iter"],
            "random_seed": MODEL_CONFIG["random_seed"],
            "input_dim": int(X.shape[1]),
            "output_dim": int(y.shape[1]),
            "dataset_csv": str(OUT_GOLD_CSV),
            "n_rows_refit": int(len(model_df)),
            "scaler_path": str(SCALER_PATH),
            "benchmark_grouping": metrics["grouping"],
            "benchmark_parent_mean_r2_variance_weighted": metrics["parent_reconstruction_from_chunk_mean"]["r2_variance_weighted"],
            "benchmark_parent_median_r2_variance_weighted": metrics["parent_reconstruction_from_chunk_median"]["r2_variance_weighted"],
        },
        MODEL_PATH,
    )

    artifact_summary = {
        "model_path": str(MODEL_PATH),
        "scaler_path": str(SCALER_PATH),
        "metadata_json": str(MODEL_METADATA_JSON),
        "refit_note": "Saved artifact is refit on all chunk rows; benchmark metrics remain the grouped LeaveOneGroupOut scores above.",
        "refit_train_metrics": {
            "n_rows": int(len(model_df)),
            "r2_variance_weighted_train": float(r2_score(y, pred, multioutput="variance_weighted")),
            "targets": {
                "height_mm_target": {
                    "r2_train": float(r2_score(y[:, 0], pred[:, 0])),
                    "rmse_train": float(np.sqrt(mean_squared_error(y[:, 0], pred[:, 0]))),
                    "mae_train": float(mean_absolute_error(y[:, 0], pred[:, 0])),
                },
                "width_mm_target": {
                    "r2_train": float(r2_score(y[:, 1], pred[:, 1])),
                    "rmse_train": float(np.sqrt(mean_squared_error(y[:, 1], pred[:, 1]))),
                    "mae_train": float(mean_absolute_error(y[:, 1], pred[:, 1])),
                },
            },
        },
    }
    MODEL_METADATA_JSON.write_text(json.dumps(artifact_summary, indent=2))
    return artifact_summary


def write_evaluation_outputs(pred_df: pd.DataFrame, parent_df: pd.DataFrame, metrics: dict[str, object]) -> None:
    pred_df.to_csv(COMPARE_CHUNK_PRED_CSV, index=False)
    parent_df.to_csv(COMPARE_PARENT_PRED_CSV, index=False)
    COMPARE_METRICS_JSON.write_text(json.dumps(metrics, indent=2))

    baseline_lines: list[str] = []
    if BASELINE_METRICS_JSON.exists():
        baseline = json.loads(BASELINE_METRICS_JSON.read_text())
        baseline_lines = [
            "",
            "Baseline comparison against the current unified non-chunked selected-gold dataset:",
            f"- baseline variance-weighted R2: `{baseline['r2_variance_weighted']:.4f}`",
            f"- chunked-with-progression parent reconstruction (median) variance-weighted R2: `{metrics['parent_reconstruction_from_chunk_median']['r2_variance_weighted']:.4f}`",
            f"- chunked-with-progression parent reconstruction (mean) variance-weighted R2: `{metrics['parent_reconstruction_from_chunk_mean']['r2_variance_weighted']:.4f}`",
        ]

    COMPARE_README.write_text(
        "\n".join(
            [
                "# Chunked Unified Dataset With Progression Torch Evaluation",
                "",
                "This is an experimental grouped evaluation.",
                "Each original bead/attempt is split into 4 chunk rows, and each chunk row gets a center-progression model input.",
                "All folds use `LeaveOneGroupOut` on `parent_group_id`, so all chunks from the same original bead stay together.",
                "",
                f"- chunk rows: `{metrics['chunk_dataset']['n_chunk_rows']}`",
                f"- parent groups: `{metrics['chunk_dataset']['n_parent_groups']}`",
                f"- chunk-level variance-weighted R2: `{metrics['chunk_dataset']['r2_variance_weighted_chunk_level']:.4f}`",
                f"- parent reconstruction from chunk median R2: `{metrics['parent_reconstruction_from_chunk_median']['r2_variance_weighted']:.4f}`",
                f"- parent reconstruction from chunk mean R2: `{metrics['parent_reconstruction_from_chunk_mean']['r2_variance_weighted']:.4f}`",
                *baseline_lines,
                "",
                "Saved artifact:",
                f"- model: `{metrics['saved_model_artifact']['model_path']}`",
                f"- scaler: `{metrics['saved_model_artifact']['scaler_path']}`",
                f"- metadata: `{metrics['saved_model_artifact']['metadata_json']}`",
            ]
        )
    )


def main() -> None:
    ensure_dirs()
    chunk_df = build_chunked_dataset()
    pred_df, parent_df, metrics = evaluate_grouped_logo(chunk_df)
    metrics["saved_model_artifact"] = save_refit_artifact(chunk_df, metrics)
    write_evaluation_outputs(pred_df, parent_df, metrics)
    print(f"Wrote {OUT_GOLD_CSV}")
    print(f"Wrote {OUT_COMPONENT_CSV}")
    print(f"Wrote {OUT_SUMMARY_JSON}")
    print(f"Wrote {OUT_README}")
    print(f"Wrote {COMPARE_METRICS_JSON}")
    print(f"Wrote {COMPARE_CHUNK_PRED_CSV}")
    print(f"Wrote {COMPARE_PARENT_PRED_CSV}")
    print(f"Wrote {COMPARE_README}")
    print(f"Wrote {MODEL_PATH}")
    print(f"Wrote {SCALER_PATH}")
    print(f"Wrote {MODEL_METADATA_JSON}")


if __name__ == "__main__":
    main()
