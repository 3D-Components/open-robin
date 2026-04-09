from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml


TOPIC_COLUMNS: dict[str, list[str]] = {
    "/robin/weld_dimensions": [
        "timestamp_ns",
        "bead_id",
        "progression",
        "height_mm",
        "width_mm",
        "toe_angle_rad",
    ],
    "/robin/data/fronius": [
        "timestamp_ns",
        "bead_id",
        "progression",
        "current",
        "voltage",
        "wire_feed_speed",
        "power",
        "energy",
    ],
    "/robin/data/active_bead": [
        "timestamp_ns",
        "bead_id",
        "ros_travel_speed_cmd_mps",
        "ros_wire_feed_speed_cmd_mpm",
        "ros_arc_length_correction_cmd_mm",
        "ros_current_recomm_A",
        "ros_voltage_recomm_V",
        "total_length_m",
    ],
    "/robin/data/progression": [
        "timestamp_ns",
        "bead_id",
        "progression",
        "is_welding",
    ],
}

TOPIC_EXPORT_NAMES = {
    "/robin/weld_dimensions": "weld_dimensions",
    "/robin/data/fronius": "fronius",
    "/robin/data/active_bead": "active_bead",
    "/robin/data/progression": "progression",
}


@dataclass
class Config:
    dataset_id: str
    schema_version: str
    feature_contract_version: str
    source_root: Path
    mapping_csv_path: Path
    bronze_dir: Path
    silver_csv: Path
    gold_csv: Path
    selection_report_csv: Path
    qc_report_json: Path
    readme_path: Path
    progression_min: float = 0.10
    progression_max: float = 0.90
    min_geometry_samples_per_attempt: int = 25
    min_welder_samples_per_attempt: int = 10
    height_min_mm: float | None = None
    height_max_mm: float | None = None
    width_min_mm: float | None = None
    width_max_mm: float | None = None
    current_min_A: float | None = None
    current_max_A: float | None = None
    wire_feed_speed_min_mpm: float | None = None
    wire_feed_speed_max_mpm: float | None = None
    enable_per_attempt_iqr_filter: bool = True
    iqr_multiplier: float = 1.5
    iqr_min_samples_per_attempt: int = 20


def load_config(path: Path) -> Config:
    data = yaml.safe_load(path.read_text())
    dataset_cfg = data.get("dataset", {})
    raw_cfg = data.get("raw", {})
    outputs_cfg = data.get("outputs", {})
    filters_cfg = data.get("filters", {})
    validation_cfg = data.get("validation", {})
    outlier_cfg = data.get("outlier_policy", {})

    def _read_min_max(section: dict[str, Any], key: str) -> tuple[float | None, float | None]:
        raw = section.get(key)
        if not isinstance(raw, dict):
            return None, None
        low = raw.get("min")
        high = raw.get("max")
        return (
            float(low) if low is not None else None,
            float(high) if high is not None else None,
        )

    height_min_mm, height_max_mm = _read_min_max(validation_cfg, "height_mm")
    width_min_mm, width_max_mm = _read_min_max(validation_cfg, "width_mm")
    current_min_A, current_max_A = _read_min_max(validation_cfg, "current_A")
    wire_feed_speed_min_mpm, wire_feed_speed_max_mpm = _read_min_max(
        validation_cfg, "wire_feed_speed_mpm"
    )

    return Config(
        dataset_id=str(dataset_cfg["dataset_id"]),
        schema_version=str(dataset_cfg["schema_version"]),
        feature_contract_version=str(dataset_cfg["feature_contract_version"]),
        source_root=Path(raw_cfg["source_root"]),
        mapping_csv_path=Path(raw_cfg["mapping_csv_path"]),
        bronze_dir=Path(outputs_cfg["bronze_dir"]),
        silver_csv=Path(outputs_cfg["silver_csv"]),
        gold_csv=Path(outputs_cfg["gold_csv"]),
        selection_report_csv=Path(outputs_cfg["selection_report_csv"]),
        qc_report_json=Path(outputs_cfg["qc_report_json"]),
        readme_path=Path(outputs_cfg["readme_path"]),
        progression_min=float(filters_cfg["progression_min"]),
        progression_max=float(filters_cfg["progression_max"]),
        min_geometry_samples_per_attempt=int(filters_cfg["min_geometry_samples_per_attempt"]),
        min_welder_samples_per_attempt=int(filters_cfg["min_welder_samples_per_attempt"]),
        height_min_mm=height_min_mm,
        height_max_mm=height_max_mm,
        width_min_mm=width_min_mm,
        width_max_mm=width_max_mm,
        current_min_A=current_min_A,
        current_max_A=current_max_A,
        wire_feed_speed_min_mpm=wire_feed_speed_min_mpm,
        wire_feed_speed_max_mpm=wire_feed_speed_max_mpm,
        enable_per_attempt_iqr_filter=bool(
            outlier_cfg.get("enable_per_attempt_iqr_filter", True)
        ),
        iqr_multiplier=float(outlier_cfg.get("iqr_multiplier", 1.5)),
        iqr_min_samples_per_attempt=int(outlier_cfg.get("min_samples_per_attempt", 20)),
    )


def ensure_dirs(cfg: Config) -> None:
    cfg.bronze_dir.mkdir(parents=True, exist_ok=True)
    cfg.silver_csv.parent.mkdir(parents=True, exist_ok=True)
    cfg.gold_csv.parent.mkdir(parents=True, exist_ok=True)
    cfg.selection_report_csv.parent.mkdir(parents=True, exist_ok=True)
    cfg.qc_report_json.parent.mkdir(parents=True, exist_ok=True)
    cfg.readme_path.parent.mkdir(parents=True, exist_ok=True)


def to_float(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def to_str(value: Any) -> str:
    if value is None:
        return ""
    return str(value).strip()


def read_fragmented_topics(bag_path: Path) -> dict[str, pd.DataFrame]:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "Missing dependency `rosbags`. Install it before running this builder."
        ) from exc

    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path does not exist: {bag_path}")

    rows: dict[str, list[dict[str, Any]]] = {topic: [] for topic in TOPIC_COLUMNS}

    with AnyReader([bag_path]) as reader:
        for conn, timestamp_ns, rawdata in reader.messages():
            topic = conn.topic
            if topic not in rows:
                continue
            msg = reader.deserialize(rawdata, conn.msgtype)

            if topic == "/robin/weld_dimensions":
                rows[topic].append(
                    {
                        "timestamp_ns": int(timestamp_ns),
                        "bead_id": to_str(getattr(msg, "bead_id", "")),
                        "progression": to_float(getattr(msg, "progression", np.nan)),
                        "height_mm": to_float(getattr(msg, "height_mm", np.nan)),
                        "width_mm": to_float(getattr(msg, "width_mm", np.nan)),
                        "toe_angle_rad": to_float(getattr(msg, "toe_angle_rad", np.nan)),
                    }
                )
            elif topic == "/robin/data/fronius":
                rows[topic].append(
                    {
                        "timestamp_ns": int(timestamp_ns),
                        "bead_id": to_str(getattr(msg, "bead_id", "")),
                        "progression": to_float(getattr(msg, "progression", np.nan)),
                        "current": to_float(getattr(msg, "current", np.nan)),
                        "voltage": to_float(getattr(msg, "voltage", np.nan)),
                        "wire_feed_speed": to_float(getattr(msg, "wire_feed_speed", np.nan)),
                        "power": to_float(getattr(msg, "power", np.nan)),
                        "energy": to_float(getattr(msg, "energy", np.nan)),
                    }
                )
            elif topic == "/robin/data/active_bead":
                rows[topic].append(
                    {
                        "timestamp_ns": int(timestamp_ns),
                        "bead_id": to_str(getattr(msg, "bead_id", "")),
                        "ros_travel_speed_cmd_mps": to_float(getattr(msg, "weld_speed", np.nan)),
                        "ros_wire_feed_speed_cmd_mpm": to_float(
                            getattr(msg, "wire_feed_speed", np.nan)
                        ),
                        "ros_arc_length_correction_cmd_mm": to_float(
                            getattr(msg, "arc_length_correction_mm", np.nan)
                        ),
                        "ros_current_recomm_A": to_float(
                            getattr(msg, "current_recommvalue", np.nan)
                        ),
                        "ros_voltage_recomm_V": to_float(
                            getattr(msg, "voltage_recommvalue", np.nan)
                        ),
                        "total_length_m": to_float(getattr(msg, "total_length", np.nan)),
                    }
                )
            elif topic == "/robin/data/progression":
                rows[topic].append(
                    {
                        "timestamp_ns": int(timestamp_ns),
                        "bead_id": to_str(getattr(msg, "bead_id", "")),
                        "progression": to_float(getattr(msg, "progression", np.nan)),
                        "is_welding": bool(getattr(msg, "is_welding", False)),
                    }
                )

    tables: dict[str, pd.DataFrame] = {}
    for topic, cols in TOPIC_COLUMNS.items():
        if rows[topic]:
            df = pd.DataFrame(rows[topic])
            for col in cols:
                if col not in df.columns:
                    df[col] = np.nan
            if "bead_id" in df.columns:
                df["bead_id"] = df["bead_id"].fillna("").astype(str).str.strip()
            df = df[cols].sort_values("timestamp_ns").reset_index(drop=True)
        else:
            df = pd.DataFrame(columns=cols)
        tables[topic] = df
    return tables


def export_bronze_tables(bag_stem: str, tables: dict[str, pd.DataFrame], bronze_dir: Path) -> None:
    for topic, df in tables.items():
        export_name = TOPIC_EXPORT_NAMES[topic]
        out_path = bronze_dir / f"{bag_stem}__{export_name}.csv"
        df.to_csv(out_path, index=False)


def filter_stable_window(df: pd.DataFrame, pmin: float, pmax: float) -> pd.DataFrame:
    out = df.copy()
    out["progression"] = pd.to_numeric(out["progression"], errors="coerce")
    return out[(out["progression"] >= pmin) & (out["progression"] <= pmax)].reset_index(drop=True)


def robust_series_stats(series: pd.Series, prefix: str) -> dict[str, Any]:
    clean = pd.to_numeric(series, errors="coerce").dropna()
    if clean.empty:
        return {
            f"{prefix}_mean": np.nan,
            f"{prefix}_median": np.nan,
            f"{prefix}_std": np.nan,
            f"{prefix}_p10": np.nan,
            f"{prefix}_p90": np.nan,
        }
    return {
        f"{prefix}_mean": float(clean.mean()),
        f"{prefix}_median": float(clean.median()),
        f"{prefix}_std": float(clean.std(ddof=0)),
        f"{prefix}_p10": float(clean.quantile(0.10)),
        f"{prefix}_p90": float(clean.quantile(0.90)),
    }


def apply_physical_bounds(
    df: pd.DataFrame,
    bounds: dict[str, tuple[float | None, float | None]],
) -> tuple[pd.DataFrame, dict[str, int]]:
    out = df.copy()
    removed_by_column: dict[str, int] = {}
    for col, (low, high) in bounds.items():
        if col not in out.columns:
            continue
        values = pd.to_numeric(out[col], errors="coerce")
        mask = pd.Series(True, index=out.index)
        if low is not None:
            mask &= values.isna() | (values >= low)
        if high is not None:
            mask &= values.isna() | (values <= high)
        removed_by_column[col] = int((~mask).sum())
        out = out[mask]
    return out.reset_index(drop=True), removed_by_column


def apply_iqr_filter(
    df: pd.DataFrame,
    value_columns: list[str],
    iqr_multiplier: float,
    min_samples: int,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    if df.empty:
        return (
            df.copy(),
            {
                "rows_in": 0,
                "rows_out": 0,
                "rows_removed_total": 0,
                "rows_removed_by_column": {},
            },
        )

    keep_mask = pd.Series(True, index=df.index)
    removed_by_column: dict[str, int] = {}

    for col in value_columns:
        if col not in df.columns:
            continue
        values = pd.to_numeric(df[col], errors="coerce")
        clean = values.dropna()
        if len(clean) < min_samples:
            removed_by_column[col] = 0
            continue
        q1 = clean.quantile(0.25)
        q3 = clean.quantile(0.75)
        iqr = q3 - q1
        if iqr <= 0:
            removed_by_column[col] = 0
            continue
        lower = q1 - iqr_multiplier * iqr
        upper = q3 + iqr_multiplier * iqr
        inlier = values.isna() | ((values >= lower) & (values <= upper))
        removed_by_column[col] = int((keep_mask & ~inlier).sum())
        keep_mask &= inlier

    filtered = df[keep_mask].reset_index(drop=True)
    return (
        filtered,
        {
            "rows_in": int(len(df)),
            "rows_out": int(len(filtered)),
            "rows_removed_total": int(len(df) - len(filtered)),
            "rows_removed_by_column": removed_by_column,
        },
    )


def quality_flag_for_attempt(
    recipe_id: str | float,
    geometry_count: int,
    welder_count: int,
    cfg: Config,
) -> str:
    if pd.isna(recipe_id) or not str(recipe_id).strip():
        return "reject_unmapped_recipe"
    if geometry_count == 0:
        return "reject_missing_geometry_samples"
    if geometry_count < cfg.min_geometry_samples_per_attempt:
        return "reject_low_geometry_samples"
    if welder_count == 0:
        return "reject_missing_welder_samples"
    if welder_count < cfg.min_welder_samples_per_attempt:
        return "reject_low_welder_samples"
    return "ok"


def infer_end_times(attempts: pd.DataFrame, bag_max_ts: dict[str, int]) -> pd.DataFrame:
    out = attempts.sort_values(["bag_file", "start_ts"]).copy()
    out["end_ts_exclusive"] = (
        out.groupby("bag_file")["start_ts"].shift(-1)
        .fillna(out["bag_file"].map(lambda name: bag_max_ts[name] + 1))
        .astype("int64")
    )
    return out


def aggregate_attempt(
    attempt: pd.Series,
    tables: dict[str, pd.DataFrame],
    cfg: Config,
) -> tuple[dict[str, Any], dict[str, Any]]:
    doe_wire_feed_speed = attempt.get("doe_wire_feed_speed", attempt.get("wire_feed_speed", np.nan))
    doe_travel_speed = attempt.get("doe_weld_speed", attempt.get("weld_speed", np.nan))
    doe_arc_length = attempt.get(
        "doe_arc_length_correction_mm",
        attempt.get("arc_length_correction_mm", np.nan),
    )

    bead_id = str(attempt["start_bead_id"])
    start_ts = int(attempt["start_ts"])
    end_ts_exclusive = int(attempt["end_ts_exclusive"])

    geom_raw = tables["/robin/weld_dimensions"]
    geom_raw = geom_raw[
        (geom_raw["bead_id"] == bead_id)
        & (geom_raw["timestamp_ns"] >= start_ts)
        & (geom_raw["timestamp_ns"] < end_ts_exclusive)
    ].copy()
    weld_raw = tables["/robin/data/fronius"]
    weld_raw = weld_raw[
        (weld_raw["bead_id"] == bead_id)
        & (weld_raw["timestamp_ns"] >= start_ts)
        & (weld_raw["timestamp_ns"] < end_ts_exclusive)
    ].copy()
    progression_raw = tables["/robin/data/progression"]
    progression_raw = progression_raw[
        (progression_raw["bead_id"] == bead_id)
        & (progression_raw["timestamp_ns"] >= start_ts)
        & (progression_raw["timestamp_ns"] < end_ts_exclusive)
    ].copy()

    geom_stable = filter_stable_window(geom_raw, cfg.progression_min, cfg.progression_max)
    weld_stable = filter_stable_window(weld_raw, cfg.progression_min, cfg.progression_max)
    progression_stable = filter_stable_window(
        progression_raw, cfg.progression_min, cfg.progression_max
    )

    geom_bounded, geom_bounds_removed = apply_physical_bounds(
        geom_stable,
        {
            "height_mm": (cfg.height_min_mm, cfg.height_max_mm),
            "width_mm": (cfg.width_min_mm, cfg.width_max_mm),
        },
    )
    weld_bounded, weld_bounds_removed = apply_physical_bounds(
        weld_stable,
        {
            "current": (cfg.current_min_A, cfg.current_max_A),
            "wire_feed_speed": (
                cfg.wire_feed_speed_min_mpm,
                cfg.wire_feed_speed_max_mpm,
            ),
        },
    )

    if cfg.enable_per_attempt_iqr_filter:
        geom_filtered, geom_iqr_stats = apply_iqr_filter(
            geom_bounded,
            value_columns=["height_mm", "width_mm"],
            iqr_multiplier=cfg.iqr_multiplier,
            min_samples=cfg.iqr_min_samples_per_attempt,
        )
        weld_filtered, weld_iqr_stats = apply_iqr_filter(
            weld_bounded,
            value_columns=["current", "voltage", "wire_feed_speed", "power"],
            iqr_multiplier=cfg.iqr_multiplier,
            min_samples=cfg.iqr_min_samples_per_attempt,
        )
    else:
        geom_filtered = geom_bounded
        weld_filtered = weld_bounded
        geom_iqr_stats = {
            "rows_in": int(len(geom_bounded)),
            "rows_out": int(len(geom_bounded)),
            "rows_removed_total": 0,
            "rows_removed_by_column": {},
        }
        weld_iqr_stats = {
            "rows_in": int(len(weld_bounded)),
            "rows_out": int(len(weld_bounded)),
            "rows_removed_total": 0,
            "rows_removed_by_column": {},
        }

    height = pd.to_numeric(geom_filtered.get("height_mm", pd.Series(dtype=float)), errors="coerce").dropna()
    width = pd.to_numeric(geom_filtered.get("width_mm", pd.Series(dtype=float)), errors="coerce").dropna()

    bag_file = str(attempt["bag_file"])
    run_id = Path(bag_file).stem
    quality_flag = quality_flag_for_attempt(
        recipe_id=attempt.get("input_id"),
        geometry_count=len(geom_filtered),
        welder_count=len(weld_filtered),
        cfg=cfg,
    )

    row: dict[str, Any] = {
        "schema_version": cfg.schema_version,
        "feature_contract_version": cfg.feature_contract_version,
        "dataset_id": cfg.dataset_id,
        "run_id": run_id,
        "bag_file": bag_file,
        "attempt_id": str(attempt["attempt_id"]),
        "attempt_idx_in_bag": int(attempt["attempt_idx_in_bag"]),
        "bead_id": bead_id,
        "doe_input_id": str(attempt["input_id"]) if pd.notna(attempt["input_id"]) else np.nan,
        "quality_flag": quality_flag,
        "start_time_ns": start_ts,
        "end_time_ns": max(start_ts, end_ts_exclusive - 1),
        "progression_min": cfg.progression_min,
        "progression_max": cfg.progression_max,
        "progression_coverage": float(max(0.0, cfg.progression_max - cfg.progression_min)),
        "n_progression_samples": int(len(progression_stable)),
        "n_geometry_samples": int(len(geom_filtered)),
        "n_welder_samples": int(len(weld_filtered)),
        "geometry_msgs_total_interval": int(len(geom_raw)),
        "welder_msgs_total_interval": int(len(weld_raw)),
        "progression_msgs_total_interval": int(len(progression_raw)),
        "ros_wire_feed_speed_cmd_mpm": float(attempt["wire_feed_speed_cmd_ros"]),
        "ros_travel_speed_cmd_mps": float(attempt["weld_speed_cmd_ros"]),
        "ros_arc_length_correction_cmd_mm": float(attempt["arc_length_correction_cmd_ros"]),
        "ros_current_recomm_A": float(attempt["current_recomm_ros"]),
        "ros_voltage_recomm_V": float(attempt["voltage_recomm_ros"]),
        "total_length_m": float(attempt["total_length_m"]),
        "doe_wire_feed_speed_mpm": float(doe_wire_feed_speed)
        if pd.notna(doe_wire_feed_speed)
        else np.nan,
        "doe_travel_speed_mps": float(doe_travel_speed)
        if pd.notna(doe_travel_speed)
        else np.nan,
        "doe_arc_length_correction_mm": float(doe_arc_length)
        if pd.notna(doe_arc_length)
        else np.nan,
        "wire_feed_speed_mpm_model_input": float(doe_wire_feed_speed)
        if pd.notna(doe_wire_feed_speed)
        else np.nan,
        "travel_speed_mps_model_input": float(doe_travel_speed)
        if pd.notna(doe_travel_speed)
        else np.nan,
        "arc_length_correction_mm_model_input": float(doe_arc_length)
        if pd.notna(doe_arc_length)
        else np.nan,
        "input_source": "doe_mapped_recipe",
        "weld_speed_match": str(attempt["weld_speed_match"])
        if pd.notna(attempt["weld_speed_match"])
        else np.nan,
        "weld_speed_ratio_ros_to_doe": float(attempt["weld_speed_ratio_ros_to_doe"])
        if pd.notna(attempt["weld_speed_ratio_ros_to_doe"])
        else np.nan,
        "active_start_msgs": int(attempt["active_start_msgs"]),
        "geometry_anomaly_flag": bool(attempt["geometry_anomaly_flag"]),
        "progression_anomaly_flag": bool(attempt["progression_anomaly_flag"]),
        "mapping_notes": str(attempt["notes"]).strip()
        if pd.notna(attempt["notes"])
        else "",
        "height_mm_target": float(height.median()) if not height.empty else np.nan,
        "width_mm_target": float(width.median()) if not width.empty else np.nan,
        "height_mm_std": float(height.std(ddof=0)) if len(height) > 1 else np.nan,
        "width_mm_std": float(width.std(ddof=0)) if len(width) > 1 else np.nan,
        "height_mm_p10": float(height.quantile(0.10)) if not height.empty else np.nan,
        "height_mm_p90": float(height.quantile(0.90)) if not height.empty else np.nan,
        "width_mm_p10": float(width.quantile(0.10)) if not width.empty else np.nan,
        "width_mm_p90": float(width.quantile(0.90)) if not width.empty else np.nan,
    }
    row.update(robust_series_stats(weld_filtered.get("current", pd.Series(dtype=float)), "measured_current_A"))
    row.update(
        robust_series_stats(
            weld_filtered.get("wire_feed_speed", pd.Series(dtype=float)),
            "measured_wire_feed_speed_mpm",
        )
    )
    row.update(robust_series_stats(weld_filtered.get("voltage", pd.Series(dtype=float)), "measured_voltage_V"))
    row.update(robust_series_stats(weld_filtered.get("power", pd.Series(dtype=float)), "measured_power_W"))

    qc_detail = {
        "attempt_id": str(attempt["attempt_id"]),
        "geometry_bounds_removed_by_column": geom_bounds_removed,
        "welder_bounds_removed_by_column": weld_bounds_removed,
        "geometry_iqr_stats": geom_iqr_stats,
        "welder_iqr_stats": weld_iqr_stats,
        "geometry_rows_stable_window": int(len(geom_stable)),
        "welder_rows_stable_window": int(len(weld_stable)),
        "progression_rows_stable_window": int(len(progression_stable)),
    }
    return row, qc_detail


def select_recipe_rows(attempt_df: pd.DataFrame) -> pd.DataFrame:
    ranked = attempt_df.copy()
    ranked["_quality_ok"] = ranked["quality_flag"].eq("ok")
    ranked["_weld_speed_exact"] = ranked["weld_speed_match"].eq("exact")
    ranked["_anomaly_count"] = (
        ranked["geometry_anomaly_flag"].fillna(False).astype(int)
        + ranked["progression_anomaly_flag"].fillna(False).astype(int)
    )
    ranked["_travel_mismatch_abs"] = (
        pd.to_numeric(ranked["ros_travel_speed_cmd_mps"], errors="coerce")
        - pd.to_numeric(ranked["doe_travel_speed_mps"], errors="coerce")
    ).abs().fillna(np.inf)

    ranked = ranked.sort_values(
        by=[
            "doe_input_id",
            "_quality_ok",
            "_weld_speed_exact",
            "_anomaly_count",
            "_travel_mismatch_abs",
            "n_geometry_samples",
            "n_welder_samples",
            "start_time_ns",
        ],
        ascending=[True, False, False, True, True, False, False, True],
    ).reset_index(drop=True)
    ranked["selection_rank_within_recipe"] = ranked.groupby("doe_input_id").cumcount() + 1
    ranked["selected_for_training"] = ranked["selection_rank_within_recipe"] == 1

    reasons: list[str] = []
    for row in ranked.itertuples(index=False):
        if not row.selected_for_training:
            reasons.append("not_selected_better_candidate_exists")
        elif row.quality_flag == "ok" and row.weld_speed_match == "exact":
            reasons.append("selected_best_quality_exact_match")
        elif row.quality_flag == "ok":
            reasons.append("selected_best_quality_available_mismatch")
        else:
            reasons.append("selected_placeholder_no_valid_attempt")
    ranked["selection_reason"] = reasons
    return ranked.drop(
        columns=[
            "_quality_ok",
            "_weld_speed_exact",
            "_anomaly_count",
            "_travel_mismatch_abs",
        ]
    )


def selection_report_columns(df: pd.DataFrame) -> list[str]:
    preferred = [
        "doe_input_id",
        "attempt_id",
        "run_id",
        "bead_id",
        "quality_flag",
        "selected_for_training",
        "selection_rank_within_recipe",
        "selection_reason",
        "weld_speed_match",
        "weld_speed_ratio_ros_to_doe",
        "n_geometry_samples",
        "n_welder_samples",
        "height_mm_target",
        "width_mm_target",
        "geometry_anomaly_flag",
        "progression_anomaly_flag",
        "mapping_notes",
    ]
    return [col for col in preferred if col in df.columns]


def build_dataset(cfg: Config) -> tuple[pd.DataFrame, pd.DataFrame, dict[str, Any]]:
    attempts = pd.read_csv(cfg.mapping_csv_path)
    bag_paths = {
        path.name: path
        for path in cfg.source_root.rglob("*.mcap")
    }

    missing_bags = sorted(set(attempts["bag_file"]) - set(bag_paths))
    if missing_bags:
        raise FileNotFoundError(
            "Missing MCAP files referenced by the mapping CSV: "
            + ", ".join(missing_bags)
        )

    tables_by_bag: dict[str, dict[str, pd.DataFrame]] = {}
    bag_max_ts: dict[str, int] = {}
    for bag_file in sorted(attempts["bag_file"].unique()):
        bag_path = bag_paths[bag_file]
        tables = read_fragmented_topics(bag_path)
        export_bronze_tables(Path(bag_file).stem, tables, cfg.bronze_dir)
        max_ts_candidates = [
            int(df["timestamp_ns"].max())
            for df in tables.values()
            if not df.empty and "timestamp_ns" in df.columns
        ]
        bag_max_ts[bag_file] = max(max_ts_candidates) if max_ts_candidates else 0
        tables_by_bag[bag_file] = tables

    attempts = infer_end_times(attempts, bag_max_ts)

    rows: list[dict[str, Any]] = []
    qc_attempt_details: list[dict[str, Any]] = []
    for attempt in attempts.itertuples(index=False):
        row, qc_detail = aggregate_attempt(
            pd.Series(attempt._asdict()),
            tables=tables_by_bag[str(attempt.bag_file)],
            cfg=cfg,
        )
        rows.append(row)
        qc_attempt_details.append(qc_detail)

    attempt_df = pd.DataFrame(rows).sort_values(["bag_file", "attempt_idx_in_bag"]).reset_index(drop=True)
    ranked_df = select_recipe_rows(attempt_df)
    selected_df = ranked_df[ranked_df["selected_for_training"]].copy()
    selected_df = selected_df.sort_values("doe_input_id").reset_index(drop=True)

    qc = {
        "dataset_id": cfg.dataset_id,
        "schema_version": cfg.schema_version,
        "feature_contract_version": cfg.feature_contract_version,
        "source_root": str(cfg.source_root),
        "mapping_csv_path": str(cfg.mapping_csv_path),
        "n_attempt_rows_total": int(len(ranked_df)),
        "n_attempt_rows_ok": int((ranked_df["quality_flag"] == "ok").sum()),
        "n_attempt_rows_rejected": int((ranked_df["quality_flag"] != "ok").sum()),
        "n_recipes_total": int(ranked_df["doe_input_id"].nunique()),
        "n_selected_rows_total": int(len(selected_df)),
        "n_selected_rows_ok": int((selected_df["quality_flag"] == "ok").sum()),
        "n_selected_rows_rejected": int((selected_df["quality_flag"] != "ok").sum()),
        "selected_quality_counts": selected_df["quality_flag"].value_counts(dropna=False).to_dict(),
        "unrecoverable_recipe_ids": selected_df.loc[
            selected_df["quality_flag"] != "ok", "doe_input_id"
        ].dropna().tolist(),
        "attempts_by_bag": ranked_df["bag_file"].value_counts().sort_index().to_dict(),
        "selection_policy": {
            "group_by": "doe_input_id",
            "priority_order": [
                "quality_flag == ok",
                "weld_speed_match == exact",
                "lower anomaly count",
                "smaller abs(ros_travel_speed_cmd_mps - doe_travel_speed_mps)",
                "higher n_geometry_samples",
                "higher n_welder_samples",
                "earlier start_time_ns",
            ],
        },
        "attempt_qc_details": qc_attempt_details,
    }
    return ranked_df, selected_df, qc


def write_output_readme(cfg: Config, qc: dict[str, Any]) -> None:
    unrecoverable = qc.get("unrecoverable_recipe_ids", [])
    unrecoverable_text = ", ".join(unrecoverable) if unrecoverable else "none"
    selected_ok = qc["n_selected_rows_ok"]
    selected_total = qc["n_selected_rows_total"]
    recoverability_note = ""
    if unrecoverable:
        recoverability_note = (
            "\nThe selected dataset keeps unrecoverable recipes as explicit rejected rows so the "
            "missing cases stay visible. In the current data, `LHS48-22` only has progression "
            "messages and no geometry/fronius samples, so training should use the 21 rows where "
            "`quality_flag == ok`.\n"
        )
    text = f"""# Fragmented Corrected-Input Dataset

This dataset is separate from the legacy `robin_data_01` dataset.

- Legacy dataset with wrong feature contract: `data/gold/train_dataset.csv`
- Corrected fragmented dataset built from `mlops/fragmented_datasets`: `{cfg.gold_csv}`

The builder uses `{cfg.mapping_csv_path}` as the canonical recipe manifest.
That mapping is expected to be regenerated from `mlops/fragmented_datasets/DOE_LHS_48.json`
before rebuilding this dataset.

## Output files

- `metadata/`: canonical mapping manifest and source inventory
- `bronze/topic_exports/`: decoded per-bag topic CSVs
- `silver/bead_level/attempt_level_dataset.csv`: one cleaned row per mapped attempt
- `gold/train_dataset.csv`: one selected row per unique DOE recipe
- `gold/selection_report.csv`: ranking of all attempts within each recipe
- `gold/qc_report.json`: counts and per-attempt filtering diagnostics

## Counts

- Attempt rows: {qc["n_attempt_rows_total"]} total, {qc["n_attempt_rows_ok"]} `ok`
- Selected recipe rows: {selected_total} total, {selected_ok} `ok`
- Unrecoverable recipe IDs: {unrecoverable_text}
{recoverability_note}

## Selection policy

For each `doe_input_id`, the selected row is the highest-ranked attempt using this order:

1. `quality_flag == ok`
2. exact canonical DOE/ROS weld-speed match when available
3. lower anomaly count
4. smaller residual travel-speed mismatch
5. more retained geometry and welder samples
"""
    cfg.readme_path.write_text(text)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    ensure_dirs(cfg)

    ranked_df, selected_df, qc = build_dataset(cfg)

    silver_df = ranked_df.drop(columns=["selected_for_training", "selection_rank_within_recipe", "selection_reason"])
    silver_df.to_csv(cfg.silver_csv, index=False)
    selected_df.to_csv(cfg.gold_csv, index=False)
    ranked_df[selection_report_columns(ranked_df)].to_csv(cfg.selection_report_csv, index=False)
    cfg.qc_report_json.write_text(json.dumps(qc, indent=2))
    write_output_readme(cfg, qc)

    print(f"Wrote {cfg.silver_csv}")
    print(f"Wrote {cfg.gold_csv}")
    print(f"Wrote {cfg.selection_report_csv}")
    print(f"Wrote {cfg.qc_report_json}")
    print(f"Wrote {cfg.readme_path}")


if __name__ == "__main__":
    main()
