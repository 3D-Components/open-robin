"""
ROS 2 MCAP -> bead-level CSV extractor with wire-speed fallback support.

Design intent
-------------
This script preserves the long-term production contract that target/instructed
features should drive the deployable model, while allowing a controlled fallback
for wire speed when the target field is missing in early prototype bags.

The fallback logic is:
- keep target_wire_feed_speed_mpm unchanged
- if target_wire_feed_speed_mpm is invalid or zero-like, compute measured
  stable-window summaries per bead
- populate wire_speed_mpm_model_input from target if valid, otherwise from the
  configured measured fallback statistic
- preserve provenance in wire_speed_source and wire_speed_is_fallback

Expected decoded topic table contract
------------------------------------
geometry:
  [timestamp_ns, bead_id, progression, height_mm, width_mm]
fronius:
  [timestamp_ns, bead_id, progression, current, voltage, wire_feed_speed, power]
active_bead:
  [timestamp_ns, bead_id, target_speed, target_current, target_voltage, target_wire_speed, target_stickout]
progression:
  [timestamp_ns, bead_id, progression, is_welding]
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml


@dataclass
class Config:
    raw_bag_path: Path
    doe_csv_path: Path | None
    bronze_dir: Path
    silver_dir: Path
    gold_csv: Path
    qc_report_json: Path
    schema_version: str
    feature_contract_version: str
    progression_min: float = 0.10
    progression_max: float = 0.90
    min_geometry_samples_per_bead: int = 25
    min_welder_samples_per_bead: int = 10
    target_wire_speed_min_valid: float = 0.001
    target_stickout_min_valid: float = 0.001
    height_min_mm: float | None = None
    height_max_mm: float | None = None
    width_min_mm: float | None = None
    width_max_mm: float | None = None
    current_min_A: float | None = None
    current_max_A: float | None = None
    wire_feed_speed_min_mpm: float | None = None
    wire_feed_speed_max_mpm: float | None = None
    allow_measured_fallback: bool = True
    measured_fallback_stat: str = "median"
    allow_stickout_doe_fallback: bool = True
    enable_per_bead_iqr_filter: bool = True
    iqr_multiplier: float = 1.5
    iqr_min_samples_per_bead: int = 20


def load_config(path: Path) -> Config:
    data = yaml.safe_load(path.read_text())
    raw_cfg = data.get("raw", {})
    outputs_cfg = data.get("outputs", {})
    dataset_cfg = data.get("dataset", {})
    filters_cfg = data.get("filters", {})
    validation_cfg = data.get("validation", {})
    wire_speed_cfg = data.get("wire_speed_policy", {})
    stickout_cfg = data.get("stickout_policy", {})
    outlier_cfg = data.get("outlier_policy", {})
    doe_csv_path_raw = raw_cfg.get("doe_csv_path")

    def _read_min_max(section: dict[str, Any], key: str) -> tuple[float | None, float | None]:
        val = section.get(key)
        if not isinstance(val, dict):
            return None, None
        low = val.get("min")
        high = val.get("max")
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
        raw_bag_path=Path(raw_cfg["bag_path"]),
        doe_csv_path=Path(doe_csv_path_raw) if doe_csv_path_raw else None,
        bronze_dir=Path(outputs_cfg["bronze_dir"]),
        silver_dir=Path(outputs_cfg["silver_dir"]),
        gold_csv=Path(outputs_cfg["gold_csv"]),
        qc_report_json=Path(outputs_cfg["qc_report_json"]),
        schema_version=str(dataset_cfg["schema_version"]),
        feature_contract_version=str(dataset_cfg["feature_contract_version"]),
        progression_min=float(filters_cfg["progression_min"]),
        progression_max=float(filters_cfg["progression_max"]),
        min_geometry_samples_per_bead=int(filters_cfg["min_geometry_samples_per_bead"]),
        min_welder_samples_per_bead=int(filters_cfg["min_welder_samples_per_bead"]),
        target_wire_speed_min_valid=float(validation_cfg.get("target_wire_speed_min_valid", 0.001)),
        target_stickout_min_valid=float(validation_cfg.get("target_stickout_min_valid", 0.001)),
        height_min_mm=height_min_mm,
        height_max_mm=height_max_mm,
        width_min_mm=width_min_mm,
        width_max_mm=width_max_mm,
        current_min_A=current_min_A,
        current_max_A=current_max_A,
        wire_feed_speed_min_mpm=wire_feed_speed_min_mpm,
        wire_feed_speed_max_mpm=wire_feed_speed_max_mpm,
        allow_measured_fallback=bool(wire_speed_cfg.get("allow_measured_fallback", True)),
        measured_fallback_stat=str(wire_speed_cfg.get("measured_fallback_stat", "median")),
        allow_stickout_doe_fallback=bool(stickout_cfg.get("allow_doe_fallback", True)),
        enable_per_bead_iqr_filter=bool(outlier_cfg.get("enable_per_bead_iqr_filter", True)),
        iqr_multiplier=float(outlier_cfg.get("iqr_multiplier", 1.5)),
        iqr_min_samples_per_bead=int(outlier_cfg.get("min_samples_per_bead", 20)),
    )


def ensure_dirs(cfg: Config) -> None:
    cfg.bronze_dir.mkdir(parents=True, exist_ok=True)
    cfg.silver_dir.mkdir(parents=True, exist_ok=True)
    cfg.gold_csv.parent.mkdir(parents=True, exist_ok=True)
    cfg.qc_report_json.parent.mkdir(parents=True, exist_ok=True)


def read_rosbag_topics(bag_path: Path) -> dict[str, pd.DataFrame]:
    """
    Decode the rosbag and return topic -> DataFrame.

    Supports ROS 2 bag directories and standalone MCAP paths via `rosbags`.
    """
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "Missing dependency `rosbags`. Install it in the active environment."
        ) from exc

    bag_path = Path(bag_path)
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path does not exist: {bag_path}")

    topic_columns: dict[str, list[str]] = {
        "/robin/weld_dimensions": [
            "timestamp_ns",
            "bead_id",
            "progression",
            "height_mm",
            "width_mm",
        ],
        "/robin/data/fronius": [
            "timestamp_ns",
            "bead_id",
            "progression",
            "current",
            "voltage",
            "wire_feed_speed",
            "power",
        ],
        "/robin/data/active_bead": [
            "timestamp_ns",
            "bead_id",
            "target_speed",
            "target_current",
            "target_voltage",
            "target_wire_speed",
            "target_stickout",
        ],
        "/robin/data/progression": [
            "timestamp_ns",
            "bead_id",
            "progression",
            "is_welding",
        ],
    }
    rows: dict[str, list[dict[str, Any]]] = {topic: [] for topic in topic_columns}

    def to_float(value: Any) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return float("nan")

    def to_str(value: Any) -> str:
        if value is None:
            return ""
        return str(value).strip()

    def first_attr(msg: Any, names: list[str], default: Any = None) -> Any:
        for name in names:
            if hasattr(msg, name):
                return getattr(msg, name)
        return default

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
                        "wire_feed_speed": to_float(
                            first_attr(
                                msg,
                                ["wire_feed_speed", "target_wire_speed", "wire_speed"],
                                np.nan,
                            )
                        ),
                        "power": to_float(getattr(msg, "power", np.nan)),
                    }
                )
            elif topic == "/robin/data/active_bead":
                rows[topic].append(
                    {
                        "timestamp_ns": int(timestamp_ns),
                        "bead_id": to_str(getattr(msg, "bead_id", "")),
                        "target_speed": to_float(
                            first_attr(
                                msg,
                                ["target_speed", "target_travel_speed", "travel_speed", "speed"],
                                np.nan,
                            )
                        ),
                        "target_current": to_float(
                            first_attr(msg, ["target_current", "current"], np.nan)
                        ),
                        "target_voltage": to_float(
                            first_attr(msg, ["target_voltage", "voltage"], np.nan)
                        ),
                        "target_wire_speed": to_float(
                            first_attr(
                                msg,
                                [
                                    "target_wire_speed",
                                    "target_wire_feed_speed",
                                    "wire_feed_speed",
                                    "wire_speed",
                                ],
                                np.nan,
                            )
                        ),
                        "target_stickout": to_float(
                            first_attr(
                                msg,
                                [
                                    "target_stickout",
                                    "target_stickout_mm",
                                    "stickout",
                                    "stickout_mm",
                                    "contact_tip_to_work_distance",
                                    "target_contact_tip_to_work_distance",
                                ],
                                np.nan,
                            )
                        ),
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

    topic_tables: dict[str, pd.DataFrame] = {}
    for topic, cols in topic_columns.items():
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
        topic_tables[topic] = df

    return topic_tables


def validate_required_topics(topic_tables: dict[str, pd.DataFrame]) -> None:
    required = {
        "/robin/weld_dimensions",
        "/robin/data/fronius",
        "/robin/data/progression",
        "/robin/data/active_bead",
    }
    missing = sorted(required - set(topic_tables))
    if missing:
        raise ValueError(f"Missing required topics: {missing}")


def robust_series_stats(s: pd.Series, prefix: str) -> dict[str, Any]:
    s = pd.to_numeric(s, errors="coerce").dropna()
    if s.empty:
        return {
            f"{prefix}_mean": np.nan,
            f"{prefix}_median": np.nan,
            f"{prefix}_std": np.nan,
            f"{prefix}_p10": np.nan,
            f"{prefix}_p90": np.nan,
        }
    return {
        f"{prefix}_mean": float(s.mean()),
        f"{prefix}_median": float(s.median()),
        f"{prefix}_std": float(s.std(ddof=0)),
        f"{prefix}_p10": float(s.quantile(0.10)),
        f"{prefix}_p90": float(s.quantile(0.90)),
    }


def filter_stable_window(df: pd.DataFrame, pmin: float, pmax: float) -> pd.DataFrame:
    out = df.copy()
    out["progression"] = pd.to_numeric(out["progression"], errors="coerce")
    return out[(out["progression"] >= pmin) & (out["progression"] <= pmax)]


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


def apply_per_bead_iqr_filter(
    df: pd.DataFrame,
    value_columns: list[str],
    bead_col: str,
    iqr_multiplier: float,
    min_samples_per_bead: int,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    if df.empty or bead_col not in df.columns:
        return (
            df.copy(),
            {
                "rows_in": int(len(df)),
                "rows_out": int(len(df)),
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
        stats_df = pd.DataFrame({bead_col: df[bead_col], col: values})
        grouped = stats_df.groupby(bead_col, dropna=False)[col]

        q1 = grouped.transform(lambda s: s.quantile(0.25))
        q3 = grouped.transform(lambda s: s.quantile(0.75))
        iqr = q3 - q1
        sample_count = grouped.transform(lambda s: s.notna().sum())

        lower = q1 - iqr_multiplier * iqr
        upper = q3 + iqr_multiplier * iqr
        use_iqr = (sample_count >= min_samples_per_bead) & (iqr > 0)
        inlier = values.isna() | (~use_iqr) | ((values >= lower) & (values <= upper))

        removed_this_col = int((keep_mask & ~inlier).sum())
        removed_by_column[col] = removed_this_col
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


def summarize_active_bead(active_bead_df: pd.DataFrame) -> pd.DataFrame:
    """
    Reduce multiple ActiveBead messages per bead into one row.

    We use median across duplicate messages to tolerate repeated publication of
    identical or nearly identical settings.
    """
    src_cols = {
        "target_speed": "target_travel_speed_mps",
        "target_current": "target_current_A",
        "target_voltage": "target_voltage_V",
        "target_wire_speed": "target_wire_feed_speed_mpm",
        "target_stickout": "target_stickout_mm",
    }
    keep = ["bead_id"] + [c for c in src_cols if c in active_bead_df.columns]
    trimmed = active_bead_df[keep].copy()
    numeric_cols = [c for c in trimmed.columns if c != "bead_id"]
    for c in numeric_cols:
        trimmed[c] = pd.to_numeric(trimmed[c], errors="coerce")
    grouped = trimmed.groupby("bead_id", dropna=False)[numeric_cols].median().reset_index()
    return grouped.rename(columns=src_cols)


def load_doe_stickout_mapping(doe_csv_path: Path | None, bead_ids: list[str]) -> dict[str, float]:
    if doe_csv_path is None:
        return {}
    if not doe_csv_path.exists():
        raise FileNotFoundError(f"DOE CSV path does not exist: {doe_csv_path}")

    doe = pd.read_csv(doe_csv_path)
    stickout_col = next(
        (
            col
            for col in [
                "Stickout (mm)",
                "Stickout",
                "stickout_mm",
                "target_stickout_mm",
                "target_stickout",
            ]
            if col in doe.columns
        ),
        None,
    )
    if stickout_col is None:
        raise ValueError(
            "DOE CSV must contain a stickout column (e.g. 'Stickout (mm)' or 'stickout_mm')."
        )

    stickout_values = pd.to_numeric(doe[stickout_col], errors="coerce").tolist()
    mapping: dict[str, float] = {}
    for idx, bead_id in enumerate(bead_ids):
        if idx < len(stickout_values) and pd.notna(stickout_values[idx]):
            mapping[bead_id] = float(stickout_values[idx])
    return mapping


def choose_wire_speed_feature(
    target_wire_speed: float | None,
    measured_mean: float,
    measured_median: float,
    cfg: Config,
) -> dict[str, Any]:
    target_valid = (
        target_wire_speed is not None
        and pd.notna(target_wire_speed)
        and float(target_wire_speed) >= cfg.target_wire_speed_min_valid
    )

    if target_valid:
        return {
            "target_wire_feed_speed_available": True,
            "wire_speed_mpm_model_input": float(target_wire_speed),
            "wire_speed_source": "target",
            "wire_speed_is_fallback": False,
        }

    if cfg.allow_measured_fallback and pd.notna(measured_mean):
        value = measured_median if cfg.measured_fallback_stat == "median" else measured_mean
        if pd.notna(value):
            return {
                "target_wire_feed_speed_available": False,
                "wire_speed_mpm_model_input": float(value),
                "wire_speed_source": "measured_bead_fallback",
                "wire_speed_is_fallback": True,
            }

    return {
        "target_wire_feed_speed_available": False,
        "wire_speed_mpm_model_input": np.nan,
        "wire_speed_source": "missing",
        "wire_speed_is_fallback": False,
    }


def choose_stickout_feature(
    target_stickout: float | None,
    doe_stickout: float | None,
    cfg: Config,
) -> dict[str, Any]:
    target_valid = (
        target_stickout is not None
        and pd.notna(target_stickout)
        and float(target_stickout) >= cfg.target_stickout_min_valid
    )
    if target_valid:
        return {
            "target_stickout_available": True,
            "stickout_mm_model_input": float(target_stickout),
            "stickout_source": "target",
            "stickout_is_fallback": False,
        }

    if cfg.allow_stickout_doe_fallback and doe_stickout is not None and pd.notna(doe_stickout):
        return {
            "target_stickout_available": False,
            "stickout_mm_model_input": float(doe_stickout),
            "stickout_source": "doe_bead_fallback",
            "stickout_is_fallback": True,
        }

    return {
        "target_stickout_available": False,
        "stickout_mm_model_input": np.nan,
        "stickout_source": "missing",
        "stickout_is_fallback": False,
    }


def aggregate_bead(
    bead_id: str,
    run_id: str,
    bead_index: int,
    geom_df: pd.DataFrame,
    weld_df: pd.DataFrame,
    active_row: pd.Series | None,
    doe_stickout_mm: float | None,
    cfg: Config,
) -> dict[str, Any]:
    row: dict[str, Any] = {
        "schema_version": cfg.schema_version,
        "feature_contract_version": cfg.feature_contract_version,
        "run_id": run_id,
        "bead_id": bead_id,
        "bead_index": bead_index,
        "quality_flag": "ok",
    }

    timestamps = []
    if not geom_df.empty:
        timestamps.extend([geom_df["timestamp_ns"].min(), geom_df["timestamp_ns"].max()])
    if not weld_df.empty:
        timestamps.extend([weld_df["timestamp_ns"].min(), weld_df["timestamp_ns"].max()])
    row["start_time_ns"] = int(min(timestamps)) if timestamps else np.nan
    row["end_time_ns"] = int(max(timestamps)) if timestamps else np.nan
    row["progression_min"] = cfg.progression_min
    row["progression_max"] = cfg.progression_max
    row["progression_coverage"] = max(0.0, cfg.progression_max - cfg.progression_min)
    row["n_geometry_samples"] = int(len(geom_df))
    row["n_welder_samples"] = int(len(weld_df))

    if active_row is not None:
        row["target_current_A"] = float(active_row["target_current_A"]) if "target_current_A" in active_row and pd.notna(active_row["target_current_A"]) else np.nan
        row["target_wire_feed_speed_mpm"] = float(active_row["target_wire_feed_speed_mpm"]) if "target_wire_feed_speed_mpm" in active_row and pd.notna(active_row["target_wire_feed_speed_mpm"]) else np.nan
        row["target_travel_speed_mps"] = float(active_row["target_travel_speed_mps"]) if "target_travel_speed_mps" in active_row and pd.notna(active_row["target_travel_speed_mps"]) else np.nan
        row["target_voltage_V"] = float(active_row["target_voltage_V"]) if "target_voltage_V" in active_row and pd.notna(active_row["target_voltage_V"]) else np.nan
        row["target_stickout_mm"] = float(active_row["target_stickout_mm"]) if "target_stickout_mm" in active_row and pd.notna(active_row["target_stickout_mm"]) else np.nan
    else:
        row["target_current_A"] = np.nan
        row["target_wire_feed_speed_mpm"] = np.nan
        row["target_travel_speed_mps"] = np.nan
        row["target_voltage_V"] = np.nan
        row["target_stickout_mm"] = np.nan
        row["quality_flag"] = "reject_missing_active_bead"

    row.update(robust_series_stats(weld_df.get("current", pd.Series(dtype=float)), "measured_current_A"))
    row.update(robust_series_stats(weld_df.get("wire_feed_speed", pd.Series(dtype=float)), "measured_wire_feed_speed_mpm"))
    row.update(robust_series_stats(weld_df.get("voltage", pd.Series(dtype=float)), "measured_voltage_V"))
    row.update(robust_series_stats(weld_df.get("power", pd.Series(dtype=float)), "measured_power_W"))

    row["fallback_wire_speed_measured_mean_mpm"] = row.get("measured_wire_feed_speed_mpm_mean", np.nan)
    row["fallback_wire_speed_measured_median_mpm"] = row.get("measured_wire_feed_speed_mpm_median", np.nan)
    row["fallback_stickout_doe_mm"] = float(doe_stickout_mm) if doe_stickout_mm is not None and pd.notna(doe_stickout_mm) else np.nan

    row.update(
        choose_wire_speed_feature(
            target_wire_speed=row.get("target_wire_feed_speed_mpm"),
            measured_mean=row.get("fallback_wire_speed_measured_mean_mpm", np.nan),
            measured_median=row.get("fallback_wire_speed_measured_median_mpm", np.nan),
            cfg=cfg,
        )
    )
    row.update(
        choose_stickout_feature(
            target_stickout=row.get("target_stickout_mm"),
            doe_stickout=row.get("fallback_stickout_doe_mm"),
            cfg=cfg,
        )
    )

    height = pd.to_numeric(geom_df.get("height_mm", pd.Series(dtype=float)), errors="coerce").dropna()
    width = pd.to_numeric(geom_df.get("width_mm", pd.Series(dtype=float)), errors="coerce").dropna()
    row["height_mm_target"] = float(height.median()) if not height.empty else np.nan
    row["width_mm_target"] = float(width.median()) if not width.empty else np.nan
    row["height_mm_std"] = float(height.std(ddof=0)) if len(height) > 1 else np.nan
    row["width_mm_std"] = float(width.std(ddof=0)) if len(width) > 1 else np.nan
    row["height_mm_p10"] = float(height.quantile(0.10)) if not height.empty else np.nan
    row["height_mm_p90"] = float(height.quantile(0.90)) if not height.empty else np.nan
    row["width_mm_p10"] = float(width.quantile(0.10)) if not width.empty else np.nan
    row["width_mm_p90"] = float(width.quantile(0.90)) if not width.empty else np.nan

    if len(height) < cfg.min_geometry_samples_per_bead or len(width) < cfg.min_geometry_samples_per_bead:
        row["quality_flag"] = "reject_low_geometry_samples"
    elif len(weld_df) < cfg.min_welder_samples_per_bead:
        row["quality_flag"] = "reject_low_welder_samples"
    elif row["wire_speed_source"] == "missing":
        row["quality_flag"] = "reject_missing_wire_speed_feature"
    elif row["stickout_source"] == "missing":
        row["quality_flag"] = "reject_missing_stickout_feature"

    return row


def build_bead_level_dataset(topic_tables: dict[str, pd.DataFrame], cfg: Config, run_id: str) -> tuple[pd.DataFrame, dict[str, Any]]:
    validate_required_topics(topic_tables)

    geom_stable = filter_stable_window(
        topic_tables["/robin/weld_dimensions"], cfg.progression_min, cfg.progression_max
    )
    weld_stable = filter_stable_window(
        topic_tables["/robin/data/fronius"], cfg.progression_min, cfg.progression_max
    )

    geom, geom_bounds_removed = apply_physical_bounds(
        geom_stable,
        {
            "height_mm": (cfg.height_min_mm, cfg.height_max_mm),
            "width_mm": (cfg.width_min_mm, cfg.width_max_mm),
        },
    )
    weld, weld_bounds_removed = apply_physical_bounds(
        weld_stable,
        {
            "current": (cfg.current_min_A, cfg.current_max_A),
            "wire_feed_speed": (
                cfg.wire_feed_speed_min_mpm,
                cfg.wire_feed_speed_max_mpm,
            ),
        },
    )

    if cfg.enable_per_bead_iqr_filter:
        geom, geom_iqr_stats = apply_per_bead_iqr_filter(
            geom,
            value_columns=["height_mm", "width_mm"],
            bead_col="bead_id",
            iqr_multiplier=cfg.iqr_multiplier,
            min_samples_per_bead=cfg.iqr_min_samples_per_bead,
        )
        weld, weld_iqr_stats = apply_per_bead_iqr_filter(
            weld,
            value_columns=["current", "voltage", "wire_feed_speed", "power"],
            bead_col="bead_id",
            iqr_multiplier=cfg.iqr_multiplier,
            min_samples_per_bead=cfg.iqr_min_samples_per_bead,
        )
    else:
        geom_iqr_stats = {
            "rows_in": int(len(geom)),
            "rows_out": int(len(geom)),
            "rows_removed_total": 0,
            "rows_removed_by_column": {},
        }
        weld_iqr_stats = {
            "rows_in": int(len(weld)),
            "rows_out": int(len(weld)),
            "rows_removed_total": 0,
            "rows_removed_by_column": {},
        }

    active = summarize_active_bead(topic_tables["/robin/data/active_bead"])

    bead_ids = sorted(set(geom["bead_id"].dropna().unique()) | set(weld["bead_id"].dropna().unique()))
    stickout_doe_map = (
        load_doe_stickout_mapping(cfg.doe_csv_path, bead_ids)
        if cfg.allow_stickout_doe_fallback
        else {}
    )
    rows: list[dict[str, Any]] = []

    for bead_index, bead_id in enumerate(bead_ids, start=1):
        geom_bead = geom[geom["bead_id"] == bead_id]
        weld_bead = weld[weld["bead_id"] == bead_id]
        active_match = active[active["bead_id"] == bead_id]
        active_row = active_match.iloc[0] if not active_match.empty else None
        rows.append(
            aggregate_bead(
                bead_id=bead_id,
                run_id=run_id,
                bead_index=bead_index,
                geom_df=geom_bead,
                weld_df=weld_bead,
                active_row=active_row,
                doe_stickout_mm=stickout_doe_map.get(bead_id),
                cfg=cfg,
            )
        )

    bead_df = pd.DataFrame(rows)
    qc = {
        "run_id": run_id,
        "schema_version": cfg.schema_version,
        "feature_contract_version": cfg.feature_contract_version,
        "n_beads_total": int(len(bead_df)),
        "n_beads_ok": int((bead_df["quality_flag"] == "ok").sum()) if not bead_df.empty else 0,
        "n_beads_rejected": int((bead_df["quality_flag"] != "ok").sum()) if not bead_df.empty else 0,
        "wire_speed_source_counts": bead_df["wire_speed_source"].value_counts(dropna=False).to_dict() if not bead_df.empty else {},
        "stickout_source_counts": bead_df["stickout_source"].value_counts(dropna=False).to_dict() if not bead_df.empty else {},
        "reject_counts": bead_df["quality_flag"].value_counts(dropna=False).to_dict() if not bead_df.empty else {},
        "sample_filtering": {
            "iqr_enabled": bool(cfg.enable_per_bead_iqr_filter),
            "iqr_multiplier": float(cfg.iqr_multiplier),
            "iqr_min_samples_per_bead": int(cfg.iqr_min_samples_per_bead),
            "geometry_rows_stable_window": int(len(geom_stable)),
            "welder_rows_stable_window": int(len(weld_stable)),
            "geometry_rows_after_bounds": int(geom_iqr_stats["rows_in"]),
            "welder_rows_after_bounds": int(weld_iqr_stats["rows_in"]),
            "geometry_bounds_removed_by_column": geom_bounds_removed,
            "welder_bounds_removed_by_column": weld_bounds_removed,
            "geometry_iqr_stats": geom_iqr_stats,
            "welder_iqr_stats": weld_iqr_stats,
        },
    }
    return bead_df, qc


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--run-id", type=str, default="run_001")
    args = parser.parse_args()

    cfg = load_config(args.config)
    ensure_dirs(cfg)

    topic_tables = read_rosbag_topics(cfg.raw_bag_path)
    bead_df, qc = build_bead_level_dataset(topic_tables, cfg, run_id=args.run_id)

    bead_df.to_csv(cfg.gold_csv, index=False)
    cfg.qc_report_json.write_text(json.dumps(qc, indent=2))

    print(f"Wrote {cfg.gold_csv}")
    print(f"Wrote {cfg.qc_report_json}")


if __name__ == "__main__":
    main()
