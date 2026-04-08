from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
PARENT_PREDICTIONS_CSV = (
    ROOT
    / "data"
    / "comparisons"
    / "unified_legacy_arc0_plus_fragmented_chunk4_progression_torch_grouped"
    / "parent_predictions.csv"
)
METRICS_JSON = (
    ROOT
    / "data"
    / "comparisons"
    / "unified_legacy_arc0_plus_fragmented_chunk4_progression_torch_grouped"
    / "metrics.json"
)
OUTDIR = (
    ROOT
    / "data"
    / "comparisons"
    / "unified_legacy_arc0_plus_fragmented_chunk4_progression_torch_grouped"
)
OUT_CSV = OUTDIR / "parent_error_contributions.csv"
OUT_TOP_CSV = OUTDIR / "worst_parent_rows.csv"
OUT_PNG = OUTDIR / "parent_error_scatter.png"
OUT_README = OUTDIR / "parent_error_summary.md"

ACTUAL_COLUMNS = [
    "height_mm_target_parent_actual",
    "width_mm_target_parent_actual",
]
PRED_COLUMNS = [
    "height_mm_target_parent_pred_from_chunk_mean",
    "width_mm_target_parent_pred_from_chunk_mean",
]


def load_inputs() -> tuple[pd.DataFrame, dict[str, object]]:
    parent_df = pd.read_csv(PARENT_PREDICTIONS_CSV)
    metrics = json.loads(METRICS_JSON.read_text())
    return parent_df, metrics


def build_error_table(parent_df: pd.DataFrame, metrics: dict[str, object]) -> pd.DataFrame:
    df = parent_df.copy()
    df["height_error_mm"] = df[PRED_COLUMNS[0]] - df[ACTUAL_COLUMNS[0]]
    df["width_error_mm"] = df[PRED_COLUMNS[1]] - df[ACTUAL_COLUMNS[1]]
    df["height_abs_error_mm"] = df["height_error_mm"].abs()
    df["width_abs_error_mm"] = df["width_error_mm"].abs()
    df["parent_rmse_mm"] = np.sqrt(
        (df["height_error_mm"] ** 2 + df["width_error_mm"] ** 2) / 2.0
    )
    df["parent_sse_total"] = df["height_error_mm"] ** 2 + df["width_error_mm"] ** 2

    total_sst = 0.0
    for actual_col in ACTUAL_COLUMNS:
        centered = df[actual_col] - df[actual_col].mean()
        total_sst += float((centered**2).sum())

    grouped_r2 = float(metrics["parent_reconstruction_from_chunk_mean"]["r2_variance_weighted"])
    total_r2_drop = 1.0 - grouped_r2

    df["r2_drop_contribution"] = df["parent_sse_total"] / total_sst
    df["share_of_total_r2_drop_pct"] = 100.0 * df["r2_drop_contribution"] / total_r2_drop
    df["error_rank"] = (
        df["r2_drop_contribution"].rank(method="first", ascending=False).astype(int)
    )
    return df.sort_values("r2_drop_contribution", ascending=False).reset_index(drop=True)


def add_panel(
    ax: plt.Axes,
    df: pd.DataFrame,
    color_column: str,
    title: str,
    colorbar_label: str,
) -> None:
    markers = {
        "fragmented_doe_corrected_selected_gold": "o",
        "legacy_robin_data_01_arc0_assumed": "^",
    }
    plotted = []
    scatter = None
    vmin = float(df[color_column].min())
    vmax = float(df[color_column].max())
    for source_dataset, marker in markers.items():
        subset = df[df["source_dataset"] == source_dataset]
        if subset.empty:
            continue
        scatter = ax.scatter(
            subset["width_mm_target_parent_actual"],
            subset["height_mm_target_parent_actual"],
            c=subset[color_column],
            cmap="viridis",
            s=70,
            marker=marker,
            edgecolors="black",
            linewidths=0.4,
            vmin=vmin,
            vmax=vmax,
        )
        plotted.append((source_dataset, marker))

    top = df.sort_values("error_rank").head(12)
    for row in top.itertuples(index=False):
        label = str(row.source_recipe_id)
        ax.annotate(
            label,
            (
                getattr(row, "width_mm_target_parent_actual"),
                getattr(row, "height_mm_target_parent_actual"),
            ),
            xytext=(5, 4),
            textcoords="offset points",
            fontsize=8,
        )

    ax.set_title(title)
    ax.set_xlabel("Actual Width (mm)")
    ax.set_ylabel("Actual Height (mm)")
    ax.grid(alpha=0.2)
    if scatter is not None:
        colorbar = plt.colorbar(scatter, ax=ax)
        colorbar.set_label(colorbar_label)
    legend_handles = [
        plt.Line2D(
            [0],
            [0],
            marker=marker,
            linestyle="",
            markerfacecolor="lightgray",
            markeredgecolor="black",
            markersize=7,
            label=dataset.replace("_", " "),
        )
        for dataset, marker in plotted
    ]
    if legend_handles:
        ax.legend(handles=legend_handles, loc="best", fontsize=8)


def write_plot(df: pd.DataFrame) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.8), constrained_layout=True)
    add_panel(
        axes[0],
        df,
        color_column="parent_rmse_mm",
        title="Parent Rows Colored By RMSE",
        colorbar_label="Parent RMSE (mm)",
    )
    add_panel(
        axes[1],
        df,
        color_column="share_of_total_r2_drop_pct",
        title="Parent Rows Colored By R2 Drop Share",
        colorbar_label="Share Of Total R2 Drop (%)",
    )
    fig.suptitle(
        "Chunked Progression Torch Model: Rows Driving The Remaining Error",
        fontsize=13,
    )
    fig.savefig(OUT_PNG, dpi=180, bbox_inches="tight")
    plt.close(fig)


def write_summary(df: pd.DataFrame, metrics: dict[str, object]) -> None:
    top = df.head(10).copy()
    lines = [
        "# Parent Error Summary",
        "",
        "This analysis uses the best current grouped benchmark:",
        f"- parent mean variance-weighted R2: `{metrics['parent_reconstruction_from_chunk_mean']['r2_variance_weighted']:.4f}`",
        "",
        "Per-row R2 is not defined, so the ranking uses:",
        "- `parent_rmse_mm`",
        "- `r2_drop_contribution`, the row's squared-error contribution to the remaining global R2 drop",
        "",
        "Top rows by contribution to the remaining R2 drop:",
    ]
    for row in top.itertuples(index=False):
        lines.append(
            f"- `{row.source_recipe_id}` ({row.source_dataset}): RMSE `{row.parent_rmse_mm:.3f}` mm, "
            f"share of total R2 drop `{row.share_of_total_r2_drop_pct:.1f}%`"
        )
    OUT_README.write_text("\n".join(lines) + "\n")


def main() -> None:
    OUTDIR.mkdir(parents=True, exist_ok=True)
    parent_df, metrics = load_inputs()
    error_df = build_error_table(parent_df, metrics)
    error_df.to_csv(OUT_CSV, index=False)
    error_df.head(15).to_csv(OUT_TOP_CSV, index=False)
    write_plot(error_df)
    write_summary(error_df, metrics)
    print(f"Wrote {OUT_CSV}")
    print(f"Wrote {OUT_TOP_CSV}")
    print(f"Wrote {OUT_PNG}")
    print(f"Wrote {OUT_README}")


if __name__ == "__main__":
    main()
