from __future__ import annotations

import json
from pathlib import Path
from textwrap import dedent


ROOT = Path(__file__).resolve().parents[3]
NOTEBOOK_PATH = ROOT / "mlops" / "fragmented_datasets" / "notebooks" / "explore_fragmented_corrected_dataset.ipynb"


def markdown_cell(source: str) -> dict:
    return {
        "cell_type": "markdown",
        "metadata": {},
        "source": dedent(source).strip() + "\n",
    }


def code_cell(source: str) -> dict:
    return {
        "cell_type": "code",
        "execution_count": None,
        "metadata": {},
        "outputs": [],
        "source": dedent(source).strip() + "\n",
    }


def build_notebook() -> dict:
    cells = [
        markdown_cell(
            """
            # Fragmented Corrected-Input Exploration

            This notebook is the cleaned exploration view for the fragmented corrected-input dataset.

            It is intentionally centered on a single retained benchmark:
            - dataset view: selected gold rows
            - validation: leave-one-out
            - model: Torch MLP `[64, 32]`
            - optimizer: `LBFGS`
            - retained score: variance-weighted `R² = 0.8681`

            The notebook answers four concrete questions:
            - What does one row in the cleaned dataset mean?
            - Which part of the intended 48-point DOE space was actually executed?
            - How fragmented are the repeated attempts?
            - How does the retained Torch MLP perform on the selected gold rows?
            """
        ),
        code_cell(
            """
            from pathlib import Path
            import json
            import sys

            import numpy as np
            import pandas as pd
            import matplotlib.pyplot as plt
            from IPython.display import Markdown, display

            ROOT = Path.cwd()
            while ROOT != ROOT.parent and not (ROOT / "pyproject.toml").exists():
                ROOT = ROOT.parent
            if str(ROOT) not in sys.path:
                sys.path.insert(0, str(ROOT))

            from mlops.fragmented_datasets.evaluate_torch_selected_gold import (
                main as run_torch_selected_gold_evaluation,
            )

            plt.style.use("seaborn-v0_8-whitegrid")
            plt.rcParams.update(
                {
                    "figure.figsize": (12, 6),
                    "figure.dpi": 120,
                    "axes.spines.top": False,
                    "axes.spines.right": False,
                    "axes.titleweight": "bold",
                    "axes.labelsize": 11,
                }
            )
            pd.options.display.max_columns = 200
            pd.options.display.float_format = lambda value: f"{value:,.3f}"

            DATASET_ROOT = ROOT / "data" / "fragmented_doe_corrected"
            ATTEMPT_CSV = DATASET_ROOT / "silver" / "bead_level" / "attempt_level_dataset.csv"
            GOLD_CSV = DATASET_ROOT / "gold" / "train_dataset.csv"
            MAPPING_CSV = ROOT / "data" / "fragmented_doe_corrected" / "metadata" / "ros_doe_mapping_summary.csv"
            METRICS_JSON = ROOT / "data" / "comparisons" / "fragmented_corrected_torch_selected_gold" / "torch_selected_gold_metrics.json"
            PREDICTIONS_CSV = ROOT / "data" / "comparisons" / "fragmented_corrected_torch_selected_gold" / "torch_selected_gold_predictions.csv"
            COVERAGE_PNG = ROOT / "data" / "fragmented_doe_corrected" / "reports" / "desired_vs_executed_coverage.png"

            FEATURE_COLS = [
                "wire_feed_speed_mpm_model_input",
                "travel_speed_mps_model_input",
                "arc_length_correction_mm_model_input",
            ]
            TARGET_COLS = ["height_mm_target", "width_mm_target"]
            FEATURE_LABELS = {
                "wire_feed_speed_mpm_model_input": "Wire feed speed (m/min)",
                "travel_speed_mps_model_input": "Travel speed (m/s)",
                "arc_length_correction_mm_model_input": "Arc length correction (mm)",
            }
            TARGET_LABELS = {
                "height_mm_target": "Height (mm)",
                "width_mm_target": "Width (mm)",
            }


            def style_ax(ax, *, title=None, xlabel=None, ylabel=None):
                if title:
                    ax.set_title(title, pad=10)
                if xlabel:
                    ax.set_xlabel(xlabel)
                if ylabel:
                    ax.set_ylabel(ylabel)
                ax.grid(alpha=0.25)


            def add_identity_line(ax, x_values, y_values):
                low = float(min(np.min(x_values), np.min(y_values)))
                high = float(max(np.max(x_values), np.max(y_values)))
                pad = (high - low) * 0.06 if high > low else 0.1
                ax.plot([low - pad, high + pad], [low - pad, high + pad], "--", color="0.35", linewidth=1)
                ax.set_xlim(low - pad, high + pad)
                ax.set_ylim(low - pad, high + pad)
                ax.set_aspect("equal", adjustable="box")


            if not METRICS_JSON.exists() or not PREDICTIONS_CSV.exists():
                run_torch_selected_gold_evaluation()

            COVERAGE_PNG.parent.mkdir(parents=True, exist_ok=True)

            attempt_df = pd.read_csv(ATTEMPT_CSV)
            gold_df = pd.read_csv(GOLD_CSV)
            mapping_df = pd.read_csv(MAPPING_CSV)
            metrics = json.loads(METRICS_JSON.read_text())
            pred_df = pd.read_csv(PREDICTIONS_CSV)

            attempt_ok_df = attempt_df[attempt_df["quality_flag"] == "ok"].copy().reset_index(drop=True)
            selected_ok_df = gold_df[gold_df["quality_flag"] == "ok"].copy().reset_index(drop=True)
            coverage_df = mapping_df.merge(
                attempt_df[["attempt_id", "quality_flag"]],
                on="attempt_id",
                how="left",
            )
            coverage_df["execution_status"] = np.where(
                coverage_df["quality_flag"].eq("ok"),
                "usable",
                "non_usable",
            )

            display(Markdown(f"**Notebook root:** `{ROOT}`"))
            display(Markdown(f"**Dataset root:** `{DATASET_ROOT}`"))
            """
        ),
        markdown_cell(
            """
            ## What One Row Means

            In `data/fragmented_doe_corrected/silver/bead_level/attempt_level_dataset.csv`, one row is one cleaned deposition attempt.

            In `data/fragmented_doe_corrected/gold/train_dataset.csv`, one row is one selected cleaned attempt per DOE recipe.

            For each attempt, the geometry target is computed from the stable welding window and stored as:
            - `height_mm_target`
            - `width_mm_target`

            The retained model uses only three inputs:
            - `wire_feed_speed_mpm_model_input`
            - `travel_speed_mps_model_input`
            - `arc_length_correction_mm_model_input`
            """
        ),
        code_cell(
            """
            summary_df = pd.DataFrame(
                [
                    {"table": "attempt_level", "rows": len(attempt_df), "ok_rows": int((attempt_df["quality_flag"] == "ok").sum())},
                    {"table": "selected_gold", "rows": len(gold_df), "ok_rows": int((gold_df["quality_flag"] == "ok").sum())},
                ]
            )
            display(summary_df)

            selected_preview_df = selected_ok_df[
                [
                    "doe_input_id",
                    "attempt_id",
                    "bag_file",
                    "wire_feed_speed_mpm_model_input",
                    "travel_speed_mps_model_input",
                    "arc_length_correction_mm_model_input",
                    "height_mm_target",
                    "width_mm_target",
                ]
            ].sort_values("doe_input_id")
            display(selected_preview_df.head(10))
            """
        ),
        markdown_cell(
            """
            ## Desired DOE Coverage Versus Executed Coverage

            The intended 48-bead DOE comes through the canonical mapping manifest stored at `data/fragmented_doe_corrected/metadata/ros_doe_mapping_summary.csv`, built from the DOE definition in `mlops/fragmented_datasets/DOE_LHS_48.json`.

            In the plots below:
            - hollow gray circles are intended DOE points
            - orange points are executed attempts that survived cleaning
            - red `x` points are executed attempts that did not survive cleaning
            - faint line segments show the drift from intended point to executed point
            """
        ),
        code_cell(
            """
            pair_specs = [
                (
                    "doe_wire_feed_speed",
                    "doe_weld_speed",
                    "wire_feed_speed_cmd_ros",
                    "weld_speed_cmd_ros",
                    "Wire feed speed (m/min)",
                    "Travel speed (m/s)",
                ),
                (
                    "doe_wire_feed_speed",
                    "doe_arc_length_correction_mm",
                    "wire_feed_speed_cmd_ros",
                    "arc_length_correction_cmd_ros",
                    "Wire feed speed (m/min)",
                    "Arc length correction (mm)",
                ),
                (
                    "doe_weld_speed",
                    "doe_arc_length_correction_mm",
                    "weld_speed_cmd_ros",
                    "arc_length_correction_cmd_ros",
                    "Travel speed (m/s)",
                    "Arc length correction (mm)",
                ),
            ]

            fig, axes = plt.subplots(1, 3, figsize=(19, 5.5), constrained_layout=True)

            usable_df = coverage_df[coverage_df["execution_status"] == "usable"].copy()
            rejected_df = coverage_df[coverage_df["execution_status"] != "usable"].copy()

            for ax, (doe_x, doe_y, exec_x, exec_y, xlabel, ylabel) in zip(axes, pair_specs):
                ax.scatter(
                    coverage_df[doe_x],
                    coverage_df[doe_y],
                    facecolors="none",
                    edgecolors="0.55",
                    linewidth=1.0,
                    s=80,
                    label="Desired DOE",
                )
                for row in coverage_df.itertuples(index=False):
                    ax.plot(
                        [getattr(row, doe_x), getattr(row, exec_x)],
                        [getattr(row, doe_y), getattr(row, exec_y)],
                        color="0.75",
                        linewidth=0.8,
                        alpha=0.45,
                    )
                ax.scatter(
                    usable_df[exec_x],
                    usable_df[exec_y],
                    color="#ea580c",
                    edgecolor="white",
                    linewidth=0.6,
                    s=70,
                    label="Executed usable",
                    zorder=3,
                )
                ax.scatter(
                    rejected_df[exec_x],
                    rejected_df[exec_y],
                    color="#b91c1c",
                    marker="x",
                    linewidth=1.3,
                    s=60,
                    label="Executed non-usable",
                    zorder=4,
                )
                style_ax(ax, xlabel=xlabel, ylabel=ylabel)

            axes[0].legend(loc="best")
            fig.suptitle("Intended DOE Coverage Versus Executed Attempt Coverage", fontsize=15, weight="bold")
            fig.savefig(COVERAGE_PNG, bbox_inches="tight")
            plt.show()

            drift_df = coverage_df.assign(
                wire_feed_abs_drift=(coverage_df["wire_feed_speed_cmd_ros"] - coverage_df["doe_wire_feed_speed"]).abs(),
                travel_abs_drift=(coverage_df["weld_speed_cmd_ros"] - coverage_df["doe_weld_speed"]).abs(),
                arc_abs_drift=(coverage_df["arc_length_correction_cmd_ros"] - coverage_df["doe_arc_length_correction_mm"]).abs(),
            )
            display(
                drift_df[
                    [
                        "input_id",
                        "attempt_id",
                        "execution_status",
                        "wire_feed_abs_drift",
                        "travel_abs_drift",
                        "arc_abs_drift",
                    ]
                ]
                .sort_values(["travel_abs_drift", "wire_feed_abs_drift", "arc_abs_drift"], ascending=False)
                .head(10)
            )
            display(Markdown(f"Coverage figure saved to `{COVERAGE_PNG}`"))
            """
        ),
        markdown_cell(
            """
            ## Fragmentation Across Repeated Attempts

            The selected gold table keeps one representative row per recipe, but the attempt-level table still shows how much repeated recipes vary before selection.
            """
        ),
        code_cell(
            """
            spread_df = (
                attempt_ok_df.groupby("doe_input_id")
                .agg(
                    n_attempts=("attempt_id", "size"),
                    height_mean=("height_mm_target", "mean"),
                    height_std=("height_mm_target", "std"),
                    width_mean=("width_mm_target", "mean"),
                    width_std=("width_mm_target", "std"),
                )
                .reset_index()
            )
            spread_df[["height_std", "width_std"]] = spread_df[["height_std", "width_std"]].fillna(0.0)
            spread_df["combined_std"] = spread_df["height_std"] + spread_df["width_std"]
            spread_df = spread_df.sort_values(["n_attempts", "combined_std"], ascending=[False, False]).reset_index(drop=True)

            fig, axes = plt.subplots(1, 2, figsize=(18, 6), constrained_layout=True)

            count_plot_df = spread_df.sort_values(["n_attempts", "doe_input_id"], ascending=[False, True])
            axes[0].bar(count_plot_df["doe_input_id"], count_plot_df["n_attempts"], color="#2563eb")
            style_ax(axes[0], title="Usable Attempt Count Per Recipe", xlabel="DOE recipe", ylabel="usable attempts")
            axes[0].tick_params(axis="x", rotation=90)

            axes[1].scatter(
                spread_df["width_std"],
                spread_df["height_std"],
                s=90 + 18 * spread_df["n_attempts"],
                color="#0f766e",
                edgecolor="white",
                linewidth=0.8,
            )
            for row in spread_df.sort_values("combined_std", ascending=False).head(8).itertuples(index=False):
                axes[1].annotate(
                    row.doe_input_id,
                    (row.width_std, row.height_std),
                    xytext=(5, 5),
                    textcoords="offset points",
                    fontsize=9,
                )
            style_ax(
                axes[1],
                title="Recipe-Level Spread Across Repeated Attempts",
                xlabel="Width std across attempts (mm)",
                ylabel="Height std across attempts (mm)",
            )

            plt.show()

            display(
                spread_df[
                    [
                        "doe_input_id",
                        "n_attempts",
                        "height_mean",
                        "height_std",
                        "width_mean",
                        "width_std",
                    ]
                ]
                .sort_values(["width_std", "height_std"], ascending=False)
                .head(10)
            )
            """
        ),
        markdown_cell(
            """
            ## Selected Gold Response View

            This is the actual training table behind the retained benchmark: one selected cleaned attempt per DOE recipe.
            """
        ),
        code_cell(
            """
            fig, axes = plt.subplots(2, 3, figsize=(18, 10), constrained_layout=True)

            colors = {
                "height_mm_target": "#0f766e",
                "width_mm_target": "#c2410c",
            }

            for row_idx, target_col in enumerate(TARGET_COLS):
                for col_idx, feature_col in enumerate(FEATURE_COLS):
                    ax = axes[row_idx, col_idx]
                    ax.scatter(
                        selected_ok_df[feature_col],
                        selected_ok_df[target_col],
                        color=colors[target_col],
                        edgecolor="white",
                        linewidth=0.7,
                        s=90,
                    )
                    style_ax(
                        ax,
                        title=f"{TARGET_LABELS[target_col]} vs {FEATURE_LABELS[feature_col]}",
                        xlabel=FEATURE_LABELS[feature_col],
                        ylabel=TARGET_LABELS[target_col],
                    )

            plt.show()

            corr_df = selected_ok_df[FEATURE_COLS + TARGET_COLS].corr().loc[FEATURE_COLS, TARGET_COLS]
            corr_df = corr_df.rename(index=FEATURE_LABELS, columns=TARGET_LABELS)
            display(corr_df)
            """
        ),
        markdown_cell(
            """
            ## Retained Torch MLP Benchmark

            This section reads the canonical output of `mlops/fragmented_datasets/evaluate_torch_selected_gold.py`.
            No alternate model families are compared here.
            """
        ),
        code_cell(
            """
            model_summary_df = pd.DataFrame(
                [
                    {"metric": "variance_weighted_r2", "value": metrics["r2_variance_weighted"]},
                    {"metric": "height_r2", "value": metrics["targets"]["height_mm_target"]["r2"]},
                    {"metric": "width_r2", "value": metrics["targets"]["width_mm_target"]["r2"]},
                    {"metric": "height_rmse_mm", "value": metrics["targets"]["height_mm_target"]["rmse"]},
                    {"metric": "width_rmse_mm", "value": metrics["targets"]["width_mm_target"]["rmse"]},
                    {"metric": "height_mae_mm", "value": metrics["targets"]["height_mm_target"]["mae"]},
                    {"metric": "width_mae_mm", "value": metrics["targets"]["width_mm_target"]["mae"]},
                ]
            )
            config_df = pd.DataFrame(
                [
                    {"parameter": "hidden_dims", "value": str(metrics["model"]["hidden_dims"])},
                    {"parameter": "optimizer", "value": metrics["model"]["optimizer"]},
                    {"parameter": "weight_decay", "value": metrics["model"]["weight_decay"]},
                    {"parameter": "max_iter", "value": metrics["model"]["max_iter"]},
                    {"parameter": "seed", "value": metrics["model"]["random_seed"]},
                    {"parameter": "n_rows", "value": metrics["n_rows"]},
                ]
            )

            display(config_df)
            display(model_summary_df)
            """
        ),
        code_cell(
            """
            fig, axes = plt.subplots(1, 2, figsize=(14, 6), constrained_layout=True)

            target_colors = {
                "height_mm_target": "#0f766e",
                "width_mm_target": "#c2410c",
            }

            for ax, target_name in zip(axes, TARGET_COLS):
                actual_col = f"{target_name}_actual"
                pred_col = f"{target_name}_pred"
                error_col = f"{target_name}_abs_error"
                scatter = ax.scatter(
                    pred_df[actual_col],
                    pred_df[pred_col],
                    c=pred_df[error_col],
                    cmap="viridis",
                    s=120,
                    edgecolor="white",
                    linewidth=0.7,
                )
                add_identity_line(ax, pred_df[actual_col], pred_df[pred_col])
                worst_df = pred_df.sort_values(error_col, ascending=False).head(5)
                for row in worst_df.itertuples(index=False):
                    ax.annotate(
                        row.doe_input_id,
                        (getattr(row, actual_col), getattr(row, pred_col)),
                        xytext=(5, 5),
                        textcoords="offset points",
                        fontsize=9,
                    )
                style_ax(
                    ax,
                    title=f"Leave-One-Out Prediction: {TARGET_LABELS[target_name]}",
                    xlabel="Actual",
                    ylabel="Predicted",
                )
                cbar = plt.colorbar(scatter, ax=ax, shrink=0.82)
                cbar.set_label("Absolute error")

            plt.show()

            error_table_df = pred_df.assign(
                total_abs_error=pred_df["height_mm_target_abs_error"] + pred_df["width_mm_target_abs_error"]
            ).sort_values("total_abs_error", ascending=False)
            display(
                error_table_df[
                    [
                        "doe_input_id",
                        "attempt_id",
                        "height_mm_target_actual",
                        "height_mm_target_pred",
                        "height_mm_target_abs_error",
                        "width_mm_target_actual",
                        "width_mm_target_pred",
                        "width_mm_target_abs_error",
                        "total_abs_error",
                    ]
                ].head(10)
            )
            """
        ),
    ]

    return {
        "cells": cells,
        "metadata": {
            "kernelspec": {
                "display_name": "Python 3",
                "language": "python",
                "name": "python3",
            },
            "language_info": {
                "name": "python",
                "version": "3.11",
            },
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }


def main() -> None:
    NOTEBOOK_PATH.write_text(json.dumps(build_notebook(), indent=2))
    print(f"Wrote {NOTEBOOK_PATH}")


if __name__ == "__main__":
    main()
