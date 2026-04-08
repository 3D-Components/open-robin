from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from joblib import dump
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from sklearn.model_selection import LeaveOneOut
from sklearn.preprocessing import StandardScaler


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_GOLD_CSV = ROOT / "data" / "fragmented_doe_corrected" / "gold" / "train_dataset.csv"
DEFAULT_OUTDIR = ROOT / "data" / "comparisons" / "fragmented_corrected_torch_selected_gold"
DEFAULT_MODEL_DIR = ROOT / "data" / "models" / "fragmented_doe_corrected" / "selected_gold_torch_mlp"
DEFAULT_MODEL_PATH = DEFAULT_MODEL_DIR / "torch_selected_gold_model.pt"
DEFAULT_SCALER_PATH = DEFAULT_MODEL_DIR / "torch_selected_gold_feature_scaler.joblib"

FEATURE_COLUMNS = [
    "wire_feed_speed_mpm_model_input",
    "travel_speed_mps_model_input",
    "arc_length_correction_mm_model_input",
]
TARGET_COLUMNS = ["height_mm_target", "width_mm_target"]

MODEL_CONFIG = {
    "hidden_dims": [64, 32],
    "weight_decay": 1e-4,
    "max_iter": 215,
    "random_seed": 6,
    "optimizer": "LBFGS",
}


class TorchMLPRegressor(nn.Module):
    def __init__(self, input_dim: int, output_dim: int, hidden_dims: list[int]) -> None:
        super().__init__()
        layers: list[nn.Module] = []
        in_dim = input_dim
        for hidden_dim in hidden_dims:
            layers.append(nn.Linear(in_dim, hidden_dim))
            layers.append(nn.ReLU())
            in_dim = hidden_dim
        layers.append(nn.Linear(in_dim, output_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate a Torch selected-gold MLP with leave-one-out.")
    parser.add_argument("--dataset-csv", type=Path, default=DEFAULT_GOLD_CSV)
    parser.add_argument("--outdir", type=Path, default=DEFAULT_OUTDIR)
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR)
    parser.add_argument("--dataset-label", type=str, default="fragmented corrected-input selected gold")
    return parser.parse_args()


def load_selected_gold(dataset_csv: Path) -> pd.DataFrame:
    df = pd.read_csv(dataset_csv)
    df = df[df["quality_flag"] == "ok"].copy()
    df = df.dropna(subset=FEATURE_COLUMNS + TARGET_COLUMNS).reset_index(drop=True)
    if len(df) < 3:
        raise ValueError("Need at least 3 usable selected-gold rows for Torch LOO evaluation.")
    return df


def fit_fold_lbfgs(
    X_train: np.ndarray,
    y_train: np.ndarray,
    X_test: np.ndarray,
    *,
    seed: int,
    hidden_dims: list[int],
    weight_decay: float,
    max_iter: int,
) -> np.ndarray:
    set_seed(seed)
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train).astype(np.float32)
    X_test_scaled = scaler.transform(X_test).astype(np.float32)

    x_train_tensor = torch.from_numpy(X_train_scaled)
    y_train_tensor = torch.from_numpy(y_train.astype(np.float32))
    x_test_tensor = torch.from_numpy(X_test_scaled)

    model = TorchMLPRegressor(
        input_dim=X_train_scaled.shape[1],
        output_dim=y_train.shape[1],
        hidden_dims=hidden_dims,
    )
    optimizer = torch.optim.LBFGS(
        model.parameters(),
        lr=1.0,
        max_iter=max_iter,
        history_size=50,
        line_search_fn="strong_wolfe",
    )
    loss_fn = nn.MSELoss()

    def closure() -> torch.Tensor:
        optimizer.zero_grad()
        pred = model(x_train_tensor)
        loss = loss_fn(pred, y_train_tensor)
        if weight_decay > 0:
            l2 = sum((param**2).sum() for param in model.parameters())
            loss = loss + (weight_decay * l2 / len(X_train_scaled))
        loss.backward()
        return loss

    optimizer.step(closure)
    model.eval()
    with torch.no_grad():
        pred = model(x_test_tensor).cpu().numpy()
    return pred


def fit_full_lbfgs(
    X: np.ndarray,
    y: np.ndarray,
    *,
    seed: int,
    hidden_dims: list[int],
    weight_decay: float,
    max_iter: int,
) -> tuple[TorchMLPRegressor, StandardScaler]:
    set_seed(seed)
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X).astype(np.float32)

    x_tensor = torch.from_numpy(X_scaled)
    y_tensor = torch.from_numpy(y.astype(np.float32))

    model = TorchMLPRegressor(
        input_dim=X_scaled.shape[1],
        output_dim=y.shape[1],
        hidden_dims=hidden_dims,
    )
    optimizer = torch.optim.LBFGS(
        model.parameters(),
        lr=1.0,
        max_iter=max_iter,
        history_size=50,
        line_search_fn="strong_wolfe",
    )
    loss_fn = nn.MSELoss()

    def closure() -> torch.Tensor:
        optimizer.zero_grad()
        pred = model(x_tensor)
        loss = loss_fn(pred, y_tensor)
        if weight_decay > 0:
            l2 = sum((param**2).sum() for param in model.parameters())
            loss = loss + (weight_decay * l2 / len(X_scaled))
        loss.backward()
        return loss

    optimizer.step(closure)
    model.eval()
    return model, scaler


def evaluate_loo(df: pd.DataFrame, dataset_csv: Path, dataset_label: str) -> tuple[pd.DataFrame, dict[str, object]]:
    X = df[FEATURE_COLUMNS].to_numpy(dtype=np.float32)
    y = df[TARGET_COLUMNS].to_numpy(dtype=np.float32)
    preds = np.zeros_like(y, dtype=np.float32)

    for fold_idx, (train_idx, test_idx) in enumerate(LeaveOneOut().split(X), start=1):
        pred = fit_fold_lbfgs(
            X[train_idx],
            y[train_idx],
            X[test_idx],
            seed=int(MODEL_CONFIG["random_seed"]),
            hidden_dims=MODEL_CONFIG["hidden_dims"],
            weight_decay=float(MODEL_CONFIG["weight_decay"]),
            max_iter=int(MODEL_CONFIG["max_iter"]),
        )
        preds[test_idx] = pred

    pred_df = df[["doe_input_id", "attempt_id"]].copy()
    for col_idx, target_name in enumerate(TARGET_COLUMNS):
        pred_df[f"{target_name}_actual"] = y[:, col_idx]
        pred_df[f"{target_name}_pred"] = preds[:, col_idx]
        pred_df[f"{target_name}_abs_error"] = np.abs(preds[:, col_idx] - y[:, col_idx])

    summary = {
        "dataset_csv": str(dataset_csv),
        "dataset_label": dataset_label,
        "n_rows": int(len(df)),
        "feature_columns": FEATURE_COLUMNS,
        "target_columns": TARGET_COLUMNS,
        "model": MODEL_CONFIG,
        "r2_variance_weighted": float(r2_score(y, preds, multioutput="variance_weighted")),
        "targets": {},
    }
    for col_idx, target_name in enumerate(TARGET_COLUMNS):
        actual = y[:, col_idx]
        pred = preds[:, col_idx]
        summary["targets"][target_name] = {
            "r2": float(r2_score(actual, pred)),
            "rmse": float(np.sqrt(mean_squared_error(actual, pred))),
            "mae": float(mean_absolute_error(actual, pred)),
        }
    return pred_df, summary


def save_refit_artifact(
    df: pd.DataFrame,
    summary: dict[str, object],
    *,
    dataset_csv: Path,
    model_path: Path,
    scaler_path: Path,
) -> dict[str, object]:
    X = df[FEATURE_COLUMNS].to_numpy(dtype=np.float32)
    y = df[TARGET_COLUMNS].to_numpy(dtype=np.float32)
    model, scaler = fit_full_lbfgs(
        X,
        y,
        seed=int(MODEL_CONFIG["random_seed"]),
        hidden_dims=MODEL_CONFIG["hidden_dims"],
        weight_decay=float(MODEL_CONFIG["weight_decay"]),
        max_iter=int(MODEL_CONFIG["max_iter"]),
    )
    X_scaled = scaler.transform(X).astype(np.float32)
    with torch.no_grad():
        pred = model(torch.from_numpy(X_scaled)).cpu().numpy()

    model_path.parent.mkdir(parents=True, exist_ok=True)
    scaler_path.parent.mkdir(parents=True, exist_ok=True)
    dump(scaler, scaler_path)
    torch.save(
        {
            "schema": "fragmented_corrected_torch_selected_gold_v1",
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
            "dataset_csv": str(dataset_csv),
            "n_rows_refit": int(len(df)),
            "scaler_path": str(scaler_path),
        },
        model_path,
    )

    refit_metrics = {
        "n_rows": int(len(df)),
        "r2_variance_weighted_train": float(r2_score(y, pred, multioutput="variance_weighted")),
        "targets": {},
    }
    for col_idx, target_name in enumerate(TARGET_COLUMNS):
        actual = y[:, col_idx]
        fitted = pred[:, col_idx]
        refit_metrics["targets"][target_name] = {
            "r2_train": float(r2_score(actual, fitted)),
            "rmse_train": float(np.sqrt(mean_squared_error(actual, fitted))),
            "mae_train": float(mean_absolute_error(actual, fitted)),
        }

    summary["saved_model_artifact"] = {
        "model_path": str(model_path),
        "scaler_path": str(scaler_path),
        "refit_note": "Saved artifact is refit on all selected-gold rows; benchmark metrics above remain leave-one-out.",
        "refit_train_metrics": refit_metrics,
    }
    return summary


def write_readme(outdir: Path, summary: dict[str, object], model_path: Path, scaler_path: Path) -> None:
    height = summary["targets"]["height_mm_target"]
    width = summary["targets"]["width_mm_target"]
    model = summary["model"]
    lines = [
        "# Torch Selected-Gold MLP Evaluation",
        "",
        f"This is a Torch leave-one-out evaluation for `{summary['dataset_label']}`.",
        "",
        f"- dataset: `{summary['dataset_csv']}`",
        f"- usable rows: `{summary['n_rows']}`",
        "- validation: leave-one-out on the selected gold rows",
        f"- model: Torch MLP with hidden layers `{model['hidden_dims']}`",
        f"- optimizer: `{model['optimizer']}`",
        f"- weight decay: `{model['weight_decay']}`",
        f"- max iterations per fold: `{model['max_iter']}`",
        f"- constant initialization seed: `{model['random_seed']}`",
        "",
        f"- variance-weighted R2: `{summary['r2_variance_weighted']:.4f}`",
        f"- height R2: `{height['r2']:.4f}`",
        f"- width R2: `{width['r2']:.4f}`",
        "",
        "Artifacts:",
        "",
        "- `torch_selected_gold_metrics.json`",
        "- `torch_selected_gold_predictions.csv`",
        f"- comparison dir: `{outdir}`",
        f"- saved model: `{model_path}`",
        f"- saved scaler: `{scaler_path}`",
        "",
        f"The saved model artifact is a refit on all `{summary['n_rows']}` usable selected-gold rows.",
        "The reported R2 benchmark remains the leave-one-out score above.",
        "",
    ]
    (outdir / "README.md").write_text("\n".join(lines))


def main() -> None:
    args = parse_args()
    outdir = args.outdir
    model_dir = args.model_dir
    model_path = model_dir / "torch_selected_gold_model.pt"
    scaler_path = model_dir / "torch_selected_gold_feature_scaler.joblib"

    outdir.mkdir(parents=True, exist_ok=True)
    model_dir.mkdir(parents=True, exist_ok=True)
    df = load_selected_gold(args.dataset_csv)
    pred_df, summary = evaluate_loo(df, args.dataset_csv, args.dataset_label)
    summary = save_refit_artifact(
        df,
        summary,
        dataset_csv=args.dataset_csv,
        model_path=model_path,
        scaler_path=scaler_path,
    )

    pred_df.to_csv(outdir / "torch_selected_gold_predictions.csv", index=False)
    (outdir / "torch_selected_gold_metrics.json").write_text(json.dumps(summary, indent=2))
    write_readme(outdir, summary, model_path, scaler_path)

    print(f"Wrote {outdir / 'torch_selected_gold_predictions.csv'}")
    print(f"Wrote {outdir / 'torch_selected_gold_metrics.json'}")
    print(f"Wrote {model_path}")
    print(f"Wrote {scaler_path}")
    print(f"Wrote {outdir / 'README.md'}")


if __name__ == "__main__":
    main()
