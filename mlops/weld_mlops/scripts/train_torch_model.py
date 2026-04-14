"""Train a bead-level multi-output regressor with PyTorch.

This pipeline expects the gold dataset produced by extract_rosbag_to_csv.py
and applies feature scaling before training.
"""

from __future__ import annotations

import argparse
import copy
import json
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from joblib import dump
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import yaml


@dataclass
class TrainConfig:
    dataset_csv: Path
    feature_columns: list[str]
    target_columns: list[str]
    quality_flag_ok_value: str
    val_fraction: float
    random_seed: int
    batch_size: int
    epochs: int
    learning_rate: float
    weight_decay: float
    hidden_dims: list[int]
    dropout: float
    patience: int
    device: str
    artifacts_dir: Path
    model_path: Path
    scaler_path: Path
    metrics_path: Path


def load_config(path: Path) -> TrainConfig:
    data = yaml.safe_load(path.read_text())
    return TrainConfig(
        dataset_csv=Path(data["dataset"]["csv_path"]),
        feature_columns=list(data["dataset"]["feature_columns"]),
        target_columns=list(data["dataset"]["target_columns"]),
        quality_flag_ok_value=str(data["dataset"].get("quality_flag_ok_value", "ok")),
        val_fraction=float(data["split"]["val_fraction"]),
        random_seed=int(data["split"]["random_seed"]),
        batch_size=int(data["training"]["batch_size"]),
        epochs=int(data["training"]["epochs"]),
        learning_rate=float(data["training"]["learning_rate"]),
        weight_decay=float(data["training"]["weight_decay"]),
        hidden_dims=[int(x) for x in data["training"]["hidden_dims"]],
        dropout=float(data["training"]["dropout"]),
        patience=int(data["training"]["patience"]),
        device=str(data["training"].get("device", "cpu")),
        artifacts_dir=Path(data["outputs"]["artifacts_dir"]),
        model_path=Path(data["outputs"]["model_path"]),
        scaler_path=Path(data["outputs"]["scaler_path"]),
        metrics_path=Path(data["outputs"]["metrics_path"]),
    )


def ensure_dirs(cfg: TrainConfig) -> None:
    cfg.artifacts_dir.mkdir(parents=True, exist_ok=True)
    cfg.model_path.parent.mkdir(parents=True, exist_ok=True)
    cfg.scaler_path.parent.mkdir(parents=True, exist_ok=True)
    cfg.metrics_path.parent.mkdir(parents=True, exist_ok=True)


def set_reproducible_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


class MLPRegressor(nn.Module):
    def __init__(self, input_dim: int, output_dim: int, hidden_dims: list[int], dropout: float) -> None:
        super().__init__()
        layers: list[nn.Module] = []
        in_dim = input_dim
        for hidden_dim in hidden_dims:
            layers.append(nn.Linear(in_dim, hidden_dim))
            layers.append(nn.ReLU())
            if dropout > 0:
                layers.append(nn.Dropout(dropout))
            in_dim = hidden_dim
        layers.append(nn.Linear(in_dim, output_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def prepare_data(cfg: TrainConfig) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    if not cfg.dataset_csv.exists():
        raise FileNotFoundError(f"Dataset CSV not found: {cfg.dataset_csv}")

    df = pd.read_csv(cfg.dataset_csv)
    summary: dict[str, Any] = {
        "dataset_csv": str(cfg.dataset_csv),
        "n_rows_raw": int(len(df)),
    }

    missing_cols = [c for c in cfg.feature_columns + cfg.target_columns if c not in df.columns]
    if missing_cols:
        raise ValueError(f"Missing required columns in dataset: {missing_cols}")

    if "quality_flag" in df.columns:
        df = df[df["quality_flag"] == cfg.quality_flag_ok_value].copy()
    summary["n_rows_after_quality_filter"] = int(len(df))

    before_dropna = len(df)
    df = df.dropna(subset=cfg.feature_columns + cfg.target_columns).copy()
    summary["n_rows_after_dropna"] = int(len(df))
    summary["n_rows_dropped_missing"] = int(before_dropna - len(df))

    if len(df) < 2:
        raise ValueError("Need at least 2 valid rows after filtering to train/validate.")

    x = df[cfg.feature_columns].to_numpy(dtype=np.float32)
    y = df[cfg.target_columns].to_numpy(dtype=np.float32)

    val_size = max(1, int(round(len(df) * cfg.val_fraction)))
    val_size = min(val_size, len(df) - 1)

    x_train, x_val, y_train, y_val = train_test_split(
        x,
        y,
        test_size=val_size,
        random_state=cfg.random_seed,
        shuffle=True,
    )
    summary["n_rows_train"] = int(len(x_train))
    summary["n_rows_val"] = int(len(x_val))
    return x_train, x_val, y_train, y_val, summary


def metrics_by_target(y_true: np.ndarray, y_pred: np.ndarray, target_columns: list[str]) -> dict[str, Any]:
    out: dict[str, Any] = {"n_samples": int(len(y_true)), "targets": {}}
    for idx, target_name in enumerate(target_columns):
        true_col = y_true[:, idx]
        pred_col = y_pred[:, idx]
        mse = mean_squared_error(true_col, pred_col)
        r2 = float(r2_score(true_col, pred_col)) if len(true_col) > 1 else float("nan")
        out["targets"][target_name] = {
            "mae": float(mean_absolute_error(true_col, pred_col)),
            "rmse": float(np.sqrt(mse)),
            "r2": r2,
        }
    return out


def generalized_r2(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    if len(y_true) <= 1:
        return float("nan")
    return float(r2_score(y_true, y_pred, multioutput="variance_weighted"))


def train(cfg: TrainConfig) -> dict[str, Any]:
    set_reproducible_seed(cfg.random_seed)
    ensure_dirs(cfg)

    x_train, x_val, y_train, y_val, data_summary = prepare_data(cfg)

    scaler = StandardScaler()
    x_train_scaled = scaler.fit_transform(x_train).astype(np.float32)
    x_val_scaled = scaler.transform(x_val).astype(np.float32)
    dump(scaler, cfg.scaler_path)

    y_train_f32 = y_train.astype(np.float32)
    y_val_f32 = y_val.astype(np.float32)

    device = torch.device(cfg.device)
    train_ds = torch.utils.data.TensorDataset(
        torch.from_numpy(x_train_scaled), torch.from_numpy(y_train_f32)
    )
    train_loader = torch.utils.data.DataLoader(
        train_ds, batch_size=min(cfg.batch_size, len(train_ds)), shuffle=True
    )

    model = MLPRegressor(
        input_dim=x_train_scaled.shape[1],
        output_dim=y_train_f32.shape[1],
        hidden_dims=cfg.hidden_dims,
        dropout=cfg.dropout,
    ).to(device)
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(
        model.parameters(),
        lr=cfg.learning_rate,
        weight_decay=cfg.weight_decay,
    )

    x_val_tensor = torch.from_numpy(x_val_scaled).to(device)
    y_val_tensor = torch.from_numpy(y_val_f32).to(device)

    best_val_loss = float("inf")
    best_state: dict[str, torch.Tensor] | None = None
    best_epoch = 0
    epochs_no_improve = 0
    history: list[dict[str, float]] = []

    for epoch in range(1, cfg.epochs + 1):
        model.train()
        batch_losses: list[float] = []
        for batch_x, batch_y in train_loader:
            batch_x = batch_x.to(device)
            batch_y = batch_y.to(device)
            optimizer.zero_grad()
            pred = model(batch_x)
            loss = criterion(pred, batch_y)
            loss.backward()
            optimizer.step()
            batch_losses.append(float(loss.item()))

        model.eval()
        with torch.no_grad():
            val_pred = model(x_val_tensor)
            val_loss = float(criterion(val_pred, y_val_tensor).item())

        train_loss = float(np.mean(batch_losses)) if batch_losses else float("nan")
        history.append({"epoch": epoch, "train_loss": train_loss, "val_loss": val_loss})

        if val_loss < best_val_loss - 1e-12:
            best_val_loss = val_loss
            best_epoch = epoch
            best_state = copy.deepcopy(model.state_dict())
            epochs_no_improve = 0
        else:
            epochs_no_improve += 1

        if epochs_no_improve >= cfg.patience:
            break

    if best_state is None:
        raise RuntimeError("Training did not produce a best model state.")

    model.load_state_dict(best_state)
    model.eval()

    x_train_tensor = torch.from_numpy(x_train_scaled).to(device)
    x_val_tensor = torch.from_numpy(x_val_scaled).to(device)

    with torch.no_grad():
        train_pred = model(x_train_tensor).cpu().numpy()
        val_pred = model(x_val_tensor).cpu().numpy()

    train_metrics = metrics_by_target(y_train_f32, train_pred, cfg.target_columns)
    val_metrics = metrics_by_target(y_val_f32, val_pred, cfg.target_columns)
    train_generalized_r2 = generalized_r2(y_train_f32, train_pred)
    test_generalized_r2 = generalized_r2(y_val_f32, val_pred)

    torch.save(
        {
            "schema": "weld_mlops_torch_regressor_v1",
            "state_dict": model.state_dict(),
            "feature_columns": cfg.feature_columns,
            "target_columns": cfg.target_columns,
            "hidden_dims": cfg.hidden_dims,
            "dropout": cfg.dropout,
            "input_dim": int(x_train_scaled.shape[1]),
            "output_dim": int(y_train_f32.shape[1]),
            "best_epoch": int(best_epoch),
            "best_val_loss": float(best_val_loss),
        },
        cfg.model_path,
    )

    results = {
        "data_summary": data_summary,
        "training": {
            "epochs_ran": int(len(history)),
            "best_epoch": int(best_epoch),
            "best_val_loss": float(best_val_loss),
            "learning_rate": cfg.learning_rate,
            "weight_decay": cfg.weight_decay,
            "batch_size": cfg.batch_size,
            "hidden_dims": cfg.hidden_dims,
            "dropout": cfg.dropout,
            "patience": cfg.patience,
            "device": str(device),
        },
        "features": cfg.feature_columns,
        "targets": cfg.target_columns,
        "metrics_train": train_metrics,
        "metrics_test": val_metrics,
        "generalized_r2_train": train_generalized_r2,
        "generalized_r2_test": test_generalized_r2,
        "artifacts": {
            "model_path": str(cfg.model_path),
            "scaler_path": str(cfg.scaler_path),
        },
        "history_tail": history[-10:],
    }
    cfg.metrics_path.write_text(json.dumps(results, indent=2))
    return results


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    results = train(cfg)

    print(f"Wrote {cfg.model_path}")
    print(f"Wrote {cfg.scaler_path}")
    print(f"Wrote {cfg.metrics_path}")
    print(
        f"Generalized R2: train={results['generalized_r2_train']:.4f}, "
        f"test={results['generalized_r2_test']:.4f}"
    )
    print(
        "Test RMSE:",
        ", ".join(
            f"{k}={v['rmse']:.4f}"
            for k, v in results["metrics_test"]["targets"].items()
        ),
    )


if __name__ == "__main__":
    main()
