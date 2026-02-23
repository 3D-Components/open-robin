"""Train a PyTorch MLP on synthetic process telemetry data."""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Sequence, Tuple

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

try:
    import matplotlib.pyplot as plt

    HAS_MPL = True
except ImportError:  # pragma: no cover - optional dependency
    HAS_MPL = False

from robin.ai import MLPConfig, ProcessGeometryMLP, save_model

ROOT_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT_DIR / 'data'
DEFAULT_DATASET = DATA_DIR / 'process_dataset_input_output.csv'
DEFAULT_MODEL_PATH = DATA_DIR / 'models' / 'process_geometry_mlp.pt'
DEFAULT_FEATURE_NAMES = ('wireSpeed', 'current', 'voltage')
TARGET_NAMES = ('height_mm', 'width_mm')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Train the process-geometry MLP'
    )
    parser.add_argument(
        '--dataset',
        type=Path,
        default=DEFAULT_DATASET,
        help='CSV file with process telemetry samples',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=DEFAULT_MODEL_PATH,
        help='Path to store the trained model checkpoint',
    )
    parser.add_argument(
        '--epochs',
        type=int,
        default=300,
        help='Number of training epochs',
    )
    parser.add_argument(
        '--batch-size',
        type=int,
        default=64,
        help='Mini-batch size',
    )
    parser.add_argument(
        '--lr',
        type=float,
        default=1e-3,
        help='Learning rate',
    )
    parser.add_argument(
        '--hidden-dim',
        type=int,
        default=64,
        help='Hidden layer width',
    )
    parser.add_argument(
        '--hidden-layers',
        type=int,
        default=3,
        help='Number of hidden layers',
    )
    parser.add_argument(
        '--dropout',
        type=float,
        default=0.01,
        help='Dropout probability',
    )
    parser.add_argument(
        '--val-ratio',
        type=float,
        default=0.2,
        help='Fraction of samples used for validation',
    )
    parser.add_argument(
        '--seed',
        type=int,
        default=42,
        help='Random seed for reproducibility',
    )
    parser.add_argument(
        '--device',
        type=str,
        default='cpu',
        help='Torch device identifier',
    )
    parser.add_argument(
        '--generate-if-missing',
        action='store_true',
        help='Generate the dataset using the simulator formulas when the CSV is absent',
    )
    parser.add_argument(
        '--samples',
        type=int,
        default=1000,
        help='Number of samples when generating a dataset',
    )
    parser.add_argument(
        '--patience',
        type=int,
        default=40,
        help='Stop training if validation loss does not improve for this many epochs (0 disables)',
    )
    parser.add_argument(
        '--min-delta',
        type=float,
        default=1e-4,
        help='Minimum relative improvement required to reset early-stopping patience',
    )
    parser.add_argument(
        '--log-every',
        type=int,
        default=1,
        help='How many epochs between progress log lines',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rng = np.random.default_rng(args.seed)
    torch.manual_seed(args.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args.seed)

    DATA_DIR.mkdir(parents=True, exist_ok=True)

    print_banner('Process Geometry MLP Training')
    print(f'Dataset path       : {args.dataset}')
    print(f'Output checkpoint  : {args.output}')
    print(
        'Hyper-parameters  : epochs={epochs}, batch={batch}, lr={lr}, hidden_dim={hd}, layers={layers}, dropout={drop}, patience={patience}, min_delta={delta}'.format(
            epochs=args.epochs,
            batch=args.batch_size,
            lr=args.lr,
            hd=args.hidden_dim,
            layers=args.hidden_layers,
            drop=args.dropout,
            patience=args.patience,
            delta=args.min_delta,
        )
    )

    dataset_columns = load_or_generate_dataset(
        args.dataset, args.generate_if_missing, args.samples, rng
    )
    features, targets, feature_names = build_features_and_targets(
        dataset_columns
    )
    if not feature_names:
        feature_names = DEFAULT_FEATURE_NAMES

    print(f"Feature order     : {', '.join(feature_names)}")
    print_dataset_summary(dataset_columns)

    indices = np.arange(features.shape[0])
    rng.shuffle(indices)
    val_size = max(1, int(len(indices) * args.val_ratio))
    train_idx = indices[val_size:]
    val_idx = indices[:val_size]

    train_features = features[train_idx]
    train_targets = targets[train_idx]
    val_features = features[val_idx]
    val_targets = targets[val_idx]

    print_banner('Data Split')
    print(f'Training samples   : {len(train_features)}')
    print(f'Validation samples : {len(val_features)}')

    feature_mean = train_features.mean(axis=0)
    feature_std = train_features.std(axis=0)
    feature_std[feature_std == 0] = 1.0

    device = torch.device(args.device)
    config = MLPConfig(
        input_dim=train_features.shape[1],
        output_dim=train_targets.shape[1],
        hidden_dim=args.hidden_dim,
        hidden_layers=args.hidden_layers,
        dropout=args.dropout,
        feature_mean=feature_mean.tolist(),
        feature_std=feature_std.tolist(),
        feature_names=feature_names,
    )
    model = ProcessGeometryMLP(config).to(device)

    train_loader = DataLoader(
        TensorDataset(
            torch.tensor(train_features, dtype=torch.float32),
            torch.tensor(train_targets, dtype=torch.float32),
        ),
        batch_size=args.batch_size,
        shuffle=True,
    )
    val_loader = DataLoader(
        TensorDataset(
            torch.tensor(val_features, dtype=torch.float32),
            torch.tensor(val_targets, dtype=torch.float32),
        ),
        batch_size=args.batch_size,
    )

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    criterion = nn.MSELoss()

    best_state = None
    best_val = float('inf')
    best_epoch = -1
    epochs_without_improve = 0
    log_every = max(1, args.log_every)
    last_epoch = -1

    for epoch in range(args.epochs):
        train_loss = train_one_epoch(
            model, train_loader, optimizer, criterion, device
        )
        val_loss = evaluate(model, val_loader, criterion, device)

        prev_best = best_val
        improved = val_loss < (best_val - args.min_delta)
        if improved or best_state is None:
            best_val = val_loss
            best_state = {
                k: v.detach().cpu().clone()
                for k, v in model.state_dict().items()
            }
            best_epoch = epoch
            epochs_without_improve = 0
        else:
            epochs_without_improve += 1

        improvement = (
            prev_best - val_loss if prev_best != float('inf') else float('nan')
        )
        should_log = (
            (epoch % log_every == 0) or improved or epoch == args.epochs - 1
        )
        if should_log:
            delta_str = '-' if np.isnan(improvement) else f'{improvement:+.6f}'
            marker = '★' if improved else ''
            print(
                f'Epoch {epoch + 1:03d}/{args.epochs:03d} | train_loss={train_loss:.6f} | val_loss={val_loss:.6f} | Δ={delta_str} {marker}'
            )

        if args.patience > 0 and epochs_without_improve >= args.patience:
            print(
                f'Early stopping after {epoch + 1} epochs (no val improvement for {args.patience}).'
            )
            last_epoch = epoch
            break

        last_epoch = epoch

    if best_state is not None:
        model.load_state_dict(best_state)
    model.cpu()

    trained_epochs = last_epoch + 1 if last_epoch >= 0 else args.epochs
    best_epoch_display = best_epoch + 1 if best_epoch >= 0 else 'n/a'
    print(
        f'Training finished after {trained_epochs} epochs. Best validation loss {best_val:.6f} (epoch {best_epoch_display}).'
    )

    train_preds = run_inference(model, train_features)
    val_preds = run_inference(model, val_features)
    train_metrics = regression_metrics(train_targets, train_preds)
    val_metrics = regression_metrics(val_targets, val_preds)
    print_metrics_table('Training', train_metrics)
    print_metrics_table('Validation', val_metrics)
    sample_predictions(
        'Validation', val_features, val_targets, val_preds, rng, feature_names
    )
    plot_actual_vs_predicted(
        'Training', train_targets, train_preds, args.output
    )
    plot_actual_vs_predicted('Validation', val_targets, val_preds, args.output)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    save_model(model, args.output)
    try:
        model_path_str = args.output.relative_to(ROOT_DIR)
    except ValueError:
        model_path_str = args.output
    print(f'Saved model to {model_path_str}')
    print(f'Best validation loss: {best_val:.6f}')


def load_or_generate_dataset(
    path: Path,
    generate_if_missing: bool,
    samples: int,
    rng: np.random.Generator,
) -> Dict[str, np.ndarray]:
    if path.exists():
        return read_dataset_columns(path)

    if not generate_if_missing:
        raise FileNotFoundError(
            f'Dataset not found at {path}. Provide --dataset or pass --generate-if-missing.'
        )

    data = generate_dataset(samples, rng)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        'wire_speed_mm_s',
        'current_A',
        'voltage_V',
        'width_mm',
        'height_mm',
    ]
    np.savetxt(
        path,
        data,
        delimiter=',',
        header=','.join(header),
        comments='',
    )
    try:
        rel_path = path.relative_to(ROOT_DIR)
    except ValueError:
        rel_path = path
    print(f'Generated dataset at {rel_path}')
    return {name: data[:, idx] for idx, name in enumerate(header)}


def generate_dataset(samples: int, rng: np.random.Generator) -> np.ndarray:
    wire_speed_mm_s = rng.uniform(60.0, 180.0, samples)
    current = rng.uniform(150.0, 250.0, samples)
    voltage = rng.uniform(18.0, 26.0, samples)

    wire_diameter_mm = 1.2
    eta_dep = 0.85
    eta_arc = 0.80
    kw = 1.20
    a_exp = 0.35

    r_mm = wire_diameter_mm / 2.0
    a_wire_mm2 = np.pi * r_mm * r_mm

    wire_feed_rate_m_per_min = wire_speed_mm_s * 60.0 / 1000.0
    travel_speed = np.maximum(
        2.0,
        5.0 + 0.02 * current + 0.50 * wire_feed_rate_m_per_min,
    )

    dep_rate_mm3_s = eta_dep * a_wire_mm2 * wire_speed_mm_s
    area_mm2 = dep_rate_mm3_s / travel_speed
    heat_input = (eta_arc * voltage * current) / travel_speed
    width = np.maximum(kw * np.power(heat_input, a_exp), 0.5)
    height = (4.0 * area_mm2) / (np.pi * width)

    return np.column_stack((wire_speed_mm_s, current, voltage, width, height))


def read_dataset_columns(path: Path) -> Dict[str, np.ndarray]:
    with path.open('r', encoding='utf-8') as fh:
        header = fh.readline().strip().split(',')
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return {name: data[:, idx] for idx, name in enumerate(header)}


def build_features_and_targets(
    columns: Dict[str, np.ndarray]
) -> Tuple[np.ndarray, np.ndarray, Tuple[str, ...]]:
    current = columns['current_A']
    voltage = columns['voltage_V']
    height = columns['height_mm']
    width = columns['width_mm']

    feature_arrays: list[np.ndarray] = []
    feature_names: list[str] = []

    if 'wire_speed_mm_s' in columns:
        feature_arrays.append(columns['wire_speed_mm_s'])
        feature_names.append('wireSpeed')
    elif 'wireSpeed' in columns:
        feature_arrays.append(columns['wireSpeed'])
        feature_names.append('wireSpeed')
    else:
        # Fallback: estimate wire speed (mm/s) from current assuming 30 A per m/min
        wire_speed_mm_s = (current / 30.0) * (1000.0 / 60.0)
        feature_arrays.append(wire_speed_mm_s)
        feature_names.append('wireSpeed')

    feature_arrays.append(current)
    feature_names.append('current')

    feature_arrays.append(voltage)
    feature_names.append('voltage')

    if not feature_arrays:
        raise ValueError('No feature columns found in dataset')

    features = np.stack(feature_arrays, axis=1)
    targets = np.stack((height, width), axis=1)

    return features, targets, tuple(feature_names)


def print_banner(title: str) -> None:
    line = '=' * len(title)
    print(f'\n{line}\n{title}\n{line}')


def print_dataset_summary(columns: Dict[str, np.ndarray]) -> None:
    ordered = [
        'wire_speed_mm_s',
        'wireSpeed',
        'travel_speed_mm_s',
        'current_A',
        'voltage_V',
        'width_mm',
        'height_mm',
    ]

    print_banner('Dataset Summary')
    first_column = next(iter(columns.values()))
    print(f'Samples: {len(first_column)}')
    print('Column                  mean     std    min    max')
    for name in ordered:
        if name not in columns:
            continue
        values = columns[name]
        print(
            f'{name:<22}'
            f'{np.mean(values):8.3f}'
            f'{np.std(values):8.3f}'
            f'{np.min(values):8.3f}'
            f'{np.max(values):8.3f}'
        )


def regression_metrics(
    y_true: np.ndarray, y_pred: np.ndarray
) -> Dict[str, np.ndarray]:
    errors = y_pred - y_true
    mae = np.mean(np.abs(errors), axis=0)
    rmse = np.sqrt(np.mean(errors**2, axis=0))
    max_err = np.max(np.abs(errors), axis=0)
    sst = np.sum((y_true - np.mean(y_true, axis=0)) ** 2, axis=0)
    sse = np.sum(errors**2, axis=0)
    with np.errstate(divide='ignore', invalid='ignore'):
        r2 = 1.0 - (sse / np.maximum(sst, 1e-12))
    return {'MAE': mae, 'RMSE': rmse, 'MAX': max_err, 'R2': r2}


def print_metrics_table(
    split: str, metric_dict: Dict[str, np.ndarray]
) -> None:
    print_banner(f'{split} Metrics')
    print('Target        MAE(mm)  RMSE(mm)  MAX(mm)     R2')
    for idx, target in enumerate(TARGET_NAMES):
        mae = metric_dict['MAE'][idx]
        rmse = metric_dict['RMSE'][idx]
        max_err = metric_dict['MAX'][idx]
        r2 = metric_dict['R2'][idx]
        print(f'{target:<12}{mae:8.4f}{rmse:10.4f}{max_err:10.4f}{r2:8.4f}')
    overall_mae = np.mean(metric_dict['MAE'])
    overall_rmse = np.mean(metric_dict['RMSE'])
    print(f'Overall      {overall_mae:8.4f}{overall_rmse:10.4f}\n')


def sample_predictions(
    split: str,
    features: np.ndarray,
    targets: np.ndarray,
    preds: np.ndarray,
    rng: np.random.Generator,
    feature_names: Sequence[str] = DEFAULT_FEATURE_NAMES,
    count: int = 5,
) -> None:
    if len(features) == 0:
        return
    count = min(count, len(features))
    indices = rng.choice(len(features), size=count, replace=False)

    print_banner(f'{split} Sample Predictions')
    for idx in indices:
        input_desc = ', '.join(
            f'{name}={features[idx, i]:.3f}'
            for i, name in enumerate(feature_names)
        )
        target_desc = ', '.join(
            f'true_{TARGET_NAMES[i]}={targets[idx, i]:.3f}'
            for i in range(targets.shape[1])
        )
        pred_desc = ', '.join(
            f'pred_{TARGET_NAMES[i]}={preds[idx, i]:.3f}'
            for i in range(preds.shape[1])
        )
        print(f'• {input_desc} | {target_desc} | {pred_desc}')


def run_inference(
    model: ProcessGeometryMLP, features: np.ndarray
) -> np.ndarray:
    model.eval()
    with torch.no_grad():
        inputs = torch.tensor(features, dtype=torch.float32)
        outputs = model(inputs).cpu().numpy()
    return outputs


def plot_actual_vs_predicted(
    split: str,
    y_true: np.ndarray,
    y_pred: np.ndarray,
    output_base: Path,
) -> None:
    if not HAS_MPL:
        print(f'Matplotlib not available. Skipping {split} scatter plot.')
        return

    num_targets = y_true.shape[1]
    fig, axes = plt.subplots(1, num_targets, figsize=(6 * num_targets, 5))
    if num_targets == 1:
        axes = [axes]

    for idx, ax in enumerate(axes):
        ax.scatter(
            y_true[:, idx],
            y_pred[:, idx],
            alpha=0.5,
            s=20,
            label='Predictions',
        )
        min_val = min(y_true[:, idx].min(), y_pred[:, idx].min())
        max_val = max(y_true[:, idx].max(), y_pred[:, idx].max())
        ax.plot(
            [min_val, max_val],
            [min_val, max_val],
            'r--',
            linewidth=1,
            label='Ideal',
        )
        ax.set_xlabel(f'True {TARGET_NAMES[idx]} (mm)')
        ax.set_ylabel(f'Predicted {TARGET_NAMES[idx]} (mm)')
        ax.set_title(TARGET_NAMES[idx])
        ax.grid(alpha=0.3)
        ax.legend()

    fig.suptitle(f'{split} Actual vs Predicted')
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    out_path = (
        output_base.parent / f'{output_base.stem}_{split.lower()}_scatter.png'
    )
    fig.savefig(out_path, dpi=150)
    plt.close(fig)

    try:
        rel = out_path.relative_to(ROOT_DIR)
    except ValueError:
        rel = out_path
    print(f'Saved {split} scatter plot to {rel}')


def train_one_epoch(
    model: ProcessGeometryMLP,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer,
    criterion: nn.Module,
    device: torch.device,
) -> float:
    model.train()
    total_loss = 0.0
    total_items = 0
    for features, targets in loader:
        features = features.to(device)
        targets = targets.to(device)

        optimizer.zero_grad()
        preds = model(features)
        loss = criterion(preds, targets)
        loss.backward()
        optimizer.step()

        batch_size = features.size(0)
        total_loss += loss.item() * batch_size
        total_items += batch_size

    return total_loss / max(total_items, 1)


def evaluate(
    model: ProcessGeometryMLP,
    loader: DataLoader,
    criterion: nn.Module,
    device: torch.device,
) -> float:
    model.eval()
    total_loss = 0.0
    total_items = 0
    with torch.no_grad():
        for features, targets in loader:
            features = features.to(device)
            targets = targets.to(device)
            preds = model(features)
            loss = criterion(preds, targets)
            batch_size = features.size(0)
            total_loss += loss.item() * batch_size
            total_items += batch_size
    return total_loss / max(total_items, 1)


if __name__ == '__main__':
    main()
