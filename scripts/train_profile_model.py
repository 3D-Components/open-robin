#!/usr/bin/env python3
"""Train synthetic demo AI models from the active profile contracts.

The script reads ``config/profiles/*.yaml``, uses each profile's
``ai.feature_order`` as the checkpoint feature names, and writes to the
profile's configured ``ai.model_path``. Existing checkpoints are skipped unless
``--overwrite`` is passed, so running the default command will not replace the
committed welding reference model.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE_DIR = ROOT / 'config' / 'profiles'
torch: Any = None
MLPConfig: Any = None
ProcessGeometryMLP: Any = None
save_model: Any = None


def _load_training_dependencies() -> None:
    global MLPConfig, ProcessGeometryMLP, save_model, torch

    if torch is not None:
        return

    import torch as torch_module
    from robin.ai import MLPConfig as mlp_config
    from robin.ai import ProcessGeometryMLP as process_geometry_mlp
    from robin.ai import save_model as save_mlp_model

    torch = torch_module
    MLPConfig = mlp_config
    ProcessGeometryMLP = process_geometry_mlp
    save_model = save_mlp_model


def _load_profiles() -> dict[str, dict[str, Any]]:
    profiles: dict[str, dict[str, Any]] = {}
    for path in sorted(PROFILE_DIR.glob('*.yaml')):
        with path.open('r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        name = data.get('profile', {}).get('name') or path.stem
        profiles[str(name)] = data
    return profiles


def _feature_order(profile: dict[str, Any]) -> list[str]:
    order = profile.get('ai', {}).get('feature_order')
    if not isinstance(order, list) or not all(isinstance(v, str) for v in order):
        raise ValueError('profile is missing ai.feature_order')
    if len(order) != 3:
        raise ValueError(f'expected exactly 3 AI input features, got {len(order)}')
    return list(order)


def _model_path(profile: dict[str, Any]) -> Path:
    raw_path = profile.get('ai', {}).get('model_path')
    if not raw_path:
        raise ValueError('profile is missing ai.model_path')
    path = Path(str(raw_path))
    return path if path.is_absolute() else ROOT / path


def _display_path(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _bounds(
    profile: dict[str, Any],
    feature_names: list[str],
) -> dict[str, tuple[float, float]]:
    raw_bounds = profile.get('ai', {}).get('inverse_bounds') or {}
    raw_features = profile.get('ai', {}).get('input_features') or []
    defaults = {
        item.get('key'): float(item.get('default'))
        for item in raw_features
        if isinstance(item, dict)
        and isinstance(item.get('key'), str)
        and isinstance(item.get('default'), (int, float))
    }

    bounds: dict[str, tuple[float, float]] = {}
    for feature in feature_names:
        raw = raw_bounds.get(feature)
        if isinstance(raw, list) and len(raw) == 2:
            low, high = float(raw[0]), float(raw[1])
        else:
            default = defaults.get(feature, 1.0)
            low, high = default * 0.5, default * 1.5
        if high < low:
            low, high = high, low
        if low == high:
            high = low + 1.0
        bounds[feature] = (low, high)
    return bounds


def _sample_features(
    feature_names: list[str],
    bounds: dict[str, tuple[float, float]],
    n: int,
) -> tuple[Any, dict[str, Any]]:
    columns: dict[str, Any] = {}
    for feature in feature_names:
        low, high = bounds[feature]
        columns[feature] = torch.empty(n).uniform_(low, high)
    x = torch.stack([columns[name] for name in feature_names], dim=1)
    return x, columns


def _find_column(
    columns: dict[str, Any],
    *names: str,
    default: float,
) -> Any:
    for name in names:
        if name in columns:
            return columns[name]
    first = next(iter(columns.values()))
    return torch.full_like(first, float(default))


def _synthetic_targets(
    profile_name: str,
    columns: dict[str, Any],
) -> Any:
    if profile_name == 'spray_coating':
        speed = _find_column(columns, 'wireSpeed', default=180.0)
        flow = _find_column(columns, 'current', default=40.0)
        pressure = _find_column(columns, 'voltage', default=3.0)
        thickness = (
            0.02
            + 0.8
            * flow
            / torch.clamp(speed, min=1.0)
            * torch.sqrt(pressure)
        )
        width = 20.0 + 8.0 * pressure - 0.03 * speed + 0.1 * flow
        noise = torch.stack(
            [torch.randn_like(thickness) * 0.004, torch.randn_like(width) * 1.0],
            dim=1,
        )
        targets = torch.stack([thickness, width], dim=1) + noise
        return torch.stack(
            [torch.clamp(targets[:, 0], min=0.01), torch.clamp(targets[:, 1], min=5.0)],
            dim=1,
        )

    if profile_name == 'welding':
        wire_feed = _find_column(
            columns,
            'wire_feed_speed_mpm_model_input',
            'wireSpeed',
            default=10.0,
        )
        travel_speed = _find_column(
            columns,
            'travel_speed_mps_model_input',
            default=0.02,
        )
        arc_correction = _find_column(
            columns,
            'arc_length_correction_mm_model_input',
            default=0.0,
        )
        heat_proxy = wire_feed / torch.clamp(travel_speed * 1000.0, min=1.0)
        height = (
            1.15
            + 0.24 * wire_feed
            - 18.0 * travel_speed
            + 0.035 * arc_correction
        )
        width = (
            3.4
            + 2.4 * torch.sqrt(torch.clamp(heat_proxy, min=0.01))
            + 0.05 * torch.abs(arc_correction)
        )
        noise = torch.stack(
            [torch.randn_like(height) * 0.08, torch.randn_like(width) * 0.12],
            dim=1,
        )
        targets = torch.stack([height, width], dim=1) + noise
        return torch.stack(
            [torch.clamp(targets[:, 0], min=0.2), torch.clamp(targets[:, 1], min=0.5)],
            dim=1,
        )

    values = list(columns.values())
    height = 0.5 + 0.015 * values[0] + 0.010 * values[1] - 0.005 * values[2]
    width = 1.0 + 0.020 * values[0] - 0.004 * values[1] + 0.030 * values[2]
    return torch.stack(
        [torch.clamp(height, min=0.1), torch.clamp(width, min=0.1)],
        dim=1,
    )


def _train(
    profile_name: str,
    x: Any,
    y: Any,
    feature_names: list[str],
    epochs: int,
    lr: float,
) -> Any:
    _load_training_dependencies()
    config = MLPConfig(
        input_dim=len(feature_names),
        hidden_dim=64,
        hidden_layers=2,
        output_dim=2,
        dropout=0.1,
        feature_mean=x.mean(dim=0).tolist(),
        feature_std=torch.clamp(x.std(dim=0), min=1e-6).tolist(),
        feature_names=feature_names,
    )
    model = ProcessGeometryMLP(config)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = torch.nn.MSELoss()

    model.train()
    report_every = max(1, epochs // 5)
    for epoch in range(1, epochs + 1):
        optimizer.zero_grad()
        loss = loss_fn(model(x), y)
        loss.backward()
        optimizer.step()
        if epoch == 1 or epoch == epochs or epoch % report_every == 0:
            print(
                f'  [{profile_name}] epoch {epoch:>4d}/{epochs} '
                f'loss={loss.item():.6f}'
            )

    model.eval()
    return model


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Train profile-scoped demo AI models',
    )
    parser.add_argument(
        '--profile',
        action='append',
        help='Profile name to train; repeatable. Defaults to all profiles.',
    )
    parser.add_argument(
        '--samples',
        type=int,
        default=2000,
        help='Synthetic samples per profile',
    )
    parser.add_argument(
        '--epochs',
        type=int,
        default=500,
        help='Training epochs per profile',
    )
    parser.add_argument(
        '--lr',
        type=float,
        default=1e-3,
        help='Optimizer learning rate',
    )
    parser.add_argument('--seed', type=int, default=42, help='Random seed')
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='Overwrite existing profile ai.model_path checkpoints',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print selected profiles and output paths without training',
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    profiles = _load_profiles()
    selected = args.profile or sorted(profiles)
    missing = [name for name in selected if name not in profiles]
    if missing:
        raise SystemExit(f'Unknown profile(s): {", ".join(missing)}')

    trained = 0
    seeded = False
    for profile_name in selected:
        profile = profiles[profile_name]
        feature_names = _feature_order(profile)
        output_path = _model_path(profile)

        print(f'\nProfile: {profile_name}')
        print(f'  features : {", ".join(feature_names)}')
        print(f'  output   : {_display_path(output_path)}')

        if args.dry_run:
            continue
        if output_path.exists() and not args.overwrite:
            print('  skipped  : checkpoint exists; pass --overwrite to replace it')
            continue

        _load_training_dependencies()
        if not seeded:
            torch.manual_seed(args.seed)
            seeded = True

        bounds = _bounds(profile, feature_names)
        x, columns = _sample_features(feature_names, bounds, args.samples)
        y = _synthetic_targets(profile_name, columns)
        model = _train(profile_name, x, y, feature_names, args.epochs, args.lr)

        output_path.parent.mkdir(parents=True, exist_ok=True)
        save_model(model, output_path)
        trained += 1
        print(f'  saved    : {_display_path(output_path)}')

        with torch.no_grad():
            sample = x[:3]
            pred = model.predict(sample)
            for inputs, outputs in zip(sample.tolist(), pred.tolist()):
                print(f'    {inputs} -> height={outputs[0]:.3f}, width={outputs[1]:.3f}')

    if trained == 0 and not args.dry_run:
        print('\nNo checkpoints written.')


if __name__ == '__main__':
    main()
