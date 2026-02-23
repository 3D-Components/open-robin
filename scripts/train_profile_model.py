#!/usr/bin/env python3
"""Generate synthetic AI models for each ROBIN profile.

Each profile maps three process parameters to two geometry outputs.  This
script creates a small synthetic dataset for each profile, trains the generic
ProcessGeometryMLP, and saves a checkpoint with the correct feature names,
normalization stats, and weights.

Run once from the repo root:

    poetry run python scripts/train_profile_model.py

Outputs:
    data/models/welding/process_geometry_mlp.pt
    data/models/spray_coating/process_geometry_mlp.pt
"""

from __future__ import annotations

import math
import random
from pathlib import Path

import torch

from robin.ai import MLPConfig, ProcessGeometryMLP, save_model


ROOT = Path(__file__).resolve().parents[1]


def _synthetic_welding_data(n: int = 2000) -> tuple[torch.Tensor, torch.Tensor]:
    """Generate (wireSpeed, current, voltage) -> (height, width) pairs.

    Uses a simplified physics-inspired model:
      - height ~ f(wireSpeed, current)
      - width  ~ g(current, voltage)
    """
    xs, ys = [], []
    for _ in range(n):
        ws = random.uniform(5.0, 15.0)
        cur = random.uniform(100.0, 200.0)
        vol = random.uniform(20.0, 30.0)

        height = 3.0 + 0.15 * ws + 0.008 * cur - 0.05 * vol + random.gauss(0, 0.15)
        width = 1.5 + 0.01 * cur + 0.12 * vol - 0.04 * ws + random.gauss(0, 0.1)

        xs.append([ws, cur, vol])
        ys.append([max(0.5, height), max(0.5, width)])

    return torch.tensor(xs, dtype=torch.float32), torch.tensor(ys, dtype=torch.float32)


def _synthetic_coating_data(n: int = 2000) -> tuple[torch.Tensor, torch.Tensor]:
    """Generate (lineSpeed, flowRate, pressure) -> (thickness, coverage_width) pairs.

    Uses a simplified model:
      - thickness ~ flowRate / lineSpeed * pressure_factor
      - coverage  ~ pressure * nozzle_spread - speed_effect
    """
    xs, ys = [], []
    for _ in range(n):
        speed = random.uniform(100.0, 300.0)
        flow = random.uniform(20.0, 70.0)
        pressure = random.uniform(2.0, 5.0)

        thickness = 0.02 + 0.8 * flow / speed * math.sqrt(pressure) + random.gauss(0, 0.005)
        width = 20.0 + 8.0 * pressure - 0.03 * speed + 0.1 * flow + random.gauss(0, 1.5)

        xs.append([speed, flow, pressure])
        ys.append([max(0.01, thickness), max(5.0, width)])

    return torch.tensor(xs, dtype=torch.float32), torch.tensor(ys, dtype=torch.float32)


def _train(
    name: str,
    x: torch.Tensor,
    y: torch.Tensor,
    feature_names: list[str],
    epochs: int = 500,
    lr: float = 1e-3,
) -> ProcessGeometryMLP:
    mean = x.mean(dim=0).tolist()
    std = x.std(dim=0).tolist()

    config = MLPConfig(
        input_dim=3,
        hidden_dim=64,
        hidden_layers=2,
        output_dim=2,
        dropout=0.1,
        feature_mean=mean,
        feature_std=std,
        feature_names=feature_names,
    )
    model = ProcessGeometryMLP(config)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = torch.nn.MSELoss()

    model.train()
    for epoch in range(1, epochs + 1):
        optimizer.zero_grad()
        pred = model(x)
        loss = loss_fn(pred, y)
        loss.backward()
        optimizer.step()
        if epoch % 100 == 0 or epoch == 1:
            print(f'  [{name}] epoch {epoch:>4d}/{epochs}  loss={loss.item():.6f}')

    model.eval()
    return model


def main() -> None:
    random.seed(42)
    torch.manual_seed(42)

    profiles = {
        'welding': {
            'generator': _synthetic_welding_data,
            'feature_names': ['wireSpeed', 'current', 'voltage'],
        },
        'spray_coating': {
            'generator': _synthetic_coating_data,
            'feature_names': ['wireSpeed', 'current', 'voltage'],
        },
    }

    for profile_name, cfg in profiles.items():
        print(f'\nTraining model for profile: {profile_name}')
        x, y = cfg['generator']()
        model = _train(profile_name, x, y, cfg['feature_names'])

        out_dir = ROOT / 'data' / 'models' / profile_name
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / 'process_geometry_mlp.pt'
        save_model(model, out_path)
        print(f'  Saved: {out_path}')

        with torch.no_grad():
            sample = x[:3]
            pred = model.predict(sample)
            print(f'  Sample predictions (first 3):')
            for i in range(3):
                inp = sample[i].tolist()
                out = pred[i].tolist()
                print(f'    {inp} -> height={out[0]:.3f}, width={out[1]:.3f}')

    print('\nDone. Models saved to data/models/<profile>/process_geometry_mlp.pt')


if __name__ == '__main__':
    main()
