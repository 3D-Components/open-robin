"""PyTorch MLP for process geometry prediction."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import torch
from torch import nn


@dataclass
class MLPConfig:
    """Configuration for :class:`ProcessGeometryMLP`."""

    input_dim: int = 3
    hidden_dim: int = 64
    hidden_layers: int = 2
    output_dim: int = 2
    dropout: float = 0.1
    feature_mean: Sequence[float] | None = None
    feature_std: Sequence[float] | None = None
    feature_names: Sequence[str] | None = None


class ProcessGeometryMLP(nn.Module):
    """Simple feed-forward network that maps process parameters to geometry."""

    def __init__(self, config: MLPConfig | None = None) -> None:
        super().__init__()
        self.config = config or MLPConfig()

        layers: list[nn.Module] = []
        in_features = self.config.input_dim
        for _ in range(self.config.hidden_layers):
            layers.append(nn.Linear(in_features, self.config.hidden_dim))
            layers.append(nn.ReLU())
            if self.config.dropout > 0:
                layers.append(nn.Dropout(self.config.dropout))
            in_features = self.config.hidden_dim
        layers.append(nn.Linear(in_features, self.config.output_dim))
        self.network = nn.Sequential(*layers)

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:  # type: ignore[override]
        inputs = self._apply_normalization(inputs)
        return self.network(inputs)

    @torch.no_grad()
    def predict(
        self, inputs: Sequence[Sequence[float]] | torch.Tensor
    ) -> torch.Tensor:
        tensor = _to_tensor(inputs)
        self.eval()
        return self(tensor)

    def _apply_normalization(self, inputs: torch.Tensor) -> torch.Tensor:
        if self.config.feature_mean is None or self.config.feature_std is None:
            return inputs

        mean = torch.tensor(
            self.config.feature_mean, dtype=inputs.dtype, device=inputs.device
        )
        std = torch.tensor(
            self.config.feature_std, dtype=inputs.dtype, device=inputs.device
        )
        std = torch.clamp(std, min=1e-6)
        return (inputs - mean) / std


def _to_tensor(
    values: Sequence[Sequence[float]] | torch.Tensor,
) -> torch.Tensor:
    if isinstance(values, torch.Tensor):
        return values
    return torch.tensor(values, dtype=torch.float32)


def save_model(model: ProcessGeometryMLP, path: str | Path) -> None:
    torch.save(
        {
            'state_dict': model.state_dict(),
            'config': model.config.__dict__,
        },
        Path(path),
    )


def load_model(
    path: str | Path, map_location: str | torch.device | None = None
) -> ProcessGeometryMLP:
    checkpoint = torch.load(Path(path), map_location=map_location)
    config = MLPConfig(**checkpoint['config'])
    model = ProcessGeometryMLP(config)
    model.load_state_dict(checkpoint['state_dict'])
    return model
