"""PyTorch MLP for process geometry prediction."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import torch
from torch import nn


@dataclass
class MLPConfig:
    """Configuration for :class:`ProcessGeometryMLP`."""

    input_dim: int = 3
    hidden_dim: int = 64
    hidden_layers: int = 2
    hidden_dims: Sequence[int] | None = None
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
        hidden_dims = self._resolved_hidden_dims()
        for hidden_features in hidden_dims:
            layers.append(nn.Linear(in_features, hidden_features))
            layers.append(nn.ReLU())
            if self.config.dropout > 0:
                layers.append(nn.Dropout(self.config.dropout))
            in_features = hidden_features
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

    def _resolved_hidden_dims(self) -> list[int]:
        if self.config.hidden_dims:
            return [int(value) for value in self.config.hidden_dims]
        return [int(self.config.hidden_dim)] * int(self.config.hidden_layers)


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
    if 'config' in checkpoint:
        config = MLPConfig(**checkpoint['config'])
        state_dict = checkpoint['state_dict']
    else:
        config = _config_from_selected_gold_checkpoint(checkpoint, Path(path))
        state_dict = _remap_selected_gold_state_dict(checkpoint['state_dict'])
    model = ProcessGeometryMLP(config)
    model.load_state_dict(state_dict)
    return model


def _config_from_selected_gold_checkpoint(
    checkpoint: dict[str, Any], path: Path
) -> MLPConfig:
    feature_names = checkpoint.get('feature_columns') or checkpoint.get(
        'feature_names'
    )
    feature_mean = None
    feature_std = None

    scaler = _load_selected_gold_scaler(checkpoint, path)
    if scaler is not None:
        mean = getattr(scaler, 'mean_', None)
        scale = getattr(scaler, 'scale_', None)
        if mean is not None and scale is not None:
            feature_mean = [float(value) for value in mean]
            feature_std = [float(value) for value in scale]

    return MLPConfig(
        input_dim=int(checkpoint.get('input_dim', len(feature_names or []))),
        output_dim=int(checkpoint.get('output_dim', 2)),
        hidden_dims=checkpoint.get('hidden_dims'),
        hidden_layers=len(checkpoint.get('hidden_dims', []) or []),
        hidden_dim=int((checkpoint.get('hidden_dims') or [64])[0]),
        dropout=0.0,
        feature_mean=feature_mean,
        feature_std=feature_std,
        feature_names=feature_names,
    )


def _load_selected_gold_scaler(
    checkpoint: dict[str, Any], model_path: Path
) -> Any | None:
    scaler_path_raw = checkpoint.get('scaler_path')
    if not scaler_path_raw:
        return None

    candidates: list[Path] = []
    raw_path = Path(str(scaler_path_raw))
    candidates.append(raw_path)
    candidates.append(model_path.with_name(raw_path.name))

    for candidate in candidates:
        if not candidate.exists():
            continue
        try:
            from joblib import load as joblib_load

            return joblib_load(candidate)
        except Exception:
            continue
    return None


def _remap_selected_gold_state_dict(
    state_dict: dict[str, Any]
) -> dict[str, Any]:
    remapped: dict[str, Any] = {}
    for key, value in state_dict.items():
        if key.startswith('net.'):
            remapped[f"network.{key[4:]}"] = value
        else:
            remapped[key] = value
    return remapped
