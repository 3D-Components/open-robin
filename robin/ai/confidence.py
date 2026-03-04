"""Forward-model confidence estimation utilities.

Confidence is computed dynamically from:
1) Predictive uncertainty via MC-dropout (when dropout is available)
2) Input distance from training feature distribution (z-score space)
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Any, Mapping, Sequence

import torch

from .mlp import ProcessGeometryMLP


@dataclass(frozen=True)
class ForwardConfidenceConfig:
    """Configuration for forward prediction confidence estimation."""

    mc_samples: int = 20
    uncertainty_weight: float = 0.65
    distance_weight: float = 0.35
    uncertainty_scale: float = 0.08
    distance_scale: float = 2.0
    min_confidence: float = 0.05
    max_confidence: float = 0.99

    @classmethod
    def from_dict(
        cls, raw: Mapping[str, Any] | None
    ) -> 'ForwardConfidenceConfig':
        """Build config from profile data with safe fallbacks."""
        base = cls()
        if not raw:
            return base
        return cls(
            mc_samples=max(1, int(raw.get('mc_samples', base.mc_samples))),
            uncertainty_weight=max(
                0.0,
                min(
                    1.0,
                    float(
                        raw.get(
                            'uncertainty_weight', base.uncertainty_weight
                        )
                    ),
                ),
            ),
            distance_weight=max(
                0.0,
                min(
                    1.0,
                    float(raw.get('distance_weight', base.distance_weight)),
                ),
            ),
            uncertainty_scale=max(
                1e-6,
                float(raw.get('uncertainty_scale', base.uncertainty_scale)),
            ),
            distance_scale=max(
                1e-6, float(raw.get('distance_scale', base.distance_scale))
            ),
            min_confidence=max(
                0.0, float(raw.get('min_confidence', base.min_confidence))
            ),
            max_confidence=min(
                0.99, float(raw.get('max_confidence', base.max_confidence))
            ),
        )


@dataclass(frozen=True)
class ForwardConfidenceResult:
    """Result bundle for forward prediction + confidence diagnostics."""

    prediction: list[float]
    confidence: float
    output_std: list[float]
    relative_uncertainty: float
    input_distance: float
    mc_samples: int


class ForwardConfidenceEstimator:
    """Estimate prediction confidence for a forward model."""

    def __init__(self, config: ForwardConfidenceConfig | None = None) -> None:
        self.config = config or ForwardConfidenceConfig()

    def estimate(
        self,
        model: ProcessGeometryMLP,
        feature_vector: Sequence[float],
    ) -> ForwardConfidenceResult:
        """Run forward prediction and return dynamic confidence."""
        x = torch.tensor([list(feature_vector)], dtype=torch.float32)
        mean_pred, std_pred, used_samples = self._predict_distribution(model, x)

        relative_uncertainty = self._relative_uncertainty(mean_pred, std_pred)
        input_distance = self._input_distance(model, x)
        confidence = self._confidence(relative_uncertainty, input_distance)

        return ForwardConfidenceResult(
            prediction=mean_pred.tolist(),
            confidence=float(confidence),
            output_std=std_pred.tolist(),
            relative_uncertainty=float(relative_uncertainty),
            input_distance=float(input_distance),
            mc_samples=int(used_samples),
        )

    def _predict_distribution(
        self, model: ProcessGeometryMLP, x: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, int]:
        has_dropout = bool(getattr(model.config, 'dropout', 0.0) > 0.0)
        use_mc = has_dropout and self.config.mc_samples > 1
        was_training = model.training

        with torch.no_grad():
            if use_mc:
                model.train()
                preds = []
                for _ in range(self.config.mc_samples):
                    preds.append(model(x).squeeze(0))
                stacked = torch.stack(preds, dim=0)
                mean_pred = stacked.mean(dim=0)
                std_pred = stacked.std(dim=0, unbiased=False)
                used = self.config.mc_samples
            else:
                model.eval()
                mean_pred = model(x).squeeze(0)
                std_pred = torch.zeros_like(mean_pred)
                used = 1

        model.train(was_training)
        return mean_pred, std_pred, used

    @staticmethod
    def _relative_uncertainty(
        mean_pred: torch.Tensor, std_pred: torch.Tensor
    ) -> float:
        # Stabilize denominator so near-zero outputs do not explode uncertainty.
        denom = torch.clamp(mean_pred.abs(), min=1.0)
        rel = std_pred / denom
        return float(torch.mean(rel).item())

    @staticmethod
    def _input_distance(model: ProcessGeometryMLP, x: torch.Tensor) -> float:
        cfg = model.config
        if cfg.feature_mean is None or cfg.feature_std is None:
            return 0.0

        mean = torch.tensor(cfg.feature_mean, dtype=x.dtype, device=x.device)
        std = torch.tensor(cfg.feature_std, dtype=x.dtype, device=x.device)
        std = torch.clamp(std, min=1e-6)
        z = (x.squeeze(0) - mean) / std
        distance = torch.linalg.vector_norm(z, ord=2).item()
        return float(distance / sqrt(max(len(z), 1)))

    def _confidence(self, rel_unc: float, input_dist: float) -> float:
        unc_penalty = rel_unc / (rel_unc + self.config.uncertainty_scale)
        dist_penalty = input_dist / (input_dist + self.config.distance_scale)

        total_weight = self.config.uncertainty_weight + self.config.distance_weight
        if total_weight <= 0:
            combined = 0.0
        else:
            combined = (
                (self.config.uncertainty_weight * unc_penalty)
                + (self.config.distance_weight * dist_penalty)
            ) / total_weight

        confidence = 1.0 - combined
        return max(
            self.config.min_confidence,
            min(self.config.max_confidence, confidence),
        )
