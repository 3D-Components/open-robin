"""Constrained inverse optimization for geometry-driven recommendations.

This module keeps the forward model as the single source of truth:
given a target geometry, it searches process parameter space and selects
the parameter set whose forward prediction best matches the target.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from random import Random
from typing import Any, Callable, Dict, Mapping, Sequence, Tuple


GeometryPredictor = Callable[[Dict[str, float]], Dict[str, float]]


@dataclass(frozen=True)
class InverseOptimizationConfig:
    """Runtime knobs for inverse optimization."""

    restarts: int = 24
    max_iterations: int = 80
    initial_step_ratio: float = 0.15
    min_step_ratio: float = 0.002
    step_decay: float = 0.5
    regularization: float = 0.05
    random_seed: int = 42
    geometry_weights: Mapping[str, float] = field(
        default_factory=lambda: {'height': 1.0, 'width': 1.0}
    )

    @classmethod
    def from_dict(
        cls, raw: Mapping[str, Any] | None
    ) -> 'InverseOptimizationConfig':
        """Create config from profile data, filling missing values safely."""
        base = cls()
        if not raw:
            return base

        raw_weights = raw.get('geometry_weights') or {}
        if not isinstance(raw_weights, Mapping):
            raw_weights = {}

        return cls(
            restarts=max(1, int(raw.get('restarts', base.restarts))),
            max_iterations=max(
                1, int(raw.get('max_iterations', base.max_iterations))
            ),
            initial_step_ratio=float(
                raw.get('initial_step_ratio', base.initial_step_ratio)
            ),
            min_step_ratio=float(
                raw.get('min_step_ratio', base.min_step_ratio)
            ),
            step_decay=float(raw.get('step_decay', base.step_decay)),
            regularization=max(
                0.0, float(raw.get('regularization', base.regularization))
            ),
            random_seed=int(raw.get('random_seed', base.random_seed)),
            geometry_weights={
                'height': float(
                    raw_weights.get(
                        'height', base.geometry_weights.get('height', 1.0)
                    )
                ),
                'width': float(
                    raw_weights.get(
                        'width', base.geometry_weights.get('width', 1.0)
                    )
                ),
            },
        )


@dataclass(frozen=True)
class InverseOptimizationResult:
    """Result of inverse optimization."""

    params: Dict[str, float]
    predicted_geometry: Dict[str, float]
    objective: float
    geometry_loss: float
    iterations: int
    restarts: int
    confidence: float


class GeometryInverseOptimizer:
    """Search parameter space to match a target geometry via forward prediction."""

    def __init__(
        self,
        feature_order: Sequence[str],
        parameter_bounds: Mapping[str, Tuple[float, float]],
        predict_geometry: GeometryPredictor,
        config: InverseOptimizationConfig | None = None,
    ) -> None:
        self.feature_order = tuple(feature_order)
        self.predict_geometry = predict_geometry
        self.config = config or InverseOptimizationConfig()
        self.parameter_bounds = self._normalize_bounds(parameter_bounds)

    def solve(
        self,
        target_geometry: Dict[str, float],
        current_params: Dict[str, float] | None = None,
    ) -> InverseOptimizationResult:
        """Compute recommended parameters for the target geometry."""
        target = {
            'height': float(target_geometry['height']),
            'width': float(target_geometry['width']),
        }
        current = self._normalize_params(
            current_params or {}, fallback='midpoint'
        )
        candidate_starts = self._initial_candidates(current)

        best_params = current
        best_pred = self._predict(best_params)
        best_obj, best_geom = self._objective(best_params, target, current)
        total_iterations = 0

        for start in candidate_starts:
            refined, pred, obj, geom_loss, iterations = (
                self._coordinate_descent(start, target, current)
            )
            total_iterations += iterations
            if obj < best_obj:
                best_params = refined
                best_pred = pred
                best_obj = obj
                best_geom = geom_loss

        confidence = self._confidence(best_pred, target)
        return InverseOptimizationResult(
            params=best_params,
            predicted_geometry=best_pred,
            objective=float(best_obj),
            geometry_loss=float(best_geom),
            iterations=total_iterations,
            restarts=len(candidate_starts),
            confidence=confidence,
        )

    def _normalize_bounds(
        self, raw_bounds: Mapping[str, Tuple[float, float]]
    ) -> Dict[str, Tuple[float, float]]:
        bounds: Dict[str, Tuple[float, float]] = {}
        for feature in self.feature_order:
            raw = raw_bounds.get(feature)
            if (
                isinstance(raw, (list, tuple))
                and len(raw) == 2
                and isinstance(raw[0], (int, float))
                and isinstance(raw[1], (int, float))
            ):
                low = float(raw[0])
                high = float(raw[1])
            else:
                low = 0.0
                high = 1.0

            if high < low:
                low, high = high, low
            if low == high:
                high = low + 1.0

            bounds[feature] = (low, high)
        return bounds

    def _midpoint(self, feature: str) -> float:
        low, high = self.parameter_bounds[feature]
        return (low + high) / 2.0

    def _span(self, feature: str) -> float:
        low, high = self.parameter_bounds[feature]
        return max(high - low, 1e-9)

    def _clamp(self, feature: str, value: float) -> float:
        low, high = self.parameter_bounds[feature]
        return min(max(float(value), low), high)

    def _normalize_params(
        self, params: Mapping[str, float], fallback: str = 'midpoint'
    ) -> Dict[str, float]:
        normalized: Dict[str, float] = {}
        for feature in self.feature_order:
            raw = params.get(feature)
            if isinstance(raw, (int, float)):
                normalized[feature] = self._clamp(feature, float(raw))
            elif fallback == 'midpoint':
                normalized[feature] = self._midpoint(feature)
            else:
                normalized[feature] = self.parameter_bounds[feature][0]
        return normalized

    def _predict(self, params: Dict[str, float]) -> Dict[str, float]:
        prediction = self.predict_geometry(params)
        return {
            'height': float(prediction['height']),
            'width': float(prediction['width']),
        }

    def _objective(
        self,
        params: Dict[str, float],
        target: Dict[str, float],
        current: Dict[str, float],
    ) -> Tuple[float, float]:
        pred = self._predict(params)

        h_scale = max(abs(target['height']), 1e-6)
        w_scale = max(abs(target['width']), 1e-6)
        h_weight = max(0.0, float(self.config.geometry_weights.get('height', 1.0)))
        w_weight = max(0.0, float(self.config.geometry_weights.get('width', 1.0)))

        height_term = ((pred['height'] - target['height']) / h_scale) ** 2
        width_term = ((pred['width'] - target['width']) / w_scale) ** 2
        geometry_loss = (h_weight * height_term) + (w_weight * width_term)

        reg_loss = 0.0
        if self.config.regularization > 0:
            for feature in self.feature_order:
                reg_loss += (
                    (params[feature] - current[feature]) / self._span(feature)
                ) ** 2
            reg_loss /= max(len(self.feature_order), 1)

        total = geometry_loss + (self.config.regularization * reg_loss)
        return float(total), float(geometry_loss)

    def _initial_candidates(
        self, current: Dict[str, float]
    ) -> list[Dict[str, float]]:
        rng = Random(self.config.random_seed)
        candidates = [dict(current)]

        midpoint = {
            feature: self._midpoint(feature) for feature in self.feature_order
        }
        candidates.append(midpoint)

        for _ in range(max(0, self.config.restarts - len(candidates))):
            sampled = {
                feature: rng.uniform(
                    self.parameter_bounds[feature][0],
                    self.parameter_bounds[feature][1],
                )
                for feature in self.feature_order
            }
            candidates.append(sampled)

        return candidates

    def _coordinate_descent(
        self,
        start: Dict[str, float],
        target: Dict[str, float],
        current: Dict[str, float],
    ) -> Tuple[Dict[str, float], Dict[str, float], float, float, int]:
        candidate = self._normalize_params(start)
        best_pred = self._predict(candidate)
        best_obj, best_geom = self._objective(candidate, target, current)

        steps = {
            feature: self._span(feature) * self.config.initial_step_ratio
            for feature in self.feature_order
        }
        iterations = 0

        for _ in range(self.config.max_iterations):
            iterations += 1
            improved = False

            for feature in self.feature_order:
                for direction in (-1.0, 1.0):
                    trial = dict(candidate)
                    trial[feature] = self._clamp(
                        feature, trial[feature] + (direction * steps[feature])
                    )
                    trial_obj, trial_geom = self._objective(
                        trial, target, current
                    )
                    if trial_obj + 1e-12 < best_obj:
                        candidate = trial
                        best_obj = trial_obj
                        best_geom = trial_geom
                        best_pred = self._predict(candidate)
                        improved = True

            if improved:
                continue

            max_norm_step = 0.0
            for feature in self.feature_order:
                steps[feature] *= self.config.step_decay
                max_norm_step = max(
                    max_norm_step, steps[feature] / self._span(feature)
                )
            if max_norm_step < self.config.min_step_ratio:
                break

        return candidate, best_pred, best_obj, best_geom, iterations

    def _confidence(
        self, predicted: Dict[str, float], target: Dict[str, float]
    ) -> float:
        h_scale = max(abs(target['height']), 1e-6)
        w_scale = max(abs(target['width']), 1e-6)
        rel_h = abs(predicted['height'] - target['height']) / h_scale
        rel_w = abs(predicted['width'] - target['width']) / w_scale
        max_rel_error = max(rel_h, rel_w)
        confidence = 1.0 - max_rel_error
        return float(min(max(confidence, 0.0), 0.99))
