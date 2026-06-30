"""Utilities for ROBIN AI models.

This package intentionally uses lazy imports so non-ML code paths can import
``robin`` without requiring ``torch`` at import time.
"""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .confidence import ForwardConfidenceConfig, ForwardConfidenceEstimator
    from .inverse import (
        GeometryInverseOptimizer,
        InverseOptimizationConfig,
        InverseOptimizationResult,
    )
    from .mlp import MLPConfig, ProcessGeometryMLP

__all__ = [
    'ForwardConfidenceConfig',
    'ForwardConfidenceEstimator',
    'GeometryInverseOptimizer',
    'InverseOptimizationConfig',
    'InverseOptimizationResult',
    'MLPConfig',
    'ProcessGeometryMLP',
    'load_model',
    'save_model',
]


def __getattr__(name):
    if name in {'ForwardConfidenceConfig', 'ForwardConfidenceEstimator'}:
        module = importlib.import_module('robin.ai.confidence')
        return getattr(module, name)

    if name in {
        'GeometryInverseOptimizer',
        'InverseOptimizationConfig',
        'InverseOptimizationResult',
    }:
        module = importlib.import_module('robin.ai.inverse')
        return getattr(module, name)

    if name in {'MLPConfig', 'ProcessGeometryMLP', 'load_model', 'save_model'}:
        module = importlib.import_module('robin.ai.mlp')
        return getattr(module, name)

    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
