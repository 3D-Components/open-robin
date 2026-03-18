"""
MCCP-UQ Inference Module (Standalone)
=====================================
Minimal, self-contained module for running MCCP uncertainty-quantified inference.
Extracted from mccp_lib for use in RobTrack without importing the full library.

Supports regression (CQR) inference on Keras/TensorFlow models.
For PyTorch model support, see mccp_inference_pytorch.py.

Dependencies: numpy, tqdm, tensorflow, scikit-learn
"""

import logging
import os
import pickle
import sys
import types
import numpy as np
import tqdm
from pathlib import Path

# ---------------------------------------------------------------------------
# TensorFlow GPU configuration - force CPU for MCCP inference by default.
# The model is small and CPU inference is fast; this avoids failures when
# CUDA / cuDNN are not installed or version-mismatched (common on Linux dev
# boxes).  Set  MCCP_USE_GPU=1  to opt into GPU inference.
# ---------------------------------------------------------------------------
import tensorflow as tf

_logger = logging.getLogger(__name__)

if os.environ.get('MCCP_USE_GPU', '0') != '1':
    try:
        tf.config.set_visible_devices([], 'GPU')
        _logger.info('MCCP inference: GPU disabled (CPU mode).')
    except RuntimeError:
        pass  # devices already initialised - best effort
else:
    _logger.info('MCCP inference: GPU enabled via MCCP_USE_GPU=1.')

from tensorflow import keras
from tensorflow.keras.backend import clear_session
from sklearn.metrics import mean_absolute_error as mae


# ---------------------------------------------------------------------------
# mccp_lib stub - allows pickle to reconstruct MCCP objects without
# installing the full mccp_lib package from the MLOps Orchestrator.
# ---------------------------------------------------------------------------


def _ensure_mccp_lib_stub():
    """Register a minimal mccp_lib.mccp.mccp.MCCP class in sys.modules
    so that ``pickle.load()`` can deserialize mccp_state.pkl."""
    if 'mccp_lib.mccp.mccp' in sys.modules:
        return

    class MCCP:
        """Pickle-compatible stub."""

        pass

    mccp_lib = types.ModuleType('mccp_lib')
    mccp_lib_mccp = types.ModuleType('mccp_lib.mccp')
    mccp_lib_mccp_mccp = types.ModuleType('mccp_lib.mccp.mccp')
    mccp_lib_mccp_mccp.MCCP = MCCP

    mccp_lib.mccp = mccp_lib_mccp
    mccp_lib_mccp.mccp = mccp_lib_mccp_mccp

    sys.modules['mccp_lib'] = mccp_lib
    sys.modules['mccp_lib.mccp'] = mccp_lib_mccp
    sys.modules['mccp_lib.mccp.mccp'] = mccp_lib_mccp_mccp


_ensure_mccp_lib_stub()


# ---------------------------------------------------------------------------
# Dynamic Monte Carlo Dropout
# ---------------------------------------------------------------------------


def dynamic_mc_predict(x_test, model, patience, min_delta, max_mc):
    """
    Adaptive MC dropout: runs stochastic forward passes through the model
    (with dropout active) until prediction variance stabilizes.

    Args:
        x_test:    input features, shape (n_samples, n_features)
        model:     Keras model (must have been trained with MC dropout)
        patience:  consecutive stable iterations before stopping
        min_delta: variance change threshold for stability
        max_mc:    maximum number of forward passes

    Returns:
        Mean predictions across MC samples, shape (n_samples, n_outputs)
    """
    montecarlo_predictions = []

    for data in tqdm.tqdm(x_test, desc='MC dropout'):
        current_patience_count = 0
        prev_variance = []
        predictions = []

        while True:
            prediction_raw = model.predict(
                np.expand_dims(data, axis=0), verbose=0
            )
            clear_session()

            if isinstance(prediction_raw, (list, tuple)):
                # Multi-head output (e.g., MQNN with one head per quantile)
                prediction = np.stack(
                    [
                        np.squeeze(np.asarray(head), axis=0)
                        for head in prediction_raw
                    ],
                    axis=0,
                )
            else:
                prediction = np.squeeze(np.asarray(prediction_raw), axis=0)

            predictions.append(prediction)
            variance = np.asarray(predictions).std(axis=0)

            if len(predictions) != 1:
                var_diff = abs(np.subtract(prev_variance, variance))
                if np.all(var_diff <= min_delta):
                    current_patience_count += 1
                else:
                    current_patience_count = 0

            if (
                current_patience_count >= patience
                or len(predictions) == max_mc
            ):
                break

            prev_variance = variance

        predictions = np.asarray(predictions)
        montecarlo_predictions.append(predictions.mean(axis=0))

    return np.asarray(montecarlo_predictions)


# ---------------------------------------------------------------------------
# Conformalized Quantile Regression (CQR) - prediction only
# ---------------------------------------------------------------------------


def cqr_predict(val_upper, val_lower, q_hat, y_test=None):
    """
    Apply CQR conformal correction to quantile predictions.

    Args:
        val_upper:  upper quantile predictions, shape (n_samples,) or (n_samples, n_targets)
        val_lower:  lower quantile predictions, same shape
        q_hat:      calibration threshold (scalar or per-target array)
        y_test:     optional ground truth for coverage metrics

    Returns:
        dict with keys: upper_pred, lower_pred, intervals,
                        full_mae, empirical_coverage, empirical_coverage_uncalibrated
                        (last three are None if y_test not provided)
    """
    val_upper = np.asarray(val_upper)
    val_lower = np.asarray(val_lower)
    q_hat = np.asarray(q_hat)

    if val_upper.ndim == 1:
        val_upper = val_upper[:, None]
    if val_lower.ndim == 1:
        val_lower = val_lower[:, None]
    if q_hat.ndim == 0:
        q_hat = np.asarray([q_hat])
    if q_hat.ndim == 1:
        q_hat = q_hat.reshape(1, -1)

    q_hat = np.broadcast_to(q_hat, val_lower.shape)

    # Calibrated prediction intervals
    lower_pred = val_lower - q_hat
    upper_pred = val_upper + q_hat
    intervals = (upper_pred - lower_pred).mean(axis=0)

    # Optional coverage / MAE metrics (only meaningful with ground truth)
    full_mae = None
    empirical_coverage = None
    empirical_coverage_uncalibrated = None

    if y_test is not None:
        y_test = np.asarray(y_test)
        if y_test.ndim == 1:
            y_test = y_test[:, None]
        empirical_coverage_uncalibrated = (
            (y_test >= val_lower) & (y_test <= val_upper)
        ).mean(axis=0)
        empirical_coverage = (
            (y_test >= lower_pred) & (y_test <= upper_pred)
        ).mean(axis=0)
        upper_mae = mae(y_test, upper_pred, multioutput='raw_values')
        lower_mae = mae(y_test, lower_pred, multioutput='raw_values')
        full_mae = upper_mae + lower_mae

    return {
        'upper_pred': upper_pred,
        'lower_pred': lower_pred,
        'intervals': intervals,
        'full_mae': full_mae,
        'empirical_coverage': empirical_coverage,
        'empirical_coverage_uncalibrated': empirical_coverage_uncalibrated,
    }


# ---------------------------------------------------------------------------
# High-level inference API
# ---------------------------------------------------------------------------


def load_artifacts(model_path, state_path):
    """
    Load a trained Keras model and its MCCP calibration state.

    Args:
        model_path: path to .h5 model file
        state_path: path to .pkl calibration state file

    Returns:
        (model, mccp_state) tuple. mccp_state is a dict-like object
        containing at minimum: q_hat, mode, patience, delta, max_mc.
    """
    model = keras.models.load_model(str(model_path), compile=False)

    with open(state_path, 'rb') as f:
        mccp_state = pickle.load(f)

    # The pickle stores an MCCP object with model=None. We re-inject.
    mccp_state.model = model

    return model, mccp_state


def predict(model, mccp_state, input_features, target_dim=2):
    """
    Run MCCP regression inference.

    Args:
        model:          loaded Keras model
        mccp_state:     loaded MCCP calibration state (from pickle)
        input_features: numpy array of shape (n_samples, n_features)
        target_dim:     number of target variables (default 2 for Energy Efficiency)

    Returns:
        dict with keys:
            lower:     lower prediction bounds, shape (n_samples, target_dim)
            upper:     upper prediction bounds, shape (n_samples, target_dim)
            intervals: mean interval widths per target, shape (target_dim,)
    """
    # Step 1: Dynamic MC dropout - multiple stochastic forward passes
    mc_predictions = dynamic_mc_predict(
        input_features,
        model,
        patience=mccp_state.patience,
        min_delta=mccp_state.delta,
        max_mc=mccp_state.max_mc,
    )

    # Step 2: Split into upper/lower quantile predictions
    # MQNN outputs [quantile_low, quantile_high] - indices [0] and [1]
    x_lower = mc_predictions[:, 0]
    x_upper = mc_predictions[:, 1]

    # Step 3: Apply CQR conformal correction using calibrated q_hat
    results = cqr_predict(x_upper, x_lower, mccp_state.q_hat)

    return {
        'lower': results['lower_pred'],
        'upper': results['upper_pred'],
        'intervals': results['intervals'],
    }


# ---------------------------------------------------------------------------
# Example usage
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    # Paths - adjust to your environment
    WORKSPACE = Path('/path/to/MLOpsOrchestrator/shared-workspace')
    MODEL_PATH = WORKSPACE / 'models/mccp-uq/model.h5'
    STATE_PATH = WORKSPACE / 'state/mccp-uq/mccp_state.pkl'

    # Load
    model, mccp_state = load_artifacts(MODEL_PATH, STATE_PATH)

    # Example input (8 features for Energy Efficiency dataset)
    sample_input = np.array(
        [[0.7, 600.0, 300.0, 150.0, 7.0, 3, 0.1, 3]],
        dtype=np.float32,
    )

    # Predict
    results = predict(model, mccp_state, sample_input, target_dim=2)

    print('=== MCCP Inference Results ===')
    for i in range(results['lower'].shape[1]):
        print(
            f"  Target {i}: [{results['lower'][0, i]:.2f}, "
            f"{results['upper'][0, i]:.2f}]"
        )
    print(f"  Mean interval widths: {results['intervals']}")
