# RobTrack Integration Handoff: MCCP-UQ Model Consumption

**Date:** 2026-02-10
**From:** MLOps Orchestrator team
**To:** RobTrack developer(s)

---

## Context

RobTrack embeds the MLOps Orchestrator (MLO) UI as a webview/iframe. Through that interface, users design and execute workflows that train AI models with uncertainty quantification (UQ) using the **MCCP-UQ** processor.

RobTrack needs to **consume the trained model artifacts** produced by MCCP-UQ to run inference in its own process.

This will be implemented in two phases. **Please read both before starting**, because Phase 2 should influence how you structure Phase 1.

---

## Phase 1: Local Shared Filesystem (current scope)

Since RobTrack and the MLO run on the same machine, Phase 1 reads model artifacts directly from the MLO's shared workspace on disk.

### What MCCP-UQ produces

After a training/calibration DAG runs in the MLO, two files are written:

| Artifact | Host path | Format | Contents |
|---|---|---|---|
| Trained model | `shared-workspace/models/mccp-uq/model.h5` | Keras H5 | MQNN architecture + weights (Multi-Quantile Neural Network) |
| Calibration state | `shared-workspace/state/mccp-uq/mccp_state.pkl` | Python pickle | `q_hat` (calibration threshold), hyperparameters. **Model reference is `None`** - the model must be loaded separately and injected. |

Absolute path on the current dev machine:

```
/Users/orpheus/Documents/Work/3DC/FAROS/Components/MLOpsOrchestrator/shared-workspace/
├── models/mccp-uq/model.h5           ← trained model
├── state/mccp-uq/mccp_state.pkl      ← calibration state
├── inputs/mccp-uq/                    ← training data (not needed for inference)
└── outputs/mccp-uq/                   ← DAG execution logs
```

Use this as the `workspace_root` for the `LocalFilesystemProvider`:

```
/Users/orpheus/Documents/Work/3DC/FAROS/Components/MLOpsOrchestrator/shared-workspace
```

### What we're providing you

Instead of requiring you to import the full `mccp_lib` library (which includes training, calibration, dataset loaders, ONNX export, and classification paths that RobTrack doesn't need), we've extracted a **single self-contained inference module** (`faros/inference/mccp_inference.py`):

```
temp_mccp/
└── mccp_inference.py    ← drop this into your project
```

This file (~170 lines) contains only the three functions needed for inference:

| Function | What it does | Origin in mccp_lib |
|---|---|---|
| `dynamic_mc_predict()` | Adaptive MC dropout - runs N stochastic forward passes, stops when variance stabilizes | `mccp_lib/dynamic_mc/dynamic_mc.py` |
| `cqr_predict()` | Applies conformal correction: `lower - q_hat`, `upper + q_hat` | `mccp_lib/conformal_prediction/cqr.py` |
| `load_artifacts()` | Loads `.h5` model + `.pkl` state, injects model into state | New convenience wrapper |
| `predict()` | High-level entrypoint: MC dropout → split quantiles → CQR correction | `mccp_lib/mccp/prediction.py` (simplified) |

**This is the exact same inference algorithm used inside MCCP-UQ**, not a reimplementation. It's just the inference slice extracted into one file.

### Python dependencies

```
numpy==1.26.4
tensorflow==2.18.0
tqdm
scikit-learn
```

(`pandas` is NOT required for inference.)

### Usage

```python
import numpy as np
from mccp_inference import load_artifacts, predict

# Load artifacts (paths should come from your config - see design note below)
model, mccp_state = load_artifacts(
    model_path="/path/to/shared-workspace/models/mccp-uq/model.h5",
    state_path="/path/to/shared-workspace/state/mccp-uq/mccp_state.pkl",
)

# Prepare input - shape (n_samples, 8) for the Energy Efficiency dataset
input_features = np.array([[0.7, 600.0, 300.0, 150.0, 7.0, 3, 0.1, 3]], dtype=np.float32)

# Run inference with uncertainty quantification
results = predict(model, mccp_state, input_features, target_dim=2)

print(results["lower"])      # shape (n_samples, n_targets) - lower prediction bounds
print(results["upper"])      # shape (n_samples, n_targets) - upper prediction bounds
print(results["intervals"])  # shape (n_targets,) - mean interval widths
```

### MCCP hyperparameters (for reference)

These are baked into the calibration state at training time. You do NOT set them at inference - they are read from the pickle. Listed here for understanding only:

| Parameter | Default | Meaning |
|---|---|---|
| `ALPHA` | `0.1` | Miscoverage rate (0.1 = 90% prediction interval coverage) |
| `PATIENCE` | `2` | MC dropout convergence patience |
| `MAX_MC` | `50` | Max Monte Carlo forward passes |
| `DELTA` | `5e-4` | Variance convergence threshold |

### Design note: abstract the artifact source

This is the most important implementation guidance for Phase 1. **Do not hardcode the shared-workspace path.** Wrap artifact loading behind an abstraction so Phase 2 is a clean swap:

```python
from abc import ABC, abstractmethod
from pathlib import Path
from mccp_inference import load_artifacts

class ModelProvider(ABC):
    """Where model artifacts come from."""

    @abstractmethod
    def get_model_path(self) -> Path: ...

    @abstractmethod
    def get_state_path(self) -> Path: ...

    def load(self):
        return load_artifacts(self.get_model_path(), self.get_state_path())


class LocalFilesystemProvider(ModelProvider):
    """Phase 1: reads from MLO shared-workspace on disk."""

    def __init__(self, workspace_root: str = "/Users/orpheus/Documents/Work/3DC/FAROS/Components/MLOpsOrchestrator/shared-workspace"):
        self.root = Path(workspace_root)

    def get_model_path(self):
        return self.root / "models/mccp-uq/model.h5"

    def get_state_path(self):
        return self.root / "state/mccp-uq/mccp_state.pkl"
```

Phase 2 adds a `MinIOProvider` that implements the same interface - see below.

### Other implementation tips

1. **Check file existence before loading.** The artifacts only exist after a training DAG has been run. Handle the "no model yet" case gracefully in the UI.

2. **Cache the loaded model in memory.** Loading `model.h5` takes a few seconds. Load once and reuse across inference calls until the user retrains.

3. **Detect when artifacts have been updated.** After the user runs a new training DAG in the MLO webview, the model files change. Use file modification timestamps (`os.path.getmtime()`) to detect this and reload. In Phase 2, this becomes checking for a new version in the MinIO bucket.

---

## Phase 2: MinIO Artifact Store (planned, do not implement yet)

### Why

The shared filesystem approach has limitations: no versioning (model is overwritten each training run), path/permission coupling between MLO and RobTrack, and no cross-machine support. Phase 2 replaces the filesystem read with a download from an S3-compatible object store.

### What changes

| Concern | Phase 1 | Phase 2 |
|---|---|---|
| Where artifacts live | Host filesystem (`shared-workspace/`) | MinIO S3 bucket (`mccp-models`) |
| How RobTrack gets them | Direct file read | `boto3` S3 download to local cache |
| Versioning | None (overwritten each run) | S3 keys include version/timestamp |
| Cross-machine | No | Yes |

### What stays the same

- **`mccp_inference.py` is identical.** The inference algorithm doesn't change.
- Python dependencies are the same (plus `boto3` for the S3 client).
- The `predict()` call and its inputs/outputs are the same.

### What we (MLO team) will do

1. Add a MinIO container to the MLO deployment.
2. Modify the MCCP-UQ plugin to upload `model.h5` + `mccp_state.pkl` to a MinIO bucket after training.
3. Provide you with: endpoint URL, bucket name, access credentials, and key naming convention.

### What RobTrack will need to add

A new provider (~30 lines) plus `boto3` as a dependency:

```python
import boto3

class MinIOProvider(ModelProvider):
    """Phase 2: downloads from MinIO S3 bucket."""

    def __init__(self, endpoint, bucket, access_key, secret_key, version="latest"):
        self.s3 = boto3.client("s3",
            endpoint_url=endpoint,
            aws_access_key_id=access_key,
            aws_secret_access_key=secret_key,
        )
        self.bucket = bucket
        self.version = version
        self.cache_dir = Path("~/.robtrack/model_cache").expanduser()
        self.cache_dir.mkdir(parents=True, exist_ok=True)

    def get_model_path(self) -> Path:
        local = self.cache_dir / f"{self.version}_model.h5"
        if not local.exists():
            self.s3.download_file(
                self.bucket, f"mccp-uq/{self.version}/model.h5", str(local)
            )
        return local

    def get_state_path(self) -> Path:
        local = self.cache_dir / f"{self.version}_mccp_state.pkl"
        if not local.exists():
            self.s3.download_file(
                self.bucket, f"mccp-uq/{self.version}/mccp_state.pkl", str(local)
            )
        return local
```

Usage stays the same:

```python
# Phase 1:
provider = LocalFilesystemProvider("/Users/orpheus/Documents/Work/3DC/FAROS/Components/MLOpsOrchestrator/shared-workspace")

# Phase 2 (swap one line):
provider = MinIOProvider("http://minio:9000", "mccp-models", "key", "secret")

# Everything else identical:
model, mccp_state = provider.load()
results = predict(model, mccp_state, input_features)
```

### PyTorch support (future note)

A PyTorch variant of the MC dropout algorithm exists at `AIT-pytorch/mccp_lib/dynamic_mc/dynamic_mc_pytorch.py`. If RobTrack ever needs to run inference on PyTorch models (e.g., for classification use cases), we can extract a `mccp_inference_pytorch.py` following the same pattern. The CQR prediction logic is framework-agnostic (pure numpy). Only the MC dropout forward pass differs between TensorFlow and PyTorch.

---

## Deliverables summary

| Item | Detail |
|---|---|
| **File to integrate** | `temp_mccp/mccp_inference.py` (single file, ~170 lines) |
| **Python deps** | `tensorflow==2.18.0`, `numpy==1.26.4`, `tqdm`, `scikit-learn` |
| **Artifact paths** | `/Users/orpheus/Documents/Work/3DC/FAROS/Components/MLOpsOrchestrator/shared-workspace/models/mccp-uq/model.h5` + `.../state/mccp-uq/mccp_state.pkl` |
| **Key design pattern** | `ModelProvider` abstraction so artifact source can be swapped for Phase 2 |
| **What NOT to do** | Don't hardcode paths. Don't couple inference logic to file I/O. Don't implement MinIO yet. |
| **Reference** | `run_standalone_inference.py` in the MCCP-UQ module (full working example) |

---

## Questions / contact

If anything is unclear about the artifact format, the inference algorithm, or the prediction output structure, reach out to the MLO integration team.
