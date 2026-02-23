# MCCP-UQ Model Consumption - Phase 1 (Local Filesystem)

Integrate the standalone MCCP inference module from the MLOps team into RobTrack so the FAROS page can run uncertainty-quantified predictions against models trained via the embedded MLO.

## Proposed Changes

### FAROS Page - inference layer

#### [NEW] [mccp_inference.py](file:///Users/orpheus/Documents/Work/RobTrack/RobTrack/src/gui/faros_page/mccp_inference.py)

The MCCP inference module (provided by the MLO team) lives at `faros/inference/mccp_inference.py` - no modifications needed.

---

#### [NEW] [model_provider.py](file:///Users/orpheus/Documents/Work/RobTrack/RobTrack/src/gui/faros_page/model_provider.py)

Abstraction over artifact location, exactly as prescribed in the handoff:

| Class | Purpose |
|---|---|
| `ModelProvider` (ABC) | `get_model_path()`, `get_state_path()`, `load()` |
| `LocalFilesystemProvider` | Reads from the MLO `shared-workspace/` on disk |

The `workspace_root` will default to the path from the handoff but is configurable via constructor arg.

---

#### [NEW] [inference_service.py](file:///Users/orpheus/Documents/Work/RobTrack/RobTrack/src/gui/faros_page/inference_service.py)

Thin service layer that owns the lifecycle of the loaded model:

- `load()` - loads model + state via the provider; records `mtime` of both files
- `predict(input_features, target_dim)` - delegates to `mccp_inference.predict()`
- `is_stale()` → `bool` - compares current file `mtime` to cached `mtime`
- `reload_if_stale()` - calls `load()` if stale
- `is_available` property - True when artifacts exist on disk

Handles the "no model yet" case gracefully (returns `None` / raises clear error).

---

#### [MODIFY] [faros_page.py](file:///Users/orpheus/Documents/Work/RobTrack/RobTrack/src/gui/faros_page/faros_page.py)

Instantiate the `InferenceService` with a `LocalFilesystemProvider` and expose it as `self.inference_service` so the rest of the application (or future UI components) can call `predict()`.

---

### Dependencies

#### [MODIFY] [pyproject.toml](file:///Users/orpheus/Documents/Work/RobTrack/RobTrack/pyproject.toml)

Add under `[tool.poetry.dependencies]`:

```toml
tensorflow = "^2.18.0"
tqdm = "^4.66.0"
```

> [!NOTE]
> `numpy` and `scikit-learn` are already satisfied by existing deps.

---

### FAROS Visual Dev Surface (Development Only)

To validate MCCP consumption interactively inside FAROS, add a dedicated **Inference DevLab** tab in the visual page:

- `visual_page/app/src/components/features/inference/InferenceDevLabTab.tsx`
  - Paste feature vectors as JSON
  - Call a local inference endpoint
  - Display lower/upper bounds and interval widths per target
- `visual_page/app/src/components/layout/Sidebar.tsx`
  - Add navigation item: `Inference DevLab`
- `visual_page/app/src/components/layout/FarosPage.tsx`
  - Render `InferenceDevLabTab` when `tab === "inference"`

Use a small local API wrapper for development:

- `visual_page/mccp_inference_devlab_server.py`
  - Loads artifacts from shared workspace (`model.h5`, `mccp_state.pkl`)
  - Calls `faros/inference/mccp_inference.py` directly
  - Exposes `GET /health` and `POST /predict`
  - Returns calibrated intervals and derived uncertainty width

This keeps production integration concerns separated while giving a concrete in-app checkpoint that model loading + UQ inference works end-to-end.

---

## Verification Plan

### Automated Tests

#### Unit tests - `tests/test_mccp_integration.py`

1. **`test_local_provider_paths`** - verify `LocalFilesystemProvider` returns the expected `Path` objects.
2. **`test_provider_load_missing_files`** - verify `FileNotFoundError` when artifacts are absent.
3. **`test_inference_service_availability`** - `is_available` returns `False` when files don't exist, `True` when they do.
4. **`test_inference_service_staleness`** - mock `os.path.getmtime` to simulate file updates; assert `is_stale()` detects them.

Run with:

```bash
python -m pytest tests/test_mccp_integration.py -v
```

#### Smoke test - real artifacts

A standalone script (or pytest test marked `@pytest.mark.slow`) that loads the actual model + state from the shared workspace and runs `predict()` on a sample input. This validates the full end-to-end pipeline including TensorFlow model loading.

```bash
python -m pytest tests/test_mccp_integration.py -v -k "smoke" --no-header
```

### Manual Verification

Since the integration is purely backend (no UI changes yet), manual verification is limited to confirming the model loads without errors on the dev machine. The user can run:

```bash
python -c "
from src.gui.faros_page.inference_service import InferenceService
from src.gui.faros_page.model_provider import LocalFilesystemProvider
svc = InferenceService(LocalFilesystemProvider())
svc.load()
import numpy as np
result = svc.predict(np.array([[0.7, 600.0, 300.0, 150.0, 7.0, 3, 0.1, 3]], dtype=np.float32))
print(result)
"
```
