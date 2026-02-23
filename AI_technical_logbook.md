# ROBIN MVP - Technical Report (AI slice)

*Living design journal for the AI modelling pipeline. Read it as the companion to the FIWARE logbook, focused on how we simulate, train, and serve process intelligence.*

> Note: this logbook includes historical MVP notes. For the current open-source baseline, use `README.md`, `current_stack_feb22.md`, and `ARISE_PUBLICATION_ROADMAP.md`.

---

## 1) Executive overview

The AI slice adds a self-contained modelling stack that learns bead geometry from process parameters, exposes the model through the Alert Engine, and keeps the workflow operator-friendly for FAROS integration. **For demonstration purposes**, synthetic process data is generated with `scripts/process_geometry_simulator.py` using analytic equations; this mock dataset serves as a placeholder that can be **effortlessly replaced** with real production data once collected and stored for training. An MLP is trained via `scripts/train_geometry_mlp.py`, and trained checkpoints live under `data/models/`. The Alert Engine provides REST endpoints to list, select, and run the active model so dashboards can stay stateless and modular.

---

## 2) Architecture in plain language

**Data generation (demonstration mode).** `scripts/process_geometry_simulator.py` reuses analytic process equations to create input/output pairs (travel speed, current, voltage → bead width/height). This synthetic approach demonstrates the pipeline end-to-end without requiring real process trials. The script writes `data/process_dataset_input_output.csv` and a diagnostic plot in the same folder. **When transitioning to production**, simply replace this CSV with real measurement data in the same format-no code changes required beyond pointing to the new file.

**Model definition.** `robin/ai/mlp.py` contains a configurable PyTorch MLP (`ProcessGeometryMLP`) and helpers to serialise checkpoints (`save_model`, `load_model`). Normalisation statistics (mean/std) and feature order are stored inside the checkpoint so inference can remain consistent even if features change.

**Training workflow.** `scripts/train_geometry_mlp.py` reads the dataset from `data/`, splits into train/validation sets, trains the MLP, and writes checkpoints to `data/models/` (default filename `process_geometry_mlp.pt`). The script can regenerate the synthetic dataset (`--generate-if-missing`) to keep the demonstration flow reproducible. **To train on real data**, place your collected measurements in `data/process_dataset_input_output.csv` with columns `[wireSpeed, current, voltage, beadWidth, beadHeight]` and run the same training script-the pipeline seamlessly adapts to real-world inputs.

**Serving path.** The Alert Engine (`robin/alert_engine.py`) maintains a singleton `AlertEngine` instance that discovers checkpoints, loads the active one, and exposes REST endpoints for listing models, selecting a checkpoint, and running predictions. If no checkpoint is available, it falls back to the legacy heuristic so the API never hard-fails. **Models trained on real data** are served through the same interface with zero API changes.

**FAROS touchpoints.** FAROS widgets only need to talk HTTP. They fetch model metadata, preview predictions, and request recommendations through the Alert Engine. Any model change propagates to Orion-LD via the existing `/ai-recommendation` endpoint, so dashboards can stay synced with backend state-**regardless of whether the model was trained on synthetic or real data**.

---

## 3) REST endpoints (Alert Engine)

| Endpoint | Method | Purpose | Notes |
| --- | --- | --- | --- |
| `/ai/models` | GET | List discovered checkpoints with metadata | Metadata includes size, modified time, stored config, and active flag. |
| `/ai/models/directories` | GET | Show directories scanned for checkpoints | Defaults to `data/models/` and `robin/models/`, optional extra dirs via `ROBIN_MLP_MODEL_DIRS`. |
| `/ai/models/active` | GET | Return the currently loaded model | Responds with `status: inactive` if nothing is loaded. |
| `/ai/models/select` | POST | Load a specific checkpoint | Body: `{ "path": "relative/or/absolute.pt" }`. Relative names are resolved against project root or known model dirs. Errors return 404/400 with clear detail. |
| `/ai/models/predict` | POST | Run a forward pass on the active model | Body: `{ "wireSpeed": float, "current": float, "voltage": float }`. Response includes `prediction`, `feature_order`, and whether the MLP or heuristic handled the request. |
| `/ai-recommendation` | POST | Domain-level AI recommendation | Reuses the active model (if present) to power parameter-driven mode before storing an `AIRecommendation` entity in Orion-LD. |

Each endpoint is stateless: the active checkpoint is kept in-memory by the singleton engine and recorded in responses. Widgets can call `/ai/models` after `/ai/models/select` to refresh their view.

---

## 4) Model lifecycle and storage

**Demonstration workflow (synthetic data):**

1. **Generate mock data** – `python scripts/process_geometry_simulator.py` → produces `data/process_dataset_input_output.csv` and `data/process_dataset_visualization.png`.
2. **Train model** – `python scripts/train_geometry_mlp.py` (add `--generate-if-missing` if the dataset is absent). The script ensures both `data/` and `data/models/` exist.
3. **Select checkpoint** – `curl -X POST /ai/models/select -d '{"path":"data/models/process_geometry_mlp.pt"}'` or the equivalent FAROS widget action.
4. **Serve predictions** – `/ai/models/predict` for raw forward passes, `/ai-recommendation` for full process recommendations.

**Production workflow (real data):**

1. **Collect & store data** – Export measurements from process trials (sensors, vision systems, or manual QC) into `data/process_dataset_input_output.csv` with the expected schema: `wireSpeed, current, voltage, beadWidth, beadHeight`. This file **directly replaces** the synthetic dataset.
2. **Train model** – Run `python scripts/train_geometry_mlp.py` exactly as before-no modifications needed. The training script reads whatever data is in `data/process_dataset_input_output.csv`.
3. **Select & serve** – Follow steps 3–4 above. The REST API is data-agnostic; operators interact with real-data models through the same endpoints and FAROS widgets.

Checkpoints bundle the MLP weights, normalisation stats, and feature names, enabling hot swaps without restarting the server. The engine automatically reloads the latest viable checkpoint on startup; if loading fails, it logs the failure and falls back to the heuristic model.

---

## 5) FAROS integration plan

- **AI Control widget** (`wirecloud-widgets/ai-model-control`) – consumes `/ai/models`, `/ai/models/select`, and `/ai/models/predict` to present operators with checkpoint metadata, allow swaps, and preview predictions before applying them. Packaged as `ai-model-control.wgt` via `scripts/wirecloud_tools.sh --bundle-widgets`.
- **Process Controls widget updates** – display the active model (`/ai/models/active`) alongside process controls so operators know which version they are using.
- **Deviation & KPI widgets** – keep using `/ai-recommendation` so new recommendations automatically reference the active checkpoint metadata stored in Orion-LD.
- **Future RobTrack bridge** – the alert engine already routes everything through `AlertEngine.predict_geometry_from_params` and `recommend_params_for_geometry`, so swapping in proprietary models (or remote API calls) will preserve the same REST contract for FAROS.

**All widgets remain unchanged** when transitioning from synthetic to real-data models, ensuring a seamless operator experience.

---

## 6) Operational notes & next steps

- **Dependencies** – install `torch` (declared in `pyproject.toml`) before running training or serving MLP endpoints.
- **Testing** – `pytest tests/test_ai_mlp.py` validates forward-pass shapes and checkpoint round-tripping. Extend with end-to-end API tests once the alert engine runs in CI.
- **Model catalogue** – storing checkpoints under `data/models/` keeps artefacts out of the Python package and compatible with container volume mounts. Additional directories can be advertised through `ROBIN_MLP_MODEL_DIRS`.
- **Data collection strategy** – the synthetic dataset demonstrates feasibility; production deployment requires establishing a data pipeline (e.g., sensor logging, periodic export from SCADA, or integration with vision systems) that writes to the expected CSV format. Once this pipeline is operational, **no further AI stack changes are needed**-simply retrain and redeploy checkpoints.
- **Roadmap** – add asynchronous training hooks (e.g. background tasks that invoke `train_geometry_mlp.py`), push active-model metadata into Orion-LD for historical traceability, and integrate RobTrack's forward/inverse APIs behind the existing interfaces.

---

### Quick reference

- **Demonstration dataset:** `data/process_dataset_input_output.csv` (synthetic, generated by `scripts/process_geometry_simulator.py`)
- **Production dataset:** Replace the above CSV with real measurements in the same format
- Trained models: `data/models/*.pt`
- MLP module: `robin/ai/mlp.py`
- Training script: `scripts/train_geometry_mlp.py` (data-source agnostic)
- Serving endpoints: `robin/alert_engine.py` (`/ai/models*`, `/ai/models/predict`, `/ai-recommendation`)
- FAROS widget: `wirecloud-widgets/ai-model-control/`

This logbook should stay synced with code changes. Update the endpoint table and lifecycle steps whenever the AI API evolves so FAROS developers and operators always have an authoritative reference.
