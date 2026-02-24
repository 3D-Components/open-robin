# FAROS - Future Test Plan

This document outlines the tests needed to robustly validate the FAROS
application. Tests are grouped by layer (backend services, frontend, Qt
integration) and listed roughly in priority order within each group.

---

## 1. MCCP-UQ Inference Engine (`robin-ui/inference/mccp_inference.py`)

### 1.1 Artifact Loading

| ID | Test | What to verify |
|----|------|----------------|
| INF-01 | Load valid `.h5` model and `.pkl` state | `load_artifacts()` returns `(model, mccp_state)` without error; `mccp_state.model` is set |
| INF-02 | Load with missing model file | `FileNotFoundError` raised |
| INF-03 | Load with missing state file | `FileNotFoundError` raised |
| INF-04 | Load with corrupted `.h5` file | Keras raises an appropriate error (not a silent bad model) |
| INF-05 | Load with corrupted `.pkl` file | `pickle.UnpicklingError` or similar raised |
| INF-06 | `mccp_lib` stub registration | `_ensure_mccp_lib_stub()` registers all three sub-modules in `sys.modules`; calling it twice is idempotent |

### 1.2 Dynamic MC Dropout

| ID | Test | What to verify |
|----|------|----------------|
| INF-07 | Single sample, variance converges quickly | Returns within `patience` iterations; output shape is `(1, n_outputs)` |
| INF-08 | Multiple samples | Output shape is `(n_samples, n_outputs)` |
| INF-09 | Hits `max_mc` limit without convergence | Returns after exactly `max_mc` passes; no infinite loop |
| INF-10 | Multi-head model output (list/tuple) | Handles `isinstance(prediction_raw, (list, tuple))` branch correctly |
| INF-11 | Single-head model output (ndarray) | Handles the `else` branch correctly |
| INF-12 | `patience=0` edge case | Should stop after the first pair of passes where variance is stable |

### 1.3 CQR Conformal Correction

| ID | Test | What to verify |
|----|------|----------------|
| INF-13 | Basic interval widening | `lower_pred < val_lower` and `upper_pred > val_upper` when `q_hat > 0` |
| INF-14 | `q_hat = 0` | Intervals unchanged: `lower_pred == val_lower`, `upper_pred == val_upper` |
| INF-15 | Per-target `q_hat` array | Different corrections applied per target dimension |
| INF-16 | 1D inputs auto-promoted to 2D | Works when `val_upper.ndim == 1` |
| INF-17 | Scalar `q_hat` broadcast | Works when `q_hat` is a single scalar value |
| INF-18 | With `y_test` provided | Coverage metrics and MAE computed correctly |
| INF-19 | Without `y_test` | Coverage/MAE fields are `None` |

### 1.4 End-to-End Predict

| ID | Test | What to verify |
|----|------|----------------|
| INF-20 | Full pipeline (load → MC → CQR) | Output dict has keys `lower`, `upper`, `intervals` with correct shapes |
| INF-21 | `lower <= upper` for all samples | Prediction intervals are well-ordered |
| INF-22 | Deterministic under seed | Same inputs produce same outputs when TF random seed is fixed |

### 1.5 GPU Configuration

| ID | Test | What to verify |
|----|------|----------------|
| INF-23 | Default (CPU mode) | `tf.config.get_visible_devices('GPU')` returns empty list |
| INF-24 | `MCCP_USE_GPU=1` | GPU devices are not hidden (if hardware available) |

---

## 2. MCCP DevLab API (`robin-ui/services/mccp_inference_devlab_server.py`)

### 2.1 Health Endpoint

| ID | Test | What to verify |
|----|------|----------------|
| API-01 | `GET /health` - artifacts exist | `model_available: true`, correct paths in response |
| API-02 | `GET /health` - no artifacts | `model_available: false`, detail message present |
| API-03 | `GET /health` - after model loaded | `loaded: true` |
| API-04 | `GET /nonexistent` | `404 Not Found` |

### 2.2 Config Endpoint

| ID | Test | What to verify |
|----|------|----------------|
| API-05 | `POST /config` - valid directory | `200 OK`, workspace updated, artifacts check returned |
| API-06 | `POST /config` - non-existent directory | `400 Bad Request`, detail says directory doesn't exist |
| API-07 | `POST /config` - missing `workspace_root` | `400 Bad Request` |
| API-08 | `POST /config` - resets loaded model | After config change, `runtime.model` is `None`; next predict triggers reload |

### 2.3 Predict Endpoint

| ID | Test | What to verify |
|----|------|----------------|
| API-09 | `POST /predict` - valid input | `200 OK`, response has `lower`, `upper`, `intervals`, `widths`, `midpoint` |
| API-10 | `POST /predict` - empty `input_features` | `400 Bad Request` |
| API-11 | `POST /predict` - 1D `input_features` | `400 Bad Request` (must be 2D) |
| API-12 | `POST /predict` - `target_dim < 1` | `400 Bad Request` |
| API-13 | `POST /predict` - missing artifacts | `404 Not Found` with artifact paths |
| API-14 | `POST /predict` - `reload_if_stale=false` | Stale model not reloaded |
| API-15 | `POST /predict` - model reloaded after artifact update | Stale detection triggers reload; response uses new model |

### 2.4 CORS

| ID | Test | What to verify |
|----|------|----------------|
| API-16 | `OPTIONS /predict` | `204 No Content`, CORS headers present |
| API-17 | Response CORS headers | `Access-Control-Allow-Origin: *` on all responses |

### 2.5 Thread Safety

| ID | Test | What to verify |
|----|------|----------------|
| API-18 | Concurrent `/predict` requests | No race condition; lock serialises access; both return valid results |
| API-19 | `/config` during `/predict` | Config waits for predict to release lock (or vice versa); no corruption |

---

## 3. Viser 3D Server (`robin-ui/services/viser_server.py`)

### 3.1 Scene Loading

| ID | Test | What to verify |
|----|------|----------------|
| VIS-01 | URDF loading (UR5, UR3) | Both robots loaded without errors; correct number of mesh nodes |
| VIS-02 | Workpiece STL loading | Mesh added to scene |
| VIS-03 | Missing mesh file | Warning logged, robot still partially loads (graceful skip) |
| VIS-04 | Missing URDF file | `FileNotFoundError` or Viser error (expected - should be caught) |

### 3.2 WebSocket Bridge

| ID | Test | What to verify |
|----|------|----------------|
| VIS-05 | Valid JSON message | `latest_state` updated for both robots |
| VIS-06 | Invalid JSON | Ignored silently (no crash) |
| VIS-07 | Partial message (only `robotA`) | `robotA` updated, `robotB` unchanged |
| VIS-08 | Connection close | Logged, server continues running |
| VIS-09 | Rapid reconnect | New connection accepted; state updates resume |

### 3.3 Animation Loop

| ID | Test | What to verify |
|----|------|----------------|
| VIS-10 | Running state | `anim_time` advances; joint config changes each frame |
| VIS-11 | Idle state | Joints smoothly return toward `INITIAL_CFG` |
| VIS-12 | Paused state | Joint config frozen; `anim_time` does not advance |
| VIS-13 | State transition Running → Idle | Robot returns to home pose over multiple frames |
| VIS-14 | Different bead indices | `welding_loop_cfg` produces different trajectories for different `bead_index` values |
| VIS-15 | Different robots | Phase offsets produce distinguishable motion for `robotA` vs `robotB` |

### 3.4 Thread Safety

| ID | Test | What to verify |
|----|------|----------------|
| VIS-16 | Concurrent WS write + animation read | No data corruption; lock ensures consistent snapshot |

---

## 4. Qt Integration (`src/gui/faros_page/`)

### 4.1 FarosPage Widget

| ID | Test | What to verify |
|----|------|----------------|
| QT-01 | Instantiation without WebEngine | `WEBENGINE_AVAILABLE=False` path: error label shown, no crash |
| QT-02 | Instantiation with WebEngine | WebView created and loads URL |
| QT-03 | Process autostart (all ports free) | All three subprocesses launched |
| QT-04 | Process skip (ports occupied) | Ports in use → processes not started, info logged |
| QT-05 | `FAROS_AUTO_START_*=0` | Respective process not started |
| QT-06 | Cleanup on `closeEvent` | All processes terminated |
| QT-07 | Cleanup on `aboutToQuit` | All processes terminated |
| QT-08 | Cleanup when process already dead | No error raised |
| QT-09 | Cleanup timeout (process hangs) | `kill()` called after `terminate()` timeout |
| QT-10 | WebEngine cleanup order | Page detached from view before deletion; no "profile still in use" warning |

### 4.2 Port Detection

| ID | Test | What to verify |
|----|------|----------------|
| QT-11 | Free port | `_port_in_use()` returns `False` |
| QT-12 | Occupied port | `_port_in_use()` returns `True` |
| QT-13 | IPv6 address | Works for both IPv4 and IPv6 |
| QT-14 | Permission-denied port (< 1024) | Returns `True` (treats `EACCES` as in use) |

### 4.3 InferenceService

| ID | Test | What to verify |
|----|------|----------------|
| QT-15 | `is_available` - artifacts exist | Returns `True` |
| QT-16 | `is_available` - artifacts missing | Returns `False` |
| QT-17 | `load()` - success | `is_loaded` becomes `True` |
| QT-18 | `load()` - missing files | `FileNotFoundError` raised |
| QT-19 | `is_stale()` - after file modification | Returns `True` |
| QT-20 | `is_stale()` - files unchanged | Returns `False` |
| QT-21 | `is_stale()` - files deleted | Returns `False` (not "stale", just gone) |
| QT-22 | `reload_if_stale()` | Reloads and returns `True` when stale |
| QT-23 | `predict()` without `load()` | `RuntimeError` raised |
| QT-24 | `predict()` after `load()` | Returns valid result dict |

### 4.4 ModelProvider

| ID | Test | What to verify |
|----|------|----------------|
| QT-25 | `LocalFilesystemProvider` with explicit path | Paths resolve correctly |
| QT-26 | `LocalFilesystemProvider` with `MCCP_WORKSPACE_ROOT` env | Reads env var |
| QT-27 | `LocalFilesystemProvider` with no config | Returns placeholder path containing `MCCP_WORKSPACE_NOT_CONFIGURED` |
| QT-28 | `load()` delegates to `mccp_inference.load_artifacts` | Integration works end-to-end |

### 4.5 MLOps Widget

| ID | Test | What to verify |
|----|------|----------------|
| QT-29 | Instantiation without WebEngine | Error label shown |
| QT-30 | Instantiation with WebEngine | WebView loads MLOps URL |
| QT-31 | Refresh button | `webview.reload()` called |
| QT-32 | Open in Browser button | `webbrowser.open()` called with correct URL |
| QT-33 | Cleanup | WebEngine objects cleaned up in correct order |

---

## 5. React Frontend (`robin-ui/frontend/`)

### 5.1 Navigation & Layout

| ID | Test | What to verify |
|----|------|----------------|
| FE-01 | Initial render | FarosPage loads with default tab (Live Ops) |
| FE-02 | Tab switching (all 7 tabs) | Each tab renders its component without error |
| FE-03 | Sidebar active state | Selected tab highlighted correctly |
| FE-04 | Dark mode toggle | `document.documentElement` class toggles; styles update |

### 5.2 Live Ops

| ID | Test | What to verify |
|----|------|----------------|
| FE-05 | Robot start button | Robot state transitions to `"Running"` |
| FE-06 | Robot pause button | State transitions to `"Paused"` |
| FE-07 | Robot abort button | State transitions to `"Idle"`, progress resets |
| FE-08 | Alert generation | Trust threshold breach creates an alert entry |
| FE-09 | Telemetry chart rendering | Recharts renders without error; data points appear |
| FE-10 | Viser iframe load | iframe loads Viser URL; fallback shown on error |

### 5.3 Inference DevLab Tab

| ID | Test | What to verify |
|----|------|----------------|
| FE-11 | Health check - server up | Green status chip shown |
| FE-12 | Health check - server down | Error message displayed |
| FE-13 | Workspace root update | `POST /config` sent; success message shown |
| FE-14 | Run prediction - valid input | Results displayed (intervals, widths, midpoints) |
| FE-15 | Run prediction - invalid JSON input | Validation error shown before request |
| FE-16 | Run prediction - server error | Error chip with detail shown |
| FE-17 | Loading state during prediction | Loading indicator visible while request in-flight |

### 5.4 WebSocket Bridge

| ID | Test | What to verify |
|----|------|----------------|
| FE-18 | Connection established | WebSocket opens to `ws://localhost:8082` |
| FE-19 | Throttling | Messages sent at most every 200 ms |
| FE-20 | Auto-reconnect | After disconnect, reconnects within ~2 s |
| FE-21 | Message format | Payload matches `{ robotA: {...}, robotB: {...} }` schema |

### 5.5 Models & Trust

| ID | Test | What to verify |
|----|------|----------------|
| FE-22 | Model routing dropdown | Selecting a model updates `routing` state |
| FE-23 | Trust threshold sliders | Adjusting sliders updates `warnThreshold` / `stopThreshold` |
| FE-24 | Trust feed chart | Chart renders with simulated data points |

### 5.6 MLOps Tab

| ID | Test | What to verify |
|----|------|----------------|
| FE-25 | iframe embed | MLOps Orchestrator URL loaded in iframe |
| FE-26 | iframe error fallback | Error state shown if URL unreachable |

### 5.7 History & Settings

| ID | Test | What to verify |
|----|------|----------------|
| FE-27 | Run history table | Mock data renders in table format |
| FE-28 | Audit log | Mock log entries displayed |
| FE-29 | Settings toggles | Each toggle updates its corresponding state |

---

## 6. Integration / End-to-End Tests

| ID | Test | What to verify |
|----|------|----------------|
| E2E-01 | Full startup sequence | Qt app → spawns all 3 services → React loads in WebView |
| E2E-02 | Robot start → Viser animation | Click start in React → WS message → Viser robot moves |
| E2E-03 | Inference round-trip | DevLab tab → POST /predict → MCCP engine → results in UI |
| E2E-04 | Config change → prediction | Change workspace via /config → next predict uses new workspace |
| E2E-05 | Graceful shutdown | Close main window → all 3 processes terminated cleanly |
| E2E-06 | Service already running externally | Start Viser manually → open FAROS page → skips autostart, connects to existing |
| E2E-07 | Model hot-reload | Replace `.h5` on disk while server running → next predict loads new model |
| E2E-08 | Network failure (MCCP API down) | React DevLab tab shows error; app does not crash |
| E2E-09 | Network failure (Viser down) | ViserViewer shows connection error with retry; WebSocket reconnects |

---

## 7. Testing Strategy Notes

### Recommended Tooling

| Layer | Framework | Notes |
|-------|-----------|-------|
| Python inference | `pytest` | Unit tests with mocked TF model; fixture for sample `.h5` + `.pkl` |
| Python HTTP API | `pytest` + `urllib` / `httpx` | Start server in fixture, hit endpoints |
| Python Viser | `pytest` + `websockets` | Connect WS client, send messages, verify state |
| Qt integration | `pytest-qt` | `QT_QPA_PLATFORM=offscreen`; mock subprocesses |
| React components | Vitest + React Testing Library | Unit/component tests |
| React E2E | Playwright or Cypress | Full browser tests against running dev server |
| Integration | `pytest` orchestrator | Start all services in fixtures; run E2E scenarios |

### Priority Order

1. **Inference engine** (INF-*) - correctness of ML predictions is safety-critical
2. **MCCP DevLab API** (API-*) - the HTTP contract the frontend depends on
3. **Qt process management** (QT-01 through QT-10) - reliability of the desktop app
4. **Frontend Inference DevLab** (FE-11 through FE-17) - the real-data integration point
5. **Viser server** (VIS-*) - 3D visualisation correctness
6. **Remaining frontend** (FE-*) - UI correctness
7. **End-to-end** (E2E-*) - full stack validation

### Test Data

- A small reference `.h5` model and `.pkl` state should be committed to
  `robin-ui/tests/fixtures/` (or generated by a fixture script) so that inference
  tests can run without the full MLOps workspace.
- Mock/stub TF models can be created with a known architecture for deterministic
  testing of the MC dropout and CQR logic.
