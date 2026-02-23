# FAROS - Welding Bead Simulator & Inference

This directory contains the FAROS web application stack: a React frontend,
Python microservices, and the MCCP-UQ inference engine. It is embedded into
the main RobTrack PySide6 desktop application via `src/gui/faros_page/`, which
manages the subprocess lifecycle and hosts a `QWebEngineView`.

## Directory Structure

```
faros/
├── frontend/          # React 19 + Vite + TypeScript + Tailwind CSS
│   ├── src/           #   Application source (components, hooks, types, utils)
│   ├── public/        #   Static assets (URDF models, STL meshes)
│   ├── package.json   #   NPM dependencies
│   └── vite.config.ts #   Vite build configuration
│
├── services/          # Python backend microservices
│   ├── viser_server.py                  # 3D robot visualization (Viser + WebSocket)
│   └── mccp_inference_devlab_server.py  # HTTP API for MCCP-UQ inference
│
├── inference/         # Core ML inference module
│   └── mccp_inference.py  # Standalone MCCP-UQ (MC Dropout + CQR)
│
└── docs/              # Integration documentation
    ├── MLOps-Handoff.md
    └── MCCP-Integration-Plan.md
```

## Quick Start

### Frontend (React)

```bash
cd faros/frontend
npm install
npm run dev          # Vite dev server on http://localhost:5174
```

### Viser Server (3D Visualization)

```bash
cd faros/services
python viser_server.py   # HTTP :8081, WebSocket :8082
```

### MCCP Inference API

```bash
cd faros/services
MCCP_WORKSPACE_ROOT=/path/to/shared-workspace python mccp_inference_devlab_server.py
# HTTP API on :8091
```

### Auto-Start (via RobTrack)

When the FAROS page is opened in the RobTrack desktop app, all three services
are started automatically as background processes. See `src/gui/faros_page/faros_page.py`.

## Architecture

```
┌──────────────────────────────────────────────┐
│          RobTrack Desktop (PySide6)          │
│  src/gui/faros_page/faros_page.py            │
│  └─ QWebEngineView → http://localhost:5174   │
└──────────────┬───────────────────────────────┘
               │ spawns subprocesses
    ┌──────────┼──────────────┐
    ▼          ▼              ▼
 React App   Viser Server   MCCP DevLab API
 :5174       :8081/:8082    :8091
```

## Environment Variables

| Variable                      | Default       | Description                             |
|-------------------------------|---------------|-----------------------------------------|
| `FAROS_HOST`                  | `localhost`   | React app host                          |
| `FAROS_PORT`                  | `5174`        | React app port                          |
| `VISER_PORT`                  | `8081`        | Viser HTTP port                         |
| `VISER_WS_PORT`               | `8082`        | Viser WebSocket port                    |
| `MCCP_API_HOST`               | `0.0.0.0`    | MCCP API bind address                   |
| `MCCP_API_PORT`               | `8091`        | MCCP API port                           |
| `MCCP_WORKSPACE_ROOT`         | *(none)*      | Path to MLOps shared-workspace          |
| `MCCP_USE_GPU`                | `0`           | Set to `1` for GPU inference            |
| `FAROS_AUTO_START_VISER`      | `1`           | Set to `0` to skip Viser auto-start     |
| `FAROS_AUTO_START_REACT`      | `1`           | Set to `0` to skip React auto-start     |
| `FAROS_AUTO_START_MCCP_DEVLAB`| `1`           | Set to `0` to skip MCCP API auto-start  |
