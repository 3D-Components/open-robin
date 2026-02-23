# Module 2 - Monitoring Dashboard (FAROS)

A React operator dashboard for live process monitoring, deviation alerts, AI model
management, and 3D visualization. All domain-specific labels are configurable via
environment variables - switch from welding to spray coating to machining without
touching the code.

## Interface Contract

**Requires:**
- Module 1 (Process Intelligence API) at `VITE_ROBIN_API_URL`

**Provides:**
- Browser UI at port 80 (mapped to 5174 in the integration example)

## Tabs

| Tab | Purpose |
|---|---|
| **Live Ops** | KPI cards, telemetry time-series chart, deviation monitor, process controls, 3D visualization |
| **Models & Trust** | AI model checkpoint management, quick predictions, trust thresholds |
| **MLOps** | Pipeline status, model routing, audit logs |
| **Inference Lab** | Model health checks, prediction testing |
| **History** | Run summaries, audit logs |
| **Robots** | Robot cell status and control |
| **Settings** | Dark mode, trust thresholds, connection status |

## Domain Vocabulary

Every domain-specific label is configurable via `VITE_TERM_*` build-time environment
variables. Defaults are generic; override for your domain:

| Variable | Default | Welding example | Spray Coating example |
|---|---|---|---|
| `VITE_TERM_PROCESS` | Process | Weld | Coat Job |
| `VITE_TERM_SPEED` | Speed | Wire Speed | Line Speed |
| `VITE_TERM_SPEED_UNIT` | mm/s | m/min | mm/s |
| `VITE_TERM_CURRENT` | Current | Current | Flow Rate |
| `VITE_TERM_CURRENT_UNIT` | A | A | ml/min |
| `VITE_TERM_VOLTAGE` | Voltage | Voltage | Pressure |
| `VITE_TERM_VOLTAGE_UNIT` | V | V | bar |
| `VITE_TERM_PROFILE_HEIGHT` | Profile Height | Bead Height | Thickness |
| `VITE_TERM_PROFILE_WIDTH` | Profile Width | Bead Width | Coverage Width |
| `VITE_TERM_TOOL_PATH` | Tool path | Torch path | Spray Path |
| `VITE_TERM_WORKPIECE` | Workpiece | Workpiece | Substrate |

Variables are resolved at build time in `src/config/domain.ts`.

## Deployment

**As a container (recommended):**

```bash
docker build -t robin-dashboard \
  --build-arg VITE_ROBIN_API_URL=http://your-api:8001 \
  faros/frontend/
docker run -p 5174:80 robin-dashboard
```

**For development:**

```bash
cd faros/frontend
npm ci
npm run dev
```

## Tech Stack

React 19, Vite, TypeScript, Tailwind CSS, Recharts, Lucide icons.
Production build is served by Nginx Alpine.
