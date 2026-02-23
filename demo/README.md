# Demo Guide - Canonical Robust Demos

This directory is intentionally focused on **two robust demos**:

- `python demo/profiles/welding_profile.py`
- `python demo/profiles/spray_coating_profile.py`

These are the primary demonstrations of telemetry, UI updates, and real AI-driven
alert generation in both operational modes.

## What "robust" means in ROBIN

A robust demo must do all of the following in one workflow:

1. Stream realistic telemetry samples continuously.
2. Update dashboard KPIs/charts in real time.
3. Execute dual operational logic explicitly:
   - **Parameter-driven**: parameters -> AI predicted geometry -> deviation check.
   - **Geometry-driven**: target geometry -> AI suggested parameters -> deviation check.
4. Call backend deviation checks continuously (`POST /check-deviation`).
5. Trigger real alerts from injected simulation deviation windows (not static mock alerts).

## Prerequisites

- Docker + Docker Compose
- Python 3.12+
- `poetry install`

## Start the stack

```bash
docker compose up -d
./demo/validate-setup.sh
```

Open the dashboard at <http://localhost:5174>.

## Demo 1: Welding (reference)

```bash
python demo/profiles/welding_profile.py --mode both --duration 120 --interval 2
```

This run creates one parameter-driven and one geometry-driven process (when `--mode both`)
and prints per-sample deviation results.

## Demo 2: Spray coating (reusability proof)

Use spray profile vocabulary/model:

```bash
ROBIN_PROFILE=spray_coating docker compose up -d
```

Then run:

```bash
python demo/profiles/spray_coating_profile.py --mode both --duration 120 --interval 2
```

## Verify in the dashboard

While a demo is running:

1. Select the process ID from the top bar.
2. Confirm live KPI updates and chart movement.
3. In **Process Controls**, switch modes and apply settings.
4. In **Deviation Monitor**, verify:
   - mode is correct,
   - expected source is shown,
   - alerts appear during deviation windows.

## Mode-specific runs

```bash
python demo/profiles/welding_profile.py --mode parameter_driven
python demo/profiles/welding_profile.py --mode geometry_driven

python demo/profiles/spray_coating_profile.py --mode parameter_driven
python demo/profiles/spray_coating_profile.py --mode geometry_driven
```

## Supporting utilities (non-canonical)

These scripts remain available for setup/integration tasks:

- `demo/validate-setup.sh`
- `demo/cleanup-demo.sh`
- `demo/simulation-demo-rosbag.sh`
- `demo/interactive-demo.sh`

## Cleanup

```bash
./demo/cleanup-demo.sh weld-
./demo/cleanup-demo.sh coating-
```

For full temporal reset see: `docs/RESET_DEMO_DATA.md`.
