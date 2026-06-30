# Demo Profiles

This folder contains the two canonical ROBIN profile demos:

- `welding_profile.py` (reference profile)
- `spray_coating_profile.py` (reusability profile)

Both scripts implement the same robust dual-mode workflow. The welding reference
profile is AI-backed in this release. The spray-coating profile uses the same
API and dashboard flow, but no spray-coating model is committed, so its demo
prints the selected expectation source and falls back to configured/default
geometry when no valid spray prediction is available.

## Geometry telemetry + process input snapshots

| Field | Role | Example mappings |
|---|---|---|
| `measuredHeight` | Primary geometry output | bead height, coating thickness |
| `measuredWidth` | Secondary geometry output | bead width, coverage width |
| `inputParams` | Controlled process input snapshot when available | welding AI inputs, other profile setpoints |

## Canonical runs

### Welding

```bash
python demo/profiles/welding_profile.py --mode both --duration 120 --interval 2
```

### Spray coating

```bash
ROBIN_PROFILE=spray_coating docker compose up -d
python demo/profiles/spray_coating_profile.py --mode both --duration 120 --interval 2
```

## Dual operational logic

### Parameter-driven

1. User sets process parameters.
2. Expected geometry is resolved from the AI path when a profile model is available.
3. Telemetry streams.
4. `POST /check-deviation` compares measured geometry with the resolved expected geometry.

### Geometry-driven

1. User sets target geometry.
2. AI suggests process parameters when a profile model is available.
3. Expected geometry is derived from AI-suggested parameters or, for no-model profiles, the target geometry.
4. Telemetry streams.
5. `POST /check-deviation` compares measured geometry with expected geometry.

## Profile mapping comparison

| Core field | Welding | Spray Coating |
|---|---|---|
| `measuredHeight` | Bead Height (mm) | Coating Thickness (mm) |
| `measuredWidth` | Bead Width (mm) | Coverage Width (mm) |
| `inputParams` | Wire Feed Speed / Travel Speed / Arc Length Correction | Optional setpoint snapshot |

## Configuration

Profiles are selected via:

```bash
ROBIN_PROFILE=<name> docker compose up -d
```

Profile files live in `config/profiles/` and control vocabulary, units, ROS topics,
skills, AI defaults, and model path.
