# Demo Profiles

This folder contains the two canonical ROBIN profile demos:

- `welding_profile.py` (reference profile)
- `spray_coating_profile.py` (reusability profile)

Both scripts now implement the same robust dual-mode workflow.

## Core model shared by all profiles

| Field | Role | Example mappings |
|---|---|---|
| `measuredHeight` | Primary geometry output | bead height, coating thickness |
| `measuredWidth` | Secondary geometry output | bead width, coverage width |
| `measuredSpeed` | Process speed | wire speed, line speed |
| `measuredCurrent` | Primary process input | welding current, flow rate |
| `measuredVoltage` | Secondary process input | arc voltage, nozzle pressure |

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

## Dual operational logic (both scripts)

### Parameter-driven

1. User sets process parameters.
2. AI predicts geometry from parameters.
3. Telemetry streams.
4. `POST /check-deviation` compares measured geometry with AI prediction.

### Geometry-driven

1. User sets target geometry.
2. AI suggests process parameters.
3. Expected geometry is derived from those AI-suggested parameters.
4. Telemetry streams.
5. `POST /check-deviation` compares measured geometry with expected geometry.

## Profile mapping comparison

| Core field | Welding | Spray Coating |
|---|---|---|
| `measuredHeight` | Bead Height (mm) | Coating Thickness (mm) |
| `measuredWidth` | Bead Width (mm) | Coverage Width (mm) |
| `measuredSpeed` | Wire Speed | Line Speed |
| `measuredCurrent` | Welding Current | Flow Rate |
| `measuredVoltage` | Arc Voltage | Nozzle Pressure |

## Configuration

Profiles are selected via:

```bash
ROBIN_PROFILE=<name> docker compose up -d
```

Profile files live in `config/profiles/` and control vocabulary, units, ROS topics,
skills, AI defaults, and model path.
