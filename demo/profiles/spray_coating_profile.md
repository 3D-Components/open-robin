# Spray Coating Profile

This profile is the reusability proof for ROBIN: same core modules, different
industrial vocabulary and parameter ranges.

## Canonical script

```bash
python demo/profiles/spray_coating_profile.py --mode both --duration 120 --interval 2
```

Use spray profile config first:

```bash
ROBIN_PROFILE=spray_coating docker compose up -d
```

## Dual operational modes in this demo

### Parameter-driven

- Operator provides line speed / flow / pressure setpoints.
- AI predicts expected coating geometry.
- Streamed measurements are checked against AI prediction.

### Geometry-driven

- Operator provides target thickness and coverage width.
- AI suggests process parameters.
- Expected geometry from AI-guided parameters is compared against measured geometry.

## Metric mapping

| Coating term | Core field | Typical range |
|---|---|---|
| Coating Thickness | `measuredHeight` | 0.05–0.20 mm |
| Coverage Width | `measuredWidth` | 30–70 mm |
| Line Speed | `measuredSpeed` | 150–250 mm/s |
| Flow Rate | `measuredCurrent` | 30–60 ml/min |
| Nozzle Pressure | `measuredVoltage` | 2.5–4.5 bar |

## Core reusability claim

What changes for spray:

- `config/profiles/spray_coating.yaml`
- `demo/profiles/spray_coating_profile.py`
- domain wording/units in UI

What stays the same:

- alert engine and deviation pipeline
- AI model interface (`3 inputs -> 2 outputs`)
- dashboard architecture
- FIWARE + Orion + Mintaka data path
