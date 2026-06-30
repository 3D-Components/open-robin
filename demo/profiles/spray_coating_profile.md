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
- The script asks the API for expected coating geometry and falls back to the
  configured default coating geometry when no valid spray-model prediction is
  available.
- Streamed measurements are checked against the resolved expected geometry.

### Geometry-driven

- Operator provides target thickness and coverage width.
- The script asks the API for process parameters and a predicted geometry when
  available.
- Because this release does not commit a spray-coating model, the no-model path
  uses the target thickness and coverage width as the deviation reference.

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
- AI model interface (`3 inputs -> 2 outputs`) when a profile model is provided
- dashboard architecture
- FIWARE + Orion + Mintaka data path
