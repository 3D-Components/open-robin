#!/usr/bin/env python3
"""Generate a Latin Hypercube Sampling DOE JSON for the ROBIN experiment system.

Interactive prompts ask for parameter ranges and sample count.
Output matches the ExperimentBeadSpec.msg schema (input_id, wire_feed_speed,
weld_speed, arc_length_correction_mm).

Usage:
    python3 generate_doe.py                     # interactive
    python3 generate_doe.py -n 48 -o my_doe.json  # with overrides
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def ask_float(prompt: str, default: float) -> float:
    raw = input(f"{prompt} [{default}]: ").strip()
    return float(raw) if raw else default


def ask_int(prompt: str, default: int) -> int:
    raw = input(f"{prompt} [{default}]: ").strip()
    return int(raw) if raw else default


def lhs_sample(n: int, d: int, seed: int) -> np.ndarray:
    """Return (n, d) array of LHS samples in [0, 1]."""
    rng = np.random.default_rng(seed)
    result = np.zeros((n, d))
    for j in range(d):
        order = rng.permutation(n)
        for i in range(n):
            result[i, j] = (order[i] + rng.random()) / n
    return result


def main():
    parser = argparse.ArgumentParser(description="Generate LHS DOE JSON")
    parser.add_argument("-n", type=int, help="Number of beads")
    parser.add_argument("-o", "--output", type=str, help="Output JSON path")
    parser.add_argument("--seed", type=int, help="Random seed")
    parser.add_argument("--non-interactive", action="store_true",
                        help="Use defaults, no prompts")
    args = parser.parse_args()

    if args.non_interactive:
        ask = lambda prompt, default: default
    else:
        ask = ask_float

    n = args.n or ask_int("Number of beads", 48) if not args.non_interactive else (args.n or 48)
    seed = args.seed if args.seed is not None else (
        ask_int("Random seed", 42) if not args.non_interactive else 42)

    print("\n── Wire Feed Speed (m/min) ──")
    wfs_min = ask("  Min", 2.0)
    wfs_max = ask("  Max", 19.0)

    print("── Weld Speed (m/s) ──")
    ws_min = ask("  Min", 0.001)
    ws_max = ask("  Max", 0.030)

    print("── Arc Length Correction (mm) ──")
    alc_min = ask("  Min", -10.0)
    alc_max = ask("  Max", 10.0)

    # Generate LHS samples
    samples = lhs_sample(n, 3, seed)

    wfs = samples[:, 0] * (wfs_max - wfs_min) + wfs_min
    ws = samples[:, 1] * (ws_max - ws_min) + ws_min
    alc = samples[:, 2] * (alc_max - alc_min) + alc_min

    beads = []
    for i in range(n):
        beads.append({
            "input_id": f"LHS{n}-{i + 1:02d}",
            "wire_feed_speed": round(float(wfs[i]), 2),
            "weld_speed": round(float(ws[i]), 4),
            "arc_length_correction_mm": round(float(alc[i]), 1),
        })

    doc = {"beads": beads}

    out_path = args.output or (
        input(f"\nOutput file [DOE_LHS_{n}.json]: ").strip()
        if not args.non_interactive else "")
    if not out_path:
        out_path = f"DOE_LHS_{n}.json"
    out_path = Path(__file__).parent / out_path if not Path(out_path).is_absolute() else Path(out_path)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")

    print(f"\nWrote {n} beads to {out_path}")
    print(f"  WFS:  {wfs_min} – {wfs_max} m/min")
    print(f"  WS:   {ws_min} – {ws_max} m/s")
    print(f"  ALC:  {alc_min} – {alc_max} mm")
    print(f"  Seed: {seed}")


if __name__ == "__main__":
    main()
