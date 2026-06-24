#!/usr/bin/env python3
"""Visualise a DOE JSON file as 2D scatter projections + 3D scatter."""

import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

def load_doe(path: str):
    with open(path) as f:
        data = json.load(f)
    beads = data["beads"]
    wfs = np.array([b["wire_feed_speed"] for b in beads])
    ws = np.array([b["weld_speed"] for b in beads])
    alc = np.array([b["arc_length_correction_mm"] for b in beads])
    return wfs, ws, alc

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else str(
        Path(__file__).parent / "DOE_LHS_48.json")

    wfs, ws, alc = load_doe(path)
    n = len(wfs)
    name = Path(path).stem

    fig = plt.figure(figsize=(14, 4), constrained_layout=True)
    fig.suptitle(f"{name}  ({n} beads)", fontsize=13)

    # 2D projections
    axes_cfg = [
        (wfs, ws * 1000, "Wire Feed Speed (m/min)", "Weld Speed (mm/s)"),
        (wfs, alc, "Wire Feed Speed (m/min)", "Arc Length Corr. (mm)"),
        (ws * 1000, alc, "Weld Speed (mm/s)", "Arc Length Corr. (mm)"),
    ]
    for i, (x, y, xl, yl) in enumerate(axes_cfg, 1):
        ax = fig.add_subplot(1, 4, i)
        ax.scatter(x, y, s=20, alpha=0.8, edgecolors="k", linewidths=0.3)
        ax.set_xlabel(xl, fontsize=9)
        ax.set_ylabel(yl, fontsize=9)
        ax.tick_params(labelsize=8)

    # 3D scatter
    ax3 = fig.add_subplot(1, 4, 4, projection="3d")
    ax3.scatter(wfs, ws * 1000, alc, s=20, alpha=0.8, edgecolors="k", linewidths=0.3)
    ax3.set_xlabel("WFS (m/min)", fontsize=8)
    ax3.set_ylabel("WS (mm/s)", fontsize=8)
    ax3.set_zlabel("ALC (mm)", fontsize=8)
    ax3.tick_params(labelsize=7)

    plt.show()

if __name__ == "__main__":
    main()
