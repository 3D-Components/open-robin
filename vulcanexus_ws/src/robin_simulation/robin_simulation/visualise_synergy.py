#!/usr/bin/env python3
"""Plot synergy model: WFS→Current and WFS→Voltage (multiple arc length corrections)."""

import numpy as np
import matplotlib.pyplot as plt
from synergy_model import synergy_lookup, ARC_LENGTH_VOLTAGE_GAIN

wfs_range = np.linspace(2.0, 19.0, 500)
arc_corrections = [-10, -5, 0, 5, 10]

fig, (ax_i, ax_v) = plt.subplots(1, 2, figsize=(14, 6))

# Current vs WFS (arc length doesn't affect current)
currents = [synergy_lookup(w, 0.0) for w in wfs_range]
ax_i.plot(wfs_range, [c[1] for c in currents], 'b-', linewidth=2)
ax_i.set_xlabel('WFS (m/min)')
ax_i.set_ylabel('Current (A)')
ax_i.set_title('Synergy: WFS → Current')
ax_i.grid(True, alpha=0.3)

# Voltage vs WFS for each arc length correction
for alc in arc_corrections:
    results = [synergy_lookup(w, alc) for w in wfs_range]
    label = f'ALC = {alc:+.0f}'
    ax_v.plot(wfs_range, [r[2] for r in results], linewidth=1.5, label=label)

ax_v.set_xlabel('WFS (m/min)')
ax_v.set_ylabel('Voltage (V)')
ax_v.set_title(f'Synergy: WFS → Voltage (gain = {ARC_LENGTH_VOLTAGE_GAIN} V/unit)')
ax_v.legend()
ax_v.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('/workspace/ros2_packages/src/robin_simulation/synergy_plot.png', dpi=150)
plt.show()