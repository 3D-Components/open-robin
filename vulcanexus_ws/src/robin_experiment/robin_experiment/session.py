"""In-memory experiment session state."""

import time
from dataclasses import dataclass, field

from robin_interfaces.msg import PlateLayout
from robin_experiment.bead_layout import PhysicalBead


@dataclass
class PlannedSession:
    """In-memory experiment planning session."""

    plan_id: str
    experiment_id: str
    beads: list
    plates: list
    layout: PlateLayout
    physical_beads: list[PhysicalBead]
    bead_runtime: dict[str, dict] = field(default_factory=dict)
    approved: bool = False
    created_at: float = field(default_factory=time.time)
