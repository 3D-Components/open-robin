from dataclasses import dataclass
from typing import Iterable, Optional


NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True)
class ProgressionSample:
    stamp_ns: int
    bead_id: str
    progression: float


@dataclass(frozen=True)
class PendingGeometrySample:
    stamp_ns: int
    height: float
    width: float
    cross_sectional_area: float


@dataclass(frozen=True)
class GeometrySample:
    stamp_ns: int
    bead_id: str
    progression: float
    height: float
    width: float
    cross_sectional_area: float


@dataclass(frozen=True)
class FroniusTelemetrySample:
    stamp_ns: int
    bead_id: str
    progression: float
    current: float
    voltage: float
    speed: float


@dataclass(frozen=True)
class AlignedTelemetrySample:
    bead_id: str
    progression: float
    height: float
    width: float
    cross_sectional_area: float
    current: float
    voltage: float
    speed: float
    geometry_stamp_ns: int
    fronius_stamp_ns: int


def clamp_progression(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def time_msg_to_ns(time_msg) -> int:
    return int(time_msg.sec) * NANOSECONDS_PER_SECOND + int(time_msg.nanosec)


def seconds_to_ns(seconds: float) -> int:
    return int(seconds * NANOSECONDS_PER_SECOND)


def find_nearest_progression(
    progression_samples: Iterable[ProgressionSample],
    stamp_ns: int,
    max_delta_ns: int,
) -> Optional[ProgressionSample]:
    best_match: Optional[ProgressionSample] = None
    best_delta_ns: Optional[int] = None

    for sample in progression_samples:
        delta_ns = abs(sample.stamp_ns - stamp_ns)
        if best_delta_ns is None or delta_ns < best_delta_ns:
            best_match = sample
            best_delta_ns = delta_ns

    if best_match is None or best_delta_ns is None or best_delta_ns > max_delta_ns:
        return None
    return best_match


def match_geometry_to_fronius(
    fronius_sample: FroniusTelemetrySample,
    geometry_samples: Iterable[GeometrySample],
    max_progression_delta: float,
    max_time_delta_ns: int,
) -> Optional[GeometrySample]:
    best_match: Optional[GeometrySample] = None
    best_score: Optional[tuple[float, int]] = None

    for geometry_sample in geometry_samples:
        if (
            fronius_sample.bead_id
            and geometry_sample.bead_id
            and fronius_sample.bead_id != geometry_sample.bead_id
        ):
            continue

        progression_delta = abs(
            geometry_sample.progression - fronius_sample.progression
        )
        if progression_delta > max_progression_delta:
            continue

        time_delta_ns = abs(geometry_sample.stamp_ns - fronius_sample.stamp_ns)
        if time_delta_ns > max_time_delta_ns:
            continue

        score = (progression_delta, time_delta_ns)
        if best_score is None or score < best_score:
            best_match = geometry_sample
            best_score = score

    return best_match


def match_fronius_to_geometry(
    geometry_sample: GeometrySample,
    fronius_samples: Iterable[FroniusTelemetrySample],
    max_progression_delta: float,
    max_time_delta_ns: int,
) -> Optional[FroniusTelemetrySample]:
    best_match: Optional[FroniusTelemetrySample] = None
    best_score: Optional[tuple[float, int]] = None

    for fronius_sample in fronius_samples:
        if (
            geometry_sample.bead_id
            and fronius_sample.bead_id
            and geometry_sample.bead_id != fronius_sample.bead_id
        ):
            continue

        progression_delta = abs(
            fronius_sample.progression - geometry_sample.progression
        )
        if progression_delta > max_progression_delta:
            continue

        time_delta_ns = abs(fronius_sample.stamp_ns - geometry_sample.stamp_ns)
        if time_delta_ns > max_time_delta_ns:
            continue

        score = (progression_delta, time_delta_ns)
        if best_score is None or score < best_score:
            best_match = fronius_sample
            best_score = score

    return best_match
