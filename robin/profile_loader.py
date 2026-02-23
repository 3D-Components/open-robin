"""Load a ROBIN profile YAML and expose it as typed configuration."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

import yaml


_DEFAULT_VOCABULARY = {
    'process': 'Process',
    'processPlural': 'Processes',
    'segment': 'Segment',
    'segmentPlural': 'Segments',
    'geometry': 'Geometry',
    'profileHeight': 'Profile Height',
    'profileWidth': 'Profile Width',
    'depositionView': 'Deposition',
    'speed': 'Speed',
    'speedUnit': 'mm/s',
    'current': 'Current',
    'currentUnit': 'A',
    'voltage': 'Voltage',
    'voltageUnit': 'V',
    'toolPath': 'Tool path',
    'workpiece': 'Workpiece',
}

_DEFAULT_AI = {
    'feature_order': ['wireSpeed', 'current', 'voltage'],
    'default_tolerance': 10.0,
    'default_mode': 'parameter_driven',
}


class Profile:
    """Parsed profile configuration."""

    def __init__(self, data: Dict[str, Any]) -> None:
        self._data = data
        meta = data.get('profile', {})
        self.name: str = meta.get('name', 'default')
        self.description: str = meta.get('description', '')

        self.vocabulary: Dict[str, str] = {
            **_DEFAULT_VOCABULARY,
            **data.get('vocabulary', {}),
        }
        self.fields: Dict[str, Dict[str, str]] = data.get('fields', {})
        self.ros2: Dict[str, Any] = data.get('ros2', {})
        self.skills: Dict[str, Any] = data.get('skills', {})

        ai_raw = data.get('ai', {})
        self.feature_order: Sequence[str] = ai_raw.get(
            'feature_order', _DEFAULT_AI['feature_order'],
        )
        self.default_tolerance: float = ai_raw.get(
            'default_tolerance', _DEFAULT_AI['default_tolerance'],
        )
        self.default_mode: str = ai_raw.get(
            'default_mode', _DEFAULT_AI['default_mode'],
        )
        self.model_path: Optional[str] = ai_raw.get('model_path')

        self.dds: Dict[str, str] = data.get('dds', {})

    def as_dict(self) -> Dict[str, Any]:
        """Return the full profile as a JSON-serialisable dict."""
        return self._data


def _resolve_profile_path() -> Optional[Path]:
    profile_name = os.getenv('ROBIN_PROFILE', 'welding')

    explicit = os.getenv('ROBIN_PROFILE_PATH')
    if explicit:
        p = Path(explicit)
        if p.is_file():
            return p

    search_dirs = [
        Path('/app/config/profiles'),
        Path(__file__).resolve().parents[1] / 'config' / 'profiles',
    ]
    for d in search_dirs:
        candidate = d / f'{profile_name}.yaml'
        if candidate.is_file():
            return candidate

    return None


def load_profile() -> Profile:
    """Load the active profile from YAML.

    Resolution order:
    1. ``ROBIN_PROFILE_PATH`` env var (exact file path)
    2. ``config/profiles/{ROBIN_PROFILE}.yaml`` searched in known directories
    3. Built-in defaults (no YAML file needed)
    """
    path = _resolve_profile_path()
    if path is not None:
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        return Profile(data)

    return Profile({})
