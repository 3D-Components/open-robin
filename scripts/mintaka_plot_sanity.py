#!/usr/bin/env python3
"""
Mintaka plot sanity check.

Queries temporal data for `urn:robin:processTelemetry` and plots the main
signals to validate the end-to-end storage pipeline.
"""

import os
import sys
from datetime import datetime

import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import requests


MINTAKA_URL = os.getenv('MINTAKA_URL', 'http://host.docker.internal:9090')
ENTITY_ID_TEMPLATE = 'urn:ngsi-ld:Process:{process_id}'


def fetch_temporal_data(process_id: str):
    entity_id = ENTITY_ID_TEMPLATE.format(process_id=process_id)
    params = {
        'attrs': 'urn:robin:processTelemetry',
        'timerel': 'between',
        'timeAt': '1970-01-01T00:00:00Z',
        'endTimeAt': '2030-01-01T00:00:00Z',
        'options': 'temporalValues',
        'lastN': '50000',
    }
    url = f'{MINTAKA_URL}/temporal/entities/{entity_id}'
    print(f'Querying: {url}')
    response = requests.get(url, params=params, timeout=30)
    print(f'Response status: {response.status_code}')
    if response.status_code not in (200, 206):
        print(f'Error: {response.text[:500]}')
        return None
    return response.json()


def parse_temporal_data(data: dict):
    telemetry = {
        'timestamps': [],
        'current': [],
        'voltage': [],
        'speed': [],
        'width': [],
        'height': [],
    }

    attr = data.get('urn:robin:processTelemetry') or data.get(
        'processTelemetry', {}
    )
    values = attr.get('values', []) if isinstance(attr, dict) else attr
    if not isinstance(values, list):
        values = []

    print(f'processTelemetry entries: {len(values)}')
    for entry in values:
        if not (isinstance(entry, list) and len(entry) >= 2):
            continue
        value, timestamp = entry[0], entry[1]
        if not isinstance(value, dict):
            continue
        try:
            ts = datetime.fromisoformat(str(timestamp).replace('Z', '+00:00'))
        except Exception:
            continue

        telemetry['timestamps'].append(ts)
        telemetry['current'].append(value.get('current', 0))
        telemetry['voltage'].append(value.get('voltage', 0))
        telemetry['speed'].append(value.get('speed', 0))
        telemetry['width'].append(value.get('width', 0))
        telemetry['height'].append(value.get('height', 0))

    return telemetry


def plot_data(telemetry, output_path: str):
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle('Mintaka Temporal Data - processTelemetry', fontsize=14, fontweight='bold')

    axes[0].plot(telemetry['timestamps'], telemetry['current'], 'b-', linewidth=1.5, alpha=0.8)
    axes[0].set_ylabel('Current (A)')
    axes[0].set_title(f"Current ({len(telemetry['current'])} samples)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(telemetry['timestamps'], telemetry['voltage'], color='orange', linewidth=1.5, alpha=0.8)
    axes[1].set_ylabel('Voltage (V)')
    axes[1].set_title(f"Voltage ({len(telemetry['voltage'])} samples)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(telemetry['timestamps'], telemetry['width'], color='purple', linewidth=1.5, alpha=0.8)
    axes[2].set_ylabel('Width (mm)')
    axes[2].set_title(f"Width ({len(telemetry['width'])} samples)")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(telemetry['timestamps'], telemetry['height'], 'r-', linewidth=1.5, alpha=0.8)
    axes[3].set_ylabel('Height (mm)')
    axes[3].set_title(f"Height ({len(telemetry['height'])} samples)")
    axes[3].set_xlabel('Time')
    axes[3].grid(True, alpha=0.3)

    for ax in axes:
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f'Plot saved to: {output_path}')


def main():
    process_id = sys.argv[1] if len(sys.argv) > 1 else 'ros_bridge'
    print('=' * 60)
    print('MINTAKA PLOT SANITY CHECK')
    print('=' * 60)
    print(f'Process ID: {process_id}')
    print(f'Mintaka URL: {MINTAKA_URL}')

    data = fetch_temporal_data(process_id)
    if data is None:
        return 1

    telemetry = parse_temporal_data(data)
    if not telemetry['timestamps']:
        print('No telemetry samples parsed.')
        return 1

    plot_data(telemetry, '/tmp/mintaka_plot_sanity.png')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
