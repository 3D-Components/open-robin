#!/usr/bin/env python3
"""
TimescaleDB plot sanity check.

Reads `urn:robin:processTelemetry` directly from TimescaleDB to verify what the
DDS -> Orion -> TROE pipeline persisted.
"""

import sys

import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import psycopg2


DB_CONFIG = {
    'host': '127.0.0.1',
    'port': 5433,
    'database': 'orion',
    'user': 'orion',
    'password': 'orionpass',
}


def fetch_data(process_id: str):
    conn = psycopg2.connect(**DB_CONFIG)
    cur = conn.cursor()

    entity_id = f'urn:ngsi-ld:Process:{process_id}'
    cur.execute(
        """
        SELECT observedat, compound
        FROM attributes
        WHERE entityid = %s AND id = 'urn:robin:processTelemetry'
        ORDER BY observedat
        """,
        (entity_id,),
    )
    rows = cur.fetchall()
    conn.close()

    telemetry = {
        'timestamps': [],
        'current': [],
        'voltage': [],
        'speed': [],
        'width': [],
        'height': [],
    }

    for ts, compound in rows:
        if not compound:
            continue
        telemetry['timestamps'].append(ts)
        telemetry['current'].append(compound.get('current', 0))
        telemetry['voltage'].append(compound.get('voltage', 0))
        telemetry['speed'].append(compound.get('speed', 0))
        telemetry['width'].append(compound.get('width', 0))
        telemetry['height'].append(compound.get('height', 0))

    return telemetry


def plot_data(telemetry, output_path: str):
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle('TimescaleDB Data - processTelemetry', fontsize=14, fontweight='bold')

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


def print_stats(telemetry):
    print('\n' + '=' * 60)
    print('TIMESCALEDB DATA STATISTICS')
    print('=' * 60)

    for key, label in (
        ('current', 'Current (A)'),
        ('voltage', 'Voltage (V)'),
        ('speed', 'Speed (mm/s)'),
        ('width', 'Width (mm)'),
        ('height', 'Height (mm)'),
    ):
        values = telemetry[key]
        if not values:
            continue
        print(f'\n{label}:')
        print(f'  Total samples: {len(values)}')
        print(f'  Min: {min(values):.2f}, Max: {max(values):.2f}')
        print(f'  Mean: {sum(values)/len(values):.2f}')

    if telemetry['timestamps']:
        start = min(telemetry['timestamps'])
        end = max(telemetry['timestamps'])
        duration = (end - start).total_seconds()
        print('\nTime Range:')
        print(f'  Start: {start}')
        print(f'  End: {end}')
        print(f'  Duration: {duration:.1f} seconds')

    print('=' * 60)


def main():
    process_id = sys.argv[1] if len(sys.argv) > 1 else 'ros_bridge'

    print('=' * 60)
    print('TIMESCALEDB PLOT SANITY CHECK')
    print('=' * 60)
    print(f'Process ID: {process_id}')

    telemetry = fetch_data(process_id)
    if not telemetry['timestamps']:
        print('No data found.')
        return 1

    print_stats(telemetry)
    plot_data(telemetry, '/tmp/timescale_plot_sanity.png')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
