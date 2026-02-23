#!/usr/bin/env python3
"""
Bag Plot Sanity Check

Reads the ROS bag directly and plots voltage, current, and geometry values
independently of Orion/API to verify what the data should look like.

Usage:
    python scripts/bag_plot_sanity.py [bag_path]
"""

import sys
from pathlib import Path
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

# Try to import rosbag2_py - if not available, provide instructions
try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print('Error: ROS 2 packages not found.')
    print('Run this script inside the vulcanexus container:')
    print('  docker exec -it vulcanexus-bridge bash')
    print('  cd /workspace/ros2_packages && source ws_setup.sh')
    print(
        '  python3 /workspace/ros2_packages/src/robin_core_data/scripts/bag_plot_sanity.py'
    )
    sys.exit(1)


def read_bag(bag_path: str):
    """Read messages from a ROS bag file."""
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic types
    topic_types = {}
    for topic_info in reader.get_all_topics_and_types():
        topic_types[topic_info.name] = topic_info.type

    # Data containers
    fronius_data = {
        'timestamps': [],
        'current': [],
        'voltage': [],
        'wire_feed_speed': [],
        'power': [],
    }
    geometry_data = {'timestamps': [], 'width': [], 'height': []}

    print(f'Reading bag: {bag_path}')
    print(f'Topics found: {list(topic_types.keys())[:10]}...')

    msg_count = 0
    while reader.has_next():
        topic_name, data, timestamp_ns = reader.read_next()
        msg_count += 1

        if topic_name == '/robin/data/fronius':
            msg_type = get_message(topic_types[topic_name])
            msg = deserialize_message(data, msg_type)

            # Convert ROS timestamp to datetime
            ts = datetime.fromtimestamp(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            )

            fronius_data['timestamps'].append(ts)
            fronius_data['current'].append(msg.current)
            fronius_data['voltage'].append(msg.voltage)
            fronius_data['wire_feed_speed'].append(msg.wire_feed_speed)
            fronius_data['power'].append(msg.power)

        elif topic_name == '/robin/weld_dimensions':
            msg_type = get_message(topic_types[topic_name])
            msg = deserialize_message(data, msg_type)

            if len(msg.data) >= 2:
                width = msg.data[0]
                height = msg.data[1]

                # Use bag timestamp since Float32MultiArray has no header
                ts = datetime.fromtimestamp(timestamp_ns / 1e9)

                geometry_data['timestamps'].append(ts)
                geometry_data['width'].append(width)
                geometry_data['height'].append(height)

    print(f'Total messages read: {msg_count}')
    print(f"Fronius samples: {len(fronius_data['timestamps'])}")
    print(f"Geometry samples: {len(geometry_data['timestamps'])}")

    return fronius_data, geometry_data


def plot_data(fronius_data, geometry_data, output_path: str = None):
    """Create plots for the bag data."""
    fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True)
    fig.suptitle(
        'ROS Bag Data - Sanity Check (Direct from Bag)',
        fontsize=14,
        fontweight='bold',
    )

    # Plot Current
    ax = axes[0]
    ax.plot(
        fronius_data['timestamps'],
        fronius_data['current'],
        'b-',
        linewidth=1.5,
        alpha=0.8,
    )
    ax.set_ylabel('Current (A)')
    ax.set_title('Process Current')
    ax.grid(True, alpha=0.3)

    # Plot Voltage
    ax = axes[1]
    ax.plot(
        fronius_data['timestamps'],
        fronius_data['voltage'],
        'orange',
        linewidth=1.5,
        alpha=0.8,
    )
    ax.set_ylabel('Voltage (V)')
    ax.set_title('Process Voltage')
    ax.grid(True, alpha=0.3)

    # Plot Wire Feed Speed
    ax = axes[2]
    ax.plot(
        fronius_data['timestamps'],
        fronius_data['wire_feed_speed'],
        'g-',
        linewidth=1.5,
        alpha=0.8,
    )
    ax.set_ylabel('Speed (m/min)')
    ax.set_title('Wire Feed Speed')
    ax.grid(True, alpha=0.3)

    # Plot Profile Width
    ax = axes[3]
    # Filter out zeros for cleaner plot
    valid_idx = [i for i, w in enumerate(geometry_data['width']) if w > 0.1]
    valid_ts = [geometry_data['timestamps'][i] for i in valid_idx]
    valid_width = [geometry_data['width'][i] for i in valid_idx]
    ax.plot(valid_ts, valid_width, 'purple', linewidth=1.5, alpha=0.8)
    ax.set_ylabel('Width (mm)')
    ax.set_title('Profile Width')
    ax.grid(True, alpha=0.3)

    # Plot Profile Height
    ax = axes[4]
    valid_height = [geometry_data['height'][i] for i in valid_idx]
    ax.plot(valid_ts, valid_height, 'red', linewidth=1.5, alpha=0.8)
    ax.set_ylabel('Height (mm)')
    ax.set_title('Profile Height')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('Time')

    # Format x-axis
    for ax in axes:
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'Plot saved to: {output_path}')

    plt.show()


def print_stats(fronius_data, geometry_data):
    """Print statistics about the data."""
    print('\n' + '=' * 60)
    print('DATA STATISTICS')
    print('=' * 60)

    if fronius_data['current']:
        current_nonzero = [c for c in fronius_data['current'] if c > 0]
        print(f'\nCurrent (A):')
        print(f"  Total samples: {len(fronius_data['current'])}")
        print(f'  Non-zero samples: {len(current_nonzero)}')
        if current_nonzero:
            print(
                f'  Min: {min(current_nonzero):.2f}, Max: {max(current_nonzero):.2f}'
            )
            print(f'  Mean: {sum(current_nonzero)/len(current_nonzero):.2f}')

    if fronius_data['voltage']:
        voltage_nonzero = [v for v in fronius_data['voltage'] if v > 0]
        print(f'\nVoltage (V):')
        print(f"  Total samples: {len(fronius_data['voltage'])}")
        print(f'  Non-zero samples: {len(voltage_nonzero)}')
        if voltage_nonzero:
            print(
                f'  Min: {min(voltage_nonzero):.2f}, Max: {max(voltage_nonzero):.2f}'
            )
            print(f'  Mean: {sum(voltage_nonzero)/len(voltage_nonzero):.2f}')

    if geometry_data['width']:
        width_valid = [w for w in geometry_data['width'] if w > 0.1]
        print(f'\nProfile Width (mm):')
        print(f"  Total samples: {len(geometry_data['width'])}")
        print(f'  Valid samples (>0.1): {len(width_valid)}')
        if width_valid:
            print(
                f'  Min: {min(width_valid):.2f}, Max: {max(width_valid):.2f}'
            )
            print(f'  Mean: {sum(width_valid)/len(width_valid):.2f}')

    if geometry_data['height']:
        height_valid = [h for h in geometry_data['height'] if h > 0.1]
        print(f'\nProfile Height (mm):')
        print(f"  Total samples: {len(geometry_data['height'])}")
        print(f'  Valid samples (>0.1): {len(height_valid)}')
        if height_valid:
            print(
                f'  Min: {min(height_valid):.2f}, Max: {max(height_valid):.2f}'
            )
            print(f'  Mean: {sum(height_valid)/len(height_valid):.2f}')

    if fronius_data['timestamps']:
        print(f'\nTime Range:')
        print(f"  Start: {fronius_data['timestamps'][0]}")
        print(f"  End: {fronius_data['timestamps'][-1]}")
        duration = (
            fronius_data['timestamps'][-1] - fronius_data['timestamps'][0]
        ).total_seconds()
        print(f'  Duration: {duration:.1f} seconds')

    print('=' * 60)


def main():
    # Default bag path
    default_bag = '/workspace/ros2_packages/exp001_rosbag_real'

    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    else:
        bag_path = default_bag

    if not Path(bag_path).exists():
        print(f'Error: Bag not found at {bag_path}')
        print(f'Trying alternative path...')
        alt_path = 'Data_ROSBAGS/exp001_rosbag_real'
        if Path(alt_path).exists():
            bag_path = alt_path
        else:
            print(f'Error: Could not find bag file')
            sys.exit(1)

    print('=' * 60)
    print('BAG PLOT SANITY CHECK')
    print('=' * 60)

    fronius_data, geometry_data = read_bag(bag_path)
    print_stats(fronius_data, geometry_data)

    # Save plot
    output_path = '/tmp/bag_plot_sanity.png'
    plot_data(fronius_data, geometry_data, output_path)


if __name__ == '__main__':
    main()
