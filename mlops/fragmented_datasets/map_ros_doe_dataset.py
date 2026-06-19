#!/usr/bin/env python3
"""
Map ROS 2 MCAP weld recordings to DOE recipe IDs.

What this script does
---------------------
1. Reads one or more MCAP bags.
2. Reads the DOE JSON.
3. Detects attempt starts from /robin/data/active_bead.
4. Collapses duplicate consecutive active_bead start messages.
5. Maps each attempt to DOE using:
       (wire_feed_speed, arc_length_correction_mm)
   and then audits whether the commanded weld_speed matches the canonical DOE.
   The pair join stays robust even when a subset of runs carried the wrong
   travel speed.
6. Emits an attempt-level CSV you can use later to clean into a unified dataset.

Recommended next step after this script
---------------------------------------
Use the output attempt table as the "index" for later cleaning. Then, for each
attempt, aggregate the per-topic samples (progression, fronius, geometry) into
stable-window bead-level features and labels.

Requires:
    pip install rosbags pandas
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader


def round_recipe_cols(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    if 'wire_feed_speed' in df.columns:
        df['wire_feed_speed_r'] = df['wire_feed_speed'].round(2)
    if 'arc_length_correction_mm' in df.columns:
        df['arc_length_correction_mm_r'] = df['arc_length_correction_mm'].round(1)
    if 'weld_speed' in df.columns:
        df['weld_speed_r'] = df['weld_speed'].round(4)
    return df


def load_doe(doe_json: Path) -> pd.DataFrame:
    payload = json.loads(doe_json.read_text())
    doe = pd.DataFrame(payload['beads'])
    doe = round_recipe_cols(doe)
    if doe.duplicated(['wire_feed_speed_r', 'arc_length_correction_mm_r']).any():
        dupes = doe[doe.duplicated(['wire_feed_speed_r', 'arc_length_correction_mm_r'], keep=False)]
        raise ValueError(
            'DOE pair key is not unique. Refusing to continue.\n'
            f'{dupes.to_string(index=False)}'
        )
    return doe


def read_messages(mcap_files: Iterable[Path]) -> tuple[pd.DataFrame, pd.DataFrame]:
    rows = []
    schema_rows = []

    for fp in sorted(mcap_files):
        with AnyReader([fp]) as reader:
            for conn in reader.connections:
                if conn.topic == '/robin/weld_dimensions':
                    schema_rows.append({
                        'bag_file': fp.name,
                        'weld_dimensions_has_toe_angle': 'toe_angle_rad' in conn.msgdef.data,
                    })

            for conn, ts, raw in reader.messages():
                if not conn.topic.startswith('/robin'):
                    continue

                msg = reader.deserialize(raw, conn.msgtype)
                row = {
                    'bag_file': fp.name,
                    'topic': conn.topic,
                    'ts': int(ts),
                    'bead_id': getattr(msg, 'bead_id', None),
                }

                if hasattr(msg, 'progression'):
                    row['progression'] = float(msg.progression)

                if conn.topic == '/robin/data/active_bead':
                    row.update({
                        'weld_speed_cmd_ros': float(msg.weld_speed),
                        'wire_feed_speed_cmd_ros': float(msg.wire_feed_speed),
                        'arc_length_correction_cmd_ros': float(msg.arc_length_correction_mm),
                        'current_recomm_ros': float(msg.current_recommvalue),
                        'voltage_recomm_ros': float(msg.voltage_recommvalue),
                        'total_length_m': float(msg.total_length),
                    })
                elif conn.topic == '/robin/data/fronius':
                    row.update({
                        'current_A': float(msg.current),
                        'voltage_V': float(msg.voltage),
                        'wire_feed_speed_meas': float(msg.wire_feed_speed),
                        'power_W': float(msg.power),
                        'energy_J': float(msg.energy),
                    })
                elif conn.topic == '/robin/weld_dimensions':
                    row.update({
                        'height_mm': float(msg.height_mm),
                        'width_mm': float(msg.width_mm),
                        'has_toe_angle': hasattr(msg, 'toe_angle_rad'),
                    })
                    if hasattr(msg, 'toe_angle_rad'):
                        row['toe_angle_rad'] = float(msg.toe_angle_rad)
                elif conn.topic == '/robin/data/progression':
                    row['is_welding'] = bool(msg.is_welding)
                elif conn.topic == '/robin/data/is_welding':
                    row['is_welding_bool'] = bool(msg.data)

                rows.append(row)

    return pd.DataFrame(rows), pd.DataFrame(schema_rows).drop_duplicates()


def detect_attempts(msgs: pd.DataFrame, doe: pd.DataFrame) -> pd.DataFrame:
    active = msgs[msgs.topic == '/robin/data/active_bead'].sort_values(['bag_file', 'ts']).copy()
    active['is_zero_recipe'] = (
        active['arc_length_correction_cmd_ros'].fillna(0).abs() < 1e-9
    ) & (
        active['current_recomm_ros'].fillna(0).abs() < 1e-9
    ) & (
        active['voltage_recomm_ros'].fillna(0).abs() < 1e-9
    )

    active['recipe_tuple'] = active.apply(
        lambda r: None if r.is_zero_recipe else (
            round(r.wire_feed_speed_cmd_ros, 2),
            round(r.weld_speed_cmd_ros, 4),
            round(r.arc_length_correction_cmd_ros, 1),
            round(r.current_recomm_ros, 3),
            r.bead_id,
        ),
        axis=1,
    )

    attempts = []
    for bag, g in active.groupby('bag_file'):
        g = g.sort_values('ts').reset_index(drop=True)

        start_idxs = []
        prev_nonzero = None
        for i, row in g.iterrows():
            if row['is_zero_recipe']:
                prev_nonzero = None
                continue
            if prev_nonzero != row['recipe_tuple']:
                start_idxs.append(i)
            prev_nonzero = row['recipe_tuple']

        for j, i in enumerate(start_idxs):
            row = g.loc[i]
            next_start_ts = int(g.loc[start_idxs[j + 1], 'ts']) if j + 1 < len(start_idxs) else int(msgs[msgs.bag_file == bag].ts.max()) + 1

            dup_count = 1
            k = i + 1
            while k < len(g):
                nxt = g.loc[k]
                if nxt['is_zero_recipe']:
                    break
                if nxt['recipe_tuple'] == row['recipe_tuple']:
                    dup_count += 1
                    k += 1
                else:
                    break

            attempts.append({
                'bag_file': bag,
                'start_ts': int(row.ts),
                'next_start_ts': next_start_ts,
                'start_bead_id': row.bead_id,
                'wire_feed_speed_cmd_ros': row.wire_feed_speed_cmd_ros,
                'weld_speed_cmd_ros': row.weld_speed_cmd_ros,
                'arc_length_correction_cmd_ros': row.arc_length_correction_cmd_ros,
                'current_recomm_ros': row.current_recomm_ros,
                'voltage_recomm_ros': row.voltage_recomm_ros,
                'total_length_m': row.total_length_m,
                'active_start_msgs': dup_count,
            })

    attempts = pd.DataFrame(attempts).sort_values(['bag_file', 'start_ts']).reset_index(drop=True)
    attempts['attempt_idx_in_bag'] = attempts.groupby('bag_file').cumcount() + 1
    attempts['attempt_id'] = attempts['bag_file'] + '::A' + attempts['attempt_idx_in_bag'].astype(str).str.zfill(3)
    attempts['wire_feed_speed_r'] = attempts['wire_feed_speed_cmd_ros'].round(2)
    attempts['arc_length_correction_mm_r'] = attempts['arc_length_correction_cmd_ros'].round(1)
    attempts['weld_speed_r_ros'] = attempts['weld_speed_cmd_ros'].round(4)

    attempts = attempts.merge(
        doe[[
            'input_id',
            'wire_feed_speed',
            'weld_speed',
            'arc_length_correction_mm',
            'wire_feed_speed_r',
            'arc_length_correction_mm_r',
            'weld_speed_r',
        ]],
        on=['wire_feed_speed_r', 'arc_length_correction_mm_r'],
        how='left',
        validate='m:1',
        suffixes=('', '_doe'),
    )

    attempts.rename(columns={
        'wire_feed_speed': 'doe_wire_feed_speed',
        'weld_speed': 'doe_weld_speed',
        'arc_length_correction_mm': 'doe_arc_length_correction_mm',
        'weld_speed_r': 'weld_speed_r_doe',
    }, inplace=True)

    attempts['weld_speed_match'] = np.where(
        attempts['weld_speed_r_ros'] == attempts['weld_speed_r_doe'],
        'exact',
        'mismatch',
    )
    attempts['weld_speed_ratio_ros_to_doe'] = attempts['weld_speed_cmd_ros'] / attempts['doe_weld_speed']

    # Same-bead boundary: use next start of the same bead_id in the same bag.
    # This is safer than raw bag-wide intervals when bead_id gets reused.
    attempts['next_same_bead_start_ts'] = (
        attempts.groupby(['bag_file', 'start_bead_id'])['start_ts']
        .shift(-1)
        .fillna(np.inf)
    )

    stats = []
    for row in attempts.itertuples(index=False):
        m = msgs[
            (msgs.bag_file == row.bag_file) &
            (msgs.bead_id == row.start_bead_id) &
            (msgs.ts >= row.start_ts) &
            (msgs.ts < row.next_same_bead_start_ts)
        ]

        stats.append({
            'attempt_id': row.attempt_id,
            'progression_msgs_same_bead': int((m.topic == '/robin/data/progression').sum()),
            'fronius_msgs_same_bead': int((m.topic == '/robin/data/fronius').sum()),
            'geometry_msgs_same_bead': int((m.topic == '/robin/weld_dimensions').sum()),
        })

    stats = pd.DataFrame(stats)
    attempts = attempts.merge(stats, on='attempt_id', how='left')

    attempts['geometry_anomaly_flag'] = attempts['geometry_msgs_same_bead'] > 500
    attempts['progression_anomaly_flag'] = attempts['progression_msgs_same_bead'] > 800
    attempts['usable_for_training_now'] = (
        attempts['input_id'].notna() &
        (attempts['geometry_msgs_same_bead'] > 0) &
        (attempts['fronius_msgs_same_bead'] > 0)
    )

    attempts['notes'] = ''
    attempts.loc[attempts['weld_speed_match'] == 'mismatch', 'notes'] += 'canonical DOE/ROS weld_speed mismatch; '
    attempts.loc[attempts['geometry_msgs_same_bead'] == 0, 'notes'] += 'missing geometry; '
    attempts.loc[attempts['fronius_msgs_same_bead'] == 0, 'notes'] += 'missing fronius; '
    attempts.loc[attempts['progression_anomaly_flag'], 'notes'] += 'high progression count; '
    attempts.loc[attempts['geometry_anomaly_flag'], 'notes'] += 'high geometry count; '
    attempts['notes'] = attempts['notes'].str.strip()

    return attempts


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--data-dir', type=Path, required=True, help='Directory containing MCAP files.')
    parser.add_argument('--doe-json', type=Path, required=True, help='DOE JSON file.')
    parser.add_argument('--outdir', type=Path, required=True, help='Where CSV outputs should be written.')
    args = parser.parse_args()

    args.outdir.mkdir(parents=True, exist_ok=True)

    mcap_files = sorted(args.data_dir.rglob('*.mcap'))
    if not mcap_files:
        raise FileNotFoundError(f'No .mcap files found in {args.data_dir}')

    doe = load_doe(args.doe_json)
    msgs, schema_df = read_messages(mcap_files)
    attempts = detect_attempts(msgs, doe)

    attempts.to_csv(args.outdir / 'ros_doe_mapping_summary.csv', index=False)

    topic_counts = msgs.groupby(['bag_file', 'topic']).size().unstack(fill_value=0).reset_index()
    topic_counts.columns = [
        c.replace('/robin/', '').replace('/', '_') if isinstance(c, str) else c
        for c in topic_counts.columns
    ]
    bag_inventory = topic_counts.merge(schema_df, on='bag_file', how='left')
    bag_inventory.to_csv(args.outdir / 'ros_bag_inventory.csv', index=False)

    print(f'Wrote {args.outdir / "ros_doe_mapping_summary.csv"}')
    print(f'Wrote {args.outdir / "ros_bag_inventory.csv"}')


if __name__ == '__main__':
    main()
