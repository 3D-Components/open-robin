#!/usr/bin/env python3
"""
Minimal dashboard simulator: generates process measurements and posts them to Orion,
so the minimal dashboard at http://localhost:8001/dashboard can display them.

This runs from the host and talks to the containerized stack via localhost ports.

Usage examples:
  python demo/dashboard-simulation.py --pid demo-quick --duration 60 --interval 0.5
  python demo/dashboard-simulation.py --pid dash-$(date +%s)
"""

import argparse
import math
import random
import time
from datetime import datetime, timezone
from typing import Dict, Tuple

import requests


ORION_URL_DEFAULT = 'http://localhost:1026'  # host-mapped port
TENANT = 'robin'
ALERT_URL_DEFAULT = 'http://localhost:8001'


CORE_CONTEXT = 'https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context.jsonld'
EMBEDDED_CONTEXT = [
    CORE_CONTEXT,
    {
        'Process': 'urn:robin:Process',
        'Measurement': 'urn:robin:Measurement',
        'GeometryTarget': 'urn:robin:GeometryTarget',
    },
]


def ensure_process(alert_url: str, process_id: str) -> None:
    """Create the Process via Alert Engine if missing (idempotent)."""
    try:
        resp = requests.post(
            f'{alert_url.rstrip('/')}/create-process',
            json={'process_id': process_id, 'mode': 'parameter_driven'},
            timeout=5,
        )
        # Accept success or warning; if it fails, it may already exist.
        if resp.ok:
            return
    except Exception:
        pass

    # Check directly on Orion
    eid = f'urn:ngsi-ld:Process:{process_id}'
    r = requests.get(
        f'{ORION_URL_DEFAULT}/ngsi-ld/v1/entities/{eid}',
        headers={'NGSILD-Tenant': TENANT},
        timeout=5,
    )
    if r.status_code == 200:
        return
    # Otherwise attempt creation directly (minimal attributes)
    entity = {
        'id': eid,
        'type': 'Process',
        'operationMode': {'type': 'Property', 'value': 'parameter_driven'},
        'processStatus': {'type': 'Property', 'value': 'active'},
        'startedAt': {'type': 'Property', 'value': datetime.now(timezone.utc).isoformat()},
        '@context': EMBEDDED_CONTEXT,
    }
    requests.post(
        f'{ORION_URL_DEFAULT}/ngsi-ld/v1/entities',
        headers={'NGSILD-Tenant': TENANT, 'Content-Type': 'application/ld+json'},
        json=entity,
        timeout=5,
    )


def estimate_travel_speed(wire_speed_mm_s: float, current_A: float) -> float:
    travel_speed_base = 5.0
    travel_speed_current_gain = 0.02
    travel_speed_wfs_gain = 0.50
    wire_feed_rate_m_per_min = wire_speed_mm_s * 60.0 / 1000.0
    return (
        travel_speed_base
        + travel_speed_current_gain * current_A
        + travel_speed_wfs_gain * wire_feed_rate_m_per_min
    )


def compute_geometry(
    wire_speed_mm_s: float, travel_speed_mm_s: float, current_A: float, voltage_V: float
) -> Tuple[float, float]:
    # Simplified version of the physics-inspired mapping
    wire_diameter_mm = 1.2
    radius = wire_diameter_mm / 2.0
    wire_area = math.pi * radius * radius
    eta_dep = 0.85
    eta_arc = 0.8
    deposition_rate = eta_dep * wire_area * wire_speed_mm_s
    cross_section = deposition_rate / max(1e-6, travel_speed_mm_s)
    heat_input = (eta_arc * voltage_V * current_A) / max(1e-6, travel_speed_mm_s)

    width_coeff = 1.20
    width_exp = 0.35
    width_mm = max(0.5, width_coeff * (heat_input ** width_exp))
    height_mm = max(0.35, (4.0 * cross_section) / (math.pi * width_mm))
    return round(width_mm, 2), round(height_mm, 2)


def gen_sample(t: float, base: Dict[str, float]) -> Dict[str, float]:
    # Smooth oscillations + noise
    ws = base['wireSpeed'] + math.sin(t * 0.3) * 8.0 + (random.random() - 0.5) * 8.0
    cur = base['current'] + math.sin(t * 0.4) * 3.0 + (random.random() - 0.5) * 2.0
    vol = base['voltage'] + math.sin(t * 0.6) * 0.3 + (random.random() - 0.5) * 0.2
    ws = max(40.0, ws)
    cur = max(90.0, cur)
    vol = max(18.0, vol)
    ts = estimate_travel_speed(ws, cur)
    w, h = compute_geometry(ws, ts, cur, vol)
    return {
        'height': h,
        'width': w,
        'wireSpeed': round(ws, 2),
        'travelSpeed': round(ts, 2),
        'current': round(cur, 1),
        'voltage': round(vol, 1),
    }


def post_measurement(orion_url: str, process_id: str, m: Dict[str, float]) -> bool:
    ts = datetime.now(timezone.utc).isoformat()
    mid = f'measure-{int(time.time()*1000)}'
    entity = {
        'id': f'urn:ngsi-ld:Measurement:{mid}',
        'type': 'Measurement',
        'measuredHeight': {'type': 'Property', 'value': m['height'], 'unitCode': 'MMT', 'observedAt': ts},
        'measuredWidth': {'type': 'Property', 'value': m['width'], 'unitCode': 'MMT', 'observedAt': ts},
        'processId': {
            'type': 'Relationship',
            'object': f'urn:ngsi-ld:Process:{process_id}',
        },
        '@context': EMBEDDED_CONTEXT,
    }
    # Optional parameters for plotting
    entity['measuredSpeed'] = {'type': 'Property', 'value': m['wireSpeed'], 'unitCode': 'mm/s', 'observedAt': ts}
    entity['measuredCurrent'] = {'type': 'Property', 'value': m['current'], 'unitCode': 'A', 'observedAt': ts}
    entity['measuredVoltage'] = {'type': 'Property', 'value': m['voltage'], 'unitCode': 'V', 'observedAt': ts}

    try:
        r = requests.post(
            f'{orion_url}/ngsi-ld/v1/entities',
            headers={'NGSILD-Tenant': TENANT, 'Content-Type': 'application/ld+json'},
            json=entity,
            timeout=5,
        )
        ok = 200 <= r.status_code < 300
        if not ok:
            print(f'! Post failed [{r.status_code}]: {r.text[:240]}')
        return ok
    except Exception as e:
        print(f'! Post exception: {e}')
        return False


def main():
    ap = argparse.ArgumentParser(description='Stream measurements for the minimal dashboard')
    ap.add_argument('--pid', '--process-id', dest='pid', required=True, help='Process ID to publish to')
    ap.add_argument('-d', '--duration', type=float, default=60.0, help='Duration in seconds')
    ap.add_argument('-i', '--interval', type=float, default=0.5, help='Interval between samples (s)')
    ap.add_argument('--orion', default=ORION_URL_DEFAULT, help='Orion URL (host)')
    ap.add_argument('--alert', default=ALERT_URL_DEFAULT, help='Alert Engine URL (host)')
    ap.add_argument('--base-wire-speed', type=float, default=100.0, help='Base wire speed (mm/s)')
    ap.add_argument('--base-current', type=float, default=150.0, help='Base current (A)')
    ap.add_argument('--base-voltage', type=float, default=24.0, help='Base voltage (V)')
    args = ap.parse_args()

    ensure_process(args.alert, args.pid)

    base = {
        'wireSpeed': args.base_wire_speed,
        'current': args.base_current,
        'voltage': args.base_voltage,
    }

    print(f'→ Streaming to process {args.pid} for {args.duration}s @ {args.interval}s')
    print(f'  Orion: {args.orion} | Tenant: {TENANT}')
    print('  Open dashboard: http://localhost:8001/dashboard?pid=' + args.pid)

    t0 = time.time()
    sent = 0
    while True:
        now = time.time()
        if now - t0 >= args.duration:
            break
        sample = gen_sample(now - t0, base)
        ok = post_measurement(args.orion, args.pid, sample)
        sent += 1 if ok else 0
        time.sleep(max(0.01, args.interval))

    print(f'Done. Sent {sent} samples.')


if __name__ == '__main__':
    main()
