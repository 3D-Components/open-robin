#!/usr/bin/env python3
"""Robust welding profile demo for ROBIN.

This script is the canonical welding demo. It exercises both operational modes
with explicit AI-in-the-loop checks:

* parameter_driven: operator sets process parameters -> AI predicts geometry ->
  measured geometry is checked against AI prediction.
* geometry_driven: operator sets target geometry -> AI suggests parameters ->
  expected geometry from those parameters is checked against measurements.

Telemetry is streamed via CLI (`python -m robin add-measurement`) so dashboard
KPIs/charts update in real time, while `/check-deviation` is called per sample to
trigger real alert entities when deviations exceed tolerance.
"""

from __future__ import annotations

import argparse
import math
import random
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import requests

CONTAINER_NAME = 'robin-alert-processor'
ALERT_ENGINE_URL = 'http://localhost:8001'
DASHBOARD_URL = 'http://localhost:5174'


@dataclass
class WeldSample:
    bead_height: float
    bead_width: float
    wire_speed: float
    current: float
    voltage: float


class DemoError(RuntimeError):
    """Raised when a required shell command fails."""


def run_command(cmd: List[str], check: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(cmd, capture_output=True, text=True)
    if check and result.returncode != 0:
        command_text = ' '.join(cmd)
        message = (
            f'Command failed: {command_text}\n'
            f'stdout: {result.stdout.strip()}\n'
            f'stderr: {result.stderr.strip()}'
        )
        raise DemoError(message)
    return result


def run_robin(args: List[str], check: bool = True) -> subprocess.CompletedProcess[str]:
    cmd = ['docker', 'exec', CONTAINER_NAME, 'python', '-m', 'robin', *args]
    return run_command(cmd, check=check)


def ensure_stack_running() -> None:
    run_command(['docker', '--version'])
    result = run_command(['docker', 'ps', '--format', '{{.Names}}'])
    names = {line.strip() for line in result.stdout.splitlines() if line.strip()}
    if CONTAINER_NAME not in names:
        print('Container not running. Starting docker compose stack...')
        run_command(['docker', 'compose', 'up', '-d'])
        time.sleep(5)


def maybe_create_process(process_id: str, mode: str) -> bool:
    result = run_robin(['create-process', process_id, '--mode', mode], check=False)
    if result.returncode == 0:
        print(f'Created process {process_id} ({mode}).')
        return True
    if 'already exists' in (result.stdout + result.stderr).lower():
        print(f'Process {process_id} already exists. Reusing it.')
        return True
    print('Warning: could not create process via CLI. Continuing anyway.')
    print(f'  stdout: {result.stdout.strip()}')
    print(f'  stderr: {result.stderr.strip()}')
    return False


def create_geometry_target(process_id: str, height: float, width: float) -> bool:
    result = run_robin(['create-target', process_id, str(height), str(width)], check=False)
    if result.returncode == 0:
        return True
    print('Warning: could not set geometry target. Continuing.')
    return False


def set_process_mode(process_id: str, mode: str, api_url: str) -> bool:
    try:
        response = requests.post(
            f'{api_url.rstrip('/')}/process/{process_id}/mode',
            json={'mode': mode},
            timeout=8,
        )
        return response.status_code == 200 and response.json().get('status') == 'success'
    except Exception:
        return False


def get_ai_recommendation(
    process_id: str,
    mode: str,
    api_url: str,
    input_params: Dict[str, float] | None = None,
    target_geometry: Dict[str, float] | None = None,
) -> Dict[str, Any] | None:
    payload: Dict[str, Any] = {'process_id': process_id, 'mode': mode}
    if input_params:
        payload['input_params'] = input_params
    if target_geometry:
        payload['target_geometry'] = target_geometry
    try:
        response = requests.post(
            f'{api_url.rstrip('/')}/ai-recommendation',
            json=payload,
            timeout=10,
        )
        if response.status_code != 200:
            return None
        body = response.json()
        if body.get('status') != 'success':
            return None
        recommendation = body.get('recommendation')
        return recommendation if isinstance(recommendation, dict) else None
    except Exception:
        return None


def predict_geometry(input_params: Dict[str, float], api_url: str) -> Dict[str, float] | None:
    payload = {
        'wireSpeed': float(input_params.get('wireSpeed', 0.0)),
        'current': float(input_params.get('current', 0.0)),
        'voltage': float(input_params.get('voltage', 0.0)),
    }
    try:
        response = requests.post(
            f'{api_url.rstrip('/')}/ai/models/predict',
            json=payload,
            timeout=10,
        )
        if response.status_code != 200:
            return None
        body = response.json()
        prediction = body.get('prediction')
        if not isinstance(prediction, dict):
            return None
        height = prediction.get('height')
        width = prediction.get('width')
        if not isinstance(height, (int, float)) or not isinstance(width, (int, float)):
            return None
        return {'height': float(height), 'width': float(width)}
    except Exception:
        return None


def check_deviation(
    process_id: str,
    mode: str,
    tolerance: float,
    measured_geometry: Dict[str, float],
    input_params: Dict[str, float],
    api_url: str,
) -> Dict[str, Any] | None:
    payload = {
        'process_id': process_id,
        'mode': mode,
        'tolerance': tolerance,
        'measured_geometry': measured_geometry,
        'input_params': input_params,
    }
    try:
        response = requests.post(
            f'{api_url.rstrip('/')}/check-deviation',
            json=payload,
            timeout=10,
        )
        if response.status_code != 200:
            return None
        body = response.json()
        return body if isinstance(body, dict) else None
    except Exception:
        return None


def extract_number(data: Dict[str, Any], keys: Sequence[str], fallback: float) -> float:
    for key in keys:
        value = data.get(key)
        if isinstance(value, (int, float)):
            return float(value)
    return float(fallback)


def build_deviation_windows(duration: int) -> List[Tuple[float, float]]:
    if duration <= 20:
        return [(max(4.0, duration * 0.4), min(float(duration), max(6.0, duration * 0.4 + 5.0)))]

    w1_start = max(10.0, duration * 0.25)
    w2_start = max(w1_start + 12.0, duration * 0.65)
    return [
        (w1_start, min(float(duration), w1_start + 8.0)),
        (w2_start, min(float(duration), w2_start + 8.0)),
    ]


def in_any_window(t: float, windows: Iterable[Tuple[float, float]]) -> bool:
    return any(start <= t < end for start, end in windows)


def generate_sample(
    t: float,
    expected_geometry: Dict[str, float],
    reference_params: Dict[str, float],
    deviation_windows: Sequence[Tuple[float, float]],
) -> WeldSample:
    height = expected_geometry['height'] + 0.20 * math.sin(t * 0.27) + random.uniform(-0.04, 0.04)
    width = expected_geometry['width'] + 0.18 * math.sin(t * 0.33) + random.uniform(-0.05, 0.05)

    wire_speed = reference_params['wireSpeed'] + 0.25 * math.sin(t * 0.21) + random.uniform(-0.08, 0.08)
    current = reference_params['current'] + 1.8 * math.sin(t * 0.39) + random.uniform(-0.5, 0.5)
    voltage = reference_params['voltage'] + 0.22 * math.sin(t * 0.18) + random.uniform(-0.07, 0.07)

    if in_any_window(t, deviation_windows):
        height *= 0.65
        width *= 1.35
        current *= 0.88

    return WeldSample(
        bead_height=max(0.5, round(height, 3)),
        bead_width=max(0.5, round(width, 3)),
        wire_speed=max(1.0, round(wire_speed, 3)),
        current=max(40.0, round(current, 3)),
        voltage=max(5.0, round(voltage, 3)),
    )


def stream_measurements(
    process_id: str,
    mode: str,
    duration: int,
    interval: float,
    tolerance: float,
    expected_geometry: Dict[str, float],
    reference_params: Dict[str, float],
    api_url: str,
) -> Tuple[int, int, int, int]:
    windows = build_deviation_windows(duration)
    print(
        f'Streaming welding telemetry for {process_id} ({mode}) for {duration}s '
        f'at {interval:.2f}s interval.'
    )
    print(
        'Reference expectation: '
        f"{expected_geometry['height']:.3f} mm x {expected_geometry['width']:.3f} mm"
    )
    print(
        'Reference parameters: '
        f"wireSpeed={reference_params['wireSpeed']:.3f}, "
        f"current={reference_params['current']:.3f}, "
        f"voltage={reference_params['voltage']:.3f}"
    )
    print(f'Deviation windows (s): {[(round(s, 1), round(e, 1)) for s, e in windows]}')
    print()

    start = time.time()
    count = 0
    ok_count = 0
    fail_count = 0
    alert_count = 0

    while True:
        elapsed = time.time() - start
        if elapsed >= duration:
            break

        sample = generate_sample(elapsed, expected_geometry, reference_params, windows)
        measurement_id = f'weld-{int(time.time() * 1000)}-{count}'

        result = run_robin(
            [
                'add-measurement',
                process_id,
                measurement_id,
                str(sample.bead_height),
                str(sample.bead_width),
                '--speed',
                str(sample.wire_speed),
                '--current',
                str(sample.current),
                '--voltage',
                str(sample.voltage),
            ],
            check=False,
        )

        count += 1
        if result.returncode == 0:
            ok_count += 1
        else:
            fail_count += 1

        check = check_deviation(
            process_id=process_id,
            mode=mode,
            tolerance=tolerance,
            measured_geometry={'height': sample.bead_height, 'width': sample.bead_width},
            input_params=reference_params,
            api_url=api_url,
        )

        deviation_pct = 0.0
        status = 'check_failed'
        source = 'n/a'
        if check:
            source = str(check.get('expected_source', 'unknown'))
            if 'deviation_percentage' in check and isinstance(check['deviation_percentage'], (int, float)):
                deviation_pct = float(check['deviation_percentage'])
            status = str(check.get('status', 'alert'))

        alert_flag = deviation_pct > tolerance
        if alert_flag:
            alert_count += 1

        marker = ' DEVIATION_WINDOW' if in_any_window(elapsed, windows) else ''
        print(
            f'[{count:03d}] '
            f'h={sample.bead_height:>6.3f}mm '
            f'w={sample.bead_width:>6.3f}mm '
            f'ws={sample.wire_speed:>6.3f} '
            f'i={sample.current:>7.3f} '
            f'v={sample.voltage:>6.3f} '
            f'dev={deviation_pct:>6.2f}% '
            f'status={status:<8} '
            f'src={source:<26} '
            f"alert={'YES' if alert_flag else 'no '}"
            f'{marker}'
        )

        time.sleep(interval)

    return ok_count, fail_count, count, alert_count


def resolve_parameter_mode_expectation(
    process_id: str,
    input_params: Dict[str, float],
    api_url: str,
) -> Tuple[Dict[str, float], str]:
    recommendation = get_ai_recommendation(
        process_id=process_id,
        mode='parameter_driven',
        input_params=input_params,
        api_url=api_url,
    )
    predicted = recommendation.get('predicted_geometry') if recommendation else None
    if isinstance(predicted, dict):
        height = predicted.get('height')
        width = predicted.get('width')
        if isinstance(height, (int, float)) and isinstance(width, (int, float)):
            return {'height': float(height), 'width': float(width)}, 'ai-recommendation'

    fallback = predict_geometry(input_params, api_url)
    if fallback:
        return fallback, 'ai-model-predict'

    return {'height': 5.2, 'width': 3.8}, 'fallback-default'


def resolve_geometry_mode_plan(
    process_id: str,
    target_geometry: Dict[str, float],
    fallback_params: Dict[str, float],
    api_url: str,
) -> Tuple[Dict[str, float], Dict[str, float], str]:
    recommendation = get_ai_recommendation(
        process_id=process_id,
        mode='geometry_driven',
        target_geometry=target_geometry,
        api_url=api_url,
    )
    raw_params = recommendation.get('recommended_params') if recommendation else {}
    if not isinstance(raw_params, dict):
        raw_params = {}

    reference_params = {
        'wireSpeed': extract_number(raw_params, ['wireSpeed', 'lineSpeedSetpoint', 'travelSpeed'], fallback_params['wireSpeed']),
        'current': extract_number(raw_params, ['current', 'flowRateSetpoint'], fallback_params['current']),
        'voltage': extract_number(raw_params, ['voltage', 'pressureSetpoint'], fallback_params['voltage']),
    }

    predicted = predict_geometry(reference_params, api_url)
    if predicted:
        return reference_params, predicted, 'ai-model-from-ai-suggested-params'

    return reference_params, target_geometry, 'geometry-target-fallback'


def run_single_mode(
    process_id: str,
    mode: str,
    duration: int,
    interval: float,
    tolerance: float,
    target_height: float,
    target_width: float,
    base_wire_speed: float,
    base_current: float,
    base_voltage: float,
    api_url: str,
) -> Tuple[int, int, int, int]:
    maybe_create_process(process_id, mode)
    set_process_mode(process_id, mode, api_url)

    fallback_params = {
        'wireSpeed': base_wire_speed,
        'current': base_current,
        'voltage': base_voltage,
    }

    if mode == 'geometry_driven':
        target_geometry = {'height': target_height, 'width': target_width}
        create_geometry_target(process_id, target_height, target_width)
        reference_params, expected_geometry, source = resolve_geometry_mode_plan(
            process_id=process_id,
            target_geometry=target_geometry,
            fallback_params=fallback_params,
            api_url=api_url,
        )
        print(f'[{process_id}] Geometry-driven plan source: {source}')
    else:
        reference_params = fallback_params
        expected_geometry, source = resolve_parameter_mode_expectation(
            process_id=process_id,
            input_params=reference_params,
            api_url=api_url,
        )
        print(f'[{process_id}] Parameter-driven expected geometry source: {source}')

    return stream_measurements(
        process_id=process_id,
        mode=mode,
        duration=duration,
        interval=interval,
        tolerance=tolerance,
        expected_geometry=expected_geometry,
        reference_params=reference_params,
        api_url=api_url,
    )


def prompt_for_mode(default_mode: str) -> str:
    options = {
        '1': 'parameter_driven',
        '2': 'geometry_driven',
        '3': 'both',
    }
    default_key = next((k for k, v in options.items() if v == default_mode), '1')
    while True:
        print('Select operational mode:')
        print('  1) parameter_driven')
        print('  2) geometry_driven')
        print('  3) both (runs both modes sequentially using separate process IDs)')
        raw = input(f'Choice [{default_key}]: ').strip()
        if raw == '':
            raw = default_key
        if raw in options:
            return options[raw]
        print('Invalid selection. Please choose 1, 2, or 3.')


def prompt_for_tolerance(default_tolerance: float) -> float:
    while True:
        raw = input(
            f'Set deviation threshold (%) [{default_tolerance:.1f}]: ',
        ).strip()
        if raw == '':
            return default_tolerance
        try:
            value = float(raw)
        except ValueError:
            print('Invalid value. Enter a number (for example: 8, 10, 12.5).')
            continue
        if value <= 0 or value > 100:
            print('Tolerance must be > 0 and <= 100.')
            continue
        return value


def interactive_config(args: argparse.Namespace) -> argparse.Namespace:
    if args.no_prompt or not sys.stdin.isatty():
        return args

    print(f'Open UI: {DASHBOARD_URL}')
    print('In the dashboard, use the top-bar process selector to pick the process created by this demo.')
    print()

    args.mode = prompt_for_mode(args.mode)
    args.tolerance = prompt_for_tolerance(args.tolerance)
    print(
        f'Configured run: mode={args.mode}, tolerance={args.tolerance:.1f}% '
        f'(set before streaming starts)'
    )
    print()
    return args


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Robust welding profile demo with explicit dual operational modes.',
    )
    parser.add_argument(
        '--process-id',
        default=f'weld-{int(time.time())}',
        help='Base process ID (default: weld-<timestamp>)',
    )
    parser.add_argument(
        '--mode',
        default='parameter_driven',
        choices=['parameter_driven', 'geometry_driven', 'both'],
        help='Which operational mode to run. "both" runs sequentially.',
    )
    parser.add_argument('--duration', type=int, default=120)
    parser.add_argument('--interval', type=float, default=2.0)
    parser.add_argument('--tolerance', type=float, default=10.0)
    parser.add_argument('--target-height', type=float, default=5.2)
    parser.add_argument('--target-width', type=float, default=3.8)
    parser.add_argument('--wire-speed', type=float, default=10.5,
                        help='Initial/setpoint wire speed')
    parser.add_argument('--current', type=float, default=150.0,
                        help='Initial/setpoint current')
    parser.add_argument('--voltage', type=float, default=24.0,
                        help='Initial/setpoint voltage')
    parser.add_argument('--api-url', default=ALERT_ENGINE_URL,
                        help='Alert Engine URL (default: http://localhost:8001)')
    parser.add_argument(
        '--no-prompt',
        action='store_true',
        help='Disable interactive prompts for mode/tolerance (useful for CI or scripted runs).',
    )
    return parser.parse_args()


def main() -> int:
    args = interactive_config(parse_args())

    print('===========================================================')
    print('  ROBIN - Robust Welding Demo (Dual Operational Modes)')
    print('===========================================================')
    print('Profile: welding')
    print('Domain mapping:')
    print('  Bead Height      -> measuredHeight')
    print('  Bead Width       -> measuredWidth')
    print('  Wire Speed       -> measuredSpeed')
    print('  Welding Current  -> measuredCurrent')
    print('  Arc Voltage      -> measuredVoltage')
    print()
    print('Dual modes demonstrated:')
    print('  parameter_driven: parameters -> AI predicted geometry -> deviation checks')
    print('  geometry_driven: target geometry -> AI suggested parameters -> deviation checks')
    print(f'Dashboard UI: {DASHBOARD_URL}')
    print()

    ensure_stack_running()

    modes = ['parameter_driven', 'geometry_driven'] if args.mode == 'both' else [args.mode]
    summary: List[Tuple[str, str, int, int, int, int]] = []

    for mode in modes:
        process_id = args.process_id if len(modes) == 1 else f'{args.process_id}-{mode.replace("_driven", "")}'
        print('-----------------------------------------------------------')
        print(f'Starting mode: {mode} (process: {process_id})')
        print('-----------------------------------------------------------')

        accepted, failed, streamed, alerts = run_single_mode(
            process_id=process_id,
            mode=mode,
            duration=args.duration,
            interval=args.interval,
            tolerance=args.tolerance,
            target_height=args.target_height,
            target_width=args.target_width,
            base_wire_speed=args.wire_speed,
            base_current=args.current,
            base_voltage=args.voltage,
            api_url=args.api_url,
        )
        summary.append((process_id, mode, accepted, failed, streamed, alerts))
        print()

    print('========================')
    print('Demo summary')
    print('========================')
    for process_id, mode, accepted, failed, streamed, alerts in summary:
        print(
            f'{process_id} [{mode}] -> '
            f'streamed={streamed}, accepted={accepted}, failed={failed}, '
            f'alerts_over_tolerance={alerts}'
        )

    print()
    print(f'Dashboard: {DASHBOARD_URL}')
    print('Tip: select the process ID above in the top-bar process selector.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
