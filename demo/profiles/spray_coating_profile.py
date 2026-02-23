#!/usr/bin/env python3
"""Robust spray coating profile demo for ROBIN.

This script is the canonical spray demo. It exercises both operational modes
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
class CoatingSample:
    thickness: float
    coverage_width: float
    line_speed: float
    flow_rate: float
    nozzle_pressure: float


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


def create_geometry_target(process_id: str, thickness: float, width: float) -> bool:
    result = run_robin(['create-target', process_id, str(thickness), str(width)], check=False)
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


def stop_process(process_id: str, api_url: str) -> None:
    """Stop the process so it waits for a UI Start signal."""
    try:
        requests.post(
            f'{api_url.rstrip("/")}/process/{process_id}/stop?reason=awaiting_start',
            timeout=5,
        )
    except Exception:
        pass


def post_progress(process_id: str, progress: float, expected_duration: float, api_url: str) -> None:
    """Report simulation progress (0-100) to the alert engine."""
    try:
        requests.post(
            f'{api_url.rstrip("/")}/process/{process_id}/progress',
            json={'progress': round(progress, 1), 'expected_duration': expected_duration},
            timeout=2,
        )
    except Exception:
        pass


def get_process_status(process_id: str, api_url: str) -> str:
    """Return the current process status from the backend ('active', 'stopped', etc.)."""
    try:
        resp = requests.get(
            f'{api_url.rstrip("/")}/process/{process_id}/status',
            timeout=5,
        )
        if resp.ok:
            return (resp.json().get('process_data') or {}).get('status', 'unknown')
    except Exception:
        pass
    return 'unknown'


def wait_for_start(process_id: str, api_url: str) -> None:
    """Poll process status until it becomes 'active' (user pressed Start in the UI)."""
    print(f'Waiting for Start from the dashboard UI for process "{process_id}"…')
    print(f'  Open {DASHBOARD_URL}, select process "{process_id}", and press Start.')
    while True:
        if get_process_status(process_id, api_url) == 'active':
            print('Start received – beginning simulation.')
            return
        time.sleep(1.0)


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
) -> CoatingSample:
    thickness = expected_geometry['height'] + 0.012 * math.sin(t * 0.27) + random.uniform(-0.004, 0.004)
    coverage_width = expected_geometry['width'] + 1.6 * math.sin(t * 0.33) + random.uniform(-0.35, 0.35)

    line_speed = reference_params['wireSpeed'] + 6.0 * math.sin(t * 0.21) + random.uniform(-1.2, 1.2)
    flow_rate = reference_params['current'] + 1.6 * math.sin(t * 0.39) + random.uniform(-0.4, 0.4)
    nozzle_pressure = reference_params['voltage'] + 0.20 * math.sin(t * 0.18) + random.uniform(-0.06, 0.06)

    if in_any_window(t, deviation_windows):
        thickness *= 0.72
        coverage_width *= 1.22
        flow_rate *= 0.86

    return CoatingSample(
        thickness=max(0.01, round(thickness, 4)),
        coverage_width=max(5.0, round(coverage_width, 3)),
        line_speed=max(1.0, round(line_speed, 3)),
        flow_rate=max(1.0, round(flow_rate, 3)),
        nozzle_pressure=max(0.2, round(nozzle_pressure, 3)),
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
        f'Streaming spray telemetry for {process_id} ({mode}) for {duration}s '
        f'at {interval:.2f}s interval.'
    )
    print(
        'Reference expectation: '
        f"{expected_geometry['height']:.4f} mm x {expected_geometry['width']:.3f} mm"
    )
    print(
        'Reference parameters: '
        f"lineSpeed={reference_params['wireSpeed']:.3f}, "
        f"flowRate={reference_params['current']:.3f}, "
        f"pressure={reference_params['voltage']:.3f}"
    )
    print(f'Deviation windows (s): {[(round(s, 1), round(e, 1)) for s, e in windows]}')
    print()

    count = 0
    ok_count = 0
    fail_count = 0
    alert_count = 0
    sim_elapsed = 0.0
    last_progress_post = 0.0
    last_tick = time.time()
    was_paused = False

    while sim_elapsed < duration:
        now = time.time()

        status = get_process_status(process_id, api_url)
        if status != 'active':
            if not was_paused:
                print('  ⏸  Paused by operator – waiting for Resume…')
                was_paused = True
            post_progress(process_id, min(100.0, (sim_elapsed / duration) * 100), float(duration), api_url)
            last_tick = now
            time.sleep(0.5)
            continue

        if was_paused:
            print('  ▶  Resumed – continuing simulation.')
            was_paused = False

        dt = now - last_tick
        last_tick = now
        sim_elapsed += dt

        sample = generate_sample(sim_elapsed, expected_geometry, reference_params, windows)
        measurement_id = f'coat-{int(time.time() * 1000)}-{count}'

        result = run_robin(
            [
                'add-measurement',
                process_id,
                measurement_id,
                str(sample.thickness),
                str(sample.coverage_width),
                '--speed',
                str(sample.line_speed),
                '--current',
                str(sample.flow_rate),
                '--voltage',
                str(sample.nozzle_pressure),
            ],
            check=False,
        )

        count += 1
        if result.returncode == 0:
            ok_count += 1
        else:
            fail_count += 1

        if now - last_progress_post >= 1.0:
            pct = min(100.0, (sim_elapsed / duration) * 100)
            post_progress(process_id, pct, float(duration), api_url)
            last_progress_post = now

        check = check_deviation(
            process_id=process_id,
            mode=mode,
            tolerance=tolerance,
            measured_geometry={'height': sample.thickness, 'width': sample.coverage_width},
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

        marker = ' DEVIATION_WINDOW' if in_any_window(sim_elapsed, windows) else ''
        print(
            f'[{count:03d}] '
            f't={sample.thickness:>7.4f}mm '
            f'w={sample.coverage_width:>7.3f}mm '
            f'ls={sample.line_speed:>7.3f} '
            f'fr={sample.flow_rate:>7.3f} '
            f'p={sample.nozzle_pressure:>6.3f} '
            f'dev={deviation_pct:>6.2f}% '
            f'status={status:<8} '
            f'src={source:<26} '
            f"alert={'YES' if alert_flag else 'no '}"
            f'{marker}'
        )

        time.sleep(interval)

    post_progress(process_id, 100, float(duration), api_url)
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

    return {'height': 0.12, 'width': 50.0}, 'fallback-default'


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
    target_thickness: float,
    target_width: float,
    base_line_speed: float,
    base_flow_rate: float,
    base_pressure: float,
    api_url: str,
    no_prompt: bool = False,
) -> Tuple[int, int, int, int]:
    maybe_create_process(process_id, mode)
    set_process_mode(process_id, mode, api_url)

    fallback_params = {
        'wireSpeed': base_line_speed,
        'current': base_flow_rate,
        'voltage': base_pressure,
    }

    if mode == 'geometry_driven':
        target_geometry = {'height': target_thickness, 'width': target_width}
        create_geometry_target(process_id, target_thickness, target_width)
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

    post_progress(process_id, 0, float(duration), api_url)
    if no_prompt:
        print(f'[{process_id}] --no-prompt: streaming immediately.')
    else:
        stop_process(process_id, api_url)
        wait_for_start(process_id, api_url)

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
        description='Robust spray coating profile demo with explicit dual operational modes.',
    )
    parser.add_argument(
        '--process-id',
        default=f'coating-{int(time.time())}',
        help='Base process ID (default: coating-<timestamp>)',
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
    parser.add_argument('--target-thickness', type=float, default=0.12)
    parser.add_argument('--target-width', type=float, default=50.0)
    parser.add_argument('--line-speed', type=float, default=200.0,
                        help='Initial/setpoint line speed')
    parser.add_argument('--flow-rate', type=float, default=45.0,
                        help='Initial/setpoint flow rate')
    parser.add_argument('--pressure', type=float, default=3.5,
                        help='Initial/setpoint nozzle pressure')
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

    print('==============================================================')
    print('  ROBIN - Robust Spray Coating Demo (Dual Operational Modes)')
    print('==============================================================')
    print('Profile: spray_coating')
    print('Domain mapping:')
    print('  Coating Thickness  -> measuredHeight')
    print('  Coverage Width     -> measuredWidth')
    print('  Line Speed         -> measuredSpeed')
    print('  Flow Rate          -> measuredCurrent')
    print('  Nozzle Pressure    -> measuredVoltage')
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
            target_thickness=args.target_thickness,
            target_width=args.target_width,
            base_line_speed=args.line_speed,
            base_flow_rate=args.flow_rate,
            base_pressure=args.pressure,
            api_url=args.api_url,
            no_prompt=args.no_prompt,
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
