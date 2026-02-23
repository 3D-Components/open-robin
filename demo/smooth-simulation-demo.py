#!/usr/bin/env python3
"""
ROBIN System - Smooth Simulation Demo
High-frequency welding process simulation with live data updates every 0.2 seconds
"""

import argparse
import math
import os
import random
import subprocess
import sys
import time
from typing import Any, Dict, Optional, Tuple

try:
    import requests
except ImportError:
    print(
        'Warning: requests module not available. Some features may be limited.'
    )
    requests = None


class Colors:
    """ANSI color codes for terminal output"""

    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    BLUE = '\033[0;34m'
    YELLOW = '\033[1;33m'
    PURPLE = '\033[0;35m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'  # No Color
    BOLD = '\033[1m'


class SimulationConfig:
    """Configuration for the welding simulation"""

    def __init__(self):
        # Timing configuration
        self.measurement_interval = 0.2  # 200ms between measurements
        self.simulation_duration = 120  # 2 minutes default
        self.progress_update_interval = (
            1.0  # Update progress display every second
        )

        # Container configuration
        self.container_name = 'robin-alert-processor'

        # Base welding parameters (operator configurable)
        self.base_height = 5.2
        self.base_width = 3.8
        self.base_wire_speed = 100.0  # mm/s wire feed target (~6 m/min)
        self.base_current = 150.0  # A
        self.base_voltage = 24.0  # V

        # Parameter oscillation amplitudes
        self.wire_speed_oscillation = 8.0
        self.current_oscillation = 3.0
        self.voltage_oscillation = 0.3

        # Random noise ranges for parameters (uniform +/- range)
        self.wire_speed_noise_range = 4.0
        self.current_noise_range = 1.0
        self.voltage_noise_range = 0.1

        # Physics model constants (aligned with scripts/process_geometry_simulator.py)
        self.wire_diameter_mm = 1.2
        self.eta_dep_mean = 0.85
        self.eta_dep_sd = 0.05
        self.eta_arc_mean = 0.80
        self.eta_arc_sd = 0.03
        self.eta_bounds = (0.60, 0.95)
        self.width_coefficient = 1.20
        self.width_exponent = 0.35
        self.width_min_mm = 0.5
        self.height_min_mm = 0.35

        # Travel speed model (wire speed is the only operator-facing speed)
        self.travel_speed_base = 5.0
        self.travel_speed_current_gain = 0.02
        self.travel_speed_wfs_gain = 0.50

        # Floors and measurement noise for realism
        self.wire_speed_floor = 40.0  # mm/s (~2.4 m/min)
        self.travel_speed_floor = 3.0

        self.current_floor = 90.0
        self.voltage_floor = 18.0
        self.width_measurement_noise = 0.02   # ±2 % (Gaussian)
        self.height_measurement_noise = 0.025  # ±2.5 % (Gaussian)

        # Alert scenarios (time points where we introduce significant deviations)
        self.alert_times = [30, 75, 105]  # seconds into simulation
        self.alert_duration = 8  # seconds each alert lasts

        # Alert engine configuration
        self.alert_engine_url = os.getenv(
            'ROBIN_ALERT_ENGINE_URL', 'http://localhost:8001'
        )
        self.default_tolerance = 10.0


class WeldingSimulator:
    """High-frequency welding process simulator"""

    def __init__(self, config: SimulationConfig):
        self.config = config
        self.start_time = None
        self.measurement_count = 0

    def generate_measurement(self, elapsed_time: float) -> Dict[str, float]:
        """Generate a measurement aligned with the process_geometry_simulator physics model."""

        t = elapsed_time

        # Smooth oscillations over time for the core process parameters
        wire_osc = math.sin(t * 0.3) * self.config.wire_speed_oscillation
        current_osc = math.sin(t * 0.4) * self.config.current_oscillation
        voltage_osc = math.sin(t * 0.6) * self.config.voltage_oscillation

        wire_noise = (
            (random.random() - 0.5) * 2 * self.config.wire_speed_noise_range
        )
        current_noise = (
            (random.random() - 0.5) * 2 * self.config.current_noise_range
        )
        voltage_noise = (
            (random.random() - 0.5) * 2 * self.config.voltage_noise_range
        )

        wire_speed = self.config.base_wire_speed + wire_osc + wire_noise
        current = self.config.base_current + current_osc + current_noise
        voltage = self.config.base_voltage + voltage_osc + voltage_noise

        # Inject alert scenarios by perturbing process parameters (not geometry directly)
        if self._should_trigger_alert(elapsed_time):
            wire_speed, current, voltage = self._apply_alert_deviation(
                elapsed_time, wire_speed, current, voltage
            )

        # Enforce realistic lower bounds
        wire_speed = max(self.config.wire_speed_floor, wire_speed)
        current = max(self.config.current_floor, current)
        voltage = max(self.config.voltage_floor, voltage)

        travel_speed = max(
            self.config.travel_speed_floor,
            self._estimate_travel_speed(wire_speed, current),
        )

        width, height = self._compute_physics_geometry(
            wire_speed, travel_speed, current, voltage
        )

        return {
            'height': round(height, 2),
            'width': round(width, 2),
            'wireSpeed': round(wire_speed, 2),
            'travelSpeed': round(travel_speed, 2),
            'current': round(current, 1),
            'voltage': round(voltage, 1),
        }

    def _should_trigger_alert(self, elapsed_time: float) -> bool:
        """Check if we should trigger an alert at this time"""
        for alert_time in self.config.alert_times:
            if (
                alert_time
                <= elapsed_time
                < alert_time + self.config.alert_duration
            ):
                return True
        return False

    def _apply_alert_deviation(
        self,
        elapsed_time: float,
        wire_speed: float,
        current: float,
        voltage: float,
    ) -> Tuple[float, float, float]:
        """Apply significant deviations during alert scenarios."""

        scenario_index = 0
        for i, alert_time in enumerate(self.config.alert_times):
            if (
                alert_time
                <= elapsed_time
                < alert_time + self.config.alert_duration
            ):
                scenario_index = i
                break

        if scenario_index == 0:
            # Height collapse: spike wire feed, trim current
            wire_speed *= 1.25
            current *= 0.9
        elif scenario_index == 1:
            # Width blow-out: bump voltage and current modestly
            voltage *= 1.22
            current *= 1.08
        else:
            # Mixed drift: slower travel with tired arc
            wire_speed *= 0.8
            current *= 0.7
            voltage *= 0.9

        return wire_speed, current, voltage

    def _estimate_travel_speed(
        self, wire_speed_mm_s: float, current_A: float
    ) -> float:
        """Deterministic travel speed estimate derived from wire speed and current."""

        cfg = self.config
        wire_feed_rate_m_per_min = wire_speed_mm_s * 60.0 / 1000.0
        vt = (
            cfg.travel_speed_base
            + cfg.travel_speed_current_gain * current_A
            + cfg.travel_speed_wfs_gain * wire_feed_rate_m_per_min
        )
        return vt

    @staticmethod
    def _bounded_gaussian(
        mean: float, sd: float, low: float, high: float
    ) -> float:
        value = random.gauss(mean, sd)
        if value < low:
            return low
        if value > high:
            return high
        return value

    def _compute_physics_geometry(
        self,
        wire_speed_mm_s: float,
        travel_speed_mm_s: float,
        current_A: float,
        voltage_V: float,
        use_noise: bool = True,
    ) -> Tuple[float, float]:
        cfg = self.config

        # Wire cross-sectional area (mm²)
        radius_mm = cfg.wire_diameter_mm / 2.0
        wire_area_mm2 = math.pi * radius_mm * radius_mm

        # Sample efficiencies with light stochasticity
        if use_noise:
            eta_dep = self._bounded_gaussian(
                cfg.eta_dep_mean,
                cfg.eta_dep_sd,
                cfg.eta_bounds[0],
                cfg.eta_bounds[1],
            )
            eta_arc = self._bounded_gaussian(
                cfg.eta_arc_mean,
                cfg.eta_arc_sd,
                cfg.eta_bounds[0],
                cfg.eta_bounds[1],
            )
        else:
            eta_dep = cfg.eta_dep_mean
            eta_arc = cfg.eta_arc_mean

        deposition_rate_mm3_s = eta_dep * wire_area_mm2 * wire_speed_mm_s
        cross_section_area_mm2 = deposition_rate_mm3_s / travel_speed_mm_s

        heat_input_J_per_mm = (
            eta_arc * voltage_V * current_A
        ) / travel_speed_mm_s

        width_mm = cfg.width_coefficient * (
            heat_input_J_per_mm**cfg.width_exponent
        )
        width_mm = max(width_mm, cfg.width_min_mm)

        height_mm = (4.0 * cross_section_area_mm2) / (math.pi * width_mm)
        height_mm = max(height_mm, cfg.height_min_mm)

        # Measurement noise (Gaussian around nominal value)
        if use_noise:
            width_mm *= 1 + random.gauss(0.0, cfg.width_measurement_noise)
            height_mm *= 1 + random.gauss(0.0, cfg.height_measurement_noise)
            width_mm = max(width_mm, cfg.width_min_mm)
            height_mm = max(height_mm, cfg.height_min_mm)

        return width_mm, height_mm


class DockerInterface:
    """Interface for Docker container operations"""

    def __init__(self, container_name: str):
        self.container_name = container_name

    def check_prerequisites(self) -> bool:
        """Check if Docker and ROBIN container are available"""
        try:
            # Check Docker
            result = subprocess.run(
                ['docker', '--version'], capture_output=True, text=True
            )
            if result.returncode != 0:
                print(
                    f'{Colors.RED}❌ Docker is not installed or not in PATH{Colors.NC}'
                )
                return False

            print(f'{Colors.GREEN}✅ Docker is available{Colors.NC}')

            # Check if container is running
            result = subprocess.run(
                ['docker', 'ps'], capture_output=True, text=True
            )
            if self.container_name not in result.stdout:
                print(
                    f'{Colors.YELLOW}⚠️  ROBIN container is not running{Colors.NC}'
                )
                print(f'{Colors.BLUE}Starting ROBIN system...{Colors.NC}')

                result = subprocess.run(
                    ['docker', 'compose', 'up', '-d'],
                    capture_output=True,
                    text=True,
                )
                if result.returncode != 0:
                    print(
                        f'{Colors.RED}❌ Failed to start ROBIN system{Colors.NC}'
                    )
                    print(f'Error: {result.stderr}')
                    return False

                print(f'{Colors.GREEN}✅ ROBIN system started{Colors.NC}')
                time.sleep(3)  # Give containers time to start
            else:
                print(f'{Colors.GREEN}✅ ROBIN container is running{Colors.NC}')

            return True

        except Exception as e:
            print(
                f'{Colors.RED}❌ Error checking prerequisites: {e}{Colors.NC}'
            )
            return False

    def run_robin_command(
        self, command: str, silent: bool = False
    ) -> Tuple[bool, str]:
        """Execute a ROBIN command in the container"""
        try:
            full_command = [
                'docker',
                'exec',
                self.container_name,
                'python',
                '-m',
                'robin',
            ] + command.split()

            if not silent:
                print(
                    f"{Colors.BLUE}➤ Executing: {' '.join(full_command[3:])}{Colors.NC}"
                )

            result = subprocess.run(
                full_command, capture_output=True, text=True, timeout=10
            )

            if result.returncode == 0:
                if not silent:
                    print(f'{Colors.GREEN}✅ Success{Colors.NC}')
                return True, result.stdout
            else:
                error_msg = result.stderr or result.stdout
                if not silent:
                    print(f'{Colors.RED}❌ Failed: {error_msg}{Colors.NC}')
                return False, error_msg

        except subprocess.TimeoutExpired:
            return False, 'Command timed out'
        except Exception as e:
            return False, str(e)


class ProgressDisplay:
    """Real-time progress display with live measurements"""

    def __init__(self, total_duration: float):
        self.total_duration = total_duration
        self.start_time = None
        self.last_update = 0

    def start(self):
        """Start the progress tracking"""
        self.start_time = time.time()

    def update(
        self,
        measurement: Dict[str, float],
        measurement_count: int,
        alert_active: bool = False,
    ):
        """Update the progress display"""
        if not self.start_time:
            return

        current_time = time.time()
        elapsed = current_time - self.start_time

        # Only update display every second to avoid flicker
        if current_time - self.last_update < 1.0:
            return
        self.last_update = current_time

        progress_percent = min(100, (elapsed / self.total_duration) * 100)
        remaining = max(0, self.total_duration - elapsed)

        # Progress bar
        bar_length = 40
        filled_length = int(bar_length * progress_percent / 100)
        bar = '█' * filled_length + '░' * (bar_length - filled_length)

        # Clear previous lines and display new info
        print('\033[2K\033[1A' * 4, end='')  # Clear 4 lines up

        print(f'{Colors.CYAN}📊 Welding Process Simulation{Colors.NC}')
        print(f'Progress: [{bar}] {progress_percent:.1f}%')
        print(
            f'Time: {elapsed:.1f}s / {self.total_duration:.0f}s | Remaining: {remaining:.0f}s | Measurements: {measurement_count}'
        )

        # Current measurements with alert indicator
        alert_indicator = (
            f'{Colors.RED}🚨 ALERT ACTIVE{Colors.NC}' if alert_active else ''
        )
        wire_speed = measurement.get('wireSpeed')
        travel_speed = measurement.get('travelSpeed')
        speed_parts = []
        if wire_speed is not None:
            speed_parts.append(f'WS={wire_speed:.1f}mm/s')
        if travel_speed is not None:
            speed_parts.append(f'TS={travel_speed:.1f}mm/s')
        speed_str = ' '.join(speed_parts)

        print(
            f"Current: H={measurement['height']:.2f}mm W={measurement['width']:.2f}mm "
            f"{speed_str} C={measurement['current']:.0f}A "
            f"V={measurement['voltage']:.1f}V {alert_indicator}"
        )


class SmoothSimulationDemo:
    """Main simulation demo class"""

    def __init__(self):
        self.config = SimulationConfig()
        self.simulator = WeldingSimulator(self.config)
        self.docker = DockerInterface(self.config.container_name)
        self.progress = None
        self.operator_params = None
        self.predicted_geometry = None
        self.tolerance = self.config.default_tolerance
        self.last_alert_active = False
        self.last_alert_payload = None

    def show_header(self):
        """Display the demo header"""
        os.system('clear' if os.name == 'posix' else 'cls')
        print(
            f'{Colors.BLUE}{Colors.BOLD}╔══════════════════════════════════════╗{Colors.NC}'
        )
        print(
            f'{Colors.BLUE}{Colors.BOLD}║     ROBIN Smooth Simulation Demo     ║{Colors.NC}'
        )
        print(
            f'{Colors.BLUE}{Colors.BOLD}║   High-Frequency Live Data Updates   ║{Colors.NC}'
        )
        print(
            f'{Colors.BLUE}{Colors.BOLD}╚══════════════════════════════════════╝{Colors.NC}'
        )
        print()

    def explain_features(self):
        """Explain the simulation features"""
        print(f'{Colors.PURPLE}💡 Smooth Simulation Features:{Colors.NC}')
        print(
            f'- {Colors.CYAN}High-frequency updates: Every {self.config.measurement_interval}s{Colors.NC}'
        )
        print(
            f'- {Colors.CYAN}Realistic oscillations with multiple frequency components{Colors.NC}'
        )
        print(
            f'- {Colors.CYAN}Smooth noise patterns for authentic sensor simulation{Colors.NC}'
        )
        print(
            f'- {Colors.CYAN}Live progress display with real-time measurements{Colors.NC}'
        )
        print(
            f'- {Colors.CYAN}Automatic alert scenarios at strategic intervals{Colors.NC}'
        )
        print(
            f'- {Colors.CYAN}AI predictions benchmark your chosen parameters against live bead geometry{Colors.NC}'
        )
        print()

    def get_process_id_from_user(self) -> str:
        """Get process ID from user input"""
        print(f'{Colors.BLUE}Step 1: Choose Process ID{Colors.NC}')
        print('=' * 50)
        print(f"{Colors.CYAN}Enter an existing process ID, or type a new one.{Colors.NC}")
        print(f"{Colors.CYAN}The simulator will automatically create it in Orion if it's missing.{Colors.NC}")
        print()

        while True:
            process_id = input(
                f'{Colors.YELLOW}Process ID: {Colors.NC}'
            ).strip()
            if process_id:
                return process_id
            print(
                f'{Colors.RED}❌ Process ID cannot be empty. Please try again.{Colors.NC}'
            )

    def ensure_process_created(self, process_id: str) -> bool:
        """Ensure a Process exists and is active.

        Tries Alert Engine first, then checks Orion on localhost, and finally
        falls back to the in-container CLI.
        """
        # Try Alert Engine endpoint
        if requests is not None:
            try:
                url = self.config.alert_engine_url.rstrip('/') + '/create-process'
                payload = {'process_id': process_id, 'mode': 'parameter_driven'}
                resp = requests.post(url, json=payload, timeout=5)
                if resp.status_code == 200:
                    data = resp.json()
                    if data.get('status') == 'success':
                        print(f"{Colors.GREEN}✅ Created process via Alert Engine{Colors.NC}")
                        return True
                    else:
                        # Continue to existence check / fallback
                        print(
                            f"{Colors.YELLOW}⚠️  Alert Engine could not create process: {data.get('error', 'unknown')}{Colors.NC}"
                        )
                else:
                    print(
                        f"{Colors.YELLOW}⚠️  Alert Engine create-process HTTP {resp.status_code}{Colors.NC}"
                    )
            except Exception as exc:
                print(
                    f"{Colors.YELLOW}⚠️  Failed to contact Alert Engine for create-process: {exc}{Colors.NC}"
                )

        # Check existence directly against Orion (host-mapped port)
        if requests is not None:
            try:
                entity_id = f'urn:ngsi-ld:Process:{process_id}'
                headers = {'NGSILD-Tenant': 'robin'}
                resp = requests.get(
                    f'http://localhost:1026/ngsi-ld/v1/entities/{entity_id}',
                    headers=headers,
                    timeout=5,
                )
                if resp.status_code == 200:
                    print(
                        f"{Colors.GREEN}ℹ️  Process already exists in Orion; continuing{Colors.NC}"
                    )
                    return True
            except Exception as exc:
                print(
                    f"{Colors.YELLOW}⚠️  Could not verify process existence on Orion: {exc}{Colors.NC}"
                )

        # Fallback: attempt creation via in-container CLI
        ok, _out = self.docker.run_robin_command(
            f'create-process {process_id}', silent=False
        )
        if ok:
            return True

        print(
            f"{Colors.RED}❌ Unable to create or verify process {process_id}. Please create it via the dashboard or CLI and retry.{Colors.NC}"
        )
        return False

    def configure_parameter_mode(self, process_id: str) -> bool:
        """Collect operator parameters and fetch AI expectations"""
        print(
            f'\n{Colors.BLUE}Step 2: Configure Parameter-Driven Inputs{Colors.NC}'
        )
        print('=' * 50)

        self.prompt_for_operator_parameters()
        if not self.operator_params:
            print(
                f'{Colors.RED}❌ No parameters provided. Aborting setup.{Colors.NC}'
            )
            return False

        if not self.fetch_ai_prediction(process_id):
            print(
                f'{Colors.YELLOW}⚠️  Using internal estimate for predicted geometry (Alert Engine unreachable){Colors.NC}'
            )
            self.predicted_geometry = self.estimate_geometry_from_params(
                self.operator_params
            )

        if self.predicted_geometry:
            self.config.base_height = self.predicted_geometry['height']
            self.config.base_width = self.predicted_geometry['width']

        self.show_parameter_overview()
        return True

    def _prompt_float(self, prompt_text: str, default: float) -> float:
        """Prompt the user for a floating point value with a default"""
        while True:
            value = input(f'{prompt_text} [{default}]: ').strip()
            if value == '':
                return default
            try:
                return float(value)
            except ValueError:
                print(
                    f'{Colors.RED}❌ Invalid number, please try again.{Colors.NC}'
                )

    def prompt_for_operator_parameters(self):
        """Collect desired welding parameters and tolerance from the user"""
        print(
            f'{Colors.CYAN}Enter the welding parameters you intend to use. These should match the values configured in the Process Controls widget.{Colors.NC}'
        )
        print(
            f'{Colors.CYAN}Press Enter to keep the suggested default shown in brackets.{Colors.NC}'
        )

        wire_speed = self._prompt_float(
            'Wire speed (mm/s)', self.config.base_wire_speed
        )
        current = self._prompt_float('Current (A)', self.config.base_current)
        voltage = self._prompt_float('Voltage (V)', self.config.base_voltage)
        tolerance = self._prompt_float(
            'Deviation tolerance (%)', self.config.default_tolerance
        )

        self.operator_params = {
            'wireSpeed': round(wire_speed, 2),
            'current': round(current, 2),
            'voltage': round(voltage, 2),
        }
        self.tolerance = tolerance

        self.config.base_wire_speed = wire_speed
        self.config.base_current = current
        self.config.base_voltage = voltage

        travel_speed = self.simulator._estimate_travel_speed(
            wire_speed, current
        )
        (
            width_nominal,
            height_nominal,
        ) = self.simulator._compute_physics_geometry(
            wire_speed, travel_speed, current, voltage, use_noise=False
        )
        self.config.base_width = round(width_nominal, 2)
        self.config.base_height = round(height_nominal, 2)

    def fetch_ai_prediction(self, process_id: str) -> bool:
        """Ask the Alert Engine for predicted geometry given the parameters"""
        if requests is None:
            print(
                f"{Colors.YELLOW}⚠️  'requests' module not available. Cannot query Alert Engine.{Colors.NC}"
            )
            return False

        url = self.config.alert_engine_url.rstrip('/') + '/ai-recommendation'
        payload = {
            'process_id': process_id,
            'mode': 'parameter_driven',
            'input_params': self.operator_params,
        }

        try:
            response = requests.post(url, json=payload, timeout=5)
            if response.status_code != 200:
                print(
                    f'{Colors.YELLOW}⚠️  Alert Engine returned {response.status_code}: {response.text}{Colors.NC}'
                )
                return False

            data = response.json()
            predicted = data.get('predicted_geometry')
            if not predicted:
                print(
                    f'{Colors.YELLOW}⚠️  Alert Engine did not send a predicted geometry payload.{Colors.NC}'
                )
                return False

            self.predicted_geometry = {
                'height': float(predicted['height']),
                'width': float(predicted['width']),
            }
            return True
        except Exception as exc:
            print(
                f'{Colors.YELLOW}⚠️  Failed to contact Alert Engine: {exc}{Colors.NC}'
            )
            return False

    def estimate_geometry_from_params(
        self, params: Dict[str, float]
    ) -> Dict[str, float]:
        """Fallback estimation aligned with the physics model."""
        wire_speed = params.get('wireSpeed', self.config.base_wire_speed)
        current = params.get('current', self.config.base_current)
        voltage = params.get('voltage', self.config.base_voltage)
        travel_speed = self.simulator._estimate_travel_speed(
            wire_speed, current
        )
        width, height = self.simulator._compute_physics_geometry(
            wire_speed, travel_speed, current, voltage, use_noise=False
        )
        return {'height': round(height, 2), 'width': round(width, 2)}

    def show_parameter_overview(self):
        """Display chosen parameters, tolerance and AI expectation"""
        print()
        print(
            f'{Colors.GREEN}✅ Parameter-driven configuration ready{Colors.NC}'
        )
        print(
            f"- Wire speed: {Colors.YELLOW}{self.operator_params['wireSpeed']} mm/s{Colors.NC}"
        )
        print(
            f"- Current: {Colors.YELLOW}{self.operator_params['current']} A{Colors.NC}"
        )
        print(
            f"- Voltage: {Colors.YELLOW}{self.operator_params['voltage']} V{Colors.NC}"
        )
        print(f'- Tolerance: {Colors.YELLOW}{self.tolerance}%{Colors.NC}')
        if self.predicted_geometry:
            print(
                f"- AI expects geometry ≈ {Colors.YELLOW}{self.predicted_geometry['height']:.2f} x {self.predicted_geometry['width']:.2f} mm{Colors.NC}"
            )
        else:
            print(
                f'- {Colors.YELLOW}AI prediction unavailable. Using internal estimate for baseline.{Colors.NC}'
            )
        print()

    def perform_deviation_check(
        self, process_id: str, measurement: Dict[str, float]
    ) -> Optional[Dict[str, Any]]:
        """Call the Alert Engine to compare measured geometry with AI expectations"""
        if requests is None or not self.operator_params:
            return None

        payload = {
            'process_id': process_id,
            'mode': 'parameter_driven',
            'input_params': self.operator_params,
            'measured_geometry': {
                'height': float(measurement['height']),
                'width': float(measurement['width']),
            },
            'tolerance': float(self.tolerance),
        }
        url = self.config.alert_engine_url.rstrip('/') + '/check-deviation'

        try:
            response = requests.post(url, json=payload, timeout=5)
            if response.status_code != 200:
                return None
            data = response.json()
            if 'deviation_percentage' in data:
                if not self.last_alert_active:
                    deviation = data['deviation_percentage']
                    expected = data['expected_value']
                    measured = data['measured_value']
                    print(
                        f"\n{Colors.RED}🚨 Deviation detected! Expected ~{expected['height']:.2f}x{expected['width']:.2f} mm, got {measured['height']:.2f}x{measured['width']:.2f} mm ({deviation:.1f}%){Colors.NC}"
                    )
                self.last_alert_active = True
                self.last_alert_payload = data
                return data
            else:
                if self.last_alert_active and self.last_alert_payload:
                    print(
                        f'\n{Colors.GREEN}✅ Deviation cleared. Geometry back within tolerance.{Colors.NC}'
                    )
                self.last_alert_active = False
                self.last_alert_payload = None
                return None
        except Exception:
            return None

    def run_simulation(self, process_id: str):
        """Run the main simulation loop"""
        print(f'\n{Colors.BLUE}Step 3: High-Frequency Simulation{Colors.NC}')
        print('=' * 50)
        print(
            f'{Colors.CYAN}Starting smooth simulation with {1 / self.config.measurement_interval:.0f} Hz updates{Colors.NC}'
        )
        print(
            f'{Colors.YELLOW}🎯 Watch the dashboard for live updates at: http://localhost:5174{Colors.NC}'
        )
        print()

        # Initialize progress display
        self.progress = ProgressDisplay(self.config.simulation_duration)
        self.progress.start()
        self.last_alert_active = False
        self.last_alert_payload = None

        # Print initial empty lines for progress display
        for _ in range(4):
            print()

        start_time = time.time()
        next_measurement_time = start_time
        measurement_count = 0

        try:
            while True:
                current_time = time.time()
                elapsed = current_time - start_time

                # Check if simulation is complete
                if elapsed >= self.config.simulation_duration:
                    break

                # Check if it's time for next measurement
                if current_time >= next_measurement_time:
                    measurement_count += 1

                    # Generate measurement
                    measurement = self.simulator.generate_measurement(elapsed)

                    # Determine if this timeframe should trigger a simulated deviation
                    simulated_alert = self.simulator._should_trigger_alert(
                        elapsed
                    )

                    # Create unique measurement ID
                    measure_id = f'measure{int(current_time * 1000)}'  # Use milliseconds for uniqueness

                    # Send measurement to ROBIN system
                    command = (
                        f'add-measurement {process_id} {measure_id} '
                        f"{measurement['height']} {measurement['width']} "
                        f"--speed {measurement['wireSpeed']} "
                        f"--current {measurement['current']} "
                        f"--voltage {measurement['voltage']}"
                    )

                    success, output = self.docker.run_robin_command(
                        command, silent=True
                    )

                    if (
                        not success and measurement_count % 10 == 0
                    ):  # Only show occasional errors
                        print(
                            f'\n{Colors.YELLOW}⚠️  Some measurements may be failing - check ROBIN system{Colors.NC}'
                        )

                    alert_payload = self.perform_deviation_check(
                        process_id, measurement
                    )
                    alert_active = bool(alert_payload)
                    if alert_payload is None and requests is None:
                        alert_active = simulated_alert

                    # Update progress display
                    self.progress.update(
                        measurement, measurement_count, alert_active
                    )

                    # Schedule next measurement
                    next_measurement_time += self.config.measurement_interval

                # Small sleep to prevent excessive CPU usage
                time.sleep(0.05)  # 50ms sleep

        except KeyboardInterrupt:
            print(
                f'\n\n{Colors.YELLOW}Simulation interrupted by user{Colors.NC}'
            )
            return measurement_count

        return measurement_count

    def show_completion_summary(self, process_id: str, measurement_count: int):
        """Show completion summary"""
        print(f'\n\n{Colors.GREEN}🎉 Smooth Simulation Complete!{Colors.NC}')
        print('=' * 50)
        print(
            f'{Colors.GREEN}Total measurements sent: {Colors.YELLOW}{measurement_count}{Colors.NC}'
        )
        print(
            f'{Colors.GREEN}Average rate: {Colors.YELLOW}{measurement_count / self.config.simulation_duration:.1f} measurements/second{Colors.NC}'
        )
        print()

        print(f'{Colors.BLUE}What you should have seen:{Colors.NC}')
        print(
            f'- {Colors.GREEN}Smooth, high-frequency measurement updates in the ROBIN dashboard{Colors.NC}'
        )
        print(
            f'- {Colors.GREEN}Oscillations around AI-predicted geometry (~{self.config.base_height:.2f}x{self.config.base_width:.2f} mm){Colors.NC}'
        )
        print(
            f'- {Colors.GREEN}Realistic time-series charts with smooth curves{Colors.NC}'
        )
        print(
            f'- {Colors.GREEN}Real-time alerts whenever measurements exceed {self.tolerance:.1f}% deviation from AI expectations{Colors.NC}'
        )
        print(
            f"- {Colors.GREEN}Alert scenarios at {', '.join(map(str, self.config.alert_times))} seconds{Colors.NC}"
        )
        print()

        print(f'{Colors.BLUE}Next steps:{Colors.NC}')
        print('- Explore the smooth historical data in the dashboard telemetry charts')
        print(
            f'- Try different update intervals: {Colors.YELLOW}--interval 0.1{Colors.NC} for even smoother updates'
        )
        print(
            f'- Run longer simulations: {Colors.YELLOW}--duration 300{Colors.NC}'
        )
        print(
            f'- Clean up demo data: {Colors.YELLOW}./demo/cleanup-demo.sh {process_id}{Colors.NC}'
        )
        print()

        print(
            f'{Colors.GREEN}Thank you for exploring the ROBIN smooth simulation demo!{Colors.NC}'
        )

    def run(self, args):
        """Main execution method"""
        # Update config from command line arguments
        if args.duration:
            self.config.simulation_duration = args.duration
        if args.interval:
            self.config.measurement_interval = args.interval

        self.show_header()
        self.explain_features()

        # Check prerequisites
        print(f'{Colors.BLUE}🔍 Checking Prerequisites{Colors.NC}')
        print('=' * 30)
        if not self.docker.check_prerequisites():
            print(f'{Colors.RED}❌ Prerequisites not met. Exiting.{Colors.NC}')
            sys.exit(1)
        print()

        # Get process ID
        process_id = self.get_process_id_from_user()
        print(
            f'{Colors.GREEN}✅ Using process ID: {Colors.YELLOW}{process_id}{Colors.NC}'
        )

        # Ensure process exists (automate creation)
        if not self.ensure_process_created(process_id):
            print(
                f'{Colors.RED}❌ Failed to set up process in Orion. Exiting.{Colors.NC}'
            )
            return

        # Configure parameter-driven workflow
        if not self.configure_parameter_mode(process_id):
            print(
                f'{Colors.RED}❌ Failed to configure parameter-driven mode. Exiting.{Colors.NC}'
            )
            return

        # Run simulation
        measurement_count = self.run_simulation(process_id)

        # Show completion summary
        self.show_completion_summary(process_id, measurement_count)


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='ROBIN Smooth Simulation Demo - High-frequency welding process simulation'
    )
    parser.add_argument(
        '-d',
        '--duration',
        type=float,
        default=120,
        help='Simulation duration in seconds (default: 120)',
    )
    parser.add_argument(
        '-i',
        '--interval',
        type=float,
        default=0.2,
        help='Measurement interval in seconds (default: 0.2)',
    )
    parser.add_argument(
        '--base-height',
        type=float,
        default=5.2,
        help='Base height target in mm (default: 5.2)',
    )
    parser.add_argument(
        '--base-width',
        type=float,
        default=3.8,
        help='Base width target in mm (default: 3.8)',
    )

    args = parser.parse_args()

    # Validate arguments
    if args.interval < 0.1:
        print(
            f'{Colors.YELLOW}⚠️  Warning: Very short intervals (<0.1s) may cause system overload{Colors.NC}'
        )
    if args.interval > 2.0:
        print(
            f'{Colors.YELLOW}⚠️  Warning: Long intervals (>2s) may not appear smooth{Colors.NC}'
        )

    try:
        demo = SmoothSimulationDemo()

        # Update config from args
        if args.base_height:
            demo.config.base_height = args.base_height
        if args.base_width:
            demo.config.base_width = args.base_width

        demo.run(args)

    except KeyboardInterrupt:
        print(f'\n{Colors.YELLOW}Demo interrupted. Goodbye!{Colors.NC}')
        sys.exit(0)
    except Exception as e:
        print(f'{Colors.RED}❌ Unexpected error: {e}{Colors.NC}')
        sys.exit(1)


if __name__ == '__main__':
    main()
