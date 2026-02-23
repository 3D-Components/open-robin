# robin/cli.py
import json
import time
from datetime import datetime, timezone
from typing import Dict, Any
from enum import Enum
import os
import typer
import requests

app = typer.Typer(help='ROBIN - Open Process Intelligence Core')

# Use the docker-compose service name for Orion-LD to ensure DNS resolution inside containers
ORION = 'http://orion-ld:1026'
# Prefer environment configuration; Mintaka listens on 8080 inside its container
MTK = os.getenv('MINTAKA_URL', 'http://mintaka:8080')
# Tenant is optional; when empty, no NGSILD-Tenant header is sent
TENANT = os.getenv('NGSILD_TENANT', '')


class OperationMode(str, Enum):
    PARAMETER_DRIVEN = 'parameter_driven'
    GEOMETRY_DRIVEN = 'geometry_driven'


class RobinFiwareClient:
    ROBIN_NS = 'urn:robin:'
    PROCESS_ENTITY_TYPE = 'urn:robin:Process'
    PROCESS_ENTITY_ID_PREFIX = 'urn:ngsi-ld:Process:'

    TYPE_MAP = {
        'Process': 'urn:robin:Process',
        'ProcessRun': 'urn:robin:ProcessRun',
        'GeometryTarget': 'urn:robin:GeometryTarget',
        'Measurement': 'urn:robin:Measurement',
        'AIRecommendation': 'urn:robin:AIRecommendation',
        'Deviation': 'urn:robin:Deviation',
        'Alert': 'urn:robin:Alert',
    }

    def __init__(self, orion_url=ORION):
        self.orion_url = orion_url
        self.headers = {
            'Content-Type': 'application/json',
        }
        if TENANT:
            self.headers['NGSILD-Tenant'] = TENANT

    def create_process(self, process_id: str, mode: OperationMode):
        """Create main process entity"""
        entity = {
            'id': f'{self.PROCESS_ENTITY_ID_PREFIX}{process_id}',
            'type': self.PROCESS_ENTITY_TYPE,
            'operationMode': {'type': 'Property', 'value': mode.value},
            # 'temperature': {'type': 'Property', 'value': 0, 'unitCode': 'CEL'},
            # 'wireSpeed': {'type': 'Property', 'value': 0, 'unitCode': 'm/min'},
            # 'voltage': {'type': 'Property', 'value': 0, 'unitCode': 'V'},
            # 'current': {'type': 'Property', 'value': 0, 'unitCode': 'A'},
            # 'travelSpeed': {
            #     'type': 'Property',
            #     'value': 0,
            #     'unitCode': 'mm/s',
            # },
            'toleranceThreshold': {
                'type': 'Property',
                'value': 10.0,  # Default 10%, configurable via WireCloud
                'unitCode': 'P1',
            },
            'processStatus': {
                'type': 'Property',
                'value': 'active',  # active, stopped, paused, error
            },
            'startedAt': {
                'type': 'Property',
                'value': datetime.now(timezone.utc).isoformat(),
            },
        }

        response = requests.post(
            f'{self.orion_url}/ngsi-ld/v1/entities',
            headers=self.headers,
            json=entity,
        )
        return response.status_code in (200, 201)

    def stop_process(self, process_id: str, reason: str = 'operator_request'):
        """Stop a running process"""
        entity_id = f'{self.PROCESS_ENTITY_ID_PREFIX}{process_id}'

        # First check if process exists and is active
        get_response = requests.get(
            f'{self.orion_url}/ngsi-ld/v1/entities/{entity_id}',
            headers=self.headers,
        )

        if get_response.status_code == 404:
            return False, f'Process {process_id} not found'

        if get_response.status_code != 200:
            return False, f'Failed to fetch process {process_id}'

        process_data = get_response.json()
        current_status = process_data.get('processStatus', {}).get(
            'value', 'unknown'
        )

        if current_status == 'stopped':
            return False, f'Process {process_id} is already stopped'

        # First update the processStatus
        status_payload = {
            'processStatus': {
                'type': 'Property',
                'value': 'stopped',
            },
        }

        status_response = requests.patch(
            f'{self.orion_url}/ngsi-ld/v1/entities/{entity_id}/attrs',
            headers=self.headers,
            json=status_payload,
        )

        if status_response.status_code != 204:
            return (
                False,
                f'Failed to update process status: {status_response.text}',
            )

        # Then add the new attributes (stoppedAt and stopReason)
        new_attrs_payload = {
            'stoppedAt': {
                'type': 'Property',
                'value': datetime.now(timezone.utc).isoformat(),
            },
            'stopReason': {
                'type': 'Property',
                'value': reason,
            },
        }

        response = requests.post(
            f'{self.orion_url}/ngsi-ld/v1/entities/{entity_id}/attrs',
            headers=self.headers,
            json=new_attrs_payload,
        )

        if response.status_code == 204:
            return True, f'Process {process_id} stopped successfully'
        else:
            return (
                False,
                f'Failed to stop process {process_id}: {response.text}',
            )

    def resume_process(self, process_id: str):
        """Resume a stopped process"""
        entity_id = f'{self.PROCESS_ENTITY_ID_PREFIX}{process_id}'

        # Check if process exists and is stopped
        get_response = requests.get(
            f'{self.orion_url}/ngsi-ld/v1/entities/{entity_id}',
            headers=self.headers,
        )

        if get_response.status_code == 404:
            return False, f'Process {process_id} not found'

        if get_response.status_code != 200:
            return False, f'Failed to fetch process {process_id}'

        process_data = get_response.json()
        current_status = process_data.get('processStatus', {}).get(
            'value', 'unknown'
        )

        if current_status == 'active':
            return False, f'Process {process_id} is already active'

        # Update process status to active
        status_payload = {
            'processStatus': {
                'type': 'Property',
                'value': 'active',
            },
        }

        status_response = requests.patch(
            f'{self.orion_url}/ngsi-ld/v1/entities/{entity_id}/attrs',
            headers=self.headers,
            json=status_payload,
        )

        if status_response.status_code != 204:
            return (
                False,
                f'Failed to update process status: {status_response.text}',
            )

        # Remove stoppedAt and stopReason attributes
        for attr in ['stoppedAt', 'stopReason']:
            delete_response = requests.delete(
                f'{self.orion_url}/ngsi-ld/v1/entities/{entity_id}/attrs/{attr}',
                headers=self.headers,
            )
            # It's okay if the attribute doesn't exist (404), but other errors should be reported
            if delete_response.status_code not in [204, 404]:
                return (
                    False,
                    f'Failed to remove {attr} attribute: {delete_response.text}',
                )

        response = status_response

        if response.status_code == 204:
            return True, f'Process {process_id} resumed successfully'
        else:
            return (
                False,
                f'Failed to resume process {process_id}: {response.text}',
            )

    def get_process_status(self, process_id: str):
        """Get current status of a process"""
        entity_id = f'{self.PROCESS_ENTITY_ID_PREFIX}{process_id}'

        response = requests.get(
            f'{self.orion_url}/ngsi-ld/v1/entities/{entity_id}',
            headers=self.headers,
        )

        if response.status_code == 404:
            return None, f'Process {process_id} not found'

        if response.status_code != 200:
            return None, f'Failed to fetch process {process_id}'

        process_data = response.json()
        telemetry = process_data.get('processTelemetry') or process_data.get(
            'urn:robin:processTelemetry', {}
        )
        status_info = {
            'process_id': process_id,
            'status': process_data.get('processStatus', {}).get(
                'value', 'unknown'
            ),
            'started_at': process_data.get('startedAt', {}).get('value'),
            'stopped_at': process_data.get('stoppedAt', {}).get('value'),
            'stop_reason': process_data.get('stopReason', {}).get('value'),
            'operation_mode': process_data.get('operationMode', {}).get(
                'value'
            ),
            'telemetry': telemetry.get('value')
            if isinstance(telemetry, dict)
            else None,
        }

        return status_info, None

    def create_geometry_target(
        self, process_id: str, height: float, width: float
    ):
        """For Geometry-Driven mode: specify target geometry"""
        entity = {
            'id': f'urn:ngsi-ld:GeometryTarget:{process_id}',
            'type': 'urn:robin:GeometryTarget',
            'targetHeight': {
                'type': 'Property',
                'value': height,
                'unitCode': 'MMT',
            },
            'targetWidth': {
                'type': 'Property',
                'value': width,
                'unitCode': 'MMT',
            },
            'processId': {
                'type': 'Relationship',
                'object': f'{self.PROCESS_ENTITY_ID_PREFIX}{process_id}',
            },
        }

        response = requests.post(
            f'{self.orion_url}/ngsi-ld/v1/entities',
            headers=self.headers,
            json=entity,
        )
        return response.status_code in (200, 201)

    def set_operation_mode(self, process_id: str, mode: OperationMode) -> bool:
        """Set the operationMode of a Process entity."""
        try:
            entity_id = f'{self.PROCESS_ENTITY_ID_PREFIX}{process_id}'
            payload = {
                'operationMode': {
                    'type': 'Property',
                    'value': mode.value,
                },
            }
            resp = requests.patch(
                f'{self.orion_url}/ngsi-ld/v1/entities/{entity_id}/attrs',
                headers=self.headers,
                json=payload,
                timeout=5,
            )
            return 200 <= resp.status_code < 300
        except Exception:
            return False

    def create_measurement(
        self,
        process_id: str,
        measurement_id: str,
        height: float,
        width: float,
        speed: float = None,
        current: float = None,
        voltage: float = None,
    ):
        """Store measurement values with optional machine parameters"""
        timestamp = datetime.now(timezone.utc).isoformat()
        entity = {
            'id': f'urn:ngsi-ld:Measurement:{measurement_id}',
            'type': 'urn:robin:Measurement',
            'measuredHeight': {
                'type': 'Property',
                'value': height,
                'unitCode': 'MMT',
                'observedAt': timestamp,
            },
            'measuredWidth': {
                'type': 'Property',
                'value': width,
                'unitCode': 'MMT',
                'observedAt': timestamp,
            },
            'processId': {
                'type': 'Relationship',
                'object': f'{self.PROCESS_ENTITY_ID_PREFIX}{process_id}',
            },
        }

        # Add machine parameters if provided
        if speed is not None:
            entity['measuredSpeed'] = {
                'type': 'Property',
                'value': speed,
                'unitCode': 'mm/s',
                'observedAt': timestamp,
            }

        if current is not None:
            entity['measuredCurrent'] = {
                'type': 'Property',
                'value': current,
                'unitCode': 'A',
                'observedAt': timestamp,
            }

        if voltage is not None:
            entity['measuredVoltage'] = {
                'type': 'Property',
                'value': voltage,
                'unitCode': 'V',
                'observedAt': timestamp,
            }

        response = requests.post(
            f'{self.orion_url}/ngsi-ld/v1/entities',
            headers=self.headers,
            json=entity,
        )
        created = response.status_code in (200, 201)

        # Additionally PATCH the Process with current measured attributes (TROE)
        troe_ok = False
        try:
            process_entity_id = f'{self.PROCESS_ENTITY_ID_PREFIX}{process_id}'
            attrs_payload: Dict[str, Any] = {
                'measuredHeight': {
                    'type': 'Property',
                    'value': height,
                    'unitCode': 'MMT',
                    'observedAt': timestamp,
                },
                'measuredWidth': {
                    'type': 'Property',
                    'value': width,
                    'unitCode': 'MMT',
                    'observedAt': timestamp,
                },
            }
            if speed is not None:
                attrs_payload['measuredSpeed'] = {
                    'type': 'Property',
                    'value': speed,
                    'unitCode': 'mm/s',
                    'observedAt': timestamp,
                }
            if current is not None:
                attrs_payload['measuredCurrent'] = {
                    'type': 'Property',
                    'value': current,
                    'unitCode': 'A',
                    'observedAt': timestamp,
                }
            if voltage is not None:
                attrs_payload['measuredVoltage'] = {
                    'type': 'Property',
                    'value': voltage,
                    'unitCode': 'V',
                    'observedAt': timestamp,
                }

            patch_resp = requests.patch(
                f'{self.orion_url}/ngsi-ld/v1/entities/{process_entity_id}/attrs',
                headers=self.headers,
                json=attrs_payload,
            )
            troe_ok = 200 <= patch_resp.status_code < 300
        except Exception:
            troe_ok = False

        final_result = created and troe_ok
        return final_result

    def create_ai_recommendation(
        self, process_id: str, params: Dict[str, Any]
    ):
        """Store AI model recommendations"""
        entity = {
            'id': f'urn:ngsi-ld:AIRecommendation:{process_id}-{int(time.time())}',
            'type': 'urn:robin:AIRecommendation',
            'recommendedParams': {'type': 'Property', 'value': params},
            'confidence': {
                'type': 'Property',
                'value': params.get('confidence', 0.95),
            },
            'modelVersion': {
                'type': 'Property',
                'value': params.get('model_version', 'v1.0'),
            },
            'timestamp': {
                'type': 'Property',
                'value': datetime.now(timezone.utc).isoformat(),
            },
            'processId': {
                'type': 'Relationship',
                'object': f'{self.PROCESS_ENTITY_ID_PREFIX}{process_id}',
            },
        }

        response = requests.post(
            f'{self.orion_url}/ngsi-ld/v1/entities',
            headers=self.headers,
            json=entity,
        )
        return response.status_code in (200, 201)

    def create_alert(self, alert_data):
        """Create alert entity in Orion"""
        entity = {
            'id': f'urn:ngsi-ld:Alert:{alert_data.process_id}-{int(time.time())}',
            'type': 'urn:robin:Alert',
            'processId': {
                'type': 'Relationship',
                'object': f'{self.PROCESS_ENTITY_ID_PREFIX}{alert_data.process_id}',
            },
            'deviationType': {
                'type': 'Property',
                'value': alert_data.deviation_type,
            },
            'expectedValue': {
                'type': 'Property',
                'value': alert_data.expected_value,
            },
            'measuredValue': {
                'type': 'Property',
                'value': alert_data.measured_value,
            },
            'deviationPercentage': {
                'type': 'Property',
                'value': alert_data.deviation_percentage,
                'unitCode': 'P1',
            },
            'recommendedActions': {
                'type': 'Property',
                'value': alert_data.recommended_actions,
            },
            'timestamp': {
                'type': 'Property',
                'value': datetime.now(timezone.utc).isoformat(),
            },
        }

        response = requests.post(
            f'{self.orion_url}/ngsi-ld/v1/entities',
            headers=self.headers,
            json=entity,
        )
        return response.status_code in (200, 201)


# CLI Commands
@app.command()
def create_process(
    process_id: str = typer.Argument(..., help='Unique process identifier'),
    mode: OperationMode = typer.Option(
        OperationMode.PARAMETER_DRIVEN, help='Operation mode'
    ),
    orion_url: str = typer.Option(ORION, help='Orion Context Broker URL'),
):
    """Create a new process"""
    client = RobinFiwareClient(orion_url)
    success = client.create_process(process_id, mode)
    if success:
        typer.echo(
            f'✅ Created process: {process_id} (mode: {mode.value})'
        )
    else:
        typer.echo(
            f'❌ Failed to create process: {process_id}', err=True
        )
        raise typer.Exit(1)


@app.command()
def create_target(
    process_id: str = typer.Argument(..., help='Process identifier'),
    height: float = typer.Argument(..., help='Target height in mm'),
    width: float = typer.Argument(..., help='Target width in mm'),
    orion_url: str = typer.Option(ORION, help='Orion Context Broker URL'),
):
    """Create geometry target for a process"""
    client = RobinFiwareClient(orion_url)
    success = client.create_geometry_target(process_id, height, width)
    if success:
        typer.echo(
            f'✅ Created geometry target for process {process_id}: {height}x{width}mm'
        )
    else:
        # Target may already exist; continue to enforce geometry-driven mode
        typer.echo(
            f'⚠️  Geometry target not created (may already exist) for process {process_id}',
            err=False,
        )

    # Ensure operationMode is set to geometry_driven whenever a target is set/updated
    mode_ok = client.set_operation_mode(
        process_id, OperationMode.GEOMETRY_DRIVEN
    )
    if mode_ok:
        typer.echo(
            f'✅ Set operation mode to geometry_driven for process {process_id}'
        )
    else:
        typer.echo(
            f'❌ Failed to set operation mode to geometry_driven for process: {process_id}',
            err=True,
        )
        raise typer.Exit(1)


@app.command()
def add_measurement(
    process_id: str = typer.Argument(..., help='Process identifier'),
    measurement_id: str = typer.Argument(
        ..., help='Unique measurement identifier'
    ),
    height: float = typer.Argument(..., help='Measured height in mm'),
    width: float = typer.Argument(..., help='Measured width in mm'),
    speed: float = typer.Option(None, help='Measured travel speed in mm/s'),
    current: float = typer.Option(None, help='Measured current in A'),
    voltage: float = typer.Option(None, help='Measured voltage in V'),
    orion_url: str = typer.Option(ORION, help='Orion Context Broker URL'),
):
    """Add a measurement for a process with optional machine parameters"""
    client = RobinFiwareClient(orion_url)
    success = client.create_measurement(
        process_id, measurement_id, height, width, speed, current, voltage
    )
    if success:
        params_str = f'{height}x{width}mm'
        if speed is not None or current is not None or voltage is not None:
            param_parts = []
            if speed is not None:
                param_parts.append(f'speed={speed}mm/s')
            if current is not None:
                param_parts.append(f'current={current}A')
            if voltage is not None:
                param_parts.append(f'voltage={voltage}V')
            params_str += f' ({", ".join(param_parts)})'

        typer.echo(
            f'✅ Added measurement {measurement_id} for process {process_id}: {params_str}'
        )
    else:
        typer.echo(
            f'❌ Failed to add measurement {measurement_id} for process: {process_id}',
            err=True,
        )
        raise typer.Exit(1)


@app.command()
def add_recommendation(
    process_id: str = typer.Argument(..., help='Process identifier'),
    params: str = typer.Argument(
        ..., help='JSON string of recommended parameters'
    ),
    orion_url: str = typer.Option(ORION, help='Orion Context Broker URL'),
):
    """Add AI recommendation for a process"""
    try:
        params_dict = json.loads(params)
    except json.JSONDecodeError:
        typer.echo('❌ Invalid JSON format for parameters', err=True)
        raise typer.Exit(1)

    client = RobinFiwareClient(orion_url)
    success = client.create_ai_recommendation(process_id, params_dict)
    if success:
        typer.echo(f'✅ Added AI recommendation for process {process_id}')
    else:
        typer.echo(
            f'❌ Failed to add AI recommendation for process: {process_id}',
            err=True,
        )
        raise typer.Exit(1)


@app.command()
def stop_process(
    process_id: str = typer.Argument(..., help='Process identifier to stop'),
    reason: str = typer.Option(
        'operator_request', help='Reason for stopping the process'
    ),
    orion_url: str = typer.Option(ORION, help='Orion Context Broker URL'),
):
    """Stop a running process"""
    client = RobinFiwareClient(orion_url)
    success, message = client.stop_process(process_id, reason)
    if success:
        typer.echo(f'✅ {message}')
    else:
        typer.echo(f'❌ {message}', err=True)
        raise typer.Exit(1)


@app.command()
def resume_process(
    process_id: str = typer.Argument(..., help='Process identifier to resume'),
    orion_url: str = typer.Option(ORION, help='Orion Context Broker URL'),
):
    """Resume a stopped process"""
    client = RobinFiwareClient(orion_url)
    success, message = client.resume_process(process_id)
    if success:
        typer.echo(f'✅ {message}')
    else:
        typer.echo(f'❌ {message}', err=True)
        raise typer.Exit(1)


@app.command()
def process_status(
    process_id: str = typer.Argument(..., help='Process identifier to check'),
    orion_url: str = typer.Option(ORION, help='Orion Context Broker URL'),
):
    """Get the current status of a process"""
    client = RobinFiwareClient(orion_url)
    status_info, error = client.get_process_status(process_id)

    if error:
        typer.echo(f'❌ {error}', err=True)
        raise typer.Exit(1)

    # Format status display
    typer.echo(f'📊 Process Status: {process_id}')
    typer.echo('=' * 40)
    typer.echo(f'Status: {status_info["status"]}')
    typer.echo(f'Mode: {status_info["operation_mode"]}')
    typer.echo(f'Started: {status_info["started_at"]}')

    if status_info['stopped_at']:
        typer.echo(f'Stopped: {status_info["stopped_at"]}')
        typer.echo(f'Stop Reason: {status_info["stop_reason"]}')

    telemetry = status_info.get('telemetry')
    if isinstance(telemetry, dict) and telemetry:
        typer.echo('\n--- Latest Telemetry ---')
        try:
            if telemetry.get('height') is not None:
                typer.echo(f'  Height : {float(telemetry["height"]):.2f} mm')
            if telemetry.get('width') is not None:
                typer.echo(f'  Width  : {float(telemetry["width"]):.2f} mm')
            if telemetry.get('speed') is not None:
                typer.echo(f'  Speed  : {float(telemetry["speed"]):.2f} mm/s')
            if telemetry.get('current') is not None:
                typer.echo(f'  Current: {float(telemetry["current"]):.2f} A')
            if telemetry.get('voltage') is not None:
                typer.echo(f'  Voltage: {float(telemetry["voltage"]):.2f} V')
        except Exception:
            typer.echo('Telemetry attribute exists, but is not yet populated.')
    else:
        typer.echo('\n--- No Telemetry Received ---')

    # Add status emoji
    status_emoji = {
        'active': '🟢',
        'stopped': '🔴',
        'paused': '🟡',
        'error': '❌',
    }.get(status_info['status'], '❓')

    typer.echo(
        f'\n{status_emoji} Process is currently {status_info["status"].upper()}'
    )


@app.command()
def list_processes(
    orion_url: str = typer.Option(ORION, help='Orion Context Broker URL'),
    status_filter: str = typer.Option(
        None, help='Filter by status (active, stopped, paused, error)'
    ),
):
    """List all processes with their status"""
    try:
        response = requests.get(
            f'{orion_url}/ngsi-ld/v1/entities?type=urn:robin:Process',
            headers={'NGSILD-Tenant': TENANT},
            timeout=5,
        )

        if response.status_code != 200:
            typer.echo('❌ Failed to fetch processes from Orion', err=True)
            raise typer.Exit(1)

        processes = response.json()

        if not processes:
            typer.echo('📭 No processes found')
            return

        # Filter by status if requested
        if status_filter:
            processes = [
                p
                for p in processes
                if p.get('processStatus', {}).get('value') == status_filter
            ]
            if not processes:
                typer.echo(
                    f'📭 No processes found with status: {status_filter}'
                )
                return

        typer.echo('📋 Processes:')
        typer.echo('=' * 80)

        for process in processes:
            process_id = process['id'].split(':')[-1]  # Extract ID from URN
            status = process.get('processStatus', {}).get('value', 'unknown')
            mode = process.get('operationMode', {}).get('value', 'unknown')
            started = process.get('startedAt', {}).get('value', 'unknown')

            status_emoji = {
                'active': '🟢',
                'stopped': '🔴',
                'paused': '🟡',
                'error': '❌',
            }.get(status, '❓')

            typer.echo(
                f'{status_emoji} {process_id:<20} | {status:<10} | {mode:<20} | {started}'
            )

    except requests.RequestException:
        typer.echo('❌ Failed to connect to Orion Context Broker', err=True)
        raise typer.Exit(1)


@app.command()
def status():
    """Check system status"""
    typer.echo('🤖 ROBIN - Alert Processor')
    typer.echo(f'📡 Orion URL: {ORION}')
    typer.echo(f'📊 Mintaka URL: {MTK}')
    typer.echo(f'🏢 Tenant: {TENANT}')

    # Test connectivity
    try:
        response = requests.get(f'{ORION}/version', timeout=5)
        if response.status_code == 200:
            typer.echo('✅ Orion Context Broker: Connected')
        else:
            typer.echo('❌ Orion Context Broker: Not responding')
    except requests.RequestException:
        typer.echo('❌ Orion Context Broker: Connection failed')

    # Probe Mintaka /health (Micronaut default)
    mintaka_connected = False
    for attempt in range(3):
        try:
            import time as _time

            response = requests.get(f"{MTK.rstrip('/')}/health", timeout=3)
            if response.status_code == 200:
                mintaka_connected = True
                break
        except requests.RequestException:
            if attempt < 2:
                _time.sleep(1)

    if mintaka_connected:
        typer.echo('✅ Mintaka: Connected')
    else:
        typer.echo('❌ Mintaka: Connection failed')


@app.command()
def serve(
    host: str = typer.Option('0.0.0.0', help='Host to bind to'),
    port: int = typer.Option(8000, help='Port to bind to'),
):
    """Start the alert processing service"""
    typer.echo(f'🚀 Starting ROBIN Alert Processor on {host}:{port}')

    # Import here to avoid circular imports and ensure FastAPI is available
    try:
        import uvicorn
        from robin.alert_engine import app as fastapi_app

        uvicorn.run(fastapi_app, host=host, port=port)
    except ImportError as e:
        typer.echo(f'❌ Failed to start server: {e}', err=True)
        raise typer.Exit(1)


if __name__ == '__main__':
    app()
