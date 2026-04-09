# robin/alert_engine.py
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Any, Dict, Optional, List, Sequence, Tuple
import os
import requests
from datetime import datetime, timezone
from pathlib import Path
import time
import torch
from robin.cli import RobinFiwareClient, OperationMode
from robin.ai import (
    ForwardConfidenceConfig,
    ForwardConfidenceEstimator,
    GeometryInverseOptimizer,
    InverseOptimizationConfig,
    ProcessGeometryMLP,
    load_model,
)
from robin.profile_loader import load_profile


ROOT_DIR = Path(__file__).resolve().parents[1]
APP_STARTED_AT = datetime.now(timezone.utc)
DEFAULT_MODEL_PATH = (
    Path(os.getenv('ROBIN_MLP_MODEL_PATH', ''))
    if os.getenv('ROBIN_MLP_MODEL_PATH')
    else ROOT_DIR / 'data' / 'models' / 'process_geometry_mlp.pt'
)

active_profile = load_profile()
MLP_FEATURE_ORDER = tuple(active_profile.feature_order)

app = FastAPI(
    title='ROBIN Alert Engine',
    description='AI-driven alert processing system for robotic processes',
    version='1.0.0',
)

# Add CORS middleware for dashboard
app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=['*'],
    allow_headers=['*'],
)


@app.get('/')
async def root():
    """Root endpoint with system information"""
    return {
        'name': 'ROBIN Alert Engine',
        'version': '1.0.0',
        'description': 'AI-driven alert processing system for robotic processes',
        'status': 'running',
        'endpoints': {
            'dashboard': '/dashboard',
            'docs': '/docs',
            'create_process': '/create-process',
            'process_data': '/process/{process_id}',
            'process_alerts': '/process/{process_id}/alerts',
            'list_processes': '/processes',
            'set_process_mode': '/process/{process_id}/mode',
            'set_process_input_params': '/process/{process_id}/input-params',
            'check_deviation': '/check-deviation',
            'operator_action': '/operator-action',
            'ai_recommendation': '/ai-recommendation',
            'ai_models': '/ai/models',
            'ai_model_select': '/ai/models/select',
            'ai_model_predict': '/ai/models/predict',
            'health': '/health',
            'profile': '/profile',
        },
    }


@app.get('/profile')
async def get_profile():
    """Return the active profile configuration (vocabulary, fields, skills, etc.)."""
    return active_profile.as_dict()


class PublishIntentRequest(BaseModel):
    intent: str
    process_id: str = 'ros_bridge'
    data: Dict[str, Any] = {}


@app.post('/intent')
async def publish_intent(request: PublishIntentRequest):
    """Persist operator intent in Orion-LD.

    Orion-LD delivers the intent to the ROS pipeline via its NGSI-LD subscription
    mechanism: PATCH pendingIntent → Orion-LD subscription notification →
    welding_http_bridge (/orion-notify) → /intents ROS2 topic.
    """
    client = RobinFiwareClient()
    try:
        client.patch_process_intent(request.process_id, request.intent, request.data)
    except Exception:
        pass  # Never block the response if Orion-LD is unavailable
    return {'status': 'published', 'intent': request.intent, 'process_id': request.process_id}


@app.get('/health')
async def health_check():
    """Health check endpoint with per-service connectivity and latency."""

    def probe(url: str, ok_statuses: Sequence[int] = (200,)) -> Dict[str, Any]:
        started = time.perf_counter()
        try:
            response = requests.get(url, timeout=5)
            latency_ms = int((time.perf_counter() - started) * 1000)
            return {
                'connected': response.status_code in ok_statuses,
                'status_code': response.status_code,
                'latency_ms': latency_ms,
                'error': None,
            }
        except Exception as exc:
            return {
                'connected': False,
                'status_code': None,
                'latency_ms': None,
                'error': str(exc),
            }

    started = time.perf_counter()
    client = RobinFiwareClient()
    orion_probe = probe(f'{client.orion_url}/version', ok_statuses=(200,))

    mintaka_url = os.getenv('MINTAKA_URL', 'http://mintaka:8080').rstrip('/')
    # Mintaka temporal endpoints require time-relation query params and can
    # return 400 on syntactically valid but incomplete requests. Probe the
    # service health endpoint instead to represent connectivity accurately.
    mintaka_probe = probe(f'{mintaka_url}/health', ok_statuses=(200,))

    backend_latency_ms = int((time.perf_counter() - started) * 1000)
    now = datetime.now(timezone.utc)
    uptime_seconds = int((now - APP_STARTED_AT).total_seconds())

    healthy = orion_probe['connected'] and mintaka_probe['connected']
    status = 'healthy' if healthy else 'unhealthy'

    response: Dict[str, Any] = {
        'status': status,
        'orion_connected': bool(orion_probe['connected']),
        'mintaka_connected': bool(mintaka_probe['connected']),
        'services': {
            'backend': {
                'connected': True,
                'latency_ms': backend_latency_ms,
            },
            'orion': {
                'connected': bool(orion_probe['connected']),
                'status_code': orion_probe['status_code'],
                'latency_ms': orion_probe['latency_ms'],
            },
            'mintaka': {
                'connected': bool(mintaka_probe['connected']),
                'status_code': mintaka_probe['status_code'],
                'latency_ms': mintaka_probe['latency_ms'],
            },
        },
        'latency_ms': {
            'backend': backend_latency_ms,
            'orion': orion_probe['latency_ms'],
            'mintaka': mintaka_probe['latency_ms'],
        },
        'uptime_seconds': uptime_seconds,
        'timestamp': now.isoformat(),
    }

    errors = []
    if orion_probe['error']:
        errors.append(f'orion: {orion_probe["error"]}')
    if mintaka_probe['error']:
        errors.append(f'mintaka: {mintaka_probe["error"]}')
    if errors:
        response['error'] = '; '.join(errors)

    return response


@app.get('/dashboard', response_class=HTMLResponse)
async def dashboard():
    """Serve the HTML dashboard"""
    try:
        with open('/app/dashboard.html', 'r') as f:
            return HTMLResponse(content=f.read())
    except FileNotFoundError:
        return HTMLResponse(
            content="""
        <html>
            <head><title>ROBIN Dashboard - Not Found</title></head>
            <body>
                <h1>Dashboard Not Available</h1>
                <p>The dashboard.html file was not found.</p>
                <p><a href="/docs">View API Documentation</a></p>
            </body>
        </html>
        """,
            status_code=404,
        )


class AlertConfig(BaseModel):
    tolerance_threshold: float = 10.0  # Percentage
    mode: str = 'parameter_driven'


class DeviationCheckRequest(BaseModel):
    process_id: str
    mode: Optional[
        str
    ] = None  # 'parameter_driven' or 'geometry_driven' (optional)
    input_params: Optional[
        Dict[str, float]
    ] = None  # For parameter_driven mode
    measured_geometry: Optional[
        Dict[str, float]
    ] = None  # Latest will be used if omitted
    tolerance: Optional[
        float
    ] = None  # Will use default from config if not provided


class DeviationAlert(BaseModel):
    process_id: str
    deviation_type: str  # "height" or "width"
    expected_value: Dict[str, float]
    measured_value: Dict[str, float]
    deviation_percentage: float
    recommended_actions: list


class AIRecommendationRequest(BaseModel):
    process_id: str
    mode: str  # 'parameter_driven' or 'geometry_driven'
    input_params: Optional[Dict[str, float]] = None
    target_geometry: Optional[Dict[str, float]] = None


class CreateProcessRequest(BaseModel):
    process_id: str
    mode: str = active_profile.default_mode  # 'parameter_driven' or 'geometry_driven'
    initial_params: Optional[Dict[str, float]] = None
    target_geometry: Optional[Dict[str, float]] = None


class SimulationProgressRequest(BaseModel):
    progress: float  # 0-100
    expected_duration: Optional[float] = None  # seconds


class SetTargetRequest(BaseModel):
    height: float
    width: float


class SetModeRequest(BaseModel):
    mode: str  # 'parameter_driven' or 'geometry_driven'


class SetInputParamsRequest(BaseModel):
    input_params: Dict[str, float]


class AIModelSelectionRequest(BaseModel):
    path: str


class AIModelPredictRequest(BaseModel):
    input_params: Optional[Dict[str, float]] = None


class AlertEngine:
    def __init__(self):
        self.client = RobinFiwareClient()
        self.model_dirs = self._init_model_directories()
        self.feature_order: Sequence[str] = MLP_FEATURE_ORDER
        self.active_model_path: Optional[Path] = None
        self.ai_model: Optional[ProcessGeometryMLP] = None
        self.forward_confidence_config = ForwardConfidenceConfig.from_dict(
            getattr(active_profile, 'forward_confidence', {}),
        )
        self.forward_confidence_estimator = ForwardConfidenceEstimator(
            self.forward_confidence_config
        )
        self.inverse_optimizer: Optional[GeometryInverseOptimizer] = None
        self.inverse_bounds: Dict[str, Tuple[float, float]] = {}
        self.inverse_config = InverseOptimizationConfig.from_dict(
            getattr(active_profile, 'inverse_optimizer', {}),
        )
        self.default_tolerance = active_profile.default_tolerance
        # Cache for GeometryTarget "not found" results to avoid hitting Orion-LD
        # on every dashboard poll when the entity doesn't exist yet.
        # Maps process_id → (result, expiry_timestamp)
        self._geo_target_cache: Dict[str, Tuple[Optional[Dict[str, float]], float]] = {}

        profile_model = None
        if active_profile.model_path:
            profile_model = Path(active_profile.model_path)
            if not profile_model.is_absolute():
                profile_model = ROOT_DIR / profile_model
        self._profile_model_path = profile_model

        self._load_ai_model()

    def _init_model_directories(self) -> List[Path]:
        directories: List[Path] = []

        env_dirs = os.getenv('ROBIN_MLP_MODEL_DIRS')
        if env_dirs:
            for entry in env_dirs.split(os.pathsep):
                entry = entry.strip()
                if entry:
                    directories.append(Path(entry).expanduser())

        directories.extend(
            [
                ROOT_DIR / 'data' / 'models',
                Path(__file__).resolve().parent / 'models',
            ]
        )

        unique: List[Path] = []
        seen: set[str] = set()
        for directory in directories:
            expanded = Path(directory).expanduser()
            key = str(expanded.resolve(strict=False))
            if key in seen:
                continue
            expanded.mkdir(parents=True, exist_ok=True)
            unique.append(expanded)
            seen.add(key)
        return unique

    def _candidate_model_paths(
        self, override: Optional[Path] = None
    ) -> List[Path]:
        candidates: List[Path] = []

        for path in [override, self._profile_model_path, self.active_model_path]:
            if path:
                candidates.append(Path(path))

        env_path = os.getenv('ROBIN_MLP_MODEL_PATH')
        if env_path:
            candidates.append(Path(env_path).expanduser())

        candidates.append(DEFAULT_MODEL_PATH)

        for directory in self.model_dirs:
            default_candidate = directory / DEFAULT_MODEL_PATH.name
            candidates.append(default_candidate)
            for checkpoint in sorted(directory.rglob('*.pt')):
                candidates.append(checkpoint)

        unique: List[Path] = []
        seen: set[str] = set()
        for candidate in candidates:
            if not candidate:
                continue
            expanded = Path(candidate).expanduser()
            key = str(expanded)
            if key in seen:
                continue
            unique.append(expanded)
            seen.add(key)
        return unique

    @staticmethod
    def _default_inverse_bound(feature: str) -> Tuple[float, float]:
        defaults: Dict[str, Tuple[float, float]] = {
            'wire_feed_speed_mpm_model_input': (1.0, 30.0),
            'travel_speed_mps_model_input': (0.001, 1.0),
            'arc_length_correction_mm_model_input': (-20.0, 20.0),
        }
        return defaults.get(feature, (0.0, 1.0))

    def input_feature_specs(self) -> List[Dict[str, Any]]:
        raw_specs = getattr(active_profile, 'ai_input_features', [])
        by_key: Dict[str, Dict[str, Any]] = {}
        for raw_spec in raw_specs:
            key = raw_spec.get('key')
            if isinstance(key, str) and key:
                by_key[key] = dict(raw_spec)

        specs: List[Dict[str, Any]] = []
        for feature in self.feature_order:
            spec = dict(by_key.get(feature, {}))
            label = spec.get('label')
            unit = spec.get('unit')
            aliases = spec.get('aliases')
            default = spec.get('default')
            step = spec.get('step')

            if not isinstance(label, str) or not label:
                label = self._default_feature_label(feature)
            if not isinstance(unit, str):
                unit = ''
            if not isinstance(aliases, list):
                aliases = []
            aliases = [str(alias) for alias in aliases if str(alias).strip()]
            aliases = [feature, *[alias for alias in aliases if alias != feature]]

            entry: Dict[str, Any] = {
                'key': feature,
                'label': label,
                'unit': unit,
                'aliases': aliases,
            }
            if isinstance(default, (int, float)):
                entry['default'] = float(default)
            if isinstance(step, (int, float)):
                entry['step'] = float(step)
            specs.append(entry)
        return specs

    @staticmethod
    def _default_feature_label(feature: str) -> str:
        defaults = {
            'wire_feed_speed_mpm_model_input': 'Wire Feed Speed',
            'travel_speed_mps_model_input': 'Travel Speed',
            'arc_length_correction_mm_model_input': 'Arc Length Correction',
        }
        return defaults.get(feature, feature)

    def _canonicalize_input_params(
        self, params: Optional[Dict[str, float]]
    ) -> Dict[str, float]:
        if not params:
            return {}

        numeric_params: Dict[str, float] = {}
        for key, value in params.items():
            if isinstance(value, bool):
                continue
            if isinstance(value, (int, float)):
                numeric_params[key] = float(value)

        canonical: Dict[str, float] = {}
        for spec in self.input_feature_specs():
            key = str(spec['key'])
            aliases = spec.get('aliases', [])
            for alias in aliases:
                if alias in numeric_params:
                    canonical[key] = numeric_params[alias]
                    break
        for feature in self.feature_order:
            if feature in numeric_params:
                canonical[feature] = numeric_params[feature]
        return canonical

    def _require_complete_input_params(
        self, params: Optional[Dict[str, float]]
    ) -> Dict[str, float]:
        canonical = self._canonicalize_input_params(params)
        missing = [
            feature for feature in self.feature_order if feature not in canonical
        ]
        if missing:
            raise HTTPException(
                status_code=422,
                detail={
                    'message': 'Missing required AI input features',
                    'missing_features': missing,
                    'feature_order': list(self.feature_order),
                    'recognized_input_params': canonical,
                },
            )
        return canonical

    def _default_feature_value(self, feature: str) -> float:
        for spec in self.input_feature_specs():
            if spec.get('key') == feature:
                default = spec.get('default')
                if isinstance(default, (int, float)):
                    return float(default)
                break
        return 0.0

    @staticmethod
    def _coerce_bound(
        raw_value: Any, default: Tuple[float, float]
    ) -> Tuple[float, float]:
        low, high = default

        if isinstance(raw_value, dict):
            min_value = raw_value.get('min')
            max_value = raw_value.get('max')
            if isinstance(min_value, (int, float)) and isinstance(
                max_value, (int, float)
            ):
                low, high = float(min_value), float(max_value)
        elif (
            isinstance(raw_value, (list, tuple))
            and len(raw_value) == 2
            and isinstance(raw_value[0], (int, float))
            and isinstance(raw_value[1], (int, float))
        ):
            low, high = float(raw_value[0]), float(raw_value[1])

        if high < low:
            low, high = high, low
        if low == high:
            high = low + 1.0
        return low, high

    def _resolve_inverse_bounds(self) -> Dict[str, Tuple[float, float]]:
        raw_bounds = getattr(active_profile, 'inverse_bounds', {})
        if not isinstance(raw_bounds, dict):
            raw_bounds = {}

        bounds: Dict[str, Tuple[float, float]] = {}
        for feature in self.feature_order:
            bounds[feature] = self._coerce_bound(
                raw_bounds.get(feature),
                self._default_inverse_bound(feature),
            )
        return bounds

    def _refresh_inverse_optimizer(self) -> None:
        self.inverse_config = InverseOptimizationConfig.from_dict(
            getattr(active_profile, 'inverse_optimizer', {}),
        )
        self.inverse_bounds = self._resolve_inverse_bounds()
        self.inverse_optimizer = GeometryInverseOptimizer(
            feature_order=self.feature_order,
            parameter_bounds=self.inverse_bounds,
            predict_geometry=self.predict_geometry_from_params,
            config=self.inverse_config,
        )

    def _load_ai_model(
        self, override: Optional[Path] = None
    ) -> Optional[ProcessGeometryMLP]:
        """Attempt to load a model, updating internal state on success."""
        for candidate in self._candidate_model_paths(override):
            if not candidate.is_file():
                continue
            try:
                model = load_model(candidate)
                self.ai_model = model
                self.active_model_path = candidate.resolve()
                if model.config.feature_names:
                    self.feature_order = tuple(model.config.feature_names)
                self._refresh_inverse_optimizer()
                return model
            except Exception:
                continue

        self.ai_model = None
        self._refresh_inverse_optimizer()
        return None

    def load_model_from_path(self, path: Path) -> ProcessGeometryMLP:
        """Load a specific checkpoint or raise an informative error."""
        resolved = Path(path).expanduser()
        if not resolved.is_file():
            raise FileNotFoundError(f'Checkpoint not found at {resolved}')

        model = load_model(resolved)
        self.ai_model = model
        self.active_model_path = resolved.resolve()
        if model.config.feature_names:
            self.feature_order = tuple(model.config.feature_names)
        else:
            self.feature_order = MLP_FEATURE_ORDER
        self._refresh_inverse_optimizer()
        return model

    def _model_metadata(self, path: Path) -> Dict[str, Any]:
        stat = path.stat()
        metadata: Dict[str, Any] = {
            'name': path.name,
            'path': str(path),
            'directory': str(path.parent),
            'size_bytes': stat.st_size,
            'modified_at': datetime.fromtimestamp(
                stat.st_mtime, timezone.utc
            ).isoformat(),
            'is_active': False,
        }

        if self.active_model_path and path.exists():
            try:
                metadata['is_active'] = path.samefile(self.active_model_path)
            except FileNotFoundError:
                metadata['is_active'] = False

        try:
            checkpoint = torch.load(path, map_location='cpu')
            config = checkpoint.get('config')
            if isinstance(config, dict):
                metadata['config'] = config
            else:
                metadata['config'] = {
                    'schema': checkpoint.get('schema'),
                    'feature_names': checkpoint.get('feature_columns')
                    or checkpoint.get('feature_names'),
                    'hidden_dims': checkpoint.get('hidden_dims'),
                    'input_dim': checkpoint.get('input_dim'),
                    'output_dim': checkpoint.get('output_dim'),
                }
        except Exception as exc:
            metadata['config_error'] = str(exc)

        return metadata

    def list_available_models(self) -> List[Dict[str, Any]]:
        """Return metadata for all checkpoints discovered in model directories."""
        models: List[Dict[str, Any]] = []
        for directory in self.model_dirs:
            if not directory.exists():
                continue
            for checkpoint in sorted(directory.rglob('*.pt')):
                models.append(self._model_metadata(checkpoint))
        return models

    def active_model_info(self) -> Optional[Dict[str, Any]]:
        if not self.active_model_path or not self.active_model_path.exists():
            return None
        return self._model_metadata(self.active_model_path)

    def _build_feature_vector(self, params: Dict[str, float]) -> List[float]:
        canonical = self._canonicalize_input_params(params)
        return [
            float(canonical.get(key, self._default_feature_value(key)))
            for key in self.feature_order
        ]

    def calculate_deviation(self, expected: float, measured: float) -> float:
        """Calculate percentage deviation"""
        if expected == 0:
            return 100.0
        return abs((measured - expected) / expected) * 100

    def check_parameter_driven_deviation(
        self,
        process_id: str,
        input_params: Dict[str, float],
        measured_geometry: Dict[str, float],
        tolerance: float,
    ) -> Tuple[Optional[DeviationAlert], Dict[str, float]]:
        """
        Parameter-Driven Mode: Compare AI prediction with measurement
        """
        # Get AI prediction based on input parameters
        predicted_geometry = self.predict_geometry_from_params(input_params)

        # Check height deviation
        height_dev = self.calculate_deviation(
            predicted_geometry['height'], measured_geometry['height']
        )

        # Check width deviation
        width_dev = self.calculate_deviation(
            predicted_geometry['width'], measured_geometry['width']
        )

        alert = None
        if height_dev > tolerance or width_dev > tolerance:
            alert = DeviationAlert(
                process_id=process_id,
                deviation_type='geometry',
                expected_value=predicted_geometry,
                measured_value=measured_geometry,
                deviation_percentage=max(height_dev, width_dev),
                recommended_actions=self.get_recommended_actions(),
            )
        return alert, predicted_geometry

    def check_geometry_driven_deviation(
        self,
        process_id: str,
        target_geometry: Dict[str, float],
        measured_geometry: Dict[str, float],
        tolerance: float,
    ) -> Tuple[Optional[DeviationAlert], Dict[str, float]]:
        """
        Geometry-Driven Mode: Compare target with measurement
        """
        height_dev = self.calculate_deviation(
            target_geometry['height'], measured_geometry['height']
        )

        width_dev = self.calculate_deviation(
            target_geometry['width'], measured_geometry['width']
        )

        alert = None
        if height_dev > tolerance or width_dev > tolerance:
            alert = DeviationAlert(
                process_id=process_id,
                deviation_type='geometry',
                expected_value=target_geometry,
                measured_value=measured_geometry,
                deviation_percentage=max(height_dev, width_dev),
                recommended_actions=self.get_recommended_actions(),
            )
        return alert, target_geometry

    def get_recommended_actions(self) -> list:
        """Return unified post-alert actions"""
        return [
            {
                'id': 'manual_adjust',
                'label': 'Manually adjust process parameters',
            },
            {'id': 'ai_retry', 'label': 'Request new AI recommendation'},
            {'id': 'fine_tune', 'label': 'Add datapoint and fine-tune model'},
            {'id': 'reset_model', 'label': 'Discard model and start new DOE'},
        ]

    def predict_geometry_from_params(
        self, params: Dict[str, float]
    ) -> Dict[str, float]:
        """Predict geometry from process parameters using the active model."""
        scored = self.predict_geometry_with_confidence(params)
        return scored['prediction']

    def predict_geometry_with_confidence(
        self, params: Dict[str, float]
    ) -> Dict[str, Any]:
        """Predict geometry and attach dynamic confidence diagnostics."""
        if self.ai_model is None:
            raise HTTPException(status_code=500, detail='AI model not loaded')

        canonical_params = self._require_complete_input_params(params)
        features = self._build_feature_vector(canonical_params)
        result = self.forward_confidence_estimator.estimate(
            self.ai_model, features
        )
        height, width = result.prediction
        prediction = {
            'height': float(max(height, 0.0)),
            'width': float(max(width, 0.0)),
        }
        print(
            '🔍 AI model predictions: '
            f"{prediction['height']}, {prediction['width']} "
            f'(confidence={result.confidence:.3f})'
        )
        return {
            'prediction': prediction,
            'confidence': float(result.confidence),
            'uncertainty': {
                'height_std': float(result.output_std[0]),
                'width_std': float(result.output_std[1]),
                'relative': float(result.relative_uncertainty),
            },
            'diagnostics': {
                'input_distance': float(result.input_distance),
                'mc_samples': int(result.mc_samples),
            },
            'input_params': canonical_params,
        }

    def recommend_params_for_geometry(
        self,
        target_geometry: Dict[str, float],
        current_params: Optional[Dict[str, float]] = None,
    ) -> Dict[str, Any]:
        """Solve inverse mapping with constrained optimization over the forward model."""
        if self.inverse_optimizer is None:
            self._refresh_inverse_optimizer()
        if self.inverse_optimizer is None:
            raise HTTPException(
                status_code=500, detail='Inverse optimizer unavailable'
            )

        result = self.inverse_optimizer.solve(
            target_geometry=target_geometry,
            current_params=self._canonicalize_input_params(current_params),
        )
        recommended_params: Dict[str, Any] = {
            feature: float(result.params.get(feature, 0.0))
            for feature in self.feature_order
        }
        recommended_params['confidence'] = float(result.confidence)
        recommended_params['predictedHeight'] = float(
            result.predicted_geometry['height']
        )
        recommended_params['predictedWidth'] = float(
            result.predicted_geometry['width']
        )
        recommended_params['optimization'] = {
            'objective': float(result.objective),
            'geometry_loss': float(result.geometry_loss),
            'iterations': int(result.iterations),
            'restarts': int(result.restarts),
        }
        return recommended_params

    def fetch_process_data(self, process_id: str) -> Dict:
        """Fetch process data from FIWARE Orion"""
        try:
            response = requests.get(
                f'{self.client.orion_url}/ngsi-ld/v1/entities/urn:ngsi-ld:Process:{process_id}',
                headers=self.client.headers,
                timeout=5,
            )
            if response.status_code == 200:
                return response.json()
            return {}
        except Exception:
            return {}

    def fetch_geometry_target(
        self, process_id: str
    ) -> Optional[Dict[str, float]]:
        """Fetch geometry target from FIWARE Orion.

        Caches 'not found' (404) results for 30 s to avoid spamming Orion-LD
        when the entity doesn't exist (e.g. in simulation / parameter-driven mode).
        """
        now = time.time()
        cached_result, expiry = self._geo_target_cache.get(process_id, (None, 0.0))
        if expiry > now:
            return cached_result
        try:
            response = requests.get(
                f'{self.client.orion_url}/ngsi-ld/v1/entities/urn:ngsi-ld:GeometryTarget:{process_id}',
                headers=self.client.headers,
                timeout=5,
            )
            if response.status_code == 200:
                data = response.json()
                result: Optional[Dict[str, float]] = {
                    'height': data.get('targetHeight', {}).get('value', 0.0),
                    'width': data.get('targetWidth', {}).get('value', 0.0),
                }
                # Clear any negative cache entry on success
                self._geo_target_cache.pop(process_id, None)
                return result
            # Entity not found — cache the miss for 30 s
            self._geo_target_cache[process_id] = (None, now + 30.0)
            return None
        except Exception:
            return None

    def fetch_latest_measurement(
        self, process_id: str
    ) -> Optional[Dict[str, float]]:
        """Fetch the most recent measurement values for a process."""
        try:
            response = requests.get(
                f'{self.client.orion_url}/ngsi-ld/v1/entities',
                headers=self.client.headers,
                params={
                    'type': 'Measurement',
                    'q': f'processId=="urn:ngsi-ld:Process:{process_id}"',
                },
                timeout=5,
            )
            if response.status_code != 200:
                return None

            entities = (
                response.json() if isinstance(response.json(), list) else []
            )
            if not entities:
                return None

            def get_observed_at(ent: Dict) -> str:
                return ent.get('measuredHeight', {}).get('observedAt', '')

            entities.sort(key=get_observed_at, reverse=True)
            latest = entities[0]
            result = {
                'height': latest.get('measuredHeight', {}).get('value'),
                'width': latest.get('measuredWidth', {}).get('value'),
                'timestamp': latest.get('measuredHeight', {}).get(
                    'observedAt'
                ),
            }

            # Add machine parameters if available
            if 'measuredSpeed' in latest:
                result['speed'] = latest.get('measuredSpeed', {}).get('value')
            if 'measuredCurrent' in latest:
                result['current'] = latest.get('measuredCurrent', {}).get(
                    'value'
                )
            if 'measuredVoltage' in latest:
                result['voltage'] = latest.get('measuredVoltage', {}).get(
                    'value'
                )
            raw_input_params = latest.get('inputParams', {}).get('value')
            if isinstance(raw_input_params, dict):
                canonical_input_params = self._canonicalize_input_params(
                    raw_input_params
                )
                if canonical_input_params:
                    result['input_params'] = canonical_input_params

            return result
        except Exception:
            return None

    def fetch_all_measurements(
        self, process_id: str, last_n: int | None = None
    ) -> List[Dict]:
        """Fetch all measurement values for a process from Mintaka, sorted by timestamp.

        Uses TROE/Mintaka temporal API against the Process entity id and supports:
        - `urn:robin:processTelemetry` compound values (DDS path)
        - `measured*` scalar temporal properties (CLI/demo path)
        """
        try:
            mintaka_url = os.getenv(
                'MINTAKA_URL', 'http://mintaka:8080'
            ).rstrip('/')
            tenant = os.getenv('NGSILD_TENANT', '')
            temporal_attrs = (
                'urn:robin:processTelemetry,processTelemetry,'
                'measuredHeight,measuredWidth,measuredSpeed,'
                'measuredCurrent,measuredVoltage,inputParams'
            )

            def call_temporal(
                timeproperty: str, options_temporal_values: bool
            ) -> Optional[Dict]:
                # Required temporal window parameters for Mintaka
                # Use Zulu format to avoid offset parsing issues in Mintaka
                end_time = datetime.now(timezone.utc).strftime(
                    '%Y-%m-%dT%H:%M:%SZ'
                )
                params = {
                    'attrs': temporal_attrs,
                    'timeproperty': timeproperty,
                    'timerel': 'between',
                    'timeAt': '1970-01-01T00:00:00Z',
                    'endTimeAt': end_time,
                }
                if last_n and isinstance(last_n, int) and last_n > 0:
                    # Ask Mintaka for the most recent N samples
                    params['lastN'] = str(last_n)
                if options_temporal_values:
                    params['options'] = 'temporalValues'

                # Mintaka temporal API endpoint
                url = f'{mintaka_url}/temporal/entities/urn:ngsi-ld:Process:{process_id}'
                headers = {}
                # Use tenant header for proper multi-tenancy if configured
                if tenant:
                    headers['NGSILD-Tenant'] = tenant

                try:
                    resp = requests.get(
                        url, headers=headers, params=params, timeout=10
                    )
                    if resp.status_code not in (200, 206):
                        if resp.status_code != 404:
                            body_preview = (
                                resp.text[:500] if hasattr(resp, 'text') else ''
                            )
                            print(
                                f'❌ Mintaka query failed with status {resp.status_code}: {body_preview}'
                            )
                        return None
                    try:
                        data = resp.json()
                    except Exception as je:
                        print(f'❌ Failed to parse Mintaka JSON: {je}')
                        return None
                except Exception as ce:
                    print(f'❌ Mintaka request error: {ce}')
                    return None
                # Entity response for single id
                if isinstance(data, dict):
                    return data
                return None

            def to_float(value: Any) -> Optional[float]:
                if isinstance(value, bool):
                    return None
                if isinstance(value, (int, float)):
                    return float(value)
                return None

            def extract_values(attr_payload: Any) -> List[Any]:
                if isinstance(attr_payload, list):
                    return attr_payload
                if isinstance(attr_payload, dict):
                    vals = attr_payload.get('values', [])
                    if isinstance(vals, list):
                        return vals
                return []

            def temporal_pairs(attr_payload: Any) -> List[Tuple[str, Any]]:
                pairs: List[Tuple[str, Any]] = []
                for item in extract_values(attr_payload):
                    timestamp: Optional[str] = None
                    value: Any = None
                    if isinstance(item, list) and len(item) >= 2:
                        value = item[0]
                        timestamp = (
                            str(item[1]) if item[1] is not None else None
                        )
                    elif isinstance(item, dict):
                        timestamp = (
                            item.get('observedAt')
                            or item.get('modifiedAt')
                            or item.get('createdAt')
                        )
                        value = item.get('value')
                    if timestamp:
                        pairs.append((timestamp, value))
                return pairs

            def ensure_entry(
                series_by_timestamp: Dict[str, Dict[str, Any]],
                timestamp: str,
            ) -> Dict[str, Any]:
                entry = series_by_timestamp.get(timestamp)
                if entry is None:
                    entry = {
                        'timestamp': timestamp,
                        'height': None,
                        'width': None,
                        'speed': None,
                        'current': None,
                        'voltage': None,
                        'input_params': {},
                    }
                    series_by_timestamp[timestamp] = entry
                return entry

            def parse_input_params_value(value: Any) -> Dict[str, float]:
                if not isinstance(value, dict):
                    return {}
                return self._canonicalize_input_params(value)

            def parse_compound_value(compound_value: Any) -> Dict[str, Any]:
                if not isinstance(compound_value, dict):
                    return {}

                speed_value = (
                    compound_value.get('speed')
                    if compound_value.get('speed') is not None
                    else compound_value.get('wireSpeed')
                )
                if speed_value is None:
                    speed_value = compound_value.get('travelSpeed')

                return {
                    'height': to_float(
                        compound_value.get('height')
                        if compound_value.get('height') is not None
                        else compound_value.get('measuredHeight')
                    ),
                    'width': to_float(
                        compound_value.get('width')
                        if compound_value.get('width') is not None
                        else compound_value.get('measuredWidth')
                    ),
                    'speed': to_float(speed_value),
                    'current': to_float(
                        compound_value.get('current')
                        if compound_value.get('current') is not None
                        else compound_value.get('measuredCurrent')
                    ),
                    'voltage': to_float(
                        compound_value.get('voltage')
                        if compound_value.get('voltage') is not None
                        else compound_value.get('measuredVoltage')
                    ),
                    'input_params': parse_input_params_value(
                        compound_value.get('inputParams')
                    ),
                }

            def merge_series(entity: Dict[str, Any]) -> List[Dict]:
                series_by_timestamp: Dict[str, Dict[str, Any]] = {}

                # Parse compound telemetry payloads (DDS path).
                telemetry_attr = entity.get(
                    'urn:robin:processTelemetry'
                ) or entity.get('processTelemetry')
                for timestamp, value in temporal_pairs(telemetry_attr):
                    sample = parse_compound_value(value)
                    if not sample:
                        continue
                    entry = ensure_entry(series_by_timestamp, timestamp)
                    for metric_name, metric_value in sample.items():
                        if metric_name == 'input_params':
                            if metric_value:
                                entry['input_params'] = metric_value
                            continue
                        if metric_value is not None:
                            entry[metric_name] = metric_value

                # Parse scalar temporal properties (CLI/demo path).
                attr_map = {
                    'measuredHeight': 'height',
                    'measuredWidth': 'width',
                    'measuredSpeed': 'speed',
                    'measuredCurrent': 'current',
                    'measuredVoltage': 'voltage',
                }
                for attr_name, metric_name in attr_map.items():
                    for timestamp, value in temporal_pairs(
                        entity.get(attr_name)
                    ):
                        metric_value = to_float(value)
                        if metric_value is None:
                            continue
                        entry = ensure_entry(series_by_timestamp, timestamp)
                        entry[metric_name] = metric_value

                for timestamp, value in temporal_pairs(entity.get('inputParams')):
                    metric_value = parse_input_params_value(value)
                    if not metric_value:
                        continue
                    entry = ensure_entry(series_by_timestamp, timestamp)
                    entry['input_params'] = metric_value

                series = [
                    s
                    for s in series_by_timestamp.values()
                    if any(
                        s.get(metric_name) is not None
                        for metric_name in (
                            'height',
                            'width',
                            'speed',
                            'current',
                            'voltage',
                        )
                    ) or s.get('input_params')
                ]
                series.sort(key=lambda x: x.get('timestamp', ''))
                return series

            # Try observedAt first
            entity = call_temporal('observedAt', options_temporal_values=True)
            series: List[Dict] = []
            if entity:
                series = merge_series(entity)
            if not series:
                entity = call_temporal(
                    'observedAt', options_temporal_values=False
                )
                if entity:
                    series = merge_series(entity)

            # If still empty, fallback to modifiedAt
            if not series:
                entity = call_temporal(
                    'modifiedAt', options_temporal_values=True
                )
                if entity:
                    series = merge_series(entity)
            if not series:
                entity = call_temporal(
                    'modifiedAt', options_temporal_values=False
                )
                if entity:
                    series = merge_series(entity)

            if last_n and isinstance(last_n, int) and last_n > 0:
                series = series[-last_n:]

            print(
                f'Returning {len(series)} measurements from Mintaka (process-scoped)'
            )
            return series
        except Exception as e:
            # On any error, do not raise. Let caller fallback to Orion.
            print(f'⚠️ Mintaka query failed (will fallback to Orion): {e}')
            return []

    def fetch_all_measurements_orion(self, process_id: str) -> List[Dict]:
        """Fetch all Measurement entities for a process from Orion, sorted by observedAt.

        This is a pragmatic path while Mintaka/TROE is being validated. It queries Orion
        for Measurement entities related to the given Process and constructs a
        time series ordered by timestamp.
        """
        try:
            response = requests.get(
                f'{self.client.orion_url}/ngsi-ld/v1/entities',
                headers=self.client.headers,
                params={
                    'type': 'urn:robin:Measurement',
                    'q': f'processId=="urn:ngsi-ld:Process:{process_id}"',
                    'limit': '1000',
                    'attrs': 'measuredHeight,measuredWidth,measuredSpeed,measuredCurrent,measuredVoltage,inputParams',
                },
                timeout=10,
            )
            if response.status_code != 200:
                return []

            entities = (
                response.json() if isinstance(response.json(), list) else []
            )
            series: List[Dict] = []
            for ent in entities:
                # Prefer observedAt from height; fallback to width if missing
                ts = ent.get('measuredHeight', {}).get(
                    'observedAt'
                ) or ent.get('measuredWidth', {}).get('observedAt')
                if not ts:
                    continue
                entry: Dict[str, float] = {
                    'timestamp': ts,
                    'height': ent.get('measuredHeight', {}).get('value'),
                    'width': ent.get('measuredWidth', {}).get('value'),
                }
                if 'measuredSpeed' in ent:
                    entry['speed'] = ent.get('measuredSpeed', {}).get('value')
                if 'measuredCurrent' in ent:
                    entry['current'] = ent.get('measuredCurrent', {}).get(
                        'value'
                    )
                if 'measuredVoltage' in ent:
                    entry['voltage'] = ent.get('measuredVoltage', {}).get(
                        'value'
                    )
                raw_input_params = ent.get('inputParams', {}).get('value')
                if isinstance(raw_input_params, dict):
                    canonical_input_params = self._canonicalize_input_params(
                        raw_input_params
                    )
                    if canonical_input_params:
                        entry['input_params'] = canonical_input_params
                series.append(entry)

            # Sort by timestamp ascending for plotting
            series.sort(key=lambda x: x['timestamp'])
            return series
        except Exception:
            return []

    def fetch_measurements_from_troe(
        self, process_id: str, last_n: Optional[int] = None
    ) -> List[Dict]:
        """Direct TimescaleDB query for DDS compound telemetry.

        Used when Mintaka cannot handle compound DDS attributes (subproperties=true).
        Reads the compound jsonb column which contains the flat telemetry fields
        stored by Orion-LD's DDS bridge.
        """
        try:
            import psycopg2
            import psycopg2.extras

            host = os.getenv('TROE_HOST', 'timescaledb')
            port = int(os.getenv('TROE_PORT', '5432'))
            dbname = os.getenv('TROE_DB', 'orion')
            user = os.getenv('TROE_USER', 'orion')
            password = os.getenv('TROE_PWD', 'orionpass')

            conn = psycopg2.connect(
                host=host, port=port, dbname=dbname,
                user=user, password=password, connect_timeout=5
            )
            cur = conn.cursor(cursor_factory=psycopg2.extras.DictCursor)
            limit_clause = f'LIMIT {int(last_n)}' if last_n else 'LIMIT 1000'
            cur.execute(
                f"""
                SELECT compound, observedat FROM attributes
                WHERE entityid = %s
                  AND id = %s
                  AND observedat IS NOT NULL
                  AND compound IS NOT NULL
                ORDER BY observedat DESC
                {limit_clause}
                """,
                (
                    f'urn:ngsi-ld:Process:{process_id}',
                    'urn:robin:processTelemetry',
                ),
            )
            rows = cur.fetchall()
            cur.close()
            conn.close()

            series = []
            for row in rows:
                compound = row[0]    # jsonb parsed as dict by psycopg2
                observedat = row[1]  # datetime object
                iso = observedat.isoformat()
                ts_str = (iso[:-6] + 'Z') if iso.endswith('+00:00') else (iso + 'Z' if '+' not in iso and not iso.endswith('Z') else iso)
                series.append({
                    'timestamp': ts_str,
                    'height': float(compound.get('height') or 0),
                    'width': float(compound.get('width') or 0),
                    'speed': float(compound.get('speed') or 0),
                    'current': float(compound.get('current') or 0),
                    'voltage': float(compound.get('voltage') or 0),
                })
            series.reverse()  # return in chronological order
            print(f'TROE direct: {len(series)} rows for {process_id}')
            return series
        except Exception as e:
            print(f'⚠️ TROE direct query failed: {e}')
            return []

    def fetch_process_alerts(
        self, process_id: str, last_n: int | None = None
    ) -> List[Dict]:
        """Fetch all alert entities for a process from Orion, sorted by timestamp."""
        try:
            page_size = 500
            offset = 0
            max_pages = 40
            alerts: List[Dict[str, Any]] = []
            process_urn = f'urn:ngsi-ld:Process:{process_id}'

            for _ in range(max_pages):
                response = requests.get(
                    f'{self.client.orion_url}/ngsi-ld/v1/entities',
                    headers=self.client.headers,
                    params={
                        'type': 'urn:robin:Alert',
                        'q': f'processId=="{process_urn}"',
                        'limit': str(page_size),
                        'offset': str(offset),
                        'attrs': 'timestamp,deviationType,expectedValue,measuredValue,deviationPercentage,recommendedActions',
                    },
                    timeout=10,
                )
                if response.status_code != 200:
                    return []

                entities = (
                    response.json() if isinstance(response.json(), list) else []
                )
                if not entities:
                    break

                for entity in entities:
                    timestamp = (
                        entity.get('timestamp', {}).get('value')
                        or entity.get('modifiedAt', {}).get('value')
                        or entity.get('createdAt', {}).get('value')
                    )
                    expected = entity.get('expectedValue', {}).get('value')
                    measured = entity.get('measuredValue', {}).get('value')
                    actions = entity.get('recommendedActions', {}).get(
                        'value'
                    )
                    if not isinstance(actions, list):
                        actions = [str(actions)] if actions is not None else []

                    alert_entry: Dict[str, Any] = {
                        'id': entity.get('id'),
                        'timestamp': timestamp,
                        'deviation_type': entity.get(
                            'deviationType', {}
                        ).get('value'),
                        'deviation_percentage': entity.get(
                            'deviationPercentage', {}
                        ).get('value'),
                        'expected_value': expected
                        if isinstance(expected, dict)
                        else None,
                        'measured_value': measured
                        if isinstance(measured, dict)
                        else None,
                        'recommended_actions': actions,
                    }
                    alerts.append(alert_entry)

                if len(entities) < page_size:
                    break
                offset += page_size

            alerts.sort(key=lambda x: x.get('timestamp') or '')
            if last_n and isinstance(last_n, int) and last_n > 0:
                alerts = alerts[-last_n:]
            return alerts
        except Exception:
            return []


ENGINE = AlertEngine()


@app.get('/ai/models')
async def list_ai_models():
    """List discovered AI checkpoints with basic metadata."""
    engine = ENGINE
    return {
        'models': engine.list_available_models(),
        'active_model': engine.active_model_info(),
    }


@app.get('/ai/models/directories')
async def list_ai_model_directories():
    """Return directories that the engine inspects for checkpoints."""
    engine = ENGINE
    return {'directories': [str(path) for path in engine.model_dirs]}


@app.get('/ai/models/active')
async def get_active_ai_model():
    """Return metadata for the currently loaded checkpoint, if any."""
    engine = ENGINE
    info = engine.active_model_info()
    if not info:
        return {'status': 'inactive'}
    return {'status': 'active', 'model': info}


@app.post('/ai/models/select')
async def select_ai_model(request: AIModelSelectionRequest):
    """Load a specific checkpoint and make it the active model."""
    engine = ENGINE

    candidate = Path(request.path)
    if not candidate.is_absolute():
        relative_candidate = ROOT_DIR / candidate
        if relative_candidate.exists():
            candidate = relative_candidate
        else:
            matched = None
            for directory in engine.model_dirs:
                option = directory / candidate
                if option.exists():
                    matched = option
                    break
            if matched is not None:
                candidate = matched

    try:
        engine.load_model_from_path(candidate)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc))
    except Exception as exc:
        raise HTTPException(
            status_code=400,
            detail=f'Failed to load checkpoint at {candidate}: {exc}',
        )

    return {
        'status': 'success',
        'active_model': engine.active_model_info(),
    }


@app.post('/ai/models/predict')
async def predict_geometry(payload: Dict[str, Any]):
    """Run a forward pass on the active model."""
    engine = ENGINE
    raw_params = payload.get('input_params')
    if not isinstance(raw_params, dict):
        raw_params = payload
    params = engine._require_complete_input_params(raw_params)
    scored = engine.predict_geometry_with_confidence(params)
    response_input_params = scored.get('input_params')
    if not isinstance(response_input_params, dict):
        response_input_params = params
    return {
        'prediction': scored['prediction'],
        'confidence': scored['confidence'],
        'uncertainty': scored['uncertainty'],
        'diagnostics': scored['diagnostics'],
        'input_params': response_input_params,
        'input_feature_specs': engine.input_feature_specs(),
        'feature_order': list(engine.feature_order),
        'using_ml_model': engine.ai_model is not None,
        'model_path': str(engine.active_model_path)
        if engine.active_model_path
        else None,
    }


@app.post('/check-deviation')
async def check_deviation(request: DeviationCheckRequest):
    """Main endpoint for deviation checking"""
    engine = ENGINE

    # First check if process is active - don't check deviations for stopped processes
    process_data = engine.fetch_process_data(request.process_id)
    if not process_data:
        return {
            'status': 'error',
            'message': f'Process {request.process_id} not found',
        }

    process_status = process_data.get('processStatus', {}).get(
        'value', 'unknown'
    )

    # Determine if caller already provided a measurement snapshot
    has_measured_geometry = bool(request.measured_geometry)

    # If status is unknown/missing (common with DDS-created entities), infer liveness
    # from recent samples. If the caller already provided a measurement snapshot and
    # status is merely 'unknown', allow deviation checking to proceed.
    if process_status != 'active':
        # If explicitly stopped, always skip regardless of provided measurement
        if process_status == 'stopped':
            return {
                'status': 'process_inactive',
                'message': f'Process {request.process_id} is {process_status}. Deviation checking skipped.',
                'process_status': process_status,
            }

        # Permit proceeding when a measured geometry is supplied and status is not 'stopped'
        # (e.g., 'unknown' when using the ROS DDS bridge which doesn't set processStatus)
        if has_measured_geometry and process_status in ('unknown', None, ''):
            pass  # proceed with deviation check
        else:
            try:
                recent = engine.fetch_all_measurements(
                    request.process_id, last_n=1
                )
                if recent:
                    # Accept modifiedAt timestamps or plain ISO strings
                    ts_str = recent[-1].get('timestamp')
                    if ts_str:
                        try:
                            last_ts = datetime.fromisoformat(
                                ts_str.replace('Z', '+00:00')
                            )
                            age = (
                                datetime.now(timezone.utc) - last_ts
                            ).total_seconds()
                            if (
                                age <= 120
                            ):  # consider process active if updated within last 2 minutes
                                process_status = 'active'
                        except Exception:
                            # If parsing fails, assume active when data exists
                            process_status = 'active'
            except Exception:
                pass

            if process_status != 'active':
                return {
                    'status': 'process_inactive',
                    'message': f'Process {request.process_id} is {process_status}. Deviation checking skipped.',
                    'process_status': process_status,
                }

    # Get tolerance from request or use default
    tolerance = request.tolerance or engine.default_tolerance

    # Fetch process data to get current tolerance if not provided
    if tolerance == engine.default_tolerance:
        if process_data and 'toleranceThreshold' in process_data:
            tolerance = process_data['toleranceThreshold'].get(
                'value', engine.default_tolerance
            )

    # Determine effective mode: prefer process's operationMode from Orion
    process_mode = process_data.get('operationMode', {}).get('value')
    effective_mode = process_mode or request.mode or 'parameter_driven'

    alert: Optional[DeviationAlert] = None
    expected_geometry: Optional[Dict[str, float]] = None
    expected_source = 'unknown'
    target_geometry: Optional[Dict[str, float]] = None

    if effective_mode == 'parameter_driven':
        if not request.input_params:
            return {
                'status': 'error',
                'message': 'input_params required for parameter_driven mode',
            }
        canonical_input_params = engine._require_complete_input_params(
            request.input_params
        )

        measured = request.measured_geometry
        if not measured:
            latest = engine.fetch_latest_measurement(request.process_id)
            if latest:
                measured = {
                    'height': latest['height'],
                    'width': latest['width'],
                }
            else:
                return {
                    'status': 'no_data',
                    'message': 'No measurements available for deviation check',
                }
        alert, expected_geometry = engine.check_parameter_driven_deviation(
            request.process_id,
            canonical_input_params,
            measured,
            tolerance,
        )
        expected_source = 'ai_prediction_from_params'
    else:  # geometry_driven mode
        target_geometry = engine.fetch_geometry_target(request.process_id)
        if not target_geometry:
            return {
                'status': 'error',
                'message': f'No geometry target found for process {request.process_id}',
            }

        measured = request.measured_geometry
        if not measured:
            latest = engine.fetch_latest_measurement(request.process_id)
            if latest:
                measured = {
                    'height': latest['height'],
                    'width': latest['width'],
                }
            else:
                return {
                    'status': 'no_data',
                    'message': 'No measurements available for deviation check',
                }

        # In geometry-driven mode we support two references:
        # 1) AI-predicted geometry from provided parameters (preferred when available)
        # 2) Explicit geometry target from Orion (fallback)
        reference_geometry = target_geometry
        expected_source = 'target_geometry'
        if request.input_params:
            canonical_input_params = engine._require_complete_input_params(
                request.input_params
            )
            try:
                reference_geometry = engine.predict_geometry_from_params(
                    canonical_input_params
                )
                expected_source = 'ai_prediction_from_params'
            except Exception:
                reference_geometry = target_geometry
                expected_source = 'target_geometry'

        alert, expected_geometry = engine.check_geometry_driven_deviation(
            request.process_id,
            reference_geometry,
            measured,
            tolerance,
        )

    deviation_breakdown = {}
    if expected_geometry and measured:
        try:
            height_dev = engine.calculate_deviation(
                expected_geometry['height'], measured['height']
            )
            width_dev = engine.calculate_deviation(
                expected_geometry['width'], measured['width']
            )
            deviation_breakdown = {
                'height': height_dev,
                'width': width_dev,
                'max': max(height_dev, width_dev),
            }
        except Exception:
            deviation_breakdown = {}

    if alert:
        # Create alert entity in Orion
        engine.client.create_alert(alert)
        data = alert.dict()
        data['expected_value'] = expected_geometry
        data['measured_value'] = measured
        data['mode'] = effective_mode
        data['expected_source'] = expected_source
        if target_geometry:
            data['target_geometry'] = target_geometry
        if deviation_breakdown:
            data['deviation_breakdown'] = deviation_breakdown
        return data

    return {
        'status': 'ok',
        'message': 'No deviation detected',
        'mode': effective_mode,
        'tolerance': tolerance,
        'expected_value': expected_geometry,
        'expected_source': expected_source,
        'measured_value': measured,
        'target_geometry': target_geometry,
        'deviation_breakdown': deviation_breakdown,
    }


@app.post('/operator-action')
async def handle_operator_action(
    process_id: str, action_id: str, params: Optional[Dict] = None
):
    """Handle operator's response to an alert"""
    engine = ENGINE

    if action_id == 'manual_adjust':
        # Update process parameters in Orion
        return {'status': 'parameters_updated', 'params': params}

    elif action_id == 'ai_retry':
        # Generate new recommendation
        # Validate payload
        if not params or 'target_geometry' not in params:
            return {
                'status': 'error',
                'message': 'target_geometry required for ai_retry',
            }
        new_params = engine.recommend_params_for_geometry(
            params['target_geometry']
        )
        engine.client.create_ai_recommendation(process_id, new_params)
        return {'status': 'new_recommendation', 'params': new_params}

    elif action_id == 'fine_tune':
        # Add to training dataset and trigger fine-tuning
        # This would integrate with your RobTrack training pipeline
        return {'status': 'fine_tuning_started', 'estimated_time': '2 minutes'}

    elif action_id == 'reset_model':
        # Start new DOE process
        return {
            'status': 'doe_initialized',
            'next_step': 'define_experiment_space',
        }


@app.post('/create-process')
async def create_process(request: CreateProcessRequest):
    """Create a new process"""
    try:
        from robin.cli import OperationMode

        if request.mode not in ('parameter_driven', 'geometry_driven'):
            return {
                'error': (
                    f'Invalid mode: {request.mode}. Must be '
                    'parameter_driven or geometry_driven'
                ),
                'status': 'error',
            }

        # Convert mode string to enum
        mode = (
            OperationMode.PARAMETER_DRIVEN
            if request.mode == 'parameter_driven'
            else OperationMode.GEOMETRY_DRIVEN
        )

        # Create the process
        client = RobinFiwareClient()
        success = client.create_process(request.process_id, mode)

        if not success:
            return {'error': 'Failed to create process', 'status': 'error'}

        if request.initial_params:
            canonical_initial_params = ENGINE._require_complete_input_params(
                request.initial_params
            )
            input_success = client.set_input_params(
                request.process_id, canonical_initial_params
            )
            if not input_success:
                return {
                    'error': 'Process created but failed to persist initial input parameters',
                    'status': 'warning',
                }

        # If geometry-driven mode, create target geometry
        if request.mode == 'geometry_driven' and request.target_geometry:
            target_success = client.create_geometry_target(
                request.process_id,
                request.target_geometry['height'],
                request.target_geometry['width'],
            )
            if not target_success:
                return {
                    'error': 'Process created but failed to set target geometry',
                    'status': 'warning',
                }

        return {
            'status': 'success',
            'process_id': request.process_id,
            'mode': request.mode,
            'message': f'Process {request.process_id} created successfully',
        }

    except Exception as e:
        return {'error': str(e), 'status': 'error'}


@app.get('/process/{process_id}')
async def get_process_data(process_id: str):
    """Get process data from Orion Context Broker, including latest measurement snapshot"""
    try:
        engine = ENGINE
        # Use the same Orion URL as the CLI client to avoid hostname mismatches
        response = requests.get(
            f'{engine.client.orion_url}/ngsi-ld/v1/entities/urn:ngsi-ld:Process:{process_id}',
            headers=engine.client.headers,
            timeout=5,
        )

        if response.status_code == 404:
            return {'error': 'Process not found', 'status': 'not_found'}
        if response.status_code != 200:
            return {'error': 'Failed to fetch process data', 'status': 'error'}

        entity = response.json()

        latest = engine.fetch_latest_measurement(process_id)
        if latest:
            entity['measuredHeight'] = {
                'type': 'Property',
                'value': latest.get('height'),
                'unitCode': 'MMT',
                'observedAt': latest.get('timestamp'),
            }
            entity['measuredWidth'] = {
                'type': 'Property',
                'value': latest.get('width'),
                'unitCode': 'MMT',
                'observedAt': latest.get('timestamp'),
            }

            # Add machine parameters if available
            if latest.get('speed') is not None:
                entity['measuredSpeed'] = {
                    'type': 'Property',
                    'value': latest.get('speed'),
                    'unitCode': 'mm/s',
                    'observedAt': latest.get('timestamp'),
                }
            if latest.get('current') is not None:
                entity['measuredCurrent'] = {
                    'type': 'Property',
                    'value': latest.get('current'),
                    'unitCode': 'A',
                    'observedAt': latest.get('timestamp'),
                }
            if latest.get('voltage') is not None:
                entity['measuredVoltage'] = {
                    'type': 'Property',
                    'value': latest.get('voltage'),
                    'unitCode': 'V',
                    'observedAt': latest.get('timestamp'),
                }
            if (
                'input_params' in latest
                and isinstance(latest.get('input_params'), dict)
                and latest.get('input_params')
            ):
                entity['inputParams'] = {
                    'type': 'Property',
                    'value': latest.get('input_params'),
                    'observedAt': latest.get('timestamp'),
                }

        return entity

    except Exception as e:
        return {'error': str(e), 'status': 'error'}


@app.get('/process/{process_id}/measurements')
async def get_process_measurements(process_id: str, last: int | None = None):
    """Get all measurements for a process with timestamps for plotting"""
    try:
        engine = ENGINE
        # Prefer Mintaka (TROE temporal storage). If empty, fallback to Orion entity listing.
        measurements_mintaka = engine.fetch_all_measurements(
            process_id, last_n=last
        )
        if measurements_mintaka:
            source = 'mintaka'
            measurements = measurements_mintaka
        else:
            measurements_orion = engine.fetch_all_measurements_orion(
                process_id
            )
            if last:
                measurements_orion = measurements_orion[-last:]
            if measurements_orion:
                source = 'orion'
                measurements = measurements_orion
            else:
                measurements_troe = engine.fetch_measurements_from_troe(
                    process_id, last_n=last
                )
                if measurements_troe:
                    source = 'troe'
                    measurements = measurements_troe
                else:
                    source = 'none'
                    measurements = []

        print(
            f'📊 Fetching measurements for process {process_id}: found {len(measurements)} measurements'
        )

        return {
            'status': 'success',
            'process_id': process_id,
            'measurements': measurements,
            'count': len(measurements),
            'debug_info': {
                'mintaka_url': os.getenv('MINTAKA_URL', 'http://mintaka:8080'),
                'tenant': os.getenv('NGSILD_TENANT', ''),
                'source': source,
            },
        }

    except Exception as e:
        print(f'❌ Error fetching measurements for {process_id}: {e}')
        return {
            'status': 'error',
            'error': str(e),
            'message': f'Failed to fetch measurements for process {process_id}',
        }


@app.get('/process/{process_id}/alerts')
async def get_process_alerts(process_id: str, last: int | None = None):
    """Get persisted alert entities for a process."""
    try:
        engine = ENGINE
        alerts = engine.fetch_process_alerts(process_id, last_n=last)
        return {
            'status': 'success',
            'process_id': process_id,
            'alerts': alerts,
            'count': len(alerts),
        }
    except Exception as e:
        return {
            'status': 'error',
            'error': str(e),
            'message': f'Failed to fetch alerts for process {process_id}',
        }


@app.get('/processes')
async def list_processes():
    """List all processes"""
    try:
        client = ENGINE.client
        response = requests.get(
            f'{client.orion_url}/ngsi-ld/v1/entities?type=urn:robin:Process',
            headers=client.headers,
            timeout=5,
        )

        if response.status_code == 200:
            return response.json()
        else:
            return {'error': 'Failed to fetch processes', 'status': 'error'}

    except Exception as e:
        return {'error': str(e), 'status': 'error'}


@app.post('/ai-recommendation')
async def get_ai_recommendation(request: AIRecommendationRequest):
    """Get AI recommendation for parameters or geometry"""
    engine = ENGINE

    try:
        if request.mode == 'parameter_driven':
            # For parameter-driven mode, predict geometry from current parameters
            if not request.input_params:
                return {
                    'error': 'input_params required for parameter_driven mode',
                    'status': 'error',
                }
            canonical_input_params = engine._require_complete_input_params(
                request.input_params
            )

            scored_prediction = engine.predict_geometry_with_confidence(
                canonical_input_params
            )
            predicted_geometry = scored_prediction['prediction']
            recommendation = {
                'process_id': request.process_id,
                'mode': request.mode,
                'predicted_geometry': predicted_geometry,
                'confidence': float(scored_prediction['confidence']),
                'prediction_uncertainty': scored_prediction['uncertainty'],
                'prediction_diagnostics': scored_prediction['diagnostics'],
                'model_version': 'v2.1',
                'recommendations': [
                    'Monitor height deviation closely',
                    'Consider adjusting wire speed if deviation exceeds 8%',
                ],
            }

        elif request.mode == 'geometry_driven':
            # For geometry-driven mode, recommend parameters for target geometry
            if not request.target_geometry:
                return {
                    'error': 'target_geometry required for geometry_driven mode',
                    'status': 'error',
                }

            if request.input_params is not None:
                canonical_current_params = engine._require_complete_input_params(
                    request.input_params
                )
                try:
                    recommended_params = engine.recommend_params_for_geometry(
                        request.target_geometry,
                        current_params=canonical_current_params,
                    )
                except TypeError:
                    # Backward-compatible fallback for patched/mocked call-sites
                    # that still expose the old single-argument signature.
                    recommended_params = engine.recommend_params_for_geometry(
                        request.target_geometry
                    )
            else:
                recommended_params = engine.recommend_params_for_geometry(
                    request.target_geometry
                )
            recommendation_confidence = float(
                recommended_params.get('confidence', 0.88)
            )
            recommendation = {
                'process_id': request.process_id,
                'mode': request.mode,
                'recommended_params': recommended_params,
                'confidence': recommendation_confidence,
                'model_version': 'v2.1',
                'recommendations': [
                    'Apply recommended parameters gradually',
                    'Monitor first few passes for stability',
                ],
            }
        else:
            return {
                'error': f'Invalid mode: {request.mode}. Must be parameter_driven or geometry_driven',
                'status': 'error',
            }

        # Store recommendation in FIWARE
        engine.client.create_ai_recommendation(
            request.process_id, recommendation
        )

        return {'status': 'success', 'recommendation': recommendation}

    except HTTPException:
        raise
    except Exception as e:
        return {
            'status': 'error',
            'error': str(e),
            'message': 'Failed to generate AI recommendation',
        }


@app.post('/process/{process_id}/stop')
async def stop_process_api(process_id: str, reason: str = 'operator_request'):
    """Stop a running process"""
    try:
        from robin.cli import RobinFiwareClient

        client = RobinFiwareClient()
        success, message = client.stop_process(process_id, reason)

        if success:
            return {
                'status': 'success',
                'process_id': process_id,
                'message': message,
                'stopped_at': datetime.now(timezone.utc).isoformat(),
                'reason': reason,
            }
        else:
            return {
                'status': 'error',
                'process_id': process_id,
                'message': message,
            }

    except Exception as e:
        return {
            'status': 'error',
            'process_id': process_id,
            'error': str(e),
            'message': f'Failed to stop process {process_id}',
        }


@app.post('/process/{process_id}/resume')
async def resume_process_api(process_id: str):
    """Resume a stopped process"""
    try:
        from robin.cli import RobinFiwareClient

        client = RobinFiwareClient()
        success, message = client.resume_process(process_id)

        if success:
            return {
                'status': 'success',
                'process_id': process_id,
                'message': message,
                'resumed_at': datetime.now(timezone.utc).isoformat(),
            }
        else:
            return {
                'status': 'error',
                'process_id': process_id,
                'message': message,
            }

    except Exception as e:
        return {
            'status': 'error',
            'process_id': process_id,
            'error': str(e),
            'message': f'Failed to resume process {process_id}',
        }


@app.get('/process/{process_id}/status')
async def get_process_status_api(process_id: str):
    """Get the current status of a process"""
    try:
        from robin.cli import RobinFiwareClient

        client = RobinFiwareClient()
        status_info, error = client.get_process_status(process_id)

        if error:
            return {
                'status': 'error',
                'process_id': process_id,
                'message': error,
            }

        return {'status': 'success', 'process_data': status_info}

    except Exception as e:
        return {
            'status': 'error',
            'process_id': process_id,
            'error': str(e),
            'message': f'Failed to get status for process {process_id}',
        }


@app.post('/process/{process_id}/mode')
async def set_process_mode(process_id: str, request: SetModeRequest):
    """Set operation mode for an existing process."""
    try:
        if request.mode not in ('parameter_driven', 'geometry_driven'):
            return {
                'status': 'error',
                'process_id': process_id,
                'message': (
                    f'Invalid mode: {request.mode}. Must be '
                    'parameter_driven or geometry_driven'
                ),
            }

        mode = (
            OperationMode.PARAMETER_DRIVEN
            if request.mode == 'parameter_driven'
            else OperationMode.GEOMETRY_DRIVEN
        )

        client = RobinFiwareClient()
        ok = client.set_operation_mode(process_id, mode)
        if not ok:
            return {
                'status': 'error',
                'process_id': process_id,
                'message': 'Failed to update operation mode',
            }

        return {
            'status': 'success',
            'process_id': process_id,
            'operation_mode': request.mode,
        }
    except Exception as e:
        return {
            'status': 'error',
            'process_id': process_id,
            'error': str(e),
            'message': f'Failed to set operation mode for process {process_id}',
        }


@app.post('/process/{process_id}/input-params')
async def set_process_input_params(
    process_id: str, request: SetInputParamsRequest
):
    """Persist the currently commanded AI/process inputs on the Process entity."""
    try:
        canonical_input_params = ENGINE._require_complete_input_params(
            request.input_params
        )
        client = RobinFiwareClient()
        ok = client.set_input_params(process_id, canonical_input_params)
        if not ok:
            return {
                'status': 'error',
                'process_id': process_id,
                'message': 'Failed to update input parameters',
            }
        return {
            'status': 'success',
            'process_id': process_id,
            'input_params': canonical_input_params,
        }
    except HTTPException:
        raise
    except Exception as e:
        return {
            'status': 'error',
            'process_id': process_id,
            'error': str(e),
            'message': f'Failed to set input parameters for process {process_id}',
        }


@app.post('/process/{process_id}/target')
async def set_process_target(process_id: str, request: SetTargetRequest):
    """Create or update the geometry target for an existing process."""
    try:
        client = RobinFiwareClient()
        # Try to create first (idempotent pattern: create → patch fallback)
        ok = client.create_geometry_target(
            process_id, request.height, request.width
        )
        if not ok:
            try:
                entity_id = f'urn:ngsi-ld:GeometryTarget:{process_id}'
                payload = {
                    'targetHeight': {
                        'type': 'Property',
                        'value': request.height,
                        'unitCode': 'MMT',
                    },
                    'targetWidth': {
                        'type': 'Property',
                        'value': request.width,
                        'unitCode': 'MMT',
                    },
                }
                resp = requests.patch(
                    f'{client.orion_url}/ngsi-ld/v1/entities/{entity_id}/attrs',
                    headers=client.headers,
                    json=payload,
                    timeout=5,
                )
                ok = 200 <= resp.status_code < 300
            except Exception:
                ok = False

        # Ensure operation mode is geometry_driven after setting target
        mode_set_ok = client.set_operation_mode(
            process_id, OperationMode.GEOMETRY_DRIVEN
        )

        if ok:
            return {
                'status': 'success',
                'process_id': process_id,
                'target_geometry': {
                    'height': request.height,
                    'width': request.width,
                },
                'mode_set': 'geometry_driven' if mode_set_ok else 'unchanged',
            }
        else:
            return {
                'status': 'error',
                'process_id': process_id,
                'message': 'Failed to set target geometry',
            }
    except Exception as e:
        return {
            'status': 'error',
            'process_id': process_id,
            'error': str(e),
            'message': f'Failed to set target for process {process_id}',
        }


@app.get('/process/{process_id}/target')
async def get_process_target(process_id: str):
    """Return geometry target if present; 'not_set' otherwise."""
    try:
        engine = ENGINE
        tg = engine.fetch_geometry_target(process_id)
        if tg:
            return {
                'status': 'success',
                'process_id': process_id,
                'target_geometry': tg,
            }
        return {'status': 'not_set', 'process_id': process_id}
    except Exception as e:
        return {'status': 'error', 'process_id': process_id, 'error': str(e)}


@app.post('/process/{process_id}/progress')
async def set_simulation_progress(
    process_id: str, request: SimulationProgressRequest
):
    """Update simulation progress (0-100) on the Process entity in Orion."""
    try:
        engine = ENGINE
        entity_id = f'urn:ngsi-ld:Process:{process_id}'
        payload: Dict[str, Any] = {
            'simulationProgress': {
                'type': 'Property',
                'value': max(0.0, min(100.0, request.progress)),
            },
        }
        if request.expected_duration is not None:
            payload['expectedDuration'] = {
                'type': 'Property',
                'value': request.expected_duration,
                'unitCode': 'SEC',
            }
        url = f'{engine.client.orion_url}/ngsi-ld/v1/entities/{entity_id}/attrs'
        resp = requests.post(
            url,
            headers=engine.client.headers,
            json=payload,
            timeout=5,
        )
        if resp.status_code in (207, 409):
            resp = requests.patch(
                url,
                headers=engine.client.headers,
                json=payload,
                timeout=5,
            )
        ok = 200 <= resp.status_code < 300
        if ok:
            return {
                'status': 'success',
                'process_id': process_id,
                'progress': request.progress,
            }
        return {
            'status': 'error',
            'process_id': process_id,
            'message': f'Orion returned {resp.status_code}',
        }
    except Exception as e:
        return {
            'status': 'error',
            'process_id': process_id,
            'error': str(e),
        }


@app.get('/processes/list')
async def list_processes_api(status_filter: str = None):
    """List all processes with their status"""
    try:
        import requests

        engine = ENGINE
        response = requests.get(
            f'{engine.client.orion_url}/ngsi-ld/v1/entities?type=urn:robin:Process',
            headers=engine.client.headers,
            timeout=5,
        )

        if response.status_code != 200:
            return {
                'status': 'error',
                'message': 'Failed to fetch processes from Orion',
            }

        processes = response.json()

        # Filter by status if requested
        if status_filter:
            processes = [
                p
                for p in processes
                if p.get('processStatus', {}).get('value') == status_filter
            ]

        # Format process data
        process_list = []
        for process in processes:
            process_id = process['id'].split(':')[-1]  # Extract ID from URN
            process_info = {
                'process_id': process_id,
                'status': process.get('processStatus', {}).get(
                    'value', 'unknown'
                ),
                'operation_mode': process.get('operationMode', {}).get(
                    'value', 'unknown'
                ),
                'started_at': process.get('startedAt', {}).get('value'),
                'stopped_at': process.get('stoppedAt', {}).get('value'),
                'stop_reason': process.get('stopReason', {}).get('value'),
                'tolerance': process.get('toleranceThreshold', {}).get(
                    'value', 10.0
                ),
            }
            process_list.append(process_info)

        return {
            'status': 'success',
            'processes': process_list,
            'total_count': len(process_list),
            'filter_applied': status_filter,
        }

    except Exception as e:
        return {
            'status': 'error',
            'error': str(e),
            'message': 'Failed to list processes',
        }
