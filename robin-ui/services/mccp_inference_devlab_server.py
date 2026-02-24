"""
Development-only MCCP inference HTTP server for FAROS visual page.

Endpoints:
  - GET  /health
  - POST /config   (update workspace_root at runtime)
  - POST /predict

This server loads MCCP artifacts from the local shared-workspace path and
exposes uncertainty-quantified predictions for the Inference DevLab tab.
"""

from __future__ import annotations

import json
import numpy as np
import os
import sys
import threading
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

# Ensure the robin-ui package root is on sys.path so inference is importable.
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]  # .../robin-ui/services -> .../robin-ui
if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))

from inference.mccp_inference import (
    load_artifacts,
    predict,
)


def _default_workspace_root() -> Path:
    """Resolve the default MCCP shared-workspace root.

    Returns the path from ``MCCP_WORKSPACE_ROOT`` env-var, or a placeholder.
    The workspace lives outside RobTrack (e.g. in the MLOpsOrchestrator tree)
    so there is no universal fallback - configure via the Inference DevLab UI
    or set the env-var.
    """
    env_val = os.environ.get('MCCP_WORKSPACE_ROOT')
    if env_val:
        return Path(env_val).expanduser().resolve()
    return Path('MCCP_WORKSPACE_NOT_CONFIGURED')


DEFAULT_WORKSPACE_ROOT = _default_workspace_root()
DEFAULT_HOST = os.environ.get('MCCP_API_HOST', '0.0.0.0')
DEFAULT_PORT = int(os.environ.get('MCCP_API_PORT', '8091'))


@dataclass
class InferenceRuntime:
    workspace_root: Path
    model: Any = None
    mccp_state: Any = None
    model_mtime: float | None = None
    state_mtime: float | None = None
    lock: threading.Lock = field(default_factory=threading.Lock)

    @property
    def model_path(self) -> Path:
        return self.workspace_root / 'models' / 'mccp-uq' / 'model.h5'

    @property
    def state_path(self) -> Path:
        return self.workspace_root / 'state' / 'mccp-uq' / 'mccp_state.pkl'

    def artifacts_exist(self) -> bool:
        return self.model_path.exists() and self.state_path.exists()

    def _current_mtimes(self) -> tuple[float | None, float | None]:
        if not self.artifacts_exist():
            return None, None
        return self.model_path.stat().st_mtime, self.state_path.stat().st_mtime

    def is_stale(self) -> bool:
        if self.model is None or self.mccp_state is None:
            return True
        current_model_mtime, current_state_mtime = self._current_mtimes()
        if current_model_mtime is None or current_state_mtime is None:
            return True
        return (
            self.model_mtime != current_model_mtime
            or self.state_mtime != current_state_mtime
        )

    def ensure_loaded(self, reload_if_stale: bool = True) -> None:
        if not self.artifacts_exist():
            raise FileNotFoundError(
                f'MCCP artifacts missing. Expected: {self.model_path} and {self.state_path}'
            )

        if self.model is None or self.mccp_state is None:
            self._load()
            return

        if reload_if_stale and self.is_stale():
            self._load()

    def _load(self) -> None:
        self.model, self.mccp_state = load_artifacts(
            self.model_path, self.state_path
        )
        self.model_mtime, self.state_mtime = self._current_mtimes()

    def predict(
        self, input_features: np.ndarray, target_dim: int
    ) -> dict[str, np.ndarray]:
        with self.lock:
            self.ensure_loaded(reload_if_stale=True)
            return predict(
                self.model,
                self.mccp_state,
                input_features,
                target_dim=target_dim,
            )


runtime = InferenceRuntime(workspace_root=DEFAULT_WORKSPACE_ROOT)


def _json_response(
    handler: BaseHTTPRequestHandler,
    status: HTTPStatus,
    payload: dict[str, Any],
) -> None:
    body = json.dumps(payload).encode('utf-8')
    handler.send_response(status)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Content-Length', str(len(body)))
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
    handler.send_header('Access-Control-Allow-Headers', 'Content-Type')
    handler.end_headers()
    handler.wfile.write(body)


class MCCPDevLabHandler(BaseHTTPRequestHandler):
    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header(
            'Access-Control-Allow-Methods', 'GET, POST, PUT, OPTIONS'
        )
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self) -> None:  # noqa: N802
        if self.path != '/health':
            _json_response(
                self,
                HTTPStatus.NOT_FOUND,
                {
                    'ok': False,
                    'detail': 'Not found. Use /health, /config, or /predict.',
                },
            )
            return

        artifacts_ok = runtime.artifacts_exist()
        payload = {
            'ok': True,
            'model_available': artifacts_ok,
            'loaded': runtime.model is not None
            and runtime.mccp_state is not None,
            'workspace_root': str(runtime.workspace_root),
            'model_path': str(runtime.model_path),
            'state_path': str(runtime.state_path),
        }
        if not artifacts_ok:
            payload[
                'detail'
            ] = 'Run MCCP training first so model/state artifacts exist.'
        _json_response(self, HTTPStatus.OK, payload)

    def do_POST(self) -> None:  # noqa: N802
        if self.path == '/config':
            self._handle_config()
            return

        if self.path != '/predict':
            _json_response(
                self,
                HTTPStatus.NOT_FOUND,
                {
                    'ok': False,
                    'detail': 'Not found. Use /health, /config, or /predict.',
                },
            )
            return

        try:
            content_length = int(self.headers.get('Content-Length', '0'))
            raw_body = self.rfile.read(content_length).decode('utf-8')
            body = json.loads(raw_body) if raw_body else {}

            input_features = np.asarray(
                body.get('input_features', []), dtype=np.float32
            )
            if input_features.ndim != 2 or input_features.shape[0] == 0:
                raise ValueError(
                    'input_features must be a non-empty 2D numeric array.'
                )

            target_dim = int(body.get('target_dim', 2))
            if target_dim < 1:
                raise ValueError('target_dim must be >= 1.')

            reload_if_stale = bool(body.get('reload_if_stale', True))

            with runtime.lock:
                runtime.ensure_loaded(reload_if_stale=reload_if_stale)
                result = predict(
                    runtime.model,
                    runtime.mccp_state,
                    input_features,
                    target_dim=target_dim,
                )

            lower = np.asarray(result['lower'])
            upper = np.asarray(result['upper'])
            intervals = np.asarray(result['intervals'])
            widths = upper - lower
            midpoint = (upper + lower) / 2.0

            payload = {
                'ok': True,
                'target_dim': target_dim,
                'sample_count': int(input_features.shape[0]),
                'lower': lower.tolist(),
                'upper': upper.tolist(),
                'intervals': intervals.tolist(),
                'widths': widths.tolist(),
                'midpoint': midpoint.tolist(),
            }
            _json_response(self, HTTPStatus.OK, payload)
        except FileNotFoundError as e:
            _json_response(
                self, HTTPStatus.NOT_FOUND, {'ok': False, 'detail': str(e)}
            )
        except Exception as e:  # broad on purpose for dev service usability
            _json_response(
                self,
                HTTPStatus.BAD_REQUEST,
                {'ok': False, 'detail': f'{type(e).__name__}: {e}'},
            )

    def _handle_config(self) -> None:
        """POST /config - update workspace_root (and optionally model/state sub-paths)."""
        try:
            content_length = int(self.headers.get('Content-Length', '0'))
            raw_body = self.rfile.read(content_length).decode('utf-8')
            body = json.loads(raw_body) if raw_body else {}

            workspace_root = body.get('workspace_root')
            if not workspace_root or not isinstance(workspace_root, str):
                raise ValueError(
                    'workspace_root is required and must be a non-empty string.'
                )

            new_root = Path(workspace_root).expanduser().resolve()
            if not new_root.is_dir():
                raise ValueError(f'Directory does not exist: {new_root}')

            with runtime.lock:
                runtime.workspace_root = new_root
                # Reset loaded artifacts so they get reloaded from the new path
                runtime.model = None
                runtime.mccp_state = None
                runtime.model_mtime = None
                runtime.state_mtime = None

            artifacts_ok = runtime.artifacts_exist()
            _json_response(
                self,
                HTTPStatus.OK,
                {
                    'ok': True,
                    'workspace_root': str(runtime.workspace_root),
                    'model_path': str(runtime.model_path),
                    'state_path': str(runtime.state_path),
                    'model_available': artifacts_ok,
                    'detail': 'Workspace updated.'
                    + (
                        ''
                        if artifacts_ok
                        else ' Warning: model artifacts not found at new path.'
                    ),
                },
            )
        except ValueError as e:
            _json_response(
                self, HTTPStatus.BAD_REQUEST, {'ok': False, 'detail': str(e)}
            )
        except Exception as e:
            _json_response(
                self,
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {'ok': False, 'detail': f'{type(e).__name__}: {e}'},
            )

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f'[mccp-devlab] {self.address_string()} - {fmt % args}')


def main() -> None:
    server = ThreadingHTTPServer(
        (DEFAULT_HOST, DEFAULT_PORT), MCCPDevLabHandler
    )
    print(f'MCCP DevLab API listening on http://{DEFAULT_HOST}:{DEFAULT_PORT}')
    print('Health endpoint: GET /health')
    print('Predict endpoint: POST /predict')
    print(f'Workspace root: {runtime.workspace_root}')
    print(f'Model path: {runtime.model_path}')
    print(f'State path: {runtime.state_path}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nShutting down MCCP DevLab API...')
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
