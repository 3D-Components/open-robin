import { useState, useEffect, useRef, useCallback } from 'react';

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/**
 * Base URL for the ROBIN alert-engine API.
 *
 * In production (nginx container) the env var is baked at build time.
 * In dev mode the Vite proxy rewrites /api/* → http://localhost:8000/*.
 */
const BASE_URL: string =
    import.meta.env.VITE_ROBIN_API_URL   // explicit override (production build)
    ?? (import.meta.env.DEV ? '/api' : 'http://localhost:8000');

// ---------------------------------------------------------------------------
// API Types (match alert_engine.py response shapes)
// ---------------------------------------------------------------------------

export interface RobinMeasurement {
    timestamp: string;
    height: number;
    width: number;
    speed?: number;
    current?: number;
    voltage?: number;
}

export interface MeasurementsDebugInfo {
    source?: string;
    mintaka_url?: string;
    tenant?: string;
}

export interface MeasurementsResponse {
    status: string;
    process_id: string;
    measurements: RobinMeasurement[];
    count: number;
    debug_info?: MeasurementsDebugInfo;
}

export interface RobinAlertRecord {
    id?: string;
    timestamp?: string | null;
    deviation_type?: string | null;
    deviation_percentage?: number | null;
    expected_value?: { height?: number; width?: number } | null;
    measured_value?: { height?: number; width?: number } | null;
    recommended_actions?: Array<string | { id?: string; label?: string }>;
}

export interface AlertsResponse {
    status: string;
    process_id: string;
    alerts: RobinAlertRecord[];
    count: number;
}

export interface HealthResponse {
    status: string;
    orion_connected?: boolean;
    mintaka_connected?: boolean;
    latency_ms?: {
        backend?: number | null;
        orion?: number | null;
        mintaka?: number | null;
    };
    services?: {
        backend?: { connected: boolean; latency_ms?: number | null };
        orion?: { connected: boolean; status_code?: number | null; latency_ms?: number | null };
        mintaka?: { connected: boolean; status_code?: number | null; latency_ms?: number | null };
    };
    uptime_seconds?: number;
    error?: string;
    version?: string;
    endpoints?: string[];
}

export interface ProcessInfo {
    process_id: string;
    status: string;
    operation_mode: string;
    started_at?: string;
    stopped_at?: string;
    stop_reason?: string;
    tolerance: number;
}

export interface ProcessListResponse {
    status: string;
    processes: ProcessInfo[];
    total_count: number;
}

export interface AIModelInfo {
    path: string;
    name?: string;
    created?: string;
    size_bytes?: number;
}

export interface AIModelsResponse {
    models: AIModelInfo[];
    active_model: AIModelInfo | null;
}

export interface DeviationCheckRequest {
    process_id: string;
    mode?: string;
    input_params?: Record<string, number>;
    measured_geometry?: Record<string, number>;
    tolerance?: number;
}

export interface DeviationCheckResponse {
    status: string;
    mode?: string;
    deviation_percentage?: number;
    expected_value?: { height: number; width: number };
    expected_source?: string;
    measured_value?: { height: number; width: number };
    target_geometry?: { height: number; width: number };
    deviation_breakdown?: { height: number; width: number };
    message?: string;
}

export interface AIRecommendationRequest {
    process_id: string;
    mode: 'parameter_driven' | 'geometry_driven';
    input_params?: { wireSpeed: number; current: number; voltage: number };
    target_geometry?: { height: number; width: number };
}

export interface AIRecommendationResponse {
    status: string;
    recommendation?: {
        mode: string;
        predicted_geometry?: { height: number; width: number };
        recommended_params?: Record<string, number>;
        confidence?: number;
    };
    error?: string;
    message?: string;
}

export interface TargetGeometryResponse {
    status: string;
    target_geometry?: { height: number; width: number };
}

export interface ProcessSnapshotResponse {
    operationMode?: { value: string };
    toleranceThreshold?: { value: number };
    wireSpeed?: { value: number };
    current?: { value: number };
    voltage?: { value: number };
    processStatus?: { value: string };
    simulationProgress?: { value: number };
    expectedDuration?: { value: number };
    [key: string]: unknown;
}

// ---------------------------------------------------------------------------
// Generic polling hook
// ---------------------------------------------------------------------------

function usePolling<T>(
    fetcher: () => Promise<T>,
    intervalMs: number,
    enabled: boolean = true,
): { data: T | null; error: string | null; loading: boolean; refetch: () => void } {
    const [data, setData] = useState<T | null>(null);
    const [error, setError] = useState<string | null>(null);
    const [loading, setLoading] = useState(true);
    const mountedRef = useRef(true);

    const doFetch = useCallback(async () => {
        try {
            const result = await fetcher();
            if (mountedRef.current) {
                setData(result);
                setError(null);
                setLoading(false);
            }
        } catch (err: unknown) {
            if (mountedRef.current) {
                setError(err instanceof Error ? err.message : String(err));
                setLoading(false);
            }
        }
    }, [fetcher]);

    useEffect(() => {
        mountedRef.current = true;
        if (!enabled) return;

        doFetch(); // initial fetch
        const id = setInterval(doFetch, intervalMs);
        return () => {
            mountedRef.current = false;
            clearInterval(id);
        };
    }, [doFetch, intervalMs, enabled]);

    return { data, error, loading, refetch: doFetch };
}

// ---------------------------------------------------------------------------
// Polling hooks
// ---------------------------------------------------------------------------

/** Poll GET /process/{processId}/measurements?last={lastN} */
export function useMeasurements(processId: string | null, lastN: number = 200, intervalMs: number = 2000) {
    const fetcher = useCallback(async (): Promise<MeasurementsResponse> => {
        if (!processId) return { status: 'no_process', process_id: '', measurements: [], count: 0 };
        const url = `${BASE_URL}/process/${encodeURIComponent(processId)}/measurements?last=${lastN}`;
        const res = await fetch(url);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
    }, [processId, lastN]);

    return usePolling(fetcher, intervalMs, !!processId);
}

/** One-shot GET /process/{processId}/measurements (all by default, lastN optional) */
export async function fetchProcessMeasurements(processId: string, lastN?: number): Promise<MeasurementsResponse> {
    const qs = typeof lastN === 'number' ? `?last=${lastN}` : '';
    const res = await fetch(`${BASE_URL}/process/${encodeURIComponent(processId)}/measurements${qs}`);
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
}

/** One-shot GET /process/{processId}/alerts (all by default, lastN optional) */
export async function fetchProcessAlerts(processId: string, lastN?: number): Promise<AlertsResponse> {
    const qs = typeof lastN === 'number' ? `?last=${lastN}` : '';
    const res = await fetch(`${BASE_URL}/process/${encodeURIComponent(processId)}/alerts${qs}`);
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
}

/** Poll GET /health */
export function useHealth(intervalMs: number = 5000) {
    const fetcher = useCallback(async (): Promise<HealthResponse> => {
        const res = await fetch(`${BASE_URL}/health`);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
    }, []);

    return usePolling(fetcher, intervalMs);
}

/** Poll GET /processes/list */
export function useProcessList(intervalMs: number = 10000) {
    const fetcher = useCallback(async (): Promise<ProcessListResponse> => {
        const res = await fetch(`${BASE_URL}/processes/list`);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
    }, []);

    return usePolling(fetcher, intervalMs);
}

/** Poll GET /process/{processId}/status */
export function useProcessStatus(processId: string | null, intervalMs: number = 3000) {
    const fetcher = useCallback(async () => {
        if (!processId) return null;
        const res = await fetch(`${BASE_URL}/process/${encodeURIComponent(processId)}/status`);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
    }, [processId]);

    return usePolling(fetcher, intervalMs, !!processId);
}

/** Fetch GET /ai/models (one-shot, not polling) */
export function useAIModels() {
    const fetcher = useCallback(async (): Promise<AIModelsResponse> => {
        const res = await fetch(`${BASE_URL}/ai/models`);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
    }, []);

    return usePolling(fetcher, 30000); // refresh every 30s
}

/** Poll GET /process/{processId} - process snapshot with operation mode etc. */
export function useProcessSnapshot(processId: string | null, intervalMs: number = 5000) {
    const fetcher = useCallback(async (): Promise<ProcessSnapshotResponse | null> => {
        if (!processId) return null;
        const res = await fetch(`${BASE_URL}/process/${encodeURIComponent(processId)}`);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
    }, [processId]);

    return usePolling(fetcher, intervalMs, !!processId);
}

/** Poll GET /process/{processId}/target - target geometry */
export function useTargetGeometry(processId: string | null, intervalMs: number = 5000) {
    const fetcher = useCallback(async (): Promise<TargetGeometryResponse | null> => {
        if (!processId) return null;
        const res = await fetch(`${BASE_URL}/process/${encodeURIComponent(processId)}/target`);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
    }, [processId]);

    return usePolling(fetcher, intervalMs, !!processId);
}

// ---------------------------------------------------------------------------
// Mutation helpers (fire-and-forget, return promise)
// ---------------------------------------------------------------------------

export async function selectModel(path: string) {
    const res = await fetch(`${BASE_URL}/ai/models/select`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ path }),
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
}

export async function predictGeometry(params: { wireSpeed: number; current: number; voltage: number }) {
    const res = await fetch(`${BASE_URL}/ai/models/predict`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params),
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
}

export async function stopProcess(processId: string, reason: string = 'operator_request') {
    const res = await fetch(`${BASE_URL}/process/${encodeURIComponent(processId)}/stop?reason=${encodeURIComponent(reason)}`, {
        method: 'POST',
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
}

export async function resumeProcess(processId: string) {
    const res = await fetch(`${BASE_URL}/process/${encodeURIComponent(processId)}/resume`, {
        method: 'POST',
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
}

export async function setTarget(processId: string, height: number, width: number) {
    const res = await fetch(`${BASE_URL}/process/${encodeURIComponent(processId)}/target`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ height, width }),
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
}

export async function setProcessMode(processId: string, mode: 'parameter_driven' | 'geometry_driven') {
    const res = await fetch(`${BASE_URL}/process/${encodeURIComponent(processId)}/mode`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode }),
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
}

export async function checkDeviation(request: DeviationCheckRequest): Promise<DeviationCheckResponse> {
    const res = await fetch(`${BASE_URL}/check-deviation`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(request),
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
}

export async function getAIRecommendation(request: AIRecommendationRequest): Promise<AIRecommendationResponse> {
    const res = await fetch(`${BASE_URL}/ai-recommendation`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(request),
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
}

export async function fetchTargetGeometry(processId: string): Promise<TargetGeometryResponse> {
    const res = await fetch(`${BASE_URL}/process/${encodeURIComponent(processId)}/target`);
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
}

export async function createProcess(processId: string, mode: string = 'parameter_driven') {
    const res = await fetch(`${BASE_URL}/create-process`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ process_id: processId, mode }),
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
}
