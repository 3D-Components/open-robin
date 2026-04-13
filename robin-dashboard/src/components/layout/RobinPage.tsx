import { useState, useEffect, useMemo, useCallback, useRef } from 'react';
import { Chip } from '../ui/Chip';
import { Button } from '../ui/Button';
import { TopBar } from './TopBar';
import { Sidebar } from './Sidebar';
import { Modal } from '../ui/Modal';
import { LiveOps } from '../features/live-ops/LiveOps';
import { RobotsTab } from '../features/robots/RobotsTab';
import { ModelsTrustTab } from '../features/models/ModelsTrustTab';
import { HistoryTab } from '../features/history/HistoryTab';
import { SettingsTab } from '../features/settings/SettingsTab';
import {
    useMeasurements,
    useHealth,
    useProcessList,
    useAIModels,
    useProcessSnapshot,
    useTargetGeometry,
    stopProcess,
    resumeProcess,
    setTarget,
    setProcessMode,
    setInputParams,
    getAIRecommendation,
    publishRosIntent,
    type RobinMeasurement,
    type AIRecommendationResponse,
} from '../../hooks/useRobinAPI';
import type {
    AIInputFeatureSpec,
    AIInputParams,
    TabKey,
    RobotCell,
    ProcessRun,
    TrustAssessment,
    MetricType,
    ConnStatus,
    MeasurementPoint,
    Alert,
    ModelVersion,
    AuditLogEntry,
    ProcessControlsState,
    OperationMode,
    TargetGeometry,
    DeviationAction,
} from '../../types';
import { domainTerms } from '../../config/domain';
import { useProfile } from '../../config/ProfileContext';
import {
    buildDefaultAIInputParams,
    formatAIInputSummary,
    mergeRecommendedAIInputParams,
    normalizeAIInputParams,
    resolveAIInputFeatures,
    resolveRecordedAIInputParams,
} from '../../config/aiInputFeatures';

const ts = (offsetMinutes: number) =>
    new Date(Date.now() - offsetMinutes * 60000).toISOString();

function shortId(prefix: string) {
    return `${prefix}-${Math.random().toString(16).slice(2, 8).toUpperCase()}`;
}

function nowIso() {
    return new Date().toISOString();
}

function gateForConfidence(conf: number, warnTh: number, stopTh: number): 'OK' | 'Warning' | 'Stop' {
    if (conf < stopTh) return 'Stop';
    if (conf < warnTh) return 'Warning';
    return 'OK';
}

function latencyTone(
    connected: boolean,
    latencyMs: number | null | undefined,
): 'good' | 'warn' | 'bad' {
    if (!connected) return 'bad';
    if (latencyMs === null || latencyMs === undefined || Number.isNaN(latencyMs)) return 'warn';
    if (latencyMs <= 250) return 'good';
    if (latencyMs <= 1000) return 'warn';
    return 'bad';
}

const initialRobot: RobotCell = {
    id: 'robot',
    name: 'Robot',
    state: 'Idle',
    isParamFrozen: false,
    currentRunId: undefined,
    taskProgressPct: 0,
    segmentIndex: 0,
    activeModel: '—',
    lastTrustScore: 0.96,
    lastDeploymentAt: ts(610),
    lastInferenceAt: ts(0),
};

const mockModels: ModelVersion[] = [
    {
        id: 'process_geometry_mlp.pt',
        createdAt: ts(610),
        artifactPath: '/app/data/models/welding/process_geometry_mlp.pt',
        trustScore: 0.96,
        goNoGo: 'Go',
        notes: 'Stable process geometry; low OOD events.',
    },
];

const mockAuditLog: AuditLogEntry[] = [
    {
        id: 'audit-001',
        at: ts(5),
        actor: 'System',
        action: 'ROBIN UI started',
        severity: 'Info',
    },
];

function apiToMeasurementPoints(
    measurements: RobinMeasurement[],
    aiInputFeatures: AIInputFeatureSpec[],
): MeasurementPoint[] {
    let firstValidTimestampMs: number | null = null;
    return measurements.map((m, i) => {
        const tsMs = Date.parse(m.timestamp);
        const hasValidTimestamp = Number.isFinite(tsMs);
        const timeSeconds =
            hasValidTimestamp
                ? (() => {
                    if (firstValidTimestampMs === null) {
                        firstValidTimestampMs = tsMs;
                    }
                    return (tsMs - firstValidTimestampMs) / 1000;
                })()
                : i;

        return {
            t: timeSeconds,
            timestamp: hasValidTimestamp ? m.timestamp : null,
            inputParams: resolveRecordedAIInputParams(
                m as unknown as Record<string, unknown>,
                aiInputFeatures,
            ),
            profileHeight: m.height ?? 0,
            profileWidth: m.width ?? 0,
            speed: m.speed,
            current: m.current,
            voltage: m.voltage,
            confidence: 0.95,
        };
    });
}

function getMetricValue(point: MeasurementPoint, metric: MetricType): number {
    if (metric === 'profileHeight') return point.profileHeight;
    if (metric === 'profileWidth') return point.profileWidth;
    if (metric === 'measuredSpeed') return point.speed ?? 0;
    if (metric === 'measuredCurrent') return point.current ?? 0;
    if (metric === 'measuredVoltage') return point.voltage ?? 0;
    return point.inputParams[metric] ?? 0;
}

function inputParamsToIntentParameters(
    inputParams: AIInputParams,
    aiInputFeatures: AIInputFeatureSpec[],
) {
    return aiInputFeatures.map((feature) => ({
        parameter_name: feature.key,
        new_value: inputParams[feature.key] ?? 0,
        unit: feature.unit || '',
    }));
}

function toNumber(value: unknown): number | null {
    if (typeof value !== 'number' || Number.isNaN(value)) return null;
    return value;
}

type StartPreviewPlan =
    | {
        mode: 'parameter_driven';
        processId: string;
        inputParams: AIInputParams;
        predictedGeometry: { height: number; width: number };
        confidence: number | null;
    }
    | {
        mode: 'geometry_driven';
        processId: string;
        targetGeometry: { height: number; width: number };
        recommendedParams: AIInputParams;
        predictedGeometry: { height: number; width: number } | null;
        confidence: number | null;
    };

type JobMonitoringState = {
    active: boolean;
    totalPoints: number;
    warningPoints: number;
};

type JobReport = {
    totalPoints: number;
    warningPoints: number;
    warningRate: number;
    recommendNewDoe: boolean;
};

type ManualAdjustDraft = AIInputParams;

type AiRecommendationPlan = {
    targetHeight: number;
    targetWidth: number;
    recommendedParams: AIInputParams | null;
    predictedGeometry: { height: number; width: number } | null;
    confidence: number | null;
};

function getRecommendationError(rec: AIRecommendationResponse): string {
    return rec.error ?? rec.message ?? 'AI recommendation failed';
}

function assertRecommendationSuccess(
    rec: AIRecommendationResponse,
): AIRecommendationResponse & { status: 'success'; recommendation: NonNullable<AIRecommendationResponse['recommendation']> } {
    if (rec.status !== 'success' || !rec.recommendation) {
        throw new Error(getRecommendationError(rec));
    }
    return rec as AIRecommendationResponse & {
        status: 'success';
        recommendation: NonNullable<AIRecommendationResponse['recommendation']>;
    };
}

export function RobinPage() {
    const { data: profileData } = useProfile();
    const aiInputFeatures = useMemo<AIInputFeatureSpec[]>(
        () => resolveAIInputFeatures(profileData),
        [profileData],
    );
    const [tab, setTab] = useState<TabKey>('live');
    const [sessionMode, setSessionMode] = useState<'Active Run' | 'Demo Mode'>(() => {
        if (typeof window === 'undefined') return 'Active Run';
        const stored = window.localStorage.getItem('robin.session_mode');
        return stored === 'Demo Mode' || stored === 'Active Run' ? stored : 'Active Run';
    });

    const [processId, setProcessId] = useState<string | null>('ros_bridge');
    const isDemoMode = sessionMode === 'Demo Mode';
    const measurementPollMs = isDemoMode ? 2000 : 1000;

    const { data: measurementsData } = useMeasurements(processId, 200, measurementPollMs);
    const { data: healthData, error: healthError } = useHealth(5000);
    const { data: processListData } = useProcessList(10000);
    const { data: aiModelsData, refetch: refetchModels } = useAIModels();
    const { data: processSnapshotData } = useProcessSnapshot(processId, 5000);
    const { data: targetGeometryData } = useTargetGeometry(processId, 5000);

    useEffect(() => {
        if (typeof window !== 'undefined') {
            window.localStorage.setItem('robin.session_mode', sessionMode);
        }
    }, [sessionMode]);

    const apiReachable = !!healthData && !healthError;

    const [processControls, setProcessControls] = useState<ProcessControlsState>({
        mode: 'parameter_driven',
        tolerance: 10,
        inputParams: buildDefaultAIInputParams(aiInputFeatures),
        targetHeight: 3.0,
        targetWidth: 6.0,
    });

    useEffect(() => {
        setProcessControls((prev) => ({
            ...prev,
            inputParams: normalizeAIInputParams(prev.inputParams, aiInputFeatures),
        }));
    }, [aiInputFeatures]);

    const processMode: OperationMode | null = useMemo(() => {
        if (processSnapshotData?.operationMode?.value) {
            const v = processSnapshotData.operationMode.value;
            if (v === 'parameter_driven' || v === 'geometry_driven') return v;
        }
        return null;
    }, [processSnapshotData]);

    const targetGeometry: TargetGeometry | null = useMemo(() => {
        if (targetGeometryData?.status === 'success' && targetGeometryData.target_geometry) {
            return targetGeometryData.target_geometry;
        }
        return null;
    }, [targetGeometryData]);

    useEffect(() => {
        const snapshotInputParams = processSnapshotData?.inputParams?.value;
        const snapshotTolerance = processSnapshotData?.toleranceThreshold?.value;
        if (
            (!snapshotInputParams || typeof snapshotInputParams !== 'object')
            && !(typeof snapshotTolerance === 'number' && Number.isFinite(snapshotTolerance))
        ) {
            return;
        }

        const signature = JSON.stringify({
            processId,
            inputParams: snapshotInputParams ?? null,
            tolerance:
                typeof snapshotTolerance === 'number' && Number.isFinite(snapshotTolerance)
                    ? snapshotTolerance
                    : null,
        });
        if (
            lastHydratedProcessIdRef.current === processId
            && lastHydratedControlsSignatureRef.current === signature
        ) {
            return;
        }

        setProcessControls((prev) => ({
            ...prev,
            inputParams:
                snapshotInputParams && typeof snapshotInputParams === 'object'
                    ? normalizeAIInputParams(
                        snapshotInputParams as AIInputParams,
                        aiInputFeatures,
                    )
                    : prev.inputParams,
            tolerance:
                typeof snapshotTolerance === 'number' && Number.isFinite(snapshotTolerance)
                    ? snapshotTolerance
                    : prev.tolerance,
        }));
        lastHydratedProcessIdRef.current = processId;
        lastHydratedControlsSignatureRef.current = signature;
    }, [aiInputFeatures, processSnapshotData?.inputParams, processSnapshotData?.toleranceThreshold]);

    const connections = useMemo<Record<string, ConnStatus>>(() => {
        if (!apiReachable) {
            return {
                'Backend API': 'red',
                'Orion-LD': 'yellow',
                'Mintaka': 'yellow',
            };
        }

        return {
            'Backend API': healthData?.status === 'healthy' ? 'green' : 'yellow',
            'Orion-LD': healthData?.orion_connected ? 'green' : 'red',
            'Mintaka': healthData?.mintaka_connected ? 'green' : 'red',
        };
    }, [apiReachable, healthData?.status, healthData?.orion_connected, healthData?.mintaka_connected]);

    const [robot, setRobot] = useState<RobotCell>(initialRobot);
    const [currentRun, _setCurrentRun] = useState<ProcessRun | null>(null);

    const [alerts, setAlerts] = useState<Alert[]>([
        { id: 'alert-001', at: nowIso(), severity: 'Info', message: 'ROBIN UI started. Waiting for live data…', source: 'System' },
    ]);
    const [jobMonitoring, setJobMonitoring] = useState<JobMonitoringState>({
        active: false,
        totalPoints: 0,
        warningPoints: 0,
    });
    const [jobReport, setJobReport] = useState<JobReport | null>(null);
    const jobMonitoringRef = useRef<JobMonitoringState>(jobMonitoring);
    const prevRobotStateRef = useRef(initialRobot.state);
    const lastMeasurementCountRef = useRef(0);
    const lastMeasurementAtRef = useRef(Date.now());
    const inactivityCompletionRequestedRef = useRef(false);
    const operatorPauseHoldRef = useRef(false);
    const lastHydratedProcessIdRef = useRef<string | null>(null);
    const lastHydratedControlsSignatureRef = useRef<string | null>(null);
    // Guards against duplicate intent publishes from rapid double-clicks.
    const intentInFlightRef = useRef(false);
    const [startPlanning, setStartPlanning] = useState(false);
    const [startPreviewPlan, setStartPreviewPlan] = useState<StartPreviewPlan | null>(null);
    const [manualAdjustDraft, setManualAdjustDraft] = useState<ManualAdjustDraft | null>(null);
    const [aiRecommendationPlan, setAiRecommendationPlan] = useState<AiRecommendationPlan | null>(null);
    const [aiRecommendationLoading, setAiRecommendationLoading] = useState(false);
    const [aiRecommendationError, setAiRecommendationError] = useState<string | null>(null);
    const [warningGateResetToken, setWarningGateResetToken] = useState(0);

    const pushAlert = useCallback((sev: 'Info' | 'Warning' | 'Critical', msg: string, src: string) => {
        setAlerts((a) =>
            [{ id: shortId('ALR'), at: nowIso(), severity: sev, message: msg, source: src }, ...a].slice(0, 8)
        );
    }, []);

    useEffect(() => {
        jobMonitoringRef.current = jobMonitoring;
    }, [jobMonitoring]);

    const beginJobMonitoring = useCallback(() => {
        operatorPauseHoldRef.current = false;
        lastMeasurementCountRef.current = measurementsData?.count ?? 0;
        lastMeasurementAtRef.current = Date.now();
        inactivityCompletionRequestedRef.current = false;
        setJobMonitoring({ active: true, totalPoints: 0, warningPoints: 0 });
        setJobReport(null);
    }, [measurementsData?.count]);

    const finalizeJobMonitoring = useCallback(() => {
        const stats = jobMonitoringRef.current;
        if (!stats.active) return;
        inactivityCompletionRequestedRef.current = false;
        const warningRate = stats.totalPoints > 0 ? stats.warningPoints / stats.totalPoints : 0;
        setJobReport({
            totalPoints: stats.totalPoints,
            warningPoints: stats.warningPoints,
            warningRate,
            recommendNewDoe: stats.totalPoints > 0 && warningRate > 0.2,
        });
        setJobMonitoring((prev) => ({ ...prev, active: false }));
    }, []);

    const handleDeviationPointEvaluated = useCallback((warningFlagged: boolean) => {
        setJobMonitoring((prev) => {
            if (!prev.active) return prev;
            return {
                ...prev,
                totalPoints: prev.totalPoints + 1,
                warningPoints: prev.warningPoints + (warningFlagged ? 1 : 0),
            };
        });
    }, []);

    useEffect(() => {
        operatorPauseHoldRef.current = false;
        lastMeasurementCountRef.current = measurementsData?.count ?? 0;
        lastMeasurementAtRef.current = Date.now();
        inactivityCompletionRequestedRef.current = false;
    }, [processId]);

    useEffect(() => {
        const count = measurementsData?.count ?? 0;
        if (count !== lastMeasurementCountRef.current) {
            lastMeasurementCountRef.current = count;
            lastMeasurementAtRef.current = Date.now();
        }
    }, [measurementsData?.count]);

    useEffect(() => {
        pushAlert(
            'Info',
            sessionMode === 'Active Run'
                ? 'Session is Active Run: backend write actions are enabled.'
                : 'Session is Demo Mode: backend write actions are disabled (simulation only).',
            'Session',
        );
    }, [sessionMode, pushAlert]);

    const handleControlsApply = useCallback(async (ctrl: ProcessControlsState) => {
        if (!processId) return;
        if (sessionMode === 'Demo Mode') {
            pushAlert(
                'Info',
                `Demo Mode: controls updated locally only (${ctrl.mode}, tolerance ${ctrl.tolerance}%). No backend write executed.`,
                'Process Controls',
            );
            return;
        }

        pushAlert('Info', `Applying controls (${ctrl.mode}, tolerance ${ctrl.tolerance}%)`, 'Process Controls');

        try {
            await setProcessMode(processId, ctrl.mode);
        } catch {
            pushAlert('Warning', 'Failed to persist operation mode on backend', 'Process Controls');
        }

        try {
            await setInputParams(processId, ctrl.inputParams);
        } catch {
            pushAlert('Warning', 'Failed to persist process input parameters on backend', 'Process Controls');
        }

        if (ctrl.mode === 'geometry_driven') {
            try {
                await setTarget(processId, ctrl.targetHeight, ctrl.targetWidth);
                pushAlert(
                    'Info',
                    `Target geometry set: ${ctrl.targetHeight.toFixed(2)} x ${ctrl.targetWidth.toFixed(2)} mm`,
                    'Process Controls',
                );
            } catch {
                pushAlert('Warning', 'Failed to set target geometry on backend', 'Process Controls');
                return;
            }

            try {
                const recRaw = await getAIRecommendation({
                    process_id: processId,
                    mode: 'geometry_driven',
                    target_geometry: {
                        height: ctrl.targetHeight,
                        width: ctrl.targetWidth,
                    },
                });
                const rec = assertRecommendationSuccess(recRaw);
                const recommended = rec.recommendation.recommended_params as Record<string, unknown> | undefined;
                if (!recommended || typeof recommended !== 'object') {
                    throw new Error('AI response missing recommended parameters');
                }

                const nextInputParams = mergeRecommendedAIInputParams(
                    recommended,
                    ctrl.inputParams,
                    aiInputFeatures,
                );
                setProcessControls((prev) => ({
                    ...prev,
                    inputParams: nextInputParams,
                }));
                pushAlert(
                    'Info',
                    `AI suggested inputs: ${formatAIInputSummary(nextInputParams, aiInputFeatures)}`,
                    'AI Recommendation',
                );
            } catch (err) {
                const message = err instanceof Error ? err.message : 'Failed to fetch AI recommendation for geometry target';
                pushAlert('Warning', message, 'AI Recommendation');
            }
            return;
        }

        try {
            const recRaw = await getAIRecommendation({
                process_id: processId,
                mode: 'parameter_driven',
                input_params: ctrl.inputParams,
            });
            const rec = assertRecommendationSuccess(recRaw);
            const pred = rec.recommendation.predicted_geometry;
            const height = toNumber(pred?.height);
            const width = toNumber(pred?.width);
            if (height === null || width === null) {
                throw new Error('AI response missing predicted geometry');
            }
            pushAlert(
                'Info',
                `AI predicts geometry ${height.toFixed(2)} x ${width.toFixed(2)} mm from the current AI inputs`,
                'AI Recommendation',
            );
        } catch (err) {
            const message = err instanceof Error ? err.message : 'Failed to fetch AI geometry prediction';
            pushAlert('Warning', message, 'AI Recommendation');
        }
    }, [aiInputFeatures, processId, pushAlert, setProcessControls, sessionMode]);

    const runGeometryRecommendation = useCallback(async (targetHeight: number, targetWidth: number) => {
        if (!processId) {
            throw new Error('No process selected');
        }

        const recRaw = await getAIRecommendation({
            process_id: processId,
            mode: 'geometry_driven',
            target_geometry: {
                height: targetHeight,
                width: targetWidth,
            },
        });
        const rec = assertRecommendationSuccess(recRaw);
        const recommended = rec.recommendation.recommended_params as Record<string, unknown> | undefined;
        if (!recommended || typeof recommended !== 'object') {
            throw new Error('AI response missing recommended parameters');
        }

        const nextInputParams = mergeRecommendedAIInputParams(
            recommended,
            processControls.inputParams,
            aiInputFeatures,
        );
        const predictedHeight =
            toNumber(rec.recommendation.predicted_geometry?.height)
            ?? toNumber(recommended.predictedHeight);
        const predictedWidth =
            toNumber(rec.recommendation.predicted_geometry?.width)
            ?? toNumber(recommended.predictedWidth);
        const predictedGeometry =
            predictedHeight === null || predictedWidth === null
                ? null
                : { height: predictedHeight, width: predictedWidth };

        return {
            recommendedParams: nextInputParams,
            predictedGeometry,
            confidence: toNumber(rec.recommendation.confidence) ?? toNumber(recommended.confidence),
        };
    }, [aiInputFeatures, processControls.inputParams, processId]);

    const requestAiRecommendation = useCallback(async (targetHeight: number, targetWidth: number) => {
        setAiRecommendationLoading(true);
        setAiRecommendationError(null);
        try {
            const recommendation = await runGeometryRecommendation(targetHeight, targetWidth);
            setAiRecommendationPlan((prev) => {
                if (!prev) return null;
                return {
                    ...prev,
                    targetHeight,
                    targetWidth,
                    ...recommendation,
                };
            });
            pushAlert(
                'Info',
                `AI suggested inputs: ${formatAIInputSummary(recommendation.recommendedParams, aiInputFeatures)}`,
                'AI Recommendation',
            );
        } catch (err) {
            const message = err instanceof Error ? err.message : 'Failed to fetch AI recommendation for geometry target';
            setAiRecommendationError(message);
            setAiRecommendationPlan((prev) => {
                if (!prev) return null;
                return {
                    ...prev,
                    targetHeight,
                    targetWidth,
                    recommendedParams: null,
                    predictedGeometry: null,
                    confidence: null,
                };
            });
            pushAlert('Warning', message, 'AI Recommendation');
        } finally {
            setAiRecommendationLoading(false);
        }
    }, [pushAlert, runGeometryRecommendation]);

    const handleDeviationAction = useCallback((action: DeviationAction) => {
        if (action === 'manual_adjust') {
            setManualAdjustDraft({
                ...processControls.inputParams,
            });
            pushAlert('Info', 'Manual adjust opened. Update parameters and apply to continue.', 'Deviation Monitor');
            return;
        }

        if (action === 'new_ai_recommendation') {
            const targetHeight = processControls.targetHeight;
            const targetWidth = processControls.targetWidth;
            setAiRecommendationPlan({
                targetHeight,
                targetWidth,
                recommendedParams: null,
                predictedGeometry: null,
                confidence: null,
            });
            setAiRecommendationError(null);
            pushAlert('Info', 'Preparing AI recommendation for geometry target…', 'Deviation Monitor');
            void requestAiRecommendation(targetHeight, targetWidth);
            return;
        }

        pushAlert('Info', 'Starting new Design of Experiments', 'Deviation Monitor');
        publishRosIntent('LAUNCH_NEW_DOE', {
            seam_id: 'seam_01', weld_speed: 5.0, wire_feed: 4.0,
        }, processId ?? 'ros_bridge')
            .then(() => pushAlert('Info', 'LAUNCH_NEW_DOE intent published', 'Intent Bridge'))
            .catch(() => {});
    }, [processControls, processId, pushAlert, requestAiRecommendation]);

    const [activeModel, setActiveModel] = useState<string>('—');

    useEffect(() => {
        if (aiModelsData?.active_model) {
            const modelName = aiModelsData.active_model.name ?? aiModelsData.active_model.path;
            setActiveModel(modelName);
            setRobot((prev) => ({ ...prev, activeModel: modelName }));
        }
    }, [aiModelsData]);

    const [trustFeed, setTrustFeed] = useState<TrustAssessment[]>([]);

    const [timelineT, setTimelineT] = useState(0);

    const [telemetry, setTelemetry] = useState<MeasurementPoint[]>([]);
    const [metric, setMetric] = useState<MetricType>('profileHeight');
    const [freezeCharts, setFreezeCharts] = useState(false);
    const metricOptions = useMemo(
        () => [
            { value: 'measuredSpeed', label: `Metric: ${domainTerms.speed}` },
            { value: 'measuredCurrent', label: `Metric: ${domainTerms.current}` },
            { value: 'measuredVoltage', label: `Metric: ${domainTerms.voltage}` },
            ...aiInputFeatures.map((feature) => ({
                value: feature.key,
                label: `Metric: ${feature.label}`,
            })),
            { value: 'profileHeight', label: `Metric: ${domainTerms.profileHeight}` },
            { value: 'profileWidth', label: `Metric: ${domainTerms.profileWidth}` },
        ],
        [aiInputFeatures],
    );
    const metricLabels = useMemo(() => {
        const entries: Array<[string, string]> = [
            ['measuredSpeed', `${domainTerms.speed} (${domainTerms.speedUnit})`],
            ['measuredCurrent', `${domainTerms.current} (${domainTerms.currentUnit})`],
            ['measuredVoltage', `${domainTerms.voltage} (${domainTerms.voltageUnit})`],
            ...aiInputFeatures.map((feature) => [
                feature.key,
                feature.unit ? `${feature.label} (${feature.unit})` : feature.label,
            ] as [string, string]),
        ];
        entries.push(['profileHeight', `${domainTerms.profileHeight} (mm)`]);
        entries.push(['profileWidth', `${domainTerms.profileWidth} (mm)`]);
        return Object.fromEntries(entries) as Record<string, string>;
    }, [aiInputFeatures]);

    useEffect(() => {
        const validMetrics = new Set(metricOptions.map((option) => option.value));
        if (!validMetrics.has(metric)) {
            setMetric('profileHeight');
        }
    }, [metric, metricOptions]);

    const simulationProgress = useMemo(() => {
        const raw = processSnapshotData?.simulationProgress?.value;
        return typeof raw === 'number' && Number.isFinite(raw) ? raw : null;
    }, [processSnapshotData]);

    useEffect(() => {
        if (!measurementsData?.measurements?.length) return;
        if (freezeCharts) return;

        const points = apiToMeasurementPoints(measurementsData.measurements, aiInputFeatures);
        setTelemetry(points);
        setTimelineT(points.length ? points[points.length - 1].t : 0);
    }, [measurementsData, freezeCharts, aiInputFeatures]);

    useEffect(() => {
        if (simulationProgress === null) return;
        setRobot((prev) => ({
            ...prev,
            taskProgressPct: Math.max(0, Math.min(100, simulationProgress)),
        }));
    }, [simulationProgress]);

    const backendProcessStatus = processSnapshotData?.processStatus?.value ?? null;
    const backendStopReason =
        processSnapshotData?.stopReason?.value
        ?? processSnapshotData?.stop_reason?.value
        ?? null;

    useEffect(() => {
        if (!backendProcessStatus) return;
        setRobot((prev) => {
            const normalizedStopReason =
                typeof backendStopReason === 'string'
                    ? backendStopReason.trim()
                    : '';
            const isPauseStop = normalizedStopReason === 'operator_pause';
            const shouldTreatAsPause =
                operatorPauseHoldRef.current || isPauseStop;

            if (backendProcessStatus === 'active') {
                operatorPauseHoldRef.current = false;
            }

            if (
                backendProcessStatus === 'active'
                && (prev.state === 'Idle' || prev.state === 'Paused')
            ) {
                return { ...prev, state: 'Running' as const };
            }
            if (
                backendProcessStatus === 'stopped'
                && shouldTreatAsPause
                && (prev.state === 'Running' || prev.state === 'Paused')
            ) {
                return { ...prev, state: 'Paused' as const };
            }
            if (
                backendProcessStatus === 'stopped'
                && !shouldTreatAsPause
                && (prev.state === 'Running' || prev.state === 'Paused')
            ) {
                operatorPauseHoldRef.current = false;
                return { ...prev, state: 'Idle' as const };
            }
            return prev;
        });
    }, [backendProcessStatus, backendStopReason]);

    useEffect(() => {
        const prev = prevRobotStateRef.current;
        if (prev === 'Idle' && robot.state === 'Running') {
            beginJobMonitoring();
        }
        if ((prev === 'Running' || prev === 'Paused') && robot.state === 'Idle') {
            finalizeJobMonitoring();
        }
        prevRobotStateRef.current = robot.state;
    }, [robot.state, beginJobMonitoring, finalizeJobMonitoring]);

    useEffect(() => {
        if (!jobMonitoring.active) return;
        if (robot.state !== 'Running') return;
        const measurementCount = measurementsData?.count ?? 0;
        if (measurementCount <= 0) return;

        const inactivityMs = Math.max(measurementPollMs * 4, 6000);
        const id = setInterval(() => {
            if (inactivityCompletionRequestedRef.current) return;
            const elapsedMs = Date.now() - lastMeasurementAtRef.current;
            if (elapsedMs < inactivityMs) return;

            inactivityCompletionRequestedRef.current = true;
            operatorPauseHoldRef.current = false;
            pushAlert('Info', 'No new measurements detected; marking job as completed.', 'System');
            setRobot((prev) => {
                if (prev.state !== 'Running') return prev;
                return { ...prev, state: 'Idle' as const, taskProgressPct: 100 };
            });
        }, 1000);

        return () => clearInterval(id);
    }, [jobMonitoring.active, robot.state, measurementsData?.count, measurementPollMs, pushAlert]);

    useEffect(() => {
        if (!measurementsData?.measurements?.length) return;
        if (measurementsData.count > 0) {
            if (measurementsData.count === 1 || (telemetry.length === 0 && measurementsData.count > 0)) {
                pushAlert('Info', `Receiving live data for process "${processId}" (${measurementsData.count} measurements)`, 'Backend API');
            }
        }
    }, [measurementsData?.count]);

    const [trustWarnTh, setTrustWarnTh] = useState(0.65);
    const [trustStopTh, setTrustStopTh] = useState(0.45);
    const [latestTrust, setLatestTrust] = useState<TrustAssessment>({
        atT: timelineT,
        robotId: 'robot',
        confidence: 0.96,
        gate: 'OK',
        reasons: ['nominal'],
    });

    const connBadge = (s: ConnStatus) => (
        <Chip tone={s === 'green' ? 'good' : s === 'yellow' ? 'warn' : 'bad'}>
            {s === 'green' ? 'Green' : s === 'yellow' ? 'Yellow' : 'Red'}
        </Chip>
    );

    const orionLatencyMs = healthData?.latency_ms?.orion ?? null;
    const mintakaLatencyMs = healthData?.latency_ms?.mintaka ?? null;
    const orionLatencyTone = latencyTone(Boolean(healthData?.orion_connected), orionLatencyMs);
    const mintakaLatencyTone = latencyTone(Boolean(healthData?.mintaka_connected), mintakaLatencyMs);
    const uptimeHrs = (healthData?.uptime_seconds ?? 0) / 3600;

    const onSwitchModel = (_next: string) => {
        setActiveModel(_next);
    };

    const [darkMode, setDarkMode] = useState(true);

    useEffect(() => {
        document.documentElement.classList.toggle('dark', darkMode);
    }, [darkMode]);

    useEffect(() => {
        if (freezeCharts) return;

        const interval = setInterval(() => {
            const t = timelineT;
            const conf = Math.max(0, Math.min(1, 0.92 - Math.random() * 0.05));
            const gate = gateForConfidence(conf, trustWarnTh, trustStopTh);

            const newTrust: TrustAssessment = {
                atT: t,
                robotId: 'robot',
                confidence: conf,
                gate,
                reasons: gate === 'OK' ? ['nominal'] : ['low confidence'],
            };

            setLatestTrust(newTrust);
            setTrustFeed(prev => [...prev, newTrust].slice(-80));
        }, 2000);

        return () => clearInterval(interval);
    }, [freezeCharts, timelineT, trustWarnTh, trustStopTh]);

    const telemetryChartData = useMemo(() => {
        return telemetry.map((p) => ({
            t: p.t,
            absT: p.timestamp ? Date.parse(p.timestamp) / 1000 : p.t,
            timestamp: p.timestamp,
            value: getMetricValue(p, metric),
        }));
    }, [telemetry, metric]);

    const measurementSource = measurementsData?.debug_info?.source ?? 'none';

    const lastValue = useMemo(() => {
        const last = telemetry[telemetry.length - 1];
        if (!last) return 0;
        return getMetricValue(last, metric);
    }, [telemetry, metric]);

    const confirmAndStart = useCallback((plan: StartPreviewPlan) => {
        if (plan.mode === 'geometry_driven') {
            setProcessControls((prev) => ({
                ...prev,
                inputParams: plan.recommendedParams,
            }));
        }

        setStartPreviewPlan(null);
        setRobot((prev) => ({ ...prev, state: 'Running' as const, taskProgressPct: 0 }));
        pushAlert('Info', `${robot.name} started.`, robot.name);
        if (processId) {
            resumeProcess(processId).catch(() => {});
        }
        const params = plan.mode === 'parameter_driven' ? plan.inputParams : plan.recommendedParams;
        publishRosIntent('START_PROCESS', {
            seam_id: plan.processId,
            ...params,
        }, processId ?? 'ros_bridge')
            .then(() => pushAlert('Info', 'START_PROCESS intent published to /intents', 'Intent Bridge'))
            .catch(() => pushAlert('Warning', 'Failed to publish START_PROCESS intent', 'Intent Bridge'));
    }, [processId, publishRosIntent, pushAlert, robot.name]);

    const buildStartPreview = useCallback(async (): Promise<StartPreviewPlan> => {
        if (!processId) {
            throw new Error('No process selected');
        }

        const mode: OperationMode = processMode ?? processControls.mode;

        if (sessionMode !== 'Demo Mode') {
            await setProcessMode(processId, mode);
            if (mode === 'geometry_driven') {
                await setTarget(
                    processId,
                    processControls.targetHeight,
                    processControls.targetWidth,
                );
            }
        }

        if (mode === 'parameter_driven') {
            const inputParams = processControls.inputParams;
            const rec = assertRecommendationSuccess(await getAIRecommendation({
                process_id: processId,
                mode: 'parameter_driven',
                input_params: inputParams,
            }));
            const height = toNumber(rec.recommendation.predicted_geometry?.height);
            const width = toNumber(rec.recommendation.predicted_geometry?.width);
            if (height === null || width === null) {
                throw new Error('AI response missing predicted geometry');
            }

            return {
                mode,
                processId,
                inputParams,
                predictedGeometry: { height, width },
                confidence: toNumber(rec.recommendation.confidence),
            };
        }

        const targetGeometry = {
            height: processControls.targetHeight,
            width: processControls.targetWidth,
        };
        const rec = assertRecommendationSuccess(await getAIRecommendation({
            process_id: processId,
            mode: 'geometry_driven',
            target_geometry: targetGeometry,
        }));
        const recommended = rec.recommendation.recommended_params as Record<string, unknown> | undefined;
        if (!recommended || typeof recommended !== 'object') {
            throw new Error('AI response missing recommended parameters');
        }
        const nextInputParams = mergeRecommendedAIInputParams(
            recommended,
            processControls.inputParams,
            aiInputFeatures,
        );
        const predictedHeight = toNumber(recommended.predictedHeight);
        const predictedWidth = toNumber(recommended.predictedWidth);
        const predictedGeometry =
            predictedHeight === null || predictedWidth === null
                ? null
                : { height: predictedHeight, width: predictedWidth };

        return {
            mode,
            processId,
            targetGeometry,
            recommendedParams: nextInputParams,
            predictedGeometry,
            confidence: toNumber(rec.recommendation.confidence) ?? toNumber(recommended.confidence),
        };
    }, [aiInputFeatures, processControls, processId, processMode, sessionMode]);

    const startRobot = () => {
        if (startPlanning) return;
        if (robot.state === 'E-Stop') {
            pushAlert('Critical', `${robot.name} is in E-Stop. Reset required.`, robot.name);
            return;
        }
        operatorPauseHoldRef.current = false;
        setStartPlanning(true);
        pushAlert('Info', 'Preparing AI pre-start plan…', 'AI Pre-Start');
        void (async () => {
            try {
                const plan = await buildStartPreview();
                setStartPreviewPlan(plan);
                pushAlert('Info', 'AI pre-start plan is ready. Review and confirm.', 'AI Pre-Start');
            } catch (err) {
                const message = err instanceof Error ? err.message : 'AI pre-start planning failed';
                pushAlert('Warning', `Start blocked: ${message}`, 'AI Pre-Start');
            } finally {
                setStartPlanning(false);
            }
        })();
    };
    const pauseRobot = () => {
        if (robot.state !== 'Running') return;
        operatorPauseHoldRef.current = true;
        setRobot((prev) => ({ ...prev, state: 'Paused' as const }));
        pushAlert('Warning', `${robot.name} paused by operator.`, robot.name);
        if (processId) {
            stopProcess(processId, 'operator_pause').catch(() => {});
        }
        publishRosIntentOnce('PAUSE_PROCESS', { reason: 'operator_pause' }, processId ?? 'ros_bridge');
        pushAlert('Info', 'PAUSE_PROCESS intent published', 'Intent Bridge');
    };
    const resumeRobotHandler = () => {
        if (robot.state !== 'Paused') return;
        // Keep pause-hold until backend reports active again.
        operatorPauseHoldRef.current = true;
        setRobot((prev) => ({ ...prev, state: 'Running' as const }));
        pushAlert('Info', `${robot.name} resumed.`, robot.name);
        if (processId) {
            resumeProcess(processId).catch(() => {});
        }
        publishRosIntentOnce('RESUME_PROCESS', { reason: 'operator_resume' }, processId ?? 'ros_bridge');
        pushAlert('Info', 'RESUME_PROCESS intent published', 'Intent Bridge');
    };
    const applyManualAdjustAndContinue = useCallback(async () => {
        if (!manualAdjustDraft) return;
        const nextControls: ProcessControlsState = {
            ...processControls,
            mode: 'parameter_driven',
            inputParams: manualAdjustDraft,
        };

        setProcessControls(nextControls);
        await handleControlsApply(nextControls);
        setManualAdjustDraft(null);
        publishRosIntent('MANUAL_ADJUST', {
<<<<<<< HEAD
            parameters: inputParamsToIntentParameters(
                nextControls.inputParams,
                aiInputFeatures,
            ),
=======
            parameters: Object.entries(nextControls.inputParams).map(([key, value]) => ({
                parameter_name: key,
                new_value: value,
                unit: aiInputFeatures.find((f) => f.key === key)?.unit ?? '',
            })),
>>>>>>> 4e30a37 (update Intent data payload messages)
        }, processId ?? 'ros_bridge')
            .then(() => pushAlert('Info', 'MANUAL_ADJUST intent published', 'Intent Bridge'))
            .catch(() => {});
        setAiRecommendationError(null);
        setWarningGateResetToken((prev) => prev + 1);

        if (robot.state === 'Paused') {
            resumeRobotHandler();
        }
    }, [aiInputFeatures, handleControlsApply, manualAdjustDraft, processControls, processId, publishRosIntent, pushAlert, robot.state, resumeRobotHandler]);

    const applyAiRecommendationAndContinue = useCallback(async () => {
        if (!aiRecommendationPlan) return;
        if (!aiRecommendationPlan.recommendedParams) {
            setAiRecommendationError('Generate an AI recommendation before applying.');
            return;
        }

        const nextControls: ProcessControlsState = {
            ...processControls,
            mode: 'geometry_driven',
            targetHeight: aiRecommendationPlan.targetHeight,
            targetWidth: aiRecommendationPlan.targetWidth,
            inputParams: aiRecommendationPlan.recommendedParams,
        };

        setProcessControls(nextControls);

        if (sessionMode !== 'Demo Mode' && processId) {
            try {
                await setProcessMode(processId, 'geometry_driven');
                await setTarget(
                    processId,
                    aiRecommendationPlan.targetHeight,
                    aiRecommendationPlan.targetWidth,
                );
            } catch {
                pushAlert('Warning', 'Failed to persist AI recommendation on backend', 'AI Recommendation');
            }
        } else {
            pushAlert(
                'Info',
                'Demo Mode: AI recommendation applied locally only. No backend write executed.',
                'AI Recommendation',
            );
        }

        pushAlert(
            'Info',
            `Applied AI recommendation: ${formatAIInputSummary(nextControls.inputParams, aiInputFeatures)}`,
            'AI Recommendation',
        );

        setAiRecommendationPlan(null);
        publishRosIntent('REQUEST_AI_RECOMMENDATION', {
            process_id: processId ?? '',
            mode: 'geometry_driven',
<<<<<<< HEAD
            parameters: inputParamsToIntentParameters(
                nextControls.inputParams,
                aiInputFeatures,
            ),
=======
            parameters: Object.entries(nextControls.inputParams).map(([key, value]) => ({
                parameter_name: key,
                new_value: value,
                unit: aiInputFeatures.find((f) => f.key === key)?.unit ?? '',
            })),
>>>>>>> 4e30a37 (update Intent data payload messages)
        }, processId ?? 'ros_bridge')
            .then(() => pushAlert('Info', 'REQUEST_AI_RECOMMENDATION intent published', 'Intent Bridge'))
            .catch(() => {});
        setAiRecommendationError(null);
        setWarningGateResetToken((prev) => prev + 1);
        if (robot.state === 'Paused') {
            resumeRobotHandler();
        }
    }, [aiInputFeatures, aiRecommendationPlan, processControls, processId, publishRosIntent, pushAlert, robot.state, sessionMode, resumeRobotHandler]);

    const publishRosIntentOnce = (intent: string, data: Record<string, unknown>, pid: string) => {
        if (intentInFlightRef.current) return;
        intentInFlightRef.current = true;
        const resetAfterMs = 1500;
        const reset = () => { intentInFlightRef.current = false; };
        publishRosIntent(intent, data, pid).then(reset, reset);
        setTimeout(reset, resetAfterMs);
    };

    const abortRobot = () => {
        operatorPauseHoldRef.current = false;
        setRobot((prev) => ({ ...prev, state: 'Idle' as const, taskProgressPct: 0, segmentIndex: 0 }));
        pushAlert('Critical', `${robot.name} aborted.`, robot.name);
        if (processId) {
            stopProcess(processId, 'operator_abort').catch(() => {});
        }
        // Publish ESTOP intent — cancels all active skill goals on the ROS2 side
        publishRosIntentOnce('ESTOP', { reason: 'operator_button' }, processId ?? 'ros_bridge');
        pushAlert('Info', 'ESTOP intent published to /intents', 'Intent Bridge');
    };
    const stopRobot = () => {
        operatorPauseHoldRef.current = false;
        setRobot((prev) => ({ ...prev, state: 'Idle' as const, taskProgressPct: 0, segmentIndex: 0 }));
        pushAlert('Warning', `${robot.name} stopped by operator.`, robot.name);
        if (processId) {
            stopProcess(processId, 'operator_stop').catch(() => {});
        }
        publishRosIntentOnce('STOP_PROCESS', { reason: 'operator_stop' }, processId ?? 'ros_bridge');
        pushAlert('Info', 'STOP_PROCESS intent published', 'Intent Bridge');
    };
    const toggleParamFreeze = () => {
        setRobot((prev) => ({ ...prev, isParamFrozen: !prev.isParamFrozen }));
    };

    const aiModelsList: ModelVersion[] = useMemo(() => {
        if (!aiModelsData?.models) return mockModels;
        return aiModelsData.models.map((m) => ({
            id: m.name ?? m.path,
            createdAt: m.created ?? ts(0),
            artifactPath: m.path,
            trustScore: 0.90,
            goNoGo: 'Go' as const,
        }));
    }, [aiModelsData]);

    const availableProcessIds = useMemo(() => {
        if (!processListData?.processes) return ['ros_bridge'];
        const ids = processListData.processes.map((p) => p.process_id);
        if (ids.length === 0) return ['ros_bridge'];
        return ids;
    }, [processListData]);

    const selectedProcessTolerance = useMemo<number | null>(() => {
        if (!processId || !processListData?.processes?.length) return null;
        const selected = processListData.processes.find((p) => p.process_id === processId);
        return selected?.tolerance ?? null;
    }, [processId, processListData]);

    useEffect(() => {
        if (!processId && availableProcessIds.length > 0) {
            setProcessId(availableProcessIds[0]);
        }
    }, [availableProcessIds, processId]);

    return (
        <div className="flex h-screen min-h-0 min-w-0 flex-col bg-slate-50 text-slate-900 dark:bg-slate-950 dark:text-slate-100">
            <TopBar
                dark={darkMode}
                setDark={setDarkMode}
                sessionMode={sessionMode}
                setSessionMode={setSessionMode}
                connections={connections}
                connBadge={connBadge}
                orionLatencyMs={orionLatencyMs}
                orionLatencyTone={orionLatencyTone}
                mintakaLatencyMs={mintakaLatencyMs}
                mintakaLatencyTone={mintakaLatencyTone}
                uptimeHrs={uptimeHrs}
                processId={processId}
                availableProcessIds={availableProcessIds}
                onProcessIdChange={setProcessId}
            />

            <div className="flex flex-1 min-h-0 min-w-0 overflow-hidden">
                <Sidebar tab={tab} setTab={setTab} />

                <main className="flex-1 min-h-0 min-w-0 overflow-y-auto p-4">
                    {tab === 'live' && (
                    <LiveOps
                            robot={robot}
                            latestTrust={latestTrust}
                            trustWarnTh={trustWarnTh}
                            trustStopTh={trustStopTh}
                            startRobot={startRobot}
                            startPending={startPlanning}
                            pauseRobot={pauseRobot}
                            resumeRobot={resumeRobotHandler}
                            stopRobot={stopRobot}
                            abortRobot={abortRobot}
                            toggleParamFreeze={toggleParamFreeze}
                            currentRun={currentRun}
                            alerts={alerts}
                            timelineT={timelineT}
                            telemetryChartData={telemetryChartData}
                            metric={metric}
                            setMetric={setMetric}
                            metricOptions={metricOptions}
                            freezeCharts={freezeCharts}
                            setFreezeCharts={setFreezeCharts}
                            metricLabel={metricLabels[metric] ?? metric}
                            lastValue={lastValue}
                            measurementSource={measurementSource}
                            measurementPollMs={measurementPollMs}
                            processId={processId}
                            processControls={processControls}
                            aiInputFeatures={aiInputFeatures}
                            onControlsChange={setProcessControls}
                            onControlsApply={handleControlsApply}
                            targetGeometry={targetGeometry}
                            processMode={processMode}
                            latestMeasurements={measurementsData?.measurements ?? null}
                            measurementCount={measurementsData?.count ?? 0}
                            onDeviationAlert={(severity, message) => pushAlert(severity, message, 'Deviation Monitor')}
                            onDeviationAction={handleDeviationAction}
                            onDeviationPointEvaluated={handleDeviationPointEvaluated}
                            onDeviationEscalation={pauseRobot}
                            warningGateResetToken={warningGateResetToken}
                        />
                    )}
                    {tab === 'robots' && (
                        <RobotsTab
                            robot={robot}
                            currentRun={currentRun}
                            telemetry={telemetry}
                            aiInputFeatures={aiInputFeatures}
                            trustWarnTh={trustWarnTh}
                            trustStopTh={trustStopTh}
                            startRobot={startRobot}
                            startPending={startPlanning}
                            pauseRobot={pauseRobot}
                            resumeRobot={resumeRobotHandler}
                            stopRobot={stopRobot}
                            abortRobot={abortRobot}
                            toggleParamFreeze={toggleParamFreeze}
                        />
                    )}
                    {tab === 'models' && (
                        <ModelsTrustTab
                            models={aiModelsList}
                            activeModel={activeModel}
                            aiInputFeatures={aiInputFeatures}
                            onSwitchModel={onSwitchModel}
                            audit={mockAuditLog}
                            trustWarnTh={trustWarnTh}
                            trustStopTh={trustStopTh}
                            setTrustWarnTh={setTrustWarnTh}
                            setTrustStopTh={setTrustStopTh}
                            latestTrust={latestTrust}
                            trustFeed={trustFeed}
                            aiModelsData={aiModelsData}
                            onModelsRefresh={refetchModels}
                        />
                    )}
                    {tab === 'history' && (
                        <HistoryTab
                            processId={processId}
                            availableProcessIds={availableProcessIds}
                            onProcessIdChange={setProcessId}
                            processTolerance={selectedProcessTolerance}
                            aiInputFeatures={aiInputFeatures}
                        />
                    )}
                    {tab === 'settings' && (
                        <SettingsTab
                            dark={darkMode}
                            setDark={setDarkMode}
                            connections={connections}
                            trustWarnTh={trustWarnTh}
                            trustStopTh={trustStopTh}
                            setTrustWarnTh={setTrustWarnTh}
                            setTrustStopTh={setTrustStopTh}
                            freezeCharts={freezeCharts}
                            setFreezeCharts={setFreezeCharts}
                        />
                    )}
                </main>
            </div>

            <Modal
                open={startPreviewPlan !== null}
                title="AI Pre-Start Confirmation"
                confirmText="Proceed and Start"
                onConfirm={() => {
                    if (!startPreviewPlan) return;
                    confirmAndStart(startPreviewPlan);
                }}
                onClose={() => setStartPreviewPlan(null)}
                dangerHint="AI can make mistakes. Validate recommendations before proceeding."
            >
                {startPreviewPlan ? (
                    <div className="space-y-3 text-sm">
                        <div>
                            <span className="text-slate-500 dark:text-slate-400">Process:</span>{' '}
                            <span className="font-mono">{startPreviewPlan.processId}</span>
                        </div>
                        <div>
                            <span className="text-slate-500 dark:text-slate-400">Mode:</span>{' '}
                            <span className="font-medium">
                                {startPreviewPlan.mode === 'parameter_driven' ? 'Parameter-driven' : 'Geometry-driven'}
                            </span>
                        </div>

                        {startPreviewPlan.mode === 'parameter_driven' ? (
                            <>
                                <div className="rounded-lg border border-slate-200 bg-slate-50 p-3 dark:border-slate-800 dark:bg-slate-900/40">
                                    <div className="text-xs uppercase tracking-wide text-slate-500 dark:text-slate-400">Selected parameters</div>
                                    <div className="mt-1 font-mono">
                                        {formatAIInputSummary(startPreviewPlan.inputParams, aiInputFeatures)}
                                    </div>
                                </div>
                                <div className="rounded-lg border border-slate-200 bg-slate-50 p-3 dark:border-slate-800 dark:bg-slate-900/40">
                                    <div className="text-xs uppercase tracking-wide text-slate-500 dark:text-slate-400">AI estimated geometry</div>
                                    <div className="mt-1 font-mono">
                                        height={startPreviewPlan.predictedGeometry.height.toFixed(2)} mm, width={startPreviewPlan.predictedGeometry.width.toFixed(2)} mm
                                    </div>
                                </div>
                            </>
                        ) : (
                            <>
                                <div className="rounded-lg border border-slate-200 bg-slate-50 p-3 dark:border-slate-800 dark:bg-slate-900/40">
                                    <div className="text-xs uppercase tracking-wide text-slate-500 dark:text-slate-400">Selected geometry target</div>
                                    <div className="mt-1 font-mono">
                                        height={startPreviewPlan.targetGeometry.height.toFixed(2)} mm, width={startPreviewPlan.targetGeometry.width.toFixed(2)} mm
                                    </div>
                                </div>
                                <div className="rounded-lg border border-slate-200 bg-slate-50 p-3 dark:border-slate-800 dark:bg-slate-900/40">
                                    <div className="text-xs uppercase tracking-wide text-slate-500 dark:text-slate-400">AI recommended parameters</div>
                                    <div className="mt-1 font-mono">
                                        {formatAIInputSummary(startPreviewPlan.recommendedParams, aiInputFeatures)}
                                    </div>
                                </div>
                                {startPreviewPlan.predictedGeometry ? (
                                    <div className="rounded-lg border border-slate-200 bg-slate-50 p-3 dark:border-slate-800 dark:bg-slate-900/40">
                                        <div className="text-xs uppercase tracking-wide text-slate-500 dark:text-slate-400">AI predicted geometry from recommendation</div>
                                        <div className="mt-1 font-mono">
                                            height={startPreviewPlan.predictedGeometry.height.toFixed(2)} mm, width={startPreviewPlan.predictedGeometry.width.toFixed(2)} mm
                                        </div>
                                    </div>
                                ) : null}
                            </>
                        )}

                        <div className="text-xs text-slate-600 dark:text-slate-400">
                            {startPreviewPlan.confidence === null
                                ? 'AI confidence: unavailable'
                                : `AI confidence: ${startPreviewPlan.confidence.toFixed(3)}`}
                        </div>
                    </div>
                ) : null}
            </Modal>

            <Modal
                open={manualAdjustDraft !== null}
                title="Manual Adjustment Confirmation"
                confirmText="Apply and Continue"
                onConfirm={() => {
                    void applyManualAdjustAndContinue();
                }}
                onClose={() => setManualAdjustDraft(null)}
            >
                {manualAdjustDraft ? (
                    <div className="space-y-3 text-sm">
                        <div>
                            <span className="text-slate-500 dark:text-slate-400">Process:</span>{' '}
                            <span className="font-mono">{processId ?? '-'}</span>
                        </div>
                        <div className="rounded-lg border border-slate-200 bg-slate-50 p-3 dark:border-slate-800 dark:bg-slate-900/40">
                            <div className="text-xs uppercase tracking-wide text-slate-500 dark:text-slate-400">
                                Manual process parameters
                            </div>
                            <div className="mt-2 grid gap-2">
                                {aiInputFeatures.map((feature) => (
                                    <label key={feature.key} className="flex items-center justify-between gap-2">
                                        <span className="text-xs text-slate-600 dark:text-slate-400">{feature.label}</span>
                                        <div className="flex items-center gap-1">
                                            <input
                                                type="number"
                                                step={feature.step ?? 0.1}
                                                min={feature.min}
                                                max={feature.max}
                                                value={manualAdjustDraft[feature.key] ?? 0}
                                                onChange={(e) => {
                                                    const next = Number(e.target.value);
                                                    if (Number.isNaN(next)) return;
                                                    setManualAdjustDraft((prev) => (prev ? { ...prev, [feature.key]: next } : prev));
                                                }}
                                                className="w-24 rounded-md border border-slate-200 bg-white px-2 py-1 text-right text-xs font-mono dark:border-slate-700 dark:bg-slate-900"
                                            />
                                            <span className="text-xs text-slate-500 min-w-10">{feature.unit || '—'}</span>
                                        </div>
                                    </label>
                                ))}
                            </div>
                        </div>
                        <div className="text-xs text-slate-600 dark:text-slate-400">
                            This applies a parameter-driven override and continues the paused process.
                        </div>
                    </div>
                ) : null}
            </Modal>

            <Modal
                open={aiRecommendationPlan !== null}
                title="AI Recommendation Confirmation"
                confirmText="Apply and Continue"
                confirmDisabled={aiRecommendationLoading || !aiRecommendationPlan?.recommendedParams}
                onConfirm={() => {
                    void applyAiRecommendationAndContinue();
                }}
                onClose={() => {
                    setAiRecommendationPlan(null);
                    setAiRecommendationError(null);
                    setAiRecommendationLoading(false);
                }}
                dangerHint="AI can make mistakes. Validate recommendations before proceeding."
            >
                {aiRecommendationPlan ? (
                    <div className="space-y-3 text-sm">
                        <div>
                            <span className="text-slate-500 dark:text-slate-400">Process:</span>{' '}
                            <span className="font-mono">{processId ?? '-'}</span>
                        </div>
                        <div>
                            <span className="text-slate-500 dark:text-slate-400">Mode:</span>{' '}
                            <span className="font-medium">Geometry-driven</span>
                        </div>

                        <div className="rounded-lg border border-slate-200 bg-slate-50 p-3 dark:border-slate-800 dark:bg-slate-900/40">
                            <div className="text-xs uppercase tracking-wide text-slate-500 dark:text-slate-400">
                                Geometry target
                            </div>
                            <div className="mt-2 grid gap-2">
                                <label className="flex items-center justify-between gap-2">
                                    <span className="text-xs text-slate-600 dark:text-slate-400">{domainTerms.profileHeight}</span>
                                    <div className="flex items-center gap-1">
                                        <input
                                            type="number"
                                            step="0.01"
                                            value={aiRecommendationPlan.targetHeight}
                                            onChange={(e) => {
                                                const next = Number(e.target.value);
                                                if (Number.isNaN(next)) return;
                                                setAiRecommendationPlan((prev) => (prev ? { ...prev, targetHeight: next } : prev));
                                            }}
                                            className="w-24 rounded-md border border-slate-200 bg-white px-2 py-1 text-right text-xs font-mono dark:border-slate-700 dark:bg-slate-900"
                                        />
                                        <span className="text-xs text-slate-500 w-10">mm</span>
                                    </div>
                                </label>
                                <label className="flex items-center justify-between gap-2">
                                    <span className="text-xs text-slate-600 dark:text-slate-400">{domainTerms.profileWidth}</span>
                                    <div className="flex items-center gap-1">
                                        <input
                                            type="number"
                                            step="0.01"
                                            value={aiRecommendationPlan.targetWidth}
                                            onChange={(e) => {
                                                const next = Number(e.target.value);
                                                if (Number.isNaN(next)) return;
                                                setAiRecommendationPlan((prev) => (prev ? { ...prev, targetWidth: next } : prev));
                                            }}
                                            className="w-24 rounded-md border border-slate-200 bg-white px-2 py-1 text-right text-xs font-mono dark:border-slate-700 dark:bg-slate-900"
                                        />
                                        <span className="text-xs text-slate-500 w-10">mm</span>
                                    </div>
                                </label>
                            </div>
                            <div className="mt-3">
                                <Button
                                    size="sm"
                                    variant="secondary"
                                    disabled={aiRecommendationLoading}
                                    onClick={() => {
                                        void requestAiRecommendation(
                                            aiRecommendationPlan.targetHeight,
                                            aiRecommendationPlan.targetWidth,
                                        );
                                    }}
                                >
                                    {aiRecommendationLoading ? 'Calculating…' : 'Get AI Recommendation'}
                                </Button>
                            </div>
                        </div>

                        {aiRecommendationError ? (
                            <div className="rounded-lg border border-rose-300 bg-rose-50 p-3 text-xs text-rose-700 dark:border-rose-800 dark:bg-rose-950/30 dark:text-rose-200">
                                {aiRecommendationError}
                            </div>
                        ) : null}

                        {aiRecommendationPlan.recommendedParams ? (
                            <>
                                <div className="rounded-lg border border-slate-200 bg-slate-50 p-3 dark:border-slate-800 dark:bg-slate-900/40">
                                    <div className="text-xs uppercase tracking-wide text-slate-500 dark:text-slate-400">
                                        AI recommended parameters
                                    </div>
                                    <div className="mt-1 font-mono">
                                        {formatAIInputSummary(aiRecommendationPlan.recommendedParams, aiInputFeatures)}
                                    </div>
                                </div>
                                {aiRecommendationPlan.predictedGeometry ? (
                                    <div className="rounded-lg border border-slate-200 bg-slate-50 p-3 dark:border-slate-800 dark:bg-slate-900/40">
                                        <div className="text-xs uppercase tracking-wide text-slate-500 dark:text-slate-400">
                                            AI predicted geometry from recommendation
                                        </div>
                                        <div className="mt-1 font-mono">
                                            height={aiRecommendationPlan.predictedGeometry.height.toFixed(2)} mm, width={aiRecommendationPlan.predictedGeometry.width.toFixed(2)} mm
                                        </div>
                                    </div>
                                ) : null}
                            </>
                        ) : null}

                        <div className="text-xs text-slate-600 dark:text-slate-400">
                            {aiRecommendationPlan.confidence === null
                                ? 'AI confidence: unavailable'
                                : `AI confidence: ${aiRecommendationPlan.confidence.toFixed(3)}`}
                        </div>
                    </div>
                ) : null}
            </Modal>

            <Modal
                open={jobReport !== null}
                title="Job Report"
                confirmText={jobReport?.recommendNewDoe ? 'Run New DOE' : 'Close Report'}
                confirmVariant={jobReport?.recommendNewDoe ? 'primary' : 'secondary'}
                onConfirm={() => {
                    if (jobReport?.recommendNewDoe) {
                        handleDeviationAction('start_new_doe');
                    }
                    setJobReport(null);
                }}
                onClose={() => setJobReport(null)}
            >
                {jobReport ? (
                    <div className="space-y-3 text-sm">
                        <div className="rounded-lg border border-slate-200 bg-slate-50 p-3 dark:border-slate-800 dark:bg-slate-900/40">
                            <div>Total evaluated points: <span className="font-mono">{jobReport.totalPoints}</span></div>
                            <div>Warning points: <span className="font-mono">{jobReport.warningPoints}</span></div>
                            <div>Warning rate: <span className="font-mono">{(jobReport.warningRate * 100).toFixed(1)}%</span></div>
                        </div>
                        {jobReport.totalPoints === 0 ? (
                            <div>No deviation points were evaluated during this job.</div>
                        ) : jobReport.recommendNewDoe ? (
                            <div>
                                The job performance was not satisfactory. Warnings were raised for more than 20% of the points.
                                We recommend running a new DOE.
                            </div>
                        ) : (
                            <div>
                                The job performance was satisfactory. Warnings were raised for {(jobReport.warningRate * 100).toFixed(1)}% of the points.
                            </div>
                        )}
                    </div>
                ) : null}
            </Modal>
        </div>
    );
}
