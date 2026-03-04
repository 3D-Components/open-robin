import { useState, useEffect, useMemo, useCallback } from 'react';
import { Chip } from '../ui/Chip';
import { TopBar } from './TopBar';
import { Sidebar } from './Sidebar';
import { Modal } from '../ui/Modal';
import { LiveOps } from '../features/live-ops/LiveOps';
import { RobotsTab } from '../features/robots/RobotsTab';
import { ModelsTrustTab } from '../features/models/ModelsTrustTab';
import { HistoryTab } from '../features/history/HistoryTab';
import { SettingsTab } from '../features/settings/SettingsTab';
import { useViserBridge } from '../../hooks/useViserBridge';
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
    getAIRecommendation,
    type RobinMeasurement,
    type AIRecommendationResponse,
} from '../../hooks/useRobinAPI';
import type {
    TabKey,
    RobotCell,
    ProcessRun,
    TrustAssessment,
    VizMode,
    CameraView,
    LayerVisibility,
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

const METRIC_LABELS: Record<MetricType, string> = {
    speed: `${domainTerms.speed} (${domainTerms.speedUnit})`,
    current: `${domainTerms.current} (${domainTerms.currentUnit})`,
    voltage: `${domainTerms.voltage} (${domainTerms.voltageUnit})`,
    profileHeight: `${domainTerms.profileHeight} (mm)`,
    profileWidth: `${domainTerms.profileWidth} (mm)`,
};

function apiToMeasurementPoints(measurements: RobinMeasurement[]): MeasurementPoint[] {
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
            speed: m.speed ?? 0,
            current: m.current ?? 0,
            voltage: m.voltage ?? 0,
            profileHeight: m.height ?? 0,
            profileWidth: m.width ?? 0,
            confidence: 0.95,
        };
    });
}

function toNumber(value: unknown): number | null {
    if (typeof value !== 'number' || Number.isNaN(value)) return null;
    return value;
}

function mapRecommendationToControls(
    recommended: Record<string, unknown> | undefined,
    fallback: ProcessControlsState,
): Pick<ProcessControlsState, 'speed' | 'current' | 'voltage'> {
    const speed =
        toNumber(recommended?.wireSpeed)
        ?? toNumber(recommended?.lineSpeedSetpoint)
        ?? toNumber(recommended?.travelSpeed)
        ?? fallback.speed;
    const current =
        toNumber(recommended?.current)
        ?? toNumber(recommended?.flowRateSetpoint)
        ?? fallback.current;
    const voltage =
        toNumber(recommended?.voltage)
        ?? toNumber(recommended?.pressureSetpoint)
        ?? fallback.voltage;
    return { speed, current, voltage };
}

type StartPreviewPlan =
    | {
        mode: 'parameter_driven';
        processId: string;
        inputParams: { wireSpeed: number; current: number; voltage: number };
        predictedGeometry: { height: number; width: number };
        confidence: number | null;
    }
    | {
        mode: 'geometry_driven';
        processId: string;
        targetGeometry: { height: number; width: number };
        recommendedParams: { wireSpeed: number; current: number; voltage: number };
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
        speed: 10,
        current: 150,
        voltage: 24,
        targetHeight: 3.0,
        targetWidth: 6.0,
    });

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

    useViserBridge(robot);

    const [alerts, setAlerts] = useState<Alert[]>([
        { id: 'alert-001', at: nowIso(), severity: 'Info', message: 'ROBIN UI started. Waiting for live data…', source: 'System' },
    ]);
    const [startPlanning, setStartPlanning] = useState(false);
    const [startPreviewPlan, setStartPreviewPlan] = useState<StartPreviewPlan | null>(null);

    const pushAlert = useCallback((sev: 'Info' | 'Warning' | 'Critical', msg: string, src: string) => {
        setAlerts((a) =>
            [{ id: shortId('ALR'), at: nowIso(), severity: sev, message: msg, source: src }, ...a].slice(0, 8)
        );
    }, []);

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

                const mapped = mapRecommendationToControls(
                    recommended,
                    ctrl,
                );
                setProcessControls((prev) => ({
                    ...prev,
                    speed: mapped.speed,
                    current: mapped.current,
                    voltage: mapped.voltage,
                }));
                pushAlert(
                    'Info',
                    `AI suggested parameters: speed ${mapped.speed.toFixed(2)}, current ${mapped.current.toFixed(2)}, voltage ${mapped.voltage.toFixed(2)}`,
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
                input_params: {
                    wireSpeed: ctrl.speed,
                    current: ctrl.current,
                    voltage: ctrl.voltage,
                },
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
                `AI predicts geometry ${height.toFixed(2)} x ${width.toFixed(2)} mm from current parameters`,
                'AI Recommendation',
            );
        } catch (err) {
            const message = err instanceof Error ? err.message : 'Failed to fetch AI geometry prediction';
            pushAlert('Warning', message, 'AI Recommendation');
        }
    }, [processId, pushAlert, setProcessControls, sessionMode]);

    const handleDeviationAction = useCallback((action: DeviationAction) => {
        const labels: Record<DeviationAction, string> = {
            manual_adjust: 'Manual parameter adjustment requested',
            new_ai_recommendation: 'Requesting new AI recommendation',
            add_data_finetune: 'Adding data for model fine-tuning',
            start_new_doe: 'Starting new Design of Experiments',
        };
        pushAlert('Info', labels[action], 'Deviation Monitor');
    }, [pushAlert]);

    const [activeModel, setActiveModel] = useState<string>('—');

    useEffect(() => {
        if (aiModelsData?.active_model) {
            const modelName = aiModelsData.active_model.name ?? aiModelsData.active_model.path;
            setActiveModel(modelName);
            setRobot((prev) => ({ ...prev, activeModel: modelName }));
        }
    }, [aiModelsData]);

    const [trustFeed, setTrustFeed] = useState<TrustAssessment[]>([]);

    const [vizMode] = useState<VizMode>('execution');
    const [layers, setLayers] = useState<LayerVisibility>({
        robotModel: true,
        torchPath: true,
        workpiece: true,
        profileSegments: true,
        frames: false,
    });
    const [camera, setCamera] = useState<CameraView>('Isometric');
    const [timelineT, setTimelineT] = useState(0);
    const [replay, setReplay] = useState(false);

    const [telemetry, setTelemetry] = useState<MeasurementPoint[]>([]);
    const [metric, setMetric] = useState<MetricType>('profileHeight');
    const [freezeCharts, setFreezeCharts] = useState(false);

    const simulationProgress = useMemo(() => {
        const raw = processSnapshotData?.simulationProgress?.value;
        return typeof raw === 'number' && Number.isFinite(raw) ? raw : null;
    }, [processSnapshotData]);

    useEffect(() => {
        if (!measurementsData?.measurements?.length) return;
        if (freezeCharts) return;

        const points = apiToMeasurementPoints(measurementsData.measurements);
        setTelemetry(points);
        setTimelineT(points.length ? points[points.length - 1].t : 0);
    }, [measurementsData, freezeCharts]);

    useEffect(() => {
        if (simulationProgress === null) return;
        setRobot((prev) => ({
            ...prev,
            taskProgressPct: Math.max(0, Math.min(100, simulationProgress)),
        }));
    }, [simulationProgress]);

    const backendProcessStatus = processSnapshotData?.processStatus?.value ?? null;

    useEffect(() => {
        if (!backendProcessStatus) return;
        setRobot((prev) => {
            if (backendProcessStatus === 'active' && prev.state === 'Idle') {
                return { ...prev, state: 'Running' as const };
            }
            if (backendProcessStatus === 'stopped' && prev.state === 'Running') {
                return { ...prev, state: 'Idle' as const };
            }
            return prev;
        });
    }, [backendProcessStatus]);

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
            timestamp: p.timestamp,
            value: metric === 'speed' ? p.speed : metric === 'current' ? p.current : metric === 'voltage' ? p.voltage : metric === 'profileHeight' ? p.profileHeight : p.profileWidth,
        }));
    }, [telemetry, metric]);

    const measurementSource = measurementsData?.debug_info?.source ?? 'none';

    const lastValue = useMemo(() => {
        const last = telemetry[telemetry.length - 1];
        if (!last) return 0;
        return metric === 'speed' ? last.speed : metric === 'current' ? last.current : metric === 'voltage' ? last.voltage : metric === 'profileHeight' ? last.profileHeight : last.profileWidth;
    }, [telemetry, metric]);

    const confirmAndStart = useCallback((plan: StartPreviewPlan) => {
        if (plan.mode === 'geometry_driven') {
            setProcessControls((prev) => ({
                ...prev,
                speed: plan.recommendedParams.wireSpeed,
                current: plan.recommendedParams.current,
                voltage: plan.recommendedParams.voltage,
            }));
        }

        setStartPreviewPlan(null);
        setRobot((prev) => ({ ...prev, state: 'Running' as const, taskProgressPct: 0 }));
        pushAlert('Info', `${robot.name} started.`, robot.name);
        if (processId) {
            resumeProcess(processId).catch(() => {});
        }
    }, [processId, pushAlert, robot.name]);

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
            const inputParams = {
                wireSpeed: processControls.speed,
                current: processControls.current,
                voltage: processControls.voltage,
            };
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
        const mapped = mapRecommendationToControls(recommended, processControls);
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
            recommendedParams: {
                wireSpeed: mapped.speed,
                current: mapped.current,
                voltage: mapped.voltage,
            },
            predictedGeometry,
            confidence: toNumber(rec.recommendation.confidence) ?? toNumber(recommended.confidence),
        };
    }, [processControls, processId, processMode, sessionMode]);

    const startRobot = () => {
        if (startPlanning) return;
        if (robot.state === 'E-Stop') {
            pushAlert('Critical', `${robot.name} is in E-Stop. Reset required.`, robot.name);
            return;
        }
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
        setRobot((prev) => ({ ...prev, state: 'Paused' as const }));
        pushAlert('Warning', `${robot.name} paused by operator.`, robot.name);
        if (processId) {
            stopProcess(processId, 'operator_pause').catch(() => {});
        }
    };
    const resumeRobotHandler = () => {
        if (robot.state !== 'Paused') return;
        setRobot((prev) => ({ ...prev, state: 'Running' as const }));
        pushAlert('Info', `${robot.name} resumed.`, robot.name);
        if (processId) {
            resumeProcess(processId).catch(() => {});
        }
    };
    const abortRobot = () => {
        setRobot((prev) => ({ ...prev, state: 'Idle' as const, taskProgressPct: 0, segmentIndex: 0 }));
        pushAlert('Critical', `${robot.name} aborted.`, robot.name);
        if (processId) {
            stopProcess(processId, 'operator_abort').catch(() => {});
        }
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
                            abortRobot={abortRobot}
                            toggleParamFreeze={toggleParamFreeze}
                            currentRun={currentRun}
                            alerts={alerts}
                            vizMode={vizMode}
                            layers={layers}
                            setLayers={setLayers}
                            camera={camera}
                            setCamera={setCamera}
                            timelineT={timelineT}
                            setTimelineT={setTimelineT}
                            replay={replay}
                            setReplay={setReplay}
                            telemetryChartData={telemetryChartData}
                            metric={metric}
                            setMetric={setMetric}
                            freezeCharts={freezeCharts}
                            setFreezeCharts={setFreezeCharts}
                            metricLabel={METRIC_LABELS[metric]}
                            lastValue={lastValue}
                            measurementSource={measurementSource}
                            measurementPollMs={measurementPollMs}
                            processId={processId}
                            processControls={processControls}
                            onControlsChange={setProcessControls}
                            onControlsApply={handleControlsApply}
                            targetGeometry={targetGeometry}
                            processMode={processMode}
                            latestMeasurements={measurementsData?.measurements ?? null}
                            measurementCount={measurementsData?.count ?? 0}
                            onDeviationAlert={(severity, message) => pushAlert(severity, message, 'Deviation Monitor')}
                            onDeviationAction={handleDeviationAction}
                        />
                    )}
                    {tab === 'robots' && (
                        <RobotsTab
                            robot={robot}
                            currentRun={currentRun}
                            telemetry={telemetry}
                            trustWarnTh={trustWarnTh}
                            trustStopTh={trustStopTh}
                            startRobot={startRobot}
                            startPending={startPlanning}
                            pauseRobot={pauseRobot}
                            resumeRobot={resumeRobotHandler}
                            abortRobot={abortRobot}
                            toggleParamFreeze={toggleParamFreeze}
                        />
                    )}
                    {tab === 'models' && (
                        <ModelsTrustTab
                            models={aiModelsList}
                            activeModel={activeModel}
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
                            replay={replay}
                            setReplay={setReplay}
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
                                        wireSpeed={startPreviewPlan.inputParams.wireSpeed.toFixed(2)}, current={startPreviewPlan.inputParams.current.toFixed(2)}, voltage={startPreviewPlan.inputParams.voltage.toFixed(2)}
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
                                        wireSpeed={startPreviewPlan.recommendedParams.wireSpeed.toFixed(2)}, current={startPreviewPlan.recommendedParams.current.toFixed(2)}, voltage={startPreviewPlan.recommendedParams.voltage.toFixed(2)}
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
        </div>
    );
}
