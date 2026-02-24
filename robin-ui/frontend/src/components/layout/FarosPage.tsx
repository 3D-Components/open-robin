import { useState, useEffect, useMemo, useCallback } from 'react';
import { Chip } from '../ui/Chip';
import { TopBar } from './TopBar';
import { Sidebar } from './Sidebar';
import { LiveOps } from '../features/live-ops/LiveOps';
import { RobotsTab } from '../features/robots/RobotsTab';
import { ModelsTrustTab } from '../features/models/ModelsTrustTab';
import { MLOpsTab } from '../features/mlops/MLOpsTab';
import { InferenceDevLabTab } from '../features/inference/InferenceDevLabTab';
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
    type RobinMeasurement,
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
    RobotFilter,
    ConnStatus,
    MeasurementPoint,
    Alert,
    ModelVersion,
    PipelineRun,
    AuditLogEntry,
    RunSummary,
    ProcessControlsState,
    OperationMode,
    TargetGeometry,
    DeviationAction,
} from '../../types';
import { domainTerms } from '../../config/domain';

// Helper for timestamp generation
const ts = (offsetMinutes: number) =>
    new Date(Date.now() - offsetMinutes * 60000).toISOString();

// Helper to generate short IDs like in monolith
function shortId(prefix: string) {
    return `${prefix}-${Math.random().toString(16).slice(2, 8).toUpperCase()}`;
}

// Helper for current ISO timestamp
function nowIso() {
    return new Date().toISOString();
}

// Helper to determine trust gate from confidence
function gateForConfidence(conf: number, warnTh: number, stopTh: number): 'OK' | 'Warning' | 'Stop' {
    if (conf < stopTh) return 'Stop';
    if (conf < warnTh) return 'Warning';
    return 'OK';
}

// Initial robot states
const initialRobots: Record<RobotCell['id'], RobotCell> = {
    robotA: {
        id: 'robotA',
        name: 'Robot A',
        position: 'PA',
        state: 'Idle',
        isParamFrozen: false,
        currentRunId: undefined,
        taskProgressPct: 0,
        segmentIndex: 0,
        activeModel: '-',
        lastTrustScore: 0.96,
        lastDeploymentAt: ts(610),
        lastInferenceAt: ts(0),
    },
    robotB: {
        id: 'robotB',
        name: 'Robot B',
        position: 'PC',
        state: 'Idle',
        isParamFrozen: false,
        currentRunId: undefined,
        taskProgressPct: 0,
        segmentIndex: 0,
        activeModel: '-',
        lastTrustScore: 0.97,
        lastDeploymentAt: ts(710),
        lastInferenceAt: ts(0),
    },
};

// Mock model versions - will be replaced when AI models endpoint is available
const mockModels: ModelVersion[] = [
    {
        id: 'Model_PA v1.3.2',
        position: 'PA',
        createdAt: ts(610),
        artifactPath: '/artifacts/PA/model_v1.3.2.onnx',
        trustScore: 0.96,
        goNoGo: 'Go',
        notes: 'Stable process geometry; low OOD events.',
    },
];

// Mock pipeline runs
const mockPipelines: PipelineRun[] = [];

// Mock audit log
const mockAuditLog: AuditLogEntry[] = [
    {
        id: 'audit-001',
        at: ts(5),
        actor: 'System',
        action: 'ROBIN UI started',
        severity: 'Info',
    },
];

// Mock run history
const mockRunHistory: RunSummary[] = [];

// Metric labels
const METRIC_LABELS: Record<MetricType, string> = {
    speed: `${domainTerms.speed} (${domainTerms.speedUnit})`,
    current: `${domainTerms.current} (${domainTerms.currentUnit})`,
    voltage: `${domainTerms.voltage} (${domainTerms.voltageUnit})`,
    profileHeight: `${domainTerms.profileHeight} (mm)`,
    profileWidth: `${domainTerms.profileWidth} (mm)`,
};

// ---------------------------------------------------------------------------
// Convert API measurements → internal MeasurementPoint format
// ---------------------------------------------------------------------------

function apiToMeasurementPoints(measurements: RobinMeasurement[]): MeasurementPoint[] {
    return measurements.map((m, i) => ({
        t: i,
        // ROBIN is single-process, so channel A = real data, channel B mirrors it
        speedA: m.speed ?? 0,
        speedB: m.speed ?? 0,
        currentA: m.current ?? 0,
        currentB: m.current ?? 0,
        voltageA: m.voltage ?? 0,
        voltageB: m.voltage ?? 0,
        profileHeightA: m.height ?? 0,
        profileHeightB: m.height ?? 0,
        profileWidthA: m.width ?? 0,
        profileWidthB: m.width ?? 0,
        confidenceA: 0.95,
        confidenceB: 0.95,
    }));
}

export function FarosPage() {
    // Navigation
    const [tab, setTab] = useState<TabKey>('live');

    // -----------------------------------------------------------------------
    // ROBIN API hooks
    // -----------------------------------------------------------------------
    const [processId, setProcessId] = useState<string | null>('ros_bridge');
    const { data: measurementsData } = useMeasurements(processId, 200, 2000);
    const { data: healthData, error: healthError } = useHealth(5000);
    const { data: processListData } = useProcessList(10000);
    const { data: aiModelsData, refetch: refetchModels } = useAIModels();
    const { data: processSnapshotData } = useProcessSnapshot(processId, 5000);
    const { data: targetGeometryData } = useTargetGeometry(processId, 5000);

    // Derive connection status from API health
    const backendConnected = healthData?.status === 'ok' && !healthError;

    // -----------------------------------------------------------------------
    // Process Controls state (replaces Wirecloud process-controls widget)
    // -----------------------------------------------------------------------
    const [processControls, setProcessControls] = useState<ProcessControlsState>({
        mode: 'parameter_driven',
        tolerance: 10,
        wireSpeed: 10,
        current: 150,
        voltage: 24,
        targetHeight: 3.0,
        targetWidth: 6.0,
    });

    // Derive operation mode from process snapshot
    const processMode: OperationMode | null = useMemo(() => {
        if (processSnapshotData?.operationMode?.value) {
            const v = processSnapshotData.operationMode.value;
            if (v === 'parameter_driven' || v === 'geometry_driven') return v;
        }
        return null;
    }, [processSnapshotData]);

    // Derive target geometry from API
    const targetGeometry: TargetGeometry | null = useMemo(() => {
        if (targetGeometryData?.status === 'success' && targetGeometryData.target_geometry) {
            return targetGeometryData.target_geometry;
        }
        return null;
    }, [targetGeometryData]);

    // Connection states as object for TopBar
    const [connections, setConnections] = useState<Record<string, ConnStatus>>({
        'Backend API': 'yellow',
        'Orion-LD': 'yellow',
        'Mintaka': 'yellow',
    });

    // Update connection status from health
    useEffect(() => {
        setConnections({
            'Backend API': backendConnected ? 'green' : 'red',
            'Orion-LD': backendConnected ? 'green' : 'yellow',
            'Mintaka': backendConnected ? 'green' : 'yellow',
        });
    }, [backendConnected]);

    // Session mode
    const [sessionMode, setSessionMode] = useState<'Active Run' | 'Demo Mode'>('Active Run');

    // Robots
    const [robots, setRobots] = useState(initialRobots);
    const [currentRun, _setCurrentRun] = useState<ProcessRun | null>(null);

    // Wire robot state to the viser 3D server (optional - works when viser is running)
    useViserBridge(robots);

    // Alerts (dynamic - can be updated)
    const [alerts, setAlerts] = useState<Alert[]>([
        { id: 'alert-001', at: nowIso(), severity: 'Info', message: 'ROBIN UI started. Waiting for live data…', source: 'System' },
    ]);

    // Helper to push an alert
    const pushAlert = useCallback((sev: 'Info' | 'Warning' | 'Critical', msg: string, src: string) => {
        setAlerts((a) =>
            [{ id: shortId('ALR'), at: nowIso(), severity: sev, message: msg, source: src }, ...a].slice(0, 8)
        );
    }, []);

    // Handle process controls apply
    const handleControlsApply = useCallback(async (ctrl: ProcessControlsState) => {
        if (!processId) return;
        pushAlert('Info', `Applied process controls (mode: ${ctrl.mode}, tolerance: ${ctrl.tolerance}%)`, 'Process Controls');

        if (ctrl.mode === 'geometry_driven') {
            try {
                await setTarget(processId, ctrl.targetHeight, ctrl.targetWidth);
                pushAlert('Info', `Target geometry set: ${ctrl.targetHeight.toFixed(2)} x ${ctrl.targetWidth.toFixed(2)} mm`, 'Process Controls');
            } catch {
                pushAlert('Warning', 'Failed to set target geometry on backend', 'Process Controls');
            }
        }
    }, [processId, pushAlert]);

    // Handle deviation action buttons
    const handleDeviationAction = useCallback((action: DeviationAction) => {
        const labels: Record<DeviationAction, string> = {
            manual_adjust: 'Manual parameter adjustment requested',
            new_ai_recommendation: 'Requesting new AI recommendation',
            add_data_finetune: 'Adding data for model fine-tuning',
            start_new_doe: 'Starting new Design of Experiments',
        };
        pushAlert('Info', labels[action], 'Deviation Monitor');
    }, [pushAlert]);

    // Model routing
    const [routing, setRouting] = useState<Record<'PA' | 'PC', string>>({
        PA: '-',
        PC: '-',
    });

    // Update model routing from API
    useEffect(() => {
        if (aiModelsData?.active_model) {
            const modelName = aiModelsData.active_model.name ?? aiModelsData.active_model.path;
            setRouting({ PA: modelName, PC: modelName });
            setRobots((prev) => ({
                ...prev,
                robotA: { ...prev.robotA, activeModel: modelName },
            }));
        }
    }, [aiModelsData]);

    // Trust feed (live updating for chart)
    const [trustFeed, setTrustFeed] = useState<TrustAssessment[]>([]);

    // Visualization state
    const [vizMode, setVizMode] = useState<VizMode>('execution');
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

    // -----------------------------------------------------------------------
    // Telemetry - sourced from ROBIN API instead of mock data
    // -----------------------------------------------------------------------
    const [telemetry, setTelemetry] = useState<MeasurementPoint[]>([]);
    const [metric, setMetric] = useState<MetricType>('profileHeight');
    const [showRobot, setShowRobot] = useState<RobotFilter>('A');
    const [freezeCharts, setFreezeCharts] = useState(false);

    // Convert API measurements to telemetry points
    useEffect(() => {
        if (!measurementsData?.measurements?.length) return;
        if (freezeCharts) return;

        const points = apiToMeasurementPoints(measurementsData.measurements);
        setTelemetry(points);
        setTimelineT(points.length);

        // Update robot state based on whether we're receiving data
        if (measurementsData.count > 0) {
            setRobots((prev) => ({
                ...prev,
                robotA: {
                    ...prev.robotA,
                    state: prev.robotA.state === 'Idle' ? 'Running' as const : prev.robotA.state,
                    taskProgressPct: Math.min(
                        100,
                        (measurementsData.count / 200) * 100
                    ),
                },
            }));
        }
    }, [measurementsData, freezeCharts]);

    // Alert when new measurement data arrives with interesting patterns
    useEffect(() => {
        if (!measurementsData?.measurements?.length) return;
        const latest = measurementsData.measurements[measurementsData.measurements.length - 1];
        if (latest && measurementsData.count > 0) {
            // Push a "data flowing" alert the first time data arrives
            if (measurementsData.count === 1 || (telemetry.length === 0 && measurementsData.count > 0)) {
                pushAlert('Info', `Receiving live data for process "${processId}" (${measurementsData.count} measurements)`, 'Backend API');
            }
        }
    }, [measurementsData?.count]);

    // Trust
    const [trustWarnTh, setTrustWarnTh] = useState(0.65);
    const [trustStopTh, setTrustStopTh] = useState(0.45);
    const [latestTrustA, setLatestTrustA] = useState<TrustAssessment>({
        atT: timelineT,
        robotId: 'robotA',
        confidence: 0.96,
        gate: 'OK',
        reasons: ['nominal'],
    });
    const [latestTrustB, setLatestTrustB] = useState<TrustAssessment>({
        atT: timelineT,
        robotId: 'robotB',
        confidence: 0.97,
        gate: 'OK',
        reasons: ['nominal'],
    });

    // Connection badge helper
    const connBadge = (s: ConnStatus) => (
        <Chip tone={s === 'green' ? 'good' : s === 'yellow' ? 'warn' : 'bad'}>
            {s === 'green' ? 'Green' : s === 'yellow' ? 'Yellow' : 'Red'}
        </Chip>
    );

    // Handler to switch model
    const onSwitchModel = (position: 'PA' | 'PC', next: string) => {
        setRouting(prev => ({ ...prev, [position]: next }));
    };

    // Settings
    const [darkMode, setDarkMode] = useState(true);

    // Apply dark mode
    useEffect(() => {
        document.documentElement.classList.toggle('dark', darkMode);
    }, [darkMode]);

    // Live trust feed generation (can remain synthetic until ROBIN has a trust engine)
    useEffect(() => {
        if (freezeCharts) return;

        const interval = setInterval(() => {
            const t = timelineT;
            const confA = Math.max(0, Math.min(1, 0.92 - Math.random() * 0.05));
            const confB = Math.max(0, Math.min(1, 0.90 - Math.random() * 0.07));

            const gateA = gateForConfidence(confA, trustWarnTh, trustStopTh);
            const gateB = gateForConfidence(confB, trustWarnTh, trustStopTh);

            const newTrustA: TrustAssessment = {
                atT: t,
                robotId: 'robotA',
                confidence: confA,
                gate: gateA,
                reasons: gateA === 'OK' ? ['nominal'] : ['low confidence'],
            };
            const newTrustB: TrustAssessment = {
                atT: t,
                robotId: 'robotB',
                confidence: confB,
                gate: gateB,
                reasons: gateB === 'OK' ? ['nominal'] : ['low confidence'],
            };

            setLatestTrustA(newTrustA);
            setLatestTrustB(newTrustB);
            setTrustFeed(prev => [...prev, newTrustA, newTrustB].slice(-80));
        }, 2000);

        return () => clearInterval(interval);
    }, [freezeCharts, timelineT, trustWarnTh, trustStopTh]);

    // Chart data
    const telemetryChartData = useMemo(() => {
        return telemetry.map((p) => ({
            t: p.t,
            A: metric === 'speed' ? p.speedA : metric === 'current' ? p.currentA : metric === 'voltage' ? p.voltageA : metric === 'profileHeight' ? p.profileHeightA : p.profileWidthA,
            B: metric === 'speed' ? p.speedB : metric === 'current' ? p.currentB : metric === 'voltage' ? p.voltageB : metric === 'profileHeight' ? p.profileHeightB : p.profileWidthB,
        }));
    }, [telemetry, metric]);

    const lastValues = useMemo(() => {
        const last = telemetry[telemetry.length - 1];
        if (!last) return { A: 0, B: 0 };
        return {
            A: metric === 'speed' ? last.speedA : metric === 'current' ? last.currentA : metric === 'voltage' ? last.voltageA : metric === 'profileHeight' ? last.profileHeightA : last.profileWidthA,
            B: metric === 'speed' ? last.speedB : metric === 'current' ? last.currentB : metric === 'voltage' ? last.voltageB : metric === 'profileHeight' ? last.profileHeightB : last.profileWidthB,
        };
    }, [telemetry, metric]);

    // Robot control handlers - wired to API
    const startRobot = (id: RobotCell['id']) => {
        const rb = robots[id];
        if (rb.state === 'E-Stop') {
            pushAlert('Critical', `${rb.name} is in E-Stop. Reset required.`, rb.name);
            return;
        }
        setRobots((prev) => ({
            ...prev,
            [id]: { ...prev[id], state: 'Running' as const, taskProgressPct: 2 },
        }));
        pushAlert('Info', `${rb.name} started.`, rb.name);
        // If process exists, resume it via API
        if (processId) {
            resumeProcess(processId).catch(() => { /* silently fail */ });
        }
    };
    const pauseRobot = (id: RobotCell['id']) => {
        const rb = robots[id];
        if (rb.state !== 'Running') return;
        setRobots((prev) => ({
            ...prev,
            [id]: { ...prev[id], state: 'Paused' as const },
        }));
        pushAlert('Warning', `${rb.name} paused by operator.`, rb.name);
        if (processId) {
            stopProcess(processId, 'operator_pause').catch(() => { /* silently fail */ });
        }
    };
    const resumeRobotHandler = (id: RobotCell['id']) => {
        const rb = robots[id];
        if (rb.state !== 'Paused') return;
        setRobots((prev) => ({
            ...prev,
            [id]: { ...prev[id], state: 'Running' as const },
        }));
        pushAlert('Info', `${rb.name} resumed.`, rb.name);
        if (processId) {
            resumeProcess(processId).catch(() => { /* silently fail */ });
        }
    };
    const abortRobot = (id: RobotCell['id']) => {
        const rb = robots[id];
        setRobots((prev) => ({
            ...prev,
            [id]: { ...prev[id], state: 'Idle' as const, taskProgressPct: 0, segmentIndex: 0 },
        }));
        pushAlert('Critical', `${rb.name} aborted.`, rb.name);
        if (processId) {
            stopProcess(processId, 'operator_abort').catch(() => { /* silently fail */ });
        }
    };
    const toggleParamFreeze = (id: RobotCell['id']) => {
        setRobots((prev) => ({
            ...prev,
            [id]: { ...prev[id], isParamFrozen: !prev[id].isParamFrozen },
        }));
    };

    // -----------------------------------------------------------------------
    // Build AI models list from API data
    // -----------------------------------------------------------------------
    const aiModelsList: ModelVersion[] = useMemo(() => {
        if (!aiModelsData?.models) return mockModels;
        return aiModelsData.models.map((m) => ({
            id: m.name ?? m.path,
            position: 'PA' as const,
            createdAt: m.created ?? ts(0),
            artifactPath: m.path,
            trustScore: 0.90,
            goNoGo: 'Go' as const,
        }));
    }, [aiModelsData]);

    // -----------------------------------------------------------------------
    // Process selector (replaces Wirecloud "Static Process ID" operator)
    // -----------------------------------------------------------------------
    const availableProcessIds = useMemo(() => {
        if (!processListData?.processes) return ['ros_bridge'];
        const ids = processListData.processes.map((p) => p.process_id);
        if (ids.length === 0) return ['ros_bridge'];
        return ids;
    }, [processListData]);

    // Auto-select first available process
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
                vizLatencyMs={0}
                vizLatencyTone={backendConnected ? 'good' : 'bad'}
                serveLatencyMs={0}
                serveLatencyTone={backendConnected ? 'good' : 'bad'}
                uptimeHrs={0}
                // ROBIN process selector props
                processId={processId}
                availableProcessIds={availableProcessIds}
                onProcessIdChange={setProcessId}
            />

            <div className="flex flex-1 min-h-0 min-w-0 overflow-hidden">
                <Sidebar tab={tab} setTab={setTab} />

                <main className="flex-1 min-h-0 min-w-0 overflow-y-auto p-4">
                    {tab === 'live' && (
                        <LiveOps
                            robots={robots}
                            latestTrustA={latestTrustA}
                            latestTrustB={latestTrustB}
                            trustWarnTh={trustWarnTh}
                            trustStopTh={trustStopTh}
                            startRobot={startRobot}
                            pauseRobot={pauseRobot}
                            resumeRobot={resumeRobotHandler}
                            abortRobot={abortRobot}
                            toggleParamFreeze={toggleParamFreeze}
                            currentRun={currentRun}
                            alerts={alerts}
                            vizMode={vizMode}
                            setVizMode={setVizMode}
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
                            showRobot={showRobot}
                            setShowRobot={setShowRobot}
                            freezeCharts={freezeCharts}
                            setFreezeCharts={setFreezeCharts}
                            metricLabel={METRIC_LABELS[metric]}
                            lastValues={lastValues}
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
                            robots={robots}
                            currentRun={currentRun}
                            telemetry={telemetry}
                            trustWarnTh={trustWarnTh}
                            trustStopTh={trustStopTh}
                            startRobot={startRobot}
                            pauseRobot={pauseRobot}
                            resumeRobot={resumeRobotHandler}
                            abortRobot={abortRobot}
                            toggleParamFreeze={toggleParamFreeze}
                        />
                    )}
                    {tab === 'models' && (
                        <ModelsTrustTab
                            models={aiModelsList}
                            routing={routing}
                            onSwitchModel={onSwitchModel}
                            audit={mockAuditLog}
                            trustWarnTh={trustWarnTh}
                            trustStopTh={trustStopTh}
                            setTrustWarnTh={setTrustWarnTh}
                            setTrustStopTh={setTrustStopTh}
                            latestTrustA={latestTrustA}
                            latestTrustB={latestTrustB}
                            trustFeed={trustFeed}
                            aiModelsData={aiModelsData}
                            onModelsRefresh={refetchModels}
                        />
                    )}
                    {tab === 'mlops' && (
                        <MLOpsTab
                            pipelineRuns={mockPipelines}
                            models={aiModelsList}
                            routing={routing}
                            onSwitchModel={onSwitchModel}
                            audit={mockAuditLog}
                        />
                    )}
                    {tab === 'inference' && <InferenceDevLabTab />}
                    {tab === 'history' && (
                        <HistoryTab
                            summaries={mockRunHistory}
                            audit={mockAuditLog}
                        />
                    )}
                    {tab === 'settings' && (
                        <SettingsTab
                            dark={darkMode}
                            setDark={setDarkMode}
                            connections={connections}
                            setConnections={setConnections}
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
        </div>
    );
}
