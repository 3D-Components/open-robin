import { useState } from 'react';
import {
    Bot,
    AlertTriangle,
    Activity,
    Shield,
    Camera,
    Layers,
    Clock,
    Play,
    Pause,
    Square,
    CheckCircle2,
    XCircle,
} from 'lucide-react';
import {
    LineChart,
    Line,
    XAxis,
    YAxis,
    CartesianGrid,
    Tooltip,
    Legend,
    ResponsiveContainer,
} from 'recharts';
import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Chip } from '../../ui/Chip';
import { Button } from '../../ui/Button';
import { Select } from '../../ui/Select';
import { Toggle } from '../../ui/Toggle';
import { ProgressBar, KV, Divider } from '../../ui/ProgressBar';
import { ProcessControlsPanel } from './ProcessControlsPanel';
import { DeviationMonitor } from './DeviationMonitor';
import { MeasurementKPIs } from './MeasurementKPIs';
import type {
    RobotCell,
    ProcessRun,
    TrustAssessment,
    VizMode,
    TrustGate,
    ProcessControlsState,
    OperationMode,
    TargetGeometry,
    DeviationAction,
} from '../../../types';
import type { RobinMeasurement } from '../../../hooks/useRobinAPI';
import { domainTerms } from '../../../config/domain';

interface LiveOpsProps {
    robots: Record<RobotCell["id"], RobotCell>;
    latestTrustA?: TrustAssessment;
    latestTrustB?: TrustAssessment;
    trustWarnTh: number;
    trustStopTh: number;
    startRobot: (id: RobotCell["id"]) => void;
    pauseRobot: (id: RobotCell["id"]) => void;
    resumeRobot: (id: RobotCell["id"]) => void;
    abortRobot: (id: RobotCell["id"]) => void;
    toggleParamFreeze: (id: RobotCell["id"]) => void;
    currentRun: ProcessRun | null;
    alerts: Array<{ id: string; at: string; severity: "Info" | "Warning" | "Critical"; message: string; source: string }>;
    vizMode: VizMode;
    setVizMode: (m: VizMode) => void;
    layers: { robotModel: boolean; torchPath: boolean; workpiece: boolean; profileSegments: boolean; frames: boolean };
    setLayers: (v: any) => void;
    camera: "Isometric" | "Top" | "Side";
    setCamera: (v: any) => void;
    timelineT: number;
    setTimelineT: (v: number) => void;
    replay: boolean;
    setReplay: (v: boolean) => void;
    telemetryChartData: any[];
    metric: "speed" | "current" | "voltage" | "profileHeight" | "profileWidth";
    setMetric: (v: any) => void;
    showRobot: "A" | "B" | "Both";
    setShowRobot: (v: any) => void;
    freezeCharts: boolean;
    setFreezeCharts: (v: boolean) => void;
    metricLabel: string;
    lastValues: { A: number; B: number };
    // New props for wirecloud replacement
    processId: string | null;
    processControls: ProcessControlsState;
    onControlsChange: (next: ProcessControlsState) => void;
    onControlsApply: (controls: ProcessControlsState) => void;
    targetGeometry: TargetGeometry | null;
    processMode: OperationMode | null;
    latestMeasurements: RobinMeasurement[] | null;
    measurementCount: number;
    onDeviationAlert: (severity: 'Info' | 'Warning' | 'Critical', message: string) => void;
    onDeviationAction: (action: DeviationAction) => void;
}

export function LiveOps(props: LiveOpsProps) {
    const {
        robots,
        latestTrustA,
        latestTrustB,
        trustWarnTh,
        trustStopTh,
        startRobot,
        pauseRobot,
        resumeRobot,
        abortRobot,
        toggleParamFreeze,
        currentRun,
        alerts,
        vizMode,
        setVizMode,
        layers,
        setLayers,
        camera,
        setCamera,
        timelineT,
        setTimelineT,
        replay,
        setReplay,
        telemetryChartData,
        metric,
        setMetric,
        showRobot,
        setShowRobot,
        freezeCharts,
        setFreezeCharts,
        metricLabel,
        lastValues,
        processId,
        processControls,
        onControlsChange,
        onControlsApply,
        targetGeometry,
        processMode,
        latestMeasurements,
        measurementCount,
        onDeviationAlert,
        onDeviationAction,
    } = props;

    return (
        <div className="space-y-4">
            {/* KPI row at top */}
            <MeasurementKPIs
                processId={processId}
                measurements={latestMeasurements}
                count={measurementCount}
            />

            {/* Main 3-column layout */}
            <div className="grid grid-cols-12 gap-4">
                {/* Left column: robot control + process controls + alerts */}
                <div className="col-span-12 space-y-4 xl:col-span-3">
                    <Card>
                        <CardHeader
                            title={
                                <span className="flex items-center gap-2">
                                    <Bot className="h-4 w-4" />
                                    Robot Control
                                </span>
                            }
                            subtitle={
                                currentRun
                                    ? `Run ${currentRun.id} · ${currentRun.mode} · started ${new Date(
                                        currentRun.startedAt
                                    ).toLocaleTimeString()}`
                                    : "No active run"
                            }
                        />
                        <CardBody className="space-y-3">
                            <RobotCard
                                robot={robots.robotA}
                                trust={latestTrustA}
                                trustWarnTh={trustWarnTh}
                                trustStopTh={trustStopTh}
                                onStart={() => startRobot("robotA")}
                                onPause={() => pauseRobot("robotA")}
                                onResume={() => resumeRobot("robotA")}
                                onAbort={() => abortRobot("robotA")}
                                onFreeze={() => toggleParamFreeze("robotA")}
                            />
                            <RobotCard
                                robot={robots.robotB}
                                trust={latestTrustB}
                                trustWarnTh={trustWarnTh}
                                trustStopTh={trustStopTh}
                                onStart={() => startRobot("robotB")}
                                onPause={() => pauseRobot("robotB")}
                                onResume={() => resumeRobot("robotB")}
                                onAbort={() => abortRobot("robotB")}
                                onFreeze={() => toggleParamFreeze("robotB")}
                            />
                        </CardBody>
                    </Card>

                    {/* Process Controls - replaces wirecloud process-controls widget */}
                    <ProcessControlsPanel
                        processId={processId}
                        controls={processControls}
                        onControlsChange={onControlsChange}
                        onApply={onControlsApply}
                        targetGeometry={targetGeometry}
                        processMode={processMode}
                    />

                    <Card>
                        <CardHeader
                            title={
                                <span className="flex items-center gap-2">
                                    <AlertTriangle className="h-4 w-4" />
                                    Alerts
                                </span>
                            }
                            subtitle="Operational + deviation events"
                            right={<Chip tone="ghost">{alerts.length} recent</Chip>}
                        />
                        <CardBody>
                            <div className="space-y-2">
                                {alerts.length === 0 ? (
                                    <div className="text-sm text-slate-600 dark:text-slate-400">
                                        No alerts.
                                    </div>
                                ) : (
                                    alerts.map((a) => (
                                        <AlertRow key={a.id} alert={a} />
                                    ))
                                )}
                            </div>
                        </CardBody>
                    </Card>
                </div>

                {/* Center column: visualization */}
                <div className="col-span-12 space-y-4 xl:col-span-6">
                    <VisualizationPanel
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
                        robots={robots}
                    />
                </div>

                {/* Right column: deviation monitor + telemetry + trust */}
                <div className="col-span-12 space-y-4 xl:col-span-3">
                    {/* Deviation Monitor - replaces wirecloud deviation-alert widget */}
                    <DeviationMonitor
                        processId={processId}
                        controls={processControls}
                        latestMeasurements={latestMeasurements}
                        targetGeometry={targetGeometry}
                        processMode={processMode}
                        onAlert={onDeviationAlert}
                        onAction={onDeviationAction}
                    />

                    <TelemetryPanel
                        telemetryChartData={telemetryChartData}
                        metric={metric}
                        setMetric={setMetric}
                        showRobot={showRobot}
                        setShowRobot={setShowRobot}
                        freezeCharts={freezeCharts}
                        setFreezeCharts={setFreezeCharts}
                        metricLabel={metricLabel}
                        lastValues={lastValues}
                    />

                    <TrustPanel
                        trustA={latestTrustA}
                        trustB={latestTrustB}
                        warnTh={trustWarnTh}
                        stopTh={trustStopTh}
                        timelineT={timelineT}
                    />
                </div>
            </div>
        </div>
    );
}

// =====================
// RobotCard Sub-component
// =====================

function RobotCard({
    robot,
    trust,
    trustWarnTh,
    trustStopTh,
    onStart,
    onPause,
    onResume,
    onAbort,
    onFreeze,
}: {
    robot: RobotCell;
    trust?: TrustAssessment;
    trustWarnTh: number;
    trustStopTh: number;
    onStart: () => void;
    onPause: () => void;
    onResume: () => void;
    onAbort: () => void;
    onFreeze: () => void;
}) {
    const stateTone =
        robot.state === "Running"
            ? "good"
            : robot.state === "Paused"
                ? "warn"
                : robot.state === "Fault" || robot.state === "E-Stop"
                    ? "bad"
                    : "neutral";

    const gate = trust?.gate;
    const gateTone = gate === "OK" ? "good" : gate === "Warning" ? "warn" : "bad";
    const conf = trust?.confidence ?? robot.lastTrustScore;

    const primaryAction =
        robot.state === "Idle" ? (
            <Button size="sm" onClick={onStart}>
                <Play className="h-4 w-4" /> Start
            </Button>
        ) : robot.state === "Running" ? (
            <Button size="sm" variant="secondary" onClick={onPause}>
                <Pause className="h-4 w-4" /> Pause
            </Button>
        ) : robot.state === "Paused" ? (
            <Button size="sm" onClick={onResume}>
                <Play className="h-4 w-4" /> Resume
            </Button>
        ) : (
            <Button size="sm" variant="secondary" disabled>
                <Square className="h-4 w-4" /> Locked
            </Button>
        );

    const trustHint =
        conf < trustStopTh
            ? "STOP: gate active"
            : conf < trustWarnTh
                ? "Warning: monitor"
                : "OK";

    return (
        <div className="rounded-xl border border-slate-200 bg-slate-50 p-3 dark:border-slate-800 dark:bg-slate-900/30">
            <div className="flex items-start justify-between gap-3">
                <div>
                    <div className="flex items-center gap-2">
                        <div className="text-sm font-semibold">{robot.name}</div>
                        <Chip tone="info">{robot.position}</Chip>
                    </div>
                    <div className="mt-1 flex flex-wrap items-center gap-2">
                        <Chip tone={stateTone as any}>{robot.state}</Chip>
                        <Chip tone={gateTone as any}>
                            <Shield className="h-3.5 w-3.5" />
                            {gate ? gate.toUpperCase() : "-"}
                        </Chip>
                        <Chip tone="ghost">
                            <span className="font-mono">{(conf ?? 0).toFixed(2)}</span>
                            <span className="opacity-70">conf</span>
                        </Chip>
                    </div>
                </div>
                <div className="flex items-center gap-2">
                    {primaryAction}
                    <Button size="sm" variant="danger" onClick={onAbort}>
                        Abort
                    </Button>
                </div>
            </div>

            <div className="mt-3 space-y-2">
                <div className="flex items-center justify-between">
                    <div className="text-xs text-slate-600 dark:text-slate-400">
                        {domainTerms.segment} progress
                    </div>
                    <div className="text-xs font-mono">
                        {Math.round(robot.taskProgressPct)}%
                    </div>
                </div>
                <ProgressBar
                    value={robot.taskProgressPct}
                    tone={
                        robot.state === "Fault" || robot.state === "E-Stop"
                            ? "bad"
                            : robot.state === "Paused"
                                ? "warn"
                                : robot.state === "Running"
                                    ? "good"
                                    : "neutral"
                    }
                />

                <div className="grid grid-cols-2 gap-2">
                    <div className="rounded-lg border border-slate-200 bg-white p-2 dark:border-slate-800 dark:bg-slate-950">
                        <KV k="Run ID" v={robot.currentRunId ?? "-"} mono />
                        <KV k={domainTerms.segment} v={`#${robot.segmentIndex}`} mono />
                    </div>
                    <div className="rounded-lg border border-slate-200 bg-white p-2 dark:border-slate-800 dark:bg-slate-950">
                        <KV k="Active model" v={robot.activeModel} />
                        <KV k="Trust" v={trustHint} />
                    </div>
                </div>

                <div className="flex items-center justify-between">
                    <Button
                        size="sm"
                        variant={robot.isParamFrozen ? "primary" : "secondary"}
                        onClick={onFreeze}
                        title="Lock last recommended parameters"
                    >
                        {robot.isParamFrozen ? "Parameters Frozen" : "Parameter Freeze"}
                    </Button>
                    <div className="text-xs text-slate-600 dark:text-slate-400">
                        Deployed {new Date(robot.lastDeploymentAt).toLocaleTimeString()}
                    </div>
                </div>
            </div>
        </div>
    );
}

// =====================
// AlertRow Sub-component
// =====================

function AlertRow({
    alert,
}: {
    alert: { id: string; at: string; severity: "Info" | "Warning" | "Critical"; message: string; source: string };
}) {
    const tone =
        alert.severity === "Info"
            ? "neutral"
            : alert.severity === "Warning"
                ? "warn"
                : "bad";
    const Icon =
        alert.severity === "Info"
            ? CheckCircle2
            : alert.severity === "Warning"
                ? AlertTriangle
                : XCircle;

    return (
        <div className="rounded-lg border border-slate-200 bg-white px-3 py-2 dark:border-slate-800 dark:bg-slate-950">
            <div className="flex items-start justify-between gap-3">
                <div className="min-w-0">
                    <div className="flex items-center gap-2">
                        <Chip tone={tone as any}>
                            <Icon className="h-3.5 w-3.5" />
                            {alert.severity}
                        </Chip>
                        <span className="text-xs text-slate-600 dark:text-slate-400">
                            {new Date(alert.at).toLocaleTimeString()}
                        </span>
                    </div>
                    <div className="mt-1 text-sm text-slate-900 dark:text-slate-100">
                        {alert.message}
                    </div>
                    <div className="mt-1 text-xs text-slate-600 dark:text-slate-400">
                        Source: {alert.source}
                    </div>
                </div>
            </div>
        </div>
    );
}

// =====================
// VisualizationPanel Sub-component
// =====================

function VisualizationPanel({
    vizMode,
    setVizMode,
    layers,
    setLayers,
    camera,
    setCamera,
    timelineT,
    setTimelineT,
    replay,
    setReplay,
    robots,
}: {
    vizMode: VizMode;
    setVizMode: (m: VizMode) => void;
    layers: {
        robotModel: boolean;
        torchPath: boolean;
        workpiece: boolean;
        profileSegments: boolean;
        frames: boolean;
    };
    setLayers: (v: any) => void;
    camera: "Isometric" | "Top" | "Side";
    setCamera: (v: any) => void;
    timelineT: number;
    setTimelineT: (v: number) => void;
    replay: boolean;
    setReplay: (v: boolean) => void;
    robots: Record<RobotCell["id"], RobotCell>;
}) {
    const [showLayers, setShowLayers] = useState(false);

    const latestT = Math.max(0, timelineT);
    const maxT = Math.max(60, latestT + 1);

    function resetView() {
        setCamera("Isometric");
        setLayers({
            robotModel: true,
            torchPath: true,
            workpiece: true,
            profileSegments: true,
            frames: false,
        });
        setShowLayers(false);
    }

    const layerCount = Object.values(layers).filter(Boolean).length;

    return (
        <Card className="overflow-hidden">
            <CardHeader
                title={
                    <span className="flex items-center gap-2">
                        <Camera className="h-4 w-4" />
                        Advanced Visualization
                    </span>
                }
                subtitle="Embedded 3D view (Viser)"
                right={
                    <div className="flex flex-wrap items-center gap-2">
                        <div className="inline-flex overflow-hidden rounded-md border border-slate-200 dark:border-slate-800">
                            <Button
                                size="sm"
                                variant={vizMode === "execution" ? "primary" : "secondary"}
                                className="rounded-none border-0"
                                onClick={() => setVizMode("execution")}
                                title="Robot execution view"
                            >
                                <Bot className="h-4 w-4" />
                                Execution
                            </Button>
                            <Button
                                size="sm"
                                variant={vizMode === "deposition" ? "primary" : "secondary"}
                                className="rounded-none border-0"
                                onClick={() => setVizMode("deposition")}
                                title={`${domainTerms.depositionView} view`}
                            >
                                <Layers className="h-4 w-4" />
                                Deposition
                            </Button>
                        </div>

                        <div className="relative">
                            <Button
                                size="sm"
                                variant="secondary"
                                onClick={() => setShowLayers((s) => !s)}
                                title="Toggle layers"
                            >
                                <Layers className="h-4 w-4" />
                                Layers
                                <Chip tone="ghost" className="ml-1">
                                    {layerCount}
                                </Chip>
                            </Button>

                            {showLayers ? (
                                <div className="absolute right-0 top-11 z-20 w-[320px] rounded-xl border border-slate-200 bg-white p-3 shadow-xl dark:border-slate-800 dark:bg-slate-950">
                                    <div className="flex items-center justify-between">
                                        <div className="text-xs font-semibold text-slate-900 dark:text-slate-100">
                                            Scene layers
                                        </div>
                                        <Button size="sm" variant="ghost" onClick={() => setShowLayers(false)}>
                                            Close
                                        </Button>
                                    </div>
                                    <div className="mt-2 grid gap-2">
                                        <Toggle
                                            checked={layers.robotModel}
                                            onChange={(v) => setLayers((x: any) => ({ ...x, robotModel: v }))}
                                            label="Robot model"
                                            hint="Robot geometry & links"
                                        />
                                        <Toggle
                                            checked={layers.torchPath}
                                            onChange={(v) => setLayers((x: any) => ({ ...x, torchPath: v }))}
                                            label={domainTerms.toolPath}
                                            hint="Trajectory overlay"
                                        />
                                        <Toggle
                                            checked={layers.workpiece}
                                            onChange={(v) => setLayers((x: any) => ({ ...x, workpiece: v }))}
                                            label={domainTerms.workpiece}
                                            hint="Fixture + target surface"
                                        />
                                        <Toggle
                                            checked={layers.profileSegments}
                                            onChange={(v) => setLayers((x: any) => ({ ...x, profileSegments: v }))}
                                            label={domainTerms.segmentPlural}
                                            hint={`${domainTerms.depositionView} geometry (colored by time/quality)`}
                                        />
                                        <Toggle
                                            checked={layers.frames}
                                            onChange={(v) => setLayers((x: any) => ({ ...x, frames: v }))}
                                            label="Coordinate frames"
                                            hint="Axes, TF frames"
                                        />
                                        <Divider />
                                        <div className="flex items-center justify-between">
                                            <Button size="sm" variant="secondary" onClick={resetView}>
                                                Reset view
                                            </Button>
                                            <Button
                                                size="sm"
                                                variant="ghost"
                                                onClick={() =>
                                                    setLayers({
                                                        robotModel: true,
                                                        torchPath: true,
                                                        workpiece: true,
                                                        profileSegments: true,
                                                        frames: false,
                                                    })
                                                }
                                            >
                                                Default layers
                                            </Button>
                                        </div>
                                    </div>
                                </div>
                            ) : null}
                        </div>

                        <Select
                            value={camera}
                            onChange={(v) => setCamera(v as any)}
                            options={[
                                { value: "Isometric", label: "Camera: Isometric" },
                                { value: "Top", label: "Camera: Top" },
                                { value: "Side", label: "Camera: Side" },
                            ]}
                            className="w-44"
                        />

                        <Button
                            size="sm"
                            variant={replay ? "primary" : "secondary"}
                            onClick={() => setReplay(!replay)}
                            title="Replay timeline (demo)"
                        >
                            {replay ? (
                                <>
                                    <Play className="h-4 w-4" /> Replay ON
                                </>
                            ) : (
                                <>
                                    <Pause className="h-4 w-4" /> Replay OFF
                                </>
                            )}
                        </Button>
                    </div>
                }
            />

            <CardBody className="space-y-3">
                <div className="relative h-[420px] overflow-hidden rounded-xl border border-slate-200 dark:border-slate-800">
                    <iframe
                        src={import.meta.env.VITE_VISER_URL || "http://localhost:8081"}
                        className="w-full h-full border-0"
                        title="Viser 3D Viewer"
                        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope"
                    />

                    <div className="absolute top-3 left-3 right-3 pointer-events-none">
                        <div className="flex items-start justify-between gap-3">
                            <div className="flex flex-wrap items-center gap-2 pointer-events-auto">
                                <Chip tone="info">
                                    <Camera className="h-3.5 w-3.5" />
                                    {vizMode === "execution" ? "Robot execution view" : `${domainTerms.depositionView} view`}
                                </Chip>
                                <Chip tone="ghost">{camera}</Chip>
                                <Chip tone="ghost">{layerCount} layers</Chip>
                            </div>

                            <div className="grid gap-2 pointer-events-auto">
                                <div className="rounded-xl border border-slate-200 bg-white/80 p-2 backdrop-blur dark:border-slate-800 dark:bg-slate-950/80">
                                    <div className="text-xs font-semibold">Robots</div>
                                    <div className="mt-1 space-y-1">
                                        <KV
                                            k={`${robots.robotA.name} (${robots.robotA.position})`}
                                            v={<Chip tone={robots.robotA.state === "Running" ? "good" : robots.robotA.state === "Paused" ? "warn" : "neutral"}>{robots.robotA.state}</Chip>}
                                        />
                                        <KV
                                            k={`${robots.robotB.name} (${robots.robotB.position})`}
                                            v={<Chip tone={robots.robotB.state === "Running" ? "good" : robots.robotB.state === "Paused" ? "warn" : "neutral"}>{robots.robotB.state}</Chip>}
                                        />
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>

                    <div className="absolute bottom-3 left-3 right-3 pointer-events-auto">
                        <div className="rounded-xl border border-slate-200 bg-white/80 p-3 backdrop-blur dark:border-slate-800 dark:bg-slate-950/80">
                            <div className="flex flex-wrap items-center justify-between gap-3">
                                <div className="flex items-center gap-2">
                                    <Chip tone="ghost">
                                        <Clock className="h-3.5 w-3.5" />
                                        t=<span className="font-mono">{Math.round(timelineT)}s</span>
                                    </Chip>
                                    <Chip tone="ghost">
                                        max <span className="font-mono">{maxT}s</span>
                                    </Chip>
                                </div>

                                <div className="flex items-center gap-2">
                                    <Button
                                        size="sm"
                                        variant="secondary"
                                        onClick={() => setTimelineT(Math.max(0, timelineT - 10))}
                                    >
                                        -10s
                                    </Button>
                                    <Button
                                        size="sm"
                                        variant="secondary"
                                        onClick={() => setTimelineT(Math.min(maxT, timelineT + 10))}
                                    >
                                        +10s
                                    </Button>
                                    <Button
                                        size="sm"
                                        variant="secondary"
                                        onClick={() => {
                                            setReplay(false);
                                            setTimelineT(maxT);
                                        }}
                                        title="Snap to live"
                                    >
                                        Live
                                    </Button>
                                </div>
                            </div>

                            <div className="mt-2">
                                <input
                                    type="range"
                                    min={0}
                                    max={maxT}
                                    value={Math.max(0, Math.min(maxT, timelineT))}
                                    onChange={(e) => setTimelineT(Number(e.target.value))}
                                    className="w-full accent-slate-900 dark:accent-slate-100"
                                />
                                <div className="mt-1 flex items-center justify-between text-[11px] text-slate-600 dark:text-slate-400">
                                    <span>0s</span>
                                    <span>scrub to replay</span>
                                    <span>{maxT}s</span>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>

                <div className="grid grid-cols-2 gap-3 md:grid-cols-4">
                    <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                        <KV k="Mode" v={vizMode === "execution" ? "Robot execution" : domainTerms.depositionView} />
                        <KV k="Camera" v={camera} />
                    </div>
                    <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                        <KV k={`Robot A ${domainTerms.segment.toLowerCase()}`} v={`#${robots.robotA.segmentIndex}`} mono />
                        <KV k={`Robot B ${domainTerms.segment.toLowerCase()}`} v={`#${robots.robotB.segmentIndex}`} mono />
                    </div>
                    <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                        <KV k="Layers enabled" v={`${layerCount}/5`} mono />
                        <KV k="Frames" v={layers.frames ? "on" : "off"} />
                    </div>
                    <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                        <KV k="Replay" v={replay ? "on" : "off"} />
                        <KV k="Scrub time" v={`${Math.round(timelineT)}s`} mono />
                    </div>
                </div>
            </CardBody>
        </Card>
    );
}

// =====================
// TelemetryPanel Sub-component
// =====================

function TelemetryPanel({
    telemetryChartData,
    metric,
    setMetric,
    showRobot,
    setShowRobot,
    freezeCharts,
    setFreezeCharts,
    metricLabel,
    lastValues,
}: {
    telemetryChartData: any[];
    metric: "speed" | "current" | "voltage" | "profileHeight" | "profileWidth";
    setMetric: (v: any) => void;
    showRobot: "A" | "B" | "Both";
    setShowRobot: (v: any) => void;
    freezeCharts: boolean;
    setFreezeCharts: (v: boolean) => void;
    metricLabel: string;
    lastValues: { A: number; B: number };
}) {
    return (
        <Card>
            <CardHeader
                title={
                    <span className="flex items-center gap-2">
                        <Activity className="h-4 w-4" />
                        Telemetry
                    </span>
                }
                subtitle="Live time series"
                right={
                    <Button
                        size="sm"
                        variant={freezeCharts ? "primary" : "secondary"}
                        onClick={() => setFreezeCharts(!freezeCharts)}
                        title="Pause chart updates"
                    >
                        {freezeCharts ? (
                            <>
                                <Pause className="h-4 w-4" />
                                Frozen
                            </>
                        ) : (
                            <>
                                <Play className="h-4 w-4" />
                                Live
                            </>
                        )}
                    </Button>
                }
            />
            <CardBody className="space-y-3">
                <div className="grid grid-cols-2 gap-2">
                    <Select
                        value={metric}
                        onChange={(v) => setMetric(v)}
                        options={[
                            { value: "speed", label: "Metric: Speed" },
                            { value: "current", label: "Metric: Current" },
                            { value: "voltage", label: "Metric: Voltage" },
                            { value: "profileHeight", label: `Metric: ${domainTerms.profileHeight}` },
                            { value: "profileWidth", label: `Metric: ${domainTerms.profileWidth}` },
                        ]}
                    />

                    <Select
                        value={showRobot}
                        onChange={(v) => setShowRobot(v)}
                        options={[
                            { value: "Both", label: "Show: A + B" },
                            { value: "A", label: "Show: A only" },
                            { value: "B", label: "Show: B only" },
                        ]}
                    />
                </div>

                <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                    <div className="flex flex-wrap items-center justify-between gap-2">
                        <div className="text-sm font-semibold">{metricLabel}</div>
                        <div className="flex items-center gap-2">
                            {showRobot !== "B" ? (
                                <Chip tone="ghost">
                                    A: <span className="font-mono">{lastValues.A.toFixed(2)}</span>
                                </Chip>
                            ) : null}
                            {showRobot !== "A" ? (
                                <Chip tone="ghost">
                                    B: <span className="font-mono">{lastValues.B.toFixed(2)}</span>
                                </Chip>
                            ) : null}
                        </div>
                    </div>

                    <div className="mt-3 h-[220px] min-h-[220px] min-w-0">
                        <ResponsiveContainer width="100%" height="100%" minHeight={120} minWidth={160}>
                            <LineChart data={telemetryChartData}>
                                <CartesianGrid strokeDasharray="3 3" />
                                <XAxis dataKey="t" tick={{ fontSize: 12 }} />
                                <YAxis tick={{ fontSize: 12 }} />
                                <Tooltip />
                                <Legend />
                                {showRobot !== "B" ? (
                                    <Line type="monotone" dataKey="A" dot={false} strokeWidth={2} />
                                ) : null}
                                {showRobot !== "A" ? (
                                    <Line type="monotone" dataKey="B" dot={false} strokeWidth={2} />
                                ) : null}
                            </LineChart>
                        </ResponsiveContainer>
                    </div>
                </div>
            </CardBody>
        </Card>
    );
}

// =====================
// TrustPanel Sub-component
// =====================

function TrustPanel({
    trustA,
    trustB,
    warnTh,
    stopTh,
    timelineT,
}: {
    trustA?: TrustAssessment;
    trustB?: TrustAssessment;
    warnTh: number;
    stopTh: number;
    timelineT: number;
}) {
    const toneFor = (g?: TrustGate) => (g === "OK" ? "good" : g === "Warning" ? "warn" : "bad");

    const reasonText = (t?: TrustAssessment) =>
        t ? t.reasons.filter((x) => x !== "nominal").join(", ") || "nominal" : "-";

    return (
        <Card>
            <CardHeader
                title={
                    <span className="flex items-center gap-2">
                        <Shield className="h-4 w-4" />
                        Runtime Trust
                    </span>
                }
                subtitle="Confidence + gate + reasons"
                right={
                    <Chip tone="ghost">
                        t=<span className="font-mono">{Math.round(timelineT)}s</span>
                    </Chip>
                }
            />
            <CardBody className="space-y-3">
                <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                    <div className="flex items-center justify-between">
                        <div className="text-xs text-slate-600 dark:text-slate-400">Thresholds</div>
                        <div className="flex items-center gap-2">
                            <Chip tone="warn">
                                warn <span className="font-mono">{warnTh.toFixed(2)}</span>
                            </Chip>
                            <Chip tone="bad">
                                stop <span className="font-mono">{stopTh.toFixed(2)}</span>
                            </Chip>
                        </div>
                    </div>
                </div>

                <div className="grid gap-2">
                    <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                        <div className="flex items-start justify-between gap-3">
                            <div>
                                <div className="text-sm font-semibold">Robot A (PA)</div>
                                <div className="mt-1 text-xs text-slate-600 dark:text-slate-400">
                                    Reasons: {reasonText(trustA)}
                                </div>
                            </div>
                            <div className="text-right">
                                <Chip tone={toneFor(trustA?.gate) as any}>
                                    <Shield className="h-3.5 w-3.5" />
                                    {trustA?.gate ? trustA.gate.toUpperCase() : "-"}
                                </Chip>
                                <div className="mt-1 text-xs">
                                    conf <span className="font-mono">{(trustA?.confidence ?? 0).toFixed(2)}</span>
                                </div>
                            </div>
                        </div>
                    </div>

                    <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                        <div className="flex items-start justify-between gap-3">
                            <div>
                                <div className="text-sm font-semibold">Robot B (PC)</div>
                                <div className="mt-1 text-xs text-slate-600 dark:text-slate-400">
                                    Reasons: {reasonText(trustB)}
                                </div>
                            </div>
                            <div className="text-right">
                                <Chip tone={toneFor(trustB?.gate) as any}>
                                    <Shield className="h-3.5 w-3.5" />
                                    {trustB?.gate ? trustB.gate.toUpperCase() : "-"}
                                </Chip>
                                <div className="mt-1 text-xs">
                                    conf <span className="font-mono">{(trustB?.confidence ?? 0).toFixed(2)}</span>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </CardBody>
        </Card>
    );
}
