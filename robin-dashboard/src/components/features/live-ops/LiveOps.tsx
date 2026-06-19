import {
    Bot,
    AlertTriangle,
    Activity,
    Shield,
    Camera,
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
    ResponsiveContainer,
} from 'recharts';
import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Chip } from '../../ui/Chip';
import { Button } from '../../ui/Button';
import { Select } from '../../ui/Select';
import { ProgressBar, KV } from '../../ui/ProgressBar';
import { ProcessControlsPanel } from './ProcessControlsPanel';
import { DeviationMonitor } from './DeviationMonitor';
import { MeasurementKPIs } from './MeasurementKPIs';
import type {
    AIInputFeatureSpec,
    RobotCell,
    ProcessRun,
    TrustAssessment,
    TrustGate,
    ProcessControlsState,
    OperationMode,
    TargetGeometry,
    DeviationAction,
} from '../../../types';
import type { RobinMeasurement } from '../../../hooks/useRobinAPI';
import { domainTerms } from '../../../config/domain';

interface LiveOpsProps {
    robot: RobotCell;
    latestTrust?: TrustAssessment;
    trustWarnTh: number;
    trustStopTh: number;
    startRobot: () => void;
    startPending?: boolean;
    pauseRobot: () => void;
    resumeRobot: () => void;
    stopRobot: () => void;
    abortRobot: () => void;
    toggleParamFreeze: () => void;
    currentRun: ProcessRun | null;
    alerts: Array<{ id: string; at: string; severity: "Info" | "Warning" | "Critical"; message: string; source: string }>;
    timelineT: number;
    telemetryChartData: any[];
    metric: string;
    setMetric: (v: any) => void;
    metricOptions: Array<{ value: string; label: string }>;
    freezeCharts: boolean;
    setFreezeCharts: (v: boolean) => void;
    metricLabel: string;
    lastValue: number;
    measurementSource: string;
    measurementPollMs: number;
    processId: string | null;
    processControls: ProcessControlsState;
    aiInputFeatures: AIInputFeatureSpec[];
    onControlsChange: (next: ProcessControlsState) => void;
    onControlsApply: (controls: ProcessControlsState) => void;
    targetGeometry: TargetGeometry | null;
    processMode: OperationMode | null;
    latestMeasurements: RobinMeasurement[] | null;
    measurementCount: number;
    onDeviationAlert: (severity: 'Info' | 'Warning' | 'Critical', message: string) => void;
    onDeviationAction: (action: DeviationAction) => void;
    onDeviationPointEvaluated: (warningFlagged: boolean) => void;
    onDeviationEscalation: () => void;
    warningGateResetToken?: number;
}

export function LiveOps(props: LiveOpsProps) {
    const {
        robot,
        latestTrust,
        trustWarnTh,
        trustStopTh,
        startRobot,
        startPending,
        pauseRobot,
        resumeRobot,
        stopRobot,
        abortRobot,
        toggleParamFreeze,
        currentRun,
        alerts,
        timelineT,
        telemetryChartData,
        metric,
        setMetric,
        metricOptions,
        freezeCharts,
        setFreezeCharts,
        metricLabel,
        lastValue,
        measurementSource,
        measurementPollMs,
        processId,
        processControls,
        aiInputFeatures,
        onControlsChange,
        onControlsApply,
        targetGeometry,
        processMode,
        latestMeasurements,
        measurementCount,
        onDeviationAlert,
        onDeviationAction,
        onDeviationPointEvaluated,
        onDeviationEscalation,
        warningGateResetToken = 0,
    } = props;

    return (
        <div className="space-y-4">
            <MeasurementKPIs
                processId={processId}
                measurements={latestMeasurements}
                count={measurementCount}
                aiInputFeatures={aiInputFeatures}
            />

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
                                robot={robot}
                                trust={latestTrust}
                                trustWarnTh={trustWarnTh}
                                trustStopTh={trustStopTh}
                                onStart={startRobot}
                                startPending={startPending}
                                onPause={pauseRobot}
                                onResume={resumeRobot}
                                onStop={stopRobot}
                                onAbort={abortRobot}
                                onFreeze={toggleParamFreeze}
                            />
                        </CardBody>
                    </Card>

                    <ProcessControlsPanel
                        processId={processId}
                        controls={processControls}
                        aiInputFeatures={aiInputFeatures}
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
                    <VisualizationPanel />
                </div>

                {/* Right column: deviation monitor + telemetry + trust */}
                <div className="col-span-12 space-y-4 xl:col-span-3">
                    <DeviationMonitor
                        processId={processId}
                        controls={processControls}
                        latestMeasurements={latestMeasurements}
                        aiInputFeatures={aiInputFeatures}
                        targetGeometry={targetGeometry}
                        processMode={processMode}
                        onAlert={onDeviationAlert}
                        onAction={onDeviationAction}
                        onPointEvaluated={onDeviationPointEvaluated}
                        onWarningEscalation={onDeviationEscalation}
                        warningGateResetToken={warningGateResetToken}
                    />

                    <TelemetryPanel
                        telemetryChartData={telemetryChartData}
                        metric={metric}
                        setMetric={setMetric}
                        metricOptions={metricOptions}
                        freezeCharts={freezeCharts}
                        setFreezeCharts={setFreezeCharts}
                        metricLabel={metricLabel}
                        lastValue={lastValue}
                        measurementSource={measurementSource}
                        measurementPollMs={measurementPollMs}
                    />

                    <TrustPanel
                        trust={latestTrust}
                        warnTh={trustWarnTh}
                        stopTh={trustStopTh}
                        timelineT={timelineT}
                    />
                </div>
            </div>
        </div>
    );
}

function RobotCard({
    robot,
    trust,
    trustWarnTh,
    trustStopTh,
    onStart,
    startPending = false,
    onPause,
    onResume,
    onStop,
    onAbort,
    onFreeze,
}: {
    robot: RobotCell;
    trust?: TrustAssessment;
    trustWarnTh: number;
    trustStopTh: number;
    onStart: () => void;
    startPending?: boolean;
    onPause: () => void;
    onResume: () => void;
    onStop: () => void;
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
            <Button size="sm" onClick={onStart} disabled={startPending}>
                <Play className="h-4 w-4" /> {startPending ? 'Preparing…' : 'Start'}
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
                    <Button size="sm" variant="secondary" onClick={onStop}>
                        Stop
                    </Button>
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

function VisualizationPanel() {
    return (
        <Card className="overflow-hidden">
            <CardHeader
                title={
                    <span className="flex items-center gap-2">
                        <Camera className="h-4 w-4" />
                        3D Visualization
                    </span>
                }
            />
            <CardBody className="space-y-3">
                <div className="relative h-[420px] overflow-hidden rounded-xl border border-slate-200 dark:border-slate-800">
                    <iframe
                        src={(() => {
                            const base = import.meta.env.VITE_LICHTBLICK_URL || 'http://localhost:8080';
                            const wsUrl = import.meta.env.VITE_FOXGLOVE_WS_URL || 'ws://localhost:8765';
                            return `${base}/?ds=foxglove-websocket&ds.url=${encodeURIComponent(wsUrl)}`;
                        })()}
                        className="w-full h-full border-0"
                        title="Lichtblick 3D Viewer"
                        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; cross-origin-isolated"
                    />
                </div>
            </CardBody>
        </Card>
    );
}

function TelemetryPanel({
    telemetryChartData,
    metric,
    setMetric,
    metricOptions,
    freezeCharts,
    setFreezeCharts,
    metricLabel,
    lastValue,
    measurementSource,
    measurementPollMs,
}: {
    telemetryChartData: any[];
    metric: string;
    setMetric: (v: any) => void;
    metricOptions: Array<{ value: string; label: string }>;
    freezeCharts: boolean;
    setFreezeCharts: (v: boolean) => void;
    metricLabel: string;
    lastValue: number;
    measurementSource: string;
    measurementPollMs: number;
}) {
    const sourceTone = measurementSource === 'mintaka' || measurementSource === 'troe' ? 'good' : measurementSource === 'orion' ? 'warn' : 'neutral';
    const sourceLabel = measurementSource === 'mintaka'
        ? 'Mintaka stored data'
        : measurementSource === 'troe'
            ? 'TROE live data'
            : measurementSource === 'orion'
                ? 'Orion fallback data'
                : 'No telemetry source';
    const fmtTimestamp = (value: unknown) => {
        if (typeof value === 'string' && value) {
            const ts = Date.parse(value);
            if (Number.isFinite(ts)) return new Date(ts).toLocaleTimeString();
        }
        if (typeof value === 'number' && Number.isFinite(value)) return `t=${value.toFixed(1)}s`;
        return 'n/a';
    };

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
                <Select
                    value={metric}
                    onChange={(v) => setMetric(v)}
                    options={metricOptions}
                />

                <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                    <div className="flex flex-wrap items-center justify-between gap-2">
                        <div className="text-sm font-semibold">{metricLabel}</div>
                        <div className="flex flex-wrap items-center gap-2">
                            <Chip tone={sourceTone as any}>{sourceLabel}</Chip>
                            <Chip tone="ghost">Poll {Math.round(measurementPollMs / 1000)}s</Chip>
                            <Chip tone="ghost">
                                <span className="font-mono">{lastValue.toFixed(2)}</span>
                            </Chip>
                        </div>
                    </div>

                    <div className="mt-3 h-[220px] min-h-[220px] min-w-0">
                        <ResponsiveContainer width="100%" height="100%" minHeight={120} minWidth={160}>
                            <LineChart data={telemetryChartData}>
                                <CartesianGrid strokeDasharray="3 3" />
                                <XAxis dataKey="absT" tick={{ fontSize: 12 }} tickFormatter={(v) => new Date(v * 1000).toLocaleTimeString([], { hour12: false })} />
                                <YAxis tick={{ fontSize: 12 }} />
                                <Tooltip
                                    labelFormatter={(_value, payload) =>
                                        fmtTimestamp(payload && payload[0] ? payload[0].payload.timestamp ?? _value : _value)
                                    }
                                />
                                <Line type="monotone" dataKey="value" dot={false} strokeWidth={2} name={metricLabel} />
                            </LineChart>
                        </ResponsiveContainer>
                    </div>
                </div>
            </CardBody>
        </Card>
    );
}

function TrustPanel({
    trust,
    warnTh,
    stopTh,
    timelineT,
}: {
    trust?: TrustAssessment;
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

                <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                    <div className="flex items-start justify-between gap-3">
                        <div>
                            <div className="text-sm font-semibold">Robot</div>
                            <div className="mt-1 text-xs text-slate-600 dark:text-slate-400">
                                Reasons: {reasonText(trust)}
                            </div>
                        </div>
                        <div className="text-right">
                            <Chip tone={toneFor(trust?.gate) as any}>
                                <Shield className="h-3.5 w-3.5" />
                                {trust?.gate ? trust.gate.toUpperCase() : "-"}
                            </Chip>
                            <div className="mt-1 text-xs">
                                conf <span className="font-mono">{(trust?.confidence ?? 0).toFixed(2)}</span>
                            </div>
                        </div>
                    </div>
                </div>
            </CardBody>
        </Card>
    );
}
