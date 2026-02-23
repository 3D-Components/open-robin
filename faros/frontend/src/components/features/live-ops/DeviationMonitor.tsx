import { useState, useEffect, useCallback, useRef } from 'react';
import {
    AlertTriangle,
    ShieldAlert,
    Crosshair,
    SlidersHorizontal,
    Sparkles,
    Database,
    FlaskConical,
    CheckCircle2,
    Loader2,
} from 'lucide-react';
import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Chip } from '../../ui/Chip';
import { Button } from '../../ui/Button';
import { KV, Divider } from '../../ui/ProgressBar';
import {
    checkDeviation,
    type DeviationCheckResponse,
    type RobinMeasurement,
} from '../../../hooks/useRobinAPI';
import type {
    OperationMode,
    ProcessControlsState,
    TargetGeometry,
    DeviationAction,
    MeasurementSnapshot,
} from '../../../types';

interface DeviationMonitorProps {
    processId: string | null;
    controls: ProcessControlsState;
    latestMeasurements: RobinMeasurement[] | null;
    targetGeometry: TargetGeometry | null;
    processMode: OperationMode | null;
    onAlert: (severity: 'Info' | 'Warning' | 'Critical', message: string) => void;
    onAction: (action: DeviationAction) => void;
    pollIntervalMs?: number;
}

function fmt(v: number | null | undefined, decimals = 2, unit = ''): string {
    if (v === null || v === undefined || isNaN(v)) return '-';
    const s = v.toFixed(decimals);
    return unit ? `${s} ${unit}` : s;
}

function fmtPct(v: number | null | undefined): string {
    if (v === null || v === undefined || isNaN(v)) return '-';
    return `${v.toFixed(1)}%`;
}

export function DeviationMonitor({
    processId,
    controls,
    latestMeasurements,
    targetGeometry: _targetGeometry,
    processMode,
    onAlert,
    onAction,
    pollIntervalMs = 3000,
}: DeviationMonitorProps) {
    const [deviationResult, setDeviationResult] = useState<DeviationCheckResponse | null>(null);
    const [checking, setChecking] = useState(false);
    const [statusMsg, setStatusMsg] = useState('Waiting for measurements…');
    const [lastCheckAt, setLastCheckAt] = useState<string | null>(null);
    const prevDeviationRef = useRef<number | null>(null);
    const [snapshot, setSnapshot] = useState<MeasurementSnapshot | null>(null);

    const extractSnapshot = useCallback((measurements: RobinMeasurement[]): MeasurementSnapshot | null => {
        if (!measurements.length) return null;
        const latest = measurements[measurements.length - 1];
        return {
            height: latest.height ?? null,
            width: latest.width ?? null,
            speed: latest.speed ?? null,
            current: latest.current ?? null,
            voltage: latest.voltage ?? null,
            timestamp: latest.timestamp ?? null,
        };
    }, []);

    const runCheck = useCallback(async () => {
        if (!processId || !latestMeasurements?.length) return;

        const snap = extractSnapshot(latestMeasurements);
        if (!snap || snap.height === null || snap.width === null) return;
        setSnapshot(snap);

        const mode: OperationMode = processMode ?? controls.mode;

        const payload: Parameters<typeof checkDeviation>[0] = {
            process_id: processId,
            mode,
            tolerance: controls.tolerance,
            measured_geometry: {
                height: snap.height,
                width: snap.width,
            },
        };

        if (mode === 'parameter_driven') {
            payload.input_params = {
                wireSpeed: snap.speed ?? controls.wireSpeed,
                current: snap.current ?? controls.current,
                voltage: snap.voltage ?? controls.voltage,
            };
        }

        setChecking(true);
        try {
            const result = await checkDeviation(payload);
            setDeviationResult(result);
            setLastCheckAt(new Date().toISOString());

            if (result.deviation_percentage != null && result.deviation_percentage > 0) {
                setStatusMsg(`Deviation detected (${fmtPct(result.deviation_percentage)})`);
                if (
                    prevDeviationRef.current === null ||
                    Math.abs((result.deviation_percentage ?? 0) - prevDeviationRef.current) > 2
                ) {
                    onAlert(
                        result.deviation_percentage > controls.tolerance ? 'Critical' : 'Warning',
                        `Profile deviation ${fmtPct(result.deviation_percentage)} - H: ${fmtPct(result.deviation_breakdown?.height)}, W: ${fmtPct(result.deviation_breakdown?.width)}`,
                    );
                }
                prevDeviationRef.current = result.deviation_percentage ?? null;
            } else if (result.status === 'error') {
                setStatusMsg(`Error: ${result.message ?? 'unknown'}`);
            } else {
                setStatusMsg('Within tolerance');
                prevDeviationRef.current = null;
            }
        } catch {
            setStatusMsg('Failed to check deviation');
            setDeviationResult(null);
        } finally {
            setChecking(false);
        }
    }, [processId, latestMeasurements, controls, processMode, onAlert, extractSnapshot]);

    useEffect(() => {
        if (!processId || !latestMeasurements?.length) {
            setStatusMsg('Waiting for measurements…');
            return;
        }

        runCheck();
        const id = setInterval(runCheck, pollIntervalMs);
        return () => clearInterval(id);
    }, [processId, latestMeasurements, pollIntervalMs, runCheck]);

    useEffect(() => {
        if (latestMeasurements?.length) {
            setSnapshot(extractSnapshot(latestMeasurements));
        }
    }, [latestMeasurements, extractSnapshot]);

    const hasDeviation =
        deviationResult?.deviation_percentage != null &&
        deviationResult.deviation_percentage > 0;
    const isOverTolerance =
        hasDeviation && (deviationResult!.deviation_percentage! > controls.tolerance);

    const mode = processMode ?? controls.mode;

    return (
        <Card>
            <CardHeader
                title={
                    <span className="flex items-center gap-2">
                        <ShieldAlert className="h-4 w-4" />
                        Deviation Monitor
                    </span>
                }
                subtitle={statusMsg}
                right={
                    <div className="flex items-center gap-2">
                        {checking && <Loader2 className="h-3.5 w-3.5 animate-spin text-slate-400" />}
                        <Chip tone={isOverTolerance ? 'bad' : hasDeviation ? 'warn' : 'good'}>
                            {isOverTolerance ? 'ALERT' : hasDeviation ? 'Deviation' : 'OK'}
                        </Chip>
                    </div>
                }
            />
            <CardBody className="space-y-3">
                {/* Status row */}
                <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                    <div className="grid grid-cols-3 gap-2 text-xs">
                        <div>
                            <span className="text-slate-500">Mode</span>
                            <div className="mt-0.5 font-medium">
                                {mode === 'parameter_driven' ? 'Parameter-driven' : 'Geometry-driven'}
                            </div>
                        </div>
                        <div>
                            <span className="text-slate-500">Tolerance</span>
                            <div className="mt-0.5 font-mono font-medium">{controls.tolerance}%</div>
                        </div>
                        <div>
                            <span className="text-slate-500">Last check</span>
                            <div className="mt-0.5 font-mono">
                                {lastCheckAt ? new Date(lastCheckAt).toLocaleTimeString() : '-'}
                            </div>
                        </div>
                    </div>
                </div>

                {/* Measurement panel */}
                <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                    <div className="flex items-center gap-2 text-sm font-semibold">
                        <Crosshair className="h-3.5 w-3.5" />
                        Latest Measurement
                    </div>
                    {snapshot ? (
                        <div className="mt-2 grid grid-cols-2 gap-x-4 gap-y-1 text-xs">
                            <KV k="Height" v={<span className="font-mono">{fmt(snapshot.height, 2, 'mm')}</span>} />
                            <KV k="Width" v={<span className="font-mono">{fmt(snapshot.width, 2, 'mm')}</span>} />
                            <KV k="Speed" v={<span className="font-mono">{fmt(snapshot.speed, 2, 'mm/s')}</span>} />
                            <KV k="Current" v={<span className="font-mono">{fmt(snapshot.current, 1, 'A')}</span>} />
                            <KV k="Voltage" v={<span className="font-mono">{fmt(snapshot.voltage, 1, 'V')}</span>} />
                            <KV
                                k="Time"
                                v={
                                    <span className="text-[11px]">
                                        {snapshot.timestamp
                                            ? new Date(snapshot.timestamp).toLocaleTimeString()
                                            : '-'}
                                    </span>
                                }
                            />
                        </div>
                    ) : (
                        <div className="mt-2 text-xs text-slate-500">Awaiting measurements</div>
                    )}
                </div>

                {/* Expected panel */}
                {deviationResult?.expected_value && (
                    <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                        <div className="flex items-center gap-2 text-sm font-semibold">
                            <Crosshair className="h-3.5 w-3.5" />
                            Expected Geometry
                        </div>
                        <div className="mt-2 grid grid-cols-2 gap-x-4 gap-y-1 text-xs">
                            <KV
                                k="Height"
                                v={<span className="font-mono">{fmt(deviationResult.expected_value.height, 2, 'mm')}</span>}
                            />
                            <KV
                                k="Width"
                                v={<span className="font-mono">{fmt(deviationResult.expected_value.width, 2, 'mm')}</span>}
                            />
                            {deviationResult.deviation_breakdown && (
                                <>
                                    <KV
                                        k="Height dev"
                                        v={
                                            <Chip tone={Math.abs(deviationResult.deviation_breakdown.height) > controls.tolerance ? 'bad' : 'warn'}>
                                                {fmtPct(deviationResult.deviation_breakdown.height)}
                                            </Chip>
                                        }
                                    />
                                    <KV
                                        k="Width dev"
                                        v={
                                            <Chip tone={Math.abs(deviationResult.deviation_breakdown.width) > controls.tolerance ? 'bad' : 'warn'}>
                                                {fmtPct(deviationResult.deviation_breakdown.width)}
                                            </Chip>
                                        }
                                    />
                                </>
                            )}
                        </div>
                        <div className="mt-2 text-xs text-slate-500">
                            Source: {mode === 'geometry_driven' ? 'Geometry target' : 'AI model prediction'}
                        </div>
                    </div>
                )}

                {/* Alert card */}
                {hasDeviation && (
                    <>
                        <div
                            className={`rounded-xl border p-3 ${
                                isOverTolerance
                                    ? 'border-red-300 bg-red-50 dark:border-red-800 dark:bg-red-950/30'
                                    : 'border-amber-300 bg-amber-50 dark:border-amber-800 dark:bg-amber-950/30'
                            }`}
                        >
                            <div className="flex items-start gap-2">
                                <AlertTriangle
                                    className={`h-5 w-5 mt-0.5 shrink-0 ${
                                        isOverTolerance ? 'text-red-600 dark:text-red-400' : 'text-amber-600 dark:text-amber-400'
                                    }`}
                                />
                                <div className="min-w-0">
                                    <div
                                        className={`text-sm font-semibold ${
                                            isOverTolerance ? 'text-red-900 dark:text-red-100' : 'text-amber-900 dark:text-amber-100'
                                        }`}
                                    >
                                        {isOverTolerance
                                            ? 'Deviation exceeds tolerance!'
                                            : 'Deviation detected'}
                                    </div>
                                    <div className="mt-1 grid grid-cols-2 gap-x-4 gap-y-1 text-xs">
                                        <KV
                                            k="Overall"
                                            v={<span className="font-mono font-bold">{fmtPct(deviationResult!.deviation_percentage)}</span>}
                                        />
                                        <KV k="Tolerance" v={<span className="font-mono">{controls.tolerance}%</span>} />
                                        {deviationResult?.measured_value && (
                                            <KV
                                                k="Measured"
                                                v={
                                                    <span className="font-mono">
                                                        {fmt(deviationResult.measured_value.height, 2)} x{' '}
                                                        {fmt(deviationResult.measured_value.width, 2)} mm
                                                    </span>
                                                }
                                            />
                                        )}
                                        {deviationResult?.expected_value && (
                                            <KV
                                                k="Expected"
                                                v={
                                                    <span className="font-mono">
                                                        {fmt(deviationResult.expected_value.height, 2)} x{' '}
                                                        {fmt(deviationResult.expected_value.width, 2)} mm
                                                    </span>
                                                }
                                            />
                                        )}
                                        {deviationResult?.deviation_breakdown && (
                                            <>
                                                <KV
                                                    k="H deviation"
                                                    v={<span className="font-mono">{fmtPct(deviationResult.deviation_breakdown.height)}</span>}
                                                />
                                                <KV
                                                    k="W deviation"
                                                    v={<span className="font-mono">{fmtPct(deviationResult.deviation_breakdown.width)}</span>}
                                                />
                                            </>
                                        )}
                                    </div>
                                </div>
                            </div>
                        </div>

                        <Divider />

                        {/* Action buttons */}
                        <div className="grid grid-cols-2 gap-2">
                            <Button
                                size="sm"
                                variant="secondary"
                                onClick={() => onAction('manual_adjust')}
                                title="Manually adjust parameters"
                            >
                                <SlidersHorizontal className="h-3.5 w-3.5" />
                                Manual Adjust
                            </Button>
                            <Button
                                size="sm"
                                variant="secondary"
                                onClick={() => onAction('new_ai_recommendation')}
                                title="Request new AI recommendation"
                            >
                                <Sparkles className="h-3.5 w-3.5" />
                                AI Recommend
                            </Button>
                            <Button
                                size="sm"
                                variant="secondary"
                                onClick={() => onAction('add_data_finetune')}
                                title="Add data and fine-tune model"
                            >
                                <Database className="h-3.5 w-3.5" />
                                Fine-tune
                            </Button>
                            <Button
                                size="sm"
                                variant="secondary"
                                onClick={() => onAction('start_new_doe')}
                                title="Start new design of experiments"
                            >
                                <FlaskConical className="h-3.5 w-3.5" />
                                New DOE
                            </Button>
                        </div>
                    </>
                )}

                {!hasDeviation && deviationResult && (
                    <div className="rounded-xl border border-green-200 bg-green-50 p-3 dark:border-green-800 dark:bg-green-950/30">
                        <div className="flex items-center gap-2 text-sm text-green-800 dark:text-green-200">
                            <CheckCircle2 className="h-4 w-4" />
                            <span className="font-medium">Within tolerance</span>
                        </div>
                    </div>
                )}
            </CardBody>
        </Card>
    );
}
