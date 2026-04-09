import { useState, useEffect, useCallback, useRef } from 'react';
import {
    AlertTriangle,
    ShieldAlert,
    Crosshair,
    SlidersHorizontal,
    Sparkles,
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
import { resolveRecordedAIInputParams } from '../../../config/aiInputFeatures';
import type {
    AIInputFeatureSpec,
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
    aiInputFeatures: AIInputFeatureSpec[];
    targetGeometry: TargetGeometry | null;
    processMode: OperationMode | null;
    onAlert: (severity: 'Info' | 'Warning' | 'Critical', message: string) => void;
    onAction: (action: DeviationAction) => void;
    onPointEvaluated?: (warningFlagged: boolean) => void;
    onWarningEscalation?: () => void;
    warningGateResetToken?: number;
    pollIntervalMs?: number;
    dataStaleThresholdMs?: number;
}

const WARNING_STREAK_THRESHOLD = 2;

function fmt(v: number | null | undefined, decimals = 2, unit = ''): string {
    if (v === null || v === undefined || isNaN(v)) return '-';
    const s = v.toFixed(decimals);
    return unit ? `${s} ${unit}` : s;
}

function fmtPct(v: number | null | undefined): string {
    if (v === null || v === undefined || isNaN(v)) return '-';
    return `${v.toFixed(1)}%`;
}

function isWarningResult(result: DeviationCheckResponse, tolerance: number): boolean {
    if (result.status === 'alert') return true;
    if (result.status === 'error' || result.status === 'process_inactive' || result.status === 'no_data') {
        return false;
    }
    if (result.deviation_percentage == null || Number.isNaN(result.deviation_percentage)) return false;
    return result.deviation_percentage > tolerance;
}

export function DeviationMonitor({
    processId,
    controls,
    latestMeasurements,
    aiInputFeatures,
    targetGeometry: _targetGeometry,
    processMode,
    onAlert,
    onAction,
    onPointEvaluated,
    onWarningEscalation,
    warningGateResetToken = 0,
    pollIntervalMs = 3000,
    dataStaleThresholdMs = 30_000,
}: DeviationMonitorProps) {
    const [deviationResult, setDeviationResult] = useState<DeviationCheckResponse | null>(null);
    const [checking, setChecking] = useState(false);
    const [statusMsg, setStatusMsg] = useState('Waiting for measurements…');
    const [lastCheckAt, setLastCheckAt] = useState<string | null>(null);
    const lastEvaluatedPointRef = useRef<string | null>(null);
    const warningStreakRef = useRef(0);
    const warningEscalatedRef = useRef(false);
    const [warningStreak, setWarningStreak] = useState(0);
    const [warningEscalated, setWarningEscalated] = useState(false);
    const [snapshot, setSnapshot] = useState<MeasurementSnapshot | null>(null);

    const extractSnapshot = useCallback((measurements: RobinMeasurement[]): MeasurementSnapshot | null => {
        if (!measurements.length) return null;
        const latest = measurements[measurements.length - 1];
        return {
            height: latest.height ?? null,
            width: latest.width ?? null,
            inputParams: resolveRecordedAIInputParams(
                latest as unknown as Record<string, unknown>,
                aiInputFeatures,
            ),
            timestamp: latest.timestamp ?? null,
        };
    }, [aiInputFeatures]);

    const runCheck = useCallback(async () => {
        if (!processId || !latestMeasurements?.length) return;

        const snap = extractSnapshot(latestMeasurements);
        if (!snap || snap.height === null || snap.width === null) return;

        // If the latest data point is older than the threshold, show idle state and bail
        if (snap.timestamp) {
            const ageMs = Date.now() - new Date(snap.timestamp).getTime();
            if (ageMs > dataStaleThresholdMs) {
                const ageSec = Math.round(ageMs / 1000);
                setStatusMsg(`No active data (last seen ${ageSec}s ago)`);
                setChecking(false);
                return;
            }
        }

        setSnapshot(snap);

        const mode: OperationMode = processMode ?? controls.mode;
        const inputParams = controls.inputParams;

        const payload: Parameters<typeof checkDeviation>[0] = {
            process_id: processId,
            mode,
            tolerance: controls.tolerance,
            measured_geometry: {
                height: snap.height,
                width: snap.width,
            },
            input_params: inputParams,
        };

        setChecking(true);
        try {
            const result = await checkDeviation(payload);
            setLastCheckAt(new Date().toISOString());

            if (result.status === 'process_inactive') {
                setStatusMsg(
                    warningEscalatedRef.current
                        ? 'Paused for warning review'
                        : 'Process inactive; waiting for resume',
                );
                return;
            }

            if (result.status === 'no_data') {
                setStatusMsg('Waiting for measurements…');
                return;
            }

            if (result.status !== 'error') {
                setDeviationResult(result);
            }

            const warningFlagged = isWarningResult(result, controls.tolerance);

            if (result.status !== 'error') {
                const pointKey =
                    snap.timestamp
                    ?? `${latestMeasurements.length}:${snap.height}:${snap.width}`;
                if (lastEvaluatedPointRef.current !== pointKey) {
                    onPointEvaluated?.(warningFlagged);
                    if (warningFlagged) {
                        const nextStreak = warningStreakRef.current + 1;
                        warningStreakRef.current = nextStreak;
                        setWarningStreak(nextStreak);

                        if (nextStreak >= WARNING_STREAK_THRESHOLD && !warningEscalatedRef.current) {
                            warningEscalatedRef.current = true;
                            setWarningEscalated(true);
                            onAlert(
                                (result.deviation_percentage ?? 0) > controls.tolerance ? 'Critical' : 'Warning',
                                `Persistent deviation ${fmtPct(result.deviation_percentage)} detected in ${WARNING_STREAK_THRESHOLD} consecutive checks. Process paused for review.`,
                            );
                            onWarningEscalation?.();
                        }
                    } else {
                        warningStreakRef.current = 0;
                        warningEscalatedRef.current = false;
                        setWarningStreak(0);
                        setWarningEscalated(false);
                    }
                    lastEvaluatedPointRef.current = pointKey;
                }
            }

            if (warningFlagged) {
                if (warningEscalatedRef.current) {
                    setStatusMsg(`Persistent deviation (${fmtPct(result.deviation_percentage)})`);
                } else {
                    setStatusMsg(
                        `Potential deviation (${warningStreakRef.current}/${WARNING_STREAK_THRESHOLD})`,
                    );
                }
            } else if (result.status === 'error') {
                setStatusMsg(`Error: ${result.message ?? 'unknown'}`);
            } else {
                setStatusMsg('Within tolerance');
                warningStreakRef.current = 0;
                warningEscalatedRef.current = false;
                setWarningStreak(0);
                setWarningEscalated(false);
            }
        } catch {
            setStatusMsg('Failed to check deviation');
        } finally {
            setChecking(false);
        }
    }, [processId, latestMeasurements, controls, processMode, onAlert, onPointEvaluated, onWarningEscalation, extractSnapshot, dataStaleThresholdMs]);

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
        lastEvaluatedPointRef.current = null;
        warningStreakRef.current = 0;
        warningEscalatedRef.current = false;
        setWarningStreak(0);
        setWarningEscalated(false);
        setDeviationResult(null);
    }, [processId, warningGateResetToken]);

    useEffect(() => {
        if (latestMeasurements?.length) {
            setSnapshot(extractSnapshot(latestMeasurements));
        }
    }, [latestMeasurements, extractSnapshot]);

    const hasDeviation = warningEscalated;
    const isWithinTolerance =
        deviationResult?.status !== 'error' &&
        deviationResult?.deviation_percentage != null &&
        deviationResult.deviation_percentage <= 0;

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
                        <Chip tone={hasDeviation ? 'bad' : 'good'}>
                            {hasDeviation ? 'ALERT' : 'OK'}
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
                    <div className="mt-2 text-[11px] text-slate-500">
                        Warning gate: {warningStreak}/{WARNING_STREAK_THRESHOLD} consecutive deviation points
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
                            {aiInputFeatures.slice(0, 3).map((feature) => (
                                <KV
                                    key={feature.key}
                                    k={feature.label}
                                    v={
                                        <span className="font-mono">
                                            {fmt(
                                                snapshot.inputParams[feature.key],
                                                feature.step && feature.step < 1 ? 3 : 2,
                                                feature.unit,
                                            )}
                                        </span>
                                    }
                                />
                            ))}
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
                            Source: {
                                deviationResult.expected_source === 'ai_prediction_from_params'
                                    ? 'AI model prediction'
                                    : 'Geometry target'
                            }
                        </div>
                    </div>
                )}

                {/* Alert card */}
                {hasDeviation && (
                    <>
                        <div className="rounded-xl border border-red-300 bg-red-50 p-3 dark:border-red-800 dark:bg-red-950/30">
                            <div className="flex items-start gap-2">
                                <AlertTriangle className="h-5 w-5 mt-0.5 shrink-0 text-red-600 dark:text-red-400" />
                                <div className="min-w-0">
                                    <div className="text-sm font-semibold text-red-900 dark:text-red-100">
                                        Deviation exceeds tolerance!
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
                        </div>
                    </>
                )}

                {isWithinTolerance && deviationResult && (
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
