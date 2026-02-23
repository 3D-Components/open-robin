import { useMemo, useState, useCallback } from 'react';
import { domainTerms } from '../../../config/domain';
import {
    Shield,
    Database,
    History,
    Cpu,
    Download,
    Sparkles,
    RefreshCw,
    Loader2,
    CheckCircle2,
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
import { KV, Divider } from '../../ui/ProgressBar';
import {
    selectModel,
    predictGeometry,
    type AIModelsResponse,
} from '../../../hooks/useRobinAPI';
import type { ModelVersion, AuditLogEntry, TrustAssessment } from '../../../types';

interface ModelsTrustTabProps {
    models: ModelVersion[];
    activeModel: string;
    onSwitchModel: (next: string) => void;
    audit: AuditLogEntry[];
    trustWarnTh: number;
    trustStopTh: number;
    setTrustWarnTh: (v: number) => void;
    setTrustStopTh: (v: number) => void;
    latestTrust: TrustAssessment;
    trustFeed: TrustAssessment[];
    aiModelsData: AIModelsResponse | null;
    onModelsRefresh: () => void;
}

function formatBytes(bytes: number | undefined | null): string {
    if (bytes === undefined || bytes === null) return '-';
    const units = ['B', 'KB', 'MB', 'GB'];
    let i = 0;
    let value = bytes;
    while (value >= 1024 && i < units.length - 1) {
        value /= 1024;
        i++;
    }
    return `${value.toFixed(i === 0 ? 0 : 1)} ${units[i]}`;
}

function formatDate(iso: string | undefined | null): string {
    if (!iso) return '-';
    try {
        return new Date(iso).toLocaleString();
    } catch {
        return iso;
    }
}

export function ModelsTrustTab({
    models,
    activeModel,
    onSwitchModel,
    audit,
    trustWarnTh,
    trustStopTh,
    setTrustWarnTh,
    setTrustStopTh,
    latestTrust,
    trustFeed,
    aiModelsData,
    onModelsRefresh,
}: ModelsTrustTabProps) {
    const optionify = (m: ModelVersion) => ({
        value: m.id,
        label: `${m.id} · ${m.goNoGo} · trust ${m.trustScore.toFixed(2)}`,
    });

    const lastAudit = audit.slice(0, 8);

    const trustTail = trustFeed.slice(-40);
    const trustSeries = useMemo(() => {
        return trustTail
            .map((row) => ({ t: row.atT, trust: row.confidence }))
            .sort((a, b) => a.t - b.t);
    }, [trustTail]);

    const aiActiveModel = aiModelsData?.active_model ?? null;
    const checkpoints = aiModelsData?.models ?? [];
    const [selectedPath, setSelectedPath] = useState('');
    const [loadingModel, setLoadingModel] = useState(false);
    const [modelStatus, setModelStatus] = useState('');

    const [predWireSpeed, setPredWireSpeed] = useState(10);
    const [predCurrent, setPredCurrent] = useState(150);
    const [predVoltage, setPredVoltage] = useState(24);
    const [predicting, setPredicting] = useState(false);
    const [predictionResult, setPredictionResult] = useState<Record<string, unknown> | null>(null);

    const handleLoadModel = useCallback(async () => {
        if (!selectedPath) return;
        setLoadingModel(true);
        setModelStatus('Loading model…');
        try {
            await selectModel(selectedPath);
            setModelStatus('Model loaded successfully');
            onModelsRefresh();
        } catch (err) {
            setModelStatus(`Failed: ${err instanceof Error ? err.message : String(err)}`);
        } finally {
            setLoadingModel(false);
        }
    }, [selectedPath, onModelsRefresh]);

    const handlePredict = useCallback(async () => {
        setPredicting(true);
        setPredictionResult(null);
        try {
            const result = await predictGeometry({
                wireSpeed: predWireSpeed,
                current: predCurrent,
                voltage: predVoltage,
            });
            setPredictionResult(result);
        } catch (err) {
            setPredictionResult({
                error: err instanceof Error ? err.message : String(err),
            });
        } finally {
            setPredicting(false);
        }
    }, [predWireSpeed, predCurrent, predVoltage]);

    const isActive = (path: string) => aiActiveModel?.path === path;

    return (
        <div className="grid grid-cols-12 gap-4">
            <div className="col-span-12 xl:col-span-7 space-y-4">
                {/* AI Model Control */}
                <Card>
                    <CardHeader
                        title={
                            <span className="flex items-center gap-2">
                                <Cpu className="h-4 w-4" />
                                AI Model Control
                            </span>
                        }
                        subtitle="Manage model checkpoints, load models, run predictions"
                        right={
                            <Button size="sm" variant="secondary" onClick={onModelsRefresh}>
                                <RefreshCw className="h-3.5 w-3.5" />
                                Refresh
                            </Button>
                        }
                    />
                    <CardBody className="space-y-3">
                        {/* Active model metadata */}
                        <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                            <div className="flex items-center justify-between">
                                <div className="text-sm font-semibold">Active Model</div>
                                {aiActiveModel ? (
                                    <Chip tone="good">
                                        <CheckCircle2 className="h-3 w-3" />
                                        Loaded
                                    </Chip>
                                ) : (
                                    <Chip tone="neutral">No model</Chip>
                                )}
                            </div>
                            {aiActiveModel ? (
                                <div className="mt-2 grid grid-cols-2 gap-x-4 gap-y-1">
                                    <KV k="Name" v={<span className="font-mono text-xs">{aiActiveModel.name ?? '-'}</span>} />
                                    <KV k="Path" v={<span className="font-mono text-xs truncate">{aiActiveModel.path ?? '-'}</span>} />
                                    <KV k="Modified" v={<span className="text-xs">{formatDate(aiActiveModel.created)}</span>} />
                                    <KV k="Size" v={<span className="font-mono text-xs">{formatBytes(aiActiveModel.size_bytes)}</span>} />
                                </div>
                            ) : (
                                <div className="mt-2 text-xs text-slate-500">No model loaded. Select a checkpoint below.</div>
                            )}
                        </div>

                        {/* Checkpoint table */}
                        <div className="overflow-auto rounded-xl border border-slate-200 dark:border-slate-800">
                            <table className="w-full text-sm">
                                <thead className="bg-slate-50 text-left text-xs text-slate-600 dark:bg-slate-900/40 dark:text-slate-400">
                                    <tr>
                                        <th className="px-3 py-2">Checkpoint</th>
                                        <th className="px-3 py-2">Modified</th>
                                        <th className="px-3 py-2">Size</th>
                                        <th className="px-3 py-2 text-right">Action</th>
                                    </tr>
                                </thead>
                                <tbody className="divide-y divide-slate-200 dark:divide-slate-800">
                                    {checkpoints.length === 0 ? (
                                        <tr>
                                            <td colSpan={4} className="px-3 py-4 text-center text-xs text-slate-500">
                                                No checkpoints found
                                            </td>
                                        </tr>
                                    ) : (
                                        checkpoints.map((cp) => (
                                            <tr
                                                key={cp.path}
                                                className={`bg-white dark:bg-slate-950 ${
                                                    isActive(cp.path)
                                                        ? 'ring-1 ring-inset ring-green-300 dark:ring-green-700'
                                                        : ''
                                                }`}
                                            >
                                                <td className="px-3 py-2 font-mono text-xs">
                                                    {cp.name ?? cp.path}
                                                    {isActive(cp.path) && (
                                                        <Chip tone="good" className="ml-2">Active</Chip>
                                                    )}
                                                </td>
                                                <td className="px-3 py-2 text-xs text-slate-600 dark:text-slate-400">
                                                    {formatDate(cp.created)}
                                                </td>
                                                <td className="px-3 py-2 font-mono text-xs">
                                                    {formatBytes(cp.size_bytes)}
                                                </td>
                                                <td className="px-3 py-2 text-right">
                                                    <Button
                                                        size="sm"
                                                        variant={isActive(cp.path) ? 'secondary' : 'primary'}
                                                        disabled={isActive(cp.path) || loadingModel}
                                                        onClick={() => {
                                                            setSelectedPath(cp.path);
                                                            selectModel(cp.path).then(() => {
                                                                onModelsRefresh();
                                                                setModelStatus('Model loaded');
                                                            }).catch((e) => {
                                                                setModelStatus(`Failed: ${e.message}`);
                                                            });
                                                        }}
                                                    >
                                                        {isActive(cp.path) ? (
                                                            'Active'
                                                        ) : (
                                                            <>
                                                                <Download className="h-3 w-3" />
                                                                Load
                                                            </>
                                                        )}
                                                    </Button>
                                                </td>
                                            </tr>
                                        ))
                                    )}
                                </tbody>
                            </table>
                        </div>

                        {/* Model selector + load */}
                        <div className="flex items-center gap-2">
                            <Select
                                value={selectedPath}
                                onChange={setSelectedPath}
                                options={[
                                    { value: '', label: 'Select a checkpoint…' },
                                    ...checkpoints.map((cp) => ({
                                        value: cp.path,
                                        label: cp.name ?? cp.path,
                                    })),
                                ]}
                                className="flex-1"
                            />
                            <Button
                                onClick={handleLoadModel}
                                disabled={!selectedPath || isActive(selectedPath) || loadingModel}
                            >
                                {loadingModel ? (
                                    <Loader2 className="h-4 w-4 animate-spin" />
                                ) : (
                                    <Download className="h-4 w-4" />
                                )}
                                {isActive(selectedPath) ? 'Active' : 'Load Model'}
                            </Button>
                        </div>

                        {modelStatus && (
                            <div className="text-xs text-slate-600 dark:text-slate-400">{modelStatus}</div>
                        )}

                        <Divider />

                        {/* Quick Prediction */}
                        <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                            <div className="flex items-center gap-2 text-sm font-semibold">
                                <Sparkles className="h-4 w-4" />
                                Quick Prediction
                            </div>
                            <div className="mt-2 grid grid-cols-3 gap-2">
                                <label className="space-y-1">
                                    <span className="text-xs text-slate-500">{domainTerms.speed} ({domainTerms.speedUnit})</span>
                                    <input
                                        type="number"
                                        step="0.1"
                                        value={predWireSpeed}
                                        onChange={(e) => setPredWireSpeed(parseFloat(e.target.value) || 0)}
                                        className="w-full rounded-md border border-slate-200 bg-white px-2 py-1.5 text-sm font-mono dark:border-slate-700 dark:bg-slate-900"
                                    />
                                </label>
                                <label className="space-y-1">
                                    <span className="text-xs text-slate-500">{domainTerms.current} ({domainTerms.currentUnit})</span>
                                    <input
                                        type="number"
                                        step="1"
                                        value={predCurrent}
                                        onChange={(e) => setPredCurrent(parseFloat(e.target.value) || 0)}
                                        className="w-full rounded-md border border-slate-200 bg-white px-2 py-1.5 text-sm font-mono dark:border-slate-700 dark:bg-slate-900"
                                    />
                                </label>
                                <label className="space-y-1">
                                    <span className="text-xs text-slate-500">{domainTerms.voltage} ({domainTerms.voltageUnit})</span>
                                    <input
                                        type="number"
                                        step="0.1"
                                        value={predVoltage}
                                        onChange={(e) => setPredVoltage(parseFloat(e.target.value) || 0)}
                                        className="w-full rounded-md border border-slate-200 bg-white px-2 py-1.5 text-sm font-mono dark:border-slate-700 dark:bg-slate-900"
                                    />
                                </label>
                            </div>
                            <div className="mt-2">
                                <Button size="sm" onClick={handlePredict} disabled={predicting}>
                                    {predicting ? (
                                        <Loader2 className="h-3.5 w-3.5 animate-spin" />
                                    ) : (
                                        <Sparkles className="h-3.5 w-3.5" />
                                    )}
                                    Predict Geometry
                                </Button>
                            </div>
                            {predictionResult && (
                                <pre className="mt-2 max-h-40 overflow-auto rounded-lg border border-slate-200 bg-slate-50 p-2 text-xs font-mono dark:border-slate-700 dark:bg-slate-900">
                                    {JSON.stringify(predictionResult, null, 2)}
                                </pre>
                            )}
                        </div>
                    </CardBody>
                </Card>

                {/* Model & Trust */}
                <Card>
                    <CardHeader
                        title={
                            <span className="flex items-center gap-2">
                                <Shield className="h-4 w-4" />
                                Active Model & Trust
                            </span>
                        }
                        subtitle="Current model routing and trust assessment"
                    />
                    <CardBody className="space-y-3">
                        {/* Active model routing */}
                        <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                            <div className="flex items-center justify-between">
                                <div className="text-sm font-semibold">Active Model</div>
                                <Chip tone="info">Current</Chip>
                            </div>
                            <div className="mt-2">
                                <Select
                                    value={activeModel}
                                    onChange={(v) => onSwitchModel(v)}
                                    options={models.map(optionify)}
                                />
                            </div>
                            <div className="mt-2 text-xs text-slate-600 dark:text-slate-400">
                                Active: <span className="font-mono">{activeModel}</span>
                            </div>
                        </div>

                        <Divider />

                        <div className="grid gap-3 md:grid-cols-2">
                            <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                <div className="text-sm font-semibold">Trust thresholds</div>
                                <div className="mt-2 space-y-2">
                                    <KV k="Warning threshold" v={<span className="font-mono">{trustWarnTh.toFixed(2)}</span>} />
                                    <input
                                        type="range"
                                        min={0.3}
                                        max={0.95}
                                        step={0.01}
                                        value={trustWarnTh}
                                        onChange={(e) => setTrustWarnTh(Number(e.target.value))}
                                        className="w-full accent-slate-900 dark:accent-slate-100"
                                    />
                                    <KV k="Stop threshold" v={<span className="font-mono">{trustStopTh.toFixed(2)}</span>} />
                                    <input
                                        type="range"
                                        min={0.1}
                                        max={0.9}
                                        step={0.01}
                                        value={trustStopTh}
                                        onChange={(e) => setTrustStopTh(Number(e.target.value))}
                                        className="w-full accent-slate-900 dark:accent-slate-100"
                                    />
                                    <div className="text-xs text-slate-600 dark:text-slate-400">
                                        Recommended: stop &lt; warn.
                                    </div>
                                </div>
                            </div>

                            <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                <div className="text-sm font-semibold">Latest trust</div>
                                <div className="mt-2 grid gap-2">
                                    <KV
                                        k={latestTrust?.robotId ?? 'Robot'}
                                        v={
                                            <span className="flex items-center gap-2">
                                                <Chip tone={latestTrust?.gate === 'OK' ? 'good' : latestTrust?.gate === 'Warning' ? 'warn' : 'bad'}>
                                                    {latestTrust?.gate ?? '-'}
                                                </Chip>
                                                <span className="font-mono">{(latestTrust?.confidence ?? 0).toFixed(2)}</span>
                                            </span>
                                        }
                                    />
                                </div>
                                <Divider />
                                <div className="h-[160px] min-h-[160px] min-w-0">
                                    <ResponsiveContainer width="100%" height="100%" minHeight={120} minWidth={160}>
                                        <LineChart data={trustSeries}>
                                            <CartesianGrid strokeDasharray="3 3" />
                                            <XAxis dataKey="t" tick={{ fontSize: 12 }} />
                                            <YAxis domain={[0, 1]} tick={{ fontSize: 12 }} />
                                            <Tooltip />
                                            <Line type="monotone" dataKey="trust" name="Trust" dot={false} strokeWidth={2} />
                                        </LineChart>
                                    </ResponsiveContainer>
                                </div>
                            </div>
                        </div>
                    </CardBody>
                </Card>

                {/* Model Registry */}
                <Card>
                    <CardHeader
                        title={
                            <span className="flex items-center gap-2">
                                <Database className="h-4 w-4" />
                                Model Registry
                            </span>
                        }
                        subtitle="Artifacts, trust score, Go/No-Go"
                    />
                    <CardBody>
                        <div className="overflow-auto rounded-xl border border-slate-200 dark:border-slate-800">
                            <table className="w-full text-sm">
                                <thead className="bg-slate-50 text-left text-xs text-slate-600 dark:bg-slate-900/40 dark:text-slate-400">
                                    <tr>
                                        <th className="px-3 py-2">Model</th>
                                        <th className="px-3 py-2">Trust</th>
                                        <th className="px-3 py-2">Gate</th>
                                        <th className="px-3 py-2">Created</th>
                                        <th className="px-3 py-2">Artifact</th>
                                    </tr>
                                </thead>
                                <tbody className="divide-y divide-slate-200 dark:divide-slate-800">
                                    {models.map((m) => (
                                        <tr key={m.id} className="bg-white dark:bg-slate-950">
                                            <td className="px-3 py-2 font-mono text-xs">{m.id}</td>
                                            <td className="px-3 py-2 font-mono text-xs">{m.trustScore.toFixed(2)}</td>
                                            <td className="px-3 py-2">
                                                <Chip tone={m.goNoGo === 'Go' ? 'good' : 'bad'}>{m.goNoGo}</Chip>
                                            </td>
                                            <td className="px-3 py-2 text-xs text-slate-600 dark:text-slate-400">
                                                {new Date(m.createdAt).toLocaleString()}
                                            </td>
                                            <td className="px-3 py-2 font-mono text-xs text-slate-600 dark:text-slate-400">
                                                {m.artifactPath}
                                            </td>
                                        </tr>
                                    ))}
                                </tbody>
                            </table>
                        </div>
                    </CardBody>
                </Card>
            </div>

            {/* Right column: Audit log */}
            <div className="col-span-12 xl:col-span-5 space-y-4">
                <Card>
                    <CardHeader
                        title={
                            <span className="flex items-center gap-2">
                                <History className="h-4 w-4" />
                                Audit Log
                            </span>
                        }
                        subtitle="Operator actions + system events"
                        right={<Chip tone="ghost">{audit.length} entries</Chip>}
                    />
                    <CardBody className="space-y-2">
                        {lastAudit.map((a) => (
                            <div
                                key={a.id}
                                className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950"
                            >
                                <div className="flex items-start justify-between gap-3">
                                    <div className="min-w-0">
                                        <div className="flex flex-wrap items-center gap-2">
                                            <Chip
                                                tone={
                                                    a.severity === 'Info'
                                                        ? 'neutral'
                                                        : a.severity === 'Warn'
                                                            ? 'warn'
                                                            : 'bad'
                                                }
                                            >
                                                {a.severity}
                                            </Chip>
                                            <span className="text-xs text-slate-600 dark:text-slate-400">
                                                {new Date(a.at).toLocaleString()}
                                            </span>
                                            <Chip tone="ghost">{a.actor}</Chip>
                                        </div>
                                        <div className="mt-2 text-sm font-medium">{a.action}</div>
                                        {a.context ? (
                                            <div className="mt-2 grid grid-cols-2 gap-2 text-xs text-slate-600 dark:text-slate-400">
                                                {Object.entries(a.context).slice(0, 6).map(([k, v]) => (
                                                    <div key={k} className="flex items-center justify-between gap-2">
                                                        <span className="opacity-70">{k}</span>
                                                        <span className="font-mono">{v}</span>
                                                    </div>
                                                ))}
                                            </div>
                                        ) : null}
                                    </div>
                                </div>
                            </div>
                        ))}
                    </CardBody>
                </Card>
            </div>
        </div>
    );
}
