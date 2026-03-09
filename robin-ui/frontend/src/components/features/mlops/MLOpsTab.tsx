import { Database, Cpu, History, ChevronRight } from 'lucide-react';
import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Chip } from '../../ui/Chip';
import { Button } from '../../ui/Button';
import { KV, Divider } from '../../ui/ProgressBar';
import type { PipelineRun, ModelVersion, PositionKey, AuditLogEntry } from '../../../types';

const MLOPS_URL = import.meta.env.VITE_MLOPS_URL || 'http://localhost:5173';

interface MLOpsTabProps {
    pipelineRuns: PipelineRun[];
    models: ModelVersion[];
    routing: Record<PositionKey, string>;
    onSwitchModel: (position: PositionKey, next: string) => void;
    audit: AuditLogEntry[];
}

export function MLOpsTab({
    pipelineRuns,
    models,
    routing,
    onSwitchModel: _onSwitchModel,
    audit,
}: MLOpsTabProps) {
    const last = pipelineRuns.slice(0, 6);
    const lastAudit = audit.slice(0, 6);

    const statusTone = (s: PipelineRun["status"]) =>
        s === "Succeeded" ? "good" : s === "Running" ? "warn" : "bad";

    return (
        <div className="grid grid-cols-12 gap-4">
            <div className="col-span-12 xl:col-span-7 space-y-4">
                <Card>
                    <CardHeader
                        title={
                            <span className="flex items-center gap-2">
                                <Database className="h-4 w-4" />
                                MLOps Orchestrator
                            </span>
                        }
                        subtitle="Design-time pipeline factory (DAG execution) + model artifacts"
                        right={<Chip tone="ghost">WebView (placeholder)</Chip>}
                    />
                    <CardBody className="space-y-3">
                        <div className="rounded-xl border border-slate-200 bg-slate-50 p-3 dark:border-slate-800 dark:bg-slate-900/30">
                            <div className="text-xs font-semibold text-slate-900 dark:text-slate-100">
                                Pipeline overview
                            </div>
                            <div className="mt-2 flex flex-wrap items-center gap-2 text-xs">
                                <Chip tone="neutral">Ingest</Chip>
                                <ChevronRight className="h-4 w-4 opacity-40" />
                                <Chip tone="neutral">Train</Chip>
                                <ChevronRight className="h-4 w-4 opacity-40" />
                                <Chip tone="neutral">Validate (Trust)</Chip>
                                <ChevronRight className="h-4 w-4 opacity-40" />
                                <Chip tone="neutral">Deploy / Handoff</Chip>
                            </div>
                            <div className="mt-2 text-xs text-slate-600 dark:text-slate-400">
                                The UI here embeds an existing MLOps web UI in production. This prototype shows placement + summary cards.
                            </div>
                        </div>

                        <div className="h-[360px] overflow-hidden rounded-xl border border-slate-200 bg-white dark:border-slate-800 dark:bg-slate-950">
                            {/* Embedded MLOps Orchestrator (Airflow-like DAG view) */}
                            <iframe
                                src={MLOPS_URL}
                                className="w-full h-full border-0"
                                title="MLOps Orchestrator"
                                sandbox="allow-same-origin allow-scripts allow-forms allow-popups allow-popups-to-escape-sandbox allow-top-navigation-by-user-activation allow-storage-access-by-user-activation"
                                allow="storage-access; camera; microphone; clipboard-write"
                                referrerPolicy="no-referrer-when-downgrade"
                            />
                        </div>

                        <Divider />

                        <div className="grid gap-3 md:grid-cols-2">
                            <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                <div className="text-sm font-semibold">Routing summary</div>
                                <div className="mt-2 space-y-2">
                                    <KV k="PA → Active model" v={<span className="font-mono">{routing.PA}</span>} />
                                    <KV k="PC → Active model" v={<span className="font-mono">{routing.PC}</span>} />
                                </div>
                                <div className="mt-3 text-xs text-slate-600 dark:text-slate-400">
                                    Switching the active model is done from the Models & Trust tab (audit-logged).
                                </div>
                            </div>

                            <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                <div className="text-sm font-semibold">Latest artifacts</div>
                                <div className="mt-2 space-y-2">
                                    {models
                                        .slice()
                                        .sort((a, b) => +new Date(b.createdAt) - +new Date(a.createdAt))
                                        .slice(0, 3)
                                        .map((m) => (
                                            <div key={m.id} className="rounded-lg border border-slate-200 bg-slate-50 p-2 dark:border-slate-800 dark:bg-slate-900/30">
                                                <div className="flex items-center justify-between gap-2">
                                                    <div className="text-xs font-mono">{m.id}</div>
                                                    <Chip tone={m.goNoGo === "Go" ? "good" : "bad"}>{m.goNoGo}</Chip>
                                                </div>
                                                <div className="mt-1 text-[11px] text-slate-600 dark:text-slate-400 font-mono">
                                                    {m.artifactPath}
                                                </div>
                                            </div>
                                        ))}
                                </div>
                            </div>
                        </div>
                    </CardBody>
                </Card>

                <Card>
                    <CardHeader
                        title={
                            <span className="flex items-center gap-2">
                                <Cpu className="h-4 w-4" />
                                Pipeline Runs
                            </span>
                        }
                        subtitle="DAG executions (mock summary)"
                        right={<Chip tone="ghost">{pipelineRuns.length} total</Chip>}
                    />
                    <CardBody className="grid gap-3 md:grid-cols-2">
                        {last.map((p) => (
                            <div
                                key={p.id}
                                className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950"
                            >
                                <div className="flex items-start justify-between gap-3">
                                    <div className="min-w-0">
                                        <div className="flex flex-wrap items-center gap-2">
                                            <Chip tone="info">{p.position}</Chip>
                                            <Chip tone={statusTone(p.status) as any}>{p.status}</Chip>
                                            <Chip tone="ghost" className="font-mono">
                                                {p.id}
                                            </Chip>
                                        </div>
                                        <div className="mt-2 text-xs text-slate-600 dark:text-slate-400">
                                            Started {new Date(p.startedAt).toLocaleString()}
                                            {p.finishedAt ? ` · Finished ${new Date(p.finishedAt).toLocaleTimeString()}` : ""}
                                        </div>
                                    </div>

                                    {p.goNoGo ? (
                                        <Chip tone={p.goNoGo === "Go" ? "good" : "bad"}>{p.goNoGo}</Chip>
                                    ) : (
                                        <Chip tone="ghost">-</Chip>
                                    )}
                                </div>

                                <Divider />

                                <div className="grid gap-2 text-xs">
                                    <KV k="Produced model" v={p.producedModelId ?? "-"} />
                                    <KV
                                        k="Trust score"
                                        v={p.trustScore !== undefined ? <span className="font-mono">{p.trustScore.toFixed(2)}</span> : "-"}
                                    />
                                    <KV k="Artifact path" v={p.artifactPath ?? "-"} mono />
                                </div>

                                <div className="mt-3 flex items-center gap-2">
                                    <Button size="sm" variant="secondary" onClick={() => { }}>
                                        View DAG
                                    </Button>
                                    <Button size="sm" onClick={() => { }}>
                                        Open run
                                    </Button>
                                </div>
                            </div>
                        ))}
                    </CardBody>
                </Card>
            </div>

            <div className="col-span-12 xl:col-span-5 space-y-4">
                <Card>
                    <CardHeader
                        title={
                            <span className="flex items-center gap-2">
                                <History className="h-4 w-4" />
                                Recent Audit
                            </span>
                        }
                        subtitle="Related operator/system actions"
                    />
                    <CardBody className="space-y-2">
                        {lastAudit.map((a) => (
                            <div
                                key={a.id}
                                className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950"
                            >
                                <div className="flex items-center justify-between gap-2">
                                    <Chip
                                        tone={
                                            a.severity === "Info"
                                                ? "neutral"
                                                : a.severity === "Warn"
                                                    ? "warn"
                                                    : "bad"
                                        }
                                    >
                                        {a.severity}
                                    </Chip>
                                    <span className="text-xs text-slate-600 dark:text-slate-400">
                                        {new Date(a.at).toLocaleTimeString()}
                                    </span>
                                </div>
                                <div className="mt-2 text-sm font-medium">{a.action}</div>
                                <div className="mt-1 text-xs text-slate-600 dark:text-slate-400">
                                    Actor: {a.actor}
                                </div>
                            </div>
                        ))}
                    </CardBody>
                </Card>
            </div>
        </div>
    );
}
