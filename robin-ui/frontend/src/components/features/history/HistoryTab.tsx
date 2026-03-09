import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Chip } from '../../ui/Chip';
import { Button } from '../../ui/Button';
import type { RunSummary, AuditLogEntry } from '../../../types';
import { domainTerms } from '../../../config/domain';

interface HistoryTabProps {
    summaries: RunSummary[];
    audit: AuditLogEntry[];
}

export function HistoryTab({ summaries, audit }: HistoryTabProps) {
    return (
        <div className="grid grid-cols-12 gap-4">
            <div className="col-span-12 xl:col-span-7 space-y-4">
                <Card>
                    <CardHeader
                        title="Run History"
                        subtitle="Summaries (mocked)"
                        right={<Chip tone="ghost">{summaries.length} runs</Chip>}
                    />
                    <CardBody>
                        <div className="overflow-auto rounded-xl border border-slate-200 dark:border-slate-800">
                            <table className="w-full text-sm">
                                <thead className="bg-slate-50 text-left text-xs text-slate-600 dark:bg-slate-900/40 dark:text-slate-400">
                                    <tr>
                                        <th className="px-3 py-2">Run</th>
                                        <th className="px-3 py-2">Started</th>
                                        <th className="px-3 py-2">Dur (min)</th>
                                        <th className="px-3 py-2">{domainTerms.segmentPlural} A</th>
                                        <th className="px-3 py-2">{domainTerms.segmentPlural} B</th>
                                        <th className="px-3 py-2">Avg Trust A</th>
                                        <th className="px-3 py-2">Avg Trust B</th>
                                        <th className="px-3 py-2">Notes</th>
                                    </tr>
                                </thead>
                                <tbody className="divide-y divide-slate-200 dark:divide-slate-800">
                                    {summaries.map((s) => (
                                        <tr key={s.runId} className="bg-white dark:bg-slate-950">
                                            <td className="px-3 py-2 font-mono text-xs">{s.runId}</td>
                                            <td className="px-3 py-2 text-xs text-slate-600 dark:text-slate-400">
                                                {new Date(s.startedAt).toLocaleString()}
                                            </td>
                                            <td className="px-3 py-2 font-mono text-xs">{s.durationMin}</td>
                                            <td className="px-3 py-2 font-mono text-xs">{s.segmentsCompletedA}</td>
                                            <td className="px-3 py-2 font-mono text-xs">{s.segmentsCompletedB}</td>
                                            <td className="px-3 py-2 font-mono text-xs">{s.avgTrustA.toFixed(2)}</td>
                                            <td className="px-3 py-2 font-mono text-xs">{s.avgTrustB.toFixed(2)}</td>
                                            <td className="px-3 py-2 text-xs text-slate-600 dark:text-slate-400">
                                                {s.notes ?? "-"}
                                            </td>
                                        </tr>
                                    ))}
                                </tbody>
                            </table>
                        </div>

                        <div className="mt-3 flex flex-wrap items-center gap-2">
                            <Button size="sm" variant="secondary" onClick={() => { }}>
                                Export CSV
                            </Button>
                            <Button size="sm" variant="secondary" onClick={() => { }}>
                                Export PDF
                            </Button>
                            <Button size="sm" onClick={() => { }}>
                                Open run details
                            </Button>
                        </div>
                    </CardBody>
                </Card>
            </div>

            <div className="col-span-12 xl:col-span-5 space-y-4">
                <Card>
                    <CardHeader
                        title="Audit (Full)"
                        subtitle="Most recent entries"
                        right={<Chip tone="ghost">{audit.length} total</Chip>}
                    />
                    <CardBody className="space-y-2">
                        {audit.slice(0, 12).map((a) => (
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
                                        {new Date(a.at).toLocaleString()}
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
