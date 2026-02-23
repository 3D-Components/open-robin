import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Toggle } from '../../ui/Toggle';
import { Chip } from '../../ui/Chip';
import { KV, Divider } from '../../ui/ProgressBar';
import type { ConnStatus } from '../../../types';

interface SettingsTabProps {
    dark: boolean;
    setDark: (v: boolean) => void;
    connections: Record<string, ConnStatus>;
    trustWarnTh: number;
    trustStopTh: number;
    setTrustWarnTh: (v: number) => void;
    setTrustStopTh: (v: number) => void;
    freezeCharts: boolean;
    setFreezeCharts: (v: boolean) => void;
    replay: boolean;
    setReplay: (v: boolean) => void;
}

export function SettingsTab({
    dark,
    setDark,
    connections,
    trustWarnTh,
    trustStopTh,
    setTrustWarnTh,
    setTrustStopTh,
    freezeCharts,
    setFreezeCharts,
    replay,
    setReplay,
}: SettingsTabProps) {
    const keys = Object.keys(connections);
    const toneForStatus = (status: ConnStatus): 'good' | 'warn' | 'bad' =>
        status === 'green' ? 'good' : status === 'yellow' ? 'warn' : 'bad';

    return (
        <div className="grid grid-cols-12 gap-4">
            <div className="col-span-12 xl:col-span-6 space-y-4">
                <Card>
                    <CardHeader
                        title="Appearance"
                        subtitle="Prototype UI settings"
                    />
                    <CardBody className="space-y-3">
                        <Toggle
                            checked={dark}
                            onChange={setDark}
                            label="Dark mode"
                            hint="Shop-floor friendly theme"
                        />
                        <Toggle
                            checked={freezeCharts}
                            onChange={setFreezeCharts}
                            label="Freeze charts"
                            hint="Pause telemetry streaming"
                        />
                        <Toggle
                            checked={replay}
                            onChange={setReplay}
                            label="Replay timeline"
                            hint="Auto-scrub visualization time slider"
                        />
                    </CardBody>
                </Card>

                <Card>
                    <CardHeader
                        title="Trust thresholds"
                        subtitle="Controls runtime gating behavior (prototype)"
                    />
                    <CardBody className="space-y-3">
                        <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                            <KV k="Warning threshold" v={<span className="font-mono">{trustWarnTh.toFixed(2)}</span>} />
                            <input
                                type="range"
                                min={0.3}
                                max={0.95}
                                step={0.01}
                                value={trustWarnTh}
                                onChange={(e) => setTrustWarnTh(Number(e.target.value))}
                                className="mt-2 w-full accent-slate-900 dark:accent-slate-100"
                            />
                            <Divider />
                            <KV k="Stop threshold" v={<span className="font-mono">{trustStopTh.toFixed(2)}</span>} />
                            <input
                                type="range"
                                min={0.1}
                                max={0.9}
                                step={0.01}
                                value={trustStopTh}
                                onChange={(e) => setTrustStopTh(Number(e.target.value))}
                                className="mt-2 w-full accent-slate-900 dark:accent-slate-100"
                            />
                            <div className="mt-2 text-xs text-slate-600 dark:text-slate-400">
                                Tip: keep stop below warn. In production we will enforce this.
                            </div>
                        </div>
                    </CardBody>
                </Card>
            </div>

            <div className="col-span-12 xl:col-span-6 space-y-4">
                <Card>
                    <CardHeader
                        title="Connections"
                        subtitle="Live backend health (read-only)"
                    />
                    <CardBody className="space-y-3">
                        <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                            <div className="text-xs text-slate-600 dark:text-slate-400">
                                Connection state is driven by API health probes in real time.
                            </div>
                        </div>

                        <div className="grid gap-3">
                            {keys.map((k) => (
                                <div
                                    key={k}
                                    className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950"
                                >
                                    <div className="flex items-center justify-between gap-3">
                                        <div className="min-w-0">
                                            <div className="text-sm font-semibold">{k}</div>
                                            <div className="text-xs text-slate-600 dark:text-slate-400">
                                                Derived from live health checks
                                            </div>
                                        </div>
                                        <Chip tone={toneForStatus(connections[k])}>
                                            {connections[k].toUpperCase()}
                                        </Chip>
                                    </div>
                                </div>
                            ))}
                        </div>
                    </CardBody>
                </Card>

                <Card>
                    <CardHeader title="About" subtitle="Runtime metadata" />
                    <CardBody className="space-y-2">
                        <KV k="Build" v="ROBIN Dashboard" />
                        <KV k="Data source" v="Live API + FIWARE health probes" />
                        <KV k="Purpose" v="Operator monitoring and process control" />
                    </CardBody>
                </Card>
            </div>
        </div>
    );
}
