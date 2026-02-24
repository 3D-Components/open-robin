import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Toggle } from '../../ui/Toggle';
import { Select } from '../../ui/Select';
import { Button } from '../../ui/Button';
import { KV, Divider } from '../../ui/ProgressBar';
import type { ConnStatus } from '../../../types';

interface SettingsTabProps {
    dark: boolean;
    setDark: (v: boolean) => void;
    connections: Record<string, ConnStatus>;
    setConnections: (v: any) => void;
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
    setConnections,
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

    const statusOptions: Array<{ value: ConnStatus; label: string }> = [
        { value: "green", label: "green" },
        { value: "yellow", label: "yellow" },
        { value: "red", label: "red" },
    ];

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
                        subtitle="Mock connection state controls"
                    />
                    <CardBody className="space-y-3">
                        <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                            <div className="text-xs text-slate-600 dark:text-slate-400">
                                These are prototype controls to simulate link health.
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
                                                Set link state (green/yellow/red)
                                            </div>
                                        </div>
                                        <Select
                                            value={connections[k]}
                                            onChange={(v) =>
                                                setConnections((c: any) => ({ ...c, [k]: v as ConnStatus }))
                                            }
                                            options={statusOptions.map((o) => ({ value: o.value, label: o.label }))}
                                            className="w-40"
                                        />
                                    </div>
                                </div>
                            ))}
                        </div>

                        <div className="flex flex-wrap items-center gap-2">
                            <Button
                                size="sm"
                                variant="secondary"
                                onClick={() =>
                                    setConnections((c: any) => {
                                        const out: any = {};
                                        for (const k of Object.keys(c)) out[k] = "green";
                                        return out;
                                    })
                                }
                            >
                                Set all green
                            </Button>
                            <Button
                                size="sm"
                                variant="secondary"
                                onClick={() =>
                                    setConnections((c: any) => {
                                        const out: any = {};
                                        for (const k of Object.keys(c)) out[k] = Math.random() < 0.6 ? "green" : Math.random() < 0.75 ? "yellow" : "red";
                                        return out;
                                    })
                                }
                            >
                                Randomize
                            </Button>
                        </div>
                    </CardBody>
                </Card>

                <Card>
                    <CardHeader title="About" subtitle="Prototype metadata" />
                    <CardBody className="space-y-2">
                        <KV k="Build" v="FAROS Cockpit Prototype v0.1" />
                        <KV k="Data" v="Simulated streaming @ 1 Hz" />
                        <KV k="Purpose" v="UX draft for RobTrack desktop integration" />
                    </CardBody>
                </Card>
            </div>
        </div>
    );
}
