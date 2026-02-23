import React from 'react';
import { Bot, Network, Clock, Cpu } from 'lucide-react';
import { Button } from '../ui/Button';
import { Chip } from '../ui/Chip';
import { Select } from '../ui/Select';
import type { ConnStatus } from '../../types';
import { domainTerms } from '../../config/domain';

interface TopBarProps {
    dark: boolean;
    setDark: (v: boolean) => void;
    sessionMode: "Active Run" | "Demo Mode";
    setSessionMode: (v: "Active Run" | "Demo Mode") => void;
    connections: Record<string, ConnStatus>;
    connBadge: (s: ConnStatus) => React.ReactNode;
    orionLatencyMs: number | null;
    orionLatencyTone: "good" | "warn" | "bad";
    mintakaLatencyMs: number | null;
    mintakaLatencyTone: "good" | "warn" | "bad";
    uptimeHrs: number;
    // ROBIN process selector
    processId?: string | null;
    availableProcessIds?: string[];
    onProcessIdChange?: (id: string) => void;
}

export function TopBar({
    dark,
    setDark,
    sessionMode,
    setSessionMode,
    connections,
    connBadge,
    orionLatencyMs,
    orionLatencyTone,
    mintakaLatencyMs,
    mintakaLatencyTone,
    uptimeHrs,
    processId,
    availableProcessIds,
    onProcessIdChange,
}: TopBarProps) {
    const fmtLatency = (ms: number | null) => (ms === null || Number.isNaN(ms) ? "n/a" : `${ms} ms`);
    const fmtUptime = (hours: number) => {
        if (!Number.isFinite(hours) || hours < 0) return 'n/a';
        const totalSeconds = Math.round(hours * 3600);
        if (totalSeconds < 60) return `${totalSeconds}s`;
        if (totalSeconds < 3600) {
            const minutes = Math.floor(totalSeconds / 60);
            const seconds = totalSeconds % 60;
            return `${minutes}m ${seconds}s`;
        }
        if (totalSeconds < 86400) return `${(totalSeconds / 3600).toFixed(1)}h`;
        const days = Math.floor(totalSeconds / 86400);
        const remHours = Math.floor((totalSeconds % 86400) / 3600);
        return `${days}d ${remHours}h`;
    };

    return (
        <div className="h-14 border-b border-slate-200 bg-white/80 backdrop-blur dark:border-slate-800 dark:bg-slate-950/70">
            <div className="flex h-full items-center justify-between gap-3 px-4">
                <div className="flex items-center gap-3">
                    <div className="flex items-center gap-2">
                        <div className="grid h-9 w-9 place-items-center rounded-lg border border-slate-200 bg-slate-50 dark:border-slate-800 dark:bg-slate-900/50">
                            <Bot className="h-5 w-5" />
                        </div>
                        <div>
                            <div className="text-sm font-semibold tracking-tight">ROBIN</div>
                            <div className="text-xs text-slate-600 dark:text-slate-400">
                                {domainTerms.process} Monitor
                            </div>
                        </div>
                    </div>

                    {/* Process ID selector - replaces Wirecloud's Static Process ID operator */}
                    {availableProcessIds && availableProcessIds.length > 0 && onProcessIdChange && (
                        <div className="hidden items-center gap-2 md:flex">
                            <span className="text-xs text-slate-600 dark:text-slate-400">
                                {domainTerms.process}
                            </span>
                            <Select
                                value={processId ?? ''}
                                onChange={(v) => onProcessIdChange(v)}
                                options={availableProcessIds.map((id) => ({
                                    value: id,
                                    label: id,
                                }))}
                                className="w-44"
                            />
                        </div>
                    )}

                    <div className="hidden items-center gap-2 md:flex">
                        <span className="text-xs text-slate-600 dark:text-slate-400">Session</span>
                        <Select
                            value={sessionMode}
                            onChange={(v) => setSessionMode(v as any)}
                            options={[
                                { value: "Active Run", label: "Active Run" },
                                { value: "Demo Mode", label: "Demo Mode" },
                            ]}
                            className="w-40"
                        />
                        <Chip tone={sessionMode === 'Active Run' ? 'good' : 'warn'}>
                            {sessionMode === 'Active Run' ? 'Writes Enabled' : 'Simulation Only'}
                        </Chip>
                    </div>

                    <div className="hidden items-center gap-2 lg:flex">
                        <Chip tone="ghost">
                            <Network className="h-3.5 w-3.5" />
                            <span className="text-xs">Connections</span>
                        </Chip>
                        {Object.entries(connections).map(([k, v]) => (
                            <div key={k} className="flex items-center gap-2">
                                <span className="text-xs text-slate-600 dark:text-slate-400">{k}</span>
                                {connBadge(v)}
                            </div>
                        ))}
                    </div>
                </div>

                <div className="flex items-center gap-2">
                    <Chip tone={orionLatencyTone}>
                        <Clock className="h-3.5 w-3.5" />
                        <span>Orion {fmtLatency(orionLatencyMs)}</span>
                    </Chip>
                    <Chip tone={mintakaLatencyTone}>
                        <Cpu className="h-3.5 w-3.5" />
                        <span>Mintaka {fmtLatency(mintakaLatencyMs)}</span>
                    </Chip>
                    <Chip tone="ghost">
                        <span>Uptime {fmtUptime(uptimeHrs)}</span>
                    </Chip>

                    <Button
                        variant="ghost"
                        onClick={() => setDark(!dark)}
                        title="Toggle light/dark"
                    >
                        {dark ? "Dark" : "Light"}
                    </Button>
                </div>
            </div>
        </div>
    );
}
