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
    vizLatencyMs: number;
    vizLatencyTone: "good" | "warn" | "bad";
    serveLatencyMs: number;
    serveLatencyTone: "good" | "warn" | "bad";
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
    vizLatencyTone,
    serveLatencyTone,
    processId,
    availableProcessIds,
    onProcessIdChange,
}: TopBarProps) {
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
                    <Chip tone={vizLatencyTone}>
                        <Clock className="h-3.5 w-3.5" />
                        <span>API</span>
                    </Chip>
                    <Chip tone={serveLatencyTone}>
                        <Cpu className="h-3.5 w-3.5" />
                        <span>Backend</span>
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
