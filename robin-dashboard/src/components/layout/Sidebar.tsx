import React from 'react';
import { Activity, Bot, Shield, History, Settings, ChevronRight } from 'lucide-react';
import { Chip } from '../ui/Chip';
import { cx } from '../../utils/helpers';
import type { TabKey } from '../../types';

interface SidebarProps {
    tab: TabKey;
    setTab: (t: TabKey) => void;
}

export function Sidebar({ tab, setTab }: SidebarProps) {
    const items: Array<{ key: TabKey; label: string; icon: React.ReactNode; hint: string }> = [
        { key: "live", label: "Live Ops", icon: <Activity className="h-4 w-4" />, hint: "Cockpit" },
        { key: "robots", label: "Robot", icon: <Bot className="h-4 w-4" />, hint: "Robot status" },
        { key: "models", label: "Models & Trust", icon: <Shield className="h-4 w-4" />, hint: "AI models" },
        { key: "history", label: "History / Reports", icon: <History className="h-4 w-4" />, hint: "Runs & exports" },
        { key: "settings", label: "Settings", icon: <Settings className="h-4 w-4" />, hint: "Thresholds" },
    ];

    return (
        <aside className="w-[260px] shrink-0 border-r border-slate-200 bg-white/70 p-3 backdrop-blur dark:border-slate-800 dark:bg-slate-950/40">
            <div className="mb-3 rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                <div className="flex items-center justify-between">
                    <div className="text-xs font-semibold text-slate-900 dark:text-slate-100">
                        ROBIN Navigation
                    </div>
                    <Chip tone="ghost">v0.1</Chip>
                </div>
                <div className="mt-2 text-xs text-slate-600 dark:text-slate-400">
                    Role-based cockpit for process monitoring and AI models.
                </div>
            </div>

            <nav className="space-y-1">
                {items.map((it) => (
                    <button
                        key={it.key}
                        onClick={() => setTab(it.key)}
                        className={cx(
                            "flex w-full items-center justify-between gap-2 rounded-lg border px-3 py-2 text-left transition",
                            tab === it.key
                                ? "border-slate-900 bg-slate-900 text-white dark:border-slate-100 dark:bg-slate-100 dark:text-slate-950"
                                : "border-slate-200 bg-white text-slate-900 hover:bg-slate-50 dark:border-slate-800 dark:bg-slate-950 dark:text-slate-100 dark:hover:bg-slate-900"
                        )}
                    >
                        <div className="flex items-center gap-2">
                            {it.icon}
                            <div>
                                <div className="text-sm font-medium leading-tight">{it.label}</div>
                                <div
                                    className={cx(
                                        "text-xs",
                                        tab === it.key
                                            ? "text-white/80 dark:text-slate-700"
                                            : "text-slate-600 dark:text-slate-400"
                                    )}
                                >
                                    {it.hint}
                                </div>
                            </div>
                        </div>
                        <ChevronRight className={cx("h-4 w-4", tab === it.key ? "opacity-90" : "opacity-40")} />
                    </button>
                ))}
            </nav>

            <div className="mt-3 rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                <div className="text-xs font-semibold">Quick Tips</div>
                <ul className="mt-2 space-y-1 text-xs text-slate-600 dark:text-slate-400">
                    <li>• Freeze charts to inspect anomalies.</li>
                    <li>• Trust gate STOP will auto-pause (demo behavior).</li>
                    <li>• Switching active model writes an audit entry.</li>
                </ul>
            </div>
        </aside>
    );
}
