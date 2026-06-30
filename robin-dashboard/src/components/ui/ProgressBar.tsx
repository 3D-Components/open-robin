import React from 'react';
import { cx } from '../../utils/helpers';

export function ProgressBar({
    value,
    tone = "neutral",
}: {
    value: number;
    tone?: "neutral" | "good" | "warn" | "bad";
}) {
    const cl = {
        neutral: "bg-slate-900 dark:bg-slate-100",
        good: "bg-emerald-600",
        warn: "bg-amber-600",
        bad: "bg-rose-600",
    }[tone];

    return (
        <div className="h-2 w-full rounded-full bg-slate-100 dark:bg-slate-900">
            <div
                className={cx("h-2 rounded-full", cl)}
                style={{ width: `${Math.max(0, Math.min(100, value))}%` }}
            />
        </div>
    );
}

export function KV({
    k,
    v,
    mono,
}: {
    k: string;
    v: React.ReactNode;
    mono?: boolean;
}) {
    return (
        <div className="flex items-center justify-between gap-3 min-w-0">
            <div className="shrink-0 text-xs text-slate-600 dark:text-slate-400">{k}</div>
            <div
                className={cx(
                    "text-xs text-slate-900 dark:text-slate-100 truncate",
                    mono && "font-mono"
                )}
                title={typeof v === "string" ? v : undefined}
            >
                {v}
            </div>
        </div>
    );
}

export function Divider() {
    return <div className="my-3 h-px bg-slate-200 dark:bg-slate-800" />;
}
