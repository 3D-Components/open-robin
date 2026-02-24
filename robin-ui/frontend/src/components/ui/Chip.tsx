import React from 'react';
import { cx } from '../../utils/helpers';

export function Chip({
    tone,
    children,
    className,
}: {
    tone: "neutral" | "good" | "warn" | "bad" | "info" | "ghost";
    children: React.ReactNode;
    className?: string;
}) {
    const base =
        "inline-flex items-center gap-1 rounded-full border px-2 py-0.5 text-xs font-medium";
    const style = {
        neutral:
            "border-slate-300/50 bg-slate-100 text-slate-800 dark:border-slate-700 dark:bg-slate-900/40 dark:text-slate-200",
        good: "border-emerald-300/60 bg-emerald-50 text-emerald-800 dark:border-emerald-700/70 dark:bg-emerald-900/30 dark:text-emerald-200",
        warn: "border-amber-300/60 bg-amber-50 text-amber-800 dark:border-amber-700/70 dark:bg-amber-900/25 dark:text-amber-200",
        bad: "border-rose-300/60 bg-rose-50 text-rose-800 dark:border-rose-700/70 dark:bg-rose-900/25 dark:text-rose-200",
        info: "border-cyan-300/60 bg-cyan-50 text-cyan-800 dark:border-cyan-700/70 dark:bg-cyan-900/25 dark:text-cyan-200",
        ghost:
            "border-slate-300/30 bg-transparent text-slate-700 dark:border-slate-700/50 dark:text-slate-200",
    }[tone];

    return <span className={cx(base, style, className)}>{children}</span>;
}
