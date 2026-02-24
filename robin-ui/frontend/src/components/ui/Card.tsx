import React from 'react';
import { cx } from '../../utils/helpers';

export function Card({
    children,
    className,
}: {
    children: React.ReactNode;
    className?: string;
}) {
    return (
        <div
            className={cx(
                "rounded-xl border border-slate-200 bg-white shadow-sm dark:border-slate-800 dark:bg-slate-950",
                className
            )}
        >
            {children}
        </div>
    );
}

export function CardHeader({
    title,
    subtitle,
    right,
}: {
    title: React.ReactNode;
    subtitle?: React.ReactNode;
    right?: React.ReactNode;
}) {
    return (
        <div className="flex items-start justify-between gap-3 border-b border-slate-200 px-4 py-3 dark:border-slate-800">
            <div className="min-w-0">
                <div className="flex items-center gap-2 text-sm font-semibold text-slate-900 dark:text-slate-100">
                    {title}
                </div>
                {subtitle ? (
                    <div className="mt-0.5 text-xs text-slate-600 dark:text-slate-400">
                        {subtitle}
                    </div>
                ) : null}
            </div>
            {right ? <div className="shrink-0">{right}</div> : null}
        </div>
    );
}

export function CardBody({
    children,
    className,
}: {
    children: React.ReactNode;
    className?: string;
}) {
    return <div className={cx("p-4", className)}>{children}</div>;
}
