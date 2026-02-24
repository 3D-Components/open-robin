import React from 'react';
import { cx } from '../../utils/helpers';

export function Button({
    children,
    variant = "primary",
    size = "md",
    className,
    disabled,
    onClick,
    title,
}: {
    children: React.ReactNode;
    variant?: "primary" | "secondary" | "ghost" | "danger";
    size?: "sm" | "md" | "lg";
    className?: string;
    disabled?: boolean;
    onClick?: () => void;
    title?: string;
}) {
    const base =
        "inline-flex select-none items-center justify-center gap-2 rounded-md border font-medium transition active:translate-y-[1px] disabled:cursor-not-allowed disabled:opacity-50";
    const sizes = {
        sm: "h-8 px-3 text-sm",
        md: "h-9 px-3 text-sm",
        lg: "h-10 px-4 text-sm",
    }[size];

    const variants = {
        primary:
            "border-slate-900 bg-slate-900 text-white hover:bg-slate-800 dark:border-slate-100 dark:bg-slate-100 dark:text-slate-950 dark:hover:bg-slate-200",
        secondary:
            "border-slate-300 bg-white text-slate-900 hover:bg-slate-50 dark:border-slate-700 dark:bg-slate-950 dark:text-slate-100 dark:hover:bg-slate-900",
        ghost:
            "border-transparent bg-transparent text-slate-800 hover:bg-slate-100 dark:text-slate-100 dark:hover:bg-slate-900",
        danger:
            "border-rose-700/60 bg-rose-600 text-white hover:bg-rose-700 dark:border-rose-500/60 dark:bg-rose-600 dark:text-white dark:hover:bg-rose-700",
    }[variant];

    return (
        <button
            title={title}
            className={cx(base, sizes, variants, className)}
            disabled={disabled}
            onClick={onClick}
        >
            {children}
        </button>
    );
}
