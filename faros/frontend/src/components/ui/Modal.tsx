import React from 'react';
import { AlertTriangle } from 'lucide-react';
import { Button } from './Button';

export function Modal({
    open,
    title,
    children,
    confirmText = "Confirm",
    confirmVariant = "primary",
    onConfirm,
    onClose,
    dangerHint,
}: {
    open: boolean;
    title: string;
    children: React.ReactNode;
    confirmText?: string;
    confirmVariant?: "primary" | "secondary" | "danger";
    onConfirm: () => void;
    onClose: () => void;
    dangerHint?: string;
}) {
    if (!open) return null;
    return (
        <div className="fixed inset-0 z-50 flex items-center justify-center">
            <div
                className="absolute inset-0 bg-black/40"
                onClick={onClose}
            />
            <div className="relative w-[min(560px,92vw)] rounded-2xl border border-slate-200 bg-white shadow-xl dark:border-slate-800 dark:bg-slate-950">
                <div className="flex items-start justify-between gap-3 border-b border-slate-200 px-5 py-4 dark:border-slate-800">
                    <div>
                        <div className="text-sm font-semibold text-slate-900 dark:text-slate-100">
                            {title}
                        </div>
                        {dangerHint ? (
                            <div className="mt-1 flex items-center gap-2 text-xs text-rose-700 dark:text-rose-300">
                                <AlertTriangle className="h-4 w-4" />
                                <span>{dangerHint}</span>
                            </div>
                        ) : null}
                    </div>
                    <Button variant="ghost" onClick={onClose}>
                        Close
                    </Button>
                </div>
                <div className="px-5 py-4">{children}</div>
                <div className="flex items-center justify-end gap-2 border-t border-slate-200 px-5 py-4 dark:border-slate-800">
                    <Button variant="secondary" onClick={onClose}>
                        Cancel
                    </Button>
                    <Button variant={confirmVariant} onClick={onConfirm}>
                        {confirmText}
                    </Button>
                </div>
            </div>
        </div>
    );
}
