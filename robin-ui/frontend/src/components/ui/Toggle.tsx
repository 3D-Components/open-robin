import { cx } from '../../utils/helpers';

export function Toggle({
    checked,
    onChange,
    label,
    hint,
}: {
    checked: boolean;
    onChange: (v: boolean) => void;
    label: string;
    hint?: string;
}) {
    return (
        <button
            className="flex w-full items-center justify-between gap-3 rounded-lg border border-slate-200 bg-white px-3 py-2 text-left hover:bg-slate-50 dark:border-slate-800 dark:bg-slate-950 dark:hover:bg-slate-900"
            onClick={() => onChange(!checked)}
        >
            <div>
                <div className="text-sm font-medium text-slate-900 dark:text-slate-100">
                    {label}
                </div>
                {hint ? (
                    <div className="text-xs text-slate-600 dark:text-slate-400">
                        {hint}
                    </div>
                ) : null}
            </div>
            <div
                className={cx(
                    "h-6 w-10 rounded-full border p-0.5 transition",
                    checked
                        ? "border-slate-900 bg-slate-900 dark:border-slate-100 dark:bg-slate-100"
                        : "border-slate-300 bg-slate-200 dark:border-slate-700 dark:bg-slate-800"
                )}
            >
                <div
                    className={cx(
                        "h-5 w-5 rounded-full bg-white transition dark:bg-slate-950",
                        checked ? "translate-x-4" : "translate-x-0"
                    )}
                />
            </div>
        </button>
    );
}
