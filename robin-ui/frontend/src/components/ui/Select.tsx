import { ChevronDown } from 'lucide-react';
import { cx } from '../../utils/helpers';

export function Select({
    value,
    options,
    onChange,
    className,
}: {
    value: string;
    options: Array<{ value: string; label: string }>;
    onChange: (v: string) => void;
    className?: string;
}) {
    return (
        <div className={cx("relative", className)}>
            <select
                className="h-9 w-full appearance-none rounded-md border border-slate-300 bg-white px-3 pr-9 text-sm text-slate-900 hover:bg-slate-50 dark:border-slate-700 dark:bg-slate-950 dark:text-slate-100 dark:hover:bg-slate-900"
                value={value}
                onChange={(e) => onChange(e.target.value)}
            >
                {options.map((o) => (
                    <option key={o.value} value={o.value}>
                        {o.label}
                    </option>
                ))}
            </select>
            <ChevronDown className="pointer-events-none absolute right-2 top-2.5 h-4 w-4 text-slate-500" />
        </div>
    );
}
