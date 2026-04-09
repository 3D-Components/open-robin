import { useMemo } from 'react';
import { Gauge, ArrowUpDown, SlidersHorizontal } from 'lucide-react';
import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Chip } from '../../ui/Chip';
import {
    resolveRecordedAIInputParams,
} from '../../../config/aiInputFeatures';
import type { RobinMeasurement } from '../../../hooks/useRobinAPI';
import type { AIInputFeatureSpec } from '../../../types';

interface MeasurementKPIsProps {
    processId: string | null;
    measurements: RobinMeasurement[] | null;
    count: number;
    aiInputFeatures: AIInputFeatureSpec[];
}

export function MeasurementKPIs({
    processId,
    measurements,
    count,
    aiInputFeatures,
}: MeasurementKPIsProps) {
    const latest = useMemo(() => {
        if (!measurements?.length) return null;
        return measurements[measurements.length - 1];
    }, [measurements]);
    const latestInputParams = useMemo(
        () =>
            latest
                ? resolveRecordedAIInputParams(
                    latest as unknown as Record<string, unknown>,
                    aiInputFeatures,
                )
                : {},
        [aiInputFeatures, latest],
    );

    const lastUpdate = latest?.timestamp
        ? new Date(latest.timestamp).toLocaleTimeString()
        : '-';

    return (
        <Card>
            <CardHeader
                title={
                    <span className="flex items-center gap-2">
                        <Gauge className="h-4 w-4" />
                        Measurement KPIs
                    </span>
                }
                subtitle={processId ? `Process: ${processId}` : 'No active process'}
                right={
                    <div className="flex items-center gap-2">
                        <Chip tone="ghost">{count} pts</Chip>
                        <Chip tone="ghost">{lastUpdate}</Chip>
                    </div>
                }
            />
            <CardBody>
                <div className="grid grid-cols-5 gap-2">
                    <KpiCard
                        label="Height"
                        unit="mm"
                        decimals={2}
                        icon={<ArrowUpDown className="h-4 w-4" />}
                        value={latest?.height}
                    />
                    <KpiCard
                        label="Width"
                        unit="mm"
                        decimals={2}
                        icon={<ArrowUpDown className="h-4 w-4 rotate-90" />}
                        value={latest?.width}
                    />
                    {aiInputFeatures.slice(0, 3).map((feature) => (
                        <KpiCard
                            key={feature.key}
                            label={feature.label}
                            unit={feature.unit}
                            decimals={feature.step && feature.step < 1 ? 3 : 2}
                            icon={<SlidersHorizontal className="h-4 w-4" />}
                            value={latestInputParams[feature.key]}
                        />
                    ))}
                </div>
            </CardBody>
        </Card>
    );
}

function KpiCard({
    label,
    unit,
    decimals,
    icon,
    value,
}: {
    label: string;
    unit: string;
    decimals: number;
    icon: React.ReactNode;
    value: number | undefined;
}) {
    const hasValue = value !== undefined && value !== null && !isNaN(value);
    return (
        <div className="rounded-xl border border-slate-200 bg-white p-3 text-center dark:border-slate-800 dark:bg-slate-950">
            <div className="flex items-center justify-center gap-1 text-xs text-slate-500">
                {icon}
                {label}
            </div>
            <div
                className={`mt-1 text-lg font-mono font-bold ${
                    hasValue
                        ? 'text-slate-900 dark:text-slate-100'
                        : 'text-slate-400'
                }`}
            >
                {hasValue ? value!.toFixed(decimals) : '-'}
            </div>
            <div className="text-[11px] text-slate-500">{unit || '—'}</div>
        </div>
    );
}
