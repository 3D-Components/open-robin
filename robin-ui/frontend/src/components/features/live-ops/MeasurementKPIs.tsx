import { useMemo } from 'react';
import { Gauge, ArrowUpDown, Zap, Activity } from 'lucide-react';
import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Chip } from '../../ui/Chip';
import type { RobinMeasurement } from '../../../hooks/useRobinAPI';

interface MeasurementKPIsProps {
    processId: string | null;
    measurements: RobinMeasurement[] | null;
    count: number;
}

interface KPIConfig {
    label: string;
    unit: string;
    decimals: number;
    icon: React.ReactNode;
    extract: (m: RobinMeasurement) => number | undefined;
    warnRange?: [number, number];
    alertRange?: [number, number];
}

const KPI_CONFIGS: KPIConfig[] = [
    {
        label: 'Height',
        unit: 'mm',
        decimals: 2,
        icon: <ArrowUpDown className="h-4 w-4" />,
        extract: (m) => m.height,
    },
    {
        label: 'Width',
        unit: 'mm',
        decimals: 2,
        icon: <ArrowUpDown className="h-4 w-4 rotate-90" />,
        extract: (m) => m.width,
    },
    {
        label: 'Speed',
        unit: 'mm/s',
        decimals: 1,
        icon: <Activity className="h-4 w-4" />,
        extract: (m) => m.speed,
    },
    {
        label: 'Current',
        unit: 'A',
        decimals: 1,
        icon: <Zap className="h-4 w-4" />,
        extract: (m) => m.current,
    },
    {
        label: 'Voltage',
        unit: 'V',
        decimals: 1,
        icon: <Zap className="h-4 w-4" />,
        extract: (m) => m.voltage,
    },
];

export function MeasurementKPIs({ processId, measurements, count }: MeasurementKPIsProps) {
    const latest = useMemo(() => {
        if (!measurements?.length) return null;
        return measurements[measurements.length - 1];
    }, [measurements]);

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
                    {KPI_CONFIGS.map((kpi) => {
                        const value = latest ? kpi.extract(latest) : undefined;
                        const hasValue = value !== undefined && value !== null && !isNaN(value);

                        return (
                            <div
                                key={kpi.label}
                                className="rounded-xl border border-slate-200 bg-white p-3 text-center dark:border-slate-800 dark:bg-slate-950"
                            >
                                <div className="flex items-center justify-center gap-1 text-xs text-slate-500">
                                    {kpi.icon}
                                    {kpi.label}
                                </div>
                                <div
                                    className={`mt-1 text-lg font-mono font-bold ${
                                        hasValue
                                            ? 'text-slate-900 dark:text-slate-100'
                                            : 'text-slate-400'
                                    }`}
                                >
                                    {hasValue ? value!.toFixed(kpi.decimals) : '-'}
                                </div>
                                <div className="text-[11px] text-slate-500">{kpi.unit}</div>
                            </div>
                        );
                    })}
                </div>
            </CardBody>
        </Card>
    );
}
