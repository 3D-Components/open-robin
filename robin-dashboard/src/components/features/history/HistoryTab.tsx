import { useCallback, useEffect, useMemo, useState } from 'react';
import { Download, RefreshCw, AlertTriangle, Database } from 'lucide-react';
import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Chip } from '../../ui/Chip';
import { Button } from '../../ui/Button';
import { Select } from '../../ui/Select';
import {
    fetchProcessAlerts,
    fetchProcessMeasurements,
    type RobinAlertRecord,
    type RobinMeasurement,
} from '../../../hooks/useRobinAPI';

interface HistoryTabProps {
    processId: string | null;
    availableProcessIds: string[];
    onProcessIdChange: (id: string) => void;
    processTolerance?: number | null;
}

type HistoryCsvRow = {
    record_type: 'measurement' | 'warning';
    process_id: string;
    timestamp: string;
    alert_id: string;
    measurement_height_mm: string;
    measurement_width_mm: string;
    measurement_speed_mm_s: string;
    measurement_current_a: string;
    measurement_voltage_v: string;
    alert_severity: string;
    alert_deviation_type: string;
    alert_deviation_percentage: string;
    alert_expected_height_mm: string;
    alert_expected_width_mm: string;
    alert_measured_height_mm: string;
    alert_measured_width_mm: string;
    alert_recommended_actions: string;
    raw_payload_json: string;
    source: 'process_measurements' | 'process_alerts';
};

function toNum(v: unknown): number | null {
    return typeof v === 'number' && Number.isFinite(v) ? v : null;
}

function csvEscape(value: string): string {
    if (/[",\n]/.test(value)) {
        return `"${value.replace(/"/g, '""')}"`;
    }
    return value;
}

function actionLabel(value: string | { id?: string; label?: string }): string {
    if (typeof value === 'string') return value;
    if (value && typeof value === 'object') {
        if (typeof value.label === 'string' && value.label.trim()) return value.label;
        if (typeof value.id === 'string' && value.id.trim()) return value.id;
    }
    return '';
}

export function HistoryTab({
    processId,
    availableProcessIds,
    onProcessIdChange,
    processTolerance,
}: HistoryTabProps) {
    const [measurements, setMeasurements] = useState<RobinMeasurement[]>([]);
    const [alerts, setAlerts] = useState<RobinAlertRecord[]>([]);
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState<string | null>(null);
    const [loadedAt, setLoadedAt] = useState<string | null>(null);

    const loadHistory = useCallback(async () => {
        if (!processId) {
            setMeasurements([]);
            setAlerts([]);
            setError('Select a process to load history.');
            return;
        }

        setLoading(true);
        setError(null);
        try {
            const [mRes, aRes] = await Promise.all([
                fetchProcessMeasurements(processId),
                fetchProcessAlerts(processId),
            ]);

            if (mRes.status !== 'success') {
                throw new Error('Failed to fetch raw measurements');
            }
            if (aRes.status !== 'success') {
                throw new Error('Failed to fetch warning alerts');
            }

            setMeasurements(mRes.measurements ?? []);
            setAlerts(aRes.alerts ?? []);
            setLoadedAt(new Date().toISOString());
        } catch (e: unknown) {
            setMeasurements([]);
            setAlerts([]);
            setError(e instanceof Error ? e.message : 'Failed to load history');
        } finally {
            setLoading(false);
        }
    }, [processId]);

    useEffect(() => {
        void loadHistory();
    }, [loadHistory]);

    const classifySeverity = useCallback((alert: RobinAlertRecord): 'Warning' | 'Critical' => {
        const deviation = toNum(alert.deviation_percentage);
        const tolerance = toNum(processTolerance);
        if (deviation === null || tolerance === null) return 'Warning';
        return deviation >= tolerance * 1.5 ? 'Critical' : 'Warning';
    }, [processTolerance]);

    const csvRows = useMemo<HistoryCsvRow[]>(() => {
        if (!processId) return [];

        const measurementRows: HistoryCsvRow[] = measurements.map((m) => ({
            record_type: 'measurement',
            process_id: processId,
            timestamp: m.timestamp ?? '',
            alert_id: '',
            measurement_height_mm: toNum(m.height)?.toString() ?? '',
            measurement_width_mm: toNum(m.width)?.toString() ?? '',
            measurement_speed_mm_s: toNum(m.speed)?.toString() ?? '',
            measurement_current_a: toNum(m.current)?.toString() ?? '',
            measurement_voltage_v: toNum(m.voltage)?.toString() ?? '',
            alert_severity: '',
            alert_deviation_type: '',
            alert_deviation_percentage: '',
            alert_expected_height_mm: '',
            alert_expected_width_mm: '',
            alert_measured_height_mm: '',
            alert_measured_width_mm: '',
            alert_recommended_actions: '',
            raw_payload_json: JSON.stringify(m),
            source: 'process_measurements',
        }));

        const alertRows: HistoryCsvRow[] = alerts.map((a) => ({
            record_type: 'warning',
            process_id: processId,
            timestamp: a.timestamp ?? '',
            alert_id: a.id ?? '',
            measurement_height_mm: '',
            measurement_width_mm: '',
            measurement_speed_mm_s: '',
            measurement_current_a: '',
            measurement_voltage_v: '',
            alert_severity: classifySeverity(a),
            alert_deviation_type: a.deviation_type ?? '',
            alert_deviation_percentage: toNum(a.deviation_percentage)?.toString() ?? '',
            alert_expected_height_mm: toNum(a.expected_value?.height)?.toString() ?? '',
            alert_expected_width_mm: toNum(a.expected_value?.width)?.toString() ?? '',
            alert_measured_height_mm: toNum(a.measured_value?.height)?.toString() ?? '',
            alert_measured_width_mm: toNum(a.measured_value?.width)?.toString() ?? '',
            alert_recommended_actions: Array.isArray(a.recommended_actions)
                ? a.recommended_actions.map(actionLabel).filter(Boolean).join('|')
                : '',
            raw_payload_json: JSON.stringify(a),
            source: 'process_alerts',
        }));

        const rows = [...measurementRows, ...alertRows];
        rows.sort((a, b) => {
            const ta = Date.parse(a.timestamp);
            const tb = Date.parse(b.timestamp);
            if (!Number.isFinite(ta) && !Number.isFinite(tb)) return 0;
            if (!Number.isFinite(ta)) return 1;
            if (!Number.isFinite(tb)) return -1;
            return ta - tb;
        });
        return rows;
    }, [alerts, classifySeverity, measurements, processId]);

    const downloadCsv = useCallback(() => {
        if (!processId || csvRows.length === 0) return;

        const headers: Array<keyof HistoryCsvRow> = [
            'record_type',
            'process_id',
            'timestamp',
            'alert_id',
            'measurement_height_mm',
            'measurement_width_mm',
            'measurement_speed_mm_s',
            'measurement_current_a',
            'measurement_voltage_v',
            'alert_severity',
            'alert_deviation_type',
            'alert_deviation_percentage',
            'alert_expected_height_mm',
            'alert_expected_width_mm',
            'alert_measured_height_mm',
            'alert_measured_width_mm',
            'alert_recommended_actions',
            'raw_payload_json',
            'source',
        ];

        const lines = [
            headers.join(','),
            ...csvRows.map((row) =>
                headers
                    .map((h) => csvEscape(row[h] ?? ''))
                    .join(','),
            ),
        ];
        const csv = lines.join('\n');
        const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        const stamp = new Date().toISOString().replace(/[:.]/g, '-');
        a.download = `${processId}-history-${stamp}.csv`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }, [csvRows, processId]);

    return (
        <div className="grid grid-cols-12 gap-4">
            <div className="col-span-12">
                <Card>
                    <CardHeader
                        title="History Report"
                        subtitle="CSV export includes all raw telemetry and warning events for the selected process"
                        right={
                            <div className="flex flex-wrap items-center gap-2">
                                <Select
                                    value={processId ?? ''}
                                    onChange={(v) => onProcessIdChange(v)}
                                    options={availableProcessIds.map((id) => ({ value: id, label: id }))}
                                    className="w-52"
                                />
                                <Button size="sm" variant="secondary" onClick={() => void loadHistory()} disabled={loading}>
                                    <RefreshCw className={`h-4 w-4 ${loading ? 'animate-spin' : ''}`} />
                                    Refresh
                                </Button>
                                <Button size="sm" onClick={downloadCsv} disabled={!processId || csvRows.length === 0 || loading}>
                                    <Download className="h-4 w-4" />
                                    Download CSV
                                </Button>
                            </div>
                        }
                    />
                    <CardBody className="space-y-3">
                        <div className="flex flex-wrap items-center gap-2">
                            <Chip tone="ghost">Measurements: {measurements.length}</Chip>
                            <Chip tone="warn">Warnings: {alerts.length}</Chip>
                            <Chip tone="ghost">CSV rows: {csvRows.length}</Chip>
                            <Chip tone="ghost">
                                Last load: {loadedAt ? new Date(loadedAt).toLocaleTimeString() : '-'}
                            </Chip>
                        </div>
                        {error ? (
                            <div className="rounded-xl border border-red-300 bg-red-50 px-3 py-2 text-sm text-red-800 dark:border-red-800 dark:bg-red-950/30 dark:text-red-200">
                                {error}
                            </div>
                        ) : null}
                    </CardBody>
                </Card>
            </div>

            <div className="col-span-12 xl:col-span-7">
                <Card>
                    <CardHeader
                        title={
                            <span className="flex items-center gap-2">
                                <Database className="h-4 w-4" />
                                Raw Measurements
                            </span>
                        }
                        subtitle="Stored telemetry samples for selected process"
                        right={<Chip tone="ghost">{measurements.length} rows</Chip>}
                    />
                    <CardBody>
                        <div className="max-h-[420px] overflow-auto rounded-xl border border-slate-200 dark:border-slate-800">
                            <table className="w-full text-sm">
                                <thead className="sticky top-0 bg-slate-50 text-left text-xs text-slate-600 dark:bg-slate-900/90 dark:text-slate-400">
                                    <tr>
                                        <th className="px-3 py-2">Timestamp</th>
                                        <th className="px-3 py-2">Height (mm)</th>
                                        <th className="px-3 py-2">Width (mm)</th>
                                        <th className="px-3 py-2">Speed (mm/s)</th>
                                        <th className="px-3 py-2">Current (A)</th>
                                        <th className="px-3 py-2">Voltage (V)</th>
                                    </tr>
                                </thead>
                                <tbody className="divide-y divide-slate-200 dark:divide-slate-800">
                                    {measurements.map((m, idx) => (
                                        <tr key={`${m.timestamp ?? 'na'}-${idx}`} className="bg-white dark:bg-slate-950">
                                            <td className="px-3 py-2 font-mono text-xs">
                                                {m.timestamp ? new Date(m.timestamp).toLocaleString() : '-'}
                                            </td>
                                            <td className="px-3 py-2 font-mono text-xs">{toNum(m.height)?.toFixed(3) ?? '-'}</td>
                                            <td className="px-3 py-2 font-mono text-xs">{toNum(m.width)?.toFixed(3) ?? '-'}</td>
                                            <td className="px-3 py-2 font-mono text-xs">{toNum(m.speed)?.toFixed(3) ?? '-'}</td>
                                            <td className="px-3 py-2 font-mono text-xs">{toNum(m.current)?.toFixed(3) ?? '-'}</td>
                                            <td className="px-3 py-2 font-mono text-xs">{toNum(m.voltage)?.toFixed(3) ?? '-'}</td>
                                        </tr>
                                    ))}
                                    {measurements.length === 0 ? (
                                        <tr>
                                            <td className="px-3 py-3 text-xs text-slate-500" colSpan={6}>
                                                No measurements for selected process.
                                            </td>
                                        </tr>
                                    ) : null}
                                </tbody>
                            </table>
                        </div>
                    </CardBody>
                </Card>
            </div>

            <div className="col-span-12 xl:col-span-5">
                <Card>
                    <CardHeader
                        title={
                            <span className="flex items-center gap-2">
                                <AlertTriangle className="h-4 w-4" />
                                Warning Events
                            </span>
                        }
                        subtitle="Persisted deviation alerts from backend"
                        right={<Chip tone="warn">{alerts.length} warnings</Chip>}
                    />
                    <CardBody>
                        <div className="max-h-[420px] overflow-auto space-y-2">
                            {alerts.length === 0 ? (
                                <div className="rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-600 dark:border-slate-800 dark:bg-slate-950 dark:text-slate-400">
                                    No warning alerts for selected process.
                                </div>
                            ) : (
                                alerts.map((a, idx) => {
                                    const severity = classifySeverity(a);
                                    return (
                                        <div
                                            key={`${a.id ?? 'alert'}-${idx}`}
                                            className="rounded-xl border border-slate-200 bg-white px-3 py-2 dark:border-slate-800 dark:bg-slate-950"
                                        >
                                            <div className="flex items-center justify-between gap-2">
                                                <Chip tone={severity === 'Critical' ? 'bad' : 'warn'}>{severity}</Chip>
                                                <span className="text-xs text-slate-600 dark:text-slate-400">
                                                    {a.timestamp ? new Date(a.timestamp).toLocaleString() : '-'}
                                                </span>
                                            </div>
                                            <div className="mt-1 text-xs">
                                                Type: <span className="font-mono">{a.deviation_type ?? '-'}</span>
                                            </div>
                                            <div className="mt-1 text-xs">
                                                Deviation: <span className="font-mono">{toNum(a.deviation_percentage)?.toFixed(2) ?? '-'}%</span>
                                            </div>
                                            <div className="mt-1 text-xs text-slate-600 dark:text-slate-400">
                                                Expected H/W: {toNum(a.expected_value?.height)?.toFixed(3) ?? '-'} / {toNum(a.expected_value?.width)?.toFixed(3) ?? '-'} mm
                                            </div>
                                            <div className="mt-1 text-xs text-slate-600 dark:text-slate-400">
                                                Measured H/W: {toNum(a.measured_value?.height)?.toFixed(3) ?? '-'} / {toNum(a.measured_value?.width)?.toFixed(3) ?? '-'} mm
                                            </div>
                                        </div>
                                    );
                                })
                            )}
                        </div>
                    </CardBody>
                </Card>
            </div>
        </div>
    );
}
