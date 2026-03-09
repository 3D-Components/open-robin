import { useState, useEffect } from 'react';
import { Settings2, Zap, Target, AlertTriangle } from 'lucide-react';
import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Chip } from '../../ui/Chip';
import { Button } from '../../ui/Button';
import { Select } from '../../ui/Select';
import { Divider } from '../../ui/ProgressBar';
import type { OperationMode, ProcessControlsState, TargetGeometry } from '../../../types';
import { domainTerms } from '../../../config/domain';

interface ProcessControlsPanelProps {
    processId: string | null;
    controls: ProcessControlsState;
    onControlsChange: (next: ProcessControlsState) => void;
    onApply: (controls: ProcessControlsState) => void;
    targetGeometry: TargetGeometry | null;
    processMode: OperationMode | null;
}

export function ProcessControlsPanel({
    processId,
    controls,
    onControlsChange,
    onApply,
    targetGeometry,
    processMode,
}: ProcessControlsPanelProps) {
    const isParameterMode = controls.mode === 'parameter_driven';
    const [dirty, setDirty] = useState(false);

    useEffect(() => {
        if (processMode && processMode !== controls.mode) {
            onControlsChange({ ...controls, mode: processMode });
        }
    }, [processMode]);

    function update(partial: Partial<ProcessControlsState>) {
        onControlsChange({ ...controls, ...partial });
        setDirty(true);
    }

    function handleApply() {
        onApply(controls);
        setDirty(false);
    }

    return (
        <Card>
            <CardHeader
                title={
                    <span className="flex items-center gap-2">
                        <Settings2 className="h-4 w-4" />
                        Process Controls
                    </span>
                }
                subtitle={processId ? `Process: ${processId}` : 'No active process'}
                right={
                    dirty ? (
                        <Chip tone="warn">Unsaved</Chip>
                    ) : (
                        <Chip tone="good">Applied</Chip>
                    )
                }
            />
            <CardBody className="space-y-3">
                {/* Mode selector */}
                <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                    <div className="flex items-center justify-between">
                        <div className="text-sm font-semibold">Control Mode</div>
                        <Chip tone="info">{isParameterMode ? 'Parameter' : 'Geometry'}</Chip>
                    </div>
                    <div className="mt-2">
                        <Select
                            value={controls.mode}
                            onChange={(v) => update({ mode: v as OperationMode })}
                            options={[
                                { value: 'parameter_driven', label: 'Parameter-driven' },
                                { value: 'geometry_driven', label: 'Geometry-driven' },
                            ]}
                        />
                    </div>
                    <div className="mt-2 text-xs text-slate-600 dark:text-slate-400">
                        {isParameterMode
                            ? 'Tune process parameters to guide the process.'
                            : 'Specify geometry targets to drive corrections.'}
                    </div>
                </div>

                {/* Parameter targets */}
                <div
                    className={`rounded-xl border p-3 transition-opacity ${
                        isParameterMode
                            ? 'border-slate-200 bg-white dark:border-slate-800 dark:bg-slate-950'
                            : 'border-blue-200 bg-blue-50/50 dark:border-blue-800/50 dark:bg-blue-950/20'
                    }`}
                >
                    <div className="flex items-center justify-between">
                        <div className="flex items-center gap-2">
                            <Zap className="h-4 w-4" />
                            <span className="text-sm font-semibold">
                                Parameter Targets
                                {!isParameterMode && (
                                    <span className="ml-1 text-xs font-normal text-blue-600 dark:text-blue-400">
                                        (AI suggestion)
                                    </span>
                                )}
                            </span>
                        </div>
                    </div>

                    <div className="mt-2 grid gap-2">
                        <label className="flex items-center justify-between gap-2">
                            <span className="text-xs text-slate-600 dark:text-slate-400 w-24">{domainTerms.speed}</span>
                            <div className="flex items-center gap-1">
                                <input
                                    type="number"
                                    step="0.1"
                                    value={controls.wireSpeed}
                                    onChange={(e) => update({ wireSpeed: parseFloat(e.target.value) || 0 })}
                                    readOnly={!isParameterMode}
                                    className="w-24 rounded-md border border-slate-200 bg-white px-2 py-1 text-right text-xs font-mono dark:border-slate-700 dark:bg-slate-900 read-only:opacity-60"
                                />
                                <span className="text-xs text-slate-500 w-10">{domainTerms.speedUnit}</span>
                            </div>
                        </label>
                        <label className="flex items-center justify-between gap-2">
                            <span className="text-xs text-slate-600 dark:text-slate-400 w-24">{domainTerms.current}</span>
                            <div className="flex items-center gap-1">
                                <input
                                    type="number"
                                    step="1"
                                    value={controls.current}
                                    onChange={(e) => update({ current: parseFloat(e.target.value) || 0 })}
                                    readOnly={!isParameterMode}
                                    className="w-24 rounded-md border border-slate-200 bg-white px-2 py-1 text-right text-xs font-mono dark:border-slate-700 dark:bg-slate-900 read-only:opacity-60"
                                />
                                <span className="text-xs text-slate-500 w-10">{domainTerms.currentUnit}</span>
                            </div>
                        </label>
                        <label className="flex items-center justify-between gap-2">
                            <span className="text-xs text-slate-600 dark:text-slate-400 w-24">{domainTerms.voltage}</span>
                            <div className="flex items-center gap-1">
                                <input
                                    type="number"
                                    step="0.1"
                                    value={controls.voltage}
                                    onChange={(e) => update({ voltage: parseFloat(e.target.value) || 0 })}
                                    readOnly={!isParameterMode}
                                    className="w-24 rounded-md border border-slate-200 bg-white px-2 py-1 text-right text-xs font-mono dark:border-slate-700 dark:bg-slate-900 read-only:opacity-60"
                                />
                                <span className="text-xs text-slate-500 w-10">{domainTerms.voltageUnit}</span>
                            </div>
                        </label>
                    </div>

                    {!isParameterMode && (
                        <div className="mt-2 flex items-start gap-1 text-xs text-blue-600 dark:text-blue-400">
                            <AlertTriangle className="h-3 w-3 mt-0.5 shrink-0" />
                            Read-only: AI-suggested values based on geometry target.
                        </div>
                    )}
                </div>

                {/* Geometry targets */}
                <div
                    className={`rounded-xl border p-3 transition-opacity ${
                        !isParameterMode
                            ? 'border-slate-200 bg-white dark:border-slate-800 dark:bg-slate-950'
                            : 'border-slate-200 bg-slate-50 opacity-60 dark:border-slate-800 dark:bg-slate-900/30'
                    }`}
                >
                    <div className="flex items-center justify-between">
                        <div className="flex items-center gap-2">
                            <Target className="h-4 w-4" />
                            <span className="text-sm font-semibold">Geometry Targets</span>
                        </div>
                        {targetGeometry && (
                            <Chip tone="ghost">
                                Current: {targetGeometry.height.toFixed(2)} x {targetGeometry.width.toFixed(2)} mm
                            </Chip>
                        )}
                    </div>

                    <div className="mt-2 grid gap-2">
                        <label className="flex items-center justify-between gap-2">
                            <span className="text-xs text-slate-600 dark:text-slate-400 w-24">
                                {domainTerms.profileHeight}
                            </span>
                            <div className="flex items-center gap-1">
                                <input
                                    type="number"
                                    step="0.01"
                                    value={controls.targetHeight}
                                    onChange={(e) => update({ targetHeight: parseFloat(e.target.value) || 0 })}
                                    disabled={isParameterMode}
                                    className="w-24 rounded-md border border-slate-200 bg-white px-2 py-1 text-right text-xs font-mono dark:border-slate-700 dark:bg-slate-900 disabled:opacity-40"
                                />
                                <span className="text-xs text-slate-500 w-10">mm</span>
                            </div>
                        </label>
                        <label className="flex items-center justify-between gap-2">
                            <span className="text-xs text-slate-600 dark:text-slate-400 w-24">
                                {domainTerms.profileWidth}
                            </span>
                            <div className="flex items-center gap-1">
                                <input
                                    type="number"
                                    step="0.01"
                                    value={controls.targetWidth}
                                    onChange={(e) => update({ targetWidth: parseFloat(e.target.value) || 0 })}
                                    disabled={isParameterMode}
                                    className="w-24 rounded-md border border-slate-200 bg-white px-2 py-1 text-right text-xs font-mono dark:border-slate-700 dark:bg-slate-900 disabled:opacity-40"
                                />
                                <span className="text-xs text-slate-500 w-10">mm</span>
                            </div>
                        </label>
                    </div>

                    {isParameterMode && (
                        <div className="mt-2 text-xs text-slate-500">
                            Geometry targets are disabled in parameter-driven mode.
                        </div>
                    )}
                </div>

                {/* Tolerance */}
                <div className="rounded-xl border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                    <div className="flex items-center justify-between">
                        <span className="text-sm font-semibold">Quality Tolerance</span>
                        <span className="font-mono text-sm">{controls.tolerance.toFixed(0)}%</span>
                    </div>
                    <input
                        type="range"
                        min={1}
                        max={50}
                        step={1}
                        value={controls.tolerance}
                        onChange={(e) => update({ tolerance: Number(e.target.value) })}
                        className="mt-2 w-full accent-slate-900 dark:accent-slate-100"
                    />
                    <div className="mt-1 flex items-center justify-between text-[11px] text-slate-500">
                        <span>1%</span>
                        <span>Deviation threshold</span>
                        <span>50%</span>
                    </div>
                </div>

                <Divider />

                {/* Apply button */}
                <Button
                    className="w-full justify-center"
                    onClick={handleApply}
                    disabled={!processId}
                >
                    Apply Settings
                </Button>
            </CardBody>
        </Card>
    );
}
