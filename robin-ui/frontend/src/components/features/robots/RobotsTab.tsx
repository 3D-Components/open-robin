import { Bot, Play, Pause } from 'lucide-react';
import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Chip } from '../../ui/Chip';
import { Button } from '../../ui/Button';
import { KV } from '../../ui/ProgressBar';
import { gateForConfidence } from '../../../utils/helpers';
import type { RobotCell, ProcessRun, MeasurementPoint } from '../../../types';
import { domainTerms } from '../../../config/domain';

interface RobotsTabProps {
    robots: Record<RobotCell["id"], RobotCell>;
    currentRun: ProcessRun | null;
    telemetry: MeasurementPoint[];
    trustWarnTh: number;
    trustStopTh: number;
    startRobot: (id: RobotCell["id"]) => void;
    pauseRobot: (id: RobotCell["id"]) => void;
    resumeRobot: (id: RobotCell["id"]) => void;
    abortRobot: (id: RobotCell["id"]) => void;
    toggleParamFreeze: (id: RobotCell["id"]) => void;
}

export function RobotsTab({
    robots,
    currentRun,
    telemetry,
    trustWarnTh,
    trustStopTh,
    startRobot,
    pauseRobot,
    resumeRobot,
    abortRobot,
    toggleParamFreeze,
}: RobotsTabProps) {
    const last = telemetry[telemetry.length - 1];

    function robotTelemetry(robot: "A" | "B") {
        if (!last) return null;
        const suffix = robot === "A" ? "A" : "B";
        const get = (k: keyof MeasurementPoint) => (last as any)[k];
        return {
            speed: get(`speed${suffix}` as any) as number,
            current: get(`current${suffix}` as any) as number,
            voltage: get(`voltage${suffix}` as any) as number,
            height: get(`profileHeight${suffix}` as any) as number,
            width: get(`profileWidth${suffix}` as any) as number,
            conf: get(`confidence${suffix}` as any) as number,
        };
    }

    const tA = robotTelemetry("A");
    const tB = robotTelemetry("B");

    const gate = (conf?: number) => gateForConfidence(conf ?? 1, trustWarnTh, trustStopTh);

    return (
        <div className="space-y-4">
            <Card>
                <CardHeader
                    title="Robots"
                    subtitle={currentRun ? `Current run: ${currentRun.id} (${currentRun.mode})` : "No active run"}
                    right={<Chip tone="ghost">2 cells</Chip>}
                />
                <CardBody className="grid grid-cols-1 gap-3 lg:grid-cols-2">
                    <Card className="overflow-hidden">
                        <CardHeader
                            title={
                                <span className="flex items-center gap-2">
                                    <Bot className="h-4 w-4" /> {robots.robotA.name} <Chip tone="info">PA</Chip>
                                </span>
                            }
                            subtitle="Flat position cell"
                            right={<Chip tone={gate(tA?.conf) === "OK" ? "good" : gate(tA?.conf) === "Warning" ? "warn" : "bad"}>{gate(tA?.conf)}</Chip>}
                        />
                        <CardBody className="space-y-3">
                            <div className="grid grid-cols-2 gap-2">
                                <div className="rounded-lg border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                    <KV k="State" v={<Chip tone={robots.robotA.state === "Running" ? "good" : robots.robotA.state === "Paused" ? "warn" : "neutral"}>{robots.robotA.state}</Chip>} />
                                    <KV k="Active model" v={robots.robotA.activeModel} />
                                </div>
                                <div className="rounded-lg border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                    <KV k={domainTerms.segment} v={`#${robots.robotA.segmentIndex}`} mono />
                                    <KV k="Confidence" v={<span className="font-mono">{(tA?.conf ?? 0).toFixed(2)}</span>} />
                                </div>
                            </div>

                            <div className="rounded-lg border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                <div className="grid grid-cols-2 gap-2">
                                    <KV k="Speed" v={<span className="font-mono">{(tA?.speed ?? 0).toFixed(2)}</span>} />
                                    <KV k="Current" v={<span className="font-mono">{(tA?.current ?? 0).toFixed(1)}</span>} />
                                    <KV k="Voltage" v={<span className="font-mono">{(tA?.voltage ?? 0).toFixed(2)}</span>} />
                                    <KV k={`${domainTerms.geometry} H/W`} v={<span className="font-mono">{(tA?.height ?? 0).toFixed(2)} / {(tA?.width ?? 0).toFixed(2)}</span>} />
                                </div>
                            </div>

                            <div className="flex flex-wrap items-center gap-2">
                                <Button size="sm" onClick={() => startRobot("robotA")}>
                                    <Play className="h-4 w-4" /> Start
                                </Button>
                                <Button size="sm" variant="secondary" onClick={() => pauseRobot("robotA")}>
                                    <Pause className="h-4 w-4" /> Pause
                                </Button>
                                <Button size="sm" onClick={() => resumeRobot("robotA")}>
                                    <Play className="h-4 w-4" /> Resume
                                </Button>
                                <Button size="sm" variant={robots.robotA.isParamFrozen ? "primary" : "secondary"} onClick={() => toggleParamFreeze("robotA")}>
                                    Parameter Freeze
                                </Button>
                                <Button size="sm" variant="danger" onClick={() => abortRobot("robotA")}>
                                    Abort
                                </Button>
                            </div>
                        </CardBody>
                    </Card>

                    <Card className="overflow-hidden">
                        <CardHeader
                            title={
                                <span className="flex items-center gap-2">
                                    <Bot className="h-4 w-4" /> {robots.robotB.name} <Chip tone="info">PC</Chip>
                                </span>
                            }
                            subtitle="Different position cell"
                            right={<Chip tone={gate(tB?.conf) === "OK" ? "good" : gate(tB?.conf) === "Warning" ? "warn" : "bad"}>{gate(tB?.conf)}</Chip>}
                        />
                        <CardBody className="space-y-3">
                            <div className="grid grid-cols-2 gap-2">
                                <div className="rounded-lg border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                    <KV k="State" v={<Chip tone={robots.robotB.state === "Running" ? "good" : robots.robotB.state === "Paused" ? "warn" : "neutral"}>{robots.robotB.state}</Chip>} />
                                    <KV k="Active model" v={robots.robotB.activeModel} />
                                </div>
                                <div className="rounded-lg border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                    <KV k={domainTerms.segment} v={`#${robots.robotB.segmentIndex}`} mono />
                                    <KV k="Confidence" v={<span className="font-mono">{(tB?.conf ?? 0).toFixed(2)}</span>} />
                                </div>
                            </div>

                            <div className="rounded-lg border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                <div className="grid grid-cols-2 gap-2">
                                    <KV k="Speed" v={<span className="font-mono">{(tB?.speed ?? 0).toFixed(2)}</span>} />
                                    <KV k="Current" v={<span className="font-mono">{(tB?.current ?? 0).toFixed(1)}</span>} />
                                    <KV k="Voltage" v={<span className="font-mono">{(tB?.voltage ?? 0).toFixed(2)}</span>} />
                                    <KV k={`${domainTerms.geometry} H/W`} v={<span className="font-mono">{(tB?.height ?? 0).toFixed(2)} / {(tB?.width ?? 0).toFixed(2)}</span>} />
                                </div>
                            </div>

                            <div className="flex flex-wrap items-center gap-2">
                                <Button size="sm" onClick={() => startRobot("robotB")}>
                                    <Play className="h-4 w-4" /> Start
                                </Button>
                                <Button size="sm" variant="secondary" onClick={() => pauseRobot("robotB")}>
                                    <Pause className="h-4 w-4" /> Pause
                                </Button>
                                <Button size="sm" onClick={() => resumeRobot("robotB")}>
                                    <Play className="h-4 w-4" /> Resume
                                </Button>
                                <Button size="sm" variant={robots.robotB.isParamFrozen ? "primary" : "secondary"} onClick={() => toggleParamFreeze("robotB")}>
                                    Parameter Freeze
                                </Button>
                                <Button size="sm" variant="danger" onClick={() => abortRobot("robotB")}>
                                    Abort
                                </Button>
                            </div>
                        </CardBody>
                    </Card>
                </CardBody>
            </Card>
        </div>
    );
}
