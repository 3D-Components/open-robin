import { Bot, Play, Pause } from 'lucide-react';
import { Card, CardHeader, CardBody } from '../../ui/Card';
import { Chip } from '../../ui/Chip';
import { Button } from '../../ui/Button';
import { KV } from '../../ui/ProgressBar';
import { gateForConfidence } from '../../../utils/helpers';
import type { RobotCell, ProcessRun, MeasurementPoint } from '../../../types';
import { domainTerms } from '../../../config/domain';

interface RobotsTabProps {
    robot: RobotCell;
    currentRun: ProcessRun | null;
    telemetry: MeasurementPoint[];
    trustWarnTh: number;
    trustStopTh: number;
    startRobot: () => void;
    pauseRobot: () => void;
    resumeRobot: () => void;
    abortRobot: () => void;
    toggleParamFreeze: () => void;
}

export function RobotsTab({
    robot,
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

    const t = last
        ? {
              speed: last.speed,
              current: last.current,
              voltage: last.voltage,
              height: last.profileHeight,
              width: last.profileWidth,
              conf: last.confidence,
          }
        : null;

    const gate = (conf?: number) => gateForConfidence(conf ?? 1, trustWarnTh, trustStopTh);

    return (
        <div className="space-y-4">
            <Card>
                <CardHeader
                    title="Robot"
                    subtitle={currentRun ? `Current run: ${currentRun.id} (${currentRun.mode})` : "No active run"}
                />
                <CardBody>
                    <Card className="overflow-hidden">
                        <CardHeader
                            title={
                                <span className="flex items-center gap-2">
                                    <Bot className="h-4 w-4" /> {robot.name}
                                </span>
                            }
                            right={<Chip tone={gate(t?.conf) === "OK" ? "good" : gate(t?.conf) === "Warning" ? "warn" : "bad"}>{gate(t?.conf)}</Chip>}
                        />
                        <CardBody className="space-y-3">
                            <div className="grid grid-cols-2 gap-2">
                                <div className="rounded-lg border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                    <KV k="State" v={<Chip tone={robot.state === "Running" ? "good" : robot.state === "Paused" ? "warn" : "neutral"}>{robot.state}</Chip>} />
                                    <KV k="Active model" v={robot.activeModel} />
                                </div>
                                <div className="rounded-lg border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                    <KV k={domainTerms.segment} v={`#${robot.segmentIndex}`} mono />
                                    <KV k="Confidence" v={<span className="font-mono">{(t?.conf ?? 0).toFixed(2)}</span>} />
                                </div>
                            </div>

                            <div className="rounded-lg border border-slate-200 bg-white p-3 dark:border-slate-800 dark:bg-slate-950">
                                <div className="grid grid-cols-2 gap-2">
                                    <KV k="Speed" v={<span className="font-mono">{(t?.speed ?? 0).toFixed(2)}</span>} />
                                    <KV k="Current" v={<span className="font-mono">{(t?.current ?? 0).toFixed(1)}</span>} />
                                    <KV k="Voltage" v={<span className="font-mono">{(t?.voltage ?? 0).toFixed(2)}</span>} />
                                    <KV k={`${domainTerms.geometry} H/W`} v={<span className="font-mono">{(t?.height ?? 0).toFixed(2)} / {(t?.width ?? 0).toFixed(2)}</span>} />
                                </div>
                            </div>

                            <div className="flex flex-wrap items-center gap-2">
                                <Button size="sm" onClick={startRobot}>
                                    <Play className="h-4 w-4" /> Start
                                </Button>
                                <Button size="sm" variant="secondary" onClick={pauseRobot}>
                                    <Pause className="h-4 w-4" /> Pause
                                </Button>
                                <Button size="sm" onClick={resumeRobot}>
                                    <Play className="h-4 w-4" /> Resume
                                </Button>
                                <Button size="sm" variant={robot.isParamFrozen ? "primary" : "secondary"} onClick={toggleParamFreeze}>
                                    Parameter Freeze
                                </Button>
                                <Button size="sm" variant="danger" onClick={abortRobot}>
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
