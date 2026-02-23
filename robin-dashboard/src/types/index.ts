// ROBIN Dashboard Domain Types

export type ConnStatus = "green" | "yellow" | "red";
export type RobotState = "Idle" | "Running" | "Paused" | "Fault" | "E-Stop";
export type VizMode = "execution" | "deposition";
export type TrustGate = "OK" | "Warning" | "Stop";
export type TabKey = "live" | "robots" | "models" | "history" | "settings";

export type RobotCell = {
    id: string;
    name: string;
    state: RobotState;
    isParamFrozen: boolean;
    currentRunId?: string;
    taskProgressPct: number;
    segmentIndex: number;
    activeModel: string;
    lastTrustScore: number;
    lastDeploymentAt: string;
    lastInferenceAt: string;
};

export type ProcessTask = {
    id: string;
    segmentIndex: number;
    startedAt: string;
    completedAt?: string;
    status: "Queued" | "InProgress" | "Done" | "Aborted";
};

export type ProcessRun = {
    id: string;
    mode: "Active" | "Demo";
    startedAt: string;
    endedAt?: string;
    robot: { tasks: ProcessTask[] };
};

export type MeasurementPoint = {
    t: number;
    timestamp: string | null;
    speed: number;
    current: number;
    voltage: number;
    profileHeight: number;
    profileWidth: number;
    confidence: number;
};

export type ModelVersion = {
    id: string;
    createdAt: string;
    artifactPath: string;
    trustScore: number;
    goNoGo: "Go" | "No-Go";
    notes?: string;
};

export type TrustAssessment = {
    atT: number;
    robotId: string;
    confidence: number;
    gate: TrustGate;
    reasons: string[];
};

export type PipelineRun = {
    id: string;
    status: "Running" | "Succeeded" | "Failed";
    startedAt: string;
    finishedAt?: string;
    producedModelId?: string;
    trustScore?: number;
    goNoGo?: "Go" | "No-Go";
    artifactPath?: string;
};

export type AuditLogEntry = {
    id: string;
    at: string;
    actor: string;
    action: string;
    severity: "Info" | "Warn" | "Critical";
    context?: Record<string, string>;
};

export type RunSummary = {
    runId: string;
    startedAt: string;
    durationMin: number;
    segmentsCompleted: number;
    avgTrust: number;
    notes?: string;
};

export type Alert = {
    id: string;
    at: string;
    severity: "Info" | "Warning" | "Critical";
    message: string;
    source: string;
};

export type OperationMode = "parameter_driven" | "geometry_driven";

export type ProcessControlsState = {
    mode: OperationMode;
    tolerance: number;
    speed: number;
    current: number;
    voltage: number;
    targetHeight: number;
    targetWidth: number;
};

export type DeviationResponse = {
    status: string;
    mode?: string;
    deviation_percentage?: number;
    expected_value?: { height: number; width: number };
    expected_source?: string;
    measured_value?: { height: number; width: number };
    target_geometry?: { height: number; width: number };
    deviation_breakdown?: { height: number; width: number };
    message?: string;
};

export type DeviationAction =
    | "manual_adjust"
    | "new_ai_recommendation"
    | "add_data_finetune"
    | "start_new_doe";

export type MeasurementSnapshot = {
    height: number | null;
    width: number | null;
    speed: number | null;
    current: number | null;
    voltage: number | null;
    timestamp: string | null;
};

export type TargetGeometry = {
    height: number;
    width: number;
};

export type LayerVisibility = {
    robotModel: boolean;
    torchPath: boolean;
    workpiece: boolean;
    profileSegments: boolean;
    frames: boolean;
};

export type CameraView = "Isometric" | "Top" | "Side";
export type MetricType = "speed" | "current" | "voltage" | "profileHeight" | "profileWidth";
