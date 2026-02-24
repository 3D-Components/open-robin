// FAROS Domain Types

export type ConnStatus = "green" | "yellow" | "red";
export type RobotState = "Idle" | "Running" | "Paused" | "Fault" | "E-Stop";
export type PositionKey = "PA" | "PC";
export type VizMode = "execution" | "deposition";
export type TrustGate = "OK" | "Warning" | "Stop";
export type TabKey = "live" | "robots" | "models" | "mlops" | "inference" | "history" | "settings";

export type RobotCell = {
    id: "robotA" | "robotB";
    name: string;
    position: PositionKey;
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
    robotA: { tasks: ProcessTask[] };
    robotB: { tasks: ProcessTask[] };
};

export type MeasurementPoint = {
    t: number;
    speedA: number;
    speedB: number;
    currentA: number;
    currentB: number;
    voltageA: number;
    voltageB: number;
    profileHeightA: number;
    profileHeightB: number;
    profileWidthA: number;
    profileWidthB: number;
    confidenceA: number;
    confidenceB: number;
};

export type ModelVersion = {
    id: string;
    position: PositionKey;
    createdAt: string;
    artifactPath: string;
    trustScore: number;
    goNoGo: "Go" | "No-Go";
    notes?: string;
};

export type TrustAssessment = {
    atT: number;
    robotId: "robotA" | "robotB";
    confidence: number;
    gate: TrustGate;
    reasons: string[];
};

export type PipelineRun = {
    id: string;
    position: PositionKey;
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
    segmentsCompletedA: number;
    segmentsCompletedB: number;
    avgTrustA: number;
    avgTrustB: number;
    notes?: string;
};

export type Alert = {
    id: string;
    at: string;
    severity: "Info" | "Warning" | "Critical";
    message: string;
    source: string;
};

// Operation modes matching the ROBIN alert engine
export type OperationMode = "parameter_driven" | "geometry_driven";

// Process control state (replaces Wirecloud process-controls widget)
export type ProcessControlsState = {
    mode: OperationMode;
    tolerance: number;
    wireSpeed: number;
    current: number;
    voltage: number;
    targetHeight: number;
    targetWidth: number;
};

// Deviation check response from /check-deviation
export type DeviationResponse = {
    status: string;
    deviation_percentage?: number;
    expected_value?: { height: number; width: number };
    measured_value?: { height: number; width: number };
    deviation_breakdown?: { height: number; width: number };
    message?: string;
};

// Deviation alert action (matches wirecloud action buttons)
export type DeviationAction =
    | "manual_adjust"
    | "new_ai_recommendation"
    | "add_data_finetune"
    | "start_new_doe";

// Latest measurement snapshot for KPI display
export type MeasurementSnapshot = {
    height: number | null;
    width: number | null;
    speed: number | null;
    current: number | null;
    voltage: number | null;
    timestamp: string | null;
};

// Target geometry
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
export type RobotFilter = "A" | "B" | "Both";
