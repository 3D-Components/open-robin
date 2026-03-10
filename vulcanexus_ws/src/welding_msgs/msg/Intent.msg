# Intent message for the welding GUI PoC.
# Mirrors the hri_actions_msgs/msg/Intent structure used in ErgoBot_AI so that
# the same MissionController pattern applies regardless of whether the intent
# originates from voice+LLM or from a GUI button press.

# ── Intent type constants ──────────────────────────────────────────────────
# Maps to ros4hri motion_skills/action/ExecuteJointTrajectory
string MOVE_TO_HOME  = "MOVE_TO_HOME"
# Maps to ros4hri navigation_skills/action/NavigateToWaypoint
string MOVE_TO_ZONE  = "MOVE_TO_ZONE"
# Maps to ros4hri motion_skills/action/ExecuteCartesianTrajectory
string EXECUTE_SEAM  = "EXECUTE_SEAM"
# Cancels all currently active skill goals
string ESTOP         = "ESTOP"

# ── ROBIN dashboard button intents ─────────────────────────────────────────
# "Start" button — starts the welding process (routes to welding_seam_skill)
string START_PROCESS             = "START_PROCESS"
# "Ask for a new AI recommendation" button
string REQUEST_AI_RECOMMENDATION = "REQUEST_AI_RECOMMENDATION"
# "Manual adjust" button — manually adjusts a single process parameter
string MANUAL_ADJUST             = "MANUAL_ADJUST"
# "Fine-tune" button — submits current session data for model fine-tuning
string FINE_TUNE_MODEL           = "FINE_TUNE_MODEL"
# "New DOE" button — triggers launch of the external DOE configuration GUI
string LAUNCH_NEW_DOE            = "LAUNCH_NEW_DOE"
# "Pause" button — gracefully cancels active seam and moves robot to home position
string PAUSE_PROCESS             = "PAUSE_PROCESS"
# "Resume" button — moves robot from home back to last active work zone
string RESUME_PROCESS            = "RESUME_PROCESS"
# "Stop" button — cancels all active goals and moves robot to home (orderly shutdown)
string STOP_PROCESS              = "STOP_PROCESS"

# ── Source constants ───────────────────────────────────────────────────────
string SOURCE_ROBOT   = "ROBOT"
string SOURCE_REMOTE  = "REMOTE"
string SOURCE_UNKNOWN = "UNKNOWN"

# ── Modality constants ─────────────────────────────────────────────────────
string MODALITY_TOUCHSCREEN = "TOUCHSCREEN"
string MODALITY_SPEECH      = "SPEECH"
string MODALITY_GESTURE     = "GESTURE"
string MODALITY_INTERNAL    = "INTERNAL"

# ── Fields ─────────────────────────────────────────────────────────────────
# One of the INTENT_* constants above
string intent

# JSON-encoded payload with intent-specific parameters.
# Examples:
#   MOVE_TO_HOME:             '{}'
#   MOVE_TO_ZONE:             '{"zone": "A", "speed": 0.5}'
#   EXECUTE_SEAM:             '{"seam_id": "seam_01", "weld_speed": 5.0}'
#   ESTOP:                    '{"reason": "operator_button"}'
#   START_PROCESS:            '{"seam_id": "seam_01", "weld_speed": 5.0}'
#   REQUEST_AI_RECOMMENDATION:'{"process_id": "ros_bridge", "mode": "geometry_driven"}'
#   MANUAL_ADJUST:            '{"parameter_name": "weld_speed", "new_value": 5.0, "unit": "mm/s"}'
#   FINE_TUNE_MODEL:          '{"process_id": "ros_bridge", "dataset_tag": "operator_submit"}'
#   PAUSE_PROCESS:            '{"reason": "operator_pause"}'
#   RESUME_PROCESS:           '{"reason": "operator_resume"}'
#   STOP_PROCESS:             '{"reason": "operator_stop"}'
string data

# Who/what expressed the intent
string source

# How the intent was expressed
string modality

# Task scheduling hint (0.0 = lowest, 1.0 = highest)
float32 priority

# Perception certainty — always 1.0 for GUI button presses
float32 confidence
