#!/bin/bash
# ROBIN System - Simulation Demo Script
# This script provides a realistic profile simulation (welding reference) with live data updates

set -e  # Exit on any error

# Colors for better output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
DEMO_ID="simulation-$(date +%s)"
CONTAINER_NAME="robin-alert-processor"
SIMULATION_DURATION=120  # 2 minutes default
MEASUREMENT_INTERVAL=2   # seconds between measurements
PROCESS_MODE="geometry_driven"

# Simulation parameters
BASE_HEIGHT=5.2
BASE_WIDTH=3.8
BASE_SPEED=10.5
BASE_CURRENT=150.0
BASE_VOLTAGE=24.0

# Oscillation parameters
HEIGHT_OSCILLATION=0.3
WIDTH_OSCILLATION=0.2
SPEED_OSCILLATION=1.0
CURRENT_OSCILLATION=5.0
VOLTAGE_OSCILLATION=0.5

# Alert scenarios (time points where we introduce significant deviations)
ALERT_TIMES=(30 75 105)  # seconds into simulation

# Functions
show_header() {
    clear
    echo -e "${BLUE}╔══════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║     ROBIN Simulation Demo            ║${NC}"
    echo -e "${BLUE}║  Live Profile Process Simulation     ║${NC}"
    echo -e "${BLUE}╚══════════════════════════════════════╝${NC}"
    echo ""
    echo -e "Demo Session ID: ${YELLOW}$DEMO_ID${NC}"
    echo -e "Duration: ${YELLOW}$SIMULATION_DURATION seconds${NC}"
    echo -e "Measurement Interval: ${YELLOW}$MEASUREMENT_INTERVAL seconds${NC}"
    echo ""
}

explain() {
    echo -e "${PURPLE}💡 $1${NC}"
    echo ""
}

pause_with_explanation() {
    echo -e "${CYAN}$1${NC}"
    echo -e "${YELLOW}Press Enter to continue...${NC}"
    read -r
    echo ""
}

check_prerequisites() {
    echo -e "${BLUE}🔍 Checking Prerequisites${NC}"
    echo -e "${BLUE}=========================${NC}"
    
    # Check Docker
    if ! command -v docker &> /dev/null; then
        echo -e "${RED}❌ Docker is not installed or not in PATH${NC}"
        exit 1
    fi
    echo -e "${GREEN}✅ Docker is available${NC}"
    
    # Check if container is running
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo -e "${YELLOW}⚠️  ROBIN container is not running${NC}"
        echo -e "${BLUE}Starting ROBIN system...${NC}"
        docker compose up -d
        echo -e "${GREEN}✅ ROBIN system started${NC}"
    else
        echo -e "${GREEN}✅ ROBIN container is running${NC}"
    fi
    
    echo ""
}

run_robin_cmd() {
    local cmd="$1"
    local description="$2"
    local silent="${3:-false}"
    
    if [ "$silent" != "true" ]; then
        echo -e "${BLUE}➤ $description${NC}"
    fi
    
    # Capture both stdout and stderr
    local output
    output=$(docker exec "$CONTAINER_NAME" python -m robin $cmd 2>&1)
    local exit_code=$?
    
    if [ $exit_code -eq 0 ]; then
        if [ "$silent" != "true" ]; then
            echo -e "${GREEN}✅ Success!${NC}"
            echo ""
        fi
        return 0
    else
        echo -e "${RED}❌ Failed: $description${NC}"
        echo -e "${RED}Error output: $output${NC}"
        echo -e "${YELLOW}Exit code: $exit_code${NC}"
        echo ""
        return 1  # Return error but don't exit the script
    fi
}

# Generate realistic measurement with oscillations and noise
generate_measurement() {
    local time_elapsed=$1
    local alert_scenario=$2
    
    # Base oscillation using sine waves with different frequencies
    local time_factor=$(echo "scale=4; $time_elapsed / 10" | bc -l)
    local height_osc=$(echo "scale=4; s($time_factor) * $HEIGHT_OSCILLATION" | bc -l)
    local width_osc=$(echo "scale=4; s($time_factor * 1.3) * $WIDTH_OSCILLATION" | bc -l)
    local speed_osc=$(echo "scale=4; s($time_factor * 0.7) * $SPEED_OSCILLATION" | bc -l)
    local current_osc=$(echo "scale=4; s($time_factor * 0.9) * $CURRENT_OSCILLATION" | bc -l)
    local voltage_osc=$(echo "scale=4; s($time_factor * 1.1) * $VOLTAGE_OSCILLATION" | bc -l)
    
    # Add random noise (small variations)
    local height_noise=$(echo "scale=4; (($RANDOM % 100) - 50) / 1000" | bc -l)
    local width_noise=$(echo "scale=4; (($RANDOM % 100) - 50) / 1000" | bc -l)
    local speed_noise=$(echo "scale=4; (($RANDOM % 100) - 50) / 100" | bc -l)
    local current_noise=$(echo "scale=4; (($RANDOM % 100) - 50) / 10" | bc -l)
    local voltage_noise=$(echo "scale=4; (($RANDOM % 100) - 50) / 100" | bc -l)
    
    # Calculate base values with oscillations and noise
    local height=$(echo "scale=2; $BASE_HEIGHT + $height_osc + $height_noise" | bc -l)
    local width=$(echo "scale=2; $BASE_WIDTH + $width_osc + $width_noise" | bc -l)
    local speed=$(echo "scale=2; $BASE_SPEED + $speed_osc + $speed_noise" | bc -l)
    local current=$(echo "scale=2; $BASE_CURRENT + $current_osc + $current_noise" | bc -l)
    local voltage=$(echo "scale=2; $BASE_VOLTAGE + $voltage_osc + $voltage_noise" | bc -l)
    
    # Apply alert scenarios (introduce significant deviations)
    if [ "$alert_scenario" = "true" ]; then
        case $((time_elapsed % 3)) in
            0) # Height deviation
                height=$(echo "scale=2; $height * 0.6" | bc -l)  # 40% reduction
                ;;
            1) # Width deviation  
                width=$(echo "scale=2; $width * 1.8" | bc -l)   # 80% increase
                ;;
            2) # Multiple parameter deviation
                height=$(echo "scale=2; $height * 1.5" | bc -l)
                current=$(echo "scale=2; $current * 0.7" | bc -l)
                ;;
        esac
    fi
    
    # Ensure positive values
    height=$(echo "if ($height < 0.5) 0.5 else $height" | bc -l)
    width=$(echo "if ($width < 0.5) 0.5 else $width" | bc -l)
    speed=$(echo "if ($speed < 5.0) 5.0 else $speed" | bc -l)
    current=$(echo "if ($current < 100.0) 100.0 else $current" | bc -l)
    voltage=$(echo "if ($voltage < 20.0) 20.0 else $voltage" | bc -l)
    
    echo "$height $width $speed $current $voltage"
}

# Check if we should trigger an alert at this time
should_trigger_alert() {
    local time_elapsed=$1
    for alert_time in "${ALERT_TIMES[@]}"; do
        if [ $time_elapsed -ge $alert_time ] && [ $time_elapsed -lt $((alert_time + 10)) ]; then
            return 0  # true
        fi
    done
    return 1  # false
}

simulate_welding_process() {
    local process_id=$1
    local duration=$2
    
    echo -e "${BLUE}🔥 Starting Profile Process Simulation${NC}"
    echo -e "${BLUE}======================================${NC}"
    echo -e "Process ID: ${YELLOW}$process_id${NC}"
    echo -e "Duration: ${YELLOW}$duration seconds${NC}"
    echo -e "Measurement interval: ${YELLOW}$MEASUREMENT_INTERVAL seconds${NC}"
    echo ""
    
    echo -e "${CYAN}📊 Geometry-Driven Simulation Features:${NC}"
    echo -e "• Target: ${BASE_HEIGHT}mm height × ${BASE_WIDTH}mm width"
    echo -e "• Realistic oscillations in all measurement parameters"
    echo -e "• Random noise to simulate real-world sensor conditions"
    echo -e "• Automatic deviation scenarios at strategic times"
    echo -e "• Live updates visible in ROBIN dashboard"
    echo ""
    
    echo -e "${YELLOW}🎯 Watch your ROBIN dashboard for live updates!${NC}"
    echo -e "${CYAN}   Open: http://localhost:5174${NC}"
    echo ""
    
    local measurement_count=0
    local start_time=$(date +%s)
    
    for ((elapsed=0; elapsed<duration; elapsed+=MEASUREMENT_INTERVAL)); do
        measurement_count=$((measurement_count + 1))
        
        # Check if this is an alert scenario
        local alert_scenario="false"
        if should_trigger_alert $elapsed; then
            alert_scenario="true"
        fi
        
        # Generate measurement values
        local values=$(generate_measurement $elapsed $alert_scenario)
        read -r height width speed current voltage <<< "$values"
        
        # Add measurement with timestamp but no hyphens (NGSI-LD compliant)
        local measure_id="measure$(date +%s)x$measurement_count"
        
        # Debug output - show the exact command being executed
        echo ""
        echo -e "${YELLOW}🔍 DEBUG: Executing measurement command:${NC}"
        echo -e "${CYAN}docker exec $CONTAINER_NAME python -m robin add-measurement $process_id $measure_id $height $width --speed $speed --current $current --voltage $voltage${NC}"
        echo -e "${YELLOW}Process ID: $process_id${NC}"
        echo -e "${YELLOW}Measure ID: $measure_id${NC}"
        echo -e "${YELLOW}Values: H=$height W=$width S=$speed C=$current V=$voltage${NC}"
        echo ""
        
        # Try to add measurement, but continue simulation even if it fails
        if run_robin_cmd "add-measurement $process_id $measure_id $height $width --speed $speed --current $current --voltage $voltage" \
                        "Adding measurement $measurement_count" false; then
            echo -e "${GREEN}✅ Measurement $measurement_count added successfully${NC}"
        else
            echo -e "${YELLOW}⚠️  Measurement $measurement_count failed, but continuing simulation...${NC}"
        fi
        
        # Display progress
        local progress=$((elapsed * 100 / duration))
        local remaining=$((duration - elapsed))
        
        printf "\r${BLUE}Progress: [%-50s] %d%% | Measurement %d | Remaining: %ds | H:%.2f W:%.2f${NC}" \
               "$(printf '%*s' $((progress/2)) '' | tr ' ' '=')" \
               "$progress" "$measurement_count" "$remaining" "$height" "$width"
        
        # Special notification for alert scenarios
        if [ "$alert_scenario" = "true" ]; then
            echo ""
            echo -e "${RED}🚨 ALERT SCENARIO TRIGGERED! Check the dashboard for deviation alerts!${NC}"
            echo -e "${YELLOW}   Values: H=${height}mm W=${width}mm (significant deviation introduced)${NC}"
            echo ""
        fi
        
        # Sleep for the measurement interval
        sleep $MEASUREMENT_INTERVAL
    done
    
    echo ""
    echo -e "${GREEN}✅ Profile process simulation completed!${NC}"
    echo -e "Total measurements: ${YELLOW}$measurement_count${NC}"
    echo ""
}

# Main simulation flow
main_simulation() {
    show_header
    
    explain "This simulation demo creates a realistic geometry-driven process using the welding profile with:"
    explain "• Target geometry: ${BASE_HEIGHT}mm height × ${BASE_WIDTH}mm width"
    explain "• Continuous measurements with natural oscillations"
    explain "• Random noise to simulate real-world sensor data"
    explain "• Automatic alert scenarios that trigger deviations from target"
    explain "• Live updates visible in ROBIN dashboard"
    
    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -d|--duration)
                SIMULATION_DURATION="$2"
                shift 2
                ;;
            -i|--interval)
                MEASUREMENT_INTERVAL="$2"
                shift 2
                ;;
            -m|--mode)
                PROCESS_MODE="$2"
                shift 2
                ;;
            -h|--help)
                echo "Usage: $0 [OPTIONS]"
                echo "Options:"
                echo "  -d, --duration SECONDS    Simulation duration (default: 120)"
                echo "  -i, --interval SECONDS    Measurement interval (default: 2)"
                echo "  -m, --mode MODE           Process mode (default: geometry_driven for now)"
                echo "  -h, --help                Show this help"
                exit 0
                ;;
            *)
                echo "Unknown option: $1"
                exit 1
                ;;
        esac
    done
    
    pause_with_explanation "Ready to start the simulation? Make sure the dashboard is open at http://localhost:5174"
    
    # Prerequisites
    check_prerequisites
    
    echo -e "${BLUE}Step 1: Dashboard Setup${NC}"
    echo -e "${BLUE}======================${NC}"
    echo -e "${CYAN}Open the ROBIN dashboard:${NC}"
    echo -e "1. Open Dashboard: ${YELLOW}http://localhost:5174${NC}"
    echo -e "2. Select or create a process from the top bar"
    echo -e "3. The Live Ops tab will show telemetry, deviation monitoring, and process controls"
    echo ""
    
    pause_with_explanation "Dashboard open? Now we need the process ID..."
    
    echo -e "${BLUE}Step 2: Get Process ID${NC}"
    echo -e "${BLUE}======================${NC}"
    echo -e "${CYAN}Check the process selector in the dashboard top bar.${NC}"
    echo -e "${CYAN}Use the default 'ros_bridge' or enter a custom process ID.${NC}"
    echo ""
    echo -e "${YELLOW}Please enter the process ID:${NC}"
    read -r DEMO_ID
    
    if [ -z "$DEMO_ID" ]; then
        echo -e "${RED}❌ No process ID provided. Exiting.${NC}"
        exit 1
    fi
    
    echo ""
    echo -e "${GREEN}✅ Using process ID: ${YELLOW}$DEMO_ID${NC}"
    echo ""
    
    # Convert the existing process to geometry-driven by adding a target
    echo -e "${BLUE}Step 3: Converting to Geometry-Driven Process${NC}"
    echo -e "${BLUE}============================================${NC}"
    
    # Try to add geometry target, but continue even if it fails
    if run_robin_cmd "create-target $DEMO_ID $BASE_HEIGHT $BASE_WIDTH" "Setting geometry target (${BASE_HEIGHT}x${BASE_WIDTH}mm)"; then
        echo -e "${GREEN}✅ Geometry target added successfully - process is now geometry-driven${NC}"
    else
        echo -e "${YELLOW}⚠️  Geometry target creation failed - process may already be parameter-driven${NC}"
        echo -e "${CYAN}Continuing with simulation anyway...${NC}"
    fi
    echo ""
    
    # Start simulation
    echo -e "${BLUE}Step 4: Live Simulation${NC}"
    echo -e "${BLUE}======================${NC}"
    
    simulate_welding_process "$DEMO_ID" "$SIMULATION_DURATION"
    
    # Final summary
    echo -e "${BLUE}🎉 Simulation Complete!${NC}"
    echo -e "${BLUE}=======================${NC}"
    echo ""
    echo -e "${GREEN}What you should have seen:${NC}"
    echo -e "• Continuous measurement updates in the ROBIN dashboard"
    echo -e "• Natural oscillations around target geometry (${BASE_HEIGHT}×${BASE_WIDTH}mm)"
    echo -e "• Time-series charts showing realistic process parameter trends"
    echo -e "• Deviation alerts when measurements drift from target geometry"
    echo -e "• Alert scenarios triggered at 30s, 75s, and 105s"
    echo ""
    echo -e "${BLUE}Your simulation session: ${YELLOW}$DEMO_ID${NC}"
    echo ""
    echo -e "${BLUE}Next steps:${NC}"
    echo -e "• Explore historical data in the dashboard telemetry charts"
    echo -e "• Try different metric views and process controls"
    echo -e "• Run again with different parameters: ${YELLOW}$0 --duration 300 --interval 1${NC}"
    echo -e "• Clean up demo data: ${YELLOW}./demo/cleanup-demo.sh $DEMO_ID${NC}"
    echo ""
    echo -e "${GREEN}Thank you for exploring the ROBIN geometry-driven simulation demo!${NC}"
}

# Handle Ctrl+C gracefully
trap 'echo -e "\n${YELLOW}Simulation interrupted. Process $DEMO_ID may still be running.${NC}"; exit 0' INT

# Run the simulation
main_simulation "$@"
