#!/bin/bash
# ROBIN System - Interactive Demo Script
# This script provides a comprehensive, step-by-step demonstration with explanations

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
DEMO_ID="interactive-$(date +%s)"
CONTAINER_NAME="robin-alert-processor"

# Functions
show_header() {
    clear
    echo -e "${BLUE}╔══════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║        ROBIN System Demo             ║${NC}"
    echo -e "${BLUE}║    Interactive Demonstration         ║${NC}"
    echo -e "${BLUE}╚══════════════════════════════════════╝${NC}"
    echo ""
    echo -e "Demo Session ID: ${YELLOW}$DEMO_ID${NC}"
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
    
    # Check Docker Compose
    if ! docker compose version &> /dev/null; then
        echo -e "${RED}❌ Docker Compose is not available${NC}"
        exit 1
    fi
    echo -e "${GREEN}✅ Docker Compose is available${NC}"
    
    # Check if container is running
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo -e "${YELLOW}⚠️  ROBIN container is not running${NC}"
        echo -e "${BLUE}Starting ROBIN system...${NC}"
        docker compose up -d
        echo -e "${GREEN}✅ ROBIN system started${NC}"
    else
        echo -e "${GREEN}✅ ROBIN container is running${NC}"
    fi
    
    # Check if container has the required endpoints (auto-rebuild if needed)
    echo -e "${BLUE}🔍 Verifying container has latest code...${NC}"
    if ! curl -s http://localhost:8001/openapi.json | grep -q "create-process"; then
        echo -e "${YELLOW}⚠️  Container appears to have old code${NC}"
        echo -e "${BLUE}🔄 Rebuilding container with latest code...${NC}"
        docker compose build alert-processor
        docker compose up -d alert-processor
        sleep 3  # Give container time to start
        echo -e "${GREEN}✅ Container updated with latest code${NC}"
    else
        echo -e "${GREEN}✅ Container has latest code${NC}"
    fi
    
    echo ""
}

run_robin_cmd() {
    local cmd="$1"
    local description="$2"
    local show_command="${3:-true}"
    
    echo -e "${BLUE}➤ $description${NC}"
    if [ "$show_command" = "true" ]; then
        echo -e "${YELLOW}$ docker exec $CONTAINER_NAME python -m robin $cmd${NC}"
    fi
    
    if docker exec "$CONTAINER_NAME" python -m robin $cmd; then
        echo -e "${GREEN}✅ Success!${NC}"
        echo ""
    else
        echo -e "${RED}❌ Failed: $description${NC}"
        exit 1
    fi
}

run_robin_cmd_json() {
    local process_id="$1"
    local json_params="$2"
    local description="$3"
    local show_command="${4:-true}"
    
    echo -e "${BLUE}➤ $description${NC}"
    if [ "$show_command" = "true" ]; then
        echo -e "${YELLOW}$ docker exec $CONTAINER_NAME python -m robin add-recommendation $process_id '$json_params'${NC}"
    fi
    
    if docker exec "$CONTAINER_NAME" python -m robin add-recommendation "$process_id" "$json_params"; then
        echo -e "${GREEN}✅ Success!${NC}"
        echo ""
    else
        echo -e "${RED}❌ Failed: $description${NC}"
        exit 1
    fi
}

run_curl_cmd() {
    local cmd="$1"
    local description="$2"
    
    echo -e "${BLUE}➤ $description${NC}"
    echo -e "${YELLOW}$ $cmd${NC}"
    
    if eval "$cmd"; then
        echo -e "${GREEN}✅ Query completed${NC}"
        echo ""
    else
        echo -e "${RED}❌ Failed: $description${NC}"
    fi
}

# Main Demo Flow
main_demo() {
    show_header
    
    explain "This interactive demo will walk you through the ROBIN system capabilities."
    explain "ROBIN demonstrates dual-mode process optimization using the welding profile:"
    explain "• Parameter-Driven: AI optimizes process parameters based on measurements"
    explain "• Geometry-Driven: Process targets specific geometric dimensions"
    echo ""
    echo -e "${CYAN}📊 You can see the complete demo flow diagram in: ./demo/README.md${NC}"
    echo -e "${CYAN}   Or view the Mermaid source: ./demo/assets/demo-flow.mmd${NC}"
    echo ""
    
    pause_with_explanation "Let's start by checking that everything is ready..."
    
    # Prerequisites
    check_prerequisites
    
    pause_with_explanation "Great! Now let's check the system status to ensure all FIWARE components are connected."
    
    # Step 1: System Status
    show_header
    echo -e "${BLUE}Step 1: System Health Check${NC}"
    echo -e "${BLUE}============================${NC}"
    explain "The status command checks connectivity to Orion Context Broker and Mintaka."
    run_robin_cmd "status" "Checking FIWARE component connectivity"
    
    pause_with_explanation "Perfect! All FIWARE components are connected. Next, we'll create two process instances for the welding profile."
    
    # Step 2: Create Processes
    show_header
    echo -e "${BLUE}Step 2: Creating Welding Profile Processes${NC}"
    echo -e "${BLUE}===================================${NC}"
    explain "We'll create two processes to demonstrate both operation modes:"
    explain "1. Parameter-Driven: Focuses on optimizing process parameters"
    explain "2. Geometry-Driven: Focuses on achieving target dimensions"
    
    run_robin_cmd "create-process $DEMO_ID-param" "Creating parameter-driven process"
    run_robin_cmd "create-process $DEMO_ID-geo --mode geometry_driven" "Creating geometry-driven process"
    
    pause_with_explanation "Excellent! Both processes are created. For geometry-driven processes, we need to set target dimensions."
    
    # Step 3: Geometry Targets
    show_header
    echo -e "${BLUE}Step 3: Setting Geometry Targets${NC}"
    echo -e "${BLUE}================================${NC}"
    explain "Geometry-driven processes need target dimensions to work towards."
    explain "We'll set a target of 5.2mm height and 3.8mm width for the welding profile geometry."
    
    run_robin_cmd "create-target $DEMO_ID-geo 5.2 3.8" "Setting geometry target (5.2x3.8mm)"
    
    pause_with_explanation "Target set! Now let's simulate some real-world measurement data from profilometer sensors."
    
    # Step 4: Add Measurements
    show_header
    echo -e "${BLUE}Step 4: Adding Measurement Data${NC}"
    echo -e "${BLUE}===============================${NC}"
    explain "In this welding profile, profilometer sensors measure the resulting geometry,"
    explain "while process monitoring captures machine parameters like speed, current, and voltage."
    explain "We'll simulate this by adding comprehensive measurement data to both processes."
    
    run_robin_cmd "add-measurement $DEMO_ID-param measure-$DEMO_ID-1 5.1 3.7 --speed 10.5 --current 148.2 --voltage 23.8" "Adding measurement to parameter process (5.1x3.7mm, 10.5mm/s, 148.2A, 23.8V)"
    sleep 1
    run_robin_cmd "add-measurement $DEMO_ID-param measure-$DEMO_ID-2 5.3 3.9 --speed 11.2 --current 152.1 --voltage 24.3" "Adding second measurement to parameter process (5.3x3.9mm, 11.2mm/s, 152.1A, 24.3V)"
    sleep 1
    run_robin_cmd "add-measurement $DEMO_ID-geo measure-$DEMO_ID-3 5.0 3.6 --speed 9.8 --current 145.7 --voltage 23.5" "Adding measurement to geometry process (5.0x3.6mm, 9.8mm/s, 145.7A, 23.5V - below target)"
    
    pause_with_explanation "Great! We have measurement data. Now let's see how AI provides optimization recommendations."
    
    # Step 5: AI Recommendations
    show_header
    echo -e "${BLUE}Step 5: AI-Powered Recommendations${NC}"
    echo -e "${BLUE}===================================${NC}"
    explain "Based on the measurements, AI models provide parameter recommendations to improve quality."
    explain "These recommendations include process parameters like temperature, wire speed, voltage, etc."
    
    run_robin_cmd_json "$DEMO_ID-param" '{"temperature": 1850, "wireSpeed": 12.5, "voltage": 28.2, "confidence": 0.92, "reason": "optimize_for_consistency"}' "Adding AI recommendation for parameter process"
    run_robin_cmd_json "$DEMO_ID-geo" '{"temperature": 1820, "wireSpeed": 11.8, "voltage": 27.5, "confidence": 0.88, "reason": "increase_penetration"}' "Adding AI recommendation for geometry process"
    
    pause_with_explanation "Excellent! All data is now stored in the FIWARE ecosystem. Let's explore how to view and query this data."
    
    # Step 6: Live Dashboard Demo
    show_header
    echo -e "${BLUE}Step 6: Live ROBIN Dashboard Demo 🔥${NC}"
    echo -e "${BLUE}=====================================${NC}"
    explain "Now for the BEST part - let's use the ROBIN operator dashboard!"
    explain "We'll add measurements via CLI and watch them appear LIVE in the dashboard."
    echo ""
    
    echo -e "${YELLOW}📊 ROBIN DASHBOARD:${NC}"
    echo -e "${CYAN}1. Open Dashboard: http://localhost:5174${NC}"
    echo -e "${CYAN}2. Select process '$DEMO_ID-param' from the top bar process selector${NC}"
    echo -e "${CYAN}3. The Live Ops tab shows KPIs, telemetry charts, deviation monitoring, and process controls${NC}"
    echo -e "${CYAN}4. Run the CLI commands below to add measurements${NC}"
    echo -e "${CYAN}5. Watch the dashboard update in REAL-TIME! 🚀${NC}"
    echo ""
    
    echo -e "${PURPLE}💡 Pro tip: The dashboard polls every 1-2 seconds and charts are sourced from Mintaka temporal storage.${NC}"
    echo ""
    echo -e "${YELLOW}Commands to run with the dashboard open:${NC}"
    echo ""
    echo -e "${GREEN}# Add first measurement with process parameters${NC}"
    echo -e "${CYAN}docker exec $CONTAINER_NAME python -m robin add-measurement $DEMO_ID-param \"measure-$(date +%s)\" 5.1 3.7 --speed 10.5 --current 148.2 --voltage 23.8${NC}"
    echo ""
    echo -e "${GREEN}# Add second measurement (wait 1-2 seconds to see the update)${NC}"
    echo -e "${CYAN}docker exec $CONTAINER_NAME python -m robin add-measurement $DEMO_ID-param \"measure-$(date +%s)\" 5.3 3.9 --speed 11.2 --current 152.1 --voltage 24.3${NC}"
    echo ""
    echo -e "${GREEN}# Add third measurement${NC}"
    echo -e "${CYAN}docker exec $CONTAINER_NAME python -m robin add-measurement $DEMO_ID-param \"measure-$(date +%s)\" 4.9 3.5 --speed 9.8 --current 145.7 --voltage 23.5${NC}"
    echo ""
    echo -e "${GREEN}# Add deviation measurement to trigger alert${NC}"
    echo -e "${CYAN}docker exec $CONTAINER_NAME python -m robin add-measurement $DEMO_ID-param \"measure-$(date +%s)\" 2.0 1.5 --speed 8.5 --current 135.0 --voltage 22.1${NC}"
    echo ""
    
    echo -e "${YELLOW}⚡ What you should see in the ROBIN dashboard:${NC}"
    echo -e "• Measurement KPI cards show latest height, width, speed, current, voltage"
    echo -e "• Telemetry chart plots the selected metric in real-time"
    echo -e "• Deviation Monitor detects deviations and shows per-axis breakdown"
    echo -e "• Process Controls let you switch modes and set tolerance"
    echo -e "• Alerts panel shows deviation and operational events"
    echo ""
    
    echo -e "${CYAN}💡 Try these dashboard features:${NC}"
    echo -e "• Switch between metric views (speed, current, voltage, profile height/width)"
    echo -e "• Adjust tolerance in Process Controls and watch deviation alerts change"
    echo -e "• Go to Models & Trust tab to load AI checkpoints and run predictions"
    echo -e "• Use the 3D visualization with layer controls"
    echo ""
    
    pause_with_explanation "Explore the dashboard with live data! Press Enter when ready to continue with data access methods..."
    
    # Step 7: Data Access Methods
    show_header
    echo -e "${BLUE}Step 7: Data Access Methods${NC}"
    echo -e "${BLUE}===========================${NC}"
    explain "ROBIN provides multiple ways to access your data:"
    explain "• ROBIN Dashboard: Professional operator interface with live telemetry and AI model control"
    explain "• REST API: Programmatic access with Swagger documentation"
    explain "• Direct FIWARE Queries: Full NGSI-LD access to Orion Context Broker"
    
    echo -e "${GREEN}📊 Access Points:${NC}"
    echo -e "• Dashboard:    ${CYAN}http://localhost:5174${NC}"
    echo -e "• API Docs:     ${CYAN}http://localhost:8001/docs${NC}"
    echo -e "• Health:       ${CYAN}http://localhost:8001/health${NC}"
    echo -e "• Legacy Dash:  ${CYAN}http://localhost:8001/dashboard${NC}"
    echo ""
    
    pause_with_explanation "Let's also query the data directly from the FIWARE Context Broker to see the raw NGSI-LD entities."
    
    # Step 8: FIWARE Queries
    show_header
    echo -e "${BLUE}Step 8: Querying FIWARE Data${NC}"
    echo -e "${BLUE}============================${NC}"
    explain "FIWARE stores all data as NGSI-LD entities. Let's query them directly:"
    
    echo -e "${BLUE}Processes:${NC}"
    run_curl_cmd 'curl -s -H "NGSILD-Tenant: robin" "http://localhost:1026/ngsi-ld/v1/entities?type=urn:robin:Process" | jq ".[].id"' "Listing all processes"
    
    echo -e "${BLUE}Measurements:${NC}"
    run_curl_cmd 'curl -s -H "NGSILD-Tenant: robin" "http://localhost:1026/ngsi-ld/v1/entities?type=urn:robin:Measurement" | jq ".[].id"' "Listing all measurements"
    
    echo -e "${BLUE}AI Recommendations:${NC}"
    run_curl_cmd 'curl -s -H "NGSILD-Tenant: robin" "http://localhost:1026/ngsi-ld/v1/entities?type=urn:robin:AIRecommendation" | jq ".[].id"' "Listing all AI recommendations"
    
    pause_with_explanation "Perfect! Let's see the detailed data for our specific demo processes."
    
    echo -e "${BLUE}Detailed Process Data:${NC}"
    run_curl_cmd "curl -s -H \"NGSILD-Tenant: robin\" \"http://localhost:1026/ngsi-ld/v1/entities?type=urn:robin:Process\" | jq '.[] | select(.id | contains(\"$DEMO_ID\"))'" "Showing our demo processes"
    
    # Final Summary
    show_header
    echo -e "${GREEN}🎉 Demo Completed Successfully!${NC}"
    echo -e "${GREEN}===============================${NC}"
    echo ""
    echo -e "${BLUE}What we demonstrated:${NC}"
    echo -e "✅ Dual-mode process creation in the welding profile (parameter & geometry driven)"
    echo -e "✅ Real-time measurement data ingestion"
    echo -e "✅ AI-powered parameter recommendations"
    echo -e "✅ FIWARE NGSI-LD data storage and querying"
    echo -e "✅ Multi-tenant data isolation"
    echo -e "✅ RESTful API access with web dashboard"
    echo ""
    echo -e "${BLUE}Your demo session ID: ${YELLOW}$DEMO_ID${NC}"
    echo ""
    echo -e "${BLUE}Next steps:${NC}"
    echo -e "• Explore the ROBIN dashboard: ${CYAN}http://localhost:5174${NC}"
    echo -e "• Try the API: ${CYAN}http://localhost:8001/docs${NC}"
    echo -e "• Legacy dashboard: ${CYAN}http://localhost:8001/dashboard${NC}"
    echo -e "• Clean up demo data: ${YELLOW}./demo/cleanup-demo.sh $DEMO_ID${NC}"
    echo ""
    echo -e "${GREEN}Thank you for exploring the ROBIN system!${NC}"
}

# Run the demo
main_demo
