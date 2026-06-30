#!/bin/bash
# ROBIN System - Setup Validation Script
# This script validates that the ROBIN system is properly configured for demos

set -e  # Exit on any error

# Colors for better output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}🔍 ROBIN System Validation${NC}"
echo -e "${BLUE}===========================${NC}"
echo ""

# Track validation status
VALIDATION_PASSED=true
ORION_REACHABLE_VIA=""

# Function to check and report status
check_requirement() {
    local description="$1"
    local command="$2"
    local success_msg="$3"
    local failure_msg="$4"
    
    echo -e "${BLUE}Checking: $description${NC}"
    
    if eval "$command" > /dev/null 2>&1; then
        echo -e "${GREEN}✅ $success_msg${NC}"
        return 0
    else
        echo -e "${RED}❌ $failure_msg${NC}"
        VALIDATION_PASSED=false
        return 1
    fi
}

# Function to check port availability
check_port() {
    local port="$1"
    local service="$2"
    
    if curl -s "http://localhost:$port" > /dev/null 2>&1 || nc -z localhost "$port" 2>/dev/null; then
        echo -e "${GREEN}✅ Port $port is accessible ($service)${NC}"
        return 0
    else
        echo -e "${RED}❌ Port $port is not accessible ($service)${NC}"
        VALIDATION_PASSED=false
        return 1
    fi
}

# Validate Orion route from the same execution context used by demo scripts.
# The robust demos call `python -m robin` inside `robin-alert-processor`.
check_orion_from_alert_processor() {
    if ! docker ps --format "{{.Names}}" | grep -q "^robin-alert-processor$"; then
        return 1
    fi

    local reachable_url
    reachable_url=$(docker exec robin-alert-processor python - <<'PY'
import os
import sys
import urllib.request

candidates = []
orion_env = os.getenv("ORION_URL", "").rstrip("/")
if orion_env:
    candidates.append(orion_env)

candidates.extend([
    "http://orion-ld:1026",
    "http://host.docker.internal:1026",
    "http://127.0.0.1:1026",
])

seen = set()
for base in candidates:
    if base in seen:
        continue
    seen.add(base)
    try:
        urllib.request.urlopen(f"{base}/version", timeout=2)
        print(base)
        sys.exit(0)
    except Exception:
        pass

sys.exit(1)
PY
)

    if [ $? -eq 0 ]; then
        if [ -z "$reachable_url" ]; then
            reachable_url="http://orion-ld:1026"
        fi
        ORION_REACHABLE_VIA="robin-alert-processor -> ${reachable_url}"
        return 0
    fi

    return 1
}

echo -e "${BLUE}1. System Requirements${NC}"
echo -e "${BLUE}======================${NC}"

# Check Docker
check_requirement "Docker installation" \
    "command -v docker" \
    "Docker is installed and available" \
    "Docker is not installed or not in PATH"

# Check Docker Compose
check_requirement "Docker Compose installation" \
    "docker compose version" \
    "Docker Compose is available" \
    "Docker Compose is not available"

# Check jq for JSON processing
check_requirement "jq installation (for JSON processing)" \
    "command -v jq" \
    "jq is installed and available" \
    "jq is not installed (install with: brew install jq or apt-get install jq)"

echo ""
echo -e "${BLUE}2. Docker Services${NC}"
echo -e "${BLUE}==================${NC}"

# Check if containers are running
CONTAINERS=("fiware-orion" "db-mongo" "fiware-timescaledb" "fiware-mintaka" "robin-alert-processor")

for container in "${CONTAINERS[@]}"; do
    if docker ps --format "table {{.Names}}" | grep -q "$container"; then
        echo -e "${GREEN}✅ Container $container is running${NC}"
    else
        echo -e "${RED}❌ Container $container is not running${NC}"
        VALIDATION_PASSED=false
    fi
done

echo ""
echo -e "${BLUE}3. Service Connectivity${NC}"
echo -e "${BLUE}=======================${NC}"

# Check service ports
if check_orion_from_alert_processor; then
    echo -e "${GREEN}✅ Orion Context Broker is reachable (${ORION_REACHABLE_VIA})${NC}"
else
    echo -e "${RED}❌ Orion Context Broker is not reachable from robin-alert-processor${NC}"
    VALIDATION_PASSED=false
fi
check_port "9090" "Mintaka"
check_port "8001" "ROBIN Alert Processor"
check_port "5174" "Dashboard"

echo ""
echo -e "${BLUE}4. FIWARE Component Health${NC}"
echo -e "${BLUE}===========================${NC}"

# Test Orion Context Broker
if check_orion_from_alert_processor; then
    echo -e "${GREEN}✅ Orion Context Broker is responding via ${ORION_REACHABLE_VIA}${NC}"
else
    echo -e "${RED}❌ Orion Context Broker is not responding${NC}"
    VALIDATION_PASSED=false
fi

# Test Mintaka
if curl -s "http://localhost:9090/health" > /dev/null; then
    echo -e "${GREEN}✅ Mintaka is responding${NC}"
else
    echo -e "${RED}❌ Mintaka is not responding${NC}"
    VALIDATION_PASSED=false
fi

# Test ROBIN Alert Processor
if curl -s "http://localhost:8001/health" > /dev/null; then
    echo -e "${GREEN}✅ ROBIN Alert Processor is responding${NC}"
else
    echo -e "${RED}❌ ROBIN Alert Processor is not responding${NC}"
    VALIDATION_PASSED=false
fi

# Test Dashboard
if curl -s "http://localhost:5174" > /dev/null; then
    echo -e "${GREEN}✅ Dashboard is responding${NC}"
else
    echo -e "${RED}❌ Dashboard is not responding${NC}"
    VALIDATION_PASSED=false
fi

echo ""
echo -e "${BLUE}5. ROBIN CLI Functionality${NC}"
echo -e "${BLUE}===========================${NC}"

# Test ROBIN CLI
if docker exec robin-alert-processor python -m robin status > /dev/null 2>&1; then
    echo -e "${GREEN}✅ ROBIN CLI is functional${NC}"
else
    echo -e "${RED}❌ ROBIN CLI is not working${NC}"
    VALIDATION_PASSED=false
fi

echo ""
echo -e "${BLUE}6. Canonical Demo Assets${NC}"
echo -e "${BLUE}========================${NC}"

CANONICAL_DEMOS=("demo/profiles/welding_profile.py" "demo/profiles/spray_coating_profile.py")
SUPPORTING_SCRIPTS=("demo/cleanup-demo.sh" "demo/validate-setup.sh")

for script in "${CANONICAL_DEMOS[@]}"; do
    if [ -f "$script" ]; then
        echo -e "${GREEN}✅ $script exists${NC}"
    else
        echo -e "${RED}❌ Missing canonical demo: $script${NC}"
        VALIDATION_PASSED=false
    fi
done

for script in "${SUPPORTING_SCRIPTS[@]}"; do
    if [ -x "$script" ]; then
        echo -e "${GREEN}✅ $script is executable${NC}"
    else
        echo -e "${RED}❌ $script is not executable${NC}"
        VALIDATION_PASSED=false
    fi
done

echo ""
echo -e "${BLUE}7. Orion→Timescale via TROE${NC}"
echo -e "${BLUE}============================${NC}"
echo -e "${GREEN}✅ No NGSI subscription needed. Orion writes to Timescale automatically when observedAt is present.${NC}"

echo ""
echo -e "${BLUE}Validation Summary${NC}"
echo -e "${BLUE}==================${NC}"

if [ "$VALIDATION_PASSED" = true ]; then
    echo -e "${GREEN}🎉 All validations passed! The ROBIN system is ready for demos.${NC}"
    echo ""
    echo -e "${BLUE}Ready to run demos:${NC}"
    echo -e "• Welding robust demo: ${YELLOW}python demo/profiles/welding_profile.py --mode both --duration 120 --interval 2${NC}"
    echo -e "• Spray robust demo: ${YELLOW}python demo/profiles/spray_coating_profile.py --mode both --duration 120 --interval 2${NC}"
    echo -e "• Full guide: ${YELLOW}demo/README.md${NC}"
    echo ""
    echo -e "${GREEN}✨ Happy demoing!${NC}"
    exit 0
else
    echo -e "${RED}❌ Some validations failed. Please fix the issues above before running demos.${NC}"
    echo ""
    echo -e "${YELLOW}Common fixes:${NC}"
    echo -e "• Start services: ${YELLOW}docker compose up -d${NC}"
    echo -e "• Install jq: ${YELLOW}brew install jq${NC} (macOS) or ${YELLOW}apt-get install jq${NC} (Linux)"
    echo -e "• Make scripts executable: ${YELLOW}chmod +x demo/*.sh${NC}"
    echo -e "• Check logs: ${YELLOW}docker compose logs${NC}"
    echo ""
    exit 1
fi
