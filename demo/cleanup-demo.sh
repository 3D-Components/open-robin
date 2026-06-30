#!/bin/bash
# ROBIN System - Demo Cleanup Script
# This script removes demo data from the FIWARE system

set -e  # Exit on any error

# Colors for better output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

DEMO_ID="${1:-demo}"

echo -e "${BLUE}🧹 ROBIN Demo Cleanup${NC}"
echo -e "${BLUE}=====================${NC}"
echo ""
echo -e "Cleaning up demo data for ID pattern: ${YELLOW}$DEMO_ID${NC}"
echo ""

# Function to delete entities by pattern
delete_entities_by_pattern() {
    local entity_type="$1"
    local pattern="$2"
    local description="$3"
    
    echo -e "${BLUE}➤ $description${NC}"
    
    # Get entity IDs that match the pattern
    local entities=$(curl -s -H "NGSILD-Tenant: robin" "http://localhost:1026/ngsi-ld/v1/entities?type=$entity_type" | jq -r ".[].id | select(contains(\"$pattern\"))")
    
    if [ -z "$entities" ]; then
        echo -e "${YELLOW}  No entities found matching pattern: $pattern${NC}"
        return
    fi
    
    local count=0
    while IFS= read -r entity_id; do
        if [ -n "$entity_id" ]; then
            echo -e "${YELLOW}  Deleting: $entity_id${NC}"
            if curl -s -X DELETE -H "NGSILD-Tenant: robin" "http://localhost:1026/ngsi-ld/v1/entities/$entity_id" > /dev/null; then
                echo -e "${GREEN}  ✅ Deleted successfully${NC}"
                ((count++))
            else
                echo -e "${RED}  ❌ Failed to delete${NC}"
            fi
        fi
    done <<< "$entities"
    
    echo -e "${GREEN}  Deleted $count entities${NC}"
    echo ""
}

# Confirm cleanup
echo -e "${YELLOW}⚠️  This will permanently delete demo data!${NC}"
echo -e "${YELLOW}Are you sure you want to continue? (y/N): ${NC}"
read -r confirm

if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    echo -e "${BLUE}Cleanup cancelled.${NC}"
    exit 0
fi

echo ""
echo -e "${BLUE}Starting cleanup...${NC}"
echo ""

# Delete entities by type and pattern
delete_entities_by_pattern "urn:robin:AIRecommendation" "$DEMO_ID" "Cleaning AI recommendations"
delete_entities_by_pattern "urn:robin:Measurement" "$DEMO_ID" "Cleaning measurements"
delete_entities_by_pattern "urn:robin:GeometryTarget" "$DEMO_ID" "Cleaning geometry targets"
delete_entities_by_pattern "urn:robin:Process" "$DEMO_ID" "Cleaning processes"

echo -e "${GREEN}🎉 Cleanup completed!${NC}"
echo ""
echo -e "${BLUE}Remaining entities in system:${NC}"
curl -s -H "NGSILD-Tenant: robin" "http://localhost:1026/ngsi-ld/v1/entities" | jq -r '.[].id' | wc -l | xargs echo "Total entities:"

echo ""
echo -e "${GREEN}✨ Demo cleanup finished for pattern: $DEMO_ID${NC}"
